"""Closed-loop test of the CAR firmware straight over USB (no Orin needed).

usage: py -3 usb_drive.py "<pct>,<pct>,<seconds>;<pct>,<pct>,<seconds>;..."   e.g. "25,25,3;0,0,2;35,35,3;0,0,2"
Sends <CMD,seq,fwa 0,bwl,bwr,cs> at 10 Hz (the firmware has a 500 ms watchdog), logs the USB debug lines
(dbg: duties + rpm, STA: signed speeds vL/vR + fault flags f). Aborts to zero on: f != 0, |rpm| > 320,
no STA for > 1 s, an unexpectedly stuck wheel (command >= 15 % but |v| < 5 rpm after 1.2 s).
Always ends with ~2 s of zero commands.
"""
import re, sys, time
import numpy as np
import serial
from serial.tools import list_ports

plan = [tuple(float(x) for x in seg.split(',')) for seg in sys.argv[1].split(';')]
log_path = sys.argv[2] if len(sys.argv) > 2 else 'drive_log.tsv'
RPM_PER_TICK = 60.0 / (1288.86 * 0.020)

port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
if port is None:
    sys.exit('no Holtek 04D9:802F COM port found')
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.02
ser.dtr = False
ser.rts = False
ser.open()

seq = 100
samples = []          # (t, phase, tgtL, tgtR, dutyL, dutyR, rpmL, rpmR, vL, vR, f)
cur = {'vL': 0.0, 'vR': 0.0, 'f': 0, 'dutyL': 0, 'dutyR': 0, 'rpmL': 0.0, 'rpmR': 0.0, 'sta_t': None}
abort_reason = None


def cmd_bytes(l, r):
    global seq
    payload = f'CMD,seq {seq},fwa 0,bwl {int(l)},bwr {int(r)}'
    seq = (seq + 1) % 65536
    cs = 0
    for ch in payload.encode():
        cs ^= ch
    return f'<{payload},cs {cs:02X}>\n'.encode()


def pump_rx(t0, phase, tl, tr):
    while True:
        line = ser.readline()
        if not line:
            return
        s = line.decode('ascii', errors='replace').strip()
        m = re.match(r'<STA,ackseq (-?\d+),S1 -?\d+,rpmL (-?[\d.]+),rpmR (-?[\d.]+),vL (-?\d+),vR (-?\d+),f (\d+),cs', s)
        if m:
            cur.update(vL=float(m.group(4)), vR=float(m.group(5)), f=int(m.group(6)), sta_t=time.time())
            continue
        m = re.match(r'dbg,(-?\d+),(-?\d+),(\d+),(\d+),(-?[\d.]+),(-?[\d.]+)', s)
        if m:
            cur.update(dutyL=int(m.group(3)), dutyR=int(m.group(4)), rpmL=float(m.group(5)), rpmR=float(m.group(6)))
            samples.append((time.time() - t0, phase, tl, tr, cur['dutyL'], cur['dutyR'], cur['rpmL'], cur['rpmR'],
                            cur['vL'], cur['vR'], cur['f']))


try:
    time.sleep(0.6)
    ser.reset_input_buffer()
    t0 = time.time()
    for ph, (tl, tr, dur) in enumerate(plan):
        t_end = time.time() + dur
        t_phase = time.time()
        next_tx = time.time()
        while time.time() < t_end and abort_reason is None:
            now = time.time()
            if now >= next_tx:
                ser.write(cmd_bytes(tl, tr))
                next_tx = now + 0.1
            pump_rx(t0, ph, tl, tr)
            # ---- guards ----
            if cur['f'] != 0:
                abort_reason = f'firmware fault flag f={cur["f"]} (bit0 L stall, bit1 R stall, bit2 PCA/dir I2C) in phase {ph}'
            elif max(abs(cur['vL']), abs(cur['vR'])) > 320:
                abort_reason = f'speed > 320 rpm in phase {ph}'
            elif cur['sta_t'] is not None and now - cur['sta_t'] > 1.0:
                abort_reason = 'no STA for > 1 s'
            elif now - t_phase > 1.2 and ((abs(tl) >= 15 and abs(cur['vL']) < 5) or (abs(tr) >= 15 and abs(cur['vR']) < 5)):
                abort_reason = f'a commanded wheel is not turning (vL={cur["vL"]:.0f}, vR={cur["vR"]:.0f}) in phase {ph}'
        if abort_reason:
            break
finally:
    t_end = time.time() + 2.0
    next_tx = time.time()
    while time.time() < t_end:
        if time.time() >= next_tx:
            ser.write(cmd_bytes(0, 0))
            next_tx = time.time() + 0.1
        pump_rx(t0, -1, 0, 0)
    ser.close()

with open(log_path, 'w', encoding='utf-8') as f:
    f.write('t\tphase\ttgtL\ttgtR\tdutyL\tdutyR\trpmL\trpmR\tvL\tvR\tf\n')
    for s in samples:
        f.write('\t'.join(f'{x:.2f}' if isinstance(x, float) else str(x) for x in s) + '\n')

print('abort:', abort_reason or 'none', '| samples:', len(samples))
print('\nphase  target(L/R %)  dur | steady last 1 s: dutyL dutyR  vL(rpm) vR(rpm) | first t to 90% of steady (L/R)  max overshoot % (L/R)')
for ph, (tl, tr, dur) in enumerate(plan):
    seg = [s for s in samples if s[1] == ph]
    if len(seg) < 3:
        print(f'  {ph}     {tl:.0f}/{tr:.0f}   {dur:.0f}s   (not enough samples)')
        continue
    arr = np.array([[s[0], s[4], s[5], s[8], s[9]] for s in seg])
    tail = arr[arr[:, 0] >= arr[-1, 0] - 1.0]
    dl, dr, vl, vr = tail[:, 1].mean(), tail[:, 2].mean(), tail[:, 3].mean(), tail[:, 4].mean()
    def t90(col, final):
        if abs(final) < 8:
            return None
        idx = np.where(np.abs(arr[:, col]) >= 0.9 * abs(final))[0]
        return float(arr[idx[0], 0] - arr[0, 0]) if len(idx) else None
    def over(col, final):
        if abs(final) < 8:
            return None
        return float((np.abs(arr[:, col]).max() - abs(final)) / abs(final) * 100)
    f2 = lambda v: '  -  ' if v is None else f'{v:5.1f}'
    print(f'  {ph}     {tl:3.0f}/{tr:3.0f}   {dur:3.0f}s |            {dl:5.0f} {dr:5.0f}  {vl:7.1f} {vr:7.1f} |      {f2(t90(3, vl))} / {f2(t90(4, vr))} s            {f2(over(3, vl))} / {f2(over(4, vr))}')
