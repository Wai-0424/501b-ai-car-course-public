"""Low-speed scan of the CAR firmware over USB (wheels lifted). Each step: hold <pct> on both wheels for HOLD s, then 0 for REST s.
A firmware stall fault (f != 0) does NOT abort the scan: it is logged as STALL, the step ends early and zero is sent for REST s.
Aborts only on overspeed (>320 rpm), no STA for >1.5 s, or Ctrl-C. Always ends with zero commands.

usage: python lowspeed_scan.py "<pct,pct,...>" [hold_s=4.5] [rest_s=2.5] [out.tsv]
"""
import re, sys, time, statistics as st
import serial
from serial.tools import list_ports

pcts = [float(x) for x in sys.argv[1].split(',')]
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 4.5
REST = float(sys.argv[3]) if len(sys.argv) > 3 else 2.5
OUT = sys.argv[4] if len(sys.argv) > 4 else None
MAXT, RPT = 116.0, 2.3275

port = next(p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F)
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.02
ser.dtr = False
ser.rts = False
ser.open()
seq = 500
cur = dict(vL=0.0, vR=0.0, f=0, dutyL=0, dutyR=0, rpmL=0.0, rpmR=0.0, sta_t=None)
samples = []


def cmd_bytes(l, r):
    global seq
    payload = f'CMD,seq {seq},fwa 0,bwl {int(l)},bwr {int(r)}'
    seq = (seq + 1) % 65536
    cs = 0
    for ch in payload.encode():
        cs ^= ch
    return f'<{payload},cs {cs:02X}>\n'.encode()


def pump(t0, phase, p):
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
            samples.append((time.time() - t0, phase, p, cur['dutyL'], cur['dutyR'], cur['rpmL'], cur['rpmR'], cur['vL'], cur['vR'], cur['f']))


def run(t0, phase, p, dur, stop_on_fault):
    end = time.time() + dur
    nxt = time.time()
    fault = False
    while time.time() < end:
        now = time.time()
        if now >= nxt:
            ser.write(cmd_bytes(p, p))
            nxt = now + 0.1
        pump(t0, phase, p)
        if max(abs(cur['vL']), abs(cur['vR'])) > 320:
            raise SystemExit('ABORT overspeed')
        if cur['sta_t'] is not None and now - cur['sta_t'] > 1.5:
            raise SystemExit('ABORT no STA')
        if stop_on_fault and cur['f'] != 0 and p != 0:
            fault = True
            break
    return fault


summary = []
try:
    time.sleep(0.6)
    ser.reset_input_buffer()
    t0 = time.time()
    run(t0, -1, 0, 1.0, False)
    for i, p in enumerate(pcts):
        fault = run(t0, i, p, HOLD, True)
        n_before = len(samples)
        run(t0, -2, 0, REST, False)   # zero (also clears a latched fault: needs 300 ms of 0)
        summary.append((i, p, fault))
finally:
    end = time.time() + 2.0
    while time.time() < end:
        ser.write(cmd_bytes(0, 0))
        time.sleep(0.1)
        pump(t0, -3, 0)
    ser.close()

if OUT:
    with open(OUT, 'w') as f:
        f.write('t\tphase\tpct\tdutyL\tdutyR\trpmL\trpmR\tvL\tvR\tf\n')
        for s in samples:
            f.write('\t'.join(f'{x:.2f}' if isinstance(x, float) else str(x) for x in s) + '\n')

print(' cmd%  target |  L: rpm   std  duty | R: rpm   std  duty | start(s) L/R | fault')
for i, p, fault in summary:
    seg = [s for s in samples if s[1] == i]
    if len(seg) < 3:
        print(f'{p:5.1f}  (no samples)')
        continue
    tgt = (5 + 0.95 * p) / 100 * MAXT * RPT
    t_a = seg[0][0]
    tail = [s for s in seg if s[0] >= seg[-1][0] - 1.5]
    def start(idx):
        for s in seg:
            if abs(s[idx]) > 10:
                return s[0] - t_a
        return None
    L = [s[5] for s in tail]; R = [s[6] for s in tail]
    fl = sorted({s[9] for s in seg})
    f2 = lambda v: ' -- ' if v is None else f'{v:4.1f}'
    print(f'{p:5.1f} {tgt:6.1f} | {st.mean(L):6.1f} {st.pstdev(L):5.1f} {tail[-1][3]:4d} | {st.mean(R):6.1f} {st.pstdev(R):5.1f} {tail[-1][4]:4d} |  {f2(start(5))}/{f2(start(6))}   | {"STALL f=" + str(fl) if fault else "ok"}')
