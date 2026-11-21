"""Independent revolution check: both wheels at <pct>% for <hold_s> s (after a countdown), then coast to a stop.
Integrates the firmware's measured wheel speed (STA rpm, 5 Hz, trapezoid) over the WHOLE motion (start -> stopped)
so it can be compared with a human revolution count (mark on the wheel).

usage (stdin on the Orin): python3 - <pct> <hold_s> <countdown_s>
Always ends with a confirmed zero. Aborts to zero on fault flag / overspeed / UART loss.
"""
import subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'car2_ws/src/car2_bringup'))
import serial
from car2_bringup.uart_protocol import build_cmd, parse_sta, send_chunked

pct = float(sys.argv[1])
hold = float(sys.argv[2])
countdown = float(sys.argv[3])
if not (0 < pct <= 60) or hold > 40 or countdown > 30:
    raise SystemExit('refusing: pct must be 1..60, hold <= 40 s, countdown <= 30 s')
expected_rpm = (5.0 + 0.95 * pct) / 100.0 * 105.0 * 2.3275

if subprocess.run(['fuser', '/dev/ttyTHS1'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
    raise SystemExit('UART already in use; cancelled')
port = serial.Serial('/dev/ttyTHS1', 460800, timeout=0.02, write_timeout=0.2, exclusive=True)
seq = 9000
cur = {'rpmL': 0.0, 'rpmR': 0.0, 'f': 0, 'sta_t': None}
samples = []          # (t, |vL|, |vR|)
abort = None
t_move_start = None


def pump(t0, record):
    line = port.readline()
    s = parse_sta(line) if line else None
    if not s:
        return
    try:
        cur.update(vL=abs(float(s['vL'])), vR=abs(float(s['vR'])), f=int(s['f']), sta_t=time.monotonic())
    except (KeyError, ValueError):
        return
    if record:
        samples.append((time.monotonic() - t0, cur['vL'], cur['vR']))


def phase(p, dur, t0, record, guard):
    global seq, abort
    start = time.monotonic()
    nxt = start
    while time.monotonic() - start < dur and abort is None:
        now = time.monotonic()
        if now >= nxt:
            send_chunked(port, build_cmd(seq, 0, p, p))
            seq = (seq + 1) % 65536
            nxt = now + 0.1
        pump(t0, record)
        if guard:
            if cur['f'] != 0:
                abort = f'firmware fault f={cur["f"]}'
            elif max(cur.get('vL', 0), cur.get('vR', 0)) > 320:
                abort = 'speed > 320 rpm'
            elif cur['sta_t'] is None or now - cur['sta_t'] > 0.8:
                abort = 'UART feedback lost'


try:
    t0 = time.monotonic()
    phase(0, countdown, t0, False, False)                   # countdown at zero (keeps the link alive)
    t0 = time.monotonic()                                    # t=0 is the moment motion is commanded
    print('>>> GO: motion commanded now (count revolutions from here)', flush=True)
    phase(pct, hold, t0, True, True)
    phase(0, 4.0, t0, True, False)                           # coast to a stop (firmware sees zero commands)
finally:
    for _ in range(15):
        send_chunked(port, build_cmd(seq, 0, 0, 0))
        seq = (seq + 1) % 65536
        pump(time.monotonic(), False)
        time.sleep(0.08)
    port.close()


def integrate(idx):
    revs = 0.0
    for (ta, *va), (tb, *vb) in zip(samples, samples[1:]):
        revs += 0.5 * (va[idx] + vb[idx]) * (tb - ta) / 60.0
    return revs


def moving_window(idx):
    mv = [s[0] for s in samples if s[1 + idx] > 3.0]
    return (mv[0], mv[-1]) if mv else (None, None)


print(f'abort: {abort or "none"} | samples: {len(samples)} | commanded {pct:.0f}% for {hold:.0f} s (target {expected_rpm:.1f} rpm)')
print(f'theory for the {hold:.0f} s hold only: {expected_rpm * hold / 60.0:.2f} revolutions per wheel\n')
for name, idx in (('LEFT ', 0), ('RIGHT', 1)):
    a, b = moving_window(idx)
    print(f'{name}: encoder-integrated total = {integrate(idx):6.2f} revolutions (from first motion to stop)'
          f'   moving from t={a:.1f}s to t={b:.1f}s' if a is not None else f'{name}: did not move')
