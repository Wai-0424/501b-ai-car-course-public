"""Speed-tracking accuracy of the CAR firmware, driven from the Orin over UART (no ROS nodes).

usage (stdin on the Orin): python3 - "<pct,pct,...>" [hold_s=3.5] [rest_s=1.5]
Each command is sent to BOTH wheels at 10 Hz for hold_s, then 0 for rest_s. Reports steady-state speed over the
last 1.5 s of each hold vs the firmware's mapping (effective% = 5 + 0.95*pct of 105 ticks/20 ms, 2.3275 rpm per tick).
Aborts to zero on: fault flag f != 0, |speed| > 320 rpm, UART feedback lost, a wheel commanded >=15 % that is not
turning after 1.2 s. Always ends with a confirmed zero.
"""
import json, math, statistics as st, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'car2_ws/src/car2_bringup'))
import serial
from car2_bringup.uart_protocol import build_cmd, parse_sta, send_chunked

cmds = [float(x) for x in sys.argv[1].split(',')]
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 3.5
REST = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
if any(abs(c) > 100 for c in cmds) or HOLD > 8:
    raise SystemExit('refusing: |pct| > 100 or hold > 8 s')


def expected_rpm(p):
    p = abs(p)
    return 0.0 if p <= 0 else (5.0 + 0.95 * p) / 100.0 * 105.0 * 2.3275


if subprocess.run(['fuser', '/dev/ttyTHS1'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
    raise SystemExit('UART already in use; cancelled')
port = serial.Serial('/dev/ttyTHS1', 460800, timeout=0.02, write_timeout=0.2, exclusive=True)
seq = 8000
cur = {'rpmL': 0.0, 'rpmR': 0.0, 'vL': 0.0, 'vR': 0.0, 'f': 0, 'sta_t': None, 'ack_t': None, 'ack': None}
rows = []        # (phase, t, rpmL, rpmR, vL, vR)
abort = None


def pump(phase, t0):
    line = port.readline()
    s = parse_sta(line) if line else None
    if not s:
        return
    try:
        cur.update(rpmL=float(s['rpmL']), rpmR=float(s['rpmR']), vL=float(s['vL']), vR=float(s['vR']), f=int(s['f']),
                   sta_t=time.monotonic())
        a = int(s['ackseq'])
        if a != cur['ack']:
            cur['ack'], cur['ack_t'] = a, time.monotonic()
    except (KeyError, ValueError):
        return
    rows.append((phase, time.monotonic() - t0, cur['rpmL'], cur['rpmR'], cur['vL'], cur['vR']))


def run(phase, pct, dur, t0):
    global seq, abort
    start = time.monotonic()
    nxt = start
    while time.monotonic() - start < dur and abort is None:
        now = time.monotonic()
        if now >= nxt:
            send_chunked(port, build_cmd(seq, 0, pct, pct))
            seq = (seq + 1) % 65536
            nxt = now + 0.1
        pump(phase, t0)
        if pct != 0:
            if cur['f'] != 0:
                abort = f'firmware fault f={cur["f"]} at {pct}%'
            elif max(abs(cur['vL']), abs(cur['vR'])) > 320:
                abort = f'speed > 320 rpm at {pct}%'
            elif cur['sta_t'] is None or now - cur['sta_t'] > 0.6 or cur['ack_t'] is None or now - cur['ack_t'] > 0.8:
                abort = f'UART feedback lost at {pct}%'
            elif now - start > 1.2 and abs(pct) >= 15 and (abs(cur['vL']) < 5 or abs(cur['vR']) < 5):
                abort = f'a wheel is not turning at {pct}% (vL={cur["vL"]:.0f}, vR={cur["vR"]:.0f})'


phases = []
final_zero = None
try:
    t0 = time.monotonic()
    run(-1, 0, 1.0, t0)                                  # alive check at zero
    for i, pct in enumerate(cmds):
        run(i, pct, HOLD, t0)
        phases.append((i, pct))
        if abort:
            break
        run(-2, 0, REST, t0)
        if abort:
            break
finally:
    for _ in range(25):
        send_chunked(port, build_cmd(seq, 0, 0, 0))
        seq = (seq + 1) % 65536
        pump(-3, time.monotonic())
        time.sleep(0.08)
    final_zero = abs(cur['vL']) <= 3 and abs(cur['vR']) <= 3
    port.close()

print(f'abort: {abort or "none"} | final zero confirmed: {final_zero} | STA samples: {len(rows)}')
print('\n cmd%  expected | L: mean   std  err%  | R: mean   std  err%  | L-R diff% | sign(v) L/R | worst |err|')
for i, pct in phases:
    seg = [r for r in rows if r[0] == i]
    if not seg:
        continue
    tail = [r for r in seg if r[1] >= seg[-1][1] - 1.5]
    if len(tail) < 3:
        continue
    exp = expected_rpm(pct)
    L = [r[2] for r in tail]
    R = [r[3] for r in tail]
    vL = st.mean(r[4] for r in tail)
    vR = st.mean(r[5] for r in tail)
    mL, mR = st.mean(L), st.mean(R)
    sL, sR = st.pstdev(L), st.pstdev(R)
    eL, eR = (mL - exp) / exp * 100, (mR - exp) / exp * 100
    diff = (mL - mR) / ((mL + mR) / 2) * 100
    sg = lambda v: '+' if v > 0 else '-'
    print(f' {pct:5.0f}  {exp:6.1f}  | {mL:6.1f} {sL:5.1f} {eL:+5.1f}% | {mR:6.1f} {sR:5.1f} {eR:+5.1f}% |  {diff:+5.1f}%   |   {sg(vL)} / {sg(vR)}    |  {max(abs(eL), abs(eR)):4.1f}%')
