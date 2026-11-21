"""Steady-state stability of BOTH wheels from the Orin feedback only (STA rpmL/rpmR/vL/vR/f over UART).

usage (stdin on the Orin): python3 - "<pct,pct,...>" [hold_s=6] [rest_s=2]
Both wheels get the same command at 10 Hz (fwa 0). Per step: steady window = last 3 s of the hold.
Aborts to zero on: fault flag f != 0, |v| > 320 rpm, UART feedback lost, a wheel commanded >= 15 % not turning after 1.5 s.
Always ends with a confirmed zero.
"""
import statistics as st, subprocess, sys, time
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'car2_ws/src/car2_bringup'))
import serial
from car2_bringup.uart_protocol import build_cmd, parse_sta, send_chunked

cmds = [float(x) for x in sys.argv[1].split(',')]
HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
REST = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
if any(abs(c) > 100 for c in cmds) or HOLD > 10:
    raise SystemExit('refusing: |pct| > 100 or hold > 10 s')


def expected_rpm(p):
    p = abs(p)
    return 0.0 if p <= 0 else (5.0 + 0.95 * p) / 100.0 * 116.0 * 2.3275  # 116 ticks/20 ms = 270 rpm at 100 % (14 V recalibration 2026-10-05)


if subprocess.run(['fuser', '/dev/ttyTHS1'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
    raise SystemExit('UART already in use; cancelled')
port = serial.Serial('/dev/ttyTHS1', 460800, timeout=0.02, write_timeout=0.2, exclusive=True)
seq = 20000
cur = {'rpmL': 0.0, 'rpmR': 0.0, 'vL': 0.0, 'vR': 0.0, 'f': 0, 'sta_t': None, 'ack': None, 'ack_t': None}
rows = []   # (phase, t_rel_to_phase_start, rpmL, rpmR, vL, vR, f)
abort = None
bad_lines = 0
n_cmd = 0


def pump(phase, tstart):
    global bad_lines
    line = port.readline()
    if not line:
        return
    s = parse_sta(line)
    if not s:
        bad_lines += 1
        return
    try:
        now = time.monotonic()
        cur.update(rpmL=float(s['rpmL']), rpmR=float(s['rpmR']), vL=float(s['vL']), vR=float(s['vR']),
                   f=int(s['f']), sta_t=now)
        a = int(s['ackseq'])
        if a != cur['ack']:
            cur['ack'], cur['ack_t'] = a, now
    except (KeyError, ValueError):
        return
    rows.append((phase, now - tstart, cur['rpmL'], cur['rpmR'], cur['vL'], cur['vR'], cur['f']))


def run(phase, pct, dur):
    global seq, abort, n_cmd
    start = time.monotonic()
    nxt = start
    while time.monotonic() - start < dur and abort is None:
        now = time.monotonic()
        if now >= nxt:
            send_chunked(port, build_cmd(seq, 0, pct, pct))
            seq = (seq + 1) % 65536
            n_cmd += 1
            nxt = now + 0.1
        pump(phase, start)
        if pct != 0:
            if cur['f'] != 0:
                abort = f'firmware fault f={cur["f"]} at {pct}%'
            elif max(abs(cur['vL']), abs(cur['vR'])) > 320:
                abort = f'speed > 320 rpm at {pct}%'
            elif cur['sta_t'] is None or now - cur['sta_t'] > 0.6 or cur['ack_t'] is None or now - cur['ack_t'] > 0.8:
                abort = f'UART feedback lost at {pct}%'
            elif now - start > 1.5 and abs(pct) >= 15 and (abs(cur['vL']) < 5 or abs(cur['vR']) < 5):
                abort = f'a wheel is not turning at {pct}% (vL={cur["vL"]:.0f}, vR={cur["vR"]:.0f})'


done = []
final_zero = None
try:
    run(-1, 0, 1.0)
    for i, pct in enumerate(cmds):
        run(i, pct, HOLD)
        done.append((i, pct))
        if abort:
            break
        run(-2, 0, REST)
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


def slope(ts, vs):
    mt, mv = st.mean(ts), st.mean(vs)
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den if den else 0.0


print(f'abort: {abort or "none"} | final zero confirmed: {final_zero} | STA samples: {len(rows)} | bad lines: {bad_lines} | CMD sent: {n_cmd}')
all_f = sorted({r[6] for r in rows})
print(f'fault flags seen: {all_f}')
print('\ncmd%  tgt  |  wheel  mean   std  p2p  CV%  drift/s | settle(s) overshoot% | L-R diff%')
for i, pct in done:
    seg = [r for r in rows if r[0] == i]
    if len(seg) < 8:
        continue
    t_end = seg[-1][1]
    tail = [r for r in seg if r[1] >= t_end - 3.0]
    if len(tail) < 5:
        continue
    exp = expected_rpm(pct)
    out = []
    for name, idx in (('L', 2), ('R', 3)):
        v = [abs(r[idx]) for r in tail]
        ts = [r[1] for r in tail]
        m = st.mean(v)
        s = st.pstdev(v)
        p2p = max(v) - min(v)
        dr = slope(ts, v)
        # settle: first time after which all samples stay within +-6 % of the steady mean
        settle = None
        for k, r in enumerate(seg):
            if all(abs(abs(x[idx]) - m) <= 0.06 * m for x in seg[k:]):
                settle = r[1]
                break
        peak = max(abs(r[idx]) for r in seg)
        over = (peak - m) / m * 100.0
        out.append((name, m, s, p2p, s / m * 100.0, dr, settle, over))
    mL, mR = out[0][1], out[1][1]
    diff = (mL - mR) / ((mL + mR) / 2) * 100.0
    for k, (name, m, s, p2p, cv, dr, settle, over) in enumerate(out):
        head = f'{pct:4.0f} {exp:5.1f}  |' if k == 0 else '                |'
        tail_txt = f' {diff:+6.1f}' if k == 0 else ''
        st_txt = f'{settle:5.2f}' if settle is not None else ' none'
        print(f'{head}   {name}   {m:6.1f} {s:5.2f} {p2p:5.1f} {cv:4.1f} {dr:+6.2f}  | {st_txt}   {over:+6.1f}   |{tail_txt}')
