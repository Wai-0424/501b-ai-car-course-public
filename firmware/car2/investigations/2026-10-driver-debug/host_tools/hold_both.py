"""Hold BOTH wheels at a fixed duty (diag firmware, HB<pct>) for N seconds, log per-second rpm from encoder deltas.
usage: python hold_both.py <pct> <seconds>
Aborts (sends x) on: |rpm| > 330, a wheel < 20 rpm after 4 s (stall / no drive), firmware err/abort, Ctrl-C. Always sends x at the end.
"""
import re, sys, time
import serial
from serial.tools import list_ports

TPR = 1288.86
pct = int(sys.argv[1])
secs = float(sys.argv[2])
if not (1 <= pct <= 90) or secs > 120:
    raise SystemExit('refusing: pct must be 1..90, seconds <= 120')
port = next(p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F)
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.05
ser.dtr = False
ser.rts = False
ser.open()


def stop():
    for _ in range(3):
        ser.write(b'x\n')
        time.sleep(0.15)


abort = None
prev = None
rows = []
try:
    time.sleep(0.5)
    ser.reset_input_buffer()
    stop()
    time.sleep(0.8)
    ser.reset_input_buffer()
    ser.write(f'HB{pct}\n'.encode())
    t0 = time.time()
    print(f'>>> HB{pct} sent (both wheels forward, ~{secs:.0f} s). Touch-check the parts now.', flush=True)
    while time.time() - t0 < secs + 1.0 and abort is None:
        line = ser.readline().decode('ascii', 'replace').strip()
        if not line:
            continue
        if line.startswith(('err', 'abort')):
            abort = line
            break
        if line.startswith('hold,'):
            print('   ', line, flush=True)
        m = re.match(r'holding,t=(\d+)s,posL=(-?\d+),posR=(-?\d+)', line)
        if m:
            t, pl, pr = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if prev is not None:
                rl = (pl - prev[1]) / TPR * 60.0
                rr = -(pr - prev[2]) / TPR * 60.0
                rows.append((t, rl, rr))
                print(f'   t={t:3d}s  rpmL={rl:7.1f}  rpmR={rr:7.1f}', flush=True)
                if max(abs(rl), abs(rr)) > 330:
                    abort = 'overspeed > 330 rpm'
                elif t >= 4 and min(abs(rl), abs(rr)) < 20:
                    abort = f'a wheel is not turning (rpmL={rl:.0f}, rpmR={rr:.0f})'
            prev = (t, pl, pr)
        if time.time() - t0 >= secs:
            break
finally:
    stop()
    ser.close()
if abort:
    print('ABORT:', abort)
if rows:
    import statistics as st
    for name, i in (('L', 1), ('R', 2)):
        v = [r[i] for r in rows if r[0] >= 4]
        if v:
            print(f'{name}: steady mean {st.mean(v):.1f} rpm (min {min(v):.1f}, max {max(v):.1f}); first 5 s mean {st.mean([r[i] for r in rows if r[0] <= 6]):.1f}, last 5 s mean {st.mean([r[i] for r in rows[-5:]]):.1f}')
print('stopped (x sent)')
