"""Open-loop max-output test on the DIAG firmware (wheels must be lifted).
1) HL100 / HR100: one wheel at duty 255 for HOLD s, rpm from per-second encoder deltas ('holding,t=,posL,posR')
2) F100: both wheels, duty 0..250 step 10 (fb,duty,pct,tL,tR,rpmL,rpmR)
Always sends 'x' at the end / on any problem / on Ctrl-C. Aborts if |rpm| > 330.
"""
import re, sys, time, statistics as st
import serial
from serial.tools import list_ports

TPR = 1288.86
HOLD = 7.0
port = next(p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F)
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.05
ser.dtr = False
ser.rts = False
ser.open()


def send(s):
    ser.write((s + '\n').encode())


def stop():
    for _ in range(3):
        send('x')
        time.sleep(0.15)


def hold_test(cmd, label, fwd_sign_r=False):
    ser.reset_input_buffer()
    send(cmd)
    t0 = time.time()
    prev = None
    rpms = []
    log = []
    abort = None
    while time.time() - t0 < HOLD + 0.5 and abort is None:
        line = ser.readline().decode('ascii', 'replace').strip()
        if not line:
            continue
        log.append(line)
        if line.startswith(('err', 'abort')):
            abort = line
            break
        m = re.match(r'holding,t=(\d+)s,posL=(-?\d+),posR=(-?\d+)', line)
        if m:
            t, pl, pr = int(m.group(1)), int(m.group(2)), int(m.group(3))
            if prev is not None:
                dl, dr = pl - prev[1], -(pr - prev[2])  # forward-positive (M2 encoder is inverted)
                rl, rr = dl / TPR * 60.0, dr / TPR * 60.0
                rpms.append((t, rl, rr))
                print(f'  {label} t={t}s  rpmL={rl:7.1f}  rpmR={rr:7.1f}', flush=True)
                if max(abs(rl), abs(rr)) > 330:
                    abort = 'overspeed > 330 rpm'
            prev = (t, pl, pr)
    stop()
    return rpms, abort, log


try:
    time.sleep(0.5)
    ser.reset_input_buffer()
    stop()
    time.sleep(1.0)
    results = {}
    for cmd, label in (('HL100', 'L@255'), ('HR100', 'R@255')):
        rp, ab, log = hold_test(cmd, label)
        results[label] = rp
        if ab:
            print('ABORT:', ab, log[-3:])
            break
        time.sleep(2.0)
    for label, rp in results.items():
        steady = [(rl if label.startswith('L') else rr) for (t, rl, rr) in rp if t >= 4]
        if steady:
            print(f'>>> {label}: steady (t>=4s) mean = {st.mean(steady):.1f} rpm, min {min(steady):.1f}, max {max(steady):.1f}')
    # both wheels sweep
    print('--- F100 both-wheel sweep (duty, pct, rpmL, rpmR)')
    ser.reset_input_buffer()
    send('F100')
    t0 = time.time()
    done = False
    while time.time() - t0 < 45 and not done:
        line = ser.readline().decode('ascii', 'replace').strip()
        if not line:
            continue
        if line.startswith('fb,'):
            p = line.split(',')
            rl, rr = float(p[5]), float(p[6])
            print(f'  duty {int(p[1]):3d} ({p[2]:>3}%)  rpmL={rl:7.1f}  rpmR={rr:7.1f}', flush=True)
            if max(abs(rl), abs(rr)) > 330:
                print('ABORT overspeed'); break
        elif line.startswith(('fdone', 'abort', 'err')):
            print(' ', line)
            done = True
finally:
    stop()
    ser.close()
    print('stopped (x sent)')
