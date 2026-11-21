"""Open-loop calibration sweeps on the DIAG firmware (wheels lifted): up-sweep L, up-sweep R, then both-wheel down-sweep (V).
usage: python calib_sweeps.py <out_prefix> [up_pct=60]
Writes <out_prefix>_up_L.csv, _up_R.csv, _down_both.csv (duty,rpm...). Always sends x at the end; aborts on |rpm|>330.
"""
import sys, time
import serial
from serial.tools import list_ports

prefix = sys.argv[1]
up_pct = sys.argv[2] if len(sys.argv) > 2 else '60'
port = next(p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F)
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.dtr = False
ser.rts = False
ser.open()


def stop():
    for _ in range(3):
        ser.write(b'x\n')
        time.sleep(0.15)


def run(cmd, tag, enders, max_s):
    rows, events = [], []
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    t0 = time.time()
    done = False
    while time.time() - t0 < max_s and not done:
        line = ser.readline().decode('ascii', 'replace').strip()
        if not line:
            continue
        p = line.split(',')
        if p[0] == tag:
            rows.append(p[1:])
            try:
                if max(abs(float(x)) for x in p[-2:]) > 330 if tag == 'vb' else abs(float(p[-1])) > 330:
                    events.append('ABORT overspeed')
                    stop()
                    done = True
            except ValueError:
                pass
        elif p[0] in enders or p[0] in ('abort', 'err'):
            events.append(line)
            done = True
        elif p[0] in ('start', 'vstart', 'pca', 'ok'):
            events.append(line)
    return rows, events


try:
    time.sleep(0.5)
    stop()
    time.sleep(0.8)
    out = {}
    for w in ('L', 'R'):
        rows, ev = run(f'{w}{up_pct}', 'sw', ('done',), 60)
        stop()
        time.sleep(1.5)
        out[w] = rows
        with open(f'{prefix}_up_{w}.csv', 'w') as f:
            f.write('duty,pct,ticks,rpm\n')
            for r in rows:
                f.write(','.join([r[2], r[3], r[4], r[5]]) + '\n')
        print(f'== up-sweep {w}: events {ev}')
        print(' duty   rpm')
        for r in rows:
            print(f' {int(r[2]):4d} {float(r[5]):7.1f}')
    rows, ev = run('V', 'vb', ('vdone',), 80)
    stop()
    with open(f'{prefix}_down_both.csv', 'w') as f:
        f.write('duty,pct,ticksL,ticksR,rpmL,rpmR\n')
        for r in rows:
            f.write(','.join(r) + '\n')
    print(f'== down-sweep both: events {ev}')
    print(' duty  rpmL   rpmR')
    for r in rows:
        print(f' {int(r[0]):4d} {float(r[4]):6.1f} {float(r[5]):6.1f}')
finally:
    stop()
    ser.close()
    print('stopped (x sent)')
