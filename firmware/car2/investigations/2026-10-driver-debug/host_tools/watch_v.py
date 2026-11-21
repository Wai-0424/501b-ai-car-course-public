"""PASSIVE watch of the new always-on speed fields (vL, vR, f). Never transmits anything.
usage (stdin on the Orin):  python3 - <seconds>
"""
import sys, time, subprocess
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'car2_ws/src/car2_bringup'))
import serial
from car2_bringup.uart_protocol import parse_sta

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
if subprocess.run(['fuser', '/dev/ttyTHS1'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0:
    raise SystemExit('UART busy')
port = serial.Serial('/dev/ttyTHS1', 460800, timeout=0.05, exclusive=True)
t0 = time.monotonic()
rows, bad, keys = [], 0, set()
while time.monotonic() - t0 < secs:
    line = port.readline()
    if not line:
        continue
    p = parse_sta(line)
    if p:
        keys |= set(p)
        try:
            rows.append((time.monotonic() - t0, float(p['rpmL']), float(p['rpmR']),
                         float(p.get('vL', 'nan')), float(p.get('vR', 'nan')), int(p.get('f', -1)), p['ackseq']))
        except ValueError:
            bad += 1
    elif line.endswith(b'\n') and line.strip():
        bad += 1
port.close()
print(f'STA frames {len(rows)} | bad {bad} | fields {sorted(keys)}')
print('ackseq seen:', sorted({r[6] for r in rows}), '| f flags seen:', sorted({r[5] for r in rows}))
print(' sec |   vL min..max    |   vR min..max    | rpmL max | rpmR max')
for k in range(int(secs)):
    w = [r for r in rows if k <= r[0] < k + 1]
    if not w:
        continue
    print(f' {k:3d} | {min(r[3] for r in w):6.0f}..{max(r[3] for r in w):<6.0f}  | {min(r[4] for r in w):6.0f}..{max(r[4] for r in w):<6.0f}  |'
          f' {max(abs(r[1]) for r in w):7.1f}  | {max(abs(r[2]) for r in w):7.1f}')
