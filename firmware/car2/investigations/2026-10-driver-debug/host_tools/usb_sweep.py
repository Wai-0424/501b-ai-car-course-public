"""Drive the per-wheel duty sweep firmware over USB (COM of the Holtek 04D9:802F).

usage: py -3 usb_sweep.py <command like L100 | R100 | l60 | r60> <log file> [max_seconds=45]
Always sends 'x' (stop) on exit, including on errors / Ctrl-C.
"""
import sys, time
import serial
from serial.tools import list_ports

cmd, log_path = sys.argv[1], sys.argv[2]
max_s = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0

port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
if port is None:
    sys.exit('no Holtek 04D9:802F COM port found')
ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.dtr = False
ser.rts = False
ser.open()
rows, events = [], []
log = open(log_path, 'w', encoding='utf-8')


def read_for(seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        line = ser.readline().decode('ascii', errors='replace').strip()
        if line:
            log.write(f'{time.time():.3f}\t{line}\n')
            yield line


try:
    for _ in read_for(0.6):  # let any boot banner / idle beat arrive, and drop it
        pass
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode())
    print('sent:', cmd)
    finished = False
    t0 = time.time()
    while time.time() - t0 < max_s and not finished:
        for line in read_for(0.5):
            if line.startswith('sw,'):
                f = line.split(',')
                rows.append((f[1], f[2], int(f[3]), int(f[4]), int(f[5]), float(f[6])))
            elif line.startswith(('start', 'done', 'abort', 'err', 'pca', 'ok')):
                events.append(line)
                if line.startswith(('done', 'abort', 'err')):
                    finished = True
finally:
    try:
        ser.write(b'x\n')
        time.sleep(0.3)
    finally:
        ser.close()
        log.close()

print('events:')
for e in events:
    print('  ', e)
print('\n duty  pct   ticks     rpm   (forward positive)')
for w, d, duty, pct, moved, rpm in rows:
    flag = '  <-- moving' if abs(moved) >= 20 else ''
    print(f' {duty:4d} {pct:4d}% {moved:7d} {rpm:8.1f}{flag}')
if not rows:
    print(' (no sweep rows received)')
