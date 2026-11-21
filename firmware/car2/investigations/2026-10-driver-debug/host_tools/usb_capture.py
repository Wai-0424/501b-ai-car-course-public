"""Passive BMduino USB debug capture (receive only; nothing is ever written to the port).

usage: py -3 usb_capture.py <seconds> <out_log> [COMx]
Finds the Holtek CMSIS-DAP virtual COM (VID 04D9 / PID 802F) unless a port is given.
Every received line is stored as  <epoch_seconds>\t<raw line>.
dbg fields: dbg,targetL,targetR,dutyL,dutyR,rpmL,rpmR,i2cCalls,i2cErr,
            stallEvM1,stallMaxM1,stallEvM2,stallMaxM2,pwmPump,poll,pid,status
"""
import sys, time, statistics as st
import serial
from serial.tools import list_ports

seconds = float(sys.argv[1])
out = sys.argv[2]
port = sys.argv[3] if len(sys.argv) > 3 else None
if port is None:
    for p in list_ports.comports():
        if p.vid == 0x04D9 and p.pid == 0x802F:
            port = p.device
            break
if port is None:
    print("no Holtek 04D9:802F COM port found; ports:", [(p.device, hex(p.vid or 0), hex(p.pid or 0)) for p in list_ports.comports()])
    sys.exit(2)

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = port, 115200, 0.1
ser.dtr = False  # try not to toggle reset on open
ser.rts = False
ser.open()
print("opened", port)
rows, t0 = [], time.time()
with open(out, "w", encoding="utf-8") as f:
    while time.time() - t0 < seconds:
        line = ser.readline()
        if not line:
            continue
        t = time.time()
        text = line.decode("ascii", errors="replace").rstrip("\r\n")
        f.write(f"{t:.3f}\t{text}\n")
        rows.append((t, text))
ser.close()

dbg = []
sta = 0
for t, text in rows:
    if text.startswith("dbg,"):
        parts = text.split(",")
        try:
            dbg.append((t, [float(x) for x in parts[1:]]))
        except ValueError:
            pass
    elif text.startswith("<STA"):
        sta += 1
print(f"lines={len(rows)} dbg_lines={len(dbg)} sta_lines={sta}  -> {out}")
for t, text in rows[:6]:
    print("  ", text[:150])
if dbg:
    cols = {"targetL": 0, "targetR": 1, "dutyL": 2, "dutyR": 3, "rpmL": 4, "rpmR": 5, "i2cErr": 7, "stallEvM1": 8, "stallEvM2": 10}
    for k, i in cols.items():
        v = [d[1][i] for d in dbg]
        print(f"  {k:<10} min {min(v):8.1f}  max {max(v):8.1f}  last {v[-1]:8.1f}")
    calls = [d[1][6] for d in dbg]
    print(f"  i2cCalls   first {calls[0]:.0f} last {calls[-1]:.0f}  (delta {calls[-1] - calls[0]:.0f})")
