import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM10"
out_path = sys.argv[2] if len(sys.argv) > 2 else "sweep_data.csv"

ser = serial.Serial(port, 115200, timeout=1.0)
time.sleep(2.0)
ser.reset_input_buffer()

rows = []
start = time.time()
while time.time() - start < 60:
    line = ser.readline()
    if not line:
        continue
    s = line.decode(errors="ignore").strip()
    if s:
        print(s)
    if s.startswith("CSV,"):
        parts = s.split(",")
        rows.append((int(parts[1]), float(parts[2])))
    if "sweep done" in s:
        break

ser.close()

with open(out_path, "w", encoding="utf-8") as f:
    f.write("duty,rpm\n")
    for duty, rpm in rows:
        f.write(f"{duty},{rpm}\n")

print(f"saved {len(rows)} rows to {out_path}")
