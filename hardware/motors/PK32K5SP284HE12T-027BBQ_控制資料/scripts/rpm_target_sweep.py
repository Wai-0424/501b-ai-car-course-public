import serial
import time

PORT = "COM10"

# 換算常數（見 hall_motor_pid_tune.ino）：
# TICKS_PER_REV=48(馬達軸), PID_INTERVAL_MS=20, 減速比 3249/121=26.848
# MAX_TICKS_PER_INTERVAL=129.52 對應馬達軸滿速 8095rpm，即輸出軸約301.5rpm
GEAR_RATIO = 3249.0 / 121.0
MAX_TICKS_PER_INTERVAL = 129.52
TICKS_TO_OUTPUT_RPM = 62.5 / GEAR_RATIO  # ticks/interval -> output rpm


def output_rpm_to_percent(output_rpm):
    motor_rpm = output_rpm * GEAR_RATIO
    ticks_per_interval = motor_rpm * 0.016
    return ticks_per_interval / MAX_TICKS_PER_INTERVAL * 100.0


ser = serial.Serial(PORT, 115200, timeout=0.2)
time.sleep(2.0)
ser.reset_input_buffer()


def send(cmd):
    ser.write((cmd + "\n").encode())


def wait_stopped(max_s=3.0):
    send("t 0")
    end = time.time() + max_s
    quiet = 0
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if s.startswith("PID,"):
            try:
                a = abs(float(s.split(",")[2]))
            except (ValueError, IndexError):
                continue
            quiet = quiet + 1 if a < 2.0 else 0
            if quiet >= 5:
                return


def run_target(output_rpm, seconds=3.5):
    pct = output_rpm_to_percent(output_rpm)
    capped = pct > 100.0
    pct_send = min(pct, 100.0)

    wait_stopped()
    send("reset")
    time.sleep(0.1)
    send(f"t {pct_send:.2f}")

    rows = []
    end = time.time() + seconds
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if s.startswith("PID,"):
            parts = s.split(",")
            try:
                actual_ticks = float(parts[2])
                duty = int(parts[3])
                rows.append((actual_ticks, duty))
            except (ValueError, IndexError):
                pass

    if not rows:
        print(f"target={output_rpm:6.1f}rpm  (no data)")
        return

    tail = rows[-max(3, len(rows) // 4):]
    tail_rpm = [r[0] * TICKS_TO_OUTPUT_RPM for r in tail]
    settled_rpm = sum(tail_rpm) / len(tail_rpm)
    final_duty = tail[-1][1]
    err_pct = (settled_rpm - output_rpm) / output_rpm * 100.0

    flag = "  [!! 超過馬達物理極速，duty已頂到255附近，理論上追不到]" if capped else ""
    print(f"target={output_rpm:6.1f}rpm (需求duty%={pct:6.1f})  settled={settled_rpm:6.1f}rpm  "
          f"err={err_pct:+6.1f}%  final_duty={final_duty:3d}{flag}")


for rpm in [60, 90, 120, 150, 180, 210, 240, 270, 300, 330]:
    run_target(rpm)

send("t 0")
ser.close()
