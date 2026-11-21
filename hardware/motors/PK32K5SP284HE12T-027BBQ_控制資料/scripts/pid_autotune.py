import serial
import time
import json

PORT = "COM10"
STEP_TARGET = 60
CAPTURE_SECONDS = 3.5
SETTLE_SECONDS = 0.6

KP_GRID = [0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
KI_GRID = [0.0, 0.005, 0.01, 0.015, 0.02, 0.03, 0.04]

ser = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2.0)
ser.reset_input_buffer()


def send(cmd):
    ser.write((cmd + "\n").encode())


def drain(seconds, collect=False):
    rows = []
    end = time.time() + seconds
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if collect and s.startswith("PID,"):
            parts = s.split(",")
            try:
                target = float(parts[1])
                actual = float(parts[2])
                duty = int(parts[3])
                rows.append((time.time(), target, actual, duty))
            except (ValueError, IndexError):
                pass
    return rows


def wait_until_stopped(max_seconds=3.0, threshold=2.0):
    # 馬達轉速快、慣量大，duty=0煞車不會瞬間停下，固定sleep容易在還沒
    # 真正停穩時就開始下一組測試，汙染下一次階躍測試的起始條件。
    # 改成持續讀 actual，直到連續幾筆都低於threshold才視為真的停了。
    send("t 0")
    end = time.time() + max_seconds
    quiet_count = 0
    while time.time() < end:
        line = ser.readline()
        if not line:
            continue
        s = line.decode(errors="ignore").strip()
        if s.startswith("PID,"):
            parts = s.split(",")
            try:
                actual = abs(float(parts[2]))
            except (ValueError, IndexError):
                continue
            if actual < threshold:
                quiet_count += 1
                if quiet_count >= 5:
                    return
            else:
                quiet_count = 0


def score_response(rows, target):
    if not rows:
        return 1e9, {}
    actuals = [r[2] for r in rows]
    peak = max(actuals)
    overshoot_pct = max(0.0, (peak - target) / target * 100.0) if target > 0 else 0.0

    tail_n = max(3, len(rows) // 4)  # 用最後1/4當穩態(比第一版窗口更晚，給積分項多點時間)
    tail = actuals[-tail_n:]
    tail_mean = sum(tail) / len(tail)
    steady_err_pct = abs(tail_mean - target) / target * 100.0 if target > 0 else 0.0
    tail_std = (sum((x - tail_mean) ** 2 for x in tail) / len(tail)) ** 0.5
    tail_std_pct = tail_std / target * 100.0 if target > 0 else 0.0

    t0 = rows[0][0]
    rise_time = None
    for r in rows:
        if r[2] >= target * 0.9:
            rise_time = r[0] - t0
            break
    rise_time_ms = (rise_time * 1000.0) if rise_time is not None else 3000.0

    score = (
        overshoot_pct * 2.0
        + steady_err_pct * 3.0
        + tail_std_pct * 2.5
        + rise_time_ms * 0.02
    )
    return score, {
        "overshoot_pct": round(overshoot_pct, 2),
        "steady_err_pct": round(steady_err_pct, 2),
        "tail_std_pct": round(tail_std_pct, 2),
        "rise_time_ms": round(rise_time_ms, 1),
    }


results = []
total = len(KP_GRID) * len(KI_GRID)
i = 0
for kp in KP_GRID:
    for ki in KI_GRID:
        i += 1
        wait_until_stopped()
        drain(0.2)
        send(f"kp {kp}")
        drain(0.05)
        send(f"ki {ki}")
        drain(0.05)
        send("reset")
        drain(0.1)

        send(f"t {STEP_TARGET}")
        rows = drain(CAPTURE_SECONDS, collect=True)

        wait_until_stopped()

        if not rows:
            continue
        target = rows[0][1]
        score, metrics = score_response(rows, target)
        results.append({"kp": kp, "ki": ki, "score": score, **metrics})
        print(f"[{i}/{total}] kp={kp:.3f} ki={ki:.3f}  score={score:7.2f}  {metrics}")

send("t 0")
ser.close()

results.sort(key=lambda r: r["score"])
print("\n=== TOP 8 ===")
for r in results[:8]:
    print(r)

with open("pid_autotune_results2.json", "w", encoding="utf-8") as f:
    json.dump(results, f, indent=2, ensure_ascii=False)
print("\nsaved pid_autotune_results2.json")
