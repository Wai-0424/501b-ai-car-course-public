#!/usr/bin/env python3
"""電腦端：對 car2_hwpwm_encoder_test 依序下指令，並記錄序列埠輸出。

輪子必須已離地。每次驅動韌體端最多 2 秒，且編碼器沒動會自動中止。
用法: python hwpwm_run.py --port COM10 --tag 20261010
輸出: ../results/hwpwm_raw_<tag>.log (原始、含時間戳) 與 hwpwm_summary_<tag>.json
"""
import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

END_MARK = "=== 結束"


def read_until(ser, marker, timeout, log):
    buf = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        log.write(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {text}\n")
        log.flush()
        buf.append(text)
        if marker and marker in text:
            return buf, True
    return buf, False


def drain(ser, secs, log):
    t0 = time.monotonic()
    out = []
    while time.monotonic() - t0 < secs:
        line = ser.readline()
        if line:
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            log.write(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {text}\n")
            out.append(text)
    log.flush()
    return out


def parse_run(lines):
    res = {"wheels": {}, "regs": [], "aborted": None}
    for ln in lines:
        m = re.search(r"(左輪|右輪): 前進計數 (-?\d+) ticks\s+約 (-?[\d.]+) rpm\s+-> (.*)$", ln)
        if m:
            res["wheels"][m.group(1)] = {
                "ticks": int(m.group(2)), "rpm": float(m.group(3)), "verdict": m.group(4).strip()}
        if "計時器=" in ln:
            res["regs"].append(ln.strip())
        if "提前結束" in ln:
            res["aborted"] = ln.split(":", 1)[1].strip()
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--tag", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--duty-plus", type=int, default=0, help="額外按幾次 + (預設 160 起)")
    args = ap.parse_args()

    out_dir = Path(__file__).resolve().parent.parent / "results"
    out_dir.mkdir(exist_ok=True)
    raw_path = out_dir / f"hwpwm_raw_{args.tag}.log"
    sum_path = out_dir / f"hwpwm_summary_{args.tag}.json"

    plan = [
        ("HW 5kHz", ["h", "5"], ["l", "r", "b"]),
        ("SW 5kHz (對照組)", ["s", "5"], ["l", "r", "b"]),
        ("HW 1kHz", ["h", "1"], ["l", "r", "b"]),
    ]
    names = {"l": "左輪(D9)", "r": "右輪(D8)", "b": "兩輪"}
    summary = {"started": datetime.now().isoformat(timespec="seconds"), "port": args.port, "runs": []}

    ser = serial.Serial(args.port, 115200, timeout=0.3)
    with open(raw_path, "w", encoding="utf-8") as log:
        log.write(f"# hwpwm_run.py tag={args.tag} port={args.port}\n")
        banner = drain(ser, 2.0, log)
        summary["banner"] = banner
        # 閒置 3 秒基線：輪子靜止，不應該有任何 [enc] 計數輸出
        idle = drain(ser, 3.0, log)
        summary["idle_enc_lines"] = [x for x in idle if x.startswith("[enc]")]

        for label, setup_cmds, runs in plan:
            for c in setup_cmds:
                ser.write(c.encode())
                time.sleep(0.15)
            drain(ser, 0.3, log)
            for c in runs:
                log.write(f"# --- {label} 指令 {c} ({names[c]}) ---\n")
                ser.write(c.encode())
                lines, ok = read_until(ser, END_MARK, 6.0, log)
                r = parse_run(lines)
                r.update({"label": label, "cmd": c, "target": names[c], "finished": ok})
                summary["runs"].append(r)
                tag = " ".join(f"{k}:{v['ticks']}t/{v['rpm']}rpm/{v['verdict']}" for k, v in r["wheels"].items())
                print(f"[{label:18s}] {names[c]:8s} -> {tag or '(沒有結果)'}"
                      + (f"  !!{r['aborted']}" if r["aborted"] else ""))
                time.sleep(1.5)  # 讓輪子停穩

        ser.write(b"?")
        summary["final_help"] = drain(ser, 1.0, log)
    ser.close()

    summary["finished"] = datetime.now().isoformat(timespec="seconds")
    sum_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"raw  -> {raw_path}\nsum  -> {sum_path}")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
