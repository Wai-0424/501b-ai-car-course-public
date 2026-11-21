#!/usr/bin/env python3
"""彙整 2026-10-08 UART 通訊測試數據 -> 數據彙整_20261008.md 與 數據彙整_20261008.xlsx。

資料來源：
  results/results_*.json          台上腳本（wp_test.py + pc_driver.py）每輪的結果
  results/usb_methodB_*.log       方法 B 的 BMduino USB 原始紀錄（重新計算，與執行當下的輸出交叉比對）
  results/orin_logs/*             Orin 上 ROS 測試的 log 與 link status
  本檔內以 TRANSCRIBED 標示的常數 = 執行當下終端機輸出，沒有存成檔案，手動轉錄
所有「判定 / 總和 / 比率」都由這支腳本從原始資料計算；xlsx 裡的衍生欄位另外寫成公式。
用法：python compile_results.py
"""
import json
import re
import statistics
from pathlib import Path

from openpyxl import Workbook
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

ROOT = Path(__file__).resolve().parent.parent
RES = ROOT / "results"
OUT_MD = ROOT / "數據彙整_20261008.md"
OUT_XLSX = ROOT / "數據彙整_20261008.xlsx"


# ----------------------------------------------------------------------------- 載入台上腳本結果
def load(name):
    return json.loads((RES / name).read_text(encoding="utf-8"))


ROUND_FILES = [
    ("1 台上・馬達不轉", ["results_20261008_130349.json"], "約 13:00 ~ 13:07"),
    ("2S 冒煙・馬達轉", ["results_smoke_20261008_131524.json"], "約 13:15"),
    ("2A 50 Hz 送法比較・馬達轉", ["results_A1_20261008_131547.json", "results_A2_20261008_132024.json"], "約 13:16 ~ 13:24"),
    ("2B 頻率天花板・馬達轉", ["results_B_20261008_132456.json"], "約 13:25 ~ 13:27"),
    ("2C CPU 負載・馬達轉", ["results_C_20261008_132808.json"], "約 13:28 ~ 13:33"),
]


def method_name(o):
    if o["mode"] == "whole":
        return "整包 write()" + (" + flush" if o.get("flush") else "")
    return f"分段 {o['chunk']} byte / {o['delay_ms']:g} ms"


TRIALS = []
for rnd, files, when in ROUND_FILES:
    for f in files:
        for t in load(f):
            o, d, e = t["orin"], t["deltas"], t["expected"]
            TRIALS.append({
                "round": rnd, "when": when, "label": t["label"], "method": method_name(o),
                "chunk": o["chunk"] if o["mode"] == "chunk" else None,
                "delay_ms": o["delay_ms"] if o["mode"] == "chunk" else None,
                "flush": bool(o.get("flush", False)),
                "rate": o["rate_hz"], "burst": o.get("burst", 1), "duration": o["duration_s"],
                "wheel": o.get("wheel_max", 0), "load": o.get("load_workers", 0), "loadavg_after": o.get("loadavg_after"),
                "sent": o["sent"], "bytes": o["bytes_sent"],
                "send_mean": o["send_ms_mean"], "send_p99": o["send_ms_p99"], "send_p999": o.get("send_ms_p999"),
                "send_max": o["send_ms_max"],
                "d_cmd": int(d["bmCmdOk"]), "e_cmd": e["cmds"], "d_bytes": int(d["bmBytes"]), "e_bytes": e["bytes"],
                "d_cs": int(d["bmCsFail"]), "d_abort": int(d["bmAbort"]), "d_ovf": int(d["bmOverflow"]), "d_fe": int(d["feS3"]),
                "d_i2c": int(d.get("i2cSteerErr", 0) + d.get("dirI2cErr", 0)),
                "rpm_max": t.get("rpm_max"), "poll_max": int(t["dbg_after"].get("maxUs_poll")),
                "sta_ok": o["sta_ok"], "sta_bad": o["sta_bad"],
                "age_mean": o.get("ack_age_ms_mean"), "age_max": o.get("ack_age_ms_max"),
                "verdict": "PASS" if t["pass_"] else "FAIL",
            })

# 完整性斷言：所有有效執行都必須收到的包數與 byte 數等於送出數，且各項錯誤計數為 0
for t in TRIALS:
    assert t["verdict"] == "PASS", t["label"]
    assert t["d_cmd"] == t["e_cmd"] and t["d_bytes"] == t["e_bytes"], t["label"]
    assert t["d_cs"] == t["d_abort"] == t["d_ovf"] == t["d_fe"] == t["d_i2c"] == 0, t["label"]


def by_round(prefix):
    return [t for t in TRIALS if t["round"].startswith(prefix)]


# ----------------------------------------------------------------------------- 方法 B：由 USB 原始紀錄重新計算
DBG_NAMES = ["dbg", "targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr",
             "stallEvM1", "stallMaxM1", "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid",
             "maxUs_status", "dirI2cCalls", "dirI2cErr", "pcaReady",
             "bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3"]


def analyze_usb(path):
    recs, last_sta, bad = [], None, 0
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*([\d.]+) (.*)$", line)
        if not m:
            continue
        t, text = float(m.group(1)), m.group(2).strip()
        if text.startswith("<STA"):
            a = re.search(r"ackseq (-?\d+)", text)
            f = re.search(r",f (\d+)", text)
            last_sta = (int(a.group(1)) if a else None, int(f.group(1)) if f else 0)
        elif text.startswith("dbg,"):
            p = text.split(",")
            if len(p) != len(DBG_NAMES) or last_sta is None or last_sta[0] is None:
                bad += 1
                continue
            try:
                v = dict(zip(DBG_NAMES[1:], [float(x) for x in p[1:]]))
            except ValueError:
                bad += 1
                continue
            v["t"], v["ack"], v["f"] = t, last_sta[0], last_sta[1]
            recs.append(v)
    last = recs[-1]
    nonzero = [r for r in recs if r["bmCmdOk"] - (r["ack"] + 1) != 0]
    resets = sum(1 for a, b in zip(recs, recs[1:]) if b["bmCmdOk"] < a["bmCmdOk"])
    per_min = []
    for m in range(int(last["t"] // 60) + 1):
        seg = [r for r in recs if m * 60 <= r["t"] < (m + 1) * 60]
        if not seg:
            continue
        e = seg[-1]
        per_min.append({
            "min": m + 1, "ack": e["ack"], "cmdOk": e["bmCmdOk"],
            "errs": e["bmCsFail"] + e["bmAbort"] + e["bmOverflow"] + e["feS3"] + e["i2cSteerErr"] + e["dirI2cErr"],
            "rpm": max(max(abs(r["rpmL"]), abs(r["rpmR"])) for r in seg),
        })
    return {
        "n_msgs": len(recs), "parse_fail": bad, "seconds": last["t"], "last_ack": int(last["ack"]),
        "cmdOk": int(last["bmCmdOk"]), "bytes": int(last["bmBytes"]), "rxdr": int(last["rxdrS3"]),
        "nonzero_diff": len(nonzero), "cs": int(last["bmCsFail"]), "abort": int(last["bmAbort"]),
        "ovf": int(last["bmOverflow"]), "fe": int(last["feS3"]),
        "i2c_steer": int(last["i2cSteerErr"]), "i2c_dir": int(last["dirI2cErr"]),
        "pca_always": all(r["pcaReady"] == 1 for r in recs), "f_nonzero": sum(1 for r in recs if r["f"] != 0),
        "resets": resets, "targetL": max(r["targetL"] for r in recs), "targetR": max(r["targetR"] for r in recs),
        "rpmL": max(abs(r["rpmL"]) for r in recs), "rpmR": max(abs(r["rpmR"]) for r in recs),
        "poll": int(last["maxUs_poll"]), "pid": int(last["maxUs_pid"]), "status": int(last["maxUs_status"]),
        "per_min": per_min,
    }


METHOD_B = [
    ("B1", "bridge 10 Hz（ROS 預設），無感知節點", 10, "usb_methodB_B1_10Hz.log", "ros_link_status_B1_10Hz.txt"),
    ("B2", "bridge 50 Hz，無感知節點", 50, "usb_methodB_B2_50Hz.log", "ros_link_status_B2_50Hz.txt"),
    ("B3", "bridge 10 Hz + 真正的感知節點（相機、depth_bev、除錯串流）", 10, "usb_methodB_B3_10Hz_perception.log",
     "ros_link_status_B3_10Hz_perception.txt"),
]
# TRANSCRIBED：方法 B 測試視窗結束當下（速度曲線結束後約 1.5 秒）的 Orin 端快照：cmd_seq_sent, last_ackseq, sta_ok, sta_bad
ORIN_END = {"B1": (3050, 3047, 1524, 1), "B2": (15250, 15238, 1515, 9), "B3": (3050, 3048, 1521, 5)}
MB = {}
for key, desc, hz, usb, st in METHOD_B:
    a = analyze_usb(RES / usb)
    s = dict(re.findall(r"^(\w+): (.*)$", (RES / "orin_logs" / st).read_text(encoding="utf-8"), re.M))
    a.update({"key": key, "desc": desc, "hz": hz, "orin_seq": ORIN_END[key][0], "orin_ack": ORIN_END[key][1],
              "sta_ok": ORIN_END[key][2], "sta_bad": ORIN_END[key][3],
              "file_seq": int(s["cmd_seq_sent"]), "file_ack": int(s["last_ackseq"]),
              "file_ok": int(s["sta_ok_count"]), "file_bad": int(s["sta_checksum_fail_count"])})
    MB[key] = a
    assert a["nonzero_diff"] == 0 and a["cs"] == a["abort"] == a["ovf"] == a["fe"] == 0
    assert a["i2c_steer"] == a["i2c_dir"] == 0 and a["pca_always"] and a["resets"] == 0 and a["f_nonzero"] == 0
    assert a["bytes"] == a["rxdr"] and a["cmdOk"] == a["last_ack"] + 1

# TRANSCRIBED：方法 B 執行當下 Orin 端每分鐘輸出的 link status（沒有存成檔案）。欄位：分鐘, cmd_seq_sent, last_ackseq, sta_ok, sta_bad, loadavg
ORIN_PER_MIN = {
    "B1": [("start", 15, 12, 7, 0, None), (1, 615, 612, 307, 0, 0.09), (2, 1215, 1212, 606, 1, 0.18), (3, 1815, 1812, 906, 1, 0.13),
           (4, 2415, 2411, 1206, 1, 0.05), (5, 3015, 3011, 1506, 1, 0.20), ("end", 3050, 3047, 1524, 1, None)],
    "B2": [("start", 75, 67, 7, 0, None), (1, 3075, 3067, 305, 2, 0.17), (2, 6075, 6067, 603, 4, 0.06), (3, 9075, 9068, 902, 5, 0.19),
           (4, 12075, 12068, 1200, 7, 0.07), (5, 15075, 15068, 1498, 9, 0.22), ("end", 15250, 15238, 1515, 9, None)],
    "B3": [("start", 15, 11, 7, 0, None), (1, 615, 611, 305, 2, 1.25), (2, 1215, 1212, 605, 2, 2.41), (3, 1815, 1812, 905, 2, 2.28),
           (4, 2415, 2412, 1204, 3, 2.17), (5, 3015, 3012, 1503, 5, 2.01), ("end", 3050, 3048, 1521, 5, None)],
}

# ----------------------------------------------------------------------------- ROS 實測（TRANSCRIBED 為終端輸出）
ROS_RUNS = [
    # (編號, 條件, 指標, 值, 來源)
    ("R1", "只啟動 uart_bridge_node（max_wheel_speed 40），/cmd_vel 30% + 轉向 -5°，10 Hz × 6 秒，輪子離地", "啟動訊息", "CMD送出方式=whole", "orin_logs/ros_bridge_20261008_134331.log"),
    ("R1", "同上", "送 cmd_vel 前（4 秒）", "cmd_seq_sent 25 / last_ackseq 23 / sta_ok 13 / 壞包 0；兩方向 UP", "終端輸出（TRANSCRIBED）"),
    ("R1", "同上", "結束時 link status", "cmd_seq_sent 125 / last_ackseq 123 / sta_ok 63 / 壞包 0；fault_gate OK", "orin_logs/ros_link_status_bridge.txt 與終端輸出"),
    ("R1", "同上（USB 讀 BMduino 原版韌體）", "馬達", "目標 L30 / R30；實際最大轉速 L96 / R95 rpm（絕對值）；轉動期間 17.0 ~ 21.6 s（24 筆 dbg）；停止後目標 0 / 轉速 0", "usb_ros_bridge_only_test.log（重新計算）"),
    ("R1", "同上", "STA ackseq", "首 1 → 末 147；dbg 211 行", "usb_ros_bridge_only_test.log（重新計算）"),
    ("R2", "-p uart_send_mode:=chunked -p uart_flush_each_chunk:=false，閒置 4 秒，輪速上限 0", "結果", "啟動正常，send_mode: chunked；兩方向 UP；sta 壞包 1", "orin_logs/ros_link_status_chunked.txt"),
    ("R2", "-p uart_send_mode:=bogus，閒置 4 秒，輪速上限 0", "結果", "印警告「uart_send_mode='bogus' 無效，改用 'chunked'」，send_mode: chunked；兩方向 UP；sta 壞包 0", "orin_logs/ros_bogus_134446.log、ros_link_status_bogus.txt"),
    ("R3", "完整 mode:=autonomous（感知 + 控制 + bridge，speed_pct 預設 0 = 輪子不動），約 55 秒", "/cmd_vel 頻率", "平均 19.675 Hz", "終端輸出（TRANSCRIBED）"),
    ("R3", "同上", "/car2/fwa_cmd 頻率（bridge 每個計時器週期發布一次）", "10.000 Hz（當時 bridge 預設 10 Hz）", "終端輸出（TRANSCRIBED）"),
    ("R3", "同上", "@ ~30 秒", "cmd_seq_sent 330 / last_ackseq 327 / sta_ok 165 / 壞包 0", "終端輸出（TRANSCRIBED）"),
    ("R3", "同上", "@ ~55 秒（最終）", "cmd_seq_sent 530 / last_ackseq 527 / sta_ok 265 / 壞包 0；兩方向 UP；fault_gate OK", "orin_logs/ros_link_status_autonomous_final.txt"),
    ("R3", "同上", "log 內 ERROR / WARN", "運行期間只有 2 則 WARN（控制節點 fixed_speed_pct=0 提示、相機 Color preset not supported）；結束階段有 uart_bridge_node 讀取執行緒 TypeError 與 depth_bev_node「publisher's context is invalid」（關閉時的競態，與送出方式無關）", "orin_logs/ros_autonomous_20261008_134515.log"),
    ("R4", "把預設改成 30 Hz 之後，以預設值（不帶頻率參數）啟動 bridge，輪速上限 0", "/car2/fwa_cmd 頻率", "30.001 Hz（標準差 0.24 ms，125 筆視窗）", "終端輸出（TRANSCRIBED）"),
    ("R4", "同上", "cmd_seq_sent", "75 → 510，14.27 秒增加 435 包 = 30.48 包/秒；last_ackseq 72 → 504", "終端輸出（TRANSCRIBED）"),
    ("R4", "同上", "最終 link status", "sta_ok 85 / 壞包 0；兩方向 UP；send_mode: whole", "orin_logs/ros_link_status_rate30.txt"),
]

# STA 壞包（BMduino → Orin）彙總。台上腳本由 TRIALS 加總；ROS 的由 link status 檔或轉錄
STA_GROUPS = []
for prefix, label, motors in [("1", "台上腳本・第一輪（馬達不轉）", "不轉"), ("2S", "台上腳本・冒煙（馬達轉）", "轉"),
                              ("2A", "台上腳本・階段 A（馬達轉，50 Hz）", "轉"), ("2B", "台上腳本・階段 B（馬達轉，100 ~ 600 Hz）", "轉"),
                              ("2C", "台上腳本・階段 C（馬達轉，CPU 負載）", "轉")]:
    rows = by_round(prefix)
    STA_GROUPS.append((label, motors, "—", sum(t["sta_ok"] for t in rows), sum(t["sta_bad"] for t in rows)))
for key in ("B1", "B2", "B3"):
    STA_GROUPS.append((f"ROS 方法 {key}（{MB[key]['hz']} Hz{'，含感知節點' if key == 'B3' else ''}）", "轉", f"{MB[key]['hz']}",
                       MB[key]["sta_ok"], MB[key]["sta_bad"]))
STA_GROUPS.append(("ROS 只啟動 bridge（R1，10 Hz）", "轉", "10", 63, 0))
STA_GROUPS.append(("ROS 完整 autonomous（R3，10 Hz，輪速鎖 0）", "不轉", "10", 265, 0))
STA_GROUPS.append(("ROS 預設 30 Hz 檢查（R4，輪速鎖 0）", "不轉", "30", 85, 0))

SUPERSEDED = [
    ("results_20261008_125606.json", "約 12:56", "冒煙 8 秒（整包 20 Hz，馬達不轉）", "FAIL（判定錯誤）",
     "兩個測試腳本錯誤：(1) 停止指令的 byte 數用 seq=0 估算（1 位數），實際 seq 是 3 位數，每包多 2 byte，5 包共差 10 byte；(2) 停止指令沒記進送出紀錄，STA 的 ackseq 超前被算成落後 65535。BM 端收到 166 / 166 個合法 CMD、各項錯誤計數為 0，通訊本身沒有問題。"),
    ("results_20261008_125659.json", "約 12:57", "冒煙（同上）", "FAIL（同樣原因）",
     "我用 PowerShell 的 scp 傳修正版腳本，但該環境找不到 scp，Orin 上仍是舊腳本，結果與上一次相同。"),
    ("results_20261008_125734.json", "約 12:57", "冒煙（同上，修正後）", "PASS", "修正腳本後重跑，166 / 166，byte 6,388 對 6,388，ackseq 最大落後 2 包。"),
    ("results_20261008_125759.json", "約 12:58 ~ 13:00", "第一次完整執行（基線、整包 20 / 50 Hz）", "整包 50 Hz 判 FAIL，後續輪次被中止",
     "BM 端 2,006 / 2,006 個 CMD、80,240 byte 完全吻合、各項錯誤 0。FAIL 只因我訂的判準「ackseq 落後 ≤ 2 包」的單位是包，頻率越高同樣的延遲換算成越多包（50 Hz 一包 20 ms，落後 4 包 = 80 ms；20 Hz 一包 50 ms，落後 2 包 = 100 ms），不是掉包指標。"),
    ("results_20261008_130155.json", "約 13:01 ~ 13:03", "第二次完整執行（判準改為 ack 年齡 ≤ 250 ms）", "基線與整包 20 Hz 都判 FAIL",
     "測試結束後 Orin 會閒置 0.6 秒等最後一包 STA，這段時間 BMduino 沒有新指令、ackseq 不再前進，STA 仍每 200 ms 報一次，年齡變成 200 / 400 / 600 ms，拉高了最大值（連分段基線也一樣，633 / 645 ms）。修正為只統計送指令期間收到的 STA。"),
]

# ----------------------------------------------------------------------------- 共用格式
def fms(x):
    if x is None:
        return ""
    return f"{x:.2f}" if x >= 1 else f"{x:.3f}"


def n(x):
    return f"{x:,}"


def md_table(headers, rows):
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def trial_rows(rows, with_poll=False):
    out = []
    for t in rows:
        r = [t["method"], f"{t['rate']:g}" + (f" × {t['burst']}" if t["burst"] > 1 else ""),
             f"{fms(t['send_mean'])} / {fms(t['send_p99'])} / {fms(t['send_max'])}",
             f"{n(t['d_cmd'])} / {n(t['e_cmd'])}", f"{n(t['d_bytes'])} / {n(t['e_bytes'])}",
             f"{t['sta_ok']} / {t['sta_bad']}", f"{t['age_mean']} / {t['age_max']}"]
        if with_poll:
            r.append(n(int(t["poll_max"])))
        out.append(r)
    return out


# ----------------------------------------------------------------------------- 計算彙總數字
tot_rounds = {k: sum(t["e_cmd"] for t in by_round(k)) for k in ("1", "2S", "2A", "2B", "2C")}
tot_motors_on = tot_rounds["2S"] + tot_rounds["2A"] + tot_rounds["2B"] + tot_rounds["2C"]
tot_methodB = sum(MB[k]["cmdOk"] for k in MB)
tot_all = tot_rounds["1"] + tot_motors_on + tot_methodB
assert tot_motors_on == 76605, tot_motors_on
bytes_all = sum(t["e_bytes"] for t in TRIALS) + sum(MB[k]["bytes"] for k in MB)
phaseA = by_round("2A")
base_a = next(t for t in phaseA if t["method"] == "分段 5 byte / 2 ms")
whole_a = next(t for t in phaseA if t["method"] == "整包 write()")
speedup = base_a["send_mean"] / whole_a["send_mean"]
sta_motor_off_1 = next(g for g in STA_GROUPS if g[0].startswith("台上腳本・第一輪"))
sta_on_script = [g for g in STA_GROUPS if g[0].startswith("台上腳本") and g[1] == "轉"]
sta_on_ok = sum(g[3] for g in sta_on_script)
sta_on_bad = sum(g[4] for g in sta_on_script)

# ----------------------------------------------------------------------------- 產生 Markdown
off_groups = [g for g in STA_GROUPS if g[1] == "不轉"]
on_groups = [g for g in STA_GROUPS if g[1] == "轉"]
off_bad = sum(g[4] for g in off_groups)
on_rates = [(g[4] / (g[3] + g[4]) * 100, g[0]) for g in on_groups]
on_min = min(r for r, _ in on_rates)
on_max, on_max_label = max(on_rates)
md = []
w = md.append
w("# 2026-10-08 UART 通訊測試數據彙整")
w("")
w("車2 Orin → BMduino 的 CMD 由「分段送出」改為「整包單一 `write()`」的比較實驗與 ROS 實測。本文所有數字由 `host_tools/compile_results.py` 從結果檔與原始紀錄計算，原始資料都在 `results/`。")
w("")
w("## 一、測試環境")
w("")
w(md_table(["項目", "內容"], [
    ["上位機", "Jetson Orin Nano Super Dev Kit（aarch64，6 核心），ROS 2 Jazzy，Python 3.12.3、pyserial 3.5"],
    ["Orin 電源模式", "`MAXN_SUPER`（nvpmodel ID 2；CPU 上限 1.728 GHz、GPU 上限 1.02 GHz）。今天所有測試都在這個模式下，`25W` 模式（設定檔上限 CPU 1.344 GHz、GPU 918 MHz）沒有測"],
    ["通訊", "Orin `/dev/ttyTHS1` → 74HC125 緩衝 → BMduino `Serial3`；460800 baud，8N1；封包為 ASCII 純文字加 XOR8 checksum"],
    ["下位機", "BMduino-UNO（BM53A367A，HT32F52367）。原版韌體 `car2_car_control.ino`（編譯 49,168 B）；測試版 `car2_car_control_rxcounters`（49,400 B，只多了 BM 線接收統計）"],
    ["燒錄 / 讀 USB", "電腦（Windows 11）以 `arduino-cli` 1.5.1 + e-Link32 Pro（CMSIS-DAP）燒錄，COM10（115200）讀 BMduino 的 USB 輸出"],
    ["車體", "輪子離地。馬達依 0 → 40% → 60% → 40% → 0 循環（每階 1.5 秒，只往前），實測最大約 180 ~ 200 rpm；前輪在 ±8° 之間每 0.5 秒擺動"],
    ["CPU 負載", "階段 C：Orin 上 4 個 `while True: pass` 行程（loadavg 升到約 4）。方法 B 的 B3：真正的感知節點（相機、depth_bev、除錯串流），loadavg 約 2.0 ~ 2.4"],
    ["ROS 專案", "`car2_bringup`，分支 `feature/uart-send-whole-20261008`（`0ec4a1c`、`b5b2e53`、`a509d64`），未 push"],
]))
w("")
w("## 二、結論摘要")
w("")
w(f"1. **整包單一 `write()` 零掉包。** 所有有效執行合計 **{n(tot_all)} 個 CMD、{n(bytes_all)} byte**，BMduino 端收到的合法 CMD 數與 byte 數都等於送出數，checksum 失敗、frame 被打斷、frame 溢位、framing error、I2C 錯誤全為 0（其中馬達轉動的台上測試 {n(tot_motors_on)} 個，ROS 方法 B {n(tot_methodB)} 個）。")
w(f"2. **單包送出耗時從約 {fms(base_a['send_mean'])} ms 降到約 {fms(whole_a['send_mean'])} ms**（50 Hz、馬達轉動下的平均值，約 1/{speedup:.0f}）。整包 + `flush()` 反而要約 {fms(next(t for t in phaseA if 'flush' in t['method'])['send_mean'])} ms，不要加。")
w("3. **穩定度在八種送法之間分不出高下**（都零錯誤），所以選擇依據是速度、尾端延遲與對 BMduino 的負擔。整包在 100 / 200 / 400 / 600 Hz 都零錯誤，找不到天花板；但頻率 ≥ 400 Hz 或多包連續送（突發）會讓 BMduino 的 `pollLink` 區段單次最長耗時從約 1.1 ~ 1.4 ms 拉到 3 ~ 5 ms（軟體 PWM 會凍結這麼久）。")
w(f"4. **ROS 實測（方法 B）：** bridge 10 Hz、50 Hz、10 Hz + 真正感知節點各 5 分鐘，共 {n(tot_methodB)} 個 CMD，每一則 USB 訊息都滿足 `bmCmdOk == ackseq + 1`（一包不少）。CMD 預設頻率改為 30 Hz 後，`ros2 topic hz` 量到 30.001 Hz。")
w(f"5. **觀察到的小問題（與 CMD 送法無關）：** BMduino → Orin 的 STA 壞包，馬達轉動時台上腳本共 {sta_on_bad} / {n(sta_on_ok + sta_on_bad)}（{sta_on_bad / (sta_on_ok + sta_on_bad) * 100:.2f}%），馬達不轉的第一輪為 {sta_motor_off_1[4]} / {n(sta_motor_off_1[3] + sta_motor_off_1[4])}。這個方向仍由韌體分段送出（未改），被 checksum 擋下。成因未驗證。")
w("6. **沒涵蓋：** 地面行駛（馬達帶負載）、輪速超過 60%、反轉、超過 5 分鐘的長時間運轉、`25W` 電源模式、30 Hz 下的方法 B。")
w("")
w("## 三、第一輪：台上、馬達不轉（約 13:00 ~ 13:07）")
w("")
w("腳本送 CMD，`bwl` / `bwr` 恆為 0，前輪 ±8° 擺動；USB 讀 BMduino 的接收計數（測試版韌體）。「收到 / 預期」中的預期含結束時 5 包停止指令。")
w("")
w(md_table(["送法", "頻率 (Hz)", "單包送出 平均 / p99 / 最大 (ms)", "Δ合法 CMD 收到 / 預期", "Δbytes 收到 / 預期", "STA 正常 / 壞包", "ack 年齡 平均 / 最大 (ms)", "BM pollLink 最長 (µs，該次執行累計)"],
           trial_rows(by_round("1"), with_poll=True)))
w("")
w("頻率欄的「× N」表示每個週期連續送 N 包（突發）。這一輪合計 CMD 與 byte 全部吻合，錯誤計數全 0。")
w("")
w("## 四、第二輪：馬達轉動（約 13:15 ~ 13:33）")
w("")
w("### 4.1 冒煙測試")
w("")
w(md_table(["送法", "頻率 (Hz)", "單包送出 平均 / p99 / 最大 (ms)", "Δ合法 CMD 收到 / 預期", "Δbytes 收到 / 預期", "STA 正常 / 壞包", "ack 年齡 平均 / 最大 (ms)"],
           trial_rows(by_round("2S"))))
w("")
w("輪速上限 40%，10 秒，馬達實際最大轉速 126 rpm，I2C 錯誤 0。")
w("")
w("### 4.2 階段 A：八種送法，50 Hz × 60 秒，馬達轉動，無 CPU 負載")
w("")
rows = []
for t in phaseA:
    rows.append([t["method"], fms(t["send_mean"]), fms(t["send_p99"]), fms(t["send_p999"]), fms(t["send_max"]),
                 f"{base_a['send_mean'] / t['send_mean']:.1f}×", f"{n(t['d_cmd'])} / {n(t['e_cmd'])}",
                 f"{t['sta_ok']} / {t['sta_bad']}", f"{t['age_mean']} / {t['age_max']}", f"{t['rpm_max']:.0f}"])
w(md_table(["送法", "單包 平均 (ms)", "p99", "p99.9", "最大", "相對基線", "Δ合法 CMD 收到 / 預期", "STA 正常 / 壞包", "ack 年齡 平均 / 最大 (ms)", "馬達最大 rpm"], rows))
w("")
w("「相對基線」＝ 分段 5 byte / 2 ms（ROS 專案原本的送法）的平均耗時 ÷ 該送法的平均耗時。八種送法 BMduino 端錯誤計數全 0，I2C 錯誤全 0。")
w("")
w("### 4.3 階段 B：頻率天花板（每輪 20 秒，馬達轉動，無 CPU 負載）")
w("")
w(md_table(["送法", "頻率 (Hz)", "單包送出 平均 / p99 / 最大 (ms)", "Δ合法 CMD 收到 / 預期", "Δbytes 收到 / 預期", "STA 正常 / 壞包", "ack 年齡 平均 / 最大 (ms)", "BM pollLink 最長 (µs，該次執行累計)"],
           trial_rows(by_round("2B"), with_poll=True)))
w("")
w("`pollLink` 最長耗時是這次執行（BMduino 重開機後）的歷史最大值：≤ 200 Hz 約 1.2 ms，400 Hz 為 1.6 ms，600 Hz 為 3.2 ms；其後三列是承接前面的數字，不代表那三輪造成。沒有任何一種送法在這個階段失敗。")
w("")
w("### 4.4 階段 C：Orin 4 個 CPU 忙碌行程 + 馬達轉動（每輪 60 秒）")
w("")
w(md_table(["送法", "頻率 (Hz)", "單包送出 平均 / p99 / 最大 (ms)", "Δ合法 CMD 收到 / 預期", "Δbytes 收到 / 預期", "STA 正常 / 壞包", "ack 年齡 平均 / 最大 (ms)", "BM pollLink 最長 (µs，該次執行累計)"],
           trial_rows(by_round("2C"), with_poll=True)))
w("")
w("CPU 負載下整包送出耗時不變；分段法尾端延遲變長（5 byte / 0.5 ms 的最大值 5 ms → 8 ms）。突發（每 0.1 秒連續 5 包）使 `pollLink` 最長耗時升到 4.8 ms。")
w("")
w("## 五、ROS 實測（輪子離地，BMduino 為原版韌體）")
w("")
w(md_table(["編號", "條件", "指標", "結果", "資料來源"], [list(r) for r in ROS_RUNS]))
w("")
w("## 六、方法 B：直接量 BMduino 收到幾包")
w("")
w("**判定法：** 韌體每 200 ms 把 STA 與 dbg 合成同一則 USB 訊息，兩者是同一瞬間的快照。bridge 的 `seq` 從 0 起、每包 +1，BMduino 每次重開機（開啟序列埠就會）計數歸零，所以沒有掉包時每一則訊息都必須滿足 **`bmCmdOk == ackseq + 1`**；中途少收任何一包，差值就會永久變負。")
w("")
rows = []
for k in ("B1", "B2", "B3"):
    a = MB[k]
    rows.append([k, a["desc"], n(a["n_msgs"]), n(a["cmdOk"]), n(a["last_ack"]), f"{a['nonzero_diff']}", f"{a['cs']} / {a['abort']} / {a['ovf']} / {a['fe']}",
                 f"{a['i2c_steer']} / {a['i2c_dir']}", f"L{a['rpmL']:.0f} / R{a['rpmR']:.0f}", f"{a['sta_bad']} / {n(a['sta_ok'] + a['sta_bad'])}"])
w(md_table(["階段", "條件", "USB 訊息數", "最終 bmCmdOk", "最終 ackseq", "等式不成立的訊息數", "csFail / abort / overflow / FE", "I2C 錯誤 steer / dir", "馬達最大 rpm", "STA 壞包 / 總數（Orin 端）"], rows))
w("")
w("三輪的 PCA9685 全程就緒、馬達故障旗標 `f` 全為 0、BMduino 沒有重開機、`bmBytes == rxdr`。")
w("")
for k in ("B1", "B2", "B3"):
    a = MB[k]
    w(f"### 6.{k[1]} {k}：{a['desc']}")
    w("")
    w("**BMduino 端（USB，每分鐘最後一則訊息）：**")
    w("")
    w(md_table(["分鐘", "ackseq", "bmCmdOk", "bmCmdOk − (ackseq + 1)", "錯誤合計", "馬達最大 rpm（絕對值）"],
               [[p["min"], n(int(p["ack"])), n(int(p["cmdOk"])), int(p["cmdOk"] - (p["ack"] + 1)), int(p["errs"]), f"{p['rpm']:.0f}"] for p in a["per_min"]]))
    w("")
    w("**Orin 端（執行當下終端輸出的 link status，TRANSCRIBED；「結束」列是統計用的測試視窗終點）：**")
    w("")
    w(md_table(["時間點", "cmd_seq_sent", "last_ackseq", "落後（包）", "sta_ok", "sta 壞包", "loadavg"],
               [[("開始" if p[0] == "start" else "結束（終端輸出）" if p[0] == "end" else f"第 {p[0]} 分鐘"), n(p[1]), n(p[2]), p[1] - p[2], n(p[3]), p[4], ("" if p[5] is None else p[5])] for p in ORIN_PER_MIN[k]]
               + [["status 檔（bridge 結束前最後一次寫入，比「結束」晚約 2 秒）", n(a["file_seq"]), n(a["file_ack"]), a["file_seq"] - a["file_ack"], n(a["file_ok"]), a["file_bad"], ""]]))
    w("")
w("## 七、STA 壞包彙總（BMduino → Orin 方向）")
w("")
rows = [[g[0], g[1], g[2], n(g[3]), g[4], f"{g[4] / (g[3] + g[4]) * 100:.2f}%"] for g in STA_GROUPS]
w(md_table(["條件", "馬達", "CMD 頻率 (Hz)", "STA 正常", "STA 壞包", "壞包率"], rows))
w("")
w(("壞包指 checksum 或格式不符的 STA；這個方向的 STA 由韌體每次 5 byte 分段送出（今天沒有改）。馬達不轉的條件（第一輪、R3、R4）合計 {off_bad} 個壞包，馬達轉動的條件為 {on_min:.2f}% ~ {on_max:.2f}%，最高的是 {on_max_label}。成因未驗證，也沒有排除「每次開啟序列埠時抓到半行 STA 被算成壞包」的可能（每輪最多約 1 個）。checksum 有擋下來，轉向控制不使用 STA，只用在里程與故障閘門。").format(off_bad=off_bad, on_min=on_min, on_max=on_max, on_max_label=on_max_label))
w("")
w("## 八、無效或被取代的執行（測試腳本判定邏輯的錯誤）")
w("")
w("以下執行的原始檔仍保留在 `results/`，但**不納入上面的任何統計**。")
w("")
w(md_table(["結果檔", "時間", "內容", "當時判定", "原因與處理"], [list(r) for r in SUPERSEDED]))
w("")
w("這些錯誤都出在測試腳本的判定邏輯，BMduino 端送收數量、byte 與各項錯誤計數從第一次執行起就一致。")
w("")
w("## 九、檔案與重現")
w("")
w("| 位置 | 內容 |")
w("|---|---|")
w("| `results/results_*.json`、`results/dbg_raw_*.log` | 台上腳本每輪的結果與 USB 原始 dbg 紀錄 |")
w("| `results/usb_methodB_*.log`、`results/usb_ros_bridge_only_test.log` | 方法 B 與 ROS 單獨 bridge 測試的 USB 原始紀錄 |")
w("| `results/orin_logs/` | Orin 上 ROS 測試的 log 與 link status |")
w("| `host_tools/` | 腳本：`wp_test.py`、`pc_driver.py`（台上測試）、`ros_method_b.sh`、`ros_cmd_vel_profile.py`、`usb_method_b.py`（方法 B）、`compile_results.py`（本彙整） |")
w("| `backup_before_flash/` | 實驗前的原版韌體備份（原始碼、hex、bin、SHA-256） |")
w("| `car2_car_control_rxcounters/` | 測試用韌體（與原版差 34 行） |")
w("| `數據彙整_20261008.xlsx` | 同一份數據的試算表（試驗總表、送法比較、方法 B、ROS 實測、STA 壞包、無效執行） |")
w("")
w("重跑本彙整：`python host_tools/compile_results.py`。方法 B 重跑步驟見 `README.md`。")
OUT_MD.write_text("\n".join(md) + "\n", encoding="utf-8")

# ----------------------------------------------------------------------------- 產生 Excel
FONT = "Arial"
f_norm = Font(name=FONT, size=10)
f_in = Font(name=FONT, size=10, color="0000FF")          # 實測 / 轉錄的輸入值：藍字
f_head = Font(name=FONT, size=10, bold=True, color="FFFFFF")
f_title = Font(name=FONT, size=12, bold=True)
fill_head = PatternFill("solid", fgColor="1F4D78")
fill_note = PatternFill("solid", fgColor="F2F4F7")
thin = Side(style="thin", color="BFBFBF")
border = Border(left=thin, right=thin, top=thin, bottom=thin)

wb = Workbook()


def style_header(ws, row, ncols):
    for c in range(1, ncols + 1):
        cell = ws.cell(row=row, column=c)
        cell.font, cell.fill, cell.border = f_head, fill_head, border
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
    ws.row_dimensions[row].height = 42


def put(ws, r, c, v, inp=True, fmt=None, wrap=False):
    cell = ws.cell(row=r, column=c, value=v)
    cell.font = f_in if inp and not (isinstance(v, str) and v.startswith("=")) else f_norm
    cell.border = border
    if fmt:
        cell.number_format = fmt
    cell.alignment = Alignment(vertical="top", wrap_text=wrap)
    return cell


# ---- 說明
ws = wb.active
ws.title = "說明"
notes = [
    ("2026-10-08 UART 通訊測試數據彙整", f_title),
    ("車2 Orin → BMduino 的 CMD：分段送出 vs 整包單一 write()。細節與結論見同資料夾的 數據彙整_20261008.md。", f_norm),
    ("", f_norm),
    ("欄位顏色：藍字 = 實測或轉錄的原始數值（輸入）；黑字 = 公式（衍生欄位）。", f_norm),
    ("公式：寫入時沒有預先計算（這台機器沒有 LibreOffice 可重算）。Excel 開啟時會自動重算；其他檢視器可能顯示空白。同樣的數字已在 md 報告中由 Python 另外計算。", f_norm),
    ("資料來源：試驗總表 = results/results_*.json；方法B 的 BMduino 端 = results/usb_methodB_*.log 重新計算；Orin 端每分鐘與 ROS實測 的部分欄位 = 執行當下終端輸出，手動轉錄（已在『資料來源』欄標示）。", f_norm),
    ("「Δ合法CMD」「Δbytes」= BMduino 端（測試版韌體）在該輪期間的接收計數；「預期」= Orin 端送出數 + 結束時 5 包停止指令。", f_norm),
    ("不含無效執行：無效 / 被取代的執行（測試腳本判定錯誤）只列在『無效執行』分頁，不納入任何統計。", f_norm),
    ("", f_norm),
    ("分頁：試驗總表（所有有效台上腳本輪次）、送法比較（階段 A）、方法B、ROS實測、STA壞包、無效執行。", f_norm),
]
for i, (txt, ft) in enumerate(notes, start=1):
    c = ws.cell(row=i, column=1, value=txt)
    c.font = ft
    c.alignment = Alignment(wrap_text=True, vertical="top")
ws.column_dimensions["A"].width = 120

# ---- 試驗總表
ws = wb.create_sheet("試驗總表")
COLS = ["輪次", "編號", "送法", "chunk (byte)", "段間隔 (ms)", "flush", "頻率 (Hz)", "每次連送包數", "時長 (s)", "輪速上限 (%)",
        "Orin CPU 負載行程", "Orin 送出包數", "送出 byte", "單包送出 平均 (ms)", "p99 (ms)", "p99.9 (ms)", "最大 (ms)",
        "BM Δ合法CMD", "預期 Δ合法CMD", "BM Δbytes", "預期 Δbytes", "ΔcsFail", "Δabort", "Δoverflow", "ΔFE", "ΔI2C錯誤",
        "馬達最大 (rpm)", "BM pollLink 最長 (µs，該次執行累計)", "STA 正常", "STA 壞包", "ack 年齡 平均 (ms)", "ack 年齡 最大 (ms)",
        "執行時判定", "收到−預期 (包)", "錯誤計數合計", "STA 壞包率", "名義送包率 (包/秒)"]
ci = {name: i + 1 for i, name in enumerate(COLS)}
L = {name: get_column_letter(i) for name, i in ci.items()}
for c, name in enumerate(COLS, start=1):
    ws.cell(row=1, column=c, value=name)
style_header(ws, 1, len(COLS))
for r, t in enumerate(TRIALS, start=2):
    vals = {
        "輪次": t["round"], "編號": t["label"], "送法": t["method"], "chunk (byte)": t["chunk"], "段間隔 (ms)": t["delay_ms"],
        "flush": "是" if t["flush"] else "否", "頻率 (Hz)": t["rate"], "每次連送包數": t["burst"], "時長 (s)": t["duration"],
        "輪速上限 (%)": t["wheel"], "Orin CPU 負載行程": t["load"], "Orin 送出包數": t["sent"], "送出 byte": t["bytes"],
        "單包送出 平均 (ms)": t["send_mean"], "p99 (ms)": t["send_p99"], "p99.9 (ms)": t["send_p999"], "最大 (ms)": t["send_max"],
        "BM Δ合法CMD": t["d_cmd"], "預期 Δ合法CMD": t["e_cmd"], "BM Δbytes": t["d_bytes"], "預期 Δbytes": t["e_bytes"],
        "ΔcsFail": t["d_cs"], "Δabort": t["d_abort"], "Δoverflow": t["d_ovf"], "ΔFE": t["d_fe"], "ΔI2C錯誤": t["d_i2c"],
        "馬達最大 (rpm)": t["rpm_max"], "BM pollLink 最長 (µs，該次執行累計)": t["poll_max"], "STA 正常": t["sta_ok"], "STA 壞包": t["sta_bad"],
        "ack 年齡 平均 (ms)": t["age_mean"], "ack 年齡 最大 (ms)": t["age_max"], "執行時判定": t["verdict"],
    }
    for name, v in vals.items():
        fmt = "#,##0" if name in ("Orin 送出包數", "送出 byte", "BM Δ合法CMD", "預期 Δ合法CMD", "BM Δbytes", "預期 Δbytes") else (
            "0.000" if name in ("單包送出 平均 (ms)", "p99 (ms)", "p99.9 (ms)", "最大 (ms)") else None)
        put(ws, r, ci[name], v, inp=True, fmt=fmt)
    put(ws, r, ci["收到−預期 (包)"], f"={L['BM Δ合法CMD']}{r}-{L['預期 Δ合法CMD']}{r}", inp=False, fmt="#,##0")
    put(ws, r, ci["錯誤計數合計"], f"=SUM({L['ΔcsFail']}{r}:{L['ΔI2C錯誤']}{r})", inp=False, fmt="0")
    put(ws, r, ci["STA 壞包率"], f"=IF(({L['STA 正常']}{r}+{L['STA 壞包']}{r})=0,0,{L['STA 壞包']}{r}/({L['STA 正常']}{r}+{L['STA 壞包']}{r}))", inp=False, fmt="0.00%")
    put(ws, r, ci["名義送包率 (包/秒)"], f"={L['頻率 (Hz)']}{r}*{L['每次連送包數']}{r}", inp=False, fmt="#,##0")
last_row = len(TRIALS) + 1
ws.freeze_panes = "D2"
for name in COLS:
    ws.column_dimensions[L[name]].width = 38 if name == "輪次" else (30 if name in ("編號", "送法") else 14)
r0 = last_row + 2
put(ws, r0, 1, "合計（公式）", inp=False)
ws.cell(row=r0, column=1).font = Font(name=FONT, size=10, bold=True)
for name in ("Orin 送出包數", "送出 byte", "BM Δ合法CMD", "預期 Δ合法CMD", "BM Δbytes", "預期 Δbytes", "ΔcsFail", "Δabort", "Δoverflow", "ΔFE", "ΔI2C錯誤", "STA 正常", "STA 壞包", "收到−預期 (包)", "錯誤計數合計"):
    put(ws, r0, ci[name], f"=SUM({L[name]}2:{L[name]}{last_row})", inp=False, fmt="#,##0")
put(ws, r0 + 1, 1, "註：以上為台上腳本的有效輪次；方法 B 的 21,520 個 CMD 見『方法B』分頁。", inp=False)

# ---- 送法比較（階段 A）
ws = wb.create_sheet("送法比較")
hdr = ["送法", "單包送出 平均 (ms)", "p99 (ms)", "p99.9 (ms)", "最大 (ms)", "相對基線的加速倍數", "BM 收到 / 預期", "STA 壞包", "ack 年齡 最大 (ms)", "馬達最大 (rpm)"]
for c, h in enumerate(hdr, start=1):
    ws.cell(row=1, column=c, value=h)
style_header(ws, 1, len(hdr))
base_idx = next(i for i, t in enumerate(phaseA) if t["method"] == "分段 5 byte / 2 ms") + 2
for r, t in enumerate(phaseA, start=2):
    put(ws, r, 1, t["method"])
    put(ws, r, 2, t["send_mean"], fmt="0.000")
    put(ws, r, 3, t["send_p99"], fmt="0.000")
    put(ws, r, 4, t["send_p999"], fmt="0.000")
    put(ws, r, 5, t["send_max"], fmt="0.000")
    put(ws, r, 6, f"=$B${base_idx}/B{r}", inp=False, fmt="0.0")
    put(ws, r, 7, f"{t['d_cmd']} / {t['e_cmd']}")
    put(ws, r, 8, t["sta_bad"])
    put(ws, r, 9, t["age_max"], fmt="0.0")
    put(ws, r, 10, t["rpm_max"], fmt="0")
put(ws, len(phaseA) + 3, 1, "50 Hz × 60 秒，馬達轉動（約 180 rpm），無 CPU 負載；相對基線 = 分段 5 byte / 2 ms 的平均耗時 ÷ 該送法的平均耗時（公式）。", inp=False)
ws.column_dimensions["A"].width = 26
for c in range(2, len(hdr) + 1):
    ws.column_dimensions[get_column_letter(c)].width = 16
ws.freeze_panes = "B2"

# ---- 方法B
ws = wb.create_sheet("方法B")
put(ws, 1, 1, "方法 B：每一則 USB 訊息都應滿足 bmCmdOk == ackseq + 1（沒有掉包）", inp=False)
ws.cell(row=1, column=1).font = f_title
hdr = ["階段", "條件", "Orin bridge 頻率 (Hz)", "USB 訊息數", "最終 bmCmdOk", "最終 ackseq", "最終 bmCmdOk − (ackseq+1)", "等式不成立的訊息數",
       "csFail", "abort", "overflow", "FE", "I2C steer", "I2C dir", "馬達最大 L (rpm)", "馬達最大 R (rpm)", "STA 正常（Orin）", "STA 壞包（Orin）", "STA 壞包率"]
for c, h in enumerate(hdr, start=1):
    ws.cell(row=3, column=c, value=h)
style_header(ws, 3, len(hdr))
for i, k in enumerate(("B1", "B2", "B3")):
    r = 4 + i
    a = MB[k]
    vals = [k, a["desc"], a["hz"], a["n_msgs"], a["cmdOk"], a["last_ack"], None, a["nonzero_diff"], a["cs"], a["abort"], a["ovf"], a["fe"],
            a["i2c_steer"], a["i2c_dir"], round(a["rpmL"]), round(a["rpmR"]), a["sta_ok"], a["sta_bad"]]
    for c, v in enumerate(vals, start=1):
        if c == 7:
            put(ws, r, c, f"=E{r}-(F{r}+1)", inp=False, fmt="0")
        else:
            put(ws, r, c, v, fmt="#,##0" if c in (4, 5, 6, 17) else None, wrap=(c == 2))
    put(ws, r, 19, f"=IF((Q{r}+R{r})=0,0,R{r}/(Q{r}+R{r}))", inp=False, fmt="0.00%")
put(ws, 7, 1, "合計", inp=False)
ws.cell(row=7, column=1).font = Font(name=FONT, size=10, bold=True)
put(ws, 7, 5, "=SUM(E4:E6)", inp=False, fmt="#,##0")
put(ws, 8, 1, "資料來源：BMduino 端欄位由 results/usb_methodB_*.log 重新計算；Orin 端 STA 欄位來自 results/orin_logs/ros_link_status_B*.txt。", inp=False)
row = 11
for k in ("B1", "B2", "B3"):
    a = MB[k]
    put(ws, row, 1, f"{k}：BMduino 端每分鐘（USB，該分鐘最後一則訊息）", inp=False)
    ws.cell(row=row, column=1).font = Font(name=FONT, size=10, bold=True)
    row += 1
    for c, h in enumerate(["分鐘", "ackseq", "bmCmdOk", "bmCmdOk − (ackseq+1)", "錯誤合計", "馬達最大 |rpm|"], start=1):
        ws.cell(row=row, column=c, value=h)
    style_header(ws, row, 6)
    row += 1
    for p in a["per_min"]:
        put(ws, row, 1, p["min"])
        put(ws, row, 2, int(p["ack"]), fmt="#,##0")
        put(ws, row, 3, int(p["cmdOk"]), fmt="#,##0")
        put(ws, row, 4, f"=C{row}-(B{row}+1)", inp=False, fmt="0")
        put(ws, row, 5, int(p["errs"]))
        put(ws, row, 6, round(p["rpm"]))
        row += 1
    row += 1
    put(ws, row, 1, f"{k}：Orin 端每分鐘 link status（終端輸出，手動轉錄）", inp=False)
    ws.cell(row=row, column=1).font = Font(name=FONT, size=10, bold=True)
    row += 1
    for c, h in enumerate(["時間點", "cmd_seq_sent", "last_ackseq", "落後（包）", "sta_ok", "sta 壞包", "loadavg", "與上一列的差（包）"], start=1):
        ws.cell(row=row, column=c, value=h)
    style_header(ws, row, 8)
    row += 1
    first = row
    for p in ORIN_PER_MIN[k]:
        put(ws, row, 1, "開始" if p[0] == "start" else "結束（終端輸出）" if p[0] == "end" else f"第 {p[0]} 分鐘")
        put(ws, row, 2, p[1], fmt="#,##0")
        put(ws, row, 3, p[2], fmt="#,##0")
        put(ws, row, 4, f"=B{row}-C{row}", inp=False, fmt="0")
        put(ws, row, 5, p[3], fmt="#,##0")
        put(ws, row, 6, p[4])
        put(ws, row, 7, p[5], fmt="0.00")
        if row > first:
            put(ws, row, 8, f"=B{row}-B{row - 1}", inp=False, fmt="#,##0")
        row += 1
    row += 2
ws.column_dimensions["A"].width = 14
ws.column_dimensions["B"].width = 40
for c in range(3, 20):
    ws.column_dimensions[get_column_letter(c)].width = 15

# ---- ROS實測
ws = wb.create_sheet("ROS實測")
hdr = ["編號", "條件", "指標", "結果", "資料來源"]
for c, h in enumerate(hdr, start=1):
    ws.cell(row=1, column=c, value=h)
style_header(ws, 1, len(hdr))
for r, row_ in enumerate(ROS_RUNS, start=2):
    for c, v in enumerate(row_, start=1):
        put(ws, r, c, v, wrap=True)
for col, wd in zip("ABCDE", (8, 55, 32, 70, 45)):
    ws.column_dimensions[col].width = wd
ws.freeze_panes = "A2"

# ---- STA壞包
ws = wb.create_sheet("STA壞包")
hdr = ["條件", "馬達", "CMD 頻率 (Hz)", "STA 正常", "STA 壞包", "壞包率", "總數"]
for c, h in enumerate(hdr, start=1):
    ws.cell(row=1, column=c, value=h)
style_header(ws, 1, len(hdr))
for r, g in enumerate(STA_GROUPS, start=2):
    put(ws, r, 1, g[0])
    put(ws, r, 2, g[1])
    put(ws, r, 3, g[2])
    put(ws, r, 4, g[3], fmt="#,##0")
    put(ws, r, 5, g[4], fmt="#,##0")
    put(ws, r, 6, f"=IF((D{r}+E{r})=0,0,E{r}/(D{r}+E{r}))", inp=False, fmt="0.00%")
    put(ws, r, 7, f"=D{r}+E{r}", inp=False, fmt="#,##0")
end = len(STA_GROUPS) + 1
put(ws, end + 2, 1, "馬達轉動（全部）", inp=False)
put(ws, end + 2, 4, f'=SUMIFS(D2:D{end},B2:B{end},"轉")', inp=False, fmt="#,##0")
put(ws, end + 2, 5, f'=SUMIFS(E2:E{end},B2:B{end},"轉")', inp=False, fmt="#,##0")
put(ws, end + 2, 6, f"=IF((D{end + 2}+E{end + 2})=0,0,E{end + 2}/(D{end + 2}+E{end + 2}))", inp=False, fmt="0.00%")
put(ws, end + 3, 1, "馬達不轉（全部）", inp=False)
put(ws, end + 3, 4, f'=SUMIFS(D2:D{end},B2:B{end},"不轉")', inp=False, fmt="#,##0")
put(ws, end + 3, 5, f'=SUMIFS(E2:E{end},B2:B{end},"不轉")', inp=False, fmt="#,##0")
put(ws, end + 3, 6, f"=IF((D{end + 3}+E{end + 3})=0,0,E{end + 3}/(D{end + 3}+E{end + 3}))", inp=False, fmt="0.00%")
put(ws, end + 5, 1, "壞包指 checksum 或格式不符的 STA（BMduino → Orin，韌體分段送出，今天未改）。成因未驗證，也未排除開啟序列埠時抓到半行 STA 被算成壞包。", inp=False)
ws.column_dimensions["A"].width = 46
for col in "BCDEFG":
    ws.column_dimensions[col].width = 14

# ---- 無效執行
ws = wb.create_sheet("無效執行")
hdr = ["結果檔", "時間", "內容", "當時判定", "原因與處理"]
for c, h in enumerate(hdr, start=1):
    ws.cell(row=1, column=c, value=h)
style_header(ws, 1, len(hdr))
for r, row_ in enumerate(SUPERSEDED, start=2):
    for c, v in enumerate(row_, start=1):
        put(ws, r, c, v, wrap=True)
for col, wd in zip("ABCDE", (34, 16, 40, 28, 110)):
    ws.column_dimensions[col].width = wd
put(ws, len(SUPERSEDED) + 3, 1, "這些執行不納入任何統計；錯誤都出在測試腳本的判定邏輯，BMduino 端的送收數量、byte 與錯誤計數一致。", inp=False)

wb.calculation.fullCalcOnLoad = True
wb.save(OUT_XLSX)

print(f"OK: {OUT_MD.name}, {OUT_XLSX.name}")
print(f"有效台上腳本輪次 {len(TRIALS)}；馬達轉動 CMD {tot_motors_on:,}；第一輪 {tot_rounds['1']:,}；方法 B {tot_methodB:,}；總計 {tot_all:,} 個 CMD、{bytes_all:,} byte")
for k in MB:
    print(k, "USB 訊息", MB[k]["n_msgs"], "解析失敗", MB[k]["parse_fail"], "最終 ackseq/cmdOk", MB[k]["last_ack"], MB[k]["cmdOk"], "bytes", MB[k]["bytes"])
