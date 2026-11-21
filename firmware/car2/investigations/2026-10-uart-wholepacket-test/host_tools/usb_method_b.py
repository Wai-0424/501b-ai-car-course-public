#!/usr/bin/env python3
"""方法 B（電腦端）：讀 BMduino USB（rxcounters 版韌體）一段時間，檢查「收到的 CMD 數」是否與 ackseq 一致。

韌體每 200 ms 把 STA 與 dbg 兩行合成一則 USB 訊息，兩者是同一瞬間的快照：
  STA 的 ackseq = 最後一次通過 checksum 的 CMD 的 seq
  dbg 的 bmCmdOk = 到目前為止 BM 線(Serial3)上通過 checksum 的 CMD 總數
開啟序列埠會讓 BMduino 重開機（計數歸零、ackseq = -1），而 ROS bridge 的 seq 從 0 開始、每包 +1，
所以只要沒有任何一包遺失，**每一則訊息都滿足 bmCmdOk == ackseq + 1**。中途少收任何一包，這個差值就會永久變成負的。
同時檢查 bmCsFail / bmAbort / bmOverflow / FE 全為 0、I2C 錯誤為 0、PCA9685 就緒、無馬達故障旗標。

用法：python usb_method_b.py COM10 <秒數> <輸出log>
"""
import re
import sys
import time

import serial

port, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
N = ["dbg", "targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr",
     "stallEvM1", "stallMaxM1", "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid",
     "maxUs_status", "dirI2cCalls", "dirI2cErr", "pcaReady",
     "bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3"]

s = serial.Serial(port, 115200, timeout=0.3)
t0 = time.time()
buf = b""
last_sta = None            # (ackseq, f)
recs = []
bad_dbg = 0
with open(out, "w", encoding="utf-8") as f:
    while time.time() - t0 < secs:
        buf += s.read(512)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            text = ln.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            t = time.time() - t0
            f.write(f"{t:8.2f} {text}\n")
            if text.startswith("<STA"):
                m = re.search(r"ackseq (-?\d+)", text)
                fm = re.search(r",f (\d+)", text)
                last_sta = (int(m.group(1)) if m else None, int(fm.group(1)) if fm else 0)
            elif text.startswith("dbg,"):
                p = text.split(",")
                if len(p) != len(N) or last_sta is None or last_sta[0] is None:
                    bad_dbg += 1
                    continue
                try:
                    v = dict(zip(N[1:], [float(x) for x in p[1:]]))
                except ValueError:
                    bad_dbg += 1
                    continue
                v["t"], v["ack"], v["f"] = t, last_sta[0], last_sta[1]
                recs.append(v)
s.close()

print(f"[usb] {len(recs)} 則訊息（解析失敗 {bad_dbg}），共 {secs:.0f} 秒")
if not recs:
    print("[usb] 沒有資料：確認韌體是 rxcounters 版")
    sys.exit(1)

diffs = [(r["bmCmdOk"] - (r["ack"] + 1), r) for r in recs]
nonzero = [(d, r) for d, r in diffs if d != 0]
resets = sum(1 for a, b in zip(recs, recs[1:]) if b["bmCmdOk"] < a["bmCmdOk"])
last = recs[-1]
first_active = next((r for r in recs if r["ack"] >= 0), None)
print(f"[usb] 最後一則：ackseq {last['ack']}，bmCmdOk {last['bmCmdOk']:.0f}，bmBytes {last['bmBytes']:.0f}，rxdr {last['rxdrS3']:.0f}")
print(f"[usb] 判定 1  bmCmdOk == ackseq+1（每則訊息）：" + ("全部成立" if not nonzero else
      f"不成立 {len(nonzero)} 則，第一次在 t={nonzero[0][1]['t']:.1f}s 差 {nonzero[0][0]:.0f}，最後差 {nonzero[-1][0]:.0f}"))
print(f"[usb] 判定 2  錯誤計數（最後值）：csFail {last['bmCsFail']:.0f}  abort {last['bmAbort']:.0f}  "
      f"overflow {last['bmOverflow']:.0f}  FE {last['feS3']:.0f}")
print(f"[usb] 判定 3  bmBytes == rxdr：{'是' if last['bmBytes'] == last['rxdrS3'] else '否'}")
print(f"[usb] 判定 4  I2C 錯誤 steer {last['i2cSteerErr']:.0f} dir {last['dirI2cErr']:.0f}；pcaReady 全程 1：{all(r['pcaReady'] == 1 for r in recs)}；"
      f"馬達故障旗標 f 非 0 的訊息數：{sum(1 for r in recs if r['f'] != 0)}；BM 重開機次數：{resets}")
print(f"[usb] 馬達：目標 max L{max(r['targetL'] for r in recs):.0f}/R{max(r['targetR'] for r in recs):.0f}；實際 max |rpm| "
      f"L{max(abs(r['rpmL']) for r in recs):.0f}/R{max(abs(r['rpmR']) for r in recs):.0f}；"
      f"loop 區段最大耗時(µs，累計) poll {last['maxUs_poll']:.0f} pid {last['maxUs_pid']:.0f} status {last['maxUs_status']:.0f}")

print("[usb] 每分鐘：分 | ackseq | bmCmdOk | 差(應為0) | 錯誤合計 | max|rpm|")
mins = int(last["t"] // 60) + 1
for m in range(mins):
    seg = [r for r in recs if m * 60 <= r["t"] < (m + 1) * 60]
    if not seg:
        continue
    e = seg[-1]
    errs = e["bmCsFail"] + e["bmAbort"] + e["bmOverflow"] + e["feS3"] + e["i2cSteerErr"] + e["dirI2cErr"]
    print(f"[usb]   {m + 1:>2} | {e['ack']:>6} | {e['bmCmdOk']:>7.0f} | {e['bmCmdOk'] - (e['ack'] + 1):>4.0f} | {errs:>4.0f} | "
          f"{max(max(abs(r['rpmL']), abs(r['rpmR'])) for r in seg):>4.0f}")

ok = (not nonzero and last["bmCsFail"] == 0 and last["bmAbort"] == 0 and last["bmOverflow"] == 0 and last["feS3"] == 0
      and last["i2cSteerErr"] == 0 and last["dirI2cErr"] == 0 and resets == 0 and all(r["pcaReady"] == 1 for r in recs))
print("[usb] 總判定：" + ("PASS（沒有掉包、沒有任何錯誤）" if ok else "FAIL（見上面不成立的項目）"))
