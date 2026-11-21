#!/usr/bin/env python3
"""從目前的正式版 car2_car_control.ino 產生「STA 回報頻率測試版」。不改任何控制/PID/PWM 邏輯。

  python make_sta20.py [--period 50] [--dbg-div 4] [--out <dir>] [--name car2_car_control_sta20]

補丁內容（每一項都在產生的 .ino 裡標 [TEST sta20]）：
  1. STATUS_PERIOD_MS 200 -> --period（預設 50 = STA 20 Hz）
  2. 傳送佇列重疊保護：sendStatus() 時上一則還沒送完就「略過這一次」並累計 staSkipBM / staSkipUSB，
     不再像正式版直接覆蓋緩衝（正式版 startSend() 沒有保護，會把送到一半的 STA 截斷）
  3. USB 只印 dbg 行（不附 STA 行），每 --dbg-div 次 STA 才印一次；dbg 尾端追加欄位：
       ackseq,bmCmdOk,bmCsFail,bmAbort,bmOverflow,bmBytes,feS3,rxdrS3,
       staCalls,staSentBM,staSkipBM,staSkipUSB,maxLoopGapUs,gap1ms,gap2ms,gap5ms
     （前 19 欄與正式版相同，順序不變）。ChunkedSender 緩衝 240 -> 255（len 是 uint8_t 的上限）。
  4. BM 線(Serial3) CMD 接收統計：合法 / checksum 失敗 / frame 被打斷 / frame 溢位 / 收到 byte 數
  5. loop() 相鄰兩次起點的最大間隔 maxLoopGapUs 與 >1/2/5 ms 的次數（「會不會堵塞」的直接指標）
  6. STA payload 追加 ,sn <第幾次呼叫 sendStatus>（Orin 端用序號缺口算遺失/損毀數；parse_sta 是通用 key-value，不受影響）
  7. 開機訊息加 [TEST sta20]，方便確認板上燒的是哪一版
"""
import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROD = HERE.parents[2] / "car2_car_control" / "car2_car_control.ino.v11_pre_sta20_20261010.bak"  # STA 20 Hz 搬進正式版之前的版本；補丁錨點對應它

GLOBALS = """
// ---- [TEST sta20] 新增的計數器 ----
uint32_t bmCmdOkCount = 0;          // BM 線(Serial3)上 checksum 通過的 CMD frame 數
uint32_t bmCsFailCount = 0;         // BM 線 checksum 不符被丟棄
uint32_t bmFrameAbortCount = 0;     // 收到 '<' 時上一個 frame 還沒收到 '>'（結尾遺失或被打斷）
uint32_t bmFrameOverflowCount = 0;  // frame 超過接收緩衝(80)
bool rxFromBM = false;              // handleFrame 當下的 frame 來自 BM 線還是 USB
uint32_t staCalls = 0;              // sendStatus() 被呼叫次數
uint32_t staSentBM = 0;             // 實際排入 BM 傳送佇列的 STA 數
uint32_t staSkipBM = 0;             // 上一則 STA 還沒送完而略過
uint32_t staSkipUSB = 0;            // USB dbg 上一則還沒送完而略過
uint32_t maxLoopGapUs = 0;          // loop() 相鄰兩次起點的最大間隔(µs)
uint32_t gapOver1ms = 0, gapOver2ms = 0, gapOver5ms = 0;
uint8_t dbgDivCount = 0;
const uint8_t DBG_DIV = %(dbg_div)d;  // USB dbg 每 DBG_DIV 次 STA 才印一次
"""

DBG_BLOCK = r'''  // [TEST sta20] USB 只印 dbg（不再附 STA 行），每 DBG_DIV 次 STA 才印一次，避免 20 Hz 時佇列來不及送完。
  if (++dbgDivCount >= DBG_DIV) {
    dbgDivCount = 0;
    char dbgLine[250];
    snprintf(dbgLine, sizeof(dbgLine),
        "dbg,%d,%d,%u,%u,%.1f,%.1f,%lu,%lu,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%d,"
        "%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        leftSpeed, rightSpeed, m1DriverDuty, m2DriverDuty, rpmM1, rpmM2,
        (unsigned long)i2cSteerCallCount, (unsigned long)i2cSteerErrorCount,
        pidM1State.stallEvents, pidM1State.maxStallMs,
        pidM2State.stallEvents, pidM2State.maxStallMs,
        (unsigned long)maxUs_pwmPump, (unsigned long)maxUs_poll,
        (unsigned long)maxUs_pid, (unsigned long)maxUs_status,
        (unsigned long)dirI2cCallCount, (unsigned long)dirI2cErrorCount, pcaReady ? 1 : 0,
        lastRxSeq,
        (unsigned long)bmCmdOkCount, (unsigned long)bmCsFailCount, (unsigned long)bmFrameAbortCount,
        (unsigned long)bmFrameOverflowCount, (unsigned long)linkBM.byteCount,
        (unsigned long)gDbgFeCount[SERIAL3_BUFFER_INDEX], (unsigned long)gDbgRxdrCount[SERIAL3_BUFFER_INDEX],
        (unsigned long)staCalls, (unsigned long)staSentBM, (unsigned long)staSkipBM, (unsigned long)staSkipUSB,
        (unsigned long)maxLoopGapUs, (unsigned long)gapOver1ms, (unsigned long)gapOver2ms, (unsigned long)gapOver5ms);
    if (usbSender.active) staSkipUSB++; else startSend(usbSender, dbgLine);
  }
'''

DBG_FIELDS = ["targetL", "targetR", "dutyL", "dutyR", "rpmL", "rpmR", "i2cSteerCalls", "i2cSteerErr",
              "stallEvM1", "stallMaxM1", "stallEvM2", "stallMaxM2", "maxUs_pwmPump", "maxUs_poll", "maxUs_pid",
              "maxUs_status", "dirI2cCalls", "dirI2cErr", "pcaReady",
              "ackseq", "bmCmdOk", "bmCsFail", "bmAbort", "bmOverflow", "bmBytes", "feS3", "rxdrS3",
              "staCalls", "staSentBM", "staSkipBM", "staSkipUSB", "maxLoopGapUs", "gap1ms", "gap2ms", "gap5ms"]


def sub1(t, pattern, repl, flags=0):
    new, n = re.subn(pattern, lambda m: repl(m) if callable(repl) else repl, t, count=1, flags=flags)
    if n != 1:
        raise SystemExit(f"補丁錨點找不到（正式版改過？）：{pattern[:70]}")
    return new


def make(period, dbg_div, name, consts=None):
    t = PROD.read_text(encoding="utf-8")
    # 診斷用：覆寫正式版的常數（例如還原舊 PID 來隔離雜訊來源），產生的檔案會標出來
    for cname, cval in (consts or {}).items():
        t = sub1(t, rf"^(const \w+ {cname} = )([^;]+)(;)",
                 lambda m, v=cval: m.group(1) + v + m.group(3) + "  // [TEST sta20 診斷覆寫]", re.M)
    assert len(DBG_FIELDS) == 35

    header = (f"// ============================================================================\n"
              f"// [TEST sta20] 由 make_sta20.py 從 car2_car_control.ino 自動產生（STA 週期 {period} ms = {1000 // period} Hz，\n"
              f"// USB dbg 每 {dbg_div} 次 STA 一次）。不改控制/PID/PWM 邏輯，只改回報頻率並加計數器。\n"
              f"// 這是測試用韌體，不是行駛用；測完請燒回 car2_car_control.ino。要改請改產生器，不要手改這個檔。\n"
              f"// ============================================================================\n")
    t = header + t

    t = sub1(t, r"^const uint32_t STATUS_PERIOD_MS = 200;", f"const uint32_t STATUS_PERIOD_MS = {period};  // [TEST sta20] 原本 200", re.M)
    t = sub1(t, r"^  char buf\[240\];", "  char buf[255];  // [TEST sta20] 240->255(len 是 uint8_t 的上限)", re.M)
    t = sub1(t, r"(^uint32_t cmdChecksumFailCount = 0;[^\n]*\n)", lambda m: m.group(1) + GLOBALS % {"dbg_div": dbg_div}, re.M)

    # handleFrame：BM 線 checksum 失敗 / 成功計數
    t = sub1(t, r"(  if \(!verifyAndStripChecksum\(frame\)\) \{\n    cmdChecksumFailCount\+\+;\n)",
             lambda m: m.group(1) + "    if (rxFromBM) bmCsFailCount++;  // [TEST sta20]\n")
    t = sub1(t, r"(  lastCmdMs = millis\(\);\n  updateSteering\(\);\n\})",
             lambda m: "  if (rxFromBM) bmCmdOkCount++;  // [TEST sta20]\n" + m.group(1))

    # pollLink：frame 被打斷 / 溢位 / 來源
    t = sub1(t, r"(void pollLink\(LinkState& ls\) \{\n)",
             lambda m: m.group(1) + "  bool isBM = (ls.port == (Stream*)&SerialBM);  // [TEST sta20]\n")
    t = sub1(t, r"(    if \(c == '<'\) \{\n)",
             lambda m: m.group(1) + "      if (isBM && ls.active && ls.len > 0) bmFrameAbortCount++;  // [TEST sta20]\n")
    t = sub1(t, r"(      ls\.buf\[ls\.len\] = '\\0';\n)(      handleFrame\(ls\.buf\);)",
             lambda m: m.group(1) + "      rxFromBM = isBM;  // [TEST sta20]\n" + m.group(2))
    t = sub1(t, r"(    if \(ls\.len < sizeof\(ls\.buf\) - 1\) \{\n      ls\.buf\[ls\.len\+\+\] = c;\n    \} else \{\n)",
             lambda m: m.group(1) + "      if (isBM) bmFrameOverflowCount++;  // [TEST sta20]\n")

    # STA payload 加 sn（= 第幾次呼叫 sendStatus）：Orin 端用序號缺口精確算出遺失/損毀的 STA 數
    t = sub1(t, r'"STA,ackseq %d,S1 -1,rpmL %\.1f,rpmR %\.1f,vL %d,vR %d,f %d",\n(\s+)lastRxSeq, rpmM1, rpmM2, vL, vR, faultFlags\);',
             lambda m: '"STA,ackseq %d,S1 -1,rpmL %.1f,rpmR %.1f,vL %d,vR %d,f %d,sn %lu",\n' + m.group(1) +
             'lastRxSeq, rpmM1, rpmM2, vL, vR, faultFlags, (unsigned long)staCalls);  // [TEST sta20] sn')

    # sendStatus：呼叫計數、BM 佇列重疊保護
    t = sub1(t, r"(void sendStatus\(\) \{\n)", lambda m: m.group(1) + "  staCalls++;  // [TEST sta20]\n")
    t = sub1(t, r"  startSend\(bmSender, line\);",
             "  if (bmSender.active) { staSkipBM++; } else { startSend(bmSender, line); staSentBM++; }  // [TEST sta20] 重疊保護")

    # USB dbg：整段換成降頻、只印 dbg 的版本
    t = sub1(t, r"  char dbgLine\[160\];.*?  startSend\(usbSender, usbCombined\);\n", DBG_BLOCK, re.S)

    # loop：相鄰兩次起點的最大間隔
    t = sub1(t, r"(void loop\(\) \{\n  uint32_t t0 = micros\(\);\n)",
             lambda m: m.group(1) +
             "  {  // [TEST sta20] loop() 相鄰兩次起點的間隔\n"
             "    static uint32_t prevLoopUs = 0;\n"
             "    if (prevLoopUs != 0) {\n"
             "      uint32_t g = t0 - prevLoopUs;\n"
             "      if (g > maxLoopGapUs) maxLoopGapUs = g;\n"
             "      if (g > 1000) gapOver1ms++;\n"
             "      if (g > 2000) gapOver2ms++;\n"
             "      if (g > 5000) gapOver5ms++;\n"
             "    }\n"
             "    prevLoopUs = t0;\n"
             "  }\n")

    t = sub1(t, r'Serial\.println\(F\("=== car2 主控韌體 ready',
             f'Serial.println(F("=== [TEST sta20 STA{1000 // period}Hz] car2 主控韌體 ready')
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--period", type=int, default=50)
    ap.add_argument("--dbg-div", type=int, default=4)
    ap.add_argument("--name", default="car2_car_control_sta20")
    ap.add_argument("--slew", help="診斷：MAX_DUTY_STEP_PER_CYCLE 的倍數(例如 6 = 舊值)")
    ap.add_argument("--kick-duty", help="診斷：KICK_DUTY（例如 115）")
    ap.add_argument("--kick-ms", help="診斷：KICK_DURATION_MS（例如 150）")
    ap.add_argument("--out", default=str(HERE.parent))
    args = ap.parse_args()
    consts = {}
    if args.slew: consts["MAX_DUTY_STEP_PER_CYCLE"] = f"{float(args.slew)}f * PID_INTERVAL_MS / 20.0f"
    if args.kick_duty: consts["KICK_DUTY"] = f"{float(args.kick_duty)}f"
    if args.kick_ms: consts["KICK_DURATION_MS"] = str(int(args.kick_ms))
    t = make(args.period, args.dbg_div, args.name, consts)
    d = Path(args.out) / args.name
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{args.name}.ino").write_text(t, encoding="utf-8")
    print(d / f"{args.name}.ino")
    print("dbg 欄位(標籤 dbg 之後)：" + ",".join(DBG_FIELDS))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
