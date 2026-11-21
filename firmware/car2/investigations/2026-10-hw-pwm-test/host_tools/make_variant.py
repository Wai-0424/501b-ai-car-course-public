#!/usr/bin/env python3
"""從正式版 car2_car_control.ino 產生「調參用測試版」：可覆寫常數、加 50 Hz 遙測，不動正式版。

  python make_variant.py <out_dir> <name> [KEY=VALUE ...] [--no-telemetry]

KEY 可為：IBAND(積分分離帶 ticks) FFSHIFT(前饋人為偏移 duty，模擬負載) AW(=1 限速時暫停積分) KP KI SLEW KICK_DUTY KICK_MS MIN_RUN LOW_ON_DUTY
  SLEW 是「每 20 ms 最多變化幾個 duty」，例如 SLEW=12 -> 12.0f * PID_INTERVAL_MS / 20.0f
產生 <out_dir>/<name>/<name>.ino（arduino-cli 要求資料夾與主檔同名）。

遙測：每個 PID 週期(20 ms)用 usbSender 非阻塞排一行 USB 輸出
  T,<ms>,<tgtL>,<tgtR>,<dutyL>,<dutyR>,<dTicksL>,<dTicksR>,<f>
dTicks 是這個週期的原始編碼器增量(前進為正)。同時關掉每 200 ms 的 STA+dbg 合併 USB 輸出，避免兩者搶同一個佇列。
BM 線(給 Orin 的 STA)不受影響。
"""
import re
import sys
from pathlib import Path

PROD = Path(__file__).resolve().parents[3] / "car2_car_control" / "car2_car_control.ino"

CONSTS = {
    "KP": ("PID_KP", "{v}"),
    "KI": ("PID_KI", "{v}"),
    "SLEW": ("MAX_DUTY_STEP_PER_CYCLE", "{v}f * PID_INTERVAL_MS / 20.0f"),
    "KICK_DUTY": ("KICK_DUTY", "{v}f"),
    "KICK_MS": ("KICK_DURATION_MS", "{v}"),
    "MIN_RUN": ("MIN_RUN_DUTY", "{v}f"),
    "LOW_ON_DUTY": ("LOW_SPEED_ON_DUTY", "{v}f"),
}

TELEMETRY = """
    {  // [TUNE] 50 Hz 遙測
      static int32_t tPrev1 = 0, tPrev2 = 0;
      int32_t t1 = posM1, t2 = posM2;
      int tf = (pidM1State.fault ? 1 : 0) | (pidM2State.fault ? 2 : 0) | (pcaReady ? 0 : 4);
      char tl[64];
      snprintf(tl, sizeof(tl), "T,%lu,%d,%d,%u,%u,%ld,%ld,%d\\n", (unsigned long)now, leftSpeed, rightSpeed,
               (unsigned)m1DriverDuty, (unsigned)m2DriverDuty, (long)(t1 - tPrev1), (long)(-(t2 - tPrev2)), tf);
      tPrev1 = t1;
      tPrev2 = t2;
      if (!usbSender.active) startSend(usbSender, tl);
    }
"""


def build(out_dir, name, overrides, telemetry=True):
    t = PROD.read_text(encoding="utf-8")
    overrides = dict(overrides)
    aw = overrides.pop("AW", "0")
    if aw == "1":
        # 限速器(slew)正在限制輸出時暫停積分：避免大階梯期間積分一直累積、到達後過衝。
        anchor = "  bool willSaturate = (tentativeOutput <= MIN_RUN_DUTY) || (tentativeOutput >= 255.0f);"
        if t.count(anchor) != 1:
            raise SystemExit("找不到 willSaturate 那行")
        t = t.replace(anchor,
                      "  bool slewLimited = (tentativeOutput > prevDuty + MAX_DUTY_STEP_PER_CYCLE) ||\n"
                      "                     (tentativeOutput < prevDuty - MAX_DUTY_STEP_PER_CYCLE);  // [TUNE AW]\n"
                      "  bool willSaturate = (tentativeOutput <= MIN_RUN_DUTY) || (tentativeOutput >= 255.0f) || slewLimited;")
    iband = overrides.pop("IBAND", None)
    if iband is not None:
        # 積分分離：只有 |誤差| < IBAND(ticks/20ms) 才累積積分。前饋已負責大部分，大階梯期間誤差很大、
        # 馬達還有約 0.3 s 機械延遲，這段時間累積的積分會造成過衝。
        anchor = "    pid->integral = constrain(pid->integral + error, -integralLimit, integralLimit);"
        if t.count(anchor) != 1:
            raise SystemExit("找不到積分更新那行")
        t = t.replace(anchor, "    if (fabsf(error) < %sf) pid->integral = constrain(pid->integral + error, -integralLimit, integralLimit);  // [TUNE IBAND]" % repr(float(iband)))
    ffshift = overrides.pop("FFSHIFT", None)
    if ffshift is not None:
        # 人為讓前饋偏離(duty)，模擬上地面負載後曲線右移、前饋不準；用來檢驗參數對模型誤差的穩健性。
        anchor = "float naturalDuty = ffDutyForTicks(ffLutTicks, targetSpeed);"
        if t.count(anchor) != 1:
            raise SystemExit("找不到 naturalDuty 那行")
        t = t.replace(anchor, "float naturalDuty = max(0.0f, ffDutyForTicks(ffLutTicks, targetSpeed) + (%sf));  // [TUNE FFSHIFT]" % repr(float(ffshift)))
    for k, v in overrides.items():
        const, tmpl = CONSTS[k]
        if "{v}f" in tmpl:
            v = repr(float(v))          # 12 -> 12.0，才能接 f 後綴
        pat = rf"^(const \w+ {const} = )([^;]+)(;)"
        t, n = re.subn(pat, lambda m: m.group(1) + tmpl.format(v=v) + m.group(3), t, count=1, flags=re.M)
        if n != 1:
            raise SystemExit(f"找不到常數 {const}")
    if telemetry:
        anchor = "updateMotorPWM();  // 2026-10-10：把這個週期算出來的 duty 寫進硬體 PWM"
        if t.count(anchor) != 1:
            raise SystemExit("遙測插入點不存在（正式版 loop 改過？）")
        t = t.replace(anchor, anchor + TELEMETRY)
        if t.count("startSend(usbSender, usbCombined);") != 1:
            raise SystemExit("找不到 usbCombined 的 startSend")
        t = t.replace("startSend(usbSender, usbCombined);", "// [TUNE] usbCombined 關閉，改用 50 Hz T 行")
    d = Path(out_dir) / name
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{name}.ino").write_text(t, encoding="utf-8")
    return d


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out_dir, name = args[0], args[1]
    ov = dict(a.split("=", 1) for a in args[2:])
    p = build(out_dir, name, ov, telemetry="--no-telemetry" not in sys.argv)
    print(p)
