#!/usr/bin/env python3
"""BTS7960 + 10 kHz 的 duty -> 轉速校正（對 car2_bts7960_pid 韌體的 d 指令 + t 1 串流）。輪子必須離地。

  python bts_calibrate.py <tag> [passes=2]

做兩件事，輸出到 results/：
  1. 起轉門檻：兩輪一起從低 duty 慢慢往上加（每步 0.4 s），各自記下第一次轉起來的 duty。 -> bts_up_<tag>.json
  2. 維持曲線：先慢慢爬升到 255 並停留(量最高轉速)，再由高往低一格一格掃，每格保持 HOLD_S 秒，取最後
     MEAS_S 秒的平均轉速；兩輪都停住就結束。重複 passes 次。      -> bts_sustain_<tag>_pass<N>.csv
安全：爬升用斜率（避免 BTS7960 無過流保護下的起動突波）；任何一輪轉速 > ABORT_RPM、韌體回報 fault、
序列埠中斷 -> 立刻送 s；結束一定送 s。duty 指令每 0.15 s 重送一次（韌體 600 ms 沒收到就自動停）。
"""
import csv
import json
import os
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

TICKS_PER_REV = 1288.86
OUT = Path(__file__).resolve().parent / "results"
OUT.mkdir(exist_ok=True)

HOLD_S = 1.3
MEAS_S = 0.6
ABORT_RPM = 380.0
GRID_DOWN = [240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100, 95, 90, 85, 80,
             76, 72, 68, 64, 60, 56, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 12]


class Abort(Exception):
    pass


class Rig:
    def __init__(self):
        port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
        if port is None:
            raise SystemExit("找不到 Holtek 04D9:802F 序列埠")
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, 115200, 0.02
        self.ser.dtr = False
        self.ser.rts = False
        self.ser.open()
        self.s = []          # (ms, posL, posR, dutyL, dutyR)
        self.flags = 0
        self.next_tx = 0.0
        time.sleep(1.2)
        self.ser.reset_input_buffer()
        # 環境變數 PRE：開頭先送的指令，以分號分隔，例如 PRE="f 20000"
        for _c in filter(None, os.environ.get("PRE", "").split(";")):
            self.cmd(_c.strip())
            time.sleep(0.05)
        self.cmd("s")
        self.cmd("t 1")
        self.pump(0.6)
        if not self.s:
            raise SystemExit("沒有收到 e 串流：刷了 car2_bts7960_pid 嗎？")

    def cmd(self, text):
        self.ser.write((text + "\n").encode())

    def pump(self, secs):
        t_end = time.time() + secs
        while True:
            line = self.ser.readline()
            if line:
                txt = line.decode("utf-8", "replace").strip()
                if txt.startswith("e,"):
                    p = txt.split(",")
                    try:
                        self.s.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5])))
                    except (ValueError, IndexError):
                        pass
                elif txt.startswith("dbg,"):
                    p = txt.split(",")
                    try:
                        self.flags = int(p[11])
                    except (ValueError, IndexError):
                        pass
            if time.time() >= t_end:
                return

    def drive(self, l, r, secs):
        """持續送 d l r 共 secs 秒；回傳這段時間內的串流樣本。"""
        n0 = len(self.s)
        t_end = time.time() + secs
        while time.time() < t_end:
            self.cmd(f"d {int(l)} {int(r)}")
            self.pump(0.15)
            self.check()
        return self.s[n0:]

    def rpm(self, samples):
        """用樣本首尾的位置差與韌體時間戳算平均 rpm。"""
        if len(samples) < 3:
            return 0.0, 0.0
        a, b = samples[0], samples[-1]
        dt = (b[0] - a[0]) / 1000.0
        if dt <= 0:
            return 0.0, 0.0
        return ((b[1] - a[1]) / TICKS_PER_REV / dt * 60.0, (b[2] - a[2]) / TICKS_PER_REV / dt * 60.0)

    def check(self):
        if self.flags & 3:
            raise Abort(f"韌體回報堵轉切斷 f={self.flags}")
        if len(self.s) >= 5:
            r = self.rpm(self.s[-5:])
            if max(abs(r[0]), abs(r[1])) > ABORT_RPM:
                raise Abort(f"轉速超過 {ABORT_RPM} rpm：{r}")

    def stop(self):
        for _ in range(3):
            try:
                self.cmd("s")
                time.sleep(0.05)
            except Exception:
                pass

    def close(self):
        self.stop()
        try:
            self.cmd("t 0")
        except Exception:
            pass
        self.ser.close()


def ramp(rig, to_duty, start=40, step=5, dt=0.06):
    d = start
    while d < to_duty:
        rig.drive(d, d, dt)
        d += step
    rig.drive(to_duty, to_duty, dt)


def do_up(rig, tag):
    print("== 起轉門檻（往上掃，每步 0.4 s）==")
    found = {"L": None, "R": None}
    d = 10
    while d <= 130 and (found["L"] is None or found["R"] is None):
        smp = rig.drive(d, d, 0.4)
        rl, rr = rig.rpm(smp[len(smp) // 2:])
        if found["L"] is None and abs(rl) > 4:
            found["L"] = d
        if found["R"] is None and abs(rr) > 4:
            found["R"] = d
        print(f"  duty {d:3d}: L {rl:6.1f}  R {rr:6.1f}")
        d += 2
    rig.stop()
    time.sleep(0.8)
    (OUT / f"bts_up_{tag}.json").write_text(json.dumps(found), encoding="utf-8")
    print("  起轉 duty：", found)
    if found["L"] is None or found["R"] is None:
        raise Abort("到 duty 130 還有一輪沒轉起來，停止（檢查那一輪的接線／機構）")
    return found


def do_sustain(rig, tag, n):
    print(f"== 維持曲線 pass {n} ==")
    ramp(rig, 255)
    smp = rig.drive(255, 255, 1.6)
    top = rig.rpm(smp[len(smp) // 3:])
    print(f"  255 (滿速): L {top[0]:6.1f}  R {top[1]:6.1f}")
    rows = [(255, top[0], top[1], 0, 0)]
    for d in GRID_DOWN:
        smp = rig.drive(d, d, HOLD_S)
        tail = [x for x in smp if x[0] >= smp[-1][0] - MEAS_S * 1000]
        rl, rr = rig.rpm(tail)
        stl, str_ = int(abs(rl) < 3), int(abs(rr) < 3)
        rows.append((d, rl, rr, stl, str_))
        print(f"  duty {d:3d}: L {rl:6.1f}{' (停)' if stl else ''}  R {rr:6.1f}{' (停)' if str_ else ''}")
        if stl and str_:
            break
    rig.stop()
    time.sleep(1.5)
    with open(OUT / f"bts_sustain_{tag}_pass{n}.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["duty", "rpmL", "rpmR", "stalledL", "stalledR"])
        for d, rl, rr, a, b in rows:
            w.writerow([d, f"{rl:.2f}", f"{rr:.2f}", a, b])


def main():
    tag = sys.argv[1]
    passes = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    rig = Rig()
    try:
        do_up(rig, tag)
        for n in range(1, passes + 1):
            do_sustain(rig, tag, n)
    except Abort as e:
        print("中止：", e)
    except KeyboardInterrupt:
        print("使用者中斷")
    finally:
        rig.close()
        print("已送停止。")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
