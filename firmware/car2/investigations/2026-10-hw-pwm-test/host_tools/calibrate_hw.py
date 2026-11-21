#!/usr/bin/env python3
"""硬體 PWM 的 duty -> 轉速校正（對 car2_hwpwm_openloop 韌體）。輪子必須離地。

  python calibrate_hw.py up      <tag>            起轉門檻（每輪單獨往上掃，動了就停）
  python calibrate_hw.py sustain <tag> [passes=3] 維持曲線（兩輪一起，kick 後由高往低掃）
  python calibrate_hw.py reverse <tag>            反轉對稱性（幾個 duty 的前進/後退轉速比）
  python calibrate_hw.py all     <tag>            依序 up -> sustain -> reverse

輸出到 ../results/：
  hwcal_up_<tag>.json、hwcal_sustain_<tag>_pass<N>.csv (duty,ticks20L,ticks20R,rpmL,rpmR)、hwcal_reverse_<tag>.json
單位：ticks20 = 每 20 ms 的編碼器計數（跟韌體 FF_LUT_TICKS_* 同單位），rpm 用 TICKS_PER_REV=1288.86 換算。
安全：轉速 >330 rpm、韌體回報 fault、序列埠中斷 -> 立刻停；結束一定送 X；堵轉的輪子（已下指令但
編碼器沒動）會被立刻關掉，不會把一個停住的馬達繼續推。
"""
import csv
import json
import statistics as st
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

TICKS_PER_REV = 1288.86
OUT = Path(__file__).resolve().parent.parent / "results"
OUT.mkdir(exist_ok=True)

# 跟 car2_car_control.ino 的 FF_LUT_DUTY 同一組格點（由高到低）
GRID_DOWN = [255, 240, 230, 220, 210, 200, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100,
             95, 90, 85, 80, 78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50]


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
        self.s = []            # (ms, fL, fR, dL, dR)
        self.events = []
        self.next_tx = 0.0
        self.cur = (0, 0)
        time.sleep(0.6)
        self.ser.reset_input_buffer()
        self.send(0, 0, force=True)
        self.pump(0.5)
        if not self.s:
            raise SystemExit("沒有收到 e 串流：燒了 car2_hwpwm_openloop 嗎？")

    def send(self, l, r, force=False):
        self.cur = (int(l), int(r))
        now = time.time()
        if force or now >= self.next_tx:
            self.ser.write(f"D {int(l)} {int(r)}\n".encode())
            self.next_tx = now + 0.1

    def pump(self, secs=0.0):
        t_end = time.time() + secs
        while True:
            line = self.ser.readline()
            if line:
                txt = line.decode("ascii", "replace").strip()
                if txt.startswith("e,"):
                    p = txt.split(",")
                    try:
                        self.s.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5])))
                    except (ValueError, IndexError):
                        pass
                elif txt.startswith("fault"):
                    self.events.append(txt)
            if time.time() >= t_end:
                return

    def stop(self):
        for _ in range(4):
            try:
                self.ser.write(b"X\n")
            except Exception:
                pass
            time.sleep(0.05)

    def rates(self, n_ms):
        """最近 n_ms 的 (rpmL, rpmR)；資料不足回傳 None。"""
        if len(self.s) < 3:
            return None
        end = self.s[-1]
        j = len(self.s) - 1
        while j > 0 and end[0] - self.s[j][0] < n_ms:
            j -= 1
        a = self.s[j]
        dt = end[0] - a[0]
        if dt < n_ms * 0.6:
            return None
        k = 1000.0 / dt / TICKS_PER_REV * 60.0
        return ((end[1] - a[1]) * k, (end[2] - a[2]) * k)

    def hold(self, l, r, secs, window_ms=700, stall_watch=True):
        """保持 (l, r) secs 秒；回傳 dict：窗口平均 ticks20/rpm、是否有輪子被切斷。
        stall_watch：輪子已下 duty 但 rpm < 6 連續 0.3 s（且已過 0.6 s）-> 該輪關掉。"""
        l_cmd, r_cmd = int(l), int(r)
        t0 = time.time()
        stalled = [False, False]
        low_since = [None, None]
        while time.time() - t0 < secs:
            self.send(0 if stalled[0] else l_cmd, 0 if stalled[1] else r_cmd)
            self.pump(0.02)
            if self.events:
                raise Abort("韌體回報 " + ";".join(self.events))
            rt = self.rates(250)
            if rt is None:
                continue
            if max(abs(rt[0]), abs(rt[1])) > 330:
                raise Abort(f"轉速過高 {rt}")
            el = time.time() - t0
            for w, cmd in ((0, l_cmd), (1, r_cmd)):
                if stall_watch and cmd != 0 and not stalled[w] and el > 0.6:
                    if abs(rt[w]) < 6:
                        low_since[w] = low_since[w] or time.time()
                        if time.time() - low_since[w] > 0.3:
                            stalled[w] = True
                    else:
                        low_since[w] = None
        # 窗口平均：最後 window_ms
        end = self.s[-1]
        j = len(self.s) - 1
        while j > 0 and end[0] - self.s[j][0] < window_ms:
            j -= 1
        a = self.s[j]
        dt = (end[0] - a[0]) / 1000.0
        res = {"dt": dt}
        for w, name in ((0, "L"), (1, "R")):
            d = (end[1 + w] - a[1 + w]) / dt if dt > 0 else 0.0
            res["ticks20" + name] = d * 0.02
            res["rpm" + name] = d / TICKS_PER_REV * 60.0
            res["stalled" + name] = stalled[w]
        return res

    def rest(self, secs):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.send(0, 0)
            self.pump(0.02)


def cmd_up(rig, tag):
    """每輪單獨往上掃，找「第一個讓輪子轉起來的 duty」。每步 0.5 s，轉起來就停。"""
    out = {}
    for w, name in ((0, "L"), (1, "R")):
        found = None
        for d in range(70, 141, 5):
            rig.rest(0.9)
            mark = len(rig.s)
            args = (d, 0) if w == 0 else (0, d)
            res = rig.hold(*args, 0.5, window_ms=300, stall_watch=False)
            moved = res["rpm" + name]
            if moved > 15:
                found = {"start_duty": d, "rpm_in_last_300ms": round(moved, 1)}
                break
        rig.rest(1.5)
        out[name] = found or {"start_duty": None}
        print(f"起轉門檻 {name}: {out[name]}")
    rig.stop()
    (OUT / f"hwcal_up_{tag}.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    return out


def cmd_sustain(rig, tag, passes=3):
    for ps in range(1, passes + 1):
        rig.rest(3.0)
        rows = []
        print(f"--- 維持曲線 pass {ps}/{passes}：先 kick 到 255 ---")
        rig.hold(255, 255, 1.2, stall_watch=False)
        alive = [True, True]
        for d in GRID_DOWN:
            l = d if alive[0] else 0
            r = d if alive[1] else 0
            if not (alive[0] or alive[1]):
                break
            res = rig.hold(l, r, 1.6, window_ms=700)
            for w, name in ((0, "L"), (1, "R")):
                if res["stalled" + name]:
                    alive[w] = False
            rows.append((d, res["ticks20L"] if alive[0] or res["rpmL"] > 6 else float("nan"),
                         res["ticks20R"] if alive[1] or res["rpmR"] > 6 else float("nan"),
                         res["rpmL"], res["rpmR"], res["stalledL"], res["stalledR"]))
            print(f"  duty {d:3d}: L {res['rpmL']:6.1f} rpm  R {res['rpmR']:6.1f} rpm"
                  + ("  [L 停住]" if res["stalledL"] else "") + ("  [R 停住]" if res["stalledR"] else ""))
        rig.stop()
        path = OUT / f"hwcal_sustain_{tag}_pass{ps}.csv"
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["duty", "ticks20L", "ticks20R", "rpmL", "rpmR", "stalledL", "stalledR"])
            for r in rows:
                w.writerow([r[0], f"{r[1]:.3f}", f"{r[2]:.3f}", f"{r[3]:.2f}", f"{r[4]:.2f}", int(r[5]), int(r[6])])
        print("  ->", path.name)


def cmd_reverse(rig, tag):
    out = {}
    for d in (100, 150, 200):
        rig.rest(2.0)
        rig.hold(255, 255, 1.0, stall_watch=False)
        fw = rig.hold(d, d, 1.6, window_ms=700)
        rig.rest(1.0)                           # 先停：韌體還會再強制停 300 ms 才反轉
        rig.hold(-255, -255, 1.2, stall_watch=False)
        bw = rig.hold(-d, -d, 1.6, window_ms=700)
        out[d] = {"fwd_rpm": [round(fw["rpmL"], 1), round(fw["rpmR"], 1)],
                  "bwd_rpm": [round(bw["rpmL"], 1), round(bw["rpmR"], 1)],
                  "bwd/fwd": [round(-bw["rpmL"] / fw["rpmL"], 3) if fw["rpmL"] else None,
                              round(-bw["rpmR"] / fw["rpmR"], 3) if fw["rpmR"] else None]}
        print(f"  duty ±{d}: {out[d]}")
    rig.stop()
    (OUT / f"hwcal_reverse_{tag}.json").write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    mode, tag = sys.argv[1], sys.argv[2]
    rig = Rig()
    try:
        if mode in ("up", "all"):
            cmd_up(rig, tag)
        if mode in ("sustain", "all"):
            cmd_sustain(rig, tag, int(sys.argv[3]) if len(sys.argv) > 3 and mode == "sustain" else 3)
        if mode in ("reverse", "all"):
            cmd_reverse(rig, tag)
    except Abort as e:
        print("!!! 中止:", e)
    finally:
        rig.stop()
        rig.ser.close()
        print("done，已送 X")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
