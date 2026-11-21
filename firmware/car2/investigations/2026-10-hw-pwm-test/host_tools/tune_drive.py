#!/usr/bin/env python3
"""對「調參用測試版」(make_variant.py 產生，會輸出 T 行)跑一個目標階梯計畫，記錄 50 Hz 遙測。輪子必須離地。

  python tune_drive.py "<l>,<r>,<秒>;<l>,<r>,<秒>;..." <out.tsv>

每 50 ms 送一次 CMD（韌體看門狗 500 ms）。守門：f != 0、估計轉速 > 340 rpm、已下 >=15% 但 >1.2 s
仍 < 5 rpm、超過 1 s 沒有 T 行 -> 中止；結束一定送 2 s 的 0。
輸出欄位：t(電腦秒) phase tgtL tgtR dutyL dutyR dL dR f ms
"""
import sys
import time

import serial
from serial.tools import list_ports

TICKS_PER_REV = 1288.86
RPM_PER_TICK = 60.0 / (TICKS_PER_REV * 0.020)


def main():
    plan = [tuple(float(x) for x in seg.split(",")) for seg in sys.argv[1].split(";")]
    out_path = sys.argv[2]
    port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
    if port is None:
        sys.exit("no Holtek 04D9:802F COM port")
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0.005
    ser.dtr = False
    ser.rts = False
    ser.open()

    seq = 1000
    rows = []
    last_t = {"t": None, "vL": 0.0, "vR": 0.0, "f": 0}
    abort = None

    def cmd(l, r):
        nonlocal seq
        p = f"CMD,seq {seq},fwa 0,bwl {int(l)},bwr {int(r)}"
        seq = (seq + 1) % 65536
        cs = 0
        for ch in p.encode():
            cs ^= ch
        return f"<{p},cs {cs:02X}>\n".encode()

    def pump(t0, phase):
        while True:
            line = ser.readline()
            if not line:
                return
            s = line.decode("ascii", "replace").strip()
            if not s.startswith("T,"):
                continue
            p = s.split(",")
            if len(p) != 9:
                continue
            try:
                ms, tl, tr, dl, dr, a, b, f = (int(x) for x in p[1:])
            except ValueError:
                continue
            now = time.time()
            rows.append((now - t0, phase, tl, tr, dl, dr, a, b, f, ms))
            last_t["t"] = now
            last_t["vL"] = 0.7 * last_t["vL"] + 0.3 * a * RPM_PER_TICK
            last_t["vR"] = 0.7 * last_t["vR"] + 0.3 * b * RPM_PER_TICK
            last_t["f"] = f

    try:
        time.sleep(0.6)
        ser.reset_input_buffer()
        t0 = time.time()
        for ph, (tl, tr, dur) in enumerate(plan):
            t_ph = time.time()
            t_end = t_ph + dur
            nxt = time.time()
            while time.time() < t_end and abort is None:
                now = time.time()
                if now >= nxt:
                    ser.write(cmd(tl, tr))
                    nxt = now + 0.05
                pump(t0, ph)
                if last_t["f"] != 0:
                    abort = f"fault flag f={last_t['f']} in phase {ph}"
                elif max(abs(last_t["vL"]), abs(last_t["vR"])) > 340:
                    abort = f"speed > 340 rpm in phase {ph}"
                elif last_t["t"] is not None and now - last_t["t"] > 1.0:
                    abort = "no T lines for > 1 s"
                elif now - t_ph > 1.2 and ((abs(tl) >= 15 and abs(last_t["vL"]) < 5) or (abs(tr) >= 15 and abs(last_t["vR"]) < 5)):
                    abort = f"commanded wheel not turning in phase {ph} (vL={last_t['vL']:.0f}, vR={last_t['vR']:.0f})"
            if abort:
                break
    finally:
        t_end = time.time() + 2.0
        nxt = time.time()
        while time.time() < t_end:
            if time.time() >= nxt:
                ser.write(cmd(0, 0))
                nxt = time.time() + 0.05
            pump(t0, -1)
        ser.close()

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("t\tphase\ttgtL\ttgtR\tdutyL\tdutyR\tdL\tdR\tf\tms\n")
        for r in rows:
            f.write("\t".join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in r) + "\n")
    print(f"abort: {abort or 'none'} | rows: {len(rows)}")


if __name__ == "__main__":
    main()
