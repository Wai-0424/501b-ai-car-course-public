#!/usr/bin/env python3
"""把 bts_calibrate.py 的 sustain 掃描整理成 car2_bts7960_pid.ino 的前饋表（duty -> rpm），並改寫 .ino 裡
BEGIN_FF_LUT / END_FF_LUT 之間的區塊。

  python build_ff.py <tag> [--write]

每個 duty 取各 pass 的平均 rpm（停住的點丟掉）-> 沿 duty 做保序回歸（轉速只增不減）-> 嚴格遞增（加最小增量）。
不加 --write 只印結果。
"""
import csv
import glob
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
INO = HERE.parent / "car2_bts7960_pid.ino"
MIN_VALID_RPM = 3.0
EPS = 0.05


def load(tag):
    files = sorted(glob.glob(str(RES / f"bts_sustain_{tag}_pass*.csv")))
    if not files:
        raise SystemExit("找不到 sustain 資料")
    per = {"L": {}, "R": {}}
    for f in files:
        for r in csv.DictReader(open(f, encoding="utf-8")):
            d = int(r["duty"])
            for w, key, st in (("L", "rpmL", "stalledL"), ("R", "rpmR", "stalledR")):
                rpm = float(r[key])
                if rpm >= MIN_VALID_RPM and not int(r[st]):
                    per[w].setdefault(d, []).append(rpm)
    return per, len(files)


def pav(ys):
    out = []
    for y in ys:
        out.append([y, 1])
        while len(out) > 1 and out[-2][0] > out[-1][0]:
            y2, n2 = out.pop()
            y1, n1 = out.pop()
            out.append([(y1 * n1 + y2 * n2) / (n1 + n2), n1 + n2])
    res = []
    for y, n in out:
        res.extend([y] * n)
    return res


def main():
    tag = sys.argv[1]
    per, npass = load(tag)
    duties = sorted(set(per["L"]) & set(per["R"]))
    table = {}
    for w in ("L", "R"):
        means = [sum(per[w][d]) / len(per[w][d]) for d in duties]
        iso = pav(means)
        vals, prev = [], -1.0
        for v in iso:
            v = max(v, prev + EPS)
            vals.append(round(v, 2))
            prev = vals[-1]
        table[w] = vals
    n = len(duties)
    block = [
        "// BEGIN_FF_LUT（build_ff.py 會改寫這個區塊）",
        "const bool FF_CALIBRATED = true;   // " + f"{tag}：{npass} 次 pass 平均，BTS7960、" + ("20 kHz" if "20k" in tag else "10 kHz") + "、輪子離地",
        f"const int FF_LUT_N = {n};",
        "const float FF_LUT_DUTY[FF_LUT_N] = {" + ", ".join(f"{d}.0f" for d in duties) + "};",
        "const float FF_LUT_RPM_M1[FF_LUT_N] = {" + ", ".join(f"{v}f" for v in table["L"]) + "};",
        "const float FF_LUT_RPM_M2[FF_LUT_N] = {" + ", ".join(f"{v}f" for v in table["R"]) + "};",
        "// END_FF_LUT",
    ]
    print("\n".join(block))
    if "--write" in sys.argv:
        t = INO.read_text(encoding="utf-8")
        new, k = re.subn(r"// BEGIN_FF_LUT.*?// END_FF_LUT", "\n".join(block).replace("\\", "\\\\"), t, flags=re.S)
        if k != 1:
            raise SystemExit("找不到 BEGIN_FF_LUT/END_FF_LUT 標記")
        INO.write_text(new, encoding="utf-8")
        print("\n已寫入", INO)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
