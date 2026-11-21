#!/usr/bin/env python3
"""把 calibrate_hw.py 的 sustain 掃描(多次 pass)整理成 car2_car_control.ino 用的 FF 查表。

  python build_lut.py <tag> [--ino path]   # 輸出 C++ 片段 + 與現行表的比較

做法：每個 duty 取各 pass 的平均 rpm（停住/不可靠的點丟掉）-> 換成 ticks/20ms -> 沿 duty 做
保序回歸(isotonic，轉速只增不減，平台區保留成「幾乎持平」)-> 取指定格點 -> 嚴格遞增(加最小增量)。
"""
import csv
import glob
import re
import sys
from pathlib import Path

TICKS_PER_REV = 1288.86
RPM_TO_T20 = TICKS_PER_REV * 0.020 / 60.0          # = 0.4296
HERE = Path(__file__).resolve().parent
RES = HERE.parent / "results"
INO_DEFAULT = HERE.parents[2] / "car2_car_control" / "car2_car_control.ino"

# 輸出格點：低端取兩輪都能維持的最低 duty(64)，210 在平台區裡與 200/220 重複所以不放。
GRID = [64, 66, 68, 70, 72, 74, 76, 78, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170,
        180, 190, 200, 220, 230, 240, 255]
MIN_VALID_RPM = 8.0
EPS_T20 = 0.03          # 嚴格遞增的最小增量(ticks/20ms)


def load(tag):
    passes = sorted(glob.glob(str(RES / f"hwcal_sustain_{tag}_pass*.csv")))
    if not passes:
        raise SystemExit("找不到 sustain 資料")
    per = {"L": {}, "R": {}}
    for p in passes:
        for r in csv.DictReader(open(p, encoding="utf-8")):
            d = int(r["duty"])
            for w, key, st in (("L", "rpmL", "stalledL"), ("R", "rpmR", "stalledR")):
                rpm = float(r[key])
                if rpm >= MIN_VALID_RPM and not int(r[st]):
                    per[w].setdefault(d, []).append(rpm)
    return per, len(passes)


def isotonic_nondecreasing(xs, ys):
    """PAV：x 遞增時 y 只增不減。"""
    blocks = [[y, 1] for y in ys]
    i = 0
    out = []
    for b in blocks:
        out.append(b)
        while len(out) > 1 and out[-2][0] > out[-1][0]:
            y2, n2 = out.pop()
            y1, n1 = out.pop()
            out.append([(y1 * n1 + y2 * n2) / (n1 + n2), n1 + n2])
    res = []
    for y, n in out:
        res.extend([y] * n)
    return res


def curve(per):
    ds = sorted(per)
    means = [sum(per[d]) / len(per[d]) for d in ds]
    iso = isotonic_nondecreasing(ds, means)
    return dict(zip(ds, iso)), dict(zip(ds, means)), {d: (max(per[d]) - min(per[d])) for d in ds}


def interp(table, d):
    ds = sorted(table)
    if d in table:
        return table[d]
    for a, b in zip(ds, ds[1:]):
        if a < d < b:
            return table[a] + (table[b] - table[a]) * (d - a) / (b - a)
    raise KeyError(d)


def parse_old(ino):
    t = Path(ino).read_text(encoding="utf-8")
    def arr(name):
        m = re.search(name + r"\[FF_LUT_N\]\s*=\s*\{([^}]*)\}", t)
        return [float(x.replace("f", "")) for x in m.group(1).split(",") if x.strip()]
    return arr("FF_LUT_DUTY"), arr("FF_LUT_TICKS_M1"), arr("FF_LUT_TICKS_M2")


def main():
    tag = sys.argv[1]
    ino = sys.argv[sys.argv.index("--ino") + 1] if "--ino" in sys.argv else INO_DEFAULT
    per, npass = load(tag)
    L_iso, L_mean, L_spread = curve(per["L"])
    R_iso, R_mean, R_spread = curve(per["R"])
    grid = [d for d in GRID if d in L_iso and d in R_iso]
    t20 = {}
    for w, iso in (("L", L_iso), ("R", R_iso)):
        vals = []
        prev = -1.0
        for d in grid:
            v = iso[d] * RPM_TO_T20
            v = max(v, prev + EPS_T20)
            vals.append(round(v, 2))
            prev = vals[-1]
        t20[w] = vals

    print(f"// {npass} 次 pass 平均；格點 {len(grid)} 個；單位 ticks/20ms (rpm = ticks x {1/RPM_TO_T20:.4f})")
    print(f"const int FF_LUT_N = {len(grid)};")
    fmt = lambda xs: ", ".join(f"{x:.1f}f" if float(x).is_integer() and False else f"{x}f" for x in xs)
    print("const float FF_LUT_DUTY[FF_LUT_N] = {" + ", ".join(f"{d}.0f" for d in grid) + "};")
    print("const float FF_LUT_TICKS_M1[FF_LUT_N] = {" + ", ".join(f"{v:.2f}f" for v in t20["L"]) + "};")
    print("const float FF_LUT_TICKS_M2[FF_LUT_N] = {" + ", ".join(f"{v:.2f}f" for v in t20["R"]) + "};")

    print("\n// --- pass 間最大差(rpm) 與保序回歸改動 ---")
    worst = max(max(L_spread.values()), max(R_spread.values()))
    print(f"pass-to-pass 最大差 {worst:.1f} rpm")
    for d in grid:
        dl, dr = L_iso[d] - L_mean[d], R_iso[d] - R_mean[d]
        if abs(dl) > 0.3 or abs(dr) > 0.3:
            print(f"  duty {d}: 保序回歸調整 L {dl:+.1f} / R {dr:+.1f} rpm")

    old_d, old_l, old_r = parse_old(ino)
    old_ld = dict(zip(old_d, [t / RPM_TO_T20 for t in old_l]))
    old_rd = dict(zip(old_d, [t / RPM_TO_T20 for t in old_r]))
    print("\n// --- 現行表(軟體 PWM 校正) vs 新表(硬體 PWM)，同 duty 的轉速 rpm ---")
    print("duty |  舊 L   新 L   差  |  舊 R   新 R   差")
    for d in [64, 70, 80, 90, 100, 120, 140, 160, 180, 200, 220, 240, 255]:
        if d not in L_iso:
            continue
        try:
            oL, oR = interp(old_ld, d), interp(old_rd, d)
        except KeyError:
            continue
        print(f"{d:4d} | {oL:6.1f} {L_iso[d]:6.1f} {L_iso[d]-oL:+5.1f} | {oR:6.1f} {R_iso[d]:6.1f} {R_iso[d]-oR:+5.1f}")
    print("\n// 100% 外部指令 = MAX_TICKS_PER_INTERVAL(116 ticks/20ms = 270 rpm) 所需 duty (新表, 線性內插)：")
    for w, vals in (("L", t20["L"]), ("R", t20["R"])):
        d_need = None
        for (d0, v0), (d1, v1) in zip(zip(grid, vals), zip(grid[1:], vals[1:])):
            if v0 <= 116.0 <= v1:
                d_need = d0 + (116.0 - v0) / (v1 - v0) * (d1 - d0)
        print(f"   {w}: duty ≈ {d_need:.0f}（滿 255，餘裕 {255 - d_need:.0f}）")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
