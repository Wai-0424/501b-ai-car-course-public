#!/usr/bin/env python3
"""分析 tune_drive.py 的 50 Hz 遙測：每個目標階梯的上升時間、過衝、整定時間、穩態誤差、漣波。

  python tune_analyze.py [-v] <tsv> [<tsv> ...]
轉速由每 20 ms 的原始編碼器增量換算(2.3276 rpm/tick)，再做 3 點移動平均(60 ms)。
定義(以該輪的階梯大小 D = 終值 - 起始值)：
  rise    : 從階梯開始到轉速走完 90% D 的時間
  over%   : 超過終值的最大量 / |D| (往下階梯則是低於終值的最大量)
  settle  : 最後一次離開 ±max(5% 終值, 3 rpm) 帶的時間(從階梯開始算)；到階段結束仍沒進帶 -> nan
  err%    : 終值(最後 0.8 s 平均) 相對期望值的誤差；期望 = (5+0.95*cmd)/100*116 ticks*2.3276
  ripple  : 最後 0.8 s 的標準差(rpm)
  iae     : 積分絕對誤差 / |D|(單位：秒)，越小越快
"""
import csv
import math
import sys

RPM = 60.0 / (1288.86 * 0.020)
MAXT = 116.0


def expected(p):
    return 0.0 if p <= 0 else (5 + 0.95 * p) / 100.0 * MAXT * RPM


def load(path):
    ph = {}
    for r in csv.DictReader(open(path), delimiter="\t"):
        ph.setdefault(int(r["phase"]), []).append({k: float(v) for k, v in r.items()})
    return ph


def smooth(vals, n=3):
    out = []
    for i in range(len(vals)):
        w = vals[max(0, i - n + 1): i + 1]
        out.append(sum(w) / len(w))
    return out


def analyze(path, verbose=False):
    ph = load(path)
    keys = sorted(k for k in ph if k >= 0)
    steps = []
    prev_final = [0.0, 0.0]
    prev_tgt = [0.0, 0.0]
    for k in keys:
        rs = ph[k]
        t0 = rs[0]["t"]
        t_end = rs[-1]["t"]
        tg = [rs[-1]["tgtL"], rs[-1]["tgtR"]]   # 取階段最後一筆：切換瞬間韌體回報的 tgt 還是舊值
        fin_now = [0.0, 0.0]
        for w, col, tcol in ((0, "dL", "tgtL"), (1, "dR", "tgtR")):
            t = [r["t"] - t0 for r in rs]
            v = smooth([r[col] * RPM for r in rs])
            tail = [x for tt, x in zip(t, v) if tt >= (t_end - t0) - 0.8]
            final = sum(tail) / len(tail) if tail else 0.0
            ripple = (sum((x - final) ** 2 for x in tail) / len(tail)) ** 0.5 if tail else 0.0
            fin_now[w] = final
            start = prev_final[w]
            D = final - start
            changed = abs(tg[w] - prev_tgt[w]) >= 5 or k == keys[0]
            if abs(D) < 12 or not changed:
                continue
            up = D > 0
            lvl = start + 0.9 * D
            rise = next((tt for tt, x in zip(t, v) if (x >= lvl if up else x <= lvl)), float("nan"))
            peak = max(v) if up else min(v)
            over = max(0.0, (peak - final) if up else (final - peak)) / abs(D) * 100
            tol = max(0.05 * abs(final), 3.0)
            outside = [i for i, x in enumerate(v) if abs(x - final) > tol]
            if not outside:
                settle = 0.0
            elif outside[-1] + 1 >= len(t):
                settle = float("nan")
            else:
                settle = t[outside[-1] + 1]
            iae = sum(abs(v[i] - final) * (t[i] - t[i - 1]) for i in range(1, len(v))) / abs(D)
            exp = expected(tg[w])
            err = (final - exp) / exp * 100 if exp > 0 and tg[w] > 0 else float("nan")
            steps.append({"phase": k, "wheel": "LR"[w], "from0": start < 3 and tg[w] > 0, "to0": tg[w] == 0,
                          "start": start, "final": final, "D": D, "rise": rise, "over": over,
                          "settle": settle, "iae": iae, "err": err, "ripple": ripple, "up": up, "tgt": tg[w]})
        prev_final = fin_now
        prev_tgt = tg
    if verbose:
        print(f"== {path}")
        print(" ph w  tgt  start->final   rise  over%  settle  iae    err%  ripple")
        for s in steps:
            print(f"{s['phase']:3d} {s['wheel']} {s['tgt']:4.0f} {s['start']:6.1f}->{s['final']:6.1f} "
                  f"{s['rise']:6.2f} {s['over']:6.1f} {s['settle']:6.2f} {s['iae']:5.2f} {s['err']:6.1f} {s['ripple']:6.2f}"
                  + ("  [from0]" if s["from0"] else "") + ("  [to0]" if s["to0"] else ""))
    return steps


def mean(xs):
    xs = [x for x in xs if not math.isnan(x)]
    return sum(xs) / len(xs) if xs else float("nan")


def summarize(steps):
    ups = [s for s in steps if s["up"] and not s["from0"] and not s["to0"]]
    dns = [s for s in steps if (not s["up"]) and not s["to0"]]
    st = [s for s in steps if s["from0"]]
    run = [s for s in steps if not s["to0"]]
    nan_settle = sum(1 for s in run if math.isnan(s["settle"]))
    return {
        "up_rise": mean([s["rise"] for s in ups]), "up_over": mean([s["over"] for s in ups]),
        "up_over_max": max([s["over"] for s in ups], default=float("nan")),
        "up_settle": mean([s["settle"] for s in ups]), "up_iae": mean([s["iae"] for s in ups]),
        "dn_settle": mean([s["settle"] for s in dns]), "dn_over": mean([s["over"] for s in dns]),
        "st_rise": mean([s["rise"] for s in st]), "st_over": mean([s["over"] for s in st]),
        "st_over_max": max([s["over"] for s in st], default=float("nan")),
        "err_abs": mean([abs(s["err"]) for s in run]), "ripple": mean([s["ripple"] for s in run]),
        "unsettled": nan_settle,
    }


COLS = ["up_rise", "up_over", "up_over_max", "up_settle", "up_iae", "dn_settle", "dn_over",
        "st_rise", "st_over", "st_over_max", "err_abs", "ripple", "unsettled"]


def main():
    verbose = "-v" in sys.argv
    files = [a for a in sys.argv[1:] if a != "-v"]
    print(f"{'file':44s} " + " ".join(f"{c:>11s}" for c in COLS))
    for f in files:
        s = summarize(analyze(f, verbose))
        print(f"{f.replace(chr(92), '/').split('/')[-1][:44]:44s} " + " ".join(f"{s[c]:11.2f}" for c in COLS))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
