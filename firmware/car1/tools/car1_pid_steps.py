"""車一 PID 階躍調校：經本機 USB 送 TUNE 設定增益，跑固定的階梯計畫並計算過衝/整定/穩態誤差。

用法：python car1_pid_steps.py --kp 0.5 --ki 0.04 --lim 400 --slew 8 [--label name] [--wheel both|left|right]
需要車輛架高。階梯為前進方向 30→60→30→100→50→15→0（每階 2.0 s），三階以上才有意義，與車二調校計畫一致。
指標以 STA 的 rpmL/rpmR（PID 濾波後的輪速，EMA 0.3）為準：
  過衝% = (峰值-目標)/目標、欠衝% 為降速階躍的反向對應；整定 = 最後一次離開 ±5% 目標帶的時間；
  穩態誤差% = 最後 0.5 s 平均相對目標的誤差。目標 RPM = 55 + (pct-1)/99*(280-55)。
"""
import argparse
import csv
import os
import time

from car1_bm import Car1

PLAN = [30, 60, 30, 100, 50, 15, 0]
STEP_S = 2.0
MIN_RPM, MAX_RPM = 55.0, 280.0


def target_rpm(pct):
    return 0.0 if pct == 0 else MIN_RPM + (pct - 1) / 99.0 * (MAX_RPM - MIN_RPM)


def analyze(series, t_start, t_end, prev_target, target):
    """series: [(t, rpm)]，回傳這一階的 (overshoot%, settle_s, sse%)。"""
    seg = [(t - t_start, r) for t, r in series if t_start <= t < t_end]
    if not seg or target == 0:
        return None
    up = target >= prev_target
    peak = max(r for _, r in seg) if up else min(r for _, r in seg)
    over = ((peak - target) / target * 100.0) if up else ((target - peak) / target * 100.0)
    band = 0.05 * target
    settle = STEP_S
    for t, r in reversed(seg):
        if abs(r - target) > band:
            settle = t
            break
    else:
        settle = 0.0
    tail = [r for t, r in seg if t >= STEP_S - 0.5]
    sse = (sum(tail) / len(tail) - target) / target * 100.0 if tail else float('nan')
    return over, settle, sse


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM25')
    ap.add_argument('--kp', type=float)
    ap.add_argument('--ki', type=float)
    ap.add_argument('--lim', type=float)
    ap.add_argument('--slew', type=float)
    ap.add_argument('--label', default='run')
    ap.add_argument('--wheel', choices=['both', 'left', 'right'], default='both')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'pid_data', 'tune_20261117'))
    args = ap.parse_args()

    c = Car1(args.port)
    c.run(0, 0, 0.6)
    tune = {k: v for k, v in (('kp', args.kp), ('ki', args.ki), ('lim', args.lim), ('slew', args.slew)) if v is not None}
    if tune:
        c.tune(**tune)
        time.sleep(0.3)
    series = []  # (t, rpmL, rpmR, dutyL, dutyR, f)
    t0 = time.monotonic()
    marks = []
    for i, pct in enumerate(PLAN):
        left = pct if args.wheel in ('both', 'left') else 0
        right = pct if args.wheel in ('both', 'right') else 0
        marks.append((i * STEP_S, (i + 1) * STEP_S, pct))
        # run() 的時間軸從 0 起算，這裡統一換算成整段計畫的時間
        step_t0 = time.monotonic() - t0
        for t, s in c.run(left, right, STEP_S):
            series.append((step_t0 + t, float(s['rpmL']), float(s['rpmR']), int(s['dutyL']), int(s['dutyR']), int(s['f'])))
    c.run(0, 0, 1.0)
    dbg = c.dbg
    c.close()

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, f'{args.label}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'rpmL', 'rpmR', 'dutyL', 'dutyR', 'f'])
        w.writerows(series)

    print(f"== {args.label}: {tune} wheel={args.wheel}  samples={len(series)} bad_frames={c.bad} faults={sorted(set(x[5] for x in series))}")
    print(f"{'step':>10} | {'L over%':>8} {'L settle':>8} {'L sse%':>7} | {'R over%':>8} {'R settle':>8} {'R sse%':>7}")
    prev = 0
    for a, b, pct in marks:
        tgt = target_rpm(pct)
        res = []
        for col in (1, 2):
            ser = [(x[0], x[col]) for x in series]
            res.append(analyze(ser, a, b, target_rpm(prev), tgt))
        if pct != 0:
            fmt = lambda r: f"{r[0]:8.1f} {r[1]:8.2f} {r[2]:7.1f}" if r else f"{'-':>8} {'-':>8} {'-':>7}"
            print(f"{prev:>3}->{pct:<3}({tgt:5.1f}) | {fmt(res[0])} | {fmt(res[1])}")
        prev = pct
    print('dbg:', dbg)


if __name__ == '__main__':
    main()
