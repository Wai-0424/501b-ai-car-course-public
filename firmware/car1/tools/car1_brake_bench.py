"""車一煞車台架測試（Orin 需求文件驗收 A）：車架高、輪子離地。

每個條件：先以 spin 指令（40% 或 100%）轉 3 s，再切到測試指令 3 s，最後 0% 1.5 s。
  A  0%            -> 滑行
  B  1%  brk 0     -> 只調降 duty（不煞車）
  C  1%  brk 1     -> 短路煞車，bmax=255
  D  1%  brk 1     -> 短路煞車，bmax=128（對照放寬前）
時間基準用 STA 的 upms（不受 USB 批次讀取影響）；轉速用 vL/vR（常駐速度，指令 0 也看得到）。
指標：切換後 vL、vR 都降到 < 60 rpm 的時間（t60）、< 100 rpm 的時間（t100）、f 旗標、煞車位元、最大 duty。
用法：python car1_brake_bench.py [--spin 40,100] [--reps 2] [--wheel both|left|right] [--label name]
"""
import argparse
import csv
import os
import time

from car1_bm import Car1

CASES = [
    ('A_0pct', 0, 0, 255),
    ('B_1pct_brk0', 1, 0, 255),
    ('C_1pct_brk1_bmax255', 1, 1, 255),
    ('D_1pct_brk1_bmax128', 1, 1, 128),
]
SPIN_S, TEST_S, STOP_S = 3.0, 3.0, 1.5


def run_phase(c, left, right, brk, seconds, rows, phase):
    t_end = time.monotonic() + seconds
    nxt = time.monotonic()
    while time.monotonic() < t_end:
        if time.monotonic() >= nxt:
            c.cmd(left, right, brk=brk)
            nxt += 0.05
        for s in c.poll(0.01):
            rows.append((phase, int(s['upms']), int(s['vL']), int(s['vR']), float(s['rpmL']), float(s['rpmR']),
                         int(s['dutyL']), int(s['dutyR']), int(s['f']), int(s.get('brk', -1))))


def first_below(rows, thr, t_step):
    for r in rows:
        if r[0] == 'test' and (r[1] - t_step) >= 0 and max(r[2], r[3]) < thr:
            return (r[1] - t_step) / 1000.0
    return None


def run_case(c, name, test_pct, brk, bmax, spin_pct, wheel, label, out):
    c.tune(bmax=bmax)
    time.sleep(0.2)
    c.run(0, 0, 1.5)
    rows = []
    sl = spin_pct if wheel in ('both', 'left') else 0
    sr = spin_pct if wheel in ('both', 'right') else 0
    tl = test_pct if wheel in ('both', 'left') else 0
    tr = test_pct if wheel in ('both', 'right') else 0
    run_phase(c, sl, sr, 0, SPIN_S, rows, 'spin')
    run_phase(c, tl, tr, brk, TEST_S, rows, 'test')
    run_phase(c, 0, 0, 0, STOP_S, rows, 'stop')
    test_rows = [r for r in rows if r[0] == 'test']
    t_step = test_rows[0][1] if test_rows else 0
    spin_rows = [r for r in rows if r[0] == 'spin']
    peak = max((max(r[2], r[3]) for r in spin_rows[-10:]), default=0)
    t60, t100 = first_below(rows, 60, t_step), first_below(rows, 100, t_step)
    fs = sorted(set(r[8] for r in rows))
    brks = sorted(set(r[9] for r in test_rows))
    dmax = max((max(r[6], r[7]) for r in test_rows), default=0)
    path = os.path.join(out, f'{label}_{name}_spin{spin_pct}_{wheel}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['phase', 'upms', 'vL', 'vR', 'rpmL', 'rpmR', 'dutyL', 'dutyR', 'f', 'brk'])
        w.writerows(rows)
    fmt = lambda v: f'{v:.2f}s' if v is not None else '  -  '
    print(f"spin{spin_pct:>3} {name:<22} start={peak:4d}rpm  t<100={fmt(t100):>7}  t<60={fmt(t60):>7}  maxDuty={dmax:3d}  f={fs}  brkBits={brks}")
    return t60, t100


def watchdog_test(c, out, label):
    """煞車中停止送命令：確認 500 ms 內歸零（duty 0、brk 位元清除、煞車功能旗標 bit4 歸 0）。"""
    c.tune(bmax=255)
    time.sleep(0.2)
    rows = []
    run_phase(c, 100, 100, 0, SPIN_S, rows, 'spin')
    run_phase(c, 1, 1, 1, 0.6, rows, 'test')       # 煞車已啟動
    t_cut = time.monotonic()
    while time.monotonic() - t_cut < 1.5:           # 不再送任何命令，只讀 STA
        for s in c.poll(0.02):
            rows.append(('cut', int(s['upms']), int(s['vL']), int(s['vR']), float(s['rpmL']), float(s['rpmR']),
                         int(s['dutyL']), int(s['dutyR']), int(s['f']), int(s.get('brk', -1))))
    cut = [r for r in rows if r[0] == 'cut']
    base = cut[0][1] if cut else 0
    off_t = next(((r[1] - base) / 1000.0 for r in cut if r[6] == 0 and r[7] == 0 and (r[9] & 0x13) == 0), None)
    print(f"watchdog: 停止送命令後 duty=0 且 brk 位元清除的時間 = {off_t if off_t is None else round(off_t, 2)} s（第一筆 cut 樣本 brk={cut[0][9] if cut else '?'}）, "
          f"cut 期間 f={sorted(set(r[8] for r in cut))}")
    c.run(0, 0, 1.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM25')
    ap.add_argument('--spin', default='40,100')
    ap.add_argument('--reps', type=int, default=2)
    ap.add_argument('--wheel', choices=['both', 'left', 'right'], default='both')
    ap.add_argument('--label', default='bench')
    ap.add_argument('--watchdog', action='store_true')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'pid_data', 'brake_bench_20261109'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    c = Car1(args.port)
    c.run(0, 0, 0.6)
    try:
        if args.watchdog:
            watchdog_test(c, args.out, args.label)
            return
        for spin in [int(x) for x in args.spin.split(',')]:
            for rep in range(args.reps):
                print(f'-- spin {spin}%  rep {rep + 1}  wheel={args.wheel}')
                for name, pct, brk, bmax in CASES:
                    run_case(c, name, pct, brk, bmax, spin, args.wheel, f'{args.label}_r{rep + 1}', args.out)
    finally:
        c.tune(bmax=255)  # TUNE 只改 RAM 但不會自己還原；D 條件把 bmax 設成 128，不還原 Orin 那邊就會一直看到 128
        time.sleep(0.2)
        c.close()
    print('bad_frames:', c.bad, 'dbg:', c.dbg)


if __name__ == '__main__':
    main()
