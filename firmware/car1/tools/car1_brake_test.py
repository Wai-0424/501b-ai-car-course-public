"""車一超速煞車測試：經本機 USB 的 TUNE 開關煞車，比較降速階躍（高速 -> 低速）的減速表現。

用法：python car1_brake_test.py [--high 100] [--low 15] [--hold 2.5] [--bmax 128] [--label name]
需要車輛架高。每個條件跑一次：先以 --high 指令跑到穩定，再降到 --low；降速後輪速超過目標很多，
等同「被外力推著超速」的瞬間版本。煞車關（brk 0）= 只調降 duty、滑行；煞車開（brk 1）= 動態煞車。
指標：降到目標 ±5% 帶內並維持到結束的時間（整定）、降速後最低點（欠衝）、最後 1 s 平均相對目標誤差。
目標 RPM = 55 + (pct-1)/99*(280-55)。優先記錄每筆 STA 的 brk，舊版才讀 dbg 旗標。
"""
import argparse
import csv
import os
import time

from car1_bm import Car1

MIN_RPM, MAX_RPM = 55.0, 280.0
# dbg 欄位索引（見韌體 sendStatus 的註解，已去掉開頭的 'dbg'）
DBG_BRAKING_L, DBG_BRAKING_R, DBG_CYC_L, DBG_CYC_R = 17, 18, 19, 20


def target_rpm(pct):
    return 0.0 if pct == 0 else MIN_RPM + (pct - 1) / 99.0 * (MAX_RPM - MIN_RPM)


def run_case(c, brake, high, low, hold, label, out):
    c.tune(brk=1 if brake else 0)
    time.sleep(0.3)
    c.run(0, 0, 1.0)  # 讓輪子停穩
    rows = []  # (t, rpmL, rpmR, dutyL, dutyR, brkL, brkR)
    t0 = time.monotonic()
    for pct, secs in ((high, hold), (low, hold), (0, 1.5)):
        step_t0 = time.monotonic() - t0
        t_end = time.monotonic() + secs
        nxt = time.monotonic()
        while time.monotonic() < t_end:
            if time.monotonic() >= nxt:
                c.cmd(pct, pct)
                nxt += 0.05
            for s in c.poll():
                d = c.dbg or []
                if 'brk' in s:
                    flags = int(s['brk'])
                    bl, br = int(bool(flags & 1)), int(bool(flags & 2))
                else:
                    bl = int(d[DBG_BRAKING_L]) if len(d) > DBG_BRAKING_R else 0
                    br = int(d[DBG_BRAKING_R]) if len(d) > DBG_BRAKING_R else 0
                rows.append((time.monotonic() - t0, float(s['rpmL']), float(s['rpmR']),
                             int(s['dutyL']), int(s['dutyR']), bl, br, pct, int(s['upms']), int(s['f'])))
    reboots = sum(1 for a, b in zip(rows, rows[1:]) if b[8] < a[8])
    tgt = target_rpm(low)
    t_step = hold
    seg = [r for r in rows if t_step <= r[0] < 2 * hold]
    res = {}
    for name, col in (('L', 1), ('R', 2)):
        settle = hold
        for r in reversed(seg):
            if abs(r[col] - tgt) > 0.05 * tgt:
                settle = r[0] - t_step
                break
        else:
            settle = 0.0
        tail = [r[col] for r in seg if r[0] >= 2 * hold - 1.0]
        res[name] = (settle, min(r[col] for r in seg), sum(tail) / len(tail) - tgt if tail else float('nan'))
    hi_seg = [r for r in rows if hold - 1.0 <= r[0] < hold]
    hi = (sum(r[1] for r in hi_seg) / len(hi_seg), sum(r[2] for r in hi_seg) / len(hi_seg)) if hi_seg else (0, 0)
    path = os.path.join(out, f'{label}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'rpmL', 'rpmR', 'dutyL', 'dutyR', 'brakingL', 'brakingR', 'cmd', 'upms', 'f'])
        w.writerows(rows)
    brk_cycles = c.dbg[DBG_CYC_L:DBG_CYC_R + 1] if c.dbg and len(c.dbg) > DBG_CYC_R else '?'
    print(f"{label}: brake={'on ' if brake else 'off'} high={hi[0]:6.1f}/{hi[1]:6.1f} rpm  target={tgt:5.1f}  "
          f"L settle={res['L'][0]:.2f}s min={res['L'][1]:6.1f} tailErr={res['L'][2]:+5.1f} | "
          f"R settle={res['R'][0]:.2f}s min={res['R'][1]:6.1f} tailErr={res['R'][2]:+5.1f}  "
          f"samples={len(rows)} brakeCycles(L,R)={brk_cycles} reboots={reboots} faults={sorted(set(r[9] for r in rows))}")
    return rows


def hand_test(c, low, seconds, out, label):
    """以 low 指令跑 seconds 秒，期間用手把輪子往前撥快，統計超速與煞車反應（需要人在旁邊）。"""
    c.tune(brk=1)
    time.sleep(0.3)
    tgt = target_rpm(low)
    print(f"目標 {tgt:.1f} RPM，{seconds:.0f} s 內請用手把輪子往『前進方向』撥快（可左右輪各撥幾次）。")
    rows = []
    t0 = time.monotonic()
    nxt = 0.0
    while time.monotonic() - t0 < seconds:
        if time.monotonic() - t0 >= nxt:
            c.cmd(low, low)
            nxt += 0.05
        for s in c.poll():
            d = c.dbg or []
            rows.append((time.monotonic() - t0, float(s['rpmL']), float(s['rpmR']), int(s['vL']), int(s['vR']),
                         int(s['dutyL']), int(s['dutyR']),
                         int(d[DBG_BRAKING_L]) if len(d) > DBG_BRAKING_R else 0,
                         int(d[DBG_BRAKING_R]) if len(d) > DBG_BRAKING_R else 0))
    c.run(0, 0, 1.0)
    with open(os.path.join(out, f'{label}_hand.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'rpmL', 'rpmR', 'vL', 'vR', 'dutyL', 'dutyR', 'brakingL', 'brakingR'])
        w.writerows(rows)
    for name, col in (('L', 1), ('R', 2)):
        peak = max(r[col] for r in rows)
        over = [r for r in rows if r[col] > tgt * 1.1]
        print(f"{name}: peak rpm={peak:.1f}  samples>110%target={len(over)}  mean(all)={sum(r[col] for r in rows) / len(rows):.1f}")
    print('brake cycles (L,R):', c.dbg[DBG_CYC_L:DBG_CYC_R + 1] if c.dbg else '?')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM25')
    ap.add_argument('--high', type=int, default=100)
    ap.add_argument('--low', type=int, default=15)
    ap.add_argument('--hold', type=float, default=2.5)
    ap.add_argument('--bmax', type=float)
    ap.add_argument('--bkp', type=float)
    ap.add_argument('--bon', type=float)
    ap.add_argument('--boff', type=float)
    ap.add_argument('--label', default='brake')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), 'pid_data', 'brake_20261117'))
    ap.add_argument('--hand', type=float, help='手撥測試秒數：不跑階躍，只跑 --low 指令並統計手撥造成的超速')
    ap.add_argument('--order', default='off,on,off,on', help='comma list of off/on, run in this order')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    c = Car1(args.port)
    c.run(0, 0, 0.6)
    extra = {k: v for k, v in (('bmax', args.bmax), ('bkp', args.bkp), ('bon', args.bon), ('boff', args.boff)) if v is not None}
    if extra:
        c.tune(**extra)
        time.sleep(0.3)
    try:
        if args.hand:
            hand_test(c, args.low, args.hand, args.out, args.label)
            return
        for i, mode in enumerate(args.order.split(',')):
            run_case(c, mode == 'on', args.high, args.low, args.hold, f'{args.label}_{args.high}to{args.low}_{i}_{mode}', args.out)
    finally:
        c.close()
    print('dbg:', c.dbg, 'bad_frames:', c.bad)


if __name__ == '__main__':
    main()
