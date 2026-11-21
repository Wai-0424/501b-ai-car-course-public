#!/usr/bin/env python3
"""Turbo 模式上板驗證（USB 直接送 CMD，不需要 Orin）。輪子必須離地。【2026-10-10 寫，尚未執行：BMduino 當時沒接上】

  python turbo_check.py <out.tsv> [--max-cmd 112] [--i-confirm-wheels-lifted]

依序做（每段的目的）：
  base100   100% 保持 4 s            基線，應該是 270 rpm 左右（換電壓後會偏高，前饋表重量後才準）
  turbo106  106% 2 s                 小步 turbo，轉速應該比基線高約 15 rpm
  rest      0% 10 s                  充電（預算充滿需要 10 s）
  turbo112  112% 2.5 s               約 300 rpm 的目標；實際被「前饋表 duty 250 的轉速」夾住
  rest      0% 10 s
  hold112   112% 持續 6 s            預期約 3 s 後被韌體夾回 100%（轉速掉回基線）並且持續被鎖，不會再進 turbo
  back100   100% 6.5 s               不要求 turbo -> 約 5 s 後解除鎖定
  again112  112% 3 s                 預期只有約 1.5 s turbo 又被夾回（預算只充回一半）
  end       0% 2 s
--max-cmd 可降低所有 turbo 命令的上限（例如 108），第一次上板建議從 108 開始。

守門（任何一個成立就立刻歸零並結束）：故障旗標 f != 0、估計轉速 > 350 rpm、超過 1 s 沒收到 STA、
已下 >=15% 但 >1.2 s 仍 < 5 rpm、dbg 行不是 24 欄（韌體不是 turbo 版）。結束一定送 2 s 的 0。

輸出 TSV 欄位：t phase cmd vL vR dutyL dutyR f turboState turboBudgetMs
之後印出每一段的穩態轉速、最高轉速、turboState 轉換時間，以及邏輯檢查（鎖定是否在約 3 s 後發生、鎖定期間是否維持
夾回基線、解除後是否又能進 turbo）。檢查結果只是提示，不取代人工看 TSV 與溫度/電流。
"""
import re
import sys
import time

import serial
from serial.tools import list_ports


def main():
    out_path = sys.argv[1]
    max_cmd = 112
    if "--max-cmd" in sys.argv:
        max_cmd = int(sys.argv[sys.argv.index("--max-cmd") + 1])
    if "--i-confirm-wheels-lifted" not in sys.argv:
        sys.exit("馬達會轉到 turbo 速度，需要 --i-confirm-wheels-lifted（輪子已離地）")
    if max_cmd > 125:
        sys.exit("max-cmd 不可超過 125")
    t106 = min(106, max_cmd)
    plan = [("base100", 100, 4.0), ("turbo106", t106, 2.0), ("rest1", 0, 10.0), ("turbo112", max_cmd, 2.5),
            ("rest2", 0, 10.0), ("hold112", max_cmd, 6.0), ("back100", 100, 6.5), ("again112", max_cmd, 3.0), ("end", 0, 2.0)]

    port = next((p.device for p in list_ports.comports() if p.vid == 0x04D9 and p.pid == 0x802F), None)
    if port is None:
        sys.exit("找不到 Holtek 04D9:802F 序列埠（BMduino 沒接上？）")
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0.02
    ser.dtr = False
    ser.rts = False
    ser.open()

    seq = 5000
    rows = []
    cur = {"vL": 0.0, "vR": 0.0, "f": 0, "dL": 0, "dR": 0, "ts": 0, "bud": 0, "sta_t": None, "dbg_ok": None}
    abort = None

    def cmd(c):
        nonlocal seq
        p = f"CMD,seq {seq},fwa 0,bwl {int(c)},bwr {int(c)}"
        seq = (seq + 1) % 65536
        cs = 0
        for ch in p.encode():
            cs ^= ch
        return f"<{p},cs {cs:02X}>\n".encode()

    def pump(t0, ph, c):
        while True:
            line = ser.readline()
            if not line:
                return
            s = line.decode("ascii", "replace").strip()
            m = re.match(r"<STA,ackseq -?\d+,S1 -?\d+,rpmL -?[\d.]+,rpmR -?[\d.]+,vL (-?\d+),vR (-?\d+),f (\d+),cs", s)
            if m:
                cur.update(vL=float(m.group(1)), vR=float(m.group(2)), f=int(m.group(3)), sta_t=time.time())
                continue
            if s.startswith("dbg,"):
                p = s.split(",")
                cur["dbg_ok"] = (len(p) == 25)
                if len(p) == 25:
                    cur.update(dL=int(p[3]), dR=int(p[4]), ts=int(p[23]), bud=int(float(p[24])))
                    rows.append((time.time() - t0, ph, c, cur["vL"], cur["vR"], cur["dL"], cur["dR"], cur["f"], cur["ts"], cur["bud"]))

    try:
        time.sleep(0.6)
        ser.reset_input_buffer()
        t0 = time.time()
        for ph, (name, c, dur) in enumerate(plan):
            t_ph = time.time()
            t_end = t_ph + dur
            nxt = time.time()
            while time.time() < t_end and abort is None:
                now = time.time()
                if now >= nxt:
                    ser.write(cmd(c))
                    nxt = now + 0.05
                pump(t0, ph, c)
                if cur["dbg_ok"] is False:
                    abort = "dbg 行不是 24 欄：韌體不是 turbo 版"
                elif cur["f"] != 0:
                    abort = f"故障旗標 f={cur['f']}（phase {name}）"
                elif max(abs(cur["vL"]), abs(cur["vR"])) > 350:
                    abort = f"轉速 > 350 rpm（phase {name}）"
                elif cur["sta_t"] is not None and now - cur["sta_t"] > 1.0:
                    abort = "超過 1 s 沒收到 STA"
                elif now - t_ph > 1.2 and c >= 15 and (abs(cur["vL"]) < 5 or abs(cur["vR"]) < 5):
                    abort = f"已下指令但輪子沒轉（phase {name}，vL={cur['vL']:.0f} vR={cur['vR']:.0f}）"
            if abort:
                break
    finally:
        t_end = time.time() + 2.0
        nxt = time.time()
        while time.time() < t_end:
            if time.time() >= nxt:
                ser.write(cmd(0))
                nxt = time.time() + 0.05
            pump(t0, -1, 0)
        ser.close()

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("t\tphase\tcmd\tvL\tvR\tdutyL\tdutyR\tf\tturboState\tturboBudgetMs\n")
        for r in rows:
            f.write("\t".join(f"{x:.2f}" if isinstance(x, float) else str(x) for x in r) + "\n")
    print("abort:", abort or "none", "| rows:", len(rows))

    # ---- 摘要 ----
    print(f"\n{'phase':10s} {'cmd':>4s} {'穩態 vL/vR (最後1 s)':>22s} {'最高 vL/vR':>14s} {'turboState 轉換 (相對段起點 s)'}")
    stats = {}
    for ph, (name, c, dur) in enumerate(plan):
        seg = [r for r in rows if r[1] == ph]
        if len(seg) < 3:
            print(f"{name:10s} {c:4d}  (資料不足)")
            continue
        t0p = seg[0][0]
        tail = [r for r in seg if r[0] >= seg[-1][0] - 1.0]
        mL = sum(r[3] for r in tail) / len(tail)
        mR = sum(r[4] for r in tail) / len(tail)
        trans = []
        for a, b in zip(seg, seg[1:]):
            if a[8] != b[8]:
                trans.append(f"{a[8]}->{b[8]}@{b[0]-t0p:.1f}")
        stats[name] = dict(mL=mL, mR=mR, pk=max(max(r[3], r[4]) for r in seg), trans=trans, seg=seg, t0=t0p)
        print(f"{name:10s} {c:4d} {mL:10.1f}/{mR:<10.1f} {max(r[3] for r in seg):6.0f}/{max(r[4] for r in seg):<6.0f} {' '.join(trans) or '-'}")

    # ---- 邏輯檢查（提示用）----
    print("\n邏輯檢查（提示，不取代人工判讀）：")
    base = stats.get("base100")
    if base:
        bL, bR = base["mL"], base["mR"]
        hold = stats.get("hold112")
        if hold:
            locked_at = next((float(t.split("@")[1]) for t in hold["trans"] if t.startswith("1->2")), None)
            print(f"  hold112：turboState 1->2（鎖定）發生在 {locked_at if locked_at is not None else '沒有發生'} s（預期約 3 s）")
            tail = [r for r in hold["seg"] if r[0] >= hold["seg"][-1][0] - 1.0]
            tl = sum(r[3] for r in tail) / len(tail)
            print(f"  hold112 最後 1 s 的 vL = {tl:.1f} rpm，基線 {bL:.1f} rpm（預期鎖定後夾回基線 ±5%）：{'OK' if abs(tl - bL) <= 0.05 * bL else '不符'}")
        b100 = stats.get("back100")
        if b100:
            un = next((float(t.split("@")[1]) for t in b100["trans"] if t.startswith("2->0")), None)
            print(f"  back100：鎖定解除（2->0）在 {un if un is not None else '沒有發生'} s（預期約 5 s）")
        ag = stats.get("again112")
        if ag:
            on = [r for r in ag["seg"] if r[8] == 1]
            span = (on[-1][0] - on[0][0]) if len(on) > 1 else 0.0
            print(f"  again112：turbo 持續約 {span:.1f} s（預期約 1.5 s，預算只充回一半）")
    for nm in ("turbo106", "turbo112"):
        if nm in stats and base:
            print(f"  {nm}：穩態 {stats[nm]['mL']:.1f}/{stats[nm]['mR']:.1f} rpm，比基線 {base['mL']:.1f}/{base['mR']:.1f} 高 "
                  f"{stats[nm]['mL'] - base['mL']:+.1f}/{stats[nm]['mR'] - base['mR']:+.1f} rpm")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
