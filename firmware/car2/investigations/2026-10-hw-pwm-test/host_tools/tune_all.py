#!/usr/bin/env python3
"""依序：產生測試版 -> 編譯 -> 燒錄 -> 跑階梯計畫 -> 存 TSV。輪子必須離地。
  python tune_all.py <tag> <variants.json>
variants.json: {"name": {"KP": "2.0", ...}, ...}（依序執行）
每個變體約 70~90 秒。結果在 ../results/tune_<tag>_<name>.tsv，編譯暫存在 scratch。
"""
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
RES = HERE.parent / "results"
RES.mkdir(exist_ok=True)
SCR = Path(sys.argv[3]) if len(sys.argv) > 3 else Path.home() / "tune_scratch"
FQBN = "HT32:HT32:BM53A367A"
PORT = "COM10"

# 計畫 P：起步 / 大小階梯 / 100% / 向下 / 低速 / 左右差動；最高 100%（輪子離地）。
PLAN = ("30,30,2.5;60,60,2.5;30,30,2.5;100,100,2.5;50,50,2.5;15,15,2.5;0,0,1.5;"
        "8,8,2.5;0,0,1.5;40,40,2.0;25,25,2.0;40,40,2.0;25,25,2.0;"
        "30,30,2.0;20,40,2.0;40,20,2.0;30,30,2.0;0,0,1.5")


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def main():
    tag = sys.argv[1]
    variants = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
    SCR.mkdir(parents=True, exist_ok=True)
    for name, ov in variants.items():
        t0 = time.time()
        out_tsv = RES / f"tune_{tag}_{name}.tsv"
        mk = run([sys.executable, str(HERE / "make_variant.py"), str(SCR / "src"), name]
                 + [f"{k}={v}" for k, v in ov.items()])
        if mk.returncode:
            print(name, "make_variant 失敗:", mk.stderr or mk.stdout)
            continue
        sketch = SCR / "src" / name
        build = SCR / "build" / name
        c = run(["arduino-cli", "compile", "--fqbn", FQBN, "--build-path", str(build), str(sketch)])
        if c.returncode:
            print(name, "編譯失敗:\n", c.stdout[-800:], c.stderr[-800:])
            continue
        u = run(["arduino-cli", "upload", "--fqbn", FQBN, "-p", PORT, "--input-dir", str(build), str(sketch)])
        if u.returncode:
            print(name, "燒錄失敗:\n", u.stdout[-500:], u.stderr[-500:])
            continue
        time.sleep(2.0)
        d = run([sys.executable, str(HERE / "tune_drive.py"), PLAN, str(out_tsv)])
        print(f"[{name:18s}] {ov} -> {d.stdout.strip()}  ({time.time() - t0:.0f}s)", flush=True)
        if d.stderr.strip():
            print("   stderr:", d.stderr.strip()[-300:])


if __name__ == "__main__":
    main()
