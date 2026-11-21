import csv
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

data_dir = Path(__file__).resolve().parent / "example_data"
csv_path = data_dir / "hall_motor_sweep.csv"

duties = []
rpms = []
with open(csv_path, encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        duties.append(int(row["duty"]))
        rpms.append(float(row["rpm"]))

duties = np.array(duties)
rpms = np.array(rpms)

# 死區以上的點做線性回歸（排除0附近雜訊點）
mask = rpms > 50
d_fit = duties[mask]
r_fit = rpms[mask]

slope, intercept = np.polyfit(d_fit, r_fit, 1)  # rpm = slope*duty + intercept
pred = slope * d_fit + intercept
ss_res = np.sum((r_fit - pred) ** 2)
ss_tot = np.sum((r_fit - np.mean(r_fit)) ** 2)
r2 = 1 - ss_res / ss_tot

# 換算成 feedforward 慣用形式: duty = ff_deadband + ff_slope*rpm
ff_slope = 1.0 / slope
ff_deadband = -intercept / slope

plt.rcParams["font.sans-serif"] = ["Microsoft JhengHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

fig, ax = plt.subplots(figsize=(9, 6))
ax.plot(duties, rpms, marker="o", label="實測", color="#1f77b4")
d_line = np.linspace(d_fit.min(), d_fit.max(), 100)
ax.plot(d_line, slope * d_line + intercept, "--", label=f"線性回歸 (R²={r2:.4f})", color="#d62728")

ax.set_xlabel("PWM duty (0-255)")
ax.set_ylabel("RPM (馬達軸轉速)")
ax.set_title("Hennkwell PK32K5SP284HE12T-027BBQ  PWM duty vs RPM 曲線 (10kHz)")
ax.grid(True, alpha=0.3)
ax.legend()
ax.set_xlim(0, 255)
ax.set_ylim(bottom=0)

fig.tight_layout()
out_path = data_dir / "hall_motor_sweep_curve.png"
fig.savefig(out_path, dpi=150)
print("saved:", out_path)

print(f"linear fit: rpm = {slope:.4f}*duty + {intercept:.2f}   R2={r2:.4f}")
print(f"feedforward form: duty = {ff_deadband:.2f} + {ff_slope:.4f}*rpm")
print(f"deadband: between duty=50 (rpm={rpms[duties==50][0]}) and duty=60 (rpm={rpms[duties==60][0]})")
print(f"max rpm: {rpms.max():.1f} @ duty={duties[np.argmax(rpms)]}")
