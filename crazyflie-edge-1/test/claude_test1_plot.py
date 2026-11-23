#!/usr/bin/env python3
"""Plot one run CSV.   Usage: python plot_log.py logs/session_XXXX/run01_....csv"""
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if len(sys.argv) < 2:
    sys.exit("usage: plot_log.py <run.csv>")

path = sys.argv[1]
df = pd.read_csv(path)

# Time axis from the firmware timestamp (wraps at 2^24 ms)
ts = df["t_fw_ms"].values
df["t"] = np.concatenate([[0], np.cumsum(np.diff(ts) % (1 << 24))]) / 1000.0

# Ranger values are in mm; very large values mean "nothing in range"
for c in ("front_mm", "back_mm", "left_mm", "right_mm", "up_mm", "down_mm"):
    df[c] = df[c].where(df[c] < 4000, np.nan) / 10.0  # -> cm

fwd = df[df["phase"] == "forward"]
i0 = fwd.index[0] if len(fwd) else df.index[0]
df["x_rel_cm"] = (df["x_m"] - df.loc[i0, "x_m"]) * 100
df["y_rel_cm"] = (df["y_m"] - df.loc[i0, "y_m"]) * 100
# Smoothed estimate (1 s median) to hide the voltage sag/ripple
df["pct_smooth"] = df["battery_pct_est"].rolling(20, center=True, min_periods=1).median()
t = df["t"]

fig, ax = plt.subplots(4, 2, figsize=(13, 13), constrained_layout=True)
ax = ax.ravel()
colors = {"ground": "#eee", "takeoff": "#ffe9a8", "hover": "#ddd", "forward": "#cfe8ff",
          "settle": "#eee", "landing": "#fde0c5", "landed": "#eee"}


def shade(a):
    for ph, g in df.groupby("phase"):
        if ph in colors:
            a.axvspan(g["t"].min(), g["t"].max(), color=colors[ph], alpha=0.5, lw=0)


for i, a in enumerate(ax):
    if i != 5:
        shade(a)
        a.set_xlabel("time [s]")
        a.grid(alpha=0.3)

ax[0].plot(t, df["x_rel_cm"], label="x (estimated)")
ax[0].axhline(100, ls="--", c="r", label="target (100 cm)")
ax[0].set(ylabel="x [cm]", title="Forward distance"); ax[0].legend()

ax[1].plot(t, df["y_rel_cm"], c="tab:orange")
ax[1].set(ylabel="y [cm]", title="Lateral drift")

ax[2].plot(t, df["z_m"] * 100, label="z (estimated)")
ax[2].plot(t, df["down_mm"], label="zrange (Flow Deck)", alpha=0.7)
ax[2].set(ylabel="height [cm]", title="Height"); ax[2].legend()

ax[3].plot(t, df["vx_ms"], label="vx")
ax[3].plot(t, df["vy_ms"], label="vy")
ax[3].set(ylabel="velocity [m/s]", title="Velocity"); ax[3].legend()

for c in ("front_mm", "back_mm", "left_mm", "right_mm"):
    ax[4].plot(t, df[c], label=c.split("_")[0])
ax[4].set(ylabel="distance [cm]", title="Multi-ranger"); ax[4].legend()

ax[5].plot(df["x_rel_cm"], df["y_rel_cm"])
ax[5].set(xlabel="x [cm]", ylabel="y [cm]", title="Top-down trajectory")
ax[5].set_aspect("equal", "datalim"); ax[5].grid(alpha=0.3)

ax[6].plot(t, df["vbat_v"], c="tab:green")
ax[6].set(ylabel="voltage [V]", title="Battery voltage (sags under load)")

ax[7].plot(t, df["battery_pct_est"], alpha=0.3, label="instantaneous")
ax[7].plot(t, df["pct_smooth"], label="smoothed (1 s median)")
ax[7].set(ylabel="battery [%]", title="Battery estimate (from voltage, not exact)")
ax[7].set_ylim(0, 100); ax[7].legend()

if len(fwd):
    print(f"Estimated distance in 'forward' phase: {fwd['x_m'].iloc[-1] - fwd['x_m'].iloc[0]:.3f} m")
print(f"Final lateral drift: {df['y_rel_cm'].iloc[-1]:.1f} cm")
g = df[df["phase"] == "ground"]
if len(g):
    print(f"Battery at rest before takeoff: {g['vbat_v'].median():.2f} V "
          f"(~{g['battery_pct_est'].median():.0f}%)")

out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=150)
print(f"Saved {out}")
plt.show()
