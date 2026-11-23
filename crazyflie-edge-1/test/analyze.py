#!/usr/bin/env python3
"""Statistics + plots for a session folder.  Usage: python analyze.py logs/session_XXXX"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if len(sys.argv) < 2:
    sys.exit("usage: analyze.py <session_folder>")
session = sys.argv[1]

s = pd.read_csv(os.path.join(session, "summary.csv"))
s["err_cm"] = s["dist_final_cm"] - s["target_cm"]
s["abs_drift_cm"] = s["drift_cm"].abs()
has_meas = s["measured_cm"].notna().any()
if has_meas:
    s["est_minus_meas_cm"] = s["dist_final_cm"] - s["measured_cm"]
    s["meas_err_cm"] = s["measured_cm"] - s["target_cm"]

# ---------- statistics per velocity ----------
agg = dict(
    n=("run", "count"),
    dist_mean=("dist_final_cm", "mean"), dist_std=("dist_final_cm", "std"),
    err_mean=("err_cm", "mean"),
    drift_mean=("drift_cm", "mean"), drift_std=("drift_cm", "std"),
    abs_drift_mean=("abs_drift_cm", "mean"),
    time_mean=("forward_time_s", "mean"), time_std=("forward_time_s", "std"),
)
if has_meas:
    agg.update(
        meas_mean=("measured_cm", "mean"), meas_std=("measured_cm", "std"),
        est_minus_meas_mean=("est_minus_meas_cm", "mean"),
    )
stats = s.groupby("velocity_ms").agg(**agg)
print("\nPer-velocity statistics (cm, s). std is NaN when n=1.\n")
print(stats.round(2).to_string())
stats.round(3).to_csv(os.path.join(session, "stats.csv"))
bad = s[s["stop_reason"] != "distance"]
if len(bad):
    print(f"\nNote: {len(bad)} run(s) did not end by reaching the distance:")
    print(bad[["run", "velocity_ms", "stop_reason"]].to_string(index=False))

# ---------- load runs ----------
vels = sorted(s["velocity_ms"].unique())
cmap = plt.get_cmap("tab10")
col = {v: cmap(i % 10) for i, v in enumerate(vels)}


def load(row):
    df = pd.read_csv(os.path.join(session, row["file"]))
    ts = df["t_fw_ms"].values
    # firmware timestamp (wraps at 2^24 ms) -> seconds
    df["t_s"] = np.concatenate([[0], np.cumsum(np.diff(ts) % (1 << 24))]) / 1000.0
    f = df[df["phase"] == "forward"]
    i0 = f.index[0] if len(f) else df.index[0]
    df["t_rel"] = df["t_s"] - df.loc[i0, "t_s"]
    df["x_rel_cm"] = (df["x_m"] - df.loc[i0, "x_m"]) * 100
    df["y_rel_cm"] = (df["y_m"] - df.loc[i0, "y_m"]) * 100
    return df[df["phase"].isin(["forward", "settle"])]


runs = [(r, load(r)) for _, r in s.iterrows()]

# ---------- figure ----------
fig, ax = plt.subplots(2, 3, figsize=(15, 8.5), constrained_layout=True)
ax = ax.ravel()
seen = set()


def lab(v):
    if v in seen:
        return None
    seen.add(v)
    return f"{v:g} m/s"


for r, d in runs:
    v = r["velocity_ms"]
    ax[0].plot(d["t_rel"], d["x_rel_cm"], c=col[v], alpha=0.7, label=lab(v))
    ax[1].plot(d["t_rel"], d["vx_ms"], c=col[v], alpha=0.7)
    ax[2].plot(d["x_rel_cm"], d["y_rel_cm"], c=col[v], alpha=0.7)

ax[0].axhline(s["target_cm"].iloc[0], ls="--", c="r", label="target")
ax[0].set(xlabel="time since start of motion [s]", ylabel="x [cm]", title="Distance vs time")
ax[0].legend()
ax[1].set(xlabel="time since start of motion [s]", ylabel="vx [m/s]", title="Forward velocity")
ax[2].set(xlabel="x [cm]", ylabel="y [cm]", title="Top-down trajectories")
ax[2].set_aspect("equal", "datalim")

rng = np.random.default_rng(0)


def strip(a, column, title, ylabel, target=None, meas=None):
    for i, v in enumerate(vels):
        sub = s[s["velocity_ms"] == v]
        xs = i + rng.uniform(-0.12, 0.12, len(sub))
        a.scatter(xs, sub[column], color=col[v], alpha=0.8, zorder=3)
        m, sd = sub[column].mean(), sub[column].std()
        a.errorbar(i, m, yerr=0 if np.isnan(sd) else sd, fmt="_", color="k",
                   capsize=8, ms=22, zorder=4)
        if meas is not None and sub[meas].notna().any():
            a.scatter(xs, sub[meas], marker="x", color="k", alpha=0.6, zorder=3,
                      label="measured (tape)" if i == 0 else None)
    if target is not None:
        a.axhline(target, ls="--", c="r")
    a.set_xticks(range(len(vels)))
    a.set_xticklabels([f"{v:g} m/s" for v in vels])
    a.set(title=title, ylabel=ylabel)
    a.grid(alpha=0.3)


strip(ax[3], "dist_final_cm", "Final distance (dots: estimated, black bar: mean ± std)",
      "distance [cm]", target=s["target_cm"].iloc[0],
      meas="measured_cm" if has_meas else None)
if has_meas:
    ax[3].legend()
strip(ax[4], "drift_cm", "Lateral drift at end of run", "y [cm]", target=0)

ax[5].plot(s["run"], s["batt_start_pct_est"], "o-", label="before takeoff")
ax[5].plot(s["run"], s["batt_end_pct_est"], "s--", label="after landing")
ax[5].set(xlabel="run #", ylabel="battery [%] (estimate)", title="Battery at rest (voltage-based)")
ax[5].set_ylim(0, 100)
ax[5].grid(alpha=0.3)
ax[5].legend()

out = os.path.join(session, "analysis.png")
fig.savefig(out, dpi=150)
print(f"\nSaved {out} and stats.csv")
plt.show()
