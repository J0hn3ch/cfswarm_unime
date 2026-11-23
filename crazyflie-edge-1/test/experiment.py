#!/usr/bin/env python3
"""Run a campaign of straight-line flights: several velocities x several repeats.

Example:
  python experiment.py --velocities 0.1 0.2 0.3 --repeats 5 --measure

Between runs the script waits for you to put the drone back on the start line
(nose along the line) and press Enter. Results go to logs/session_<date>/.
"""
import argparse
import csv
import os
import random
from datetime import datetime

import cflib.crtp

from claude_test1_cf_line import fly_once

COLS = [
    "run", "velocity_ms", "rep", "target_cm",
    "dist_at_stop_cm", "dist_final_cm", "drift_cm", "yaw_change_deg",
    "forward_time_s", "stop_reason",
    "vbat_start_v", "vbat_end_v", "batt_start_pct_est", "batt_end_pct_est",
    "measured_cm", "file", "wall_time",
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--uri", default="radio://0/80/2M/E7E7E7E702")
    p.add_argument("--distance", type=float, default=0.5, help="metres")
    p.add_argument("--velocities", type=float, nargs="+", default=[0.2], help="m/s")
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--height", type=float, default=0.4)
    p.add_argument("--period", type=int, default=50, help="log period, ms")
    p.add_argument("--min-front-mm", type=int, default=250)
    p.add_argument("--outdir", default="logs")
    p.add_argument("--measure", action="store_true",
                   help="ask for the tape-measured distance after each run")
    p.add_argument("--no-shuffle", action="store_true",
                   help="keep runs ordered (default: random order, so battery "
                        "drain is not correlated with velocity)")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    plan = [(v, r + 1) for v in args.velocities for r in range(args.repeats)]
    if not args.no_shuffle:
        random.Random(args.seed).shuffle(plan)

    session = os.path.join(args.outdir, f"session_{datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(session)
    cflib.crtp.init_drivers()
    print(f"Session folder: {session}  ({len(plan)} runs)")

    with open(os.path.join(session, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLS, extrasaction="ignore", restval="")
        w.writeheader()
        try:
            for i, (v, rep) in enumerate(plan, 1):
                print(f"\n--- Run {i}/{len(plan)}: v = {v} m/s (rep {rep}) ---")
                input("Place the drone on the start line, nose along the line. "
                      "Press Enter to fly (Ctrl+C to quit)... ")
                name = f"run{i:02d}_v{v:.2f}_rep{rep}.csv"
                s = fly_once(args.uri, os.path.join(session, name),
                             args.distance, v, args.height,
                             args.period, args.min_front_mm)

                row = {"run": i, "velocity_ms": v, "rep": rep,
                       "target_cm": args.distance * 100, "file": name,
                       "wall_time": datetime.now().isoformat(timespec="seconds")}
                row.update(s)

                print(f"Estimated distance: {s.get('dist_final_cm')} cm, "
                      f"drift: {s.get('drift_cm')} cm, stop: {s['stop_reason']}, "
                      f"battery ~{s.get('batt_end_pct_est')}%")
                if args.measure:
                    txt = input("Measured distance in cm (Enter to skip): ").strip()
                    if txt:
                        try:
                            row["measured_cm"] = float(txt.replace(",", "."))
                        except ValueError:
                            print("Not a number, skipped.")
                w.writerow(row)
                f.flush()

                pct = s.get("batt_end_pct_est")
                if pct is not None and pct < 25:
                    print("WARNING: battery is low, consider swapping it "
                          "and letting the motors cool down.")
        except KeyboardInterrupt:
            print("\nInterrupted. Partial results are saved.")

    print(f"\nDone. Copy {session} to the laptop and run: python analyze.py {session}")


if __name__ == "__main__":
    main()
