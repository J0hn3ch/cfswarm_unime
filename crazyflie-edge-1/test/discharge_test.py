#!/usr/bin/env python3
"""Hover until the battery is (nearly) empty and log pm.vbat & co.

  python discharge_test.py --battery-id B1                # lands at 3.2 V
  python discharge_test.py --battery-id B1 --cutoff-v 0   # run until it falls

START WITH A FULLY CHARGED, COOL BATTERY, in a clear area, and stay next to the drone.
Running to power-off over-discharges the cell: use a battery you can afford to degrade.
"""
import argparse
import csv
import os
import statistics
import threading
import time
from collections import deque
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Logged only if present in the firmware's log TOC
CANDIDATES = ["pm.vbat", "pm.batteryLevel", "pm.state", "stabilizer.thrust",
              "stateEstimate.z", "range.zrange"]


def in_toc(cf, full_name):
    group, name = full_name.split(".")
    return name in cf.log.toc.toc.get(group, {})


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--uri", default="radio://0/80/2M/E7E7E7E7E7")
    p.add_argument("--battery-id", default="B1")
    p.add_argument("--height", type=float, default=0.3)
    p.add_argument("--period", type=int, default=100, help="log period, ms")
    p.add_argument("--cutoff-v", type=float, default=3.2,
                   help="land when filtered vbat < this (0 = run until power-off)")
    p.add_argument("--filter-s", type=float, default=2.0,
                   help="window of the median filter used for the cutoff")
    p.add_argument("--max-minutes", type=float, default=20)
    p.add_argument("--outdir", default="logs")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    path = os.path.join(args.outdir,
                        f"discharge_{args.battery_id}_{datetime.now():%Y%m%d_%H%M%S}.csv")

    lock = threading.Lock()
    st = {"phase": "init", "last_rx": time.monotonic(), "n": 0}
    recent = deque()  # (t_monotonic, vbat)
    t0 = time.monotonic()
    f = None
    writer = None

    def on_data(ts, d, _):
        now = time.monotonic()
        with lock:
            if writer is None or f.closed:
                return
            writer.writerow([round(now - t0, 3), ts, st["phase"]] +
                            [d.get(k) for k in avail])
            st["n"] += 1
            if st["n"] % 10 == 0:
                f.flush()  # survive an abrupt end
            st["last_rx"] = now
            recent.append((now, d.get("pm.vbat")))

    def filtered_vbat():
        now = time.monotonic()
        with lock:
            while recent and now - recent[0][0] > args.filter_s:
                recent.popleft()
            vals = [v for _, v in recent if v is not None]
        return statistics.median(vals) if len(vals) >= 5 else None

    cflib.crtp.init_drivers()
    avail = []
    try:
        with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            avail = [n for n in CANDIDATES if in_toc(cf, n)]
            missing = [n for n in CANDIDATES if n not in avail]
            if "pm.vbat" not in avail:
                raise SystemExit("pm.vbat not found in the log TOC")
            if missing:
                print("Not available in this firmware, skipped:", ", ".join(missing))

            f = open(path, "w", newline="")
            writer = csv.writer(f)
            writer.writerow(["t_host_s", "t_fw_ms", "phase"] + avail)

            lc = LogConfig(name="discharge", period_in_ms=args.period)
            for n in avail:
                lc.add_variable(n)  # type taken from the TOC
            cf.log.add_config(lc)
            lc.data_received_cb.add_callback(on_data)
            st["phase"] = "ground"
            lc.start()
            time.sleep(2.0)

            v0 = filtered_vbat()
            print(f"Rest voltage before takeoff: {v0} V")
            if v0 is not None and v0 < 4.1:
                print("WARNING: the battery does not look fully charged (< 4.1 V).")
            input("Clear area, battery fully charged? Press Enter to take off "
                  "(Ctrl+C afterwards = controlled landing)... ")

            reason = "unknown"
            try:
                st["phase"] = "takeoff"  # takeoff happens inside __enter__
                with MotionCommander(scf, default_height=args.height):
                    st["phase"] = "hover"
                    t_hover = time.monotonic()
                    while True:
                        time.sleep(0.2)
                        now = time.monotonic()
                        if now - st["last_rx"] > 2.0:
                            reason = "no data for 2 s (link lost / drone off)"
                            break
                        v = filtered_vbat()
                        if args.cutoff_v > 0 and v is not None and v < args.cutoff_v:
                            reason = f"cutoff: filtered vbat {v:.2f} V < {args.cutoff_v} V"
                            break
                        if now - t_hover > args.max_minutes * 60:
                            reason = "max time reached"
                            break
                    st["phase"] = "landing"
            except KeyboardInterrupt:
                reason = "Ctrl+C"
            print(f"\nEnded: {reason}")
            print(f"Hover time: {(time.monotonic() - t_hover) / 60:.1f} min")
            try:
                lc.stop()
            except Exception:
                pass
    except Exception as e:
        print(f"Stopped by exception: {e!r}")
    finally:
        with lock:
            if f is not None and not f.closed:
                f.flush()
                f.close()
        print(f"Saved {path}")


if __name__ == "__main__":
    main()
