"""Shared helpers (runs on the Pi): CSV recorder + one straight-line flight."""
import bisect
import csv
import threading
import time

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.reset_estimator import reset_estimator

FIELDS = [
    "t_host_s", "t_fw_ms", "phase",
    "x_m", "y_m", "z_m", "vx_ms", "vy_ms", "yaw_deg",
    "front_mm", "back_mm", "left_mm", "right_mm", "up_mm", "down_mm",
    "vbat_v", "battery_pct_est",
]

# Generic 1S LiPo open-circuit (REST) voltage -> state of charge. Unverified,
# valid only with the motors off (before takeoff / after landing).
_V = [3.27, 3.61, 3.69, 3.71, 3.73, 3.75, 3.77, 3.79, 3.80, 3.82, 3.84,
      3.85, 3.87, 3.91, 3.95, 3.98, 4.02, 4.08, 4.11, 4.15, 4.20]
_P = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50,
      55, 60, 65, 70, 75, 80, 85, 90, 95, 100]


# Measured IN-FLIGHT curve (hover, 0.3 m), battery B1, from a full discharge
# (discharge_B1_20261016_184657.csv). 0 % = the drone can no longer hold altitude,
# 100 % = start of hover. Percent = fraction of remaining hover time.
# Under load the voltage is ~0.5 V lower than at rest, so the rest table
# must NOT be used while the motors are running.
_V_FLIGHT = [2.634, 2.777, 2.866, 2.919, 2.951, 3.009, 3.031, 3.072, 3.088,
             3.109, 3.130, 3.167, 3.194, 3.257, 3.267, 3.320, 3.368, 3.399,
             3.452, 3.521, 3.632]
_P_FLIGHT = list(range(0, 101, 5))

NOT_FLYING = ("init", "ground", "landed")


def _interp(v, xs, ps):
    if v <= xs[0]:
        return 0.0
    if v >= xs[-1]:
        return 100.0
    i = bisect.bisect_right(xs, v)
    x0, x1, p0, p1 = xs[i - 1], xs[i], ps[i - 1], ps[i]
    return round(p0 + (p1 - p0) * (v - x0) / (x1 - x0), 1)


def vbat_to_pct(v, flying=False):
    """Battery estimate from voltage. flying=True uses the in-flight (loaded) table."""
    if v is None:
        return None
    if flying:
        return _interp(v, _V_FLIGHT, _P_FLIGHT)
    return _interp(v, _V, _P)


class Recorder:
    """Receives log packets, merges them, writes one CSV row per state packet."""

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(FIELDS)
        self._lock = threading.Lock()
        self._closed = False
        self._rng = {}
        self.x = self.y = self.yaw = 0.0
        self.phase = "init"
        self.t0 = time.monotonic()  # monotonic: immune to NTP/clock jumps

    @property
    def ranges(self):
        return self._rng

    def state_cb(self, ts, d, _):
        with self._lock:
            if self._closed:
                return
            self.x = d["stateEstimate.x"]
            self.y = d["stateEstimate.y"]
            self.yaw = d["stabilizer.yaw"]
            r = self._rng
            vb = r.get("pm.vbat")
            self._w.writerow([
                round(time.monotonic() - self.t0, 4), ts, self.phase,
                self.x, self.y, d["stateEstimate.z"],
                d["stateEstimate.vx"], d["stateEstimate.vy"], self.yaw,
                r.get("range.front"), r.get("range.back"), r.get("range.left"),
                r.get("range.right"), r.get("range.up"), r.get("range.zrange"),
                vb, vbat_to_pct(vb, flying=self.phase not in NOT_FLYING),
            ])

    def range_cb(self, ts, d, _):
        with self._lock:
            self._rng = dict(d)

    def close(self):
        with self._lock:
            if not self._closed:
                self._closed = True
                self._f.flush()
                self._f.close()


def make_logconfigs(cf, rec, period_ms):
    # Each log packet must be <= 26 bytes
    lc_state = LogConfig(name="state", period_in_ms=period_ms)
    for v in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z",
              "stateEstimate.vx", "stateEstimate.vy", "stabilizer.yaw"):
        lc_state.add_variable(v, "float")

    lc_rng = LogConfig(name="ranges", period_in_ms=period_ms)
    for v in ("range.front", "range.back", "range.left",
              "range.right", "range.up", "range.zrange"):
        lc_rng.add_variable(v, "uint16_t")
    lc_rng.add_variable("pm.vbat", "float")

    cf.log.add_config(lc_state)
    cf.log.add_config(lc_rng)
    lc_state.data_received_cb.add_callback(rec.state_cb)
    lc_rng.data_received_cb.add_callback(rec.range_cb)
    return lc_state, lc_rng


def fly_once(uri, csv_path, distance=1.0, velocity=0.2, height=0.3,
             period_ms=50, min_front_mm=250):
    """Take off, fly `distance` metres forward, land. Log to csv_path.
    Returns a dict with the run summary."""
    rec = Recorder(csv_path)
    s = {"stop_reason": "distance"}
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
            scf.wait_for_params()
            scf.cf.param.set_value('stabilizer.estimator', str(2))
            scf.cf.param.set_value('stabilizer.controller', str(0))
            scf.cf.param.get_value("hlCommander.vtoff", timeout=60)
            reset_estimator(scf)

            lc_state, lc_rng = make_logconfigs(scf.cf, rec, period_ms)
            rec.phase = "ground"
            lc_state.start()
            lc_rng.start()
            time.sleep(1.0)
            vb = rec.ranges.get("pm.vbat")
            s["vbat_start_v"] = vb
            s["batt_start_pct_est"] = vbat_to_pct(vb)

            try:
                # MotionCommander lands automatically when leaving the 'with'
                rec.phase = "takeoff"  # takeoff happens inside __enter__
                with MotionCommander(scf, default_height=height) as mc:
                    rec.phase = "hover"
                    time.sleep(2.0)

                    x0, y0, yaw0 = rec.x, rec.y, rec.yaw
                    rec.phase = "forward"
                    t_start = time.monotonic()
                    timeout = 2 * distance / velocity + 2
                    mc.start_forward(velocity=velocity)
                    while True:
                        if rec.x - x0 >= distance:
                            mc.start_turn_left(rate=180.0)
                            break
                        front = rec.ranges.get("range.front")
                        if front is not None and front < min_front_mm:
                            s["stop_reason"] = "obstacle"
                            break
                        if time.monotonic() - t_start > timeout:
                            s["stop_reason"] = "timeout"
                            break
                        time.sleep(0.02)
                    s["forward_time_s"] = round(time.monotonic() - t_start, 2)
                    mc.stop()
                    s["dist_at_stop_cm"] = round((rec.x - x0) * 100, 1)

                    rec.phase = "settle"
                    time.sleep(1.5)
                    s["dist_final_cm"] = round((rec.x - x0) * 100, 1)
                    s["drift_cm"] = round((rec.y - y0) * 100, 1)
                    s["yaw_change_deg"] = round(rec.yaw - yaw0, 1)
                    rec.phase = "landing"

                rec.phase = "landed"
                time.sleep(1.0)
                vb = rec.ranges.get("pm.vbat")
                s["vbat_end_v"] = vb
                s["batt_end_pct_est"] = vbat_to_pct(vb)
            finally:
                lc_state.stop()
                lc_rng.stop()
                time.sleep(0.2)
    finally:
        rec.close()
    return s
