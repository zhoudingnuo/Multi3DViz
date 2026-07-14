"""gravity_calib.py — Compute roll/pitch from a static IMU sample window.

FAST-LIO's /cloud_registered is published in the robot's `camera_init` frame,
which is level at power-on but tilts as the dog moves. The control side
applies a fixed gravity-correction rotation R = R_roll(roll_deg) @ R_pitch(pitch_deg)
(data_utils.py:34-37) to flatten clouds. We compute roll_deg/pitch_deg here
from the IMU's gravity vector while the robot is still at startup, and write
gravity_calibration.json with just those two fields (everything else in that
file is ignored by the control side).

This matches the ccenter convention: gravity is measured ONCE at the start of
each run, not continuously — the rotation is a static calibration of the
LiDAR's mounting tilt relative to the gravity vector at boot.
"""
from __future__ import annotations
import math
import logging
import numpy as np

from .atomic_io import save_json_atomic

log = logging.getLogger("m3v_agent.recorder.gravity")


class GravityCalibrator:
    """Accumulate IMU accel samples; when enough are collected, compute + save.

    The robot should be still during collection. We average the accel vectors
    to suppress noise, then derive roll/pitch from the direction of gravity:

        roll  = atan2(g_y, g_z)      (rotation about X)
        pitch = atan2(-g_x, hypot(g_y, g_z))   (rotation about Y)

    Sign convention chosen so that feeding roll_deg/pitch_deg back into the
    control side's data_utils.load_gravity reproduces the identity rotation
    when the sensor is already level (g = [0,0,9.8]). The result is written
    in DEGREES because that's what the control side reads (data_utils.py:33).
    """

    def __init__(self, gravity_path: str, n_samples: int = 500, enabled: bool = True):
        self.path = gravity_path
        self.target = max(10, int(n_samples))
        self.enabled = enabled
        self._accels: list = []          # accumulated [ax, ay, az]
        self._done = False

    def feed_imu(self, accel_xyz):
        """Add one accel sample. accel_xyz = [ax, ay, az] in m/s^2."""
        if not self.enabled or self._done:
            return
        self._accels.append(list(accel_xyz))
        if len(self._accels) >= self.target:
            self.finalize()

    def finalize(self):
        """Compute roll_deg/pitch_deg from collected samples and write the file."""
        if self._done or not self._accels:
            return
        arr = np.asarray(self._accels, dtype=np.float64)
        mean = arr.mean(axis=0)
        gx, gy, gz = mean
        # Guard against zero/NaN (bad sensor).
        if not np.isfinite(mean).all() or np.linalg.norm(mean) < 1e-3:
            log.warning("gravity calibration skipped: bad accel mean %s", mean)
            self._done = True
            return
        roll = math.atan2(gy, gz)
        pitch = math.atan2(-gx, math.hypot(gy, gz))
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        save_json_atomic(self.path, {
            "roll_deg": round(roll_deg, 3),
            "pitch_deg": round(pitch_deg, 3),
            # Informational fields (control side ignores these but they help debugging).
            "raw_acc": [round(float(v), 4) for v in mean.tolist()],
            "samples": int(len(self._accels)),
        })
        log.info("gravity calibration written: roll=%.3f° pitch=%.3f° (n=%d)",
                 roll_deg, pitch_deg, len(self._accels))
        self._done = True

    @property
    def ready(self) -> bool:
        return self._done
