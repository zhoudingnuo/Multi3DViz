"""agibot_driver.py — Agibot D1 Edu-Ultra driver (mc_sdk_zsl_1_py, UDP 43988).

Based on the verified Agibot control doc (2026-07-13 real-robot validation).
Key facts that shaped this driver:

  - SDK: mc_sdk_zsl_1_py.HighLevel. initRobot(local_ip, port, robot_ip).
  - Transport: UDP 43988. Robot's sdk_config.yaml target_ip must point at the
    machine running this driver (default 192.168.234.18). The robot sends
    state to that IP; the SDK sends commands back to the robot IP.
  - State machine: standUp() before move(). lieDown() releases. passive() =
    emergency stop (damping). move(vx,vy,yaw) requires standing.
  - Return values are reliable for ACTION commands (0x0 = success) but
    STATE getters are buggy (getBatteryPower→0, ctrlmode→58 always). So we
    rely on:
      - action return codes for confirming standUp/lieDown/move acceptance
      - the recorder's odom cache for pose (not the SDK getters)
  - move() has a 0.1 deadzone; magnitudes below that are ignored by the SDK,
    so we floor our commands to 0.1 to make sure small corrections register.

Velocity ranges (from the doc, enforced by the robot):
    vx  -3..-0.05 / 0.05..3 m/s        forward/back
    vy  -1.0..-0.1 / 0.1..1.0 m/s      lateral
    yaw -3.0..-0.02 / 0.02..3.0 rad/s  turn

This driver doesn't push those limits — the navigator already clamps to
cfg.max_fwd (0.5) / cfg.max_turn (1.0), well inside the safe envelope.
"""
from __future__ import annotations
import sys
import time
import logging
import threading

from ..executor.base_driver import BaseDriver

log = logging.getLogger("m3v_agent.driver.agibot")

# Action-command return code meaning success (from the doc §4.3).
RC_OK = 0x0
# move() deadzone from speed.yaml — magnitudes below this are dropped.
MOVE_DEADZONE = 0.1


class AgibotDriver(BaseDriver):
    """mc_sdk_zsl_1_py adapter for the Agibot D1 Edu-Ultra."""

    def __init__(self, cfg, recorder=None):
        # `cfg` is a DriverCfg dataclass.
        self.cfg = cfg
        self.recorder = recorder
        self._app = None             # mc_sdk_zsl_1_py.HighLevel
        self._connected = False
        self._standing = False
        self._lock = threading.Lock()

    # --- lifecycle ---
    def connect(self) -> bool:
        # mc_sdk_zsl_1_py is a compiled .so shipped by the vendor under
        # lib/zsl-1/aarch64/. The user must point cfg.agibot_sdk_lib_path at it.
        lib = self.cfg.agibot_sdk_lib_path
        if lib and lib not in sys.path:
            sys.path.insert(0, lib)
        try:
            import mc_sdk_zsl_1_py
        except ImportError as e:
            log.error("mc_sdk_zsl_1_py not found (looked in %s): %s", lib or "sys.path", e)
            log.error("set driver.agibot_sdk_lib_path to the dir containing the .so")
            return False
        try:
            app = mc_sdk_zsl_1_py.HighLevel()
            app.initRobot(self.cfg.agibot_local_ip,
                          self.cfg.agibot_local_port,
                          self.cfg.agibot_robot_ip)
            self._app = app
            # Give the UDP link a moment to come up. The doc recommends a brief
            # settle before issuing commands.
            time.sleep(0.5)
            # Sanity-check connectivity.
            ok = False
            try:
                ok = bool(app.checkConnect())
            except Exception:
                pass
            self._connected = True
            log.info("agibot sdk initialized (checkConnect=%s) local=%s robot=%s",
                     ok, self.cfg.agibot_local_ip, self.cfg.agibot_robot_ip)
            return True
        except Exception as e:
            log.exception("agibot initRobot failed: %s", e)
            return False

    def disconnect(self):
        self._app = None
        self._connected = False
        self._standing = False

    # --- helpers ---
    def _rc_ok(self, rc) -> bool:
        """Interpret an action return code. 0x0 = success (doc §4.3)."""
        try:
            return int(rc) == RC_OK
        except Exception:
            return False

    def _ensure_standing(self) -> bool:
        """move() requires standing first (doc §4.2). Stand up if needed."""
        if self._standing:
            return True
        if self._app is None:
            return False
        try:
            rc = self._app.standUp()
            ok = self._rc_ok(rc)
            if ok:
                # Give the WBC controller ~2s to stabilize the stance (doc §7.2).
                time.sleep(2.0)
                self._standing = True
                log.info("standUp ok")
            else:
                log.warning("standUp returned 0x%x", int(rc) if rc is not None else -1)
            return ok
        except Exception:
            log.exception("standUp failed")
            return False

    # --- motion primitives ---
    def stand_up(self) -> bool:
        with self._lock:
            return self._ensure_standing()

    def lie_down(self) -> bool:
        with self._lock:
            if self._app is None:
                return False
            try:
                rc = self._app.lieDown()
                ok = self._rc_ok(rc)
                if ok:
                    self._standing = False
                    log.info("lieDown ok")
                return ok
            except Exception:
                log.exception("lieDown failed")
                return False

    def move(self, vx: float, vy: float, yaw_rate: float) -> bool:
        with self._lock:
            if self._app is None:
                return False
            if not self._ensure_standing():
                return False
            # Floor non-zero commands at the deadzone so small corrections
            # register. The doc says move() ignores |v| < 0.1.
            def _floor(v):
                if abs(v) < 1e-6:
                    return 0.0
                if abs(v) < MOVE_DEADZONE:
                    return MOVE_DEADZONE if v > 0 else -MOVE_DEADZONE
                return float(v)
            vx, vy, yaw_rate = _floor(vx), _floor(vy), _floor(yaw_rate)
            try:
                rc = self._app.move(vx, vy, yaw_rate)
                return self._rc_ok(rc)
            except Exception:
                log.exception("move failed")
                return False

    def stop(self) -> bool:
        """Hold position: move(0,0,0). Stays standing so the next goto is fast."""
        with self._lock:
            if self._app is None or not self._standing:
                return True
            try:
                rc = self._app.move(0.0, 0.0, 0.0)
                return self._rc_ok(rc)
            except Exception:
                log.exception("stop failed")
                return False

    def emergency_stop(self) -> bool:
        """passive() = motors to damping, robot drops (doc §7.5)."""
        with self._lock:
            if self._app is None:
                return False
            try:
                self._app.passive()
                self._standing = False
                log.warning("agibot emergency stop (passive)")
                return True
            except Exception:
                log.exception("passive failed")
                return False
