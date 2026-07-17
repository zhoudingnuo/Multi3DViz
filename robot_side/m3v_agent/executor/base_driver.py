"""base_driver.py — Abstract robot driver interface.

Both the Agibot (mc_sdk_zsl_1_py, UDP) and Unitree (unitree_sdk2py, DDS)
drivers implement this interface so the navigator + target poller are
robot-agnostic. A FakeDriver (tests/test_fake_driver.py) also implements it
so the executor pipeline can be tested end-to-end without a real robot.

The interface is deliberately small and motion-focused. Pose comes from the
recorder's odom cache (BaseDriver.get_pose reads from there) rather than each
SDK's state getter — this sidesteps known SDK quirks (Agibot's ctrlmode
returns garbage, Unitree's DDS state needs extra plumbing) and keeps both
drivers behaving identically.
"""
from __future__ import annotations
import math
import logging
from abc import ABC, abstractmethod
from typing import Optional

log = logging.getLogger("m3v_agent.executor.driver")


class BaseDriver(ABC):
    """Motion-control interface shared by all robot drivers.

    Coordinate convention: all positions/velocities are in the robot's own
    odometry frame (camera_init at boot), units in meters / (m/s) / (rad/s).
    """

    # The recorder that supplies pose data. Set by RobotAgent after both are built.
    recorder = None
    # File-backed pose provider (OdomFilePoseProvider). Used as a fallback when
    # `recorder` is None — i.e. split-process deployment where the recorder
    # (rospy, in a noetic container) and the driver (host) run separately. The
    # provider tails the odom_stream.jsonl the recorder writes. Set by RobotAgent
    # when running in execute mode without a local recorder.
    odom_file_pose = None

    # --- lifecycle ---
    @abstractmethod
    def connect(self) -> bool:
        """Open the SDK connection. Return False on failure (non-fatal — the
        agent keeps retrying)."""

    @abstractmethod
    def disconnect(self):
        """Release the SDK connection. Idempotent."""

    # --- motion primitives ---
    @abstractmethod
    def stand_up(self) -> bool:
        """Transition to standing. Returns True if the state change succeeded."""

    @abstractmethod
    def lie_down(self) -> bool:
        """Transition to lying down (motors locked, safe to approach)."""

    @abstractmethod
    def move(self, vx: float, vy: float, yaw_rate: float) -> bool:
        """Continuous velocity command. vx forward, vy left, yaw_rate CCW.
        All zero = hold position. Returns True if accepted."""

    @abstractmethod
    def stop(self) -> bool:
        """Halt (equivalent to move(0,0,0)) but may also drop to a safer mode."""

    def emergency_stop(self) -> bool:
        """Aggressive stop — drop to passive/damping if the SDK supports it.
        Default delegates to stop()."""
        return self.stop()

    # --- pose (sourced from recorder odom cache, NOT the SDK) ---
    def get_pose(self) -> Optional[tuple]:
        """Return (x, y, yaw) in the robot odom frame, or None if no odom yet.

        Pose is read from the recorder's odom cache (same-process 'both' mode),
        keeping both drivers consistent — the Agibot SDK's getRPY works but
        ctrlmode is unreliable, and Unitree's DDS state needs extra plumbing.
        Odom is already being recorded and is the authoritative navigation
        reference.

        In split-process deployment (Agibot ROS1: recorder in a container,
        driver on the host) there is no recorder object; we fall back to
        OdomFilePoseProvider, which tails the odom_stream.jsonl the recorder
        writes to the shared filesystem."""
        if self.recorder is not None:
            pose = self.recorder.latest_pose()
            if pose:
                return (pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0))
        if self.odom_file_pose is not None:
            return self.odom_file_pose.latest_pose()
        return None

    # --- helpers shared by subclasses ---
    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap an angle to (-pi, pi]."""
        return math.atan2(math.sin(a), math.cos(a))
