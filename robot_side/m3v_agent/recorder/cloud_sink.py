"""cloud_sink.py — FastlioRecorder.

Subscribes to FAST-LIO's /cloud_registered (PointCloud2) + /Odometry (Odometry)
and writes them to disk in ccenter format, exactly what Multi3DViz's
LocalReplaySource expects:

    <data_root>/<robot>/data/run_YYYYMMDD_HHMMSS/
        cloud_registered/000000.npy           float32 (N,3), atomic write
        Odometry/odom_stream.jsonl            one JSON per line (appended)
        gravity_calibration.json              optional, from IMU static window

Two ROS stacks are supported (auto-detected at start):
  - ros2 (rclpy): FAST-LIO running natively (e.g. Agibot native stack)
  - ros1 (rospy) : FAST-LIO in a noetic Docker container (e.g. Agibot ROS1 path)

PointCloud2 XYZ extraction is hand-rolled (not via sensor_msgs_py) so we don't
add a hard dep — the field layout is fixed: x@off[p], y@off[p+4], z@off[p+8]
float32 little-endian. We assume the standard livox/ouster MID360 layout
(point_step typically 16 or 32). If the cloud is organized as RGB XYZI we still
read the first 3 floats per point.

Thread model: the recorder runs ROS callbacks on the ROS spinner thread. The
actual disk writes are cheap (one np.save + one line append) and atomic, so we
do them inline rather than queueing — under 10 Hz that's <1ms per frame.
"""
from __future__ import annotations
import os
import sys
import time
import struct
import logging
import threading
from datetime import datetime

import numpy as np

from .atomic_io import save_npy_atomic, append_jsonl, ensure_dir
from .gravity_calib import GravityCalibrator

log = logging.getLogger("m3v_agent.recorder")


def _make_run_dir(data_root: str, robot: str) -> str:
    """Create a fresh run_YYYYMMDD_HHMMSS/ dir and its cloud/odom subdirs.

    Returns the absolute run dir path. `max()` sorting on the control side
    picks the newest, so the timestamp name must sort chronologically —
    zero-padded YYYYMMDD_HHMMSS does."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run = os.path.join(data_root, robot, "data", f"run_{ts}")
    ensure_dir(os.path.join(run, "cloud_registered"))
    ensure_dir(os.path.join(run, "Odometry"))
    return run


def _frame_name(idx: int, naming: str) -> str:
    """Build the .npy basename. `naming` ∈ {"index","indexed"}.

    Both schemes produce a 6-digit zero-padded prefix so sorted-glob ordering
    matches frame order (control side does sorted(glob('*.npy'))). The indexed
    variant adds a wall-clock suffix for human readability — control side
    doesn't parse it."""
    base = f"{idx:06d}"
    if naming == "indexed":
        now = datetime.now()
        return f"{base}_{now.strftime('%H%M%S')}_{now.microsecond // 1000:03d}.npy"
    return f"{base}.npy"


def _extract_xyz_from_pc2(msg) -> np.ndarray:
    """Pull XYZ out of a sensor_msgs/PointCloud2 message → (N,3) float32.

    Reads the first three float32 fields per point. Works for the standard
    livox MID360 layout (point_step 16-32, xyz at offset 0). Returns empty
    array if the message is malformed."""
    # msg.fields is a list of (name, offset, datatype, count) — but the ROS1
    # and ROS2 msg types differ slightly. We support both duck-typed shapes:
    #   ROS2 PointCloud2: msg.fields[i].name/.offset/.datatype
    #   ROS1 PointCloud2: msg.fields[i].name/.offset/.datatype
    # They're structurally identical for our purposes.
    n = msg.width * msg.height
    if n == 0 or not msg.point_step:
        return np.zeros((0, 3), dtype=np.float32)
    data = msg.data
    # ROS2 gives a bytes-like `array.array`; ROS1 gives a string/bytes. Normalize.
    if isinstance(data, str):
        data = data.encode("latin-1")
    data = bytes(data) if not isinstance(data, (bytes, bytearray)) else bytes(data)
    # Find x/y/z field offsets.
    offs = {}
    for f in msg.fields:
        name = getattr(f, "name", None) or f
        if name in ("x", "y", "z"):
            offs[name] = getattr(f, "offset", None)
    if not all(k in offs for k in ("x", "y", "z")):
        # Fall back to offsets 0/4/8 (most common xyz-first layout).
        offs = {"x": 0, "y": 4, "z": 8}
    step = msg.point_step
    # Slice each field out of every point in one vectorized pass per axis.
    # data is row-major: point p starts at p*step.
    arr = np.zeros((n, 3), dtype=np.float32)
    try:
        raw = np.frombuffer(data, dtype=np.uint8)
        # Trim to whole points only (ignore trailing padding).
        usable = (raw.size // step) * step
        raw = raw[:usable].reshape(-1, step)
        for col, key in enumerate(("x", "y", "z")):
            # Read 4 bytes little-endian float32 at the field offset.
            col_bytes = raw[:, offs[key]:offs[key] + 4]
            arr[:, col] = np.frombuffer(col_bytes.tobytes(), dtype="<f4")
    except Exception as e:
        log.warning("PointCloud2 parse failed (%s); saving empty cloud", e)
        return np.zeros((0, 3), dtype=np.float32)
    return arr


def _odom_to_dict(msg) -> dict:
    """Pull pose + stamp from a nav_msgs/Odometry message → odom dict.

    The control side reads x,y,z,qx,qy,qz,qw (data_utils.quat_to_mat /
    explorer_service). stamp/yaw/frame_id are extras kept for debugging —
    the control side ignores them."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Stamp: ROS2 has header.stamp.sec/nanosec; ROS1 has header.stamp.secs/nsecs.
    st = msg.header.stamp
    if hasattr(st, "sec"):
        stamp = float(st.sec) + float(getattr(st, "nanosec", 0)) * 1e-9
    else:
        stamp = float(getattr(st, "secs", 0)) + float(getattr(st, "nsecs", 0)) * 1e-9
    # Yaw from quaternion (informational; control side recomputes if needed).
    qw, qx, qy, qz = q.w, q.x, q.y, q.z
    yaw = float(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
    return {
        "stamp": round(stamp, 6),
        "frame_id": "camera_init",
        "x": float(p.x), "y": float(p.y), "z": float(p.z),
        "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w),
        "yaw": round(yaw, 6),
    }


class FastlioRecorder:
    """Subscribes to FAST-LIO topics and writes ccenter-format files.

    Lifecycle:
      start()  → opens a run dir, spins up the ROS node
      (ROS callbacks fire, writing .npy + odom lines)
      stop()   → finalizes gravity, shuts down ROS
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.run_dir: str = ""
        self.cloud_dir: str = ""
        self.odom_path: str = ""
        self.gravity: GravityCalibrator = None  # type: ignore[assignment]
        self._frame_idx = 0
        self._lock = threading.Lock()           # guards _frame_idx + writes
        self._ros: str = ""
        self._node = None
        self._spin_thread = None
        self._stop = threading.Event()
        # Latest pose cache — drivers (Agibot/Unitree) read this for navigation
        # so they don't depend on the SDK's unreliable state getters.
        self._latest_pose: dict = {}

    # --- lifecycle ---
    def start(self):
        """Open a run dir and start the ROS subscriber."""
        self.run_dir = _make_run_dir(self.cfg.data_root, self.cfg.robot)
        self.cloud_dir = os.path.join(self.run_dir, "cloud_registered")
        self.odom_path = os.path.join(self.run_dir, "Odometry", "odom_stream.jsonl")
        if self.cfg.gravity_enabled:
            self.gravity = GravityCalibrator(
                os.path.join(self.run_dir, "gravity_calibration.json"),
                n_samples=self.cfg.gravity_samples,
            )
        log.info("recorder started: run_dir=%s", self.run_dir)
        self._ros = (self.cfg.ros or "ros2").lower()
        if self._ros == "ros2":
            self._start_ros2()
        elif self._ros == "ros1":
            self._start_ros1()
        else:
            raise ValueError(f"unknown ros stack: {self.cfg.ros!r}")

    def stop(self):
        self._stop.set()
        if self.gravity is not None and not self.gravity.ready:
            self.gravity.finalize()
        try:
            if self._ros == "ros2":
                self._stop_ros2()
            elif self._ros == "ros1":
                self._stop_ros1()
        except Exception:
            log.exception("error stopping ROS")
        log.info("recorder stopped: wrote %d frames to %s", self._frame_idx, self.run_dir)

    # --- latest pose accessor (for drivers/navigator) ---
    def latest_pose(self) -> dict:
        """Return the most recent odom dict (x,y,z, qx,qy,qz,qw, yaw).

        Drivers use this instead of their SDK's state getter because both
        Agibot's getBatteryPower/ctrlmode and Unitree's DDS state need extra
        plumbing — odom is already being recorded and is authoritative for
        navigation."""
        with self._lock:
            return dict(self._latest_pose) if self._latest_pose else {}

    # --- ROS2 backend ---
    def _start_ros2(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        rclpy.init(args=None)
        node = Node("m3v_recorder")
        self._node = node
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        def on_cloud(msg):
            self._on_cloud(msg)

        def on_odom(msg):
            self._on_odom(msg)

        def on_imu(msg):
            if self.gravity is not None and not self.gravity.ready:
                a = msg.linear_acceleration
                self.gravity.feed_imu([a.x, a.y, a.z])

        node.create_subscription(self._pc2_type(), self.cfg.cloud_topic, on_cloud, qos)
        node.create_subscription(self._odom_type(), self.cfg.odom_topic, on_odom, qos)
        if self.cfg.gravity_enabled:
            node.create_subscription(self._imu_type(), self.cfg.imu_topic, on_imu, qos)

        self._spin_thread = threading.Thread(
            target=self._ros2_spin, name="ros2-spin", daemon=True)
        self._spin_thread.start()

    def _ros2_spin(self):
        import rclpy
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _stop_ros2(self):
        import rclpy
        if self._node is not None:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # --- ROS1 backend (rospy, e.g. noetic container) ---
    def _start_ros1(self):
        import rospy  # type: ignore
        from sensor_msgs.msg import PointCloud2, Imu
        from nav_msgs.msg import Odometry

        rospy.init_node("m3v_recorder", anonymous=True, disable_signals=True)
        rospy.Subscriber(self.cfg.cloud_topic, PointCloud2, self._on_cloud, queue_size=10)
        rospy.Subscriber(self.cfg.odom_topic, Odometry, self._on_odom, queue_size=10)
        if self.cfg.gravity_enabled:
            rospy.Subscriber(self.cfg.imu_topic, Imu,
                             lambda m: self.gravity.feed_imu(
                                 [m.linear_acceleration.x,
                                  m.linear_acceleration.y,
                                  m.linear_acceleration.z]),
                             queue_size=200)
        # rospy spins its own threads; no spin thread needed.
        self._node = rospy

    def _stop_ros1(self):
        import rospy  # type: ignore
        if rospy.core.is_shutdown():
            return
        rospy.signal_shutdown("m3v recorder stop")

    # --- type accessors (deferred imports so the module loads without ROS) ---
    @staticmethod
    def _pc2_type():
        from sensor_msgs.msg import PointCloud2
        return PointCloud2

    @staticmethod
    def _odom_type():
        from nav_msgs.msg import Odometry
        return Odometry

    @staticmethod
    def _imu_type():
        from sensor_msgs.msg import Imu
        return Imu

    # --- callbacks ---
    def _on_cloud(self, msg):
        """PointCloud2 → .npy. Called on the ROS spinner thread."""
        pts = _extract_xyz_from_pc2(msg)
        with self._lock:
            idx = self._frame_idx
            name = _frame_name(idx, self.cfg.naming)
            path = os.path.join(self.cloud_dir, name)
            try:
                save_npy_atomic(path, pts)
            except Exception:
                log.exception("failed to write cloud frame %d", idx)
                return
            self._frame_idx += 1
        if idx % 50 == 0:
            log.info("cloud frame %d: %d pts", idx, len(pts))

    def _on_odom(self, msg):
        """Odometry → odom_stream.jsonl line. Called on the ROS spinner thread."""
        d = _odom_to_dict(msg)
        with self._lock:
            try:
                append_jsonl(self.odom_path, d)
            except Exception:
                log.exception("failed to append odom line")
                return
            self._latest_pose = d
