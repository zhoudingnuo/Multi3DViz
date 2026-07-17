"""config.py — Robot-side configuration.

One YAML file (templates/<robot>/config.yaml) drives everything. Fields can be
overridden by environment variables (M3V_<UPPER_FIELD>) so systemd unit files
can inject secrets without editing the file.

The config binds three contracts together:
  - control-side data layout   (<robot>/data/run_*/...)  — recorder writes here
  - control-side target file   (ccenter_target_*.txt)    — executor reads here
  - SDK connection             (host/port/iface)         — driver uses this
"""
from __future__ import annotations
import os
import dataclasses
from dataclasses import dataclass, field
from typing import Optional

try:
    import yaml  # PyYAML
except ImportError:  # pragma: no cover — yaml is in requirements.txt
    yaml = None


@dataclass
class RecorderCfg:
    """What to subscribe to and where to write ccenter-format data."""
    enabled: bool = True
    # ROS stack: "ros2" (rclpy, FAST-LIO native) or "ros1" (rospy, in noetic container).
    ros: str = "ros2"
    cloud_topic: str = "/cloud_registered"          # sensor_msgs/PointCloud2
    odom_topic: str = "/Odometry"                   # nav_msgs/Odometry
    imu_topic: str = "/livox/imu"                   # for gravity calibration
    # Local data root. Recorder writes <data_root>/<robot>/data/run_<ts>/...
    # This mirrors exactly what the control side's LocalReplaySource scans.
    data_root: str = "/home/unitree/m3v_data"
    robot: str = "unitree"                          # subdir name under data_root
    # Filename pattern for .npy frames. "index"   -> 000000.npy
    #                                   "indexed" -> 000000_HHMMSS_mmm.npy (Unitree convention)
    naming: str = "index"
    # Gravity calibration: collect this many IMU samples at startup while the
    # robot is still, compute roll_deg/pitch_deg, write gravity_calibration.json.
    gravity_samples: int = 500
    gravity_enabled: bool = True


@dataclass
class TransportCfg:
    """SCP push daemon (robot → Windows). Windows must run OpenSSH Server."""
    enabled: bool = True
    # Windows SSH endpoint (the control-side machine).
    host: str = "192.168.1.10"
    port: int = 22
    user: str = "Z790"
    password: Optional[str] = None        # None → SSH key auth
    # Remote data root on Windows. The recorder's local <data_root>/<robot>/data
    # maps to <remote_root>/<robot>/data on the Windows side — this is exactly
    # what Multi3DViz's LocalReplaySource.data_root points at.
    remote_root: str = r"C:\Users\Z790\ccenter"
    # Push loop cadence (seconds). New .npy frames discovered since last push
    # are uploaded; odom_stream.jsonl is re-uploaded whole each cycle.
    interval: float = 1.0
    # Use atomic rename on the remote (needs OpenSSH >= 8.0 for posix-rename).
    atomic_remote: bool = True
    # Delete local .npy frames after they are confirmed uploaded. This keeps
    # the robot's disk from filling up during long recording sessions (each
    # frame is ~70KB at 10Hz = ~7MB/min = ~400MB/hour). The odom_stream.jsonl
    # is kept (it's tiny + the host-side execute agent reads pose from it).
    # The gravity file is also kept (uploaded once, needed for reloads).
    delete_after_push: bool = True


@dataclass
class ExecutorCfg:
    """Target-file polling + navigation."""
    enabled: bool = True
    # Local path the control side SSH-writes the target into.
    #   Unitree A: /home/unitree/sda2/online/ccenter_target_a.txt
    #   Agibot  B: /home/orin-001/ccenter_target_b.txt
    target_path: str = "/home/unitree/sda2/online/ccenter_target_a.txt"
    poll_interval: float = 0.5           # seconds between target file reads
    # If the target file mtime is older than this, assume control side is dead
    # and halt the robot (safety).
    stale_timeout: float = 10.0
    # Navigation P-gains + limits.
    arrive_threshold: float = 0.3        # meters — matches explorer.py:118 reached()
    max_fwd: float = 0.5                 # m/s
    max_turn: float = 1.0                # rad/s
    turn_kp: float = 1.2
    # Don't re-issue goto if target moved less than this (avoids jitter).
    target_deadband: float = 0.1         # meters


@dataclass
class DriverCfg:
    """Which robot SDK template to use + its connection params."""
    kind: str = "unitree"                # "unitree" | "agibot" | "fake"
    # --- Agibot (mc_sdk_zsl_1_py, UDP) ---
    agibot_local_ip: str = "192.168.234.18"
    agibot_local_port: int = 43988
    agibot_robot_ip: str = "192.168.234.1"
    agibot_sdk_lib_path: str = "/home/orin-001/ZCodeProject/lib/zsl-1/aarch64"
    # --- Unitree (unitree_sdk2py, DDS) ---
    unitree_network_iface: str = "eth0"  # network interface for CycloneDDS
    unitree_protocol: str = ""           # "" lets SDK pick (_unset picks UDP)


@dataclass
class WebCfg:
    """Built-in status panel (ZCode-style). Served on the robot itself so the
    control-side browser can monitor recorder/transport/executor state and
    trigger an emergency stop. stdlib http.server — no extra dependency."""
    enabled: bool = True
    host: str = "0.0.0.0"               # bind all interfaces (LAN-reachable)
    port: int = 8765


@dataclass
class RobotSideConfig:
    recorder: RecorderCfg = field(default_factory=RecorderCfg)
    transport: TransportCfg = field(default_factory=TransportCfg)
    executor: ExecutorCfg = field(default_factory=ExecutorCfg)
    driver: DriverCfg = field(default_factory=DriverCfg)
    web: WebCfg = field(default_factory=WebCfg)
    # Which subsystems to run. "both" = recorder + transport + executor.
    mode: str = "both"                   # "record" | "execute" | "both"
    log_level: str = "INFO"

    # --- load/save ---
    @classmethod
    def from_yaml(cls, path: str) -> "RobotSideConfig":
        if yaml is None:
            raise RuntimeError("PyYAML not installed; pip install pyyaml")
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        return cls.from_dict(raw)

    @classmethod
    def from_dict(cls, raw: dict) -> "RobotSideConfig":
        """Build from a nested dict (parsed YAML). Unknown keys ignored."""
        def _sub(cls_, data):
            kw = {f.name: data[f.name] for f in dataclasses.fields(cls_)
                  if f.name in data}
            return cls_(**kw)
        return cls(
            recorder=_sub(RecorderCfg, raw.get("recorder", {})),
            transport=_sub(TransportCfg, raw.get("transport", {})),
            executor=_sub(ExecutorCfg, raw.get("executor", {})),
            driver=_sub(DriverCfg, raw.get("driver", {})),
            web=_sub(WebCfg, raw.get("web", {})),
            mode=raw.get("mode", "both"),
            log_level=raw.get("log_level", "INFO"),
        )

    def apply_env_overrides(self):
        """M3V_<SECTION>_<FIELD> overrides (e.g. M3V_TRANSPORT_PASSWORD).

        Flat walk over every dataclass field in every sub-config so new fields
        are picked up automatically."""
        for section in (self.recorder, self.transport, self.executor,
                        self.driver, self.web):
            for f in dataclasses.fields(section):
                key = f"M3V_{type(section).__name__.replace('Cfg','').upper()}_{f.name.upper()}"
                if key in os.environ:
                    val = os.environ[key]
                    # cast to the declared type
                    if f.type is bool or f.type == "bool":
                        val = val.lower() in ("1", "true", "yes", "on")
                    elif f.type is int or f.type == "int":
                        val = int(val)
                    elif f.type is float or f.type == "float":
                        val = float(val)
                    setattr(section, f.name, val)
        # Top-level overrides
        if "M3V_MODE" in os.environ:
            self.mode = os.environ["M3V_MODE"]
        if "M3V_LOG_LEVEL" in os.environ:
            self.log_level = os.environ["M3V_LOG_LEVEL"]
