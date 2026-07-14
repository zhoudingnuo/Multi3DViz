"""m3v_agent — Multi3DViz 受控端 (robot-side) agent.

Runs on each robot (Ubuntu ARM64). Three jobs, all driven by the control-side
(Multi3DViz on Windows) data contract documented in docs/DATA_CONTRACT.md:

  1. Recorder  — subscribe FAST-LIO /cloud_registered + /Odometry,
                 write ccenter-format .npy + odom_stream.jsonl, SCP-push to Windows.
  2. Transport — SCP pusher daemon (robot → Windows, since Windows is the sink).
  3. Executor  — poll ccenter_target_*.txt (SSH-written by control side),
                 parse local_x/local_y, drive the robot there via a driver template.

Two driver templates: agibot (mc_sdk_zsl_1_py, UDP) and unitree (unitree_sdk2py, DDS).
Entry point: python -m m3v_agent.agent --config <yaml> [--mode record|execute|both]
"""
__version__ = "0.1.0"
