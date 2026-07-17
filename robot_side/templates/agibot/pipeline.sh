#!/usr/bin/env bash
# pipeline.sh — Agibot ROS1 SLAM pipeline launcher (runs INSIDE the noetic container).
#
# Invoked by the control side (ssh_launcher) via:
#   docker exec fastlio_noetic bash /opt/m3v-agent/pipeline.sh
#
# Starts the full ROS1 stack + the m3v_agent recorder (rospy) which writes
# cloud_registered/*.npy + Odometry/odom_stream.jsonl and SCP-pushes them to the
# Windows control machine. Designed to be idempotent: re-running kills stale
# processes first.
#
# The container runs with --network host, so its roscore is reachable from the
# host's loopback (127.0.0.1:11311). The recorder and FAST-LIO share that master.
# NOTE: do NOT use `set -u` — the ROS setup.bash / roslaunch scripts reference
# many vars (ROS_MASTER_URI etc.) that may be unbound, and -u makes that fatal.
set -o pipefail

# --- ROS1 workspace overlay (FAST-LIO + livox_ros_driver2, already built) ---
if [ -f /fastlio_ws/devel/setup.bash ]; then
    # shellcheck disable=SC1091
    source /fastlio_ws/devel/setup.bash
else
    echo "[pipeline] ERROR: /fastlio_ws/devel/setup.bash not found — workspace not built" >&2
    exit 1
fi

# --- m3v_agent on PYTHONPATH (bind-mounted from host /opt/m3v-agent) ---
export PYTHONPATH="/scripts/m3v_agent:${PYTHONPATH:-}"

LOG_DIR="/tmp/m3v_pipeline"
mkdir -p "$LOG_DIR"

echo "[pipeline] cleaning up any stale processes..."
pkill -f fastlio_mapping 2>/dev/null || true
pkill -f livox_ros_driver2 2>/dev/null || true
pkill -f "roscore" 2>/dev/null || true
pkill -f "rosmaster" 2>/dev/null || true
pkill -f "m3v_agent.*record" 2>/dev/null || true
sleep 2

# --- 1. roscore (the ROS1 master) ---
echo "[pipeline] starting roscore..."
roscore > "$LOG_DIR/roscore.log" 2>&1 &
sleep 4
if ! rostopic list >/dev/null 2>&1; then
    echo "[pipeline] ERROR: roscore did not come up (rostopic list failed)" >&2
    exit 1
fi

# --- 2. Livox MID360 driver (publishes /livox/lidar + /livox/imu) ---
echo "[pipeline] starting livox_ros_driver2 (MID360)..."
roslaunch livox_ros_driver2 msg_MID360.launch \
    rviz_enable:=false rosbag_enable:=false \
    > "$LOG_DIR/livox.log" 2>&1 &
sleep 5

# --- 3. FAST-LIO mapping (publishes /cloud_registered + /Odometry) ---
echo "[pipeline] starting FAST-LIO mapping_mid360..."
roslaunch fast_lio mapping_mid360.launch rviz:=false \
    > "$LOG_DIR/fastlio.log" 2>&1 &
sleep 4

# --- 4. m3v_agent recorder (rospy) — writes .npy + odom + SCP push ---
# record-only mode: this process does NOT drive the robot (the host-side
# execute agent does). It just captures + ships the data.
# Config: the host's ~/m3v-agent is mounted at /scripts/m3v_agent, so the config
# lives at /scripts/m3v_agent/config.yaml (written there by the deploy script).
REC_CFG="/scripts/m3v_agent/config.yaml"
echo "[pipeline] starting m3v_agent recorder (record mode), config=$REC_CFG ..."
# -u = unbuffered so log lines flush to recorder.log immediately (otherwise
# they sit in the stdout buffer until the process exits and we see no output).
PYTHONUNBUFFERED=1 python3 -u -m m3v_agent.agent \
    --config "$REC_CFG" \
    --mode record \
    > "$LOG_DIR/recorder.log" 2>&1 &
RECORDER_PID=$!
echo "[pipeline] recorder PID=$RECORDER_PID"

# --- verify data is flowing (best-effort, non-fatal) ---
sleep 6
if rostopic hz /cloud_registered 2>&1 | head -1 | grep -q "average rate"; then
    echo "[pipeline] OK: /cloud_registered is publishing"
else
    echo "[pipeline] WARNING: /cloud_registered not yet publishing (check $LOG_DIR/fastlio.log)"
fi

echo "[pipeline] all nodes up. Logs in $LOG_DIR/. Waiting on recorder..."
echo "[pipeline] (Ctrl+C or cleanup.sh to stop everything)"
wait "$RECORDER_PID"
