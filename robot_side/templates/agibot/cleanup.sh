#!/usr/bin/env bash
# cleanup.sh — stop the Agibot ROS1 SLAM pipeline (runs INSIDE the container).
#
# Invoked by the control side via:
#   docker exec fastlio_noetic bash /opt/m3v-agent/cleanup.sh
#
# Kills the recorder first (so it flushes + stops SCP), then the SLAM stack.
# No `set -u` — ROS env vars may be unbound.
set -o pipefail

echo "[cleanup] stopping m3v_agent recorder..."
pkill -TERM -f "m3v_agent.*record" 2>/dev/null || true
sleep 1

echo "[cleanup] stopping FAST-LIO + livox driver..."
pkill -f fastlio_mapping 2>/dev/null || true
pkill -f livox_ros_driver2 2>/dev/null || true
pkill -f roslaunch 2>/dev/null || true
sleep 1

echo "[cleanup] stopping roscore / rosmaster..."
pkill -f rosmaster 2>/dev/null || true
pkill -f roscore 2>/dev/null || true
pkill -f rosout 2>/dev/null || true

# Ros1 leftover cleanup
pkill -f static_transform_publisher 2>/dev/null || true
rm -rf /tmp/ros2* /tmp/RMW* 2>/dev/null || true

echo "[cleanup] done. Remaining ROS processes (should be empty):"
pgrep -af "roscore|rosmaster|fastlio|livox|m3v_agent" 2>/dev/null || echo "  (none)"
