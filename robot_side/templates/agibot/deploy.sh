#!/usr/bin/env bash
# deploy.sh — one-shot deployment of m3v-agent onto an Agibot D1 (orin-001).
#
# Run FROM the robot itself (or modify to push from a dev machine). Idempotent.
#
# What it does:
#   1. Checks ROS, paramiko, the Agibot SDK .so.
#   2. pip installs this package in editable mode.
#   3. Copies the agibot config + systemd unit into place.
#   4. Enables + starts the systemd service.
#
# Usage:
#   cd Multi3DViz/robot_side
#   ./templates/agibot/deploy.sh
#
# Override the install root with M3V_INSTALL_DIR (default /opt/m3v-agent).
set -euo pipefail

INSTALL_DIR="${M3V_INSTALL_DIR:-/opt/m3v-agent}"
SERVICE_NAME="m3v-agent-agibot"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"   # robot_side/

echo "=== m3v-agent Agibot deploy ==="
echo "  package : $PKG_DIR"
echo "  install : $INSTALL_DIR"
echo ""

# --- 1. dependency checks ---
echo "[1/5] checking dependencies..."
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found" >&2; exit 1
fi
PYVER="$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
echo "  python : $PYVER"

# ROS (ros2 = rclpy). Source the setup if not already.
if ! python3 -c "import rclpy" 2>/dev/null; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "  sourcing /opt/ros/humble/setup.bash"
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    elif [ -f /opt/ros/noetic/setup.bash ]; then
        echo "  sourcing /opt/ros/noetic/setup.bash"
        # shellcheck disable=SC1091
        source /opt/ros/noetic/setup.bash
    else
        echo "WARN: rclpy not importable and no ROS setup.bash found" >&2
        echo "      install ROS first, or set recorder.ros accordingly" >&2
    fi
fi

# SDK .so for the Agibot. The user's ZCodeProject ships it.
SDK_LIB="/home/orin-001/ZCodeProject/lib/zsl-1/aarch64"
if [ -f "$SDK_LIB/mc_sdk_zsl_1_py.cpython-310-aarch64-linux-gnu.so" ] \
    || [ -f "$SDK_LIB/mc_sdk_zsl_1_py"*".so" ]; then
    echo "  sdk   : found in $SDK_LIB"
else
    echo "WARN: Agibot SDK .so not in $SDK_LIB" >&2
    echo "      set driver.agibot_sdk_lib_path in config.yaml to wherever it lives" >&2
fi

# Pure-Python deps.
python3 -m pip install -q -r "$PKG_DIR/requirements.txt" || {
    echo "ERROR: pip install failed" >&2; exit 1; }

# --- 2. install the package ---
echo "[2/5] installing m3v-agent package..."
sudo mkdir -p "$INSTALL_DIR"
sudo cp -r "$PKG_DIR/." "$INSTALL_DIR/"
cd "$INSTALL_DIR"
sudo python3 -m pip install -q -e . || {
    echo "ERROR: editable install failed" >&2; exit 1; }

# --- 3. config + systemd unit ---
echo "[3/5] installing config + systemd unit..."
sudo mkdir -p /etc/m3v-agent
sudo cp "$SCRIPT_DIR/config.yaml" /etc/m3v-agent/config.yaml
sudo cp "$SCRIPT_DIR/$SERVICE_NAME.service" /etc/systemd/system/

# --- 4. reload + enable ---
echo "[4/5] enabling systemd service..."
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME.service"

# --- 5. status ---
echo "[5/5] starting service..."
sudo systemctl restart "$SERVICE_NAME.service"
sleep 2
sudo systemctl status "$SERVICE_NAME.service" --no-pager -l || true

echo ""
echo "=== deploy complete ==="
echo "  config : /etc/m3v-agent/config.yaml  (edit host/user/password under transport)"
echo "  logs   : journalctl -u $SERVICE_NAME -f"
echo "  stop   : sudo systemctl stop $SERVICE_NAME"
echo "  restart: sudo systemctl restart $SERVICE_NAME"
