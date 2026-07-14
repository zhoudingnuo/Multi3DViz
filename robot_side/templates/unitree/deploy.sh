#!/usr/bin/env bash
# deploy.sh — one-shot deployment of m3v-agent onto a Unitree Go2.
#
# Run FROM the Go2 itself. Idempotent. See templates/agibot/deploy.sh for the
# Agibot variant (this script differs only in the SDK check + service name).
#
# Usage:
#   cd Multi3DViz/robot_side
#   ./templates/unitree/deploy.sh
set -euo pipefail

INSTALL_DIR="${M3V_INSTALL_DIR:-/opt/m3v-agent}"
SERVICE_NAME="m3v-agent-unitree"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"   # robot_side/

echo "=== m3v-agent Unitree deploy ==="
echo "  package : $PKG_DIR"
echo "  install : $INSTALL_DIR"
echo ""

# --- 1. dependency checks ---
echo "[1/5] checking dependencies..."
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found" >&2; exit 1
fi

# Source ROS2 if available (Go2 ships foxy/galaxy/humble depending on firmware).
if ! python3 -c "import rclpy" 2>/dev/null; then
    for d in /opt/ros/humble /opt/ros/foxy /opt/ros/galactic; do
        if [ -f "$d/setup.bash" ]; then
            echo "  sourcing $d/setup.bash"
            # shellcheck disable=SC1091
            source "$d/setup.bash"
            break
        fi
    done
fi
python3 -c "import rclpy" 2>/dev/null \
    && echo "  rclpy  : ok" \
    || echo "WARN: rclpy still not importable — recorder won't run" >&2

# unitree_sdk2py + cyclonedds (pip-installable).
if ! python3 -c "import unitree_sdk2py" 2>/dev/null; then
    echo "  installing unitree_sdk2py + cyclonedds..."
    python3 -m pip install -q unitree_sdk2py cyclonedds || {
        echo "WARN: unitree_sdk2py install failed — driver won't connect" >&2; }
else
    echo "  sdk   : unitree_sdk2py ok"
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
echo "  notes  : see docs/UNITREE_SDK_NOTES.md for CycloneDDS network iface setup"
