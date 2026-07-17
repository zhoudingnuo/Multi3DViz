#!/usr/bin/env bash
# deploy.sh — deploy m3v-agent onto an Agibot D1 (orin-001) for the SPLIT
# ROS1 deployment: recorder in a noetic container, executor on the host.
#
# Run FROM the robot itself (orin-001). Idempotent.
#
# What it does:
#   1. Installs m3v-agent onto the host (/opt/m3v-agent) for the EXECUTE agent
#      (mc_sdk direct, no ROS needed) + installs the pipeline/cleanup scripts.
#   2. Makes the same m3v-agent code available INSIDE the noetic container by
#      bind-mounting /opt/m3v-agent → container /scripts/m3v_agent (done via a
#      container-recreate step if the mount isn't present).
#   3. Copies config + the execute-only systemd unit, enables + starts it.
#
# The container-side recorder is NOT a systemd service — it is launched on
# demand by pipeline.sh (which the control side's "启动" button triggers via
# docker exec). This keeps the recorder lifecycle tied to the SLAM pipeline.
#
# Usage:
#   cd Multi3DViz/robot_side
#   ./templates/agibot/deploy.sh
set -euo pipefail

INSTALL_DIR="${M3V_INSTALL_DIR:-/opt/m3v-agent}"
EXEC_SERVICE="m3v-agent-agibot-exec"
CONTAINER="${M3V_CONTAINER:-fastlio_noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"   # robot_side/

echo "=== m3v-agent Agibot deploy (ROS1 split: container recorder + host executor) ==="
echo "  package  : $PKG_DIR"
echo "  install  : $INSTALL_DIR"
echo "  container: $CONTAINER"
echo ""

# --- 1. host-side dependency checks ---
echo "[1/6] checking host dependencies..."
if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 not found" >&2; exit 1
fi
PYVER="$(python3 -c 'import sys;print("%d.%d"%sys.version_info[:2])')"
echo "  python : $PYVER"

# The host EXECUTE agent needs the Agibot SDK .so (mc_sdk_zsl_1_py). It does NOT
# need ROS — pose comes from the odom_stream.jsonl file the recorder writes.
SDK_LIB="/home/orin-001/ZCodeProject/lib/zsl-1/aarch64"
if ls "$SDK_LIB"/mc_sdk_zsl_1_py*.so >/dev/null 2>&1; then
    echo "  sdk   : found in $SDK_LIB"
else
    echo "WARN: Agibot SDK .so not in $SDK_LIB" >&2
    echo "      set driver.agibot_sdk_lib_path in config.yaml" >&2
fi

# Pure-Python deps (numpy, paramiko for SCP, pyyaml, ...).
python3 -m pip install -q -r "$PKG_DIR/requirements.txt" || {
    echo "ERROR: pip install failed" >&2; exit 1; }

# --- 2. install m3v-agent on the host ---
echo "[2/6] installing m3v-agent on host..."
sudo mkdir -p "$INSTALL_DIR"
sudo cp -r "$PKG_DIR/." "$INSTALL_DIR/"
cd "$INSTALL_DIR"
sudo python3 -m pip install -q -e . || {
    echo "ERROR: editable install failed" >&2; exit 1; }

# --- 3. install pipeline + cleanup scripts (used inside the container) ---
echo "[3/6] installing pipeline scripts..."
sudo cp "$SCRIPT_DIR/pipeline.sh" "$INSTALL_DIR/pipeline.sh"
sudo cp "$SCRIPT_DIR/cleanup.sh" "$INSTALL_DIR/cleanup.sh"
sudo chmod +x "$INSTALL_DIR/pipeline.sh" "$INSTALL_DIR/cleanup.sh"

# --- 4. ensure the container can see m3v-agent + scripts ---
# The recorder runs INSIDE the container and imports m3v_agent (rospy). We
# expose the host install by bind-mounting /opt/m3v-agent → /scripts/m3v_agent.
# pipeline.sh also lives at /opt/m3v-agent and is exec'd via docker exec.
echo "[4/6] ensuring container $CONTAINER can access m3v-agent..."
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
    echo "WARN: container $CONTAINER not found — pipeline launch will fail." >&2
    echo "      Create it first (see agibotnav Docker setup)." >&2
else
    # Check if the bind mount for /opt/m3v-agent is already present.
    if ! docker inspect "$CONTAINER" --format '{{json .Mounts}}' \
        | grep -q "/opt/m3v-agent"; then
        echo "  adding bind mount /opt/m3v-agent → /scripts/m3v_agent ..."
        # Recreate the container with the extra mount, preserving image + the
        # other mounts/network. This is the least-invasive way to add a mount
        # to an existing container on hosts without docker-compose.
        IMG="$(docker inspect "$CONTAINER" --format '{{.Config.Image}}')"
        NET="$(docker inspect "$CONTAINER" --format '{{.HostConfig.NetworkMode}}')"
        MOUNTS="$(docker inspect "$CONTAINER" --format '{{range .Mounts}}-v {{.Source}}:{{.Destination}} {{end}}')-v /opt/m3v-agent:/scripts/m3v_agent"
        echo "  stopping + removing old container (image preserved)..."
        docker stop "$CONTAINER" >/dev/null 2>&1 || true
        docker rm "$CONTAINER" >/dev/null 2>&1 || true
        docker run -d --name "$CONTAINER" --network "$NET" --privileged \
            $MOUNTS "$IMG" bash >/dev/null
        echo "  recreated $CONTAINER with m3v-agent mounted."
    else
        echo "  bind mount already present."
    fi
    # Make sure it's running (pipeline.sh will docker exec into it).
    docker start "$CONTAINER" >/dev/null 2>&1 || true
fi

# --- 5. config + execute-only systemd unit ---
echo "[5/6] installing config + execute systemd unit..."
sudo mkdir -p /etc/m3v-agent
sudo cp "$SCRIPT_DIR/config.yaml" /etc/m3v-agent/config.yaml
sudo cp "$SCRIPT_DIR/$EXEC_SERVICE.service" /etc/systemd/system/

echo "[6/6] enabling + starting execute service..."
sudo systemctl daemon-reload
sudo systemctl enable "$EXEC_SERVICE.service"
sudo systemctl restart "$EXEC_SERVICE.service"
sleep 2
sudo systemctl status "$EXEC_SERVICE.service" --no-pager -l || true

echo ""
echo "=== deploy complete ==="
echo "  config        : /etc/m3v-agent/config.yaml  (edit transport.host/user)"
echo "  exec service  : journalctl -u $EXEC_SERVICE -f"
echo "  recorder      : launched on demand by the control-side 启动 button"
echo "                  (docker exec $CONTAINER bash /opt/m3v-agent/pipeline.sh)"
echo "  pipeline logs : docker exec $CONTAINER cat /tmp/m3v_pipeline/recorder.log"
echo ""
echo "  NOTE: edit /etc/m3v-agent/config.yaml → transport.host/user to point at"
echo "        your Windows control machine (for SCP push of .npy + odom), then"
echo "        'sudo systemctl restart $EXEC_SERVICE'."
