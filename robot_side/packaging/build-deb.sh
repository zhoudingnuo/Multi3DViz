#!/usr/bin/env bash
# build-deb.sh — build the m3v-agent .deb for Ubuntu arm64.
#
# RUN ON A LINUX MACHINE (the robot itself, or an arm64 CI runner). This
# script does NOT work on Windows — electron-builder needs a real Linux
# filesystem to assemble the deb payload + run dpkg-deb.
#
# Two build flavors:
#   --native    build for the machine you're on (default)
#   --arm64     cross-compile for arm64 (needs qemu-user-static + arm64 toolchain)
#
# Output: dist/m3v-agent_<version>_<arch>.deb
set -euo pipefail

cd "$(dirname "$0")/.."   # robot_side/

ARCH_FLAG=""
case "${1:---native}" in
    --arm64) ARCH_FLAG="--arm64" ;;
    --native) ARCH_FLAG="" ;;
    *) echo "usage: $0 [--native|--arm64]"; exit 1 ;;
esac

echo "=== building m3v-agent .deb ==="
echo "  arch flag: ${ARCH_FLAG:-(native)}"

# 1. Node deps (electron + electron-builder).
if [ ! -d node_modules ]; then
    echo "[1/3] installing node deps..."
    npm install
fi

# 2. Pre-flight: Python deps for the agent (the deb postinst will redo this on
#    the target, but we want a working agent here for any spec/test step).
echo "[2/3] checking python deps..."
python3 -m pip install -q -r requirements.txt 2>/dev/null || \
    echo "  (some pip deps skipped — postinst will install on target)"

# 3. electron-builder → deb.
echo "[3/3] electron-builder → deb..."
npx electron-builder --linux deb $ARCH_FLAG

echo ""
echo "=== done ==="
ls -lh dist/*.deb 2>/dev/null
echo ""
echo "install on the robot:"
echo "  sudo dpkg -i dist/m3v-agent_*.deb"
echo "  sudo apt-get install -f   # resolve any missing deps"
echo ""
echo "then either:"
echo "  - launch 'm3v-agent 受控端' from the desktop menu, OR"
echo "  - sudo systemctl enable --now m3v-agent.service   (headless)"
