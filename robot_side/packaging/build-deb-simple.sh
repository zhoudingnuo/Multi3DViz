#!/usr/bin/env bash
# build-deb-simple.sh — assemble the m3v-agent .deb WITHOUT electron-builder.
#
# Why a second script: electron-builder needs to download the arm64 Electron
# runtime (hundreds of MB) which is painfully slow on the Tegra. This script
# uses plain `dpkg-deb` to assemble the payload from files already on disk —
# no network, ~2 seconds. The Electron desktop shell is packaged separately
# (or run via `npm start` in dev mode on the robot).
#
# Run ON the robot (arm64 Ubuntu):
#   cd /home/unitree/m3v_robot_side
#   ./packaging/build-deb-simple.sh
#
# Output: dist/m3v-agent_<version>_arm64.deb
set -euo pipefail

cd "$(dirname "$0")/.."   # robot_side/
VERSION="0.1.0"
ARCH="arm64"
PKGNAME="m3v-agent"
BUILDROOT="$(mktemp -d)"
DIST="dist"
mkdir -p "$DIST"

# --- payload layout ---
OPT="$BUILDROOT/opt/$PKGNAME"
ETC="$BUILDROOT/etc/$PKGNAME"
SYSTEMD="$BUILDROOT/lib/systemd/system"
APPS="$BUILDROOT/usr/share/applications"
ICONS="$BUILDROOT/usr/share/icons/hicolor/256x256/apps"

mkdir -p "$OPT" "$ETC" "$SYSTEMD" "$APPS" "$ICONS" "$BUILDROOT/DEBIAN"

# 1. Python package + templates + requirements
cp -r m3v_agent "$OPT/"
cp -r templates "$OPT/"
cp requirements.txt pyproject.toml "$OPT/"
cp -r packaging "$OPT/packaging"

# 2. Default config (unitree is the common case for this robot)
cp templates/unitree/config.yaml "$ETC/config.yaml"

# 3. systemd unit (headless mode)
sed "s|/opt/m3v-agent|/opt/$PKGNAME|g" packaging/deb/m3v-agent.service > "$SYSTEMD/m3v-agent.service"

# 4. Desktop menu entry (launches the dev-mode Electron shell for now; once
#    the arm64 Electron deb is built separately, this points at its binary).
cat > "$APPS/$PKGNAME.desktop" <<EOF
[Desktop Entry]
Name=m3v-agent 受控端
Comment=Multi3DViz robot-side agent status shell
Exec=bash -c 'cd /opt/$PKGNAME && python3 -m m3v_agent.agent --ui-stdio -c /etc/$PKGNAME/config.yaml'
Icon=$PKGNAME
Terminal=true
Type=Application
Categories=Science;Robotics;
EOF

# 5. control file
INSTALLED_SIZE="$(du -sk "$OPT" | cut -f1)"
cat > "$BUILDROOT/DEBIAN/control" <<EOF
Package: $PKGNAME
Version: $VERSION
Architecture: $ARCH
Maintainer: Multi3DViz <noreply@multi3dviz>
Installed-Size: $INSTALLED_SIZE
Depends: python3 (>= 3.8), python3-numpy, python3-yaml
Recommends: python3-scipy, python3-paramiko
Section: science
Priority: optional
Homepage: https://github.com/multi3dviz
Description: Multi3DViz robot-side agent
 Records FAST-LIO data, pushes to the Windows control side via SCP, reads
 navigation targets written by the control side, and drives the robot via a
 per-platform driver (Unitree TCP bridge / Agibot SDK). Supports a headless
 systemd mode and a desktop shell (--ui-stdio).
EOF

# 6. maintainer scripts (chmod +x; postinst installs pip deps + sets up menu)
cp packaging/deb/postinst "$BUILDROOT/DEBIAN/postinst"
cp packaging/deb/prerm    "$BUILDROOT/DEBIAN/prerm"
chmod 0755 "$BUILDROOT/DEBIAN/postinst" "$BUILDROOT/DEBIAN/prerm"

# 7. build the deb
OUT="$DIST/${PKGNAME}_${VERSION}_${ARCH}.deb"
dpkg-deb --build --root-owner-group "$BUILDROOT" "$OUT"
rm -rf "$BUILDROOT"

echo ""
echo "=== built ==="
ls -lh "$OUT"
echo ""
echo "install:  sudo dpkg -i $OUT  &&  sudo apt-get install -f"
echo "service:  sudo systemctl enable --now m3v-agent.service"
echo "config:   /etc/$PKGNAME/config.yaml"
