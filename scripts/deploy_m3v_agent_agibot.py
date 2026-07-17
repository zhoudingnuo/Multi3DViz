#!/usr/bin/env python3
"""Deploy m3v-agent to the Agibot D1 from the Windows control side.

"本端去唤起机器人端" — run this on Windows; it pushes the full split-ROS1
deployment to the robot via SSH/SFTP and configures everything.

This runs ENTIRELY as the login user (no sudo needed):
  - Install dir : ~/m3v-agent            (user-writable)
  - Config dir  : ~/.config/m3v-agent    (user-writable)
  - Service     : systemctl --user       (no root; user units)
  - Docker      : user is in 'docker' group (no sudo for container ops)

Steps:
  1. Push robot_side/ → ~/m3v-agent.
  2. Push pipeline.sh / cleanup.sh → ~/m3v-agent (run inside the container).
  3. Push config.yaml → ~/.config/m3v-agent/config.yaml (transport.host = this
     Windows machine's IP, so SCP-push of .npy + odom lands back here).
  4. pip install --user the package deps (numpy, paramiko, ...).
  5. Recreate the noetic container (fastlio_noetic) with ~/m3v-agent mounted at
     /scripts/m3v_agent so the recorder (rospy) can import m3v_agent.
  6. Install + enable + start the user systemd execute service. Enable linger
     so the service survives logout / runs at boot.
  7. Verify: import checks, container reachability.

The control-side "启动" button then triggers pipeline.sh via docker exec
(see ssh_launcher.py), which starts roscore + livox + FAST-LIO + the recorder.

Usage:
    python scripts/deploy_m3v_agent_agibot.py [host] [user]
Defaults: host=10.60.77.154 user=orin-001 (key auth)
"""
from __future__ import annotations
import os
import sys
import shlex
import time

import paramiko

HOST = sys.argv[1] if len(sys.argv) > 1 else "10.60.77.154"
USER = sys.argv[2] if len(sys.argv) > 2 else "orin-001"
WIN_HOST = os.environ.get("WIN_HOST", "")
WIN_USER = os.environ.get("WIN_USER", "Z790")
REMOTE_ROOT = "C:/Users/Z790/ccenter"

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ROBOT_SIDE = os.path.join(REPO, "robot_side")
TMPL = os.path.join(ROBOT_SIDE, "templates", "agibot")

# User-space install paths (no sudo).
INSTALL_DIR = "$HOME/m3v-agent"          # ~/m3v-agent
CONFIG_DIR = "$HOME/.config/m3v-agent"   # ~/.config/m3v-agent
EXEC_SERVICE = "m3v-agent-agibot-exec"
CONTAINER = "fastlio_noetic"


def run(c, cmd, timeout=60):
    """Run a command over SSH (no sudo — user-level). Return (rc, out)."""
    cmd = f"bash -lc {shlex.quote(cmd)}"
    _, stdout, stderr = c.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", "replace")
    err = stderr.read().decode("utf-8", "replace")
    rc = stdout.channel.recv_exit_status()
    return rc, out + (f"\n[stderr] {err}" if err.strip() else "")


def push_tree_sftp(c, local_dir, remote_dir):
    """Recursively push a local directory tree to remote_dir via SFTP."""
    sftp = c.open_sftp()
    count = 0
    for root, dirs, files in os.walk(local_dir):
        dirs[:] = [d for d in dirs if d not in
                   ("__pycache__", ".git", "build", "dist", "m3v_agent.egg-info")]
        rel = os.path.relpath(root, local_dir)
        rdir = remote_dir if rel == "." else remote_dir + "/" + rel.replace("\\", "/")
        try:
            sftp.stat(rdir)
        except IOError:
            # mkdir -p style
            parts = rdir.split("/")
            for i in range(1, len(parts) + 1):
                p = "/".join(parts[:i])
                try:
                    sftp.mkdir(p)
                except IOError:
                    pass
        for fn in files:
            if fn.endswith((".pyc", ".swp")) or ".egg-info" in fn:
                continue
            sftp.put(os.path.join(root, fn), rdir + "/" + fn)
            count += 1
    sftp.close()
    return count


def main():
    win_ip = detect_win_ip()
    print(f"=== m3v-agent Agibot deploy (user-space, no sudo) ===")
    print(f"  robot   : {USER}@{HOST}")
    print(f"  win ip  : {win_ip or '(undetected — set WIN_HOST env)'}")
    print(f"  install : {INSTALL_DIR}")
    print()

    print(f"[connect] {USER}@{HOST} ...")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        c.connect(HOST, username=USER, timeout=12)
    except Exception as e:
        print(f"[connect] FAILED: {e}")
        print("         Is the robot online? Check it's powered + on the LAN.")
        return 1

    # Resolve $HOME so SFTP (which doesn't expand vars) writes to absolute paths.
    rc, home = run(c, "echo $HOME")
    home = home.strip()
    inst = f"{home}/m3v-agent"
    cfgdir = f"{home}/.config/m3v-agent"
    print(f"[resolved] home={home} install={inst}")

    # --- 1. push robot_side/ → ~/m3v-agent ---
    print(f"[1/7] pushing robot_side/ → ~/m3v-agent ...")
    run(c, f"mkdir -p {inst}")
    n = push_tree_sftp(c, ROBOT_SIDE, inst)
    print(f"      wrote {n} files")

    # --- 2. push pipeline.sh + cleanup.sh ---
    print("[2/7] pushing pipeline.sh + cleanup.sh ...")
    sftp = c.open_sftp()
    for fn in ("pipeline.sh", "cleanup.sh"):
        sftp.put(os.path.join(TMPL, fn), f"{inst}/{fn}")
        sftp.chmod(f"{inst}/{fn}", 0o755)
    sftp.close()

    # --- 3. push config.yaml with transport.host = this machine ---
    print("[3/7] pushing config.yaml (transport.host = this Windows machine) ...")
    cfg = open(os.path.join(TMPL, "config.yaml"), "r", encoding="utf-8").read()
    if win_ip:
        cfg = cfg.replace("host: 192.168.1.10", f"host: {win_ip}")
        cfg = cfg.replace("user: Z790", f"user: {WIN_USER}")
    # Also point data_root + config at the user-space install.
    cfg = cfg.replace("data_root: /home/orin-001/m3v_data", f"data_root: {home}/m3v_data")
    sftp = c.open_sftp()
    run(c, f"mkdir -p {cfgdir}")
    # Write config in TWO places:
    #  1. ~/.config/m3v-agent/config.yaml  — host execute agent reads this.
    #  2. ~/m3v-agent/config.yaml          — container recorder reads this
    #     (~/m3v-agent is mounted at /scripts/m3v_agent inside the container).
    for path in (f"{cfgdir}/config.yaml", f"{inst}/config.yaml"):
        with sftp.file(path, "w") as f:
            f.write(cfg)
    sftp.close()
    rc, out = run(c, f"grep -A5 'transport:' {cfgdir}/config.yaml")
    print("      " + "\n      ".join(out.strip().splitlines()[:6]))

    # --- 4. pip install deps ---
    print("[4/7] pip install --user (numpy, paramiko, ...) ...")
    rc, out = run(c, f"cd {inst} && python3 -m pip install --user -q -r requirements.txt 2>&1 | tail -3", timeout=180)
    print("      " + out.strip().replace("\n", "\n      "))

    # --- 5. recreate container with m3v-agent mounted ---
    print(f"[5/7] ensuring container {CONTAINER} can import m3v_agent ...")
    rc, out = run(c, f"docker inspect {CONTAINER} >/dev/null 2>&1 && echo EXISTS || echo MISSING")
    if "MISSING" in out:
        print(f"      WARNING: container {CONTAINER} not found — recorder launch will fail.")
    else:
        rc, mounts = run(c, f"docker inspect {CONTAINER} --format '{{{{json .Mounts}}}}'")
        if inst in mounts:
            print("      bind mount already present in container.")
        else:
            print(f"      recreating {CONTAINER} with {inst} → /scripts/m3v_agent ...")
            script = f"""
IMG=$(docker inspect {CONTAINER} --format '{{{{.Config.Image}}}}')
NET=$(docker inspect {CONTAINER} --format '{{{{.HostConfig.NetworkMode}}}}')
MOUNTS=$(docker inspect {CONTAINER} --format '{{{{range .Mounts}}}}-v {{{{.Source}}}}:{{{{.Destination}}}} {{{{end}}}}')
docker stop {CONTAINER} >/dev/null 2>&1 || true
docker rm {CONTAINER} >/dev/null 2>&1 || true
# Ensure the data dir exists on the host (the recorder writes here; the host
# execute agent reads odom back from it; SCP pushes from here to Windows).
mkdir -p {home}/m3v_data
# sleep infinity keeps the container alive so docker exec can run pipeline.sh
# in it (plain bash exits immediately with no tty).
#
# THREE mounts are critical for the split deployment:
#   1. {inst}:/scripts/m3v_agent  — recorder imports m3v_agent + reads config
#   2. {home}/m3v_data:...        — recorder writes here, host reads it back
#      (without this the data lives in the container's layer, invisible to host)
#   3. {home}/.ssh:/host_ssh:ro   — the recorder's ScpPusher SSHes data back to
#      the Windows control machine. The container runs as root with no keys of
#      its own, so we copy the host user's keys into /root/.ssh at startup with
#      correct ownership/perms (paramiko's look_for_keys needs 600 on the key).
docker run -d --name {CONTAINER} --network "$NET" --privileged \
    $MOUNTS -v {inst}:/scripts/m3v_agent -v {home}/m3v_data:{home}/m3v_data \
    -v {home}/.ssh:/host_ssh:ro \
    "$IMG" bash -c "cp -r /host_ssh /root/.ssh && chmod 700 /root/.ssh && chmod 600 /root/.ssh/id_* 2>/dev/null; chmod 644 /root/.ssh/*.pub /root/.ssh/config 2>/dev/null; sleep infinity" >/dev/null
echo RECREATED
"""
            rc, out = run(c, script.strip(), timeout=90)
            print("      " + out.strip().replace("\n", "\n      "))
        run(c, f"docker start {CONTAINER} >/dev/null 2>&1 || true")
        rc, out = run(c, f"docker exec {CONTAINER} bash -lc "
                     f"'PYTHONPATH=/scripts/m3v_agent python3 -c "
                     f"\"import m3v_agent; print(\\\"container import OK\\\")\"' 2>&1")
        last = out.strip().splitlines()[-1] if out.strip() else "(no output)"
        print(f"      container import check: {last}")

    # --- 6. user systemd unit (no sudo) ---
    print(f"[6/7] installing + enabling user service {EXEC_SERVICE} ...")
    # Render the unit with the resolved paths (it references INSTALL_DIR + cfg).
    unit_tmpl = open(os.path.join(TMPL, f"{EXEC_SERVICE}.service"), "r", encoding="utf-8").read()
    unit = (unit_tmpl
            .replace("WorkingDirectory=/opt/m3v-agent", f"WorkingDirectory={inst}")
            .replace("cd /opt/m3v-agent", f"cd {inst}")
            .replace("/etc/m3v-agent/config.yaml", f"{cfgdir}/config.yaml")
            .replace("User=root", "# user-unit runs as the login user"))
    unit_dir = f"{home}/.config/systemd/user"
    run(c, f"mkdir -p {unit_dir}")
    sftp = c.open_sftp()
    with sftp.file(f"{unit_dir}/{EXEC_SERVICE}.service", "w") as f:
        f.write(unit)
    sftp.close()
    run(c, "systemctl --user daemon-reload")
    run(c, f"systemctl --user enable {EXEC_SERVICE}.service 2>&1 | tail -1")
    # Linger so the user service runs at boot / survives logout.
    rc, out = run(c, "loginctl show-user $USER 2>/dev/null | grep Linger || true")
    if "Linger=no" in out or not out.strip():
        print("      enabling linger (needs sudo once) ...")
        rc, out = run(c, f"sudo -n loginctl enable-linger {USER} 2>&1")
        if "password" in out.lower() or rc != 0:
            print(f"      NOTE: linger needs sudo. Run once on the robot:")
            print(f"            sudo loginctl enable-linger {USER}")
    run(c, f"systemctl --user restart {EXEC_SERVICE}.service")
    time.sleep(2)
    rc, out = run(c, f"systemctl --user is-active {EXEC_SERVICE}.service")
    print(f"      service status: {out.strip()}")

    # --- 7. verify ---
    print("[7/7] verification ...")
    rc, out = run(c, f"cd {inst} && python3 -c 'import numpy, paramiko, yaml; print(\"host deps OK\")' 2>&1")
    print("      host deps: " + (out.strip().splitlines()[-1] if out.strip() else "(no output)"))
    rc, out = run(c, "python3 -c 'import sys; sys.path.insert(0,\"/home/orin-001/ZCodeProject/lib/zsl-1/aarch64\"); import mc_sdk_zsl_1_py; print(\"mc_sdk OK\")' 2>&1")
    print("      mc_sdk   : " + (out.strip().splitlines()[-1] if out.strip() else "(no output)"))
    rc, out = run(c, f"ls {inst}/pipeline.sh {inst}/cleanup.sh 2>&1")
    print("      scripts  : " + ("present" if "No such" not in out else "MISSING"))

    c.close()
    print()
    print("=== deploy complete ===")
    print(f"  启动 button → docker exec {CONTAINER} bash {inst}/pipeline.sh")
    print(f"  Data → SCP to {win_ip or '<win-ip>'} → {REMOTE_ROOT}/agibot/data/")
    print(f"  Exec logs   : journalctl --user -u {EXEC_SERVICE} -f")
    print(f"  Recorder log: docker exec {CONTAINER} cat /tmp/m3v_pipeline/recorder.log")
    return 0


def detect_win_ip():
    global WIN_HOST
    if WIN_HOST:
        return WIN_HOST
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((HOST, 22))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return ""


if __name__ == "__main__":
    sys.exit(main())
