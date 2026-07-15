#!/usr/bin/env python3
"""Deploy m3v_agibot.py to the Agibot D1 via SFTP (raw bytes — no shell mangling).

m3v_agibot.py is the Agibot counterpart to m3v_move on the Unitree: a persistent
process that reads 'vx vy yaw\\n' from stdin and calls mc_sdk_zsl_1_py.move().
The same stdin protocol as m3v_move:
    "stand" / "lie" / "stop" / "passive" / "vx vy yaw"

We write the file via SFTP (not heredoc/cat over SSH) so the \\n in Python
string literals survive untouched — a previous heredoc deploy corrupted every
"\\n" escape into a literal newline, causing a SyntaxError on startup.

Usage:
    python scripts/deploy_m3v_agibot.py [host] [user]
Defaults: host=10.60.77.154 user=orin-001 (key auth)
"""
import paramiko
import sys

# NOTE: keep \\n as literal backslash-n inside string literals — do NOT let
# any shell/transport turn it into a real newline. Writing this constant via
# SFTP (paramiko write) preserves it byte-for-byte.
PY_CODE = r'''#!/usr/bin/env python3
"""m3v_agibot.py - Persistent velocity controller for Agibot D1.
Reads commands from stdin, sends via mc_sdk_zsl_1_py (UDP 43988).
Safe mode: watchdog sends move(0,0,0) if no command in 500ms.
No auto-stand on startup - user must send 'stand' explicitly."""
import sys, time, threading

sys.path.insert(0, "/home/orin-001/ZCodeProject/lib/zsl-1/aarch64")
import mc_sdk_zsl_1_py

app = mc_sdk_zsl_1_py.HighLevel()
app.initRobot("192.168.234.18", 43988, "192.168.234.1")
time.sleep(0.5)
sys.stderr.write("m3v_agibot ready (SAFE MODE)\n")
sys.stderr.flush()

running = True
last_cmd = time.time()

def watchdog():
    while running:
        time.sleep(0.1)
        if time.time() - last_cmd > 0.5:
            app.move(0.0, 0.0, 0.0)

wd = threading.Thread(target=watchdog, daemon=True)
wd.start()

for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    last_cmd = time.time()
    if line == "stand":
        app.standUp()
        time.sleep(2)
        sys.stderr.write("cmd: stand\n"); sys.stderr.flush()
    elif line == "lie":
        app.move(0.0, 0.0, 0.0)
        time.sleep(0.3)
        app.lieDown()
        sys.stderr.write("cmd: lie\n"); sys.stderr.flush()
    elif line == "stop":
        app.move(0.0, 0.0, 0.0)
        sys.stderr.write("cmd: stop\n"); sys.stderr.flush()
    elif line == "passive":
        app.passive()
        sys.stderr.write("cmd: passive (EMERGENCY)\n"); sys.stderr.flush()
    else:
        parts = line.split()
        if len(parts) == 3:
            try:
                vx, vy, yaw = float(parts[0]), float(parts[1]), float(parts[2])
                app.move(vx, vy, yaw)
            except Exception:
                pass

# stdin closed - safe shutdown
running = False
app.move(0.0, 0.0, 0.0)
time.sleep(0.3)
app.lieDown()
sys.stderr.write("m3v_agibot: stdin closed, safe shutdown\n")
'''


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.60.77.154"
    user = sys.argv[2] if len(sys.argv) > 2 else "orin-001"
    print(f"connecting to {user}@{host} ...")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(host, username=user, timeout=8)

    # Write the source via SFTP so no shell/heredoc can mangle the \n escapes.
    sftp = c.open_sftp()
    with sftp.file("/home/orin-001/m3v_agibot.py", "w") as f:
        f.write(PY_CODE)
    sftp.chmod("/home/orin-001/m3v_agibot.py", 0o755)
    sftp.close()
    print("m3v_agibot.py written (via SFTP, \\n escapes preserved)")

    # Syntax check + show the first few lines to confirm the escapes survived.
    _, o, e = c.exec_command(
        "python3 -c 'import py_compile; py_compile.compile("
        '"/home/orin-001/m3v_agibot.py", doraise=True)\' 2>&1 && '
        "echo SYNTAX_OK", timeout=15)
    o.channel.recv_exit_status()
    print("compile:", o.read().decode("utf-8", "replace").rstrip())
    err = e.read().decode("utf-8", "replace").rstrip()
    if err:
        print("[stderr]", err)

    _, o2, _ = c.exec_command(
        "sed -n '12,16p' /home/orin-001/m3v_agibot.py", timeout=5)
    o2.channel.recv_exit_status()
    print("--- lines 12-16 (should show \\n inside strings, not newlines):")
    print(o2.read().decode().rstrip())

    c.close()
    print("done")


if __name__ == "__main__":
    main()
