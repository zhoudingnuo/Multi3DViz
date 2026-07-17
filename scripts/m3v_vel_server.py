#!/usr/bin/env python3
"""m3v_vel_server.py — TCP velocity server for Agibot D1 (crash-safe).

Listens on TCP 7777. Each line ('vx vy yaw' / 'stand' / 'lie' / 'stop') is
forwarded to mc_sdk. Replaces the SSH channel for velocity — zero SSH lock
contention. Every SDK call is wrapped in try/except so a bad command can't
crash the server.
"""
import sys, os, time, socket, threading, traceback

sys.path.insert(0, "/home/orin-001/ZCodeProject/lib/zsl-1/aarch64")
import mc_sdk_zsl_1_py

HOST = "0.0.0.0"
PORT = 7777

def log(msg):
    print(msg, flush=True)
    sys.stderr.write(msg + "\n"); sys.stderr.flush()

try:
    app = mc_sdk_zsl_1_py.HighLevel()
    app.initRobot("192.168.234.18", 43988, "192.168.234.1")
    time.sleep(0.5)
    log("VEL_SERVER: SDK initialized")
except Exception as e:
    log("VEL_SERVER: SDK init failed: " + str(e))
    sys.exit(1)

running = True
last_cmd = time.time()
lock = threading.Lock()

def safe_move(vx, vy, yaw):
    try:
        with lock:
            app.move(float(vx), float(vy), float(yaw))
    except Exception:
        pass

def watchdog():
    while running:
        time.sleep(0.1)
        if time.time() - last_cmd > 0.5:
            safe_move(0.0, 0.0, 0.0)

threading.Thread(target=watchdog, daemon=True).start()

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT))
srv.listen(2)
log(f"VEL_SERVER: listening on {HOST}:{PORT}")

while running:
    try:
        conn, addr = srv.accept()
        log(f"VEL_SERVER: control connected from {addr}")
        conn.settimeout(1.0)
        buf = b""
        while running:
            try:
                data = conn.recv(256)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip().decode("utf-8", "replace")
                    if not line:
                        continue
                    last_cmd = time.time()
                    try:
                        if line == "stand":
                            with lock:
                                app.standUp()
                            time.sleep(2)
                            log("VEL_SERVER: standUp ok")
                        elif line == "lie":
                            safe_move(0,0,0); time.sleep(0.3)
                            with lock:
                                app.lieDown()
                            log("VEL_SERVER: lieDown ok")
                        elif line == "stop":
                            safe_move(0,0,0)
                        elif line == "passive":
                            with lock:
                                app.passive()
                        else:
                            parts = line.split()
                            if len(parts) == 3:
                                safe_move(parts[0], parts[1], parts[2])
                    except Exception:
                        pass  # individual command failure doesn't kill server
            except socket.timeout:
                continue
            except OSError:
                break
        try:
            conn.close()
        except Exception:
            pass
        log("VEL_SERVER: control disconnected, stopping robot")
        safe_move(0, 0, 0)
    except Exception:
        log("VEL_SERVER: accept error: " + traceback.format_exc())
        time.sleep(1)
