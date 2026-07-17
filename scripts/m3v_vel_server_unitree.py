#!/usr/bin/env python3
"""m3v_vel_server_unitree.py — TCP velocity server for Unitree Go2 (debug version).
Logs every connection + command so we can see what the control side actually sends."""
import sys, os, time, socket, threading, subprocess

HOST = "0.0.0.0"
PORT = 7777
M3V_MOVE = "/home/unitree/m3v_move"
IFACE = "eth0"

def log(msg):
    line = "VEL %s: %s" % (time.strftime("%H:%M:%S"), msg)
    sys.stderr.write(line + "\n"); sys.stderr.flush()

proc = subprocess.Popen([M3V_MOVE, IFACE], stdin=subprocess.PIPE,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0)
time.sleep(2)
log("m3v_move started (pid=%d)" % proc.pid)

running = True
last_cmd = time.time()
lock = threading.Lock()

def safe_write(line):
    try:
        with lock:
            proc.stdin.write((line + "\n").encode())
            proc.stdin.flush()
    except Exception as e:
        log("write error: %s" % e)

def watchdog():
    while running:
        time.sleep(0.1)
        if time.time() - last_cmd > 0.5:
            safe_write("0.0 0.0 0.0")

threading.Thread(target=watchdog, daemon=True).start()

def drain():
    while running:
        try:
            if not proc.stdout.readline(): break
        except Exception: break
threading.Thread(target=drain, daemon=True).start()

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT))
srv.listen(5)  # allow multiple pending connections
log("listening on %s:%d" % (HOST, PORT))

# Allow MULTIPLE simultaneous control connections — each gets its own handler.
def handle_client(conn, addr):
    global last_cmd
    log("control connected from %s" % str(addr))
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
                log("cmd: %s" % line)
                safe_write(line)
        except socket.timeout:
            continue
        except OSError:
            break
    try: conn.close()
    except: pass
    log("control disconnected from %s" % str(addr))

while running:
    try:
        conn, addr = srv.accept()
        threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except Exception as e:
        log("accept error: %s" % e)
        time.sleep(1)
