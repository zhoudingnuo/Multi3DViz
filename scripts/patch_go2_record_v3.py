#!/usr/bin/env python3
"""Patch go2_record.py v3: fix SCP blocking the ROS callback.

v2 made cloud SCP synchronous inside the rospy callback — this blocks the
callback thread when SCP is slow, freezing all frame processing. v3 uses a
single dedicated uploader thread that:
  - Receives (local_path, remote_dir) via a 1-slot slot (not a queue)
  - Each new frame OVERWRITES the pending slot — old pending frames are
    dropped (we only want the freshest)
  - SCPs to remote as latest.npy
  - Deletes the local file after push
This way the callback never blocks, and we always push the newest available
frame."""
import paramiko

HOST = "10.60.77.187"
USER = "unitree"
PWD = "123"
PATH = "/home/unitree/sda2/ws/src/go2_nav/scripts/go2_record.py"

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PWD, timeout=5)
sftp = c.open_sftp()

with sftp.open(PATH, 'r') as f:
    code = f.read().decode('utf-8')

# === PATCH: replace the synchronous cloud scp in the callback with async slot ===
old_cb = (
    '                    else:\n'
    '                        fname = sf(td, i, msg)\n'
    '                        local_path = os.path.join(td, fname)\n'
    '                        if t == "/cloud_registered":\n'
    '                            # SYNCHRONOUS scp: overwrite remote latest.npy.\n'
    '                            # No queue backlog — the freshest frame always\n'
    '                            # wins. Delete local after push (single frame).\n'
    '                            remote_dir = f"{SSH_REMOTE_ROOT}/{self.run_name}/{tn}"\n'
    '                            _ssh_ensure_dir(remote_dir)\n'
    '                            try:\n'
    '                                _ssh_run(["scp", local_path,\n'
    '                                          f"{SSH_USER}@{SSH_HOST}:{remote_dir}/latest.npy"])\n'
    '                                os.remove(local_path)\n'
    '                            except Exception:\n'
    '                                pass\n'
    '                        else:\n'
    '                            ssh_sync(local_path, self.run_name, tn, fname)'
)
new_cb = (
    '                    else:\n'
    '                        fname = sf(td, i, msg)\n'
    '                        local_path = os.path.join(td, fname)\n'
    '                        if t == "/cloud_registered":\n'
    '                            # ASYNC single-slot: hand off to the cloud\n'
    '                            # uploader thread (non-blocking). Old pending\n'
    '                            # frame is replaced — only freshest gets pushed.\n'
    '                            _cloud_slot_put(local_path, self.run_name, tn)\n'
    '                        else:\n'
    '                            ssh_sync(local_path, self.run_name, tn, fname)'
)
assert old_cb in code, "callback v2 pattern not found"
code = code.replace(old_cb, new_cb)
print("PATCH A OK: callback → async cloud slot (non-blocking)")

# === PATCH: add the cloud uploader thread + slot right after _ssh_worker_odom ===
anchor = "TOPIC_TYPES = {"
inject = (
    '# --- Cloud single-slot uploader (non-blocking, overwrite) ---\n'
    '# One slot: each new frame replaces the pending one. A dedicated thread\n'
    '# SCPs whatever is in the slot to Windows as latest.npy. This NEVER blocks\n'
    '# the ROS callback — if SCP is slow, old frames are simply dropped.\n'
    '_cloud_lock = threading.Lock()\n'
    '_cloud_item = [None]  # [(local_path, run_name, topic_name)] or [None]\n'
    '_cloud_thread = None\n'
    '\n'
    '\n'
    'def _cloud_slot_put(local_path, run_name, topic_name):\n'
    '    """Non-blocking: replace the pending cloud frame. Called from ROS cb."""\n'
    '    global _cloud_item\n'
    '    with _cloud_lock:\n'
    '        # Delete the previously-pending local file if it was replaced\n'
    '        # before being uploaded (avoid orphan files on disk).\n'
    '        old = _cloud_item[0]\n'
    '        if old is not None:\n'
    '            try:\n'
    '                os.remove(old[0])\n'
    '            except OSError:\n'
    '                pass\n'
    '        _cloud_item[0] = (local_path, run_name, topic_name)\n'
    '\n'
    '\n'
    'def _cloud_worker():\n'
    '    """Dedicated thread: SCP the latest cloud frame to latest.npy."""\n'
    '    global _cloud_item\n'
    '    while True:\n'
    '        item = None\n'
    '        with _cloud_lock:\n'
    '            item = _cloud_item[0]\n'
    '            _cloud_item[0] = None\n'
    '        if item is None:\n'
    '            time.sleep(0.01)  # idle — no pending frame\n'
    '            continue\n'
    '        local_path, run_name, topic_name = item\n'
    '        remote_dir = f"{SSH_REMOTE_ROOT}/{run_name}/{topic_name}"\n'
    '        _ssh_ensure_dir(remote_dir)\n'
    '        try:\n'
    '            _ssh_run(["scp", local_path,\n'
    '                      f"{SSH_USER}@{SSH_HOST}:{remote_dir}/latest.npy"])\n'
    '        except Exception:\n'
    '            pass\n'
    '        try:\n'
    '            os.remove(local_path)\n'
    '        except OSError:\n'
    '            pass\n'
    '\n'
    '\n'
)
# Need to import time in go2_record — check if already imported
if "import time" not in code.split('\n')[:30].__str__():
    inject = "import time\n" + inject

code = code.replace(anchor, inject + anchor, 1)
print("PATCH B OK: added cloud uploader thread + slot")

# === PATCH: start the cloud thread in main() ===
old_start = (
    '        _ssh_thread_odem = threading.Thread(target=_ssh_worker_odom, daemon=True)\n'
    '        _ssh_thread_odom.start()'
)
# Check actual variable name
if old_start not in code:
    old_start = (
    '        _ssh_thread_odom = threading.Thread(target=_ssh_worker_odom, daemon=True)\n'
    '        _ssh_thread_odom.start()'
    )
if old_start not in code:
    # Try with odom spelling
    import re
    m = re.search(r'(_ssh_thread_odom\w* = threading\.Thread\(target=_ssh_worker_odom.*?\n.*?\.start\(\))', code)
    if m:
        old_start = m.group(1)
    else:
        old_start = ""

if old_start and old_start in code:
    new_start = old_start + '\n'
    new_start += (
        '        # Cloud uploader: single-slot, overwrites pending frame.\n'
        '        global _cloud_thread\n'
        '        _cloud_thread = threading.Thread(target=_cloud_worker, daemon=True)\n'
        '        _cloud_thread.start()'
    )
    code = code.replace(old_start, new_start)
    print("PATCH C OK: cloud thread started in main()")
else:
    print("PATCH C SKIP: could not find odom thread start — manual fix needed")

with sftp.open(PATH, 'w') as f:
    f.write(code)
sftp.close()
c.close()
print("Done — go2_record.py v3 patched")
