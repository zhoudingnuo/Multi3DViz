#!/usr/bin/env python3
"""Patch go2_record.py v2: fix the SCP race condition.
save_cloud writes frame_xxx.npy but does NOT delete others (that races
with the SCP queue). Instead, _ssh_worker handles single-file overwrite.
Also: go2_record uses a SYNCHRONOUS scp for cloud (no queue) so there's
no backlog — the latest frame always gets through."""
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

# === PATCH: save_cloud → just save + scp synchronously, no queue, no delete ===
old_cloud = (
    'def save_cloud(topic_dir: str, idx: int, msg: PointCloud2) -> str:\n'
    '    pts = pc2numpy(msg)\n'
    '    # SINGLE-FRAME MODE: write one timestamped file, delete all others.\n'
    '    # The timestamp lets us distinguish new vs stale frames.\n'
    '    # SCP pushes it to Windows as latest.npy (overwrite).\n'
    '    fname = f"frame_{stamp2fname_suffix(msg.header.stamp)}.npy"\n'
    '    np.save(os.path.join(topic_dir, fname), pts)\n'
    '    # Delete all other .npy files — keep only the latest frame.\n'
    '    for old in os.listdir(topic_dir):\n'
    '        if old.endswith(".npy") and old != fname:\n'
    '            try:\n'
    '                os.remove(os.path.join(topic_dir, old))\n'
    '            except OSError:\n'
    '                pass\n'
    '    return fname'
)
new_cloud = (
    'def save_cloud(topic_dir: str, idx: int, msg: PointCloud2) -> str:\n'
    '    pts = pc2numpy(msg)\n'
    '    fname = f"frame_{stamp2fname_suffix(msg.header.stamp)}.npy"\n'
    '    local = os.path.join(topic_dir, fname)\n'
    '    np.save(local, pts)\n'
    '    return fname'
)
assert old_cloud in code, "save_cloud v1 pattern not found"
code = code.replace(old_cloud, new_cloud)
print("PATCH A OK: save_cloud simplified (no delete, no listdir)")

# === PATCH: callback for cloud → SYNCHRONOUS scp, not queue ===
old_cb = (
    '                    else:\n'
    '                        fname = sf(td, i, msg)\n'
    '                        ssh_sync(os.path.join(td, fname), self.run_name, tn, fname)'
)
new_cb = (
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
assert old_cb in code, "callback pattern not found"
code = code.replace(old_cb, new_cb)
print("PATCH B OK: cloud callback → synchronous scp latest.npy")

with sftp.open(PATH, 'w') as f:
    f.write(code)
sftp.close()
c.close()
print("Done — go2_record.py v2 patched")
