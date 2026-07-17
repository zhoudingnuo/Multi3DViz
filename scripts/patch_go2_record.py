#!/usr/bin/env python3
"""Patch go2_record.py on the robot for single-frame overwrite mode:
- Cloud: write one timestamped file, delete old, SCP as latest.npy
- Odom: overwrite (truncate) instead of append"""
import paramiko, sys

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

# Save backup (only if not already backed up)
try:
    with sftp.open(PATH + '.orig', 'r') as f:
        pass  # exists
except Exception:
    with sftp.open(PATH + '.orig', 'w') as f:
        f.write(code)
    print("Saved original backup to", PATH + '.orig')

# === PATCH 1: save_cloud → single timestamped frame + delete old ===
old_cloud = (
    'def save_cloud(topic_dir: str, idx: int, msg: PointCloud2) -> str:\n'
    '    pts = pc2numpy(msg)\n'
    '    fname = f"{idx:06d}_{stamp2fname_suffix(msg.header.stamp)}.npy"\n'
    '    np.save(os.path.join(topic_dir, fname), pts)\n'
    '    return fname'
)
new_cloud = (
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
assert old_cloud in code, "PATCH1 FAIL: save_cloud pattern not found"
code = code.replace(old_cloud, new_cloud)
print("PATCH 1 OK: save_cloud → single timestamped frame")

# === PATCH 2: _ssh_worker → SCP as latest.npy + delete local after push ===
old_worker = (
    'def _ssh_worker() -> None:\n'
    '    """\u540e\u53f0\u7ebf\u7a0b\uff1a\u4ece\u961f\u5217\u53d6\u6587\u4ef6 scp \u4e0a\u4f20\u3002"""\n'
    '    assert _ssh_queue is not None\n'
    '    while True:\n'
    '        item = _ssh_queue.get()\n'
    '        if item is None:\n'
    '            break\n'
    '        local_path, remote_dir = item\n'
    '        _ssh_ensure_dir(remote_dir)\n'
    '        try:\n'
    '            _ssh_run(["scp", local_path, f"{SSH_USER}@{SSH_HOST}:{remote_dir}/"])\n'
    '        except Exception:\n'
    '            pass\n'
    '        _ssh_queue.task_done()'
)
new_worker = (
    'def _ssh_worker() -> None:\n'
    '    """\u540e\u53f0\u7ebf\u7a0b\uff1a\u4ece\u961f\u5217\u53d6\u6587\u4ef6 scp \u4e0a\u4f20\u3002\n'
    '    \u70b9\u4e91\u59cb\u7ec8\u8986\u76d6\u4e3a latest.npy (Windows \u7aef\u53ea\u4fdd\u7559\u4e00\u5e27)\u3002"""\n'
    '    assert _ssh_queue is not None\n'
    '    while True:\n'
    '        item = _ssh_queue.get()\n'
    '        if item is None:\n'
    '            break\n'
    '        local_path, remote_dir = item\n'
    '        _ssh_ensure_dir(remote_dir)\n'
    '        try:\n'
    '            _ssh_run(["scp", local_path,\n'
    '                      f"{SSH_USER}@{SSH_HOST}:{remote_dir}/latest.npy"])\n'
    '            # Delete local file after push — no accumulation.\n'
    '            try:\n'
    '                os.remove(local_path)\n'
    '            except OSError:\n'
    '                pass\n'
    '        except Exception:\n'
    '            pass\n'
    '        _ssh_queue.task_done()'
)
assert old_worker in code, "PATCH2 FAIL: _ssh_worker pattern not found"
code = code.replace(old_worker, new_worker)
print("PATCH 2 OK: SCP cloud → latest.npy + delete local")

# === PATCH 3: odom stream → overwrite not append ===
# The remote python opens file in 'a' (append) mode. Change to overwrite.
old_odom = "\"f=open(p,'a',1)\\n\"  # 1=\u884c\u7f13\u51b2, \u6bcf\u884c flush\n                \"for line in sys.stdin:\\n\"\n                \"    f.write(line); f.flush()\\n\""
new_odom = "\"for line in sys.stdin:\\n\"\n                \"    open(p,'w').write(line)\\n\"  # \u8986\u76d6: \u53ea\u4fdd\u7559\u6700\u65b0\u4e00\u884c"
if old_odom in code:
    code = code.replace(old_odom, new_odom)
    print("PATCH 3 OK: odom → overwrite")
else:
    # Try alternate quoting
    old_odom2 = '"f=open(p,\'a\',1)\\n"  # 1=\u884c\u7f13\u51b2, \u6bcf\u884c flush'
    if old_odom2 in code:
        # Replace the whole append block
        lines = code.split('\n')
        out = []
        skip = 0
        for i, line in enumerate(lines):
            if skip > 0:
                skip -= 1
                continue
            if old_odom2 in line:
                out.append('                "for line in sys.stdin:\\n"')
                out.append('                "    open(p,\'w\').write(line)\\n"  # \u8986\u76d6: \u53ea\u4fdd\u7559\u6700\u65b0\u4e00\u884c')
                # Skip the next 2 lines (the for/f.write lines)
                skip = 2
            else:
                out.append(line)
        code = '\n'.join(out)
        print("PATCH 3 OK: odom → overwrite (line-by-line)")
    else:
        print("PATCH 3 SKIP: odom pattern not found — manual fix needed")

with sftp.open(PATH, 'w') as f:
    f.write(code)
sftp.close()
c.close()
print("Done — go2_record.py patched on robot")
