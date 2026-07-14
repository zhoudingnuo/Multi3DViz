# 数据契约 (DATA_CONTRACT.md)

受控端 (`robot_side/`) 与控制端 (Multi3DViz Windows) 之间的双向文件契约。**精确到字段名、dtype、单位**——任何一处不匹配都会断链。

---

## 一、录制契约（受控端 → 控制端）

### 1.1 目录布局

控制端 `backend/plugins/source/local_replay.py:153-156` 扫描：

```python
root = self.get("data_root")          # 默认 C:\Users\Z790\ccenter
robot = self.get("robot")             # "unitree" | "agibot"
scan_dir = os.path.join(root, robot, "data")
```

所以受控端 SCP 推送必须落到 Windows 的：

```
<remote_root>/<robot>/data/run_YYYYMMDD_HHMMSS/
    ├── cloud_registered/
    │   └── 000000.npy  (000001.npy, ...)
    ├── Odometry/
    │   └── odom_stream.jsonl
    └── gravity_calibration.json     (可选)
```

`<remote_root>` = transport.remote_root（默认 `C:/Users/Z790/ccenter`）。
`<robot>` = recorder.robot（"unitree" 或 "agibot"，和控制端 LocalReplaySource 的 `robot` 属性一致）。

### 1.2 运行目录命名

- 格式：`run_YYYYMMDD_HHMMSS`（零填充，字典序 = 时间序）
- 控制端取最新：`max(os.listdir(scan_dir))`（`player.py:_latest_run_dir`）
- 受控端 `cloud_sink.py:_make_run_dir` 用 `datetime.now().strftime("%Y%m%d_%H%M%S")` 生成

### 1.3 点云文件 `cloud_registered/*.npy`

| 属性 | 值 | 受控端实现 |
|---|---|---|
| 格式 | numpy `.npy`，`np.load` 可读 | `save_npy_atomic` (`atomic_io.py`) |
| dtype | `float32` | `np.float32` |
| shape | `(N, 3)`，N = 点数（每帧不同） | `_extract_xyz_from_pc2` 返回 (N,3) |
| 单位 | 米 | PointCloud2 的 x/y/z 字段 |
| 坐标系 | robot odometry frame (`camera_init`)，开机位姿为原点 | FAST-LIO 输出原样 |
| 重力校正 | **不做**（控制端加载后自行乘 R） | 受控端写原始 XYZ |
| 最小字节 | 1024 (`MIN_NPY_BYTES`, `player.py:13`)；更小视为半写 | 受控端原子写保证永远完整 |

**命名**（两种都可，控制端只 `sorted(glob('*.npy'))`）：
- `index`：`000000.npy`（Agibot 默认）
- `indexed`：`000000_HHMMSS_mmm.npy`（Unitree 默认，带时间戳便于人读）

两种都以 6 位零填充递增计数器开头，排序结果正确。

**原子写契约（Linux 上头号要求）**：
受控端 `save_npy_atomic` 写 `<path>.<pid>.tmp` → fsync → `os.replace(tmp, path)`。读者要么看到旧版本要么看到新版本，**绝不会看到半文件**。Linux 没有 Windows 的独占锁探测，所以这条契约必须由写入方保证。

### 1.4 里程计 `Odometry/odom_stream.jsonl`（优先）

控制端 `player.py:120-144` **优先**读 JSONL，回退到逐帧 `.json`。

| 属性 | 值 |
|---|---|
| 格式 | JSONL，每行一个 JSON 对象 |
| 每行 schema | 见下表 |
| 读取方式 | 逐行读，遇解析失败（半写）就停，下 tick 重试 |
| 行号 | = 帧索引（与 cloud_registered/NNNNNN.npy 按序对齐） |

**每行 JSON schema**：

```json
{
  "stamp":    1783681485.276,
  "frame_id": "camera_init",
  "x":        0.003807,
  "y":       -0.019023,
  "z":        0.010433,
  "qx":      -0.000528,
  "qy":       0.000507,
  "qz":       0.000958,
  "qw":       0.999999,
  "yaw":      0.001917
}
```

| 字段 | 类型 | 控制端是否读 | 说明 |
|---|---|:-:|---|
| `x` | float | ✅ | 平移 X（米） |
| `y` | float | ✅ | 平移 Y（米） |
| `z` | float | ✅ | 平移 Z（米） |
| `qx` | float | ✅ | 四元数 X |
| `qy` | float | ✅ | 四元数 Y |
| `qz` | float | ✅ | 四元数 Z |
| `qw` | float | ✅ | 四元数 W |
| `stamp` | float | ❌ | UNIX 时间戳（秒），便于排查 |
| `frame_id` | string | ❌ | 固定 `"camera_init"` |
| `yaw` | float | ❌ | 弧度，便于排查 |

**四元数为标准 Hamilton 形式**。控制端 `data_utils.py:81-89` 的 `quat_to_mat(qx,qy,qz,qw)` 用 `(qx,qy,qz,qw)` 构建 4×4 齐次变换矩阵。**键名必须精确匹配**。

受控端 `cloud_sink.py:_odom_to_dict` 从 `nav_msgs/Odometry` 的 `pose.pose` 提取这些字段。

### 1.5 重力校准 `gravity_calibration.json`（可选）

| 字段 | 类型 | 控制端读 | 说明 |
|---|---|:-:|---|
| `roll_deg` | float | ✅ | IMU 倾斜 roll（**度**） |
| `pitch_deg` | float | ✅ | IMU 倾斜 pitch（**度**） |
| 其余 | any | ❌ | 信息性，控制端忽略 |

控制端 `data_utils.py:34-37` 构建 `R = R_roll(roll_deg) @ R_pitch(pitch_deg)`，应用到每帧点云：`(R @ f.T).T`。**文件缺失则用单位矩阵**（跳过重力校正）。

受控端 `gravity_calib.py:GravityCalibrator` 启动时采静止段 IMU accel，算 `roll=atan2(gy,gz)`、`pitch=atan2(-gx,hypot(gy,gz))`，转度数写入。

### 1.6 索引对齐（核心约束）

`cloud_registered[i] ↔ odom_stream.jsonl[i]` 必须**严格一一对应**。受控端保证：
1. cloud 和 odom 都在各自 ROS 回调里写，按到达顺序递增命名/追加。
2. 同一帧的 cloud + odom 几乎同时到达（FAST-LIO 同步发布）。
3. 不跳帧、不丢帧——`save_npy_atomic` 失败时不递增 `_frame_idx`（`cloud_sink.py:_on_cloud`）。

---

## 二、目标契约（控制端 → 受控端）

### 2.1 文件位置（每机器人不同）

| 机器人 | 本地路径 | SSH 认证（控制端写入用） |
|---|---|---|
| Unitree (A) | `/home/unitree/sda2/online/ccenter_target_a.txt` | 密码 `123` |
| Agibot (B) | `/home/orin-001/ccenter_target_b.txt` | **密钥免密** |

路径由控制端 `explorer_service.py:65-71` 的 `target_path_a/target_path_b` 决定，可配。

### 2.2 文件格式

控制端 `explorer_service.py:233` 序列化（与 ccenter `remote_flag.py:127` 一致）：

```python
content = "\n".join(f"{k}: {v}" for k, v in fields.items()) + "\n"
```

写入方式：SSH `mkdir -p <parent> && cat > <path>`（`robot_manager.py:write_file`）。

**完整实例**：
```
mode: explore
global_x: 2.340
global_y: 1.080
local_x: 2.340
local_y: 1.080
frame: 116
timestamp: 2026-07-13 14:22:05
```

注意：冒号后有空格，每行 `\n` 结尾。

### 2.3 字段语义

| 键 | 类型 | 示例 | 含义 |
|---|---|---|---|
| `mode` | string | `explore` / `stop` | 指令模式 |
| `global_x` | float (3位小数) | `2.340` | 目标 X，**合并世界坐标**（米） |
| `global_y` | float (3位小数) | `1.080` | 目标 Y，合并世界坐标（米） |
| `local_x` | float (3位小数) | `2.340` | 目标 X，**本机器人里程计坐标系**（米） |
| `local_y` | float (3位小数) | `1.080` | 目标 Y，本机器人里程计坐标系（米） |
| `frame` | int | `116` | 控制端下发时的云帧计数器 |
| `timestamp` | string | `2026-07-13 14:22:05` | `%Y-%m-%d %H:%M:%S` |

### 2.4 global vs local 坐标语义

- `global_*` = 合并帧（ICP 对齐后的共享地图）中的目标；Unitree A 是原点。
- `local_*` = 目标转换到**接收机器人自己的里程计坐标系**。
  - Robot A (Unitree)：`local == global`（A 是合并帧原点）
  - Robot B (Agibot)：`local = inv(T_b_to_a) @ global`
- **受控端只需读 `local_x` / `local_y`** 即可导航，无需处理坐标变换。

> ⚠️ **Multi3DViz 当前实现**：`explorer_service.py:234` 把 `wx,wy` 同时写进 global 和 local（即 local == global）。ccenter 原版做了 B 的 inv(T) 变换。如果控制端升级到做完整变换，受控端无需改——它本来就只读 local。

### 2.5 mode 值

| mode | 含义 | 坐标 |
|---|---|---|
| `explore` | 有前沿目标，导航到 `(local_x, local_y)` | 有效 |
| `stop` | 无前沿可用，目标为 None | 全为 `0.000` |

### 2.6 下发频率与判活

- 目标文件每 ~1-2 秒重写（即使不变也重写）。
- 受控端用 mtime 判活：超过 `stale_timeout`（默认 10s）未更新 → 视为控制端离线 → 停车（安全）。
- 受控端只在 mtime 变化时重新解析（`target_poller.py:_tick`），避免无谓重算。

---

## 三、契约对齐速查

| 受控端组件 | 对应控制端读取代码 | 契约点 |
|---|---|---|
| `save_npy_atomic` | `player.py:_is_ready` + `np.load` | 原子写 + (N,3) float32 |
| `append_jsonl` | `player.py:poll_new_odometry` | JSONL，遇坏行停 |
| `_odom_to_dict` | `data_utils.py:quat_to_mat` | qx,qy,qz,qw 键名 |
| `GravityCalibrator` | `data_utils.py:load_gravity` | roll_deg/pitch_deg |
| `_make_run_dir` | `player.py:_latest_run_dir` | run_YYYYMMDD_HHMMSS 排序 |
| `parse_target_file` | `explorer_service.py:_maybe_dispatch` | key:value + mode 语义 |
| `ScpPusher` 远端路径 | `local_replay.py:_run_dir` | `<root>/<robot>/data/` |
