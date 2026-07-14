# 受控端架构 (AGENT_ARCHITECTURE.md)

> 与 `ccenter/docs/ARCHITECTURE.md` 风格一致，描述 Multi3DViz 受控端（`robot_side/`）的完整数据流和线程模型。

---

## 一、系统全景

```
┌────────────────────────── Windows 控制端 (Multi3DViz) ──────────────────────────┐
│                                                                                 │
│  LocalReplaySource                SSHLauncherService      ExplorerService        │
│  读 <data_root>/<robot>/data/     心跳 + 启动 FAST-LIO     算前沿目标 + SSH 写   │
│         ▲                                │                      │               │
│         │ SFTP put (robot→win)           │ SSH                  │ SSH cat>file  │
└─────────┼────────────────────────────────┼──────────────────────┼──────────────┘
          │                                │                      │
          │ SCP                            ▼                      ▼
┌─────────┼──────────────────────────────────────────────────────────────────────┐
│         │              机器人 (Ubuntu ARM64)  m3v_agent.agent                   │
│         │                                                                          │
│   ┌─────┴──────────┐  ┌───────────────────┐  ┌──────────────────────────────┐  │
│   │ ScpPusher      │  │ FastlioRecorder   │  │ TargetPoller                 │  │
│   │ 后台线程        │  │  订阅 ROS topics  │  │  轮询 ccenter_target_*.txt   │  │
│   │  增量上传帧     │◄─┤  写 ccenter 格式   │  │  解析 mode/local_x/local_y  │  │
│   └────────────────┘  │  + gravity 校准    │  └──────────┬───────────────────┘  │
│                       └────────┬───────────┘             │                      │
│                                │ odom cache              ▼                      │
│                                │ (x,y,yaw)       ┌───────────────┐              │
│                                ├────────────────►│ Navigator      │              │
│                                │                 │ P 控制器        │              │
│                                │                 └───────┬───────┘              │
│                                │                         │ vx,vy,yaw           │
│                                ▼                         ▼                      │
│                       ┌─────────────────┐        ┌───────────────┐              │
│                       │ /cloud_reg      │        │ Driver        │              │
│                       │ /Odometry       │        │ Agibot/Unitree│              │
│                       │ (FAST-LIO)      │        │ SDK 运动指令   │              │
│                       └─────────────────┘        └───────┬───────┘              │
│                                                         │                      │
│                                                         ▼                      │
│                                                机器狗本体 (Go2 / D1)            │
└────────────────────────────────────────────────────────────────────────────────┘
```

## 二、三条数据流

### 2.1 录制流 (机器人 → Windows)

```
FAST-LIO ROS 节点
   │  发布 /cloud_registered (PointCloud2, ~10 Hz)
   │  发布 /Odometry (nav_msgs/Odometry)
   ▼
FastlioRecorder (cloud_sink.py)
   │  ROS spinner 线程回调:
   │    _on_cloud → _extract_xyz_from_pc2 → save_npy_atomic(000000.npy)
   │    _on_odom → _odom_to_dict → append_jsonl(odom_stream.jsonl)
   │    _on_imu  → GravityCalibrator.feed_imu (启动静止段)
   ▼
磁盘: <data_root>/<robot>/data/run_YYYYMMDD_HHMMSS/
   │
   ▼
ScpPusher (scp_pusher.py, 后台线程)
   │  每 cfg.interval 秒:
   │    扫描新 .npy (last_idx++) → sftp.put + posix_rename (原子)
   │    odom_stream.jsonl 全量重传
   │    gravity_calibration.json 一次性传
   ▼
Windows: <remote_root>/<robot>/data/run_YYYYMMDD_HHMMSS/  ← 控制端读这里
```

**关键不变量**：磁盘上的 `.npy` 永远是原子写的（tmp+fsync+os.replace）。ScpPusher 上传到 Windows 也是 tmp+posix_rename。所以控制端的 `player.py:_is_ready` 永远不会撞上半个文件。

### 2.2 目标流 (Windows → 机器人)

```
控制端 ExplorerService._maybe_dispatch
   │  SSH: cat > /home/.../ccenter_target_*.txt
   │  内容: mode/local_x/local_y/...  (key: value 换行)
   ▼
机器人本地文件 ccenter_target_*.txt
   │
   ▼
TargetPoller._tick (target_poller.py, 后台线程)
   │  每 cfg.poll_interval (0.5s):
   │    stat mtime → 跳过未变化
   │    检查 staleness (mtime > 10s → 急停)
   │    read → parse_target_file
   │    mode==explore → nav.goto(local_x, local_y)
   │    mode==stop   → nav.abort()
   ▼
Navigator._loop (navigator.py, 后台线程, 20 Hz)
   │  读 driver.get_pose() (来自 recorder odom cache)
   │  算 dx,dy,dist,yaw_err
   │  转 → move(0, 0, turn)
   │  行 → move(fwd, 0, turn)
   │  到位 (<0.3m) → driver.stop()
   ▼
Driver.move(vx, vy, yaw)
   │  Agibot: app.move(vx, vy, yaw)  via mc_sdk UDP 43988
   │  Unitree: client.Move(vx, vy, yaw)  via SportClient DDS
   ▼
机器狗本体执行
```

### 2.3 心跳/状态流 (控制端探测机器人在线)

这条流在 Multi3DViz 控制端 (`robot_manager.py` 的心跳线程) 完成，受控端**不参与**——机器人只要保持 SSH 可达就行。FAST-LIO 启动也由控制端的 `ssh_launcher.py` 触发（`robot_command` WS 消息）。

### 2.4 状态面板流 (浏览器 ↔ 机器人本机)

```
控制端浏览器 (任意机器)
   │  GET / (HTML) + GET /app.js
   │  GET /api/state (1 Hz)
   │  POST /api/estop (急停按钮)
   ▼
StatusServer (web/status_server.py, ThreadingHTTPServer)
   │  snapshot()  → 读各子模块状态
   │  on_estop()  → driver.emergency_stop() + navigator.abort()
   ▼
RobotAgent.snapshot() 聚合: robot/recorder/transport/executor
```

面板是**只读监控 + 急停**，不参与录制/执行逻辑。无 WebSocket（1 Hz 轮询更稳，机器人网络下尤其如此）。零依赖（stdlib `http.server`）。ZCode token 与控制端 `frontend/css/theme.css` 完全一致。

## 三、线程模型

```
主线程 (agent.py main)
   │
   ├─── RobotAgent.start() 启动各子模块, 然后 while True: sleep (阻塞等信号)
   │
   各后台 daemon 线程:
   │
   ├── ros2-spin (rclpy.spin_once 循环)         ← FAST-LIO 回调在这触发
   │     ├─ _on_cloud  → save_npy_atomic (毫秒级, 不阻塞)
   │     ├─ _on_odom   → append_jsonl  (微秒级)
   │     └─ _on_imu    → GravityCalibrator.feed_imu (启动期)
   │
   ├── scp-push (ScpPusher._loop)               ← cfg.interval 秒一轮
   │     └─ sftp.put (网络阻塞, 但在独立线程)
   │
   ├── nav (Navigator._loop, 20 Hz)             ← 闭环控制
   │     └─ driver.move / driver.stop
   │
   ├── tgt-poll (TargetPoller._loop, 2 Hz)      ← 读目标文件
   │     └─ nav.goto / nav.abort
   │
   └── web-status (ThreadingHTTPServer)         ← 状态面板
         └─ agent.snapshot() (per-request) / agent.emergency_stop() (POST)
```

**线程安全要点**：
- `FastlioRecorder._lock` 保护 `_frame_idx` 和 `_latest_pose`（ROS 回调写，driver 读）。
- `BaseDriver._lock`（Agibot/Unitree driver 各自）保护 SDK 调用（navigator 和 poller 都可能触发 move/stop）。
- `save_npy_atomic` / `append_jsonl` 自身线程安全（POSIX rename 原子 + O_APPEND 单次 write）。

## 四、为何这样切分

| 决策 | 理由 |
|---|---|
| ROS 双栈 (ros1/ros2) | Agibot FAST-LIO 可能跑在 noetic 容器内（rospy），也可能原生（rclpy）。Unitree Go2 是 ROS2。`cloud_sink.py` 用 try-import 探测。 |
| pose 走 odom cache 而非 SDK getter | Agibot `getBatteryPower`→0、`ctrlmode`→58 恒定（文档第8节）。Unitree DDS state 需额外订阅。odom 已在录，权威且统一。 |
| Navigator 不用 nav2 | 两台机器人 ROS 版本/导航栈差异大；纯 P 控制器（转向→直行→到位）自包含、可测、够用。 |
| SCP 推送而非 SFTP 拉取 | Windows 默认无 SSH 服务端；让机器人主动推只需 Windows 开 OpenSSH Server 一次。控制端零改动。 |
| `.npy` 原子写 | Linux 无 Windows 独占锁；半写 .npy 被 np.load 当截断数组加载，破坏 frames[i]↔odom[i] 对齐——这是控制端整条 pipeline 的地基。 |
| odom 用 JSONL 而非每帧 JSON | 控制端 `player.py:120-144` 优先读 JSONL；单追加文件比数千微文件快一个量级。 |

## 五、与控制端的耦合点

本包**不改控制端任何代码**。耦合仅通过两个文件契约：

1. **数据契约**：`<data_root>/<robot>/data/run_*/` 的目录布局 + `.npy`/`.jsonl` 格式。详见 [DATA_CONTRACT.md](DATA_CONTRACT.md)。
2. **目标契约**：`ccenter_target_*.txt` 的 `key: value` 文本格式 + 字段语义。

只要这两个契约对齐，控制端升级不需要受控端跟着改，反之亦然。
