# Multi3DViz 受控端 Agent (`robot_side/`)

跑在每台机器人（Ubuntu ARM64）上的 Python 服务。把控制端（Windows 上的 Multi3DViz）和机器狗本体之间的整条链路补齐：录制 FAST-LIO 数据 → 推回 Windows → 读取控制端下发的目标 → 驱动机器狗。**自带 ZCode 风格 web 状态面板**（机器人本机 `http://<robot-ip>:8765/`），实时显示录制/推送/目标/姿态，带急停按钮。

```
┌─────────────────────── Windows (Multi3DViz 控制端) ───────────────────────┐
│  LocalReplaySource 读本地磁盘  ←──────────── SCP 推送 ──────────┐         │
│  ExplorerService SSH 写 target 文件 ──────────────────┐         │         │
└───────────────────────────────────────────────────────┼─────────┼─────────┘
                                                         │         │
              SSH (paramiko, 控制端→机器人)              │         │ SFTP
                                                         ▼         │ (机器人→控制端)
┌─────────────────── 机器人 (Ubuntu ARM64) ───────────────────────▼─────────┐
│  m3v_agent.agent                                                            │
│   ├─ FastlioRecorder  订阅 /cloud_registered + /Odometry                    │
│   │   → cloud_registered/NNNNNN.npy  (原子写 tmp+rename)                    │
│   │   → Odometry/odom_stream.jsonl                                          │
│   │   → gravity_calibration.json (启动静止段算 roll/pitch)                  │
│   ├─ ScpPusher       后台线程把帧推到 Windows 的 <remote_root>/<robot>/data │
│   ├─ TargetPoller    读 ccenter_target_*.txt, 解析 mode/local_x/local_y     │
│   ├─ Navigator       P 控制: 转向→直行→到位(<0.3m)                          │
│   ├─ Driver          Agibot (mc_sdk UDP) | Unitree (SportClient DDS)        │
│   └─ StatusServer    http://<robot-ip>:8765/  ZCode 风格状态面板 + 急停     │
└────────────────────────────────────────────────────────────────────────────┘
```

## 状态面板（ZCode 风格）

部署后浏览器打开 `http://<机器人IP>:8765/` 即可看到（和控制端 Multi3DViz 同一套 zinc/灰 + 绿色 accent token）：

- **机器人卡**：robot_id / 本机 IP / ROS 栈 / driver 连接态 / 站立态
- **录制卡**：帧数 / 重力校准态 / 实时位姿 (x,y,yaw) / cloud+odom topic
- **回传卡**：SCP 连接态 / 已推帧数 / 目标 host / 远端路径
- **执行卡**：nav 状态（idle/turn/drive/arrived）/ 目标文件路径+新旧 / 当前目标坐标
- **🔴 紧急停止按钮**：一键 `driver.emergency_stop()`（趴下+电机阻尼），同时 abort navigator

面板 1Hz 轮询 `/api/state`，无 WebSocket（机器人网络下更稳）。零额外依赖（stdlib `http.server`）。


## 两个预置模板

| 模板 | 机器人 | SDK | 配置 | 部署 |
|---|---|---|---|---|
| **Agibot** | D1 Edu-Ultra | `mc_sdk_zsl_1_py` (UDP 43988) | `templates/agibot/config.yaml` | `templates/agibot/deploy.sh` |
| **Unitree** | Go2 | `unitree_sdk2py` SportClient (CycloneDDS) | `templates/unitree/config.yaml` | `templates/unitree/deploy.sh` |

## 快速开始

详见 [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)。最短路径：

```bash
# 在机器人上（已 ssh 进去）
cd /path/to/Multi3DViz/robot_side
sudo ./templates/<agibot|unitree>/deploy.sh

# 改配置（Windows 的 IP / 账号）
sudo vim /etc/m3v-agent/config.yaml

# 看日志
journalctl -u m3v-agent-<agibot|unitree> -f
```

## 三种运行模式

```bash
python -m m3v_agent.agent --config <yaml> --mode record    # 只录数据+推送
python -m m3v_agent.agent --config <yaml> --mode execute   # 只执行目标
python -m m3v_agent.agent --config <yaml> --mode both      # 闭环 (默认)
```

## 文档

- [docs/AGENT_ARCHITECTURE.md](docs/AGENT_ARCHITECTURE.md) — 受控端架构 + 数据流图
- [docs/DATA_CONTRACT.md](docs/DATA_CONTRACT.md) — 与控制端的双向数据/目标契约（精确 schema）
- [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) — 两台机器人逐步部署 + Windows 开 OpenSSH
- [docs/UNITREE_SDK_NOTES.md](docs/UNITREE_SDK_NOTES.md) — unitree_sdk2py SportClient 速查

## 测试

不需要真机即可跑通整条执行链路（FakeDriver 模拟运动）+ web 面板：

```bash
cd robot_side
python -m pip install pytest
python -m pytest tests/ -v
```

23 个测试覆盖：原子写（并发读写不产生半文件）、目标文件解析（含 stop/junk）、FakeDriver + Navigator + TargetPoller 端到端（写目标文件 → 到位）、web 面板 4 个 HTTP 端点（HTML/JS/state/estop）。

## 目录

```
robot_side/
├── m3v_agent/          # 核心包 (pip install -e .)
│   ├── config.py       # RobotSideConfig (YAML + 环境变量)
│   ├── recorder/       # cloud_sink + gravity_calib + atomic_io
│   ├── transport/      # scp_pusher (机器人→Windows)
│   ├── executor/       # base_driver + navigator + target_poller
│   ├── drivers/        # agibot_driver + unitree_driver
│   ├── web/            # status_server (ZCode 风格 web 面板 + 急停)
│   └── agent.py        # 单入口 + CLI + 状态采集
├── templates/          # 两套 config.yaml + deploy.sh + systemd unit
├── docs/               # 四篇文档
└── tests/              # 23 个测试
```

## 设计要点

1. **原子写**：`.npy` 永远 tmp+fsync+os.replace，Linux 上读者绝不会看到截断数组（破坏 frames[i]↔odom[i] 对齐的元凶）。
2. **odom 用 JSONL**：单追加文件，控制端 `player.py:120-144` 自动优先读取，比每帧一个 JSON 快。
3. **pose 来自 recorder odom 缓存**：两个 driver 都从 recorder 读 (x,y,yaw)，绕开各自 SDK 的不可靠 getter（Agibot ctrlmode 恒 58、Unitree DDS state 需额外订阅）。
4. **不依赖 nav2**：Navigator 是纯 P 控制器（转向→直行→到位 0.3m），两台机器栈差异大，自包含更稳。
5. **零改动控制端**：本包纯加在 `robot_side/`，Multi3DViz 主仓代码一行不改。
6. **ZCode 风格 web 面板**：和控制端同 token（zinc/灰 + #4ec9b0 绿），机器人本机 `http://<ip>:8765/` 实时看状态 + 急停。stdlib `http.server` 零依赖。
