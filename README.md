# Multi3DViz

插件化多机协同 3D 可视化框架。Electron 外壳（ZCode 风格深色 Web UI）+ Three.js 主 3D 渲染 + Python 后端（复用 ccenter 全部算法 + Open3D）+ WebSocket 数据桥。

前身是 ccenter（Win32+GDI+Open3D 单体），现重构成 Web 架构。详见 `docs/ARCHITECTURE.md`。

## 架构

```
Electron Main ──spawn──▶ Python Backend (asyncio WS)
   │                         │  PluginRegistry 自动发现 backend/plugins/
   │                         │  SceneBridge 序列化 SceneUpdate→WS
   ▼                         ▼
BrowserWindow ◀──WebSocket── (JSON 控制 + ArrayBuffer 点云)
  Three.js 渲染
```

四类 Python 插件（对齐 RViz2 Display 模型）：
- **DataSource** — 取数据（本地回放/SSH流/FAST-LIO）
- **Display** — 产出声明式 SceneUpdate（点云/网格/盒/标签）
- **Tool** — 交互式（点击设目标）
- **Service** — 后台计算/控制（ICP/探索/SSH拉起/断线检测）

## 运行

### 前置
- Node.js（开发用 v24 验证通过）
- Python 3.11 + open3d/numpy/scipy/paramiko/torch/cv2
- 项目 venv（`.venv`）已配好，继承 hermes venv 的重型包，单独装了 websockets

### 启动
```bash
npm start      # Electron 拉起 Python sidecar + 加载前端窗口
```
首次需要 `npm install` 装 electron。

## Phase 1 已完成（端到端验证）

- ✅ 项目骨架 + venv + 算法复用（gridmap/data_utils/player）
- ✅ 插件系统（四类基类 + 自动发现）
- ✅ WS 协议（JSON 控制 + ArrayBuffer 二进制点云帧）
- ✅ LocalReplay 源插件（回放 ccenter 数据，108 帧 / 97 万点）
- ✅ PointCloud 显示插件（高度上色）
- ✅ Three.js 视口（OrbitControls 轨道交互）
- ✅ ZCode 风格深色 UI + 插件勾选/属性面板
- ✅ Electron 主进程（spawn sidecar + 端口发现 + 生命周期管理）

验证：`npm start` → 窗口弹出 → 自动显示 unitree 点云（高度上色蓝→红），可拖拽旋转。

## 工程化补全（配置持久化 + 崩溃监控 + 状态面板 + 轨迹导出）

补齐 ccenter 有而 Multi3DViz 缺的工程化/辅助功能：

- ✅ **配置持久化**（`core/config_store.py`）：插件属性 + 启用集合 + 机器人机群 + app 配置存 `multi3dviz_config.json`，重启自动恢复（防抖 1s 写盘，原子替换，退出强存）
- ✅ **崩溃监控**（`core/logger.py`）：psutil 内存/CPU + `sys.excepthook`/`threading.excepthook` 捕获未处理异常 → 写 `logs/crash_*.log`（含堆栈+环境+最后日志），PRODUCT_GOAL 强调的稳定性基础
- ✅ **状态信息面板**：底部状态栏实时显示 frame/pts/reg状态fitness/frontier数/explored%/robots在线数/mem/cpu/fps（聚合各插件状态，对标 ccenter ui_info）
- ✅ **轨迹导出**（复用 `lib/trajectory_plot.py`）：📷 按钮一键导出 PNG（栅格+双机轨迹+目标+覆盖区）

## Phase 6 已完成（端到端验证）

- ✅ 复用 ccenter `sem_infer.py`（UNet 语义分割，类 1-4：墙/房间/走廊/家具）+ `room_detect.py`（scipy 连通分量房间检测）
- ✅ SemanticsService（默认关闭，opt-in）：从 explorer 的合并栅格读 → UNet 推理（后台线程，5s 节流）+ 房间检测
- ✅ 双模式 overlay：`semantic`（按类着色：墙红/房间黄/走廊橙/家具紫）/ `rooms`（按房间 id 轮换色板）
- ✅ 三层 grid2d 叠加渲染（base 占用 + explorer 探索/frontier + semantics 语义/房间）
- ✅ 31MB UNet 权重已纳入 `.gitignore`（单独版本管理）

验证：UNet 对真实障碍物预测出语义类（类 3=走廊）；房间检测识别出房间数；两种模式 overlay 都正确发布。8 插件全部稳定，145 万点 + 542×449 栅格。

## Phase 5 已完成（端到端验证）

- ✅ 复用 ccenter `explorer.py`（DualAgentExplorer：frontier 检测 + 连通性约束 + 目标分配，原样拷贝）
- ✅ DualAgentExplorerService：ICP 成功后建合并栅格 → mark_explored → frontier 检测 → assign_targets → SSH 下发目标
- ✅ 可视化：explorer overlay（探索区绿/frontier 黄）+ 双机轨迹线（橙/品红）+ 目标标记（box）
- ✅ SetNavigationTarget Tool：3D 视口 **Shift+点击** → 地面射线求交 → set_target WS → 覆盖目标 → SSH 下发
- ✅ 目标下发复用 ccenter 协议（`mode/global_x/global_y/local_x/local_y` 写目标文件），2s 冷却防刷
- ✅ WS 协议：set_target（手动目标）/ explorer overlay grid2d 二进制帧

验证：双机数据 → ICP 对齐 → explorer 检测 frontier → 分配目标 → box 标记发布到前端；手动 Shift+点击覆盖目标成功（grid (70,70)→world (2.42,2.42)）。7 插件全部稳定，126 万点 + 542×449 栅格。

## Phase 4 已完成（端到端验证）

- ✅ 复用 ccenter `registration.py`（FGR + 多阶段 ICP，原样拷贝，零改动）
- ✅ ICPRegistrationService：监控双机数据 → 帧数达标自动触发 → 后台线程跑 ICP → 发布合并点云
- ✅ 进度实时推送（registration_progress：每 trial 的 fitness/rmse/score + init/done 阶段）
- ✅ 合并点云（merged_cloud）：A（蓝红高度色）+ T@B（青黄高度色），两机可区分，实时随扫描更新
- ✅ 前端 reg_panel.js：状态徽章（idle/running/aligned/failed）+ fitness/rmse + 实时 trial 详情 + "↻ 重新配准"按钮
- ✅ WS 协议：register（强制重配准）/ registration_status（0.5Hz 快照）/ registration_progress（每 trial）

验证：ICP 恢复已知刚体变换（fitness=1.0, rmse=0, 平移 [2,1,0.5] 精确还原）→ 合并点云正确生成 → 进度事件实时到达前端 → 6 插件全部稳定运行。

## Phase 3 已完成（端到端验证）

- ✅ RobotManager（动态多机注册，替代 ccenter 硬编码 2 台）
- ✅ RobotConnection：持久 SSH 客户端 + 后台心跳（3s ping）+ 断线自动重连（指数退避 2→30s）
- ✅ SSHLauncherService：拉起/停止 FAST-LIO（nohup 保活）、通用 SSH 命令、auto-launch 选项
- ✅ ConnectionMonitorService：跟踪 uptime/重连次数
- ✅ WS 协议：robot_add / robot_remove / robot_list / robot_command / robot_status（心跳线程→asyncio 线程安全 marshal）
- ✅ 前端 robot_panel.js：机器人列表（实时状态色：绿=在线/黄=连接中/红=断开）+ 添加表单（host/user/password/data_path/launch_cmd）+ ▶启动 ■停止 ✕删除

验证：运行时动态添加机器人 → 心跳检测连接 → 状态实时推送到 UI → 断线自动重连 → SSH 命令路由到对应机器人的 SSHLauncher。重复 ID 被拒、不可达主机不崩溃（进入 reconnecting）。

## Phase 2 已完成（端到端验证）

- ✅ GridMap 显示插件（复用 lib/gridmap.py，2态占用栅格 0 free/100 obstacle）
- ✅ grid2d 二进制序列化（int8 cells + origin/resolution，走二进制帧）
- ✅ 2D 顶视图 canvas（grid_view.js）：缩放（光标锚定）/右键平移/点击→世界坐标
- ✅ 回放控制条：播放/暂停/逐帧/拖动跳转/0.5×–8× 变速
- ✅ playback_state 后端→前端周期同步（4Hz，覆盖循环跳转等后端主导变化）
- ✅ 布局：3D 视口（左）| 2D 栅格（右，无数据时自动折叠）| 底部回放条

验证：3 个插件自动启用 → 3D 点云 + 2D 栅格同屏 → 栅格随探索扩展（420×444→541×447）→ 点云累积到 128 万 → 回放条实时同步帧号。

## Phase 7（部分完成：文档 + 打包配置；实际 exe 生成待独立会话）

- ✅ 开发者文档：`docs/PLUGIN_DEV.md`（四类插件开发指南 + 示例）、`docs/WS_PROTOCOL.md`（WS 协议参考）、`docs/PACKAGING.md`（PyInstaller + electron-builder 打包方案）
- ✅ `package.json` 加 electron-builder 配置（nsis 安装包 + extraResources 打 Python sidecar）
- ✅ `electron/main.js` 支持打包后路径（PyInstaller 冻结的 exe vs 开发态 venv）
- ⏳ 实际 exe 生成未执行（PyInstaller 冻结 Open3D+PyTorch 耗时长、包体 GB 级，建议单独会话在干净环境验证）

## 受控端 App（`robot_side/`）— 双机器人协同闭环

跑在每台机器人上的 **桌面 App**（Electron 窗口 + m3v_agent Python 服务，和控制端同技术栈）。补齐 Multi3DViz 控制端↔机器狗之间的整条链路。**零改动控制端代码**，纯加子目录。打成 `.deb` 装到 Ubuntu arm64 机器人。

```
控制端 (Windows Multi3DViz exe)  ←──SCP推送──  受控端 (机器人 .deb)
                                 ──SSH写目标文件──▶
                                                ↓
                              Electron 窗口 (ZCode 风格面板) + 急停按钮
                                                ↓ stdout/stdin IPC
                              m3v_agent: 录制 + 推送 + 目标执行 + driver
```

- **桌面 App**（`robot_side/electron/` + `shell/`）：Electron 窗口加载 `shell/index.html`（ZCode 风格面板，和控制端同 token），通过 stdout/stdin IPC 和 m3v_agent 通信（无 HTTP 服务器）。状态卡：机器人/录制/回传/执行 + 红色急停按钮
- **m3v_agent**（`m3v_agent/`）：核心 Python 服务，三种模式（`--mode record|execute|both`），可独立跑（systemd 无头）或被 Electron 壳拉起（`--ui-stdio`）
  - **Recorder**：订阅 FAST-LIO `/cloud_registered`+`/Odometry` → 写 ccenter 格式（原子 `.npy` + `odom_stream.jsonl` + `gravity_calibration.json`）
  - **ScpPusher**：后台推到 Windows 的 `C:\Users\Z790\ccenter\<robot>\data\`
  - **Executor**：轮询 `ccenter_target_*.txt` → P 控制器导航（到位 0.3m）
  - **两个预置模板**：`agibot`（mc_sdk UDP 43988）/ `unitree`（go2_bridge TCP 桥）
- **打包**：`packaging/build-deb.sh` → electron-builder → `m3v-agent_<ver>_arm64.deb`（含 postinst 自动装 Python 依赖 + systemd unit + 桌面菜单项）

部署 + 详见 `robot_side/README.md` 与 `docs/PACKAGING.md`（含控制端 exe PyInstaller spec）。26 个测试覆盖：原子写、目标解析、FakeDriver 端到端导航、web 面板 HTTP 端点、stdio IPC 协议（READY/STATE/ESTOP_ACK）。

## 文档索引

| 文档 | 内容 |
|---|---|
| `README.md` | 本文件（总览 + 各 Phase 总结） |
| `docs/ARCHITECTURE.md` | 整体架构（Electron + Three.js + Python + WS + 插件系统） |
| `docs/PLUGIN_DEV.md` | 插件开发指南（四类插件 + SceneObject kind + 示例） |
| `docs/WS_PROTOCOL.md` | WebSocket 协议参考（所有消息类型 + 二进制帧格式） |
| `docs/PACKAGING.md` | 打包方案（PyInstaller 冻结 + electron-builder + 签名 + 便携模式） |
| `robot_side/README.md` | 受控端 agent 总览 + 快速开始 |
| `robot_side/docs/AGENT_ARCHITECTURE.md` | 受控端架构 + 三条数据流 + 线程模型 |
| `robot_side/docs/DATA_CONTRACT.md` | 受控端↔控制端双向文件契约（精确 schema） |
| `robot_side/docs/DEPLOYMENT.md` | 两台机器人逐步部署 + Windows 开 OpenSSH |
| `robot_side/docs/UNITREE_SDK_NOTES.md` | unitree_sdk2py SportClient 速查 + CycloneDDS 配置 |

## 目录

```
electron/main.js        主进程（spawn sidecar + 窗口）
frontend/               前端（Three.js + 深色 UI）
  index.html, css/theme.css, js/{ws_client,scene,plugin_panel,app}.js
backend/
  main.py               asyncio WS 入口（打印 READY ws://... 供 Electron 读端口）
  core/                 框架核心
    plugin_base.py      四类插件基类 + SceneUpdate/PluginContext
    plugin_registry.py  自动发现 + 生命周期
    scene_bridge.py     SceneUpdate→WS（二进制点云帧）
    ws_protocol.py      JSON 协议
  plugins/<category>/   插件实现
    source/local_replay.py
    display/point_cloud.py
  lib/                  从 ccenter 复用的纯算法（gridmap/data_utils/player）
docs/ARCHITECTURE.md    架构详解
```
