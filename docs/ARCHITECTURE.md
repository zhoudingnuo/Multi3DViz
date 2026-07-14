# Multi3DViz — 架构说明

> 插件化多机协同 3D 可视化框架。前身是 ccenter（Win32+GDI+Open3D 单体），现重构成 Electron + Three.js + Python 架构。

## 一、技术栈

| 层 | 技术 | 职责 |
|---|---|---|
| 外壳 | Electron | 主进程管理 Python sidecar 生命周期 + BrowserWindow |
| 前端 | HTML/CSS/vanilla JS + Three.js | ZCode 风格深色 UI + WebGL 3D 渲染 |
| 通信 | WebSocket (`ws://localhost:PORT`) | JSON 控制消息 + ArrayBuffer 二进制点云帧 |
| 后端 | Python + asyncio + websockets | 插件运行时 + 数据桥 + 算法引擎 |
| 算法 | Open3D + numpy + scipy | ICP 配准、栅格、探索、语义（从 ccenter 复用） |
| 远控 | paramiko SSH | FAST-LIO 拉起、目标下发、断线检测 |

## 二、进程模型

```
Electron Main 进程
  ├── spawn → Python sidecar (backend/main.py)
  │            └─ 监听随机端口，把 PORT 打到 stdout 给 Electron 读
  └── BrowserWindow (frontend/index.html)
       └─ WebSocket 连 ws://localhost:PORT
```

Electron 负责把 Python 当子进程拉起、健康检查、退出时 kill。
Python 后端跑 asyncio WS 服务端，处理所有插件逻辑。
前端通过 WS 收发消息，Three.js 渲染。

## 三、插件系统（对齐 RViz2 Display 模型）

四类插件，全部 Python：

| 类型 | 作用 | 例子 |
|---|---|---|
| **DataSource** | 取数据（SSH 流/本地回放/FAST-LIO 输出），产出规范化传感器帧 | `LocalReplaySource` |
| **Display** | 消费 DataSource，产出 3D/2D 可视化；带用户可调属性 | `PointCloudDisplay`、`GridMapDisplay` |
| **Tool** | 交互式（点击设目标、测距） | `SetNavigationTarget` |
| **Service** | 后台计算/控制 | `ICPRegistrationService`、`SSHLauncherService`、`ConnectionMonitorService` |

**核心契约**：插件产出**声明式 SceneUpdate**（add/update/remove 哪些 Three.js 对象），前端统一渲染。插件不碰前端代码 —— 这是 RViz 的 "Display 产生数据 + 框架渲染" 模型。

**插件发现**：`backend/plugins/<category>/*.py` 自动扫描注册。前端通过 `list_plugins` 消息拿清单，用户勾选启用。

## 四、数据流

```
DataSource(SSH/local) → 传感器帧(点云/odom)
  → Display.update(dt) → SceneUpdate(增量点/网格/盒/标签)
    → SceneBridge → WS 推送(JSON 描述 + ArrayBuffer 二进制)
      → 前端 scene.js 增量更新 Three.js 几何体
```

点云走二进制帧（Float32 positions + Float32 colors），不走 JSON，扛得住百万点。
后端做 voxel downsample（复用 ccenter VOXEL_VIS=0.1）+ 增量推送（只发新增点）。

## 五、从 ccenter 复用的算法模块

| ccenter 模块 | 在 Multi3DViz 的位置 | 复用方式 |
|---|---|---|
| gridmap.py | backend/lib/gridmap.py | 原样（纯 numpy） |
| data_utils.py | backend/lib/data_utils.py | 原样 |
| player.py | backend/lib/player.py | 原样 |
| explorer.py | backend/lib/explorer.py | 原样（Phase 5） |
| registration.py | backend/lib/registration.py | 原样（Phase 4） |
| sem_infer.py | backend/lib/sem_infer.py | 原样（Phase 6） |
| remote_flag.py 的 SSH | 拆进 RobotManager + Service | 改写：硬编码 2 台 → 动态多机 + 心跳/重连 |
| ccenter_app.py / win32_container.py / ui_*.py | — | 废弃（Web 替代） |

## 六、关键设计决策

1. **插件用 Python，不用 JS**：算法资产全在 Python，前端写插件 = 算法重写。前端只做标准化渲染。
2. **前端用 vanilla JS 不用 React**：单人 Python 开发者，降低学习成本。UI 层与逻辑层隔离，将来换 React 只换 UI 层。
3. **点云走二进制 WS 帧**：JSON 编码百万点会爆，必须 ArrayBuffer。
4. **OpenMP 线程限流延续**：backend/main.py 入口最前面 `OMP_NUM_THREADS = cpu//2`，防 WHEA BSOD（延续 ccenter 的防护）。

## 七、目录结构

```
Multi3DViz/
├── package.json              # Electron + npm
├── electron/main.js          # 主进程
├── frontend/
│   ├── index.html
│   ├── css/theme.css         # ZCode 风格 design tokens
│   ├── js/
│   │   ├── ws_client.js      # WS 客户端 + 重连
│   │   ├── scene.js          # Three.js 场景 + 增量更新
│   │   ├── plugin_panel.js   # 插件列表 + 属性面板
│   │   └── robot_panel.js    # 机器人管理
│   └── vendor/three.min.js
├── backend/
│   ├── main.py               # asyncio WS 入口
│   ├── core/                 # 框架核心（插件/场景/协议/机器人）
│   ├── plugins/<category>/   # 插件实现
│   └── lib/                  # ccenter 复用算法
└── docs/
```
