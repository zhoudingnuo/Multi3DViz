# Multi3DViz — 打包方案（Phase 7）

## 挑战

Multi3DViz 是 **Electron（前端壳）+ Python sidecar（算法后端）** 的混合架构。Electron 用 `electron-builder` 打 exe 很成熟，但 Python sidecar 不能简单打进 electron 包——它依赖 Open3D/PyTorch/numpy 等大依赖（GB 级），需要单独处理。

## 三种 Python 分发方案

| 方案 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A. PyInstaller 冻结** | `pyinstaller --onedir backend/main.py` 把 Python+依赖冻结成一个文件夹，electron-builder 把它作为 extraResources 打进 exe | 自包含、用户无需装 Python | 包体大（Open3D+PyTorch≈2-3GB）、首次冻结慢 |
| **B. 嵌入式 Python + pip** | 随包分发 Python embeddable + requirements，首次启动自动 `pip install` | 包体小（只发 wheel） | 首次启动慢（下载 GB）、需联网 |
| **C. 独立 Python 环境（当前开发态）** | 假设用户已装好 Python+依赖，只发 Electron 壳 | 最简单 | 不可交付（企业用户不会装） |

**推荐：A（PyInstaller 冻结）** —— 这是企业桌面软件的标准做法，自包含、离线可用。

## 推荐架构（方案 A）

```
Multi3DViz-Setup.exe (electron-builder + Inno Setup)
└── Multi3DViz/
    ├── Multi3DViz.exe          ← Electron 壳
    ├── resources/
    │   ├── app.asar            ← 前端 + electron/main.js
    │   └── backend/            ← PyInstaller 冻结的 Python sidecar
    │       ├── main.exe        ← Python 入口
    │       └── _internal/      ← Open3D/PyTorch/numpy/... DLL + .pyd
    └── ...
```

Electron 的 `main.js` 把 `resolvePython()` 改成指向 `resources/backend/main.exe`（冻结后的入口），不再找 venv。

## 实施步骤

### 1. 生成 PyInstaller spec
```bash
# 在项目根，用 .venv 的 python
.venv/Scripts/python.exe -m pip install pyinstaller
.venv/Scripts/python.exe -m PyInstaller --onedir --name m3v_backend \
    --add-data "backend/lib;backend/lib" \
    --add-data "backend/plugins;backend/plugins" \
    --add-data "backend/core;backend/core" \
    --hidden-import open3d --hidden-import torch --hidden-import scipy \
    --hidden-import paramiko --hidden-import websockets \
    backend/main.py
```
产物：`dist/m3v_backend/m3v_backend.exe` + `dist/m3v_backend/_internal/`。

### 2. electron-builder 配置（package.json 加 build 段）
```json
"build": {
  "appId": "com.multi3dviz.app",
  "productName": "Multi3DViz",
  "files": ["frontend/**", "electron/**"],
  "extraResources": [{ "from": "dist/m3v_backend", "to": "backend" }],
  "win": { "target": "nsis", "icon": "build/icon.ico" },
  "nsis": { "oneClick": false, "allowToChangeInstallationDirectory": true }
}
```

### 3. 改 main.js 的 resolvePython()
```js
function resolvePython() {
  // 打包后：resources/backend/m3v_backend.exe
  const packed = process.resourcesPath && path.join(process.resourcesPath, 'backend', 'm3v_backend.exe');
  if (packed && fs.existsSync(packed)) return packed;
  // 开发态：项目 venv
  const venvPy = path.join(ROOT, '.venv', 'Scripts', 'python.exe');
  if (fs.existsSync(venvPy)) return venvPy;
  return 'python';
}
// 同时 backend entry 改为：packed ? packed : path.join(ROOT,'backend','main.py')
```

### 4. 代码签名（避免 SmartScreen 拦截）
- 用代码签名证书给 exe 签名（`signtool` 或 electron-builder 的证书配置）
- 企业部署必需，否则 SmartScreen 会警告

## 已知坑

1. **PyTorch 体积**：CUDA 版 PyTorch 单独就 2GB+。考虑改 ONNX Runtime 推理（PRODUCT_GOAL 里提过），能把 UNet 依赖从 GB 降到百 MB。已有 `unet_sem.onnx` 可用。
2. **Open3D DLL**：PyInstaller 要正确收集 Open3D 的 DLL/资源，可能需要 `--collect-all open3d`。
3. **模型权重**：31MB 的 `unet_sem.pt` 要么打进包，要么首次启动下载。
4. **WHEA BSOD 防护**：`backend/main.py` 开头设 `OMP_NUM_THREADS` 的逻辑在冻结后仍生效（在 import open3d 前设环境变量）。

## 当前状态（控制端 exe）

- ✅ 开发态可运行（`npm start`）
- ✅ PyInstaller spec：`m3v_backend.spec`（含 plugins/core/lib 数据 + hidden imports + unet_sem.pt）
- ✅ electron-builder 配置已在 `package.json`（nsis target + extraResources 打 backend）
- ✅ electron/main.js 已支持冻结态路径（`PACKAGED_BACKEND` + `resolvePython`）
- ✅ **冻结实测通过**（2026-07-13）：`m3v_backend.exe` 58MB + `_internal/` 7.7G，`discovered 8 plugins`、`READY ws://127.0.0.1:PORT` 正常
  - 修了 PyInstaller 6.21 的三个坑：pywin32 hook 崩（excludes）、onnx.reference import 段错误（excludes）、`collect_data_files` 2-tuple 与 COLLECT 归一化冲突（datas 合并到初始 list）
  - 修了 `backend/main.py` 的 frozen 路径：`sys._MEIPASS` + `backend` 子目录（冻结后插件发现从 0 → 8）
- ⏳ NSIS 安装包生成中（electron-builder 打 7.7G extraResources，耗时 20-30min）
- ⏳ 代码签名证书未申请（企业部署需签名避免 SmartScreen 拦截）

### 控制端 exe 实测记录（2026-07-13）

冻结 + electron-builder 全流程在本机跑了，**NSIS 安装包失败，但免安装版可用**：

| 步骤 | 结果 |
|---|---|
| PyInstaller 冻结 | ✅ `m3v_backend.exe` 56MB + `_internal/` 7.7G，`discovered 8 plugins` |
| win-unpacked 打包 | ✅ `Multi3DViz.exe` 181MB + `resources/backend/m3v_backend.exe`，总 7.9G |
| Electron 壳启动 | ✅ `[main] spawning backend: ...resources\backend\m3v_backend.exe`，READY 正常 |
| **桌面快捷方式** | ✅ `C:\Users\Z790\Desktop\Multi3DViz.lnk` → 双击即启动 |
| **NSIS 安装包** | ❌ 失败：`makensis.exe` 是 32 位，7.9G payload 超过地址空间（`failed creating mmap of .nsis.7z`） |

**NSIS 失败的根因**：Open3D + PyTorch(CUDA) 把 backend 撑到 7.7G，32 位 NSIS makensis.exe 最多寻址 ~2-3G，无法 mmap 这么大的 7z。

**可行方案**（按推荐度）：
1. **免安装版（当前已用）**：win-unpacked 目录 + 桌面快捷方式。零安装，直接跑。适合开发/现场。
2. **自解压 7z SFX**：`7z a -sfx Multi3DViz.exe dist/win-unpacked/` 生成自解压包（~3G 压缩后），用户双击解压到任意位置。比 NSIS 限制小（7z 是 64 位）。
3. **瘦身 backend**：把 PyTorch CUDA 换成 CPU-only 或 ONNX Runtime（PACKAGING.md 已知坑里提过），backend 从 7.7G 降到 ~500M，NSIS 就能打。这是长期正解。
4. **Inno Setup**：用 Inno Setup（64 位）替代 NSIS，无 2G 限制。electron-builder 不直接支持，要手写 .iss。

当前现场用方案 1（桌面快捷方式 → win-unpacked），分发后续转方案 2 或 3。

### 控制端 exe 已知坑（实测）

| 问题 | 原因 | 解决 |
|---|---|---|
| `pythoncom.__file__` AttributeError | venv 里 pywin32 没跑 postinstall，pywintypes DLL 缺 | spec excludes 加 pywin32/pythoncom/pywintypes/win32* |
| `onnx.reference` 子进程段错误 (exit 3221225477) | PyInstaller import probe 撞 onnx | spec excludes 加 onnx/onnxruntime |
| COLLECT `not enough values to unpack` | PyInstaller 6.21 的 `a.datas += collect_data_files(...)` 拼接 2-tuple 破坏归一化 | 把 collect_data_files 结果合并进初始 `datas=[...]` 而非事后 `+=` |
| `discovered 0 plugins`（冻结后） | `__file__` 在冻结态是 exe 路径，sys.path 没指向 `_internal/backend/` | main.py 识别 `sys.frozen`，path 指向 `sys._MEIPASS` + `/backend` |

### 控制端 exe 构建步骤

```bash
# 1. 冻结 Python sidecar（产物 dist/m3v_backend/）
.venv\Scripts\python.exe -m pip install pyinstaller
npm run pack:backend            # = PyInstaller m3v_backend.spec --noconfirm

# 2. 打 NSIS 安装包（产物 dist/Multi3DViz-Setup-x.y.z.exe）
npm run dist                    # = electron-builder --win
```

---

## 受控端 deb 打包（Ubuntu arm64）

受控端现在也是**桌面 App**（Electron 窗口 + m3v_agent），和控制端同技术栈。打 `.deb` 装到机器人（Unitree/Agibot，Ubuntu arm64）。

### 架构

```
m3v-agent_<ver>_arm64.deb
├── /opt/m3v-agent/                       安装根
│   ├── m3v_agent/                        Python 包（extraResources）
│   ├── templates/                        两套 config.yaml
│   ├── packaging/deb/                    postinst/prerm/service
│   └── requirements.txt
├── /opt/m3v-agent-shell/                 Electron 壳（electron-builder 产物）
│   ├── m3v-agent                         主可执行（Electron）
│   └── resources/
├── /usr/share/applications/m3v-agent.desktop   桌面菜单项
└── /etc/m3v-agent/config.yaml            默认配置（postinst 从 templates 拷）
```

### IPC：Electron 壳 ↔ m3v_agent

桌面 App **不开 HTTP 服务器**，用 stdout/stdin 标签协议（最干净）：

```
m3v_agent (子进程, --ui-stdio)
   stdout → READY: {mode, driver}      启动成功
            STATE: {robot, recorder, transport, executor, ...}   1Hz 状态
            ESTOP_ACK: {ok}             急停响应
            DYING: {}                   退出前
   stdin  ← ESTOP                       触发急停
            STOP                        优雅退出
```

Electron main.js 解析 stdout 标签行 → IPC 推给 renderer（`shell/index.html`，ZCode 风格面板）。急停按钮 → main.js 写 stdin。日志走 stderr，不污染 stdout IPC 通道。

### 受控端 deb 构建步骤

**在 Linux 机器上跑**（机器人本身或 arm64 CI——electron-builder 需要真实 Linux 文件系统组装 deb）：

```bash
cd robot_side
./packaging/build-deb.sh              # 本机架构
# 或交叉编译 arm64：
./packaging/build-deb.sh --arm64
# 产物：dist/m3v-agent_<ver>_arm64.deb
```

### 在机器人上安装

```bash
sudo dpkg -i dist/m3v-agent_*.deb
sudo apt-get install -f               # 解决缺失依赖（python3-numpy 等）

# 编辑配置（设 Windows IP/user）
sudo vim /etc/m3v-agent/config.yaml

# 两种运行方式（二选一）：
#   A. 桌面 App（有显示器）：菜单里点 "m3v-agent 受控端"，或：
#      /opt/m3v-agent-shell/m3v-agent
#   B. 无头服务（systemd，无 GUI）：
sudo systemctl enable --now m3v-agent.service
```

### 两种运行模式

| 模式 | 启动方式 | 适用场景 |
|---|---|---|
| **桌面 App** | 菜单 / `/opt/m3v-agent-shell/m3v-agent` | 有显示器，想看状态面板 + 急停按钮 |
| **无头服务** | `systemctl enable --now m3v-agent.service` | 无显示器/常驻部署，状态走控制端 Multi3DViz 的 robot panel |

> ⚠️ 不要同时跑两种——两者都会启 m3v_agent 子进程，会重复录制/抢运动控制。deb 的 prerm 会停 systemd 服务；桌面 App 退出时杀自己的子进程。

### 受控端 deb 实测记录（2026-07-13，Unitree Go2 真机）

在 `unitree@10.60.77.187`（Ubuntu 20.04 aarch64 Tegra）上完整测试：

| 步骤 | 结果 |
|---|---|
| `build-deb-simple.sh` 构建 | ✅ 39K deb（dpkg-deb，不依赖 electron-builder 下载） |
| `sudo dpkg -i` 安装 | ✅ `Status: install ok installed`（postinst 加 90s 超时避免 Tegra 上 pip 慢卡死） |
| 文件布局 | ✅ `/opt/m3v-agent/` + `/etc/m3v-agent/config.yaml` + `/etc/systemd/system/m3v-agent.service` + `/usr/share/applications/m3v-agent.desktop` |
| `systemctl start` | ✅ agent 启动，`agent ready`，web 面板 `http://0.0.77.187:8765/` |
| Windows 控制端访问 web 面板 | ✅ `GET /api/state` 返回正确 JSON（mode/robot/recorder） |
| `--ui-stdio` IPC 协议 | ✅ READY + STATE + DYING 标签行正确，日志走 stderr |
| rospy（ROS1 模式） | ✅ `rospy.topics: topicmanager initialized`（service 加 `source /opt/ros/noetic/setup.bash` 后可用） |
| recorder 等待 roscore | ⏳ 卡在 "master may not be running"（FAST-LIO pipeline 未拉起，预期行为） |
| unitree_sdk2py driver | ❌ 未装（Unitree 运动控制实际走 go2_bridge TCP 桥，不走 unitree_sdk2py——下一轮换 TCP 桥 driver） |

**真机探查发现**（影响代码调整，下一轮）：
- Unitree 是 **ROS1 Noetic**（不是文档假设的 ROS2），用 **Livox MID360** 雷达 + **FAST-LIO**（`mapping_mid360.launch`）。config 模板已改为 `ros: ros1`。
- Unitree 已有验证过的完整 pipeline：`/home/unitree/sda2/restart_all.sh`（livox+fastlio+grid_map+record+bridge）+ `go2_search.py`（frontier 探索）+ `go2_bridge_ros2.py`（ROS1↔ROS2 桥连 Go2 硬件）+ `go2_record.py`（录制+SCP 同步）。
- 运动控制路径：`Go2TcpClient`（TCP localhost:21520）→ `go2_bridge_ros2.py`（ROS2 Foxy + CycloneDDS）→ Go2 硬件。**不走 unitree_sdk2py 直连**。
- 受控端 driver 需重写为 TCP 桥版（替换当前错误的 unitree_sdk2py driver）。

## 便携模式（可选）

安装包之外，可提供免安装便携版：把整个 `Multi3DViz/` 目录打包成 zip，配置存 exe 同目录。企业用户可放 U 盘多机使用。electron-builder 的 `portable` target 直接支持。
