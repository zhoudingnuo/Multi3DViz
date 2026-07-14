# 部署指南 (DEPLOYMENT.md)

逐步把受控端 agent 部署到两台机器人，并配好 Windows 控制端的接收通道。

---

## 0. 前置条件

| 组件 | 要求 |
|---|---|
| Windows 控制端 | Multi3DViz 已能跑（`backend/` + Electron），LocalReplaySource 默认读 `C:\Users\Z790\ccenter\<robot>\data\` |
| Windows OpenSSH Server | **必须开**（受控端 SCP 推送目标）。见 §1。 |
| Agibot 机器人 | SSH 可达，FAST-LIO ROS2（或 noetic 容器）已装，mc_sdk .so 在 `/home/orin-001/ZCodeProject/lib/zsl-1/aarch64/` |
| Unitree Go2 | SSH 可达（密码 `123`），ROS2 + FAST-LIO 已装，`unitree_sdk2py` 待装 |
| 网络 | 三台机器（Windows + 2 机器人）同一局域网或路由可达 |

---

## 1. Windows 开 OpenSSH Server（一次性）

受控端用 SCP 把数据推回 Windows，所以 Windows 必须开 SSH 服务端。

### 1.1 安装 + 启动

以**管理员**身份开 PowerShell：

```powershell
# 安装 OpenSSH Server
Add-WindowsCapability -Online -Name OpenSSH.Server~~~~0.0.1.0

# 启动服务 + 设开机自启
Start-Service sshd
Set-Service -Name sshd -StartupType Automatic

# 确认防火墙放行 22（安装时通常自动加规则）
Get-NetFirewallRule -Name *ssh*
# 若无：
New-NetFirewallRule -Name sshd -DisplayName 'OpenSSH Server (sshd)' `
  -Enabled True -Direction Inbound -Protocol TCP -Action Allow -LocalPort 22
```

### 1.2 配 SSH key（推荐，免密推送）

在**机器人上**生成 key 并推到 Windows（以 Agibot 为例）：

```bash
# 在 orin-001 上
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
# 推到 Windows（会问 Windows 密码）
ssh-copy-id -i ~/.ssh/id_ed25519.pub Z790@<windows-ip>
# 测试
ssh Z790@<windows-ip> "echo ok"
```

Unitree 同理（在 unitree 用户下做）。

### 1.3 Windows 上的接收目录

确保 `C:\Users\Z790\ccenter\` 存在且 `Z790` 用户可写（受控端会创建 `<robot>\data\run_*\` 子目录）：

```powershell
mkdir C:\Users\Z790\ccenter -Force
```

> 控制端 Multi3DViz 的 LocalReplaySource 默认 `data_root = C:\Users\Z790\ccenter`，所以 SCP 推到 `C:/Users/Z790/ccenter/<robot>/data/...` 正好被控制端扫到。

---

## 2. 部署到 Agibot (Robot B)

### 2.1 拷贝代码到机器人

```bash
# 在 Windows 上（或用 U 盘/git）
scp -r C:/Users/Z790/Multi3DViz/robot_side orin-001@<agibot-ip>:~/

# SSH 进去
ssh orin-001@<agibot-ip>
```

### 2.2 改配置

编辑 `robot_side/templates/agibot/config.yaml`，确认/改这几处：

```yaml
transport:
  host: <你的-Windows-IP>      # 例如 192.168.1.10
  user: Z790
  password: null                # null = 用 §1.2 配的 key
  remote_root: C:/Users/Z790/ccenter

recorder:
  ros: ros2                     # 或 ros1（如果在 noetic 容器内跑）

driver:
  agibot_local_ip: <本机-在机器人网段的-IP>   # 例如 192.168.234.18
  agibot_robot_ip: 192.168.234.1
  agibot_sdk_lib_path: /home/orin-001/ZCodeProject/lib/zsl-1/aarch64
```

> **Agibot SDK 网络要点**：机器人的 `sdk_config.yaml` 的 `target_ip` 必须指向 `agibot_local_ip`（控制电脑的 IP）。改完 sdk_config 要重启机器人。详见 `agibot控制文档.md` §2.4。

### 2.3 一键部署

```bash
cd ~/robot_side
sudo ./templates/agibot/deploy.sh
```

脚本会：装依赖 → pip install -e . → 装 config + systemd unit → 启动服务。

### 2.4 验证

```bash
# 看日志
journalctl -u m3v-agent-agibot -f

# 应看到:
#   agibot sdk initialized (checkConnect=True) ...
#   recorder started: run_dir=/home/orin-001/m3v_data/agibot/data/run_...
#   scp connected to Z790@<windows-ip>
#   pushed 1 frames ...
#   status panel: http://0.0.0.0:8765/

# Windows 上确认收到数据
dir C:\Users\Z790\ccenter\agibot\data\run_*\cloud_registered\
# 应有 000000.npy, 000001.npy, ...
```

**打开状态面板**（浏览器，任意机器）：

```
http://<agibot-ip>:8765/
```

应看到 ZCode 风格深色面板：机器人卡（绿色连接态）/ 录制卡（帧数实时增长）/ 回传卡（pushing）/ 执行卡（当前目标坐标）。底部红色 **紧急停止** 按钮可一键趴下。

---

## 3. 部署到 Unitree Go2 (Robot A)

### 3.1 拷贝 + SSH

```bash
scp -r C:/Users/Z790/Multi3DViz/robot_side unitree@<go2-ip>:~/
ssh unitree@<go2-ip>     # 密码 123
```

### 3.2 改配置

编辑 `robot_side/templates/unitree/config.yaml`：

```yaml
transport:
  host: <你的-Windows-IP>
  user: Z790
  password: null
  remote_root: C:/Users/Z790/ccenter

driver:
  unitree_network_iface: eth0     # 能到 Go2 内部运动控制器的网卡
```

### 3.3 装 unitree_sdk2py（Go2 上默认没有）

```bash
# 在 Go2 上
pip3 install unitree_sdk2py cyclonedds
```

详见 [UNITREE_SDK_NOTES.md](UNITREE_SDK_NOTES.md)。

### 3.4 一键部署

```bash
cd ~/robot_side
sudo ./templates/unitree/deploy.sh
```

### 3.5 验证

```bash
journalctl -u m3v-agent-unitree -f
```

状态面板：浏览器打开 `http://<go2-ip>:8765/`，同 §2.4。

---

## 4. 控制端验证（Windows Multi3DViz）

启动 Multi3DViz（`npm start` 或 Electron），在右侧机器人面板：

1. **添加 Agibot**：host=agibot-ip, user=orin-001, password 留空（key auth）, robot_id=robot_b。
   - 状态应变绿（online）。
2. **添加 Unitree**：host=go2-ip, user=unitree, password=123, robot_id=robot_a。
   - 状态应变绿。
3. 把两个 LocalReplay 实例的 `stream_mode` 打开，`data_root` 指向 `C:\Users\Z790\ccenter`。
4. 视口里应看到两台机器人的点云开始增长（实时流）。
5. 右侧 FAST-LIO 按钮可远程启停机器人上的 SLAM（走 SSHLauncher）。
6. Shift+点击视口某点 → 该机器人收到 `ccenter_target_*.txt` → 导航过去。

---

## 5. 排错

| 现象 | 检查 |
|---|---|
| SCP 推送失败 "auth failed" | Windows OpenSSH 是否开（§1.1）；key 是否配好（§1.2）；config 的 user/host 是否对 |
| Windows 收不到数据 | 防火墙 22 端口；`C:\Users\Z790\ccenter` 是否存在且可写；journalctl 看推送日志 |
| 控制端视口无点云 | LocalReplay 的 `data_root`/`robot` 是否和受控端推的路径对齐；`stream_mode` 是否开 |
| Agibot driver 连不上 | sdk_config.yaml 的 target_ip 是否指向 agibot_local_ip；改完要重启机器人 |
| Unitree driver 连不上 | `unitree_sdk2py` 装了吗；`unitree_network_iface` 选对网卡了吗 |
| 机器人不动 | executor 启动了吗（journalctl）；target 文件写进去了吗（`cat <target_path>`）；mode 是 explore 还是 stop |
| 机器人乱动 / 撞墙 | navigator 的 max_fwd/max_turn 调小；检查 odom 是否正确（pose 来自 odom cache） |

---

## 6. 卸载

```bash
# 机器人上
sudo systemctl stop m3v-agent-<agibot|unitree>
sudo systemctl disable m3v-agent-<agibot|unitree>
sudo rm /etc/systemd/system/m3v-agent-*.service
sudo rm -rf /opt/m3v-agent /etc/m3v-agent
sudo systemctl daemon-reload
```
