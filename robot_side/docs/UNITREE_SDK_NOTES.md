# Unitree Go2 SDK 速查 (UNITREE_SDK_NOTES.md)

受控端 `drivers/unitree_driver.py` 用官方 `unitree_sdk2py` 的 `SportClient`（高层运动接口，走 CycloneDDS）。这份文档是该 driver 的 API 依据 + 安装/排错参考。

> **关于 `go2-search`**：这是一个社区前沿探索项目，仓库非公开可见（多次 web 搜索 + 文件系统搜索均未找到 `zhoudingnuo/go2-search`）。它的底层用 `unitree_sdk2py` 的 SportClient + 自研前沿算法。受控端 driver 只管运动接口（接到坐标 → 走过去），前沿算法在控制端 Multi3DViz 的 `DualAgentExplorer` 里，所以直接用官方 SDK 即可，不依赖 `go2-search`。

---

## 一、安装

### 1.1 装 unitree_sdk2py + CycloneDDS

在 Go2 上：

```bash
pip3 install unitree_sdk2py cyclonedds
```

`unitree_sdk2py` 是官方 Python 绑定（`github.com/unitreerobotics/unitree_sdk2_python`）。`cyclonedds` 是它依赖的 DDS 中间件。

### 1.2 验证

```bash
python3 -c "from unitree_sdk2py.go2.sport.sport_client import SportClient; print('ok')"
```

### 1.3 网络接口

Go2 内部运动控制器通过 DDS 通信。受控端 driver 需要绑定到能到控制器的网卡：

```yaml
# config.yaml
driver:
  unitree_network_iface: eth0    # 改成实际网卡名
```

driver 会据此设 `CYCLONEDDS_URI`（见 `unitree_driver.py:_dds_uri`）。查网卡：

```bash
ip addr
# 找到连到 Go2 内部网络的那块
```

如果 driver 跑在 Go2 **本机**上（运动控制器在 loopback），iface 可能要设 `lo`。

---

## 二、SportClient API（driver 用到的）

### 2.1 初始化

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# domain 0 是 Go2 默认。第二参数是网卡名（可空）。
ChannelFactoryInitialize(0, "eth0")

client = SportClient()
client.SetTimeout(10.0)   # 秒
client.Init()
```

### 2.2 运动指令

| driver 方法 | SportClient 调用 | 说明 |
|---|---|---|
| `stand_up()` | `client.StandUp()` | 从趴下到站立（~2s） |
| `lie_down()` | `client.Prone()` | 趴下（电机锁定） |
| `move(vx,vy,yaw)` | `client.Move(vx, vy, yaw)` | 连续速度（m/s, m/s, rad/s） |
| `stop()` | `client.Move(0,0,0)` | 原地保持 |
| `emergency_stop()` | `client.StopMove()` + `client.Prone()` | 急停 |

### 2.3 速度范围（Go2 安全包络）

| 参数 | 含义 | 范围 | 单位 |
|---|---|---|---|
| `vx` | 前后速度 | -1.0 ~ 1.0（建议） | m/s |
| `vy` | 侧向速度 | -0.5 ~ 0.5 | m/s |
| `yaw` | 转向角速度 | -1.5 ~ 1.5 | rad/s |

> 受控端 navigator 的 `max_fwd=0.5` / `max_turn=1.0` 已经在安全包络内（config 可调）。

### 2.4 状态读取（driver 不直接用）

driver 的 `get_pose()` 从 recorder 的 odom cache 读 (x,y,yaw)，**不**走 SDK 的状态订阅。原因：

1. odom 已在录（FAST-LIO 发布 `/Odometry`），权威且和控制端看到的位姿一致。
2. SDK 状态订阅（`SportModeState` topic）需额外 DDS 订阅 + 回调，两个 driver 接口会不一致。
3. Agibot 的 SDK getter 有已知 bug（ctrlmode 恒 58），统一走 odom 避免类似坑。

如需直接读 SDK 状态（电量、关节角等），参考：

```python
from unitree_sdk2py.go2.sport.sport_client import SportClient
# 这些是查询接口（具体返回值看 SDK 版本）:
# client.GetState()        # SportModeState
# client.GetFootRaiseHeight()
# client.GetSpeedLevel()
```

---

## 三、CycloneDDS 配置

driver 用 `CYCLONEDDS_URI` 环境变量绑定网卡。生成的 XML：

```xml
<CycloneDDS><Domain><General>
  <NetworkInterfaceAddress>eth0</NetworkInterfaceAddress>
  <AllowMulticast>true</AllowMulticast>
</General></Domain></CycloneDDS>
```

如果默认配置不通（发现不了 Go2），可手写更完整的 `~/.cyclonedds.xml`：

```xml
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
      <MaxMessageSize>65500</MaxMessageSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="Go2的IP"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

然后 `export CYCLONEDDS_URI=file:///path/to/.cyclonedds.xml`。

---

## 四、排错

| 现象 | 原因/解决 |
|---|---|
| `ImportError: unitree_sdk2py` | `pip3 install unitree_sdk2py cyclonedds` 没装；或不在同一个 Python 环境 |
| `ChannelFactoryInitialize` 卡住 | 网卡名错（`unitree_network_iface`）；CycloneDDS 绑不到网卡 |
| `client.Init()` 超时 | DDS 发现不了 Go2；检查网络/防火墙；试 `AllowMulticast=true` |
| `Move` 无反应 | 没先 `StandUp`（driver 内部会自动调）；速度太小（< 死区）；Go2 处于保护态（电量低/温度高） |
| Go2 自己乱动 | driver 的 vx/vy/yaw 符号约定和 Go2 不一致——检查坐标系（X 前、Y 左、Z 上、yaw CCW） |

---

## 五、参考

- 官方 SDK: https://github.com/unitreerobotics/unitree_sdk2_python
- 官方 C++ SDK: https://github.com/unitreerobotics/unitree_sdk2
- 官方 ROS2 封装: https://github.com/unitreerobotics/unitree_ros2
- CycloneDDS: https://github.com/eclipse-cyclonedds/cyclonedds
