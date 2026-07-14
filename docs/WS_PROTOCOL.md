# Multi3DViz — WebSocket 协议参考

前端 ↔ 后端通过一条 WebSocket 连接通信。两种帧类型共享连接：

1. **JSON 文本帧** — 控制消息（请求/响应/事件）
2. **二进制帧** — 大数组（点云/网格/栅格）

## 连接握手

Electron 启动 Python sidecar → Python 绑定随机端口 → 打印 `READY ws://127.0.0.1:PORT` 到 stdout → Electron 读端口 → preload 注入 `window.M3V.wsUrl` → 前端连接。

连接后后端依次推送：`ready` → `catalog` → `state` →（自动启用默认插件后）`state`。

## JSON 消息格式

```
请求:    {"type": <msg_type>, "id": <req_id>, ...payload}
响应:    {"type": "response", "id": <req_id>, ...payload}
事件:    {"type": <event_type>, ...payload}    # 无 id，服务端主动推
错误:    {"type": "error", "id": <req_id>, "message": "..."}
```

`id` 用于请求-响应配对。前端 `ws.request(obj)` 返回 Promise，自动用 id 匹配。

## 前端 → 后端（请求）

| type | payload | 响应 | 说明 |
|---|---|---|---|
| `hello` | client, version | ok, server, version | 握手 |
| `list_plugins` | — | plugins[] | 插件目录 |
| `enable_plugin` | name | ok | 启用插件 |
| `disable_plugin` | name | ok | 禁用插件 |
| `set_property` | name, key, value | ok | 改插件属性 |
| `get_state` | — | enabled[] | 当前启用的插件+属性 |
| `playback` | action, value | ok | 播放控制（play/pause/toggle/seek/rate） |
| `robot_add` | robot_id, host, user, password, ... | ok | 添加机器人 |
| `robot_remove` | robot_id | ok | 删除机器人 |
| `robot_list` | — | robots[] | 机器人列表+状态 |
| `robot_command` | robot_id, action, value | ok, rc?, output? | SSH 命令（launch/stop/restart/run） |
| `register` | — | ok | 强制重跑 ICP |
| `set_target` | robot_id, world[x,y] | ok | 手动导航目标 |

## 后端 → 前端（事件）

| type | payload | 说明 |
|---|---|---|
| `ready` | — | 后端就绪 |
| `catalog` | plugins[] | 插件目录 |
| `state` | enabled[] | 启用插件变化 |
| `scene` | ops[] | 小场景操作（box/line/label/remove，纯 JSON） |
| `scene_binary` | layouts[] | **二进制帧头**（紧随其后一个二进制帧） |
| `playback_state` | sources[] | 播放状态快照（4Hz） |
| `registration_status` | state, fitness, rmse, ... | ICP 状态快照（2Hz） |
| `registration_progress` | phase, fitness, rmse, ... | ICP 每 trial 进度 |
| `robot_status` | robots[], changed | 机器人状态变化 |

## 二进制帧协议

`scene_binary` JSON 帧描述布局，**紧接着**一个二进制 WS 帧承载拼接的 float32/int8 数组：

```json
{"type":"scene_binary","layouts":[
  {"id":"robot_a_cloud","kind":"points","op":"update","n_points":9000,"has_colors":true,"point_size":0.04},
  {"id":"merged_grid","kind":"grid2d","op":"update","width":542,"height":449,"origin":[-2,-2],"resolution":0.05}
]}
```

二进制帧按 layouts 顺序拼接，前端按 meta 切片：

| kind | 二进制内容（little-endian） |
|---|---|
| `points` | positions: n×3 float32，colors?: n×3 float32（若 has_colors） |
| `mesh` | positions: nv×3 float32，indices: nt×3 uint32，colors?: nv×3 float32 |
| `grid2d` | cells: w×h int8 |

## 点云为什么走二进制

10⁵–10⁶ 个 float 走 JSON+base64 会膨胀 4×+ 且解析慢。二进制帧直接 `Float32Array(buf, offset, n*3)` 切片，零拷贝传给 Three.js BufferGeometry。

## 消息流（典型会话）

```
前端                          后端
  │── hello ──────────────────►│
  │◄── ready ─────────────────│
  │◄── catalog ───────────────│
  │◄── state ─────────────────│
  │── list_plugins ───────────►│
  │◄── response(plugins) ─────│
  │                            │ (自动启用默认插件)
  │◄── state ─────────────────│
  │◄── playback_state ────────│  (4Hz)
  │◄── scene_binary ──────────│  (每 tick)
  │◄── [binary] ──────────────│
  │── set_property ───────────►│
  │◄── response(ok) ──────────│
  │── robot_add ──────────────►│
  │◄── response(ok) ──────────│
  │◄── robot_status ──────────│  (心跳触发)
```
