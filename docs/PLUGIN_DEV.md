# Multi3DViz — 插件开发指南

Multi3DViz 的所有功能都以**插件**形式存在。本指南讲怎么写一个新插件。

## 四类插件

| 类型 | 基类 | 何时运行 | 产出 |
|---|---|---|---|
| **DataSource** | `DataSourcePlugin` | 每 tick | 往数据总线 `ctx.data.publish(robot_id, frame)` 推传感器帧 |
| **Display** | `DisplayPlugin` | 每 tick | 产出 `SceneUpdate`（3D/2D 可视化对象） |
| **Tool** | `ToolPlugin` | 事件驱动 | 响应前端点击（`on_scene_click`/`on_grid_click`） |
| **Service** | `ServicePlugin` | 每 tick（后台计算/控制） | 可产出 `SceneUpdate`，也可纯控制（SSH/ICP） |

## 最小 Display 插件示例

在 `backend/plugins/display/` 下新建 `my_marker.py`：

```python
from core.plugin_base import DisplayPlugin, SceneUpdate, SceneObject

class MyMarkerDisplay(DisplayPlugin):
    name = "MyMarker"            # 唯一标识，前端用它 enable/disable
    category = "display"         # 必须匹配所在目录
    description = "在原点放一个红色立方体"
    default_enabled = False

    properties = {               # 用户可调，前端自动生成属性面板
        'size': {'type': 'float', 'default': 0.5, 'min': 0.1, 'max': 5.0,
                 'step': 0.1, 'label': 'Size (m)', 'group': 'Geometry'},
        'color': {'type': 'select', 'options': ['red', 'green', 'blue'],
                  'default': 'red', 'label': 'Color', 'group': 'Geometry'},
    }

    def update(self, dt):
        # 每 tick 被调用。返回 SceneUpdate 或 None。
        s = self.get('size', 0.5)
        col = {'red':[1,0,0], 'green':[0,1,0], 'blue':[0,0,1]}[self.get('color','red')]
        obj = SceneObject(
            id='my_marker',
            kind='box',
            payload={'size':[s,s,s], 'color':col, 'pose':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]},
        )
        upd = SceneUpdate()
        upd.update.append(obj)   # update-on-missing-id 前端当 add 处理
        return upd
```

重启应用 → 插件自动出现在左侧面板 → 勾选启用 → 原点出现红立方体 → 改属性面板的 size/color 实时生效。

## SceneObject 的 kind

| kind | payload 字段 | 前端渲染 |
|---|---|---|
| `points` | positions(N×3), colors?(N×3), point_size | THREE.Points（二进制帧） |
| `mesh` | positions, indices, colors? | THREE.Mesh |
| `box` | size[3], color[3], pose(4×4) | THREE.Mesh BoxGeometry |
| `line` | positions(L×3), color[3], width | THREE.Line |
| `label` | text, position[3], color? | CSS 标签 |
| `grid2d` | cells(H×W int8), origin[2], resolution | 2D canvas |

> 点云和 mesh 走**二进制 WS 帧**（Float32 数组），不走 JSON——这是性能关键。

## 属性 schema 类型

`type` 字段决定前端生成的控件：`float`/`int`（滑块）、`select`（下拉）、`bool`（开关）、`string`（文本）、`path`（文件路径）、`robot_ref`（机器人选择）。

## DataSource 插件

```python
class MySource(DataSourcePlugin):
    name = "MySource"
    category = "source"
    default_enabled = False
    properties = {'robot_id': {'type':'string','default':'robot_c'}, ...}

    def update(self, dt):
        # 每 tick 读数据 → 推到总线
        self.ctx.data.publish(self.get('robot_id'), {
            'robot_id': self.get('robot_id'),
            'frame_idx': self._frame,
            'max_frame': self._max,
            'positions': points_array,   # (N,3) float
            'colors': colors_array,      # (N,3) float 可选
            'odom': {...},               # 可选
        })
```

Display 插件通过 `self.ctx.data.latest(robot_id)` 读最新帧。多个 Display 可消费同一个 Source。

## Service 插件（后台计算/控制）

Service 和 Display 一样每 tick 运行，但语义上是"自主计算/控制"，不是"可视化"。例：ICPRegistration、DualAgentExplorer、SSHLauncher。Service 也可以产出 SceneUpdate（如 explorer 的 overlay）。

## 插件发现机制

`backend/plugins/<category>/<name>.py` 自动扫描注册。要求：
- 文件在正确的 category 子目录（`source`/`display`/`tool`/`service`）
- 类继承对应的 `*Plugin` 基类
- 类的 `category` 属性 == 目录名
- `__module__` 是定义它的模块（不被误当成导入的基类）

## 访问后端能力

通过 `self.ctx`（PluginContext）：
- `ctx.data` — DataBus（DataSource 写 / Display 读）
- `ctx.robots` — RobotManager（SSH 连接池）
- `ctx.icp_ref` / `ctx.explorer_ref` — 跨服务松耦合引用（读 ICP transform / explorer 栅格）
- `ctx.emit(SceneUpdate)` — tick 外推送场景更新（如异步计算完成时）

## 调试

后端日志打到 stderr（Electron 主进程可见）。`log = logging.getLogger("multi3dviz.service.xxx")` 然后用 `log.info/warning`。前端 `console.log` 通过 Electron 的 console-message 转发。

## 不要做什么

- ❌ 插件里直接 `import` 前端模块（前后端隔离）
- ❌ 在 `update()` 里做阻塞 I/O（用后台线程 + `ctx.emit`）
- ❌ 依赖其他插件的具体类（用 `ctx.xxx_ref` 松耦合）
