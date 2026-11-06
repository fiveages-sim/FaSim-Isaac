# 层2.5：在运行的编排里旁路采集（新路线验证）

> **目标**：编排当主干（闭环、高成功率），Replicator 当工具（出相机数据）。
> 不额外起 ROS 通信、不接管控制 —— 只挂在同一个 Isaac 进程里"看"。
>
> **不需要 Script Editor**：扩展随 Isaac 自动加载，用控制文件触发。

## 为什么可行

`ROS=fiveages` 变体用的是 `fasim.ros2.nodes.ROS2ControlLaunch`，
**ros2_control 跑在 Isaac 进程内**，和渲染循环共享同一个 `app.update()`
时钟。所以不存在"ROS 时间戳和 Isaac 时间戳不一致"的问题 —— 它们本来就是
同一个时钟。

## 一次性安装

扩展走和 `fasim.ros2.nodes` / `fasim.steering.nodes` 完全一样的安装路径：

```bash
cd /home/fiveages/FaSim-Isaac
./init.sh          # 选 isaac_local；会调用 install_fasim_replicator_sdg
```

它只做两件事（都不碰仓库跟踪的文件）：

1. 把 `exts/fasim.replicator.sdg` 软链到 `~/isaacsim/extsUser/`；
2. 在 `~/isaacsim/apps/*.kit` 的 `[dependencies]` 里加一行
   `"fasim.replicator.sdg" = {}`。

Isaac 升级后重跑一次即可。想手动撤销就删掉那个软链和 kit 里的那一行。

## 完整流程

### 1) 启动仿真（和以前完全一样）

```bash
cd /home/fiveages/FaSim-Isaac
./run.sh                      # GUI；物理 physx，ROS WS 选 ~/ros2_ws
```

加载场景、切 `ROS=fiveages` 变体、确认运控起来 —— 和平时一样。
扩展加载后控制台会打印：

```
[live_capture] ==================================================================
[live_capture] 旁路采集已就绪（文件触发，无需 Script Editor）
[live_capture] 控制文件: /home/fiveages/fasim_live/control.txt
[live_capture]   开始采集: echo start > /home/fiveages/fasim_live/control.txt
[live_capture]   结束采集: echo stop  > /home/fiveages/fasim_live/control.txt
[live_capture] ==================================================================
```

### 2) 开始采集（新开一个终端，一行命令）

```bash
echo 'start out=~/fasim_live/pick_01 every=20 camera=head' \
  > ~/fasim_live/control.txt
```

看到这条就说明挂上了：

```
[capture] 相机 head: 640x480 @ /World/G1D/G1_Body/torso_link/d435_link/head_camera
[capture] 已挂载 —— 输出 /home/fiveages/fasim_live/pick_01
```

### 3) 跑编排（终端，和以前完全一样）

```bash
cd /home/fiveages/lerobot_ros2/examples/IsaacSim/robots/Unitree/G1D/tools
motion-generation --robot unitree_g1d --task-key pick_tube_full --scene lift
```

采集在后台跟着物理步进走，不影响控制。

### 4) 结束

```bash
echo stop > ~/fasim_live/control.txt
```

输出：

```
[capture] 停止 —— 共 412 帧 / 4120 物理步 (20.6s)
[capture] 产物: /home/fiveages/fasim_live/pick_01
```

随时可以用 `echo status > ~/fasim_live/control.txt` 看进度。

## 控制文件命令

| 命令 | 作用 |
|---|---|
| `start` | 用默认参数开始 |
| `start every=20 camera=head out=~/fasim_live/p2` | 带参数开始 |
| `stop` | 结束并写 session.json |
| `status` | 打印当前进度 |

支持的参数：`out`(目录) / `every`(每 N 个物理步) / `camera`(逗号分隔) /
`width` / `height` / `depth`(true/false) / `record_joints`(true/false)。

## 产物

```
~/fasim_live/pick_01/
├── frames/
│   └── frame_000000.npz     每帧一个
└── session.json             相机参数、帧数、物理步数、错误数、耗时
```

每个 `frame_XXXXXX.npz` 含：

| key | 含义 |
|---|---|
| `head_rgb` | (480, 640, 4) uint8 RGBA |
| `head_depth` | (480, 640) float32，`distance_to_image_plane` |
| `head_size` / `head_prim` | 分辨率 / 相机 prim 路径 |
| `joint_pos` | (1, 32) 关节角 —— action 标签 |
| `joint_names` | 32 个 DOF 名字 |
| `frame` / `sim_step` / `wall_time` | 帧号 / 物理步 / 相对时间 |

## 实现要点（踩过的坑）

**Kit 进程内不能调 `rep.orchestrator.step()`**：

```
OrchestratorError: Synchronous call to `step` can only be performed in a
standalone workflow and may not be made from within Kit. Please use the
async function `step_async`.
```

层2 能调是因为 `SimulationApp(headless=True)` 属于 standalone workflow。
在 Kit 里渲染由 Replicator 的 tick 图跟着 `app.update()` **自动驱动**，
所以回调里只要 `get_data()` 取最近一帧。

实测：1000 物理步回调里取 33 帧，**33/33 互不相同**，不存在重复缓冲。

**存盘用 `np.savez` 而不是 `savez_compressed`**：压缩一帧 640×480 RGB-D
要 52ms（比渲染还久），无压缩 1.3ms（快 40 倍）。

## 性能（实测，RTX 4090 / 640×480 / 头相机 / 无头模式）

```
基线（不采集，纯 app.update）:   12.63 ms/步
采集 every=10（20fps）:          21.49 ms/步   增量 +8.85
采集 every=40（5fps）:           16.75 ms/步   增量 +4.12
```

**基线 12.63ms 是环境固有开销**（含 `ROS=fiveages` 的 ROS2ControlLaunch），
和采集无关。采集净增量 **4~9 ms/步**。

**存盘必须用 `np.savez` 而不是 `savez_compressed`**：实测压缩一帧
640×480 RGB-D 要 **52ms**，比渲染本身还久，会把物理步进拖到 45ms/步。
无压缩只要 1.3ms（快 40 倍），代价只是文件大一点。这一项优化把
整体耗时从 45.11 降到 23.49 ms/步，慢步（>100ms）从 66/600 降到 1/600。

渲染本身（`orchestrator.step`）约 19~25ms/次，按 `every` 摊销。

## 调参建议

| 现象 | 调法 |
|---|---|
| 编排跑得明显变慢 / 控制不稳 | `every` 调大（20 / 40） |
| 数据分辨率太高、文件太大 | `width=320, height=240` |
| 不需要深度 | `depth=False` |
| 想同时采手腕相机 | `camera="head,wrist_left,wrist_right"`（开销成倍） |

## 注意事项

- **绝不要**在 Script Editor 或任何回调里开 `while` / `time.sleep()` 循环 ——
  会饿死 Kit 主循环，物理停住、OCS2 卡死。本模块全部用回调，不阻塞。
- 回调里抛异常会打断物理步进，所以内部全部 catch。但**首次失败会把完整
  traceback 打到 stdout**（否则表现是"物理步数在涨、帧数一直是 0"，极难定位）。
  错误计数写进 `session.json` 的 `capture_errors`。
- 渲染由 tick 图自动驱动，采集频率完全由 `every` 控制。
- 采集器不管 timeline 状态，只跟物理步进。编排没在跑就不会有帧
  （`frames_saved=0` 时会明确警告）。

## 实测结果（full kit，2026-11-06）

```
[live_capture] 扩展 enabled = True
控制文件 start out=/tmp/live_e2e every=20 camera=head
[capture] 相机 head: 640x480
[capture] 已挂载
→ 采到 10 帧 / 203 物理步
→ 10 个唯一 hash（无重复帧）
→ rgb (480,640,4) mean=138.6，depth (480,640) float32，joint_pos (1,32)
→ capture_errors: 0
```

## 与层2（离线回放）的区别

| | 层2 回放 | 层2.5 旁路采集 |
|---|---|---|
| 控制 | 开环重放关节角 | **闭环，编排真跑** |
| 物理 | 重演，可能偏差 | **真实，任务成功率 = 编排的成功率** |
| 精细操作 | 做不到（放孔失败） | **能做到** |
| 需要 npz | 是 | 不需要 |
| 速度 | 快（无控制开销） | 受编排速度限制 |

层2 仍然有用：批量出数据、验证编排轨迹的物理可行性、作为移植的对照组。
