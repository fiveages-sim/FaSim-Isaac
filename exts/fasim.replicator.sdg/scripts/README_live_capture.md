# 层2.5：在运行的编排里旁路采集（新路线验证）

> **目标**：编排当主干（闭环、高成功率），Replicator 当工具（出相机数据）。
> 不额外起 ROS 通信、不接管控制 —— 只挂在同一个 Isaac 进程里"看"。

## 为什么可行

`ROS=fiveages` 变体用的是 `fasim.ros2.nodes.ROS2ControlLaunch`，
**ros2_control 跑在 Isaac 进程内**，和渲染循环共享同一个 `app.update()`
时钟。所以不存在"ROS 时间戳和 Isaac 时间戳不一致"的问题 —— 它们本来就是
同一个时钟。

## 完整流程

### 1) 启动仿真（和以前完全一样）

```bash
cd /home/fiveages/FaSim-Isaac
./run.sh                      # GUI；物理选 physx，ROS WS 选 ~/ros2_ws
```

等你平时跑编排时的那些准备工作都做完（加载场景、切到 `ROS=fiveages` 变体、
确认运控起来了）。

### 2) 挂载采集器（在 Isaac 的 Script Editor 里）

`Window > Script Editor`，粘贴执行：

```python
import sys
sys.path.insert(0, "/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
exec(open("/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/live_capture.py").read())

# 立刻返回，不会阻塞
arm_capture(
    out_dir="~/fasim_live/pick_01",
    every=10,          # 每 10 个物理步采 1 帧（200Hz → 20fps）
    camera="head",     # 只采头部相机；"" = 全部
)
```

看到这些输出就说明挂上了：

```
[capture] 相机 head: 640x480 @ /World/G1D/G1_Body/torso_link/d435_link/head_camera
[capture] 关节标签: 32 个 DOF
[capture] 已挂载 —— 输出 /home/fiveages/fasim_live/pick_01
[capture] 现在去终端跑 motion-generation；结束后回来执行 stop_capture()
```

### 3) 跑编排（终端，和以前完全一样）

```bash
cd /home/fiveages/lerobot_ros2/examples/IsaacSim/robots/Unitree/G1D/tools
# 你平时怎么跑 motion-generation 就怎么跑
motion-generation --robot unitree_g1d --task-key pick_tube_full --scene lift
```

采集在后台跟着物理步进走，不影响控制。

### 4) 结束

回到 Script Editor：

```python
stop_capture()
```

输出：

```
[capture] 停止 —— 共 412 帧 / 4120 物理步 (20.6s)
[capture] 产物: /home/fiveages/fasim_live/pick_01
```

## 产物

```
~/fasim_live/pick_01/
├── frames/
│   └── frame_000000.npz     每帧一个
└── session.json             相机参数、帧数、物理步数、耗时
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

- **绝不要**在 Script Editor 里写 `while` / `time.sleep()` 循环 ——
  会饿死 Kit 主循环，物理停住、OCS2 卡死。本模块全部用回调，不阻塞。
- 回调里抛异常会打断物理步进，所以内部全部 catch 并只 warn。
- `orchestrator.step()` 是同步渲染。若发现控制受影响，先把 `every` 调大，
  再考虑降分辨率。
- 采集器不管 timeline 状态，只跟物理步进。编排没在跑就不会有帧
  （`frames_saved=0` 时会明确警告）。

## 与层2（离线回放）的区别

| | 层2 回放 | 层2.5 旁路采集 |
|---|---|---|
| 控制 | 开环重放关节角 | **闭环，编排真跑** |
| 物理 | 重演，可能偏差 | **真实，任务成功率 = 编排的成功率** |
| 精细操作 | 做不到（放孔失败） | **能做到** |
| 需要 npz | 是 | 不需要 |
| 速度 | 快（无控制开销） | 受编排速度限制 |

层2 仍然有用：批量出数据、验证编排轨迹的物理可行性、作为移植的对照组。
