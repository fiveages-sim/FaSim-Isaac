# 层2.5 旁路采集 —— 使用说明

> 编排当主干（闭环、高成功率），Replicator 当工具（出相机数据）。
> 不额外起 ROS 通信、不接管控制 —— 只挂在同一个 Isaac 进程里"看"。

## 一分钟上手

```bash
# 1) 安装扩展（只需一次；Isaac 升级后重跑）
cd ~/FaSim-Isaac && ./init.sh        # 选 isaac_local

# 2) 启动仿真（和平常一样）
./run.sh

# 3) 另开终端：交互式开始采集
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
./scripts/sdg.sh capture

# 4) 终端跑编排（和以前完全一样）

# 5) 结束
./scripts/sdg.sh stop
```

`capture` 会依次问你：机器人 → 任务 → 相机 → 采集频率 → 分辨率 →
深度 → 输出目录。全部有默认值，一路回车即可。

随时看进度：

```bash
./scripts/sdg.sh status
# [sdg] 采集中: 312 帧 / 3120 物理步
# [sdg]   相机 ['head']  every=10  错误 0
# [sdg]   输出 /home/xxx/fasim_data/unitree_g1d/sinopec_test_tube/20261110-1102
```

## 架构

```
┌─────────────────────────────────────────────────────────────┐
│ 终端（用户只碰这层）                                          │
│   ./scripts/sdg.sh capture|status|stop|doctor               │
│   用系统 python3，不需要 source ROS、不需要 Isaac python      │
└──────────────────────┬──────────────────────────────────────┘
                       │  JSON over Unix socket
                       │  <data>/.runtime/capture.sock
┌──────────────────────▼──────────────────────────────────────┐
│ Isaac Sim 进程（扩展随启动自动加载）                          │
│   fasim/replicator/sdg/scripts/extension.py                 │
│     on_startup → 监听 socket（挂在 app.update 事件上）        │
│                 ↓ 收到 start 指令                            │
│   scripts/live_capture.py                                   │
│     挂 physics step 回调 → get_data() → np.savez             │
└─────────────────────────────────────────────────────────────┘
```

**为什么用 socket 而不是控制文件**

早期版本用 `echo start > control.txt` 轮询文件。改动原因：

| | 文件轮询 | Unix socket |
|---|---|---|
| 用户能否知道成功 | ❌ 要去看 Isaac 终端日志 | ✅ CLI 同步拿到响应 |
| 残留误触发 | ⚠️ 有 | ✅ 无 |
| 实时状态 | ❌ 只能看日志 | ✅ `status` 查帧数 |

**兼容**：文件方式保留（`<data>/.runtime/control.txt`），老脚本可用。

## 路径与配置

所有路径都能用环境变量覆盖，**代码里没有绝对路径**：

| 变量 | 默认 | 说明 |
|---|---|---|
| `FASIM_SDG_DATA` | `~/fasim_data` | 采集数据根目录 |
| `FASIM_SDG_RUNTIME` | `<data>/.runtime` | socket / status 目录 |
| `FASIM_SDG_HOME` | 自动探测 | 扩展根目录 |
| `SDG_PYTHON` | 自动找 | CLI 用的 python（需要 PyYAML） |
| `ISAACSIM_DIR` | `~/isaacsim` | render/record 子命令需要 |

输出目录结构：

```
<data>/<robot>/<task>/<时间戳>/
├── frames/
│   └── frame_000000.npz
└── session.json
```

## 每帧内容

| key | 形状 | 说明 |
|---|---|---|
| `<cam>_rgb` | (H, W, 4) uint8 | RGBA |
| `<cam>_depth` | (H, W) float32 | `distance_to_image_plane`，单位 m |
| `<cam>_size` / `<cam>_prim` | — | 分辨率 / prim 路径 |
| `joint_pos` | (1, 32) float32 | 关节角（action 标签）|
| `joint_names` | (32,) | DOF 名字 |
| `sim_step` | — | 物理步计数（**时间轴唯一依据**）|
| `wall_time` | — | 墙钟（仅诊断用，**不要作为时间轴**）|

## 数据的时间轴可靠性

采集由**物理步**驱动，与墙钟无关：

```python
subscribe_physics_step_events(cb)   # 每物理步回调
if _S["step"] % every != 0: return  # 按物理步降采样
```

实测（177Hz 机器，`every=20`，371 帧）：

```
sim_step 间隔: mean=20.00  min=20  max=20   ← 恒定
```

所以**物理跑不到 200Hz 不影响数据正确性**，只影响采集速度。

## 调参与开销

| 参数 | 建议 | 说明 |
|---|---|---|
| `every` | 10（默认） | 每 N 物理步采一帧。177Hz 下 every=10 ≈ 17fps |
| 分辨率 | 640×480 / 320×240 | 320×240 约省 3/4 磁盘与渲染 |
| `depth` | true | 不要深度可省一半磁盘 |
| 相机 | head / 全部 | 每多一个相机开销成倍 |

每帧约 2.4MB（640×480 RGB+depth）。长时间采集注意磁盘。

**渲染会拖慢物理**（实测 640×480+采集时物理降 ~10%）。若控制受影响，
把 `every` 调大（20 / 40）。

## 实现要点（踩过的坑）

**① Kit 内不能调 `rep.orchestrator.step()`**

```
OrchestratorError: Synchronous call to `step` can only be performed in a
standalone workflow and may not be made from within Kit.
```

层2 能调是因为 `SimulationApp(headless=True)` 属于 standalone。Kit 里渲染
由 tick 图跟着 `app.update()` 自动驱动，回调里只 `get_data()`。

**② 存盘用 `np.savez` 不用 `savez_compressed`**

压缩一帧 640×480 RGB-D 要 52ms（比渲染还久），无压缩只要 1.3ms（快 40 倍）。

**③ 绝不在回调里开 `while` / `sleep`**

会饿死 Kit 主循环 → 物理停住 → OCS2 卡死。全部用回调。

**④ 回调异常必须打 traceback**

否则表现为"物理步数在涨、帧数一直是 0"，极难定位。错误计数写进
`session.json` 的 `capture_errors`。

## 故障排查

先跑自检：

```bash
./scripts/sdg.sh doctor
```

```
扩展根目录 : /path/to/exts/fasim.replicator.sdg
  存在      : True
数据根目录 : /home/xxx/fasim_data
指令 socket: /home/xxx/fasim_data/.runtime/capture.sock
  存在      : True
  响应      : {'ok': True, 'armed': False}
```

| 现象 | 原因 | 处理 |
|---|---|---|
| `socket 不存在` | 仿真没启动 / 扩展没装 | 跑 `./init.sh`；看 Isaac 终端有没有 `[sdg]` 日志 |
| `扩展响应超时` | Isaac 卡住 / 主循环被占 | 看 Isaac 终端；检查是否有 sync 阻塞代码 |
| `frames_saved: 0` | timeline 没 PLAY | 仿真要真正在跑 |
| 磁盘爆 | 每帧 2.4MB | 降分辨率 / `--no-depth` / 调大 `every` |

## 与层2（离线回放）的区别

| | 层2 回放 | 层2.5 旁路采集 |
|---|---|---|
| 控制 | 开环重放关节角 | **闭环，编排真跑** |
| 物理 | 重演，可能偏差 | **真实，成功率 = 编排成功率** |
| 精细操作 | 做不到（放孔失败） | **能做到** |
| 需要 npz | 是 | 不需要 |
| 速度 | 快 | 受编排速度限制 |

层2 仍有用：批量出数据、验证轨迹物理可行性、作为移植对照组。
