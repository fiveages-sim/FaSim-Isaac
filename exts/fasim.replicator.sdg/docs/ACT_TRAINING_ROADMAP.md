# ACT 训练与推理路线（sdg 数据）

日期：2026-11-12
适用：lerobot **v0.5.1**（submodule，2026-04-07）

---

## 0. 环境划分（重要）

仓库用两个 venv，**numpy 版本冲突**是拆分的唯一原因：

| 环境 | numpy | lerobot | 用途 |
|---|---|---|---|
| `.venv` | **1.26.4** | 0.3.4（旧 submodule editable） | 编排 / 采集 / 导出 / 数据处理 |
| `.venv-train` | **2.0~2.2** | **0.5.1（PyPI）** | 只跑训练与推理 |

`lerobot 0.5.x` 强制 `numpy>=2.0.0,<2.3.0`；而系统 `cv_bridge` 按 numpy 1.x ABI
编译，升 2.x 会破坏头相机采集。所以：

- **采集/导出用 `.venv`**（要 ROS）
- **训练/推理用 `.venv-train`**（不要 ROS）

> ⚠️ `.venv-train` **不要** `source /opt/ros/jazzy/setup.bash` ——
> 那会把系统 numpy 1.26 塞进 PYTHONPATH，污染训练环境。

### 创建训练环境

```bash
cd ~/lerobot_ros2
./init.sh setup-train-env          # 等价于菜单 t
```

**如果在 Cursor 内置终端执行**，必须先覆盖 `UV_CACHE_DIR`，否则 uv 会把包写到
`/tmp/cursor-sandbox-cache/...`，`.venv-train` 永远只有 100K 空壳（已实测踩坑两次）：

```bash
env UV_CACHE_DIR="$HOME/.cache/uv" ./init.sh setup-train-env
```

验证：

```bash
.venv-train/bin/python -c "
import numpy, torch, lerobot
print('numpy', numpy.__version__)      # 期望 2.0~2.2
print('lerobot', lerobot.__version__)  # 期望 0.5.1
print('cuda', torch.cuda.is_available())
"
```

---

## 1. 数据准备

### 1.1 采集

```bash
cd ~/FaSim-Isaac && ./run.sh          # 终端 A：Isaac + 运控
./exts/fasim.replicator.sdg/scripts/sdg.sh capture
```

菜单选择建议：

| 项 | 建议 | 原因 |
|---|---|---|
| 采集项 | `camera,joint,ee,object` 全选 | ACT 用图像+关节；ee/object 供校验与成功判定 |
| 相机 | `all`（3 路） | 单相机遮挡时会瞎 |
| 频率 | `every=10`（20fps 仿真时间） | ACT 通常 10~30Hz，够用。见下方调参 |
| 深度 | 要 | ACT 不用，但后续 DP3/点云需要，省得重采 |

**帧率可调**（`every` 决定，不是硬件上限）：

```
落盘帧率(仿真时间) = 200Hz / every
  every=10 → 20 fps   ← 默认
  every=5  → 40 fps
  every=2  → 100 fps

落盘帧率(墙钟) = 墙钟物理步频(约130Hz) / every
  every=10 → 13 fps
  every=2  → 65 fps   ← 想接近同事的 57Hz 用这个
```

`every` 越小，`np.savez` 调用越频繁（1.3ms/帧），会拖慢物理。**先跑通再调**。

### 1.2 pack → trim → export

```bash
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
S=~/fasim_data/unitree_g1d/sinopec_test_tube/run_04

./scripts/sdg.sh pack   $S --robot unitree_g1d     # 会话 → episode.npz
./scripts/sdg.sh trim   $S                          # 切掉静止段（可选）
./scripts/sdg.sh export $S \
    --out ~/lerobot_datasets/tube_v1 \
    --action-space absolute                         # 或 delta / ee_delta
```

### 1.3 `action-space` 怎么选

| 选项 | 内容 | 什么时候用 |
|---|---|---|
| `absolute`（默认） | `joint_pos_cmd` 绝对关节指令 | **先跑这个**。部署时策略直接输出目标角，无累积误差；G1D 是位置控制，天然吃绝对值 |
| `delta` | `joint_pos[t+1] - joint_pos[t]` | 对齐同事 data-collector-platform（他们用 delta），便于混训/横向对比。实测自洽误差 1.86e-09 |
| `ee_delta` | 末端相邻增量 14 维 | 想学任务空间（与同事 pose 模式同构） |

> 注意 `delta` **不是** `cmd - 实测`。后者是控制器修正量，实测比真实位移大
> **7.43 倍**（0.0092 vs 0.0012 rad），会让策略学到「每步都要大修正」，部署时抖。

### 1.4 校验（必做）

```bash
./scripts/sdg.sh export-check ~/lerobot_datasets/tube_v1
# 必须看到 ✅ 数据集可被 LeRobotDataset 正常加载
```

---

## 2. 训练

### 2.0 实测跑通的命令（2026-11-13，lerobot 0.5.1 / numpy 2.2.6 / torch 2.7.1+cu128）

```bash
cd ~/lerobot_ros2
source .venv-train/bin/activate

python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=tube_smoke \
  --dataset.root=$HOME/lerobot_datasets/tube_smoke \
  --dataset.video_backend=pyav \
  --policy.type=act \
  --policy.chunk_size=100 --policy.n_action_steps=1 \
  --policy.push_to_hub=false \
  --output_dir=$HOME/act_runs/tube_v1 \
  --batch_size=8 --num_workers=4 \
  --steps=3000 --save_freq=1000 --eval_freq=0 \
  --wandb.enable=false
```

**两个必需的参数**（不加会失败，都是实机踩出来的）：

| 参数 | 不加会怎样 |
|---|---|
| `--policy.push_to_hub=false` | `ValueError: 'policy.repo_id' argument missing`（默认 true，想推 HF Hub） |
| `--dataset.video_backend=pyav` | DataLoader worker 崩溃：`torchcodec` 加载 `libavutil.so.59` 失败 |

> **FFmpeg 坑**：`torchcodec` 需要匹配的系统 FFmpeg 共享库（libavutil 56/57/59）。
> 本机系统 FFmpeg 版本不匹配 → 加载失败。lerobot 支持 `pyav` 后端（纯 Python 绑定的
> FFmpeg），`--dataset.video_backend=pyav` 绕开即可。日志里那串
> `OSError: libavutil.so.59` 是探测噪音，只要最终用 pyav 就不影响。

### 2.1 实测结果

```
cfg.steps=3000    dataset.num_frames=651    dataset.num_episodes=1
num_learnable_params=51650464 (52M)
```

| step | loss | 备注 |
|---|---|---|
| 200 | 6.319 | 起始（smoke 200 步测试是 7.058） |
| 800 | 2.016 | |
| 2000 | 1.049 | |
| 3000 | **0.598** | 37 epoch，5:53 分钟，8.48 step/s |

产物：`~/act_runs/tube_v1/checkpoints/{001000,002000,003000,last}/pretrained_model/`
（1.8G，含 `model.safetensors` + `train_config.json` + 归一化 stats）

**推理端可加载**（已验证）：

```python
from lerobot.policies.act.modeling_act import ACTPolicy
p = ACTPolicy.from_pretrained("~/act_runs/tube_v1/checkpoints/003000/pretrained_model")
# ✅ loaded, chunk_size=100, n_action_steps=1
```

> loss 0.598 看着不错，但**这只说明拟合了这 1 条轨迹**（37 个 epoch 反复看同一条）。
> 换个初始位置大概率失败。别被数字骗了 —— 见 §2.3。

### 2.2 关键参数

| 参数 | 默认 | 我们数据上的建议 | 原因 |
|---|---|---|---|
| `batch_size` | 8 | 8~16 | 4090 24GB，3 路 640×480（实测 8 够用） |
| `steps` | 100000 | 20000~50000 | 数据少易过拟合，多存 ckpt |
| `save_freq` | 20000 | 1000~5000 | 数据少，想早点看到 ckpt |
| `eval_freq` | 20000 | **0（关闭）** | 没有 test split |
| `num_workers` | 4 | 4~8 | 3 路视频解码是瓶颈 |
| `policy.chunk_size` | 100 | 100 | ACT 论文默认 |
| `policy.n_action_steps` | 100 | **1**（闭环最稳）或 chunk_size/2（提速） | 每步重预测，避免开环漂 |

### 2.3 数据量警告

ACT 论文用 **50~200 条**轨迹。我们目前：

| 会话 | 帧数 | 条数 |
|---|---|---|
| `run_03` | 674 | 1 |
| `run_04` | 651 | 1 |

**1~2 条只能验证流水线，训不出可用策略。** 20k 步后 loss 会很低，但那是对
这一条路径的过拟合 —— 换个初始位置就失败。

判断是否「只是流水线验证」：用同一会话训练 + 同一会话评测，loss 能收敛
说明代码没问题，但**不能说明策略可用**。

---

## 3. 推理（仓库里已经有）

**有，而且是完整的。** 关键文件：

| 路径 | 作用 |
|---|---|
| `examples/IsaacSim/inference.py` | 统一推理启动器（LeRobot + ROS2） |
| `examples/IsaacSim/online_infer/core.py` | 策略循环 + ROS2 发送 |
| `examples/IsaacSim/robots/Unitree/G1D/` | **G1D 的任务配置已在** |

推理内部走标准 LeRobot API（已在 0.5.1 核对）：

```python
# online_infer/core.py
from lerobot.policies.factory import make_policy          # :36
train_cfg = TrainPipelineConfig.from_pretrained(...)      # :372
policy = make_policy(cfg.policy, ds_meta=meta_obj)        # :420
action = policy.select_action(batch)                      # :532
```

手工加载 ckpt（已验证可用）：

```python
from lerobot.policies.act.modeling_act import ACTPolicy   # ← 类名是 ACTPolicy

p = ACTPolicy.from_pretrained("~/act_runs/tube_v1/checkpoints/003000/pretrained_model")
# ✅ chunk_size=100, n_action_steps=1
```

> ⚠️ 注意别导错：`modeling_act.py` 里有 `ACTPolicy`（PreTrainedPolicy 子类，
> 有 `from_pretrained`/`select_action`）和 `ACT`（纯 nn.Module）。
> 要加载 ckpt 用 **`ACTPolicy`**。

### 3.1 启动方式

```bash
cd ~/lerobot_ros2/examples/IsaacSim
source ~/lerobot_ros2/.venv-train/bin/activate   # 推理要 0.5.1 环境

python inference.py --robot g1d --task <task_key> \
    --policy-path ~/act_runs/tube_v1/checkpoints/last/pretrained_model
```

> 参数以 `python inference.py --help` 为准（我没跑过完整推理，只核对了代码结构）。

### 3.2 推理链路

```
Isaac 渲染相机 + 关节状态
    ↓（ROS2 话题，由 lerobot_camera_ros2 / lerobot_robot_ros2 订阅）
obs 张量 → policy.select_action() → action
    ↓（反归一化，转成关节目标）
/joint_states 目标 → OCS2 追踪 → PhysX
```

---

## 4. 三层评测（从便宜到贵）

| 层 | 做什么 | 成本 | 能回答 | 命令 |
|---|---|---|---|---|
| **L1 离线** | 留出帧上算 MSE | 秒 | 拟合好坏（**不能说明能用**） | 训练日志的 loss |
| **L2 开环** | 逐帧预测 action 与真值对比，**不驱动** | 分钟 | 动作方向/量级/抖动对不对 | `l2_openloop.py` |
| **L3 闭环** | 策略实时算 → 驱动关节 → 新观测 → 循环 | 分钟 | **能不能真的做任务** | `sdg.sh policy` |

### 4.1 L2 离线（已验证，25 条数据）

```bash
cd ~/lerobot_ros2
source .venv-train/bin/activate        # ← 必须先激活，才有 python

python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/l2_openloop.py \
    --dataset ~/lerobot_datasets/tube_25eps \
    --checkpoint ~/act_runs/tube25/checkpoints/020000/pretrained_model \
    --episode 0 --start 300 --max-frames 200
```

**`--start 300` 很重要**：跳过开头静止段。静止段所有动作接近 0，
预测再准也说明不了问题（第一次跑没加，得到"真值 med=0、相关性 NaN"）。

实测（25 条 / 20k 步 / loss 0.044）：

| 指标 | 值 | 说明 |
|---|---|---|
| MAE | **0.0057 rad** | ≈0.33°，精度高 |
| 量级 | 真值 0.1564 / 预测 0.1595 | 一致 |
| **逐维相关性** | **mean 0.996，中位 0.998** | **20/20 维 > 0.8** |
| 抖动比 | 0.94x | 预测略平滑，合理 |

相关性是更关键的指标 —— 它说明**动作曲线形状**对上了，而非数值巧合。

**归一化陷阱（踩过）**：lerobot 0.5.x 把归一化抽到 `PolicyProcessorPipeline`，
`policy.select_action()` 只做前向、不归一化也不反归一化。漏掉后预测值
是真值的 **1000 倍**、21.9% 饱和到 ±1。必须：

```python
batch = preprocessor(batch)          # 归一化输入
a = policy.select_action(batch)      # 前向
a = postprocessor({"action": a})["action"]   # 反归一化输出
```

### 4.2 L3 闭环（在 GUI 里看机器人实时动）

```bash
# 终端 A：起 Isaac（场景加载完）
cd ~/FaSim-Isaac && ./run.sh

# 终端 B：装载策略（非阻塞，立刻返回）
./exts/fasim.replicator.sdg/scripts/sdg.sh policy \
    ~/act_runs/tube25/checkpoints/020000/pretrained_model \
    --robot unitree_g1d --task sinopec_test_tube \
    --max-frames 600

# → 在 Isaac 窗口点 ▶ Play，机器人就动起来了
```

**每帧的循环**（在 `app.update()` 回调里，不阻塞）：

```
读 3 路 RGB + 32 关节角
    → 归一化 → policy.select_action() → 反归一化
    → set_dof_position_targets()
    → 物理推进一步 → 下一帧
```

与 `live`（回放）的区别：**动作不是预存的，是策略根据当前画面现算的**。
所以它能应对偏差（真的闭环），而回放只是照着演。

其他命令：`sdg.sh policy-status` / `sdg.sh policy-stop`。

**为什么不在回调里跑 for 循环**：在 socket 回调或 Script Editor 里调
`app.update()` 会**重入 Kit 主循环**，画面卡死或崩溃。所以只「装载 +
注册回调」，逐步驱动交给 Kit 自己的 update 循环（与 `live_replay` 同一套机制）。

### 4.3 EE 驱动（走 OCS2 IK）

采集时用 FK 从 `joint_pos_cmd` 反算了 `*_cmd_pos/_cmd_quat`（`robot.yaml`
的 `planning` 段），那些是**末端指令位姿**。推理若输出位姿，可直接发

```
/left_target  /right_target  (geometry_msgs/Pose) → OCS2 做 IK → 关节
```

`live_replay` 的 `drive="ee"` 已经这么做了（`live_replay.py:22`）。
所以 **EE 推理不需要自己写 IK**，复用这条链路即可。

| 驱动 | 精度 | 依赖 |
|---|---|---|
| `joints`（当前 `sdg.sh policy` 默认） | 精确（直接给关节目标） | 无 |
| `ee` | 取决于 IK 收敛 | 需 OCS2 在跑（`ROS=fiveages`） |

### 4.4 成功判定（L3 量化还缺这个）

目前 L3 只能肉眼观察。要算成功率需要自动判定，可用 `tracked_prims`：

```python
# 试管是否进孔：位置 + 高度双阈值
d = ||test_tube_pos - target_hole_pos||
ok = d < 0.02 and tube_z < rack_top_z
```

---

## 5. 与同事数据横向对比（可选）

等 L3 通了再做：

| 组 | 数据 | 训练 | 注意 |
|---|---|---|---|
| A | 我们 3 条（absolute） | 同配置 | — |
| B | 我们 3 条（delta） | 同配置 | 验证 delta 是否更好 |
| C | 同事 11 条（pose delta） | 同配置 | **他们 fps 声明 30 但实际 57Hz**，视频按 30fps 编码 → 时间轴错 2 倍。直接混训会让策略学到慢一倍的动作 |

> 同事那边「机器人抽搐」很可能是这个时间轴问题，或 delta 被当成绝对值发。
> 这正是我们平台在 fps 自洽性上的价值。

---

## 6. 当前缺口

| 缺口 | 影响 | 优先级 |
|---|---|---|
| 数据量（1~2 条） | 训不出可用策略 | **最高** |
| 闭环成功判定 | 无法量成功率 | 高 |
| `intervention` 人工干预标记 | 做不了 DAgger / 只学纠偏片段 | 中（对方有，我们没有） |
| `.venv-train` 未装完 | 训不了 | 你正在装 |

---

## 7. 快速开始（复制粘贴，已实测跑通）

```bash
# ---- 一次性：装训练环境（跳过已装的 torch）----
cd ~/lerobot_ros2
env UV_CACHE_DIR="$HOME/.cache/uv" ./init.sh setup-train-env
.venv-train/bin/python -c "import numpy, lerobot; print(numpy.__version__, lerobot.__version__)"
# 期望: 2.2.x  0.5.1

# ---- 数据（run_04 已导出过可跳过）----
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
S=~/fasim_data/unitree_g1d/sinopec_test_tube/run_04
./scripts/sdg.sh export $S --out ~/lerobot_datasets/tube_smoke \
    --action-space absolute --overwrite
./scripts/sdg.sh export-check ~/lerobot_datasets/tube_smoke

# ---- 训练 ----
cd ~/lerobot_ros2 && source .venv-train/bin/activate
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=tube_smoke \
  --dataset.root=$HOME/lerobot_datasets/tube_smoke \
  --dataset.video_backend=pyav \
  --policy.type=act \
  --policy.chunk_size=100 --policy.n_action_steps=1 \
  --policy.push_to_hub=false \
  --output_dir=$HOME/act_runs/tube_v1 \
  --batch_size=8 --num_workers=4 \
  --steps=3000 --save_freq=1000 --eval_freq=0 \
  --wandb.enable=false
# 实测: 5:53 分钟, loss 7.06 → 0.598

# ---- 验证 ckpt 可加载 ----
.venv-train/bin/python -c "
from lerobot.policies.act.modeling_act import ACTPolicy
p = ACTPolicy.from_pretrained('$HOME/act_runs/tube_v1/checkpoints/003000/pretrained_model')
print('OK', p.config.chunk_size, p.config.n_action_steps)
"
```

跑通后再：
1. 录 5~10 条 → 重训 → 看 L1 loss 是否收敛
2. 做 L2 开环回放
3. 加成功判定 → L3 闭环
