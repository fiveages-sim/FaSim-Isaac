# sdg 数据 → ACT 训练（本文范围：采集侧）

日期：2026-11-13
适用：lerobot **v0.5.1**

> **职责边界**：sdg 只负责**采集 / 打包 / 导出**。训练与推理在
> `~/lerobot_ros2`，见 [`TRAINING_AND_INFERENCE.md`](../../../../lerobot_ros2/docs/TRAINING_AND_INFERENCE.md)。
>
> 为什么推理不放 sdg：需要 `lerobot`+`torch`（numpy 2.x），与 Isaac 进程的
> numpy 1.x / cv_bridge 冲突；且驱动应走运控（OCS2 做 IK），不是 Isaac 内部
> 直接改关节。

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
    --action-space ee_absolute_gripper             # 抓取任务用这个（14 维 ee + 2 维夹爪开关）
```

### 1.3 `action-space` 怎么选

| 选项 | 内容 | 什么时候用 |
|---|---|---|
| `absolute`（默认） | `joint_pos_cmd` 绝对关节指令 | 想直接控关节时用 |
| `delta` | `joint_pos[t+1] - joint_pos[t]` | 对齐同事 data-collector-platform（他们用 delta），便于混训/横向对比。实测自洽误差 1.86e-09 |
| `ee_delta` | 末端相邻增量 14 维 | 想学任务空间（与同事 pose 模式同构） |
| `ee_absolute` | 末端**绝对**位姿 14 维 | 推理走运控 IK（`inference.py`） |
| **`ee_absolute_gripper`** | 上面 14 维 + **夹爪开关量 2 维** | **做抓取任务必须用这个**，否则策略没有夹爪通道、抓不住东西 |

> 注意 `delta` **不是** `cmd - 实测`。后者是控制器修正量，实测比真实位移大
> **7.43 倍**（0.0092 vs 0.0012 rad），会让策略学到「每步都要大修正」，部署时抖。

#### 夹爪为什么是**开关量**而不是关节位置（踩过的坑）

运控的 `AdaptiveGripperController` 是 `target_command` 模式：闭合时它不回"录制时的
关节角"，而是回到**距初始值最近的限位**，靠压出夹持力夹紧物体：

```cpp
closed_position_ = (|upper - init| < |lower - init|) ? upper : lower;
new_target = closed_position_ + percent * (open_position_ - closed_position_);
```

如果把采集到的夹爪关节位置（`±0.02` 这种）当指令发下去，闭合会停在"刚好碰到管子"
的位置，**夹持力不足**。所以 action 里只保留「开/合」这一位，幅度交给运控决定。

**sdg 侧怎么拿到这一位**（不用走话题）：sdg 采的是 Isaac 内部
`get_dof_position_targets()`，而 `target_command` 模式下 ros2_control 只会往位置命令
接口写**两个限位之一**，所以 `joint_pos_cmd` 的夹爪列天然离散。实测 41 条 run 全部
只有 3 个值：

| cmd | 含义（run_01 物理结果反推） |
|---|---|
| `-0.0200` | 夹住：右手持管抬起，tube_z `0.743 → 0.920` |
| `+0.0245` | 张开：管子落回 `0.743` |
| `0.0000` | 上电闲置值（两指并拢，属闭合点） |

判据 = **离哪个限位更近**（复刻控制器规则），而不是"高于/低于中点"——后者在初始值
靠近上限的机器人上会反掉。dex1 的 URDF 限位是 `lower=-0.02, upper=0.0245, init=0`，
算出 `closed_position_ = -0.02`，与上表一致。

`export-check` 会强制校验：夹爪维必须是 0/1、且至少出现过一次闭合；否则报
`GRIPPER_NOT_BINARY` / `GRIPPER_NEVER_CLOSED`。

#### ⚠️ `observation.ee` 的四元数是 wxyz，而 `action` 是 xyzw

同一数据集里两种约定并存：`action` 的 `*_quat.{x,y,z,w}` 名字明确（xyzw，对齐
`geometry_msgs/Pose`），而 `observation.ee` 是 14 维裸数组、**没有 names**，其值来自
`episode.npz` 的 wxyz。直接用会差 **166°**（看起来像"左右臂镜像"）。

当前 ACT 只吃 `observation.state` + 三路图像，`observation.ee` 不在 `input_features`
里，所以**不影响训练**。但如果以后要把它喂给策略，必须先做 wxyz→xyzw 轮转。

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

## 4. 推理在哪里做（重要：不在 sdg）

**sdg 只负责采集与数据导出。推理属于 `lerobot_ros2`。**

原因：
1. 推理需要 `lerobot` + `torch`（`.venv-train`，numpy 2.x），与 Isaac 的
   python 环境（numpy 1.x / cv_bridge）冲突，装不进 Isaac 进程
2. 推理与「数采平台」无关，是策略执行环节
3. 驱动方式应该**走运控**（OCS2 做 IK），而不是 Isaac 内部直接改关节
   —— 这样才与真实部署一致

所以 sdg 这边的交付边界是：**采到干净数据 + 导出成 LeRobot 数据集**。
之后的训练与推理全在 `lerobot_ros2`：

| 环节 | 仓库 | 文档 |
|---|---|---|
| 采集 / pack / trim / export | `FaSim-Isaac`（sdg） | 本文 |
| 训练 | `lerobot_ros2` | [`docs/TRAINING_AND_INFERENCE.md`](../../../../lerobot_ros2/docs/TRAINING_AND_INFERENCE.md) |
| 推理（走运控驱动） | `lerobot_ros2` | `examples/IsaacSim/inference.py` + `online_infer/core.py` |

### 4.1 sdg 侧能做的最终校验：`export-check`

导出后必须验证能否被官方 loader 加载，这是 sdg 的最后一道关：

```bash
./scripts/sdg.sh export $TASK --out ~/lerobot_datasets/tube_v1 -y
./scripts/sdg.sh export-check ~/lerobot_datasets/tube_v1
```

### 4.2 数据质量判据（sdg 的 `diag`）

不要只看帧数，要看物理结果。`diag` 会检查物体是否到达目标、关节是否跳变：

```bash
./scripts/sdg.sh diag <session目录>
```

25 条试管数据的实测判据，可作为参考基线：

| 判据 | 合格范围 |
|---|---|
| 起点距右架 | < 5 cm |
| 终点距左架 | < 6 cm |
| 试管抬起高度 | > 5 cm（实测 18.8） |
| 终点落架高度误差 | < 4 cm（实测 1.2） |
| 关节跳变（相邻采样） | 0 次 |

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
```

跑通后再：
1. 录 5~10 条 → 重训 → 看 L1 loss 是否收敛
2. **转去 `lerobot_ros2` 做 L2 开环验证**（`online_infer/openloop_check.py`）
3. 再做 L3 闭环推理（`examples/IsaacSim/inference.py`，走运控 IK）

> sdg 的职责到「导出可用的 LeRobot 数据集」为止。
> 训练与推理的详细说明见
> `~/lerobot_ros2/docs/TRAINING_AND_INFERENCE.md`。
