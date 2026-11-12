# ACT 训练路线（sdg 数据）

日期：2026-11-12
目标：用 sdg 采集的 G1D 试管插拔数据训 ACT，并建立可复现的评测闭环。

---

## 0. 前置检查（本机实测状态）

| 项 | 状态 | 说明 |
|---|---|---|
| GPU | ✅ RTX 4090 / 24GB | ACT（~50M 参数）足够，单卡可训 |
| `lerobot` | ✅ 0.3.4（submodule editable） | `lerobot_ros2/submodules/lerobot` |
| `torch` | ✅ 2.7.1+cu128 | 在 `lerobot_ros2/.venv` |
| `numpy` | ⚠️ 1.26.4 | **ACT 训练要在 numpy 2.x 环境下**（`lerobot 0.5.x`）；当前 1.26 是为 `cv_bridge` 兼容 |
| `.venv-train` | ❌ **不存在** | 之前规划过但没建 |

> **重要**：`.venv` 里 numpy 1.26.4 是为了系统 `cv_bridge`（ROS 采图）。训练不需要
> ROS，应该单独建一个 numpy 2.x 环境，避免污染采集环境。

### 导入路径注意

lerobot 0.3.4 的 ACT **没有** `ACTPolicy` 这个导出名，实际是：

```python
from lerobot.policies.act.modeling_act import ACT          # 模型
from lerobot.policies.act.configuration_act import ACTConfig
```

---

## 1. 环境准备

```bash
cd ~/lerobot_ros2
uv venv .venv-train --python 3.12
source .venv-train/bin/activate

# 训练侧依赖（不装 ROS 相关）
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install -e submodules/lerobot
uv pip install av pyarrow datasets
```

验证：

```bash
python -c "
import torch, numpy
print('torch', torch.__version__, 'cuda', torch.cuda.is_available())
print('numpy', numpy.__version__)
from lerobot.policies.act.modeling_act import ACT
print('ACT OK')
"
```

**要求**：`torch.cuda.is_available()` 必须是 `True`、`numpy` 应是 2.x。

---

## 2. 数据准备

### 2.1 采集

```bash
cd ~/FaSim-Isaac && ./run.sh          # 终端 A：起 Isaac + 运控
cd ~/FaSim-Isaac
./exts/fasim.replicator.sdg/scripts/sdg.sh capture
```

关键选择（菜单里）：

| 项 | 建议 | 原因 |
|---|---|---|
| 采集项 | `camera,joint,ee,object` 全选 | ACT 需要图像 + 关节；ee/object 用于校验 |
| 相机 | `all`（3 路） | 单相机在遮挡时会瞎 |
| 频率 | 取决于目标 fps（见下） | **不是硬件上限，是 `every` 决定的** |
| 深度 | 要（虽然 ACT 不用，但后续换 DP3 需要） | 省得重采 |

**帧率可以调高**（实测推论，待验证）：

```
落盘帧率 = 墙钟物理步频 / every
当前 every=10 → 13 fps 墙钟（20 fps 仿真时间）
every=1  → 理论 130 fps 墙钟（200 fps 仿真）
```

`every` 越小，物理步进越频繁调用 `np.savez`（1.3ms/帧），会拖慢物理。
**建议先试 `every=2`（约 65fps 墙钟）**，看机器人运动是否还顺滑、RTF 掉多少。

### 2.2 pack（会话 → episode.npz）

```bash
./exts/fasim.replicator.sdg/scripts/sdg.sh pack <会话目录> --robot unitree_g1d
```

### 2.3 trim（去掉静止段）

我们数据开头常有 5~8 秒静止（等运控稳定）。训练前必须切掉，否则策略学到"先发呆"：

```bash
./exts/fasim.replicator.sdg/scripts/sdg.sh trim <会话目录>
```

### 2.4 export（episode → LeRobot）

```bash
./exts/fasim.replicator.sdg/scripts/sdg.sh export <会话目录> \
    --out ~/lerobot_datasets/tube_v1 \
    --action-space delta        # 或 absolute
```

**`action-space` 怎么选**：

| 选项 | 适用 | 理由 |
|---|---|---|
| `absolute`（默认） | **先跑这个** | 部署时策略直接输出目标关节角，无累积误差；G1D 用 ros2_control 位置控制，天然吃绝对值 |
| `delta` | 对齐同事数据 / 混训 | 与 data-collector-platform 同语义，便于横向对比 |
| `ee_delta` | 想学任务空间 | 与同事 pose 模式同构，但需要 IK 执行 |

> 我们的 `absolute` 是 `joint_pos_cmd`（**指令**），不是实测值。
> 这点很关键：部署时策略只能输出命令，训练目标就该是命令。

### 2.5 校验（必做）

```bash
./exts/fasim.replicator.sdg/scripts/sdg.sh export-check ~/lerobot_datasets/tube_v1
```

必须看到 `✅ 数据集可被 LeRobotDataset 正常加载`。

---

## 3. 训练

### 3.1 最小可用命令

```bash
cd ~/lerobot_ros2
source .venv-train/bin/activate

python -m lerobot.scripts.train \
    --dataset.repo_id tube_v1 \
    --dataset.root ~/lerobot_datasets/tube_v1 \
    --policy.type act \
    --output_dir ~/act_runs/tube_v1 \
    --batch_size 8 \
    --steps 20000 \
    --policy.chunk_size 100 \
    --policy.n_action_steps 1
```

> `--dataset.root` 指向本地目录（不走 HF hub）。若 lerobot 版本参数名不同，
> 用 `python -m lerobot.scripts.train --help` 确认。

### 3.2 关键超参（针对我们的数据）

| 参数 | 建议 | 为什么 |
|---|---|---|
| `chunk_size` | 100 | ACT 论文默认；我们 20fps → 5 秒预测窗口 |
| `n_action_steps` | 1 | 每步都重新预测（闭环最稳）；要提速可设 chunk_size/2 |
| `batch_size` | 8~16 | 4090 24GB，图像 640×480 ×3 路 |
| `steps` | 20000~50000 | 数据量小（几百帧/条），易过拟合，多存 checkpoint |
| `lr` | 1e-5（默认） | 先用默认 |
| `vision_backbone` | `resnet18`（默认） | 24GB 够，可试 `resnet34` |

**数据量警告**：ACT 通常要 50+ 条轨迹。我们现在只有 1~2 条
（`run_03` 674 帧、`run_04` 651 帧）。**先跑通流水线，别期待效果**。

### 3.3 过拟合自检（数据量小的时候必做）

```bash
# 用同一条数据训练 + 评测，loss 应该能降到很低
# 若 20k 步后 loss 仍 > 0.01，说明是流水线问题，不是数据问题
```

---

## 4. 评测（这是难点）

### 4.1 三层评测，从便宜到贵

| 层 | 做什么 | 成本 | 能回答什么 |
|---|---|---|---|
| **L1 离线** | 在留出帧上算 MSE / 动作相似度 | 秒级 | 拟合好坏（**不能说明能不能用**） |
| **L2 开环回放** | 把预测 action 喂给 `sdg.sh live` | 分钟级 | 动作序列是否合理、会不会发散 |
| **L3 闭环仿真** | 策略输出 → 运控执行 → 观察新状态 | 小时级 | **真实成功率** |

### 4.2 L2：开环回放（最优先做）

我们已经有 `live_replay.py`，需要加一个"策略推理"入口：

```
sdg.sh policy <episode.npz> --checkpoint <ckpt> --mode openloop
```

流程：加载 obs → 策略出 action → 写进 `joint_pos_cmd` → 推进物理 → 循环。
这能立刻发现"策略输出量级不对/全部饱和/抖"这类低级错误，**不用等闭环**。

### 4.3 L3：闭环仿真

需要打通三件事：

1. **策略服务**：把 ACT 包成一个 ROS2 节点或 socket 服务，输入 obs（图像+关节），输出 action。
2. **执行链路**：action → `/joint_states` 目标 → OCS2 追踪 → 物理。
3. **成功判定**：试管是否插进目标孔位。可以用 `tracked_prims` 的位姿判断
   （`test_tube` 相对 `test_tube_rack` 的位置/姿态阈值）。

我们现在**缺 1 和 3**，2 已经通（`sdg.sh live` 证明回放链路可用）。

### 4.4 建议顺序

```
采集 2~3 条 → pack → trim → export → export-check
    ↓
训练 20k 步 → L1 离线 loss 可接受？
    ↓ 是
加 policy 推理入口 → L2 开环回放（看图像+动作）
    ↓ 动作合理
加成功判定 → L3 闭环跑 20 次算成功率
    ↓
再决定：加数据？改 action_space？改相机？
```

---

## 5. 与同事数据的横向对比（可选，但有价值）

一旦 L3 通了，可以做对照实验：

| 组 | 数据 | 训练 | 评测 |
|---|---|---|---|
| A | 我们 3 条（absolute） | 同配置 | 同一套 L3 |
| B | 我们 3 条（delta） | 同配置 | 同一套 L3 |
| C | 同事 11 条（delta/pose） | 同配置 | 同一套 L3（**需注意他们的 fps 是 57Hz 但声明 30**） |

C 组有个坑：他们的视频按 30fps 编码但真实时长只有一半，
直接训会让策略学到"慢一倍的动作"。**必须先重采样到同一时间基准**，
或者干脆只用 parquet 里的数据 + 自己按真实时间戳抽帧。

> 同事那边"机器人抽搐"——很可能是他们的 action 在部署时语义不对
> （delta 被当成绝对值发），或时间轴 2x 错导致控制频率不匹配。
> 这正是我们平台在 fps 自洽性上的价值。

---

## 6. 当前缺口（按优先级）

| 缺口 | 影响 | 工作量 |
|---|---|---|
| `.venv-train` 未建 | 训不了 | 10 分钟 |
| 数据量太少（1~2 条） | 学不出东西 | 每条 1 小时录 + 标注 |
| 无策略推理入口 | L2/L3 都做不了 | 半天 |
| 无成功判定 | 无法量成功率 | 半天 |
| `intervention` 标记缺失 | 做不了 DAgger / 人工纠偏 | 需要设计 |

---

## 7. 快速开始（复制粘贴）

```bash
# 1) 建训练环境
cd ~/lerobot_ros2
uv venv .venv-train --python 3.12
source .venv-train/bin/activate
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install -e submodules/lerobot av pyarrow datasets

# 2) 验证
python -c "import torch,numpy;print(torch.cuda.is_available(),numpy.__version__)"

# 3) 用已有会话跑通流水线（不采集）
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
./scripts/sdg.sh pack ~/fasim_data/unitree_g1d/sinopec_test_tube/run_04 --robot unitree_g1d
./scripts/sdg.sh export ~/fasim_data/unitree_g1d/sinopec_test_tube/run_04 \
    --out ~/lerobot_datasets/tube_smoke --action-space absolute --overwrite
./scripts/sdg.sh export-check ~/lerobot_datasets/tube_smoke

# 4) 试训（数据少，只看能否跑起来）
cd ~/lerobot_ros2 && source .venv-train/bin/activate
python -m lerobot.scripts.train \
    --dataset.repo_id tube_smoke \
    --dataset.root ~/lerobot_datasets/tube_smoke \
    --policy.type act --output_dir ~/act_runs/smoke \
    --batch_size 4 --steps 2000
```
