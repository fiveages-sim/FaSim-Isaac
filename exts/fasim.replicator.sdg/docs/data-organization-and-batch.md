# 数据组织与批量处理

日期：2026-11-13
适用：sdg 采集平台（`exts/fasim.replicator.sdg`）

---

## 1. 目录结构

```
~/fasim_data/
└── <robot_key>/                    ← 如 unitree_g1d
    └── <task_key>/                 ← 如 sinopec_test_tube
        ├── run_01/                 ← 一次采集 = 一个会话
        │   ├── frames/             ← 逐帧 npz
        │   ├── session.json        ← 采集参数与健康自检
        │   ├── episode.npz         ← pack 产物（可选）
        │   └── contact_*.png       ← shots 相机联络图（可选）
        ├── run_02/
        └── run_03/
```

**三级语义**：

| 层级 | 含义 | 例子 |
|---|---|---|
| `<robot>` | 机器人型号 | `unitree_g1d` |
| `<task>` | 任务（一个训练目标） | `sinopec_test_tube` |
| `run_NN` | 一次采集（一条轨迹） | `run_04` |

一个训练数据集 = **一个 task 下的多个 run**。所以：

```bash
# 采集一次 → 自动进 run_NN
./scripts/sdg.sh capture --robot unitree_g1d --task sinopec_test_tube

# 采够 20 条后，一次性把它们导成一个数据集（20 个 episode）
./scripts/sdg.sh export ~/fasim_data/unitree_g1d/sinopec_test_tube \
    --out ~/lerobot_datasets/tube_v1
```

---

## 2. 采集预设：保证多批次格式对齐

### 2.1 问题

同一个任务要采几十条轨迹。如果每次手选参数，很容易某次改了个分辨率或
`every` —— 那批数据就和之前的不兼容了：

- 关节数/相机不同 → feature shape 冲突，数据集装不下
- `every` 不同 → 每帧时长不同，时间轴混在一起，策略学到错误的动作速度

这类错误**在导出时才爆**，白采了。

### 2.2 解决：预设固化在任务配置里

```yaml
# configs/<robot>/tasks/<task>.yaml
capture_presets:
  act_train:
    desc: "ACT 训练（20fps / 640x480 / 全字段）"
    what: [camera, joint, ee, object]
    camera: all
    target_fps: 20        # 按任务 physics_hz 自动反推 every
    depth: true

  act_hires:
    desc: "ACT 高分辨率（15fps / 1280x720）"
    what: [camera, joint, ee, object]
    camera: all
    target_fps: 15
    width: 1280
    height: 720
    depth: true

  smoke:
    desc: "冒烟测试（10fps / 320x240 / 无深度）"
    what: [camera, joint]
    camera: head
    target_fps: 10
    width: 320
    height: 240
    depth: false
```

**`target_fps` 优于写死 `every`**：它按任务的 `record.physics_hz` 反推
（`every = round(physics_hz / target_fps)`）。即使换了场景、步频变了，
预设的语义（"我要 20fps"）仍然成立。

200Hz 场景下换算结果（实测）：

| 预设 | target_fps | 推导 every | 实际帧率 |
|---|---|---|---|
| `act_train` | 20 | 10 | 20.0 fps |
| `act_hires` | 15 | 13 | 15.4 fps |
| `smoke` | 10 | 20 | 10.0 fps |

### 2.3 用法

**交互菜单**（推荐）：

```bash
./scripts/sdg.sh
# → 选「开始采集」
# → 第一问就是「采集预设」，列出任务配置里的预设
# → 选了之后只问输出目录，其余参数全部锁定
```

**命令行**（脚本/自动化）：

```bash
./scripts/sdg.sh capture --robot unitree_g1d --task sinopec_test_tube \
    --preset act_train -y
```

预设名会写进 `session.json` 的 `preset` 字段，事后可回溯「这批是哪个预设采的」。

### 2.4 手选参数仍然保留

不想用预设时，菜单里选「逐项手选」。用途：

- 回放 / 定向排查（比如只采关节，不采相机）
- 临时试验新参数（试完满意了再固化成预设）
- 新任务还没有预设时

---

## 3. 批量处理

### 3.1 一条命令处理整个任务

`pack` / `trim` / `export` 现在都接受 **任务目录**：

```bash
TASK=~/fasim_data/unitree_g1d/sinopec_test_tube

./scripts/sdg.sh pack   $TASK -y         # 所有 run → episode.npz
./scripts/sdg.sh trim   $TASK -y         # 所有 run 切片段
./scripts/sdg.sh export $TASK \
    --out ~/lerobot_datasets/tube_v1 -y  # 合并成多 episode 数据集
```

路径展开规则（`_expand_sessions`）：

| 输入 | 行为 |
|---|---|
| 单个会话目录（含 `frames/`） | 只处理它 |
| 任务目录（直下有多个会话） | 处理该任务下全部会话 |
| 机器人目录（再下一层是任务） | 递归收集所有任务的会话 |

判定「是会话」不依赖命名，看是否含 `frames/`、`session.json` 或
`episode.npz` 之一 —— 所以 `run_04`、`bench01` 这些名字都能认出来。

### 3.2 合并前的参数守门（**重要**）

批量合并只有在**采集参数一致**时才成立。所以 `export` 在合并前会自动比对：

```
采集参数检查：2 个会话
  ✓ 关键参数一致（可以合并）
  · frames  [建议一致]
      run_03                   674
      run_04                   651
```

不一致时**直接拒绝**：

```
采集参数检查：2 个会话
  ✗ 关键参数不一致 —— 不能合并成一个数据集
  ✗ cameras  [必须一致]
      run_04                   [head@640x480, wrist_left@640x480, wrist_right@640x480]
      run_bad                  [head@640x480, wrist_left@640x480]
  ✗ every  [时间轴]
      run_04                   10
      run_bad                  20

  怎么办：
    1) 用同一套采集参数重采 —— 见 configs/<robot>/tasks/<task>.yaml
       或菜单里选「采集预设」（自动带出全部参数）
    2) 或者只导出参数一致的子集：给单个会话路径而不是任务目录
```

比对的字段分三类（`capture_profile.py`）：

| 类别 | 字段 | 不一致时 |
|---|---|---|
| **critical** | `joint_count` / `joint_names` / `cameras` / `what` / `robot_key` / `task_key` / `ee_captured` / `objects_captured` | 拒绝合并（feature 定义冲突） |
| **timing** | `every` / `physics_hz_setting` | 拒绝合并（时间轴错乱，且**静默成功**所以更危险） |
| **info** | `frames` / `preset` | 只提示（每条轨迹长短天然不同） |

强制合并（危险，仅用于排查）：`--allow-mismatch`。

### 3.3 实测

```
2 个 run（run_03 674 帧 + run_04 651 帧）
→ export 合并
→ 1325 帧 / 2 episode

info.json:  codebase_version=v3.0  total_episodes=2  total_frames=1325  fps=20
parquet:    ep0 index 0..673（连续）  ep1 index 674..1324（连续）
export-check: ✅ 可被官方 LeRobotDataset 加载
```

关键验证点：**跨 episode 不串**。`delta` 模式下 `action[t] = pos[t+1] - pos[t]`，
如果不小心用了全局帧列表，episode 边界会取到下一个 episode 的第一帧 ——
worker 里显式用 `ep_frames`（本 episode 内）避免了这个坑。

---

## 4. 从采集到训练的完整流程

```bash
# ── 1. 采集 N 条（推荐用预设，保证格式一致）──
cd ~/FaSim-Isaac && ./run.sh          # 终端 A：Isaac + 运控
cd ~/FaSim-Isaac
for i in $(seq 1 10); do
  ./exts/fasim.replicator.sdg/scripts/sdg.sh capture \
      --robot unitree_g1d --task sinopec_test_tube \
      --preset act_train -y --out ~/fasim_data/unitree_g1d/sinopec_test_tube/run_$i
  # 等这条跑完（手动停或脚本控制）
  ./exts/fasim.replicator.sdg/scripts/sdg.sh stop
done

# ── 2. 批量 pack + trim ──
TASK=~/fasim_data/unitree_g1d/sinopec_test_tube
./exts/fasim.replicator.sdg/scripts/sdg.sh pack $TASK -y
./exts/fasim.replicator.sdg/scripts/sdg.sh trim $TASK -y

# ── 3. 合并导出（自动做参数一致性检查）──
./exts/fasim.replicator.sdg/scripts/sdg.sh export $TASK \
    --out ~/lerobot_datasets/tube_v10 -y
./exts/fasim.replicator.sdg/scripts/sdg.sh export-check ~/lerobot_datasets/tube_v10

# ── 4. 训练 ──
cd ~/lerobot_ros2 && source .venv-train/bin/activate
python -m lerobot.scripts.lerobot_train \
  --dataset.repo_id=tube_v10 --dataset.root=$HOME/lerobot_datasets/tube_v10 \
  --dataset.video_backend=pyav \
  --policy.type=act --policy.chunk_size=100 --policy.n_action_steps=1 \
  --policy.push_to_hub=false \
  --output_dir=$HOME/act_runs/tube_v10 \
  --batch_size=8 --num_workers=4 --steps=20000 --save_freq=2000 --eval_freq=0 \
  --wandb.enable=false
```

---

## 5. 后续：自动化采集

目前**采集仍需手动**（起 Isaac → 选参数 → 跑编排 → 停）。要做批量无人值守，
需要补三块（尚未实现，按优先级）：

| 缺口 | 说明 |
|---|---|
| **任务编排触发接口** | sdg 已有 socket 服务控制采集，但没有「跑一遍编排」的入口。需要调 `motion-generation` 或直接发轨迹 |
| **episode 自动分段** | 一条长采集里自动识别「一次完整任务」的起止（`trim` 已能切，但需人工确认） |
| **失败重试与复位** | 抓失败了要复位场景 + 重跑，现在是人工 |

设计建议：不要做「一键跑 100 条」的黑盒。先做「一次采集后自动 pack + 质检，
不达标就标记待重采」，人只需审阅报告 —— 这样即使自动化出错也不会污染数据集。
