# 数据组织与批量处理

日期：2026-11-13
适用：sdg 采集平台（`exts/fasim.replicator.sdg`）

---

## 0. 目录职责速查（避免数据分散）

| 路径 | 作用 | 能否删 |
|---|---|---|
| `~/fasim_data` | **采集输出根**（`FASIM_SDG_DATA`，默认值）。live 采集写这里 | 采集数据可删，但建议先 pack/导出 |
| `~/lerobot_datasets` | 导出后的 LeRobot 数据集 | 可由 `fasim_data` 重建 |
| `~/act_runs` | 训练产物（checkpoints、日志） | 可重建（重训即可） |
| `~/fasim_sdg` | **旧版输出根**（已被 `~/fasim_data` 取代） | ✅ 可删（仅历史残留） |
| `~/bench` | 话题方案评测产物（已放弃该路线） | ✅ 可删 |

> ⚠️ **历史坑**：`configs/_default.yaml` 的 `output.dir` 曾指向 `~/fasim_sdg`，
> 而 live 采集实际走 `runtime.data_root()`（`~/fasim_data`），导致数据分散在
> 两个根目录。现已统一：**以 `runtime.data_root()` 为准**，`output.dir` 只被
> 旧版离线 recorder 使用。

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

### 2.5 完整的一次采集操作流程（实测步骤）

```
终端 A                          终端 B
────────                        ────────
cd ~/FaSim-Isaac
./run.sh                        （等 Isaac 起来、场景加载完）

                                ./scripts/sdg.sh capture \
                                    --robot unitree_g1d \
                                    --task sinopec_test_tube \
                                    --preset act_train -y
                                → 显示参数确认页，回车

                                （Isaac 内开始挂载采集器）

                                跑任务编排让机器人做一遍任务

                                ./scripts/sdg.sh status   # 看帧数
                                ./scripts/sdg.sh stop     # 结束

                                产物: ~/fasim_data/unitree_g1d/
                                      sinopec_test_tube/run_NN/
```

**批量采 10 条的循环**（手动版，等编排自动化后再改）：

```bash
cd ~/FaSim-Isaac
TASK=~/fasim_data/unitree_g1d/sinopec_test_tube

for i in $(seq 1 10); do
  echo "=== 第 $i 条 ==="

  # 1) 开始采集（自动用时间戳命名，避免覆盖）
  ./exts/fasim.replicator.sdg/scripts/sdg.sh capture \
      --robot unitree_g1d --task sinopec_test_tube \
      --preset act_train -y

  # 2) 手动/脚本触发一次任务编排，跑完整个插拔动作
  #    （这一步目前需要人工；见 §5 关于自动化的说明）
  read -p "跑完一遍后按回车继续..."

  # 3) 结束采集
  ./exts/fasim.replicator.sdg/scripts/sdg.sh stop
done

# 4) 一次性检查全部参数是否一致（批量合并的前提）
./exts/fasim.replicator.sdg/scripts/sdg.sh export $TASK \
    --out ~/lerobot_datasets/tube_10eps -y
```

第 4 步会自动跑参数一致性检查 —— 如果 10 条里有某条参数不对，
会直接列出是哪条、哪个字段不一致，而不是等到训练时报错。

### 2.6 `--preset` 与手选参数的关系

| 优先级 | 来源 | 说明 |
|---|---|---|
| 1（最高） | 命令行显式参数 | `--every 5 --width 1280` 会覆盖预设 |
| 2 | `--preset` | 预设里的 what/camera/target_fps/width/height/depth |
| 3 | 交互提问 | 没给预设时逐项问 |
| 4（最低） | 任务配置默认值 | `record.*` 段 |

所以「用预设但临时改一项」是可以的：

```bash
# 用 act_train 的所有参数，但分辨率临时降一半（只试一次）
./scripts/sdg.sh capture --preset act_train --width 320 --height 240 -y
```

但**不要**这样做后还指望和之前的数据合并 —— 守门会拦下来（这是对的）。

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

## 5. 后续路线

### 5.1 自动化采集（当前最大瓶颈）

目前**采集仍需手动**（起 Isaac → 选预设 → 跑编排 → 停）。要批量无人值守，
需要补三块：

| 缺口 | 现状 | 建议 |
|---|---|---|
| **任务编排触发接口** | sdg 有 socket 控制采集，但没有「跑一遍编排」的入口 | 最优先。调 `motion-generation` 或直接发轨迹 |
| **episode 自动分段** | `trim` 能切片段，但需人工确认 | 结合物体位姿判断「取到→放下」的起止 |
| **失败重试与复位** | 纯人工 | 先做标记待重采，别做全自动重跑 |

**设计原则：先做「半自动」，不要一步到「一键跑 100 条」。**

理由：自动化出错时（比如抓取失败但还在录），会污染整个数据集且难以追溯。
更稳的路径：

1. 每条采完自动 `pack` + `diag`（已有 7 项体检）
2. 不达标的自动标记 `need_retry`
3. 人只看质检报告决定重录哪些

这样 20 条里得到 20 条**有效**数据，而不是 25 条里混进 5 条脏数据。

### 5.2 与 data-collector-platform 的对比评测

**这是接下来最有价值的实验**，比我们内部方案对比有用得多。

背景：同事用 `data-collector-platform` 采了 11 条同任务数据
（`hole_4_to_hole_3`，末端 pose delta，57Hz，1280×720）。他们的数据有已知问题
（fps 声明 30 实际 57 → 时间轴 2x 错；32 维里 16 维是全零 pad），但
**采样率和分辨率比我们高**。

评测设计（等同事跑通 ACT 后再做）：

| 组 | 数据 | 训练 | 评测平台 |
|---|---|---|---|
| A | sdg 采 10+ 条（`act_train` 预设：20fps/640×480/关节绝对指令） | 同一套 ACT 配置 | 我们的 Isaac 闭环 |
| B | 同事 11 条（pose delta，需先重采样到统一时间基准） | 同上 | 同上 |

**关键前提**：B 组必须先修正时间轴（他们的视频按 30fps 编码但真实时长只有一半），
否则策略学到的是慢一倍的动作 —— 这不是公平对比。

建议在**本地部署一份他们的平台**，用同一个场景采几条，这样：
- 排除「他们的机器/场景配置不同」的干扰
- 能直接控制 fps、相机分辨率这些变量

如果之后要做，需要向同事要：
1. `config.py` 的实际运行值（仓库里 `STATE_ACTION_SOURCE = "joint"` 但数据是 pose，说明运行时被覆盖）
2. 采集循环挂在什么回调上（为什么 15/20ms 而 `FPS=30`）
3. 一条 raw 轨迹（未转 lerobot）+ 转换日志
4. 他们训 ACT 的 `train_config.json`

详见 `colleague-dataset-analysis.md`。
