# 同事数据集分析：hole_4_to_hole_3（data-collector-platform 产出）

分析日期：2026-11-12
数据来源：`~/Downloads/hole_4_to_hole_3_lerobot/hole_4_to_hole_3/`
工具：`analysis_dcp_dataset.py`（同目录）

---

## 1. 数据集概况

| 项 | 值 |
|---|---|
| LeRobot 版本 | **v2.1**（我们导出的是 v3.0） |
| Episode 数 | **11**（`ls` 看到 12 个目录，1 个无 parquet） |
| 总帧数 | 27,701 |
| 单条帧数 | 2091 ~ 4046（中位 2329） |
| 任务 | 用右臂从右侧试管架第 4 孔位拿起试管 → 交给左臂 → 放入左侧试管架第 3 孔位 |
| 磁盘 | 171MB / 11 条 ≈ **15.5MB/条** |
| `episode_done.json` | 11/11 全部 `status: success`, `frames_skipped: 0` |
| `robot_type` | `offline_robot` |

### 字段（每条 32 维）

```
observation.state:  left_pos_xyz(3) left_quat_xyzw(4) left_gripper(1)
                    right_pos_xyz(3) right_quat_xyzw(4) right_gripper(1)
                    pad_0..pad_15(16)                    → 32
action:             left_dpos_xyz(3) left_dquat_xyzw(4) left_gripper(1)
                    right_dpos_xyz(3) right_dquat_xyzw(4) right_gripper(1)
                    pad_0..pad_15(16)                    → 32
observation.intervention: int64（人工干预标记）
capture_timestamp_ns / action_timestamp_ns: int64
timestamp / frame_index / episode_index / index / task_index: 标准 LeRobot
```

相机：`base_0_rgb` **1280×720**、`left_wrist_0_rgb` / `right_wrist_0_rgb` 640×480，
全部 h264 / yuv420p。

---

## 2. ⚠️ 发现的问题

### 2.1 fps 声明错误：声明 30，实际约 57Hz

```
capture dt: min=15.00ms  med=15.00ms  p95=20.00ms  max=20.00ms
            15ms × 1504 (66.7%)  /  20ms × 752 (33.3%)
```

- 只有 15ms 和 20ms 两个值（抖动 0），说明是**渲染帧间隔**（50Hz 渲染，
  每帧 20ms，但采集回调有时触发两次 → 15ms/20ms 交替）
- 平均间隔 = (15×1504 + 20×752)/2256 = **16.67ms → 60Hz 平均**
- 但**时间轴不匀**：60Hz 的平均值由 66.7Hz 和 50Hz 混合而成
- `meta/info.json` 写 `"fps": 30`，`timestamp` 字段是 `frame_index/30`
- **视频也按 30fps 编码**：2257 帧 / 30fps = 75.2s，真实时长只有 37.6s

**后果**：
1. 索引 1:1 对齐（parquet 2257 行 = 视频 2257 帧），训练取帧时间点正确，
   但**回放速度只有真实的一半**（动作看起来像慢动作）。
2. 用 `timestamp` 字段算速度/加速度会**低估 2 倍**。
3. 真实采样间隔 15~20ms 且不匀（每 3 帧里 2 个 15ms + 1 个 20ms），
   对学速度的策略（Diffusion/Flow）会引入节拍噪声。

**根因推测**：采集循环挂在渲染回调上（每渲染帧采一次），但 LeRobot 元数据里
写死了配置的 `FPS = 30`（来自 `config.py`），两者没对齐。

### 2.2 action 是"下一帧减当前帧"，但 timestamp 标的是"下一帧"

验证：

```
left pos:  state[t] + action[t] == state[t+1]   误差 0.000mm（完全精确）
right pos: 同上                                  误差 0.000mm
action_timestamp[t] == capture_timestamp[t+1]   100% 成立
```

这一点**其实是自洽的**（同事的 `0a27510` 提交说明"pose 保留下一有效帧 +1"）。
`action[t]` 是"为了从 state[t] 到达 state[t+1] 需要的位姿增量"，
把 `action_timestamp` 标成 `t+1` 的时刻是合理的。

**但要注意**：这种 delta 表示法在训练时如果直接用 `action` 当作"目标位姿"，
会和 `observation.state` 的量纲差一个数量级（action 量级 1e-3，state 量级 1e0）。
如果用 ACT 这类直接回归 action 的策略，**必须确认训练脚本知道 action 是 delta**。

### 2.3 pad_0..pad_15 全是 0（32 维里 16 维是死的）

```
pad_0 .. pad_15    min=0  max=0  std=0    （state 和 action 都是）
```

这就是同事 config 里 `PI05_MAX_DIM = 48` 的填充位（32 真实 + 16 填充 = 48）。
**这 16 维纯浪费**，占了一半的 action/state 带宽。PI0.5 需要固定维度，
但如果目标是 ACT，完全没必要 pad。

### 2.4 observation.state 的 19% 重复：**全部来自静止段，不是缺陷**

```
平均 state 重复率 19.0%   （每条 18.3% ~ 20.2%）
平均 action 重复率  0.3%
```

初看像是"采集频率超过控制更新率导致重复"，但**按运动/静止拆开就清楚了**：

| episode | n | state 重复 | 其中静止段 | **运动时重复** |
|---|---|---|---|---|
| 20261111_181632 | 2257 | 20.2% | 99% | **0.0%** |
| 20261111_181802 | 2154 | 19.0% | 100% | **0.0%** |
| 20261111_181901 | 2927 | 18.5% | 100% | **0.0%** |
| 20261111_182021 | 4046 | 19.3% | 100% | **0.0%** |
| 20261111_182831 | 2329 | 18.3% | 100% | **0.0%** |

**运动时 0.00% 重复。** 那 19% 全部落在机器人静止的帧上 ——
位姿没变，数值逐位相同是**正确的**，不是数据问题。

> 判据提醒：算重复率必须按"是否在动"分层。
> 静止场景下任何"重复率"指标都会虚高（图像同理，见
> `frame-duplication-metrics.md`）。

### 2.5 `observation.intervention` 全程为 1（2257/2257）

```
每条 episode 的 intervention 总和 == 帧数
```

全 1 表示"全程人工干预"。如果这个字段本意是"标记哪些帧是人手动接管的"，
那它**没有提供任何信息**——要么是全手动遥操作（合理），要么是标记逻辑写反了。

### 2.6 11 条轨迹高度同质（不是坏事，但要警惕）

| 指标 | 范围 |
|---|---|
| 左臂行程 | 24.7 ~ 24.9 cm（几乎完全一致） |
| 右臂行程 | 34.6 ~ 35.6 cm |
| 运动帧数 | 1348 ~ 1384 |
| 起点（左） | 全部 ≈ `[2.107, -0.031, 0.885]` |
| 终点（左） | 全部 ≈ `[2.004, 0.025, 1.03]` |

跨 episode 轨迹 RMS 差 36~104mm，说明**不是同一条轨迹重放**，是真实多次演示。
但起点终点几乎完全钉死，多样性很低 —— 对训练 ACT 来说，11 条同质轨迹
能学到"这条固定路径"，泛化能力有限。

### 2.7 前段有静止段（0.1~9s）

| episode | 首个运动帧 |
|---|---|
| 大部分 | 6~7（≈0.1s，几乎立刻开始） |
| `181632` | 463（≈7s 静止） |
| `182021` | 543（≈8s 静止） |
| `182831` | 320（≈5s 静止） |

3 条轨迹开头有 5~8 秒的纯静止（速度 0.002mm/帧）。训练时应该 trim 掉，
否则策略会学到"先发呆 5 秒"。

---

## 3. 运动时间线（`181632` 样本）

```
frames     mean spd(mm/帧)   max   静止占比  Lgrip Rgrip
   0- 100        0.0020      0.01    30%    1.0   1.0   ← 静止
 400- 500        0.1849      1.22    27%    1.0   1.0   ← 开始动
 500- 600        1.2261      2.67    21%    1.0   1.0
 600- 700        0.2923      2.14    19%    1.0   0.0   ← 右爪张开（抓取）
 700- 800        2.0149      6.15    19%    1.0   0.0   ← 抬起（最快）
1100- 1200        1.1593      4.39    21%    0.0   0.0   ← 左爪张开（交接）
1200- 1300        0.7741      2.12    19%    0.0   1.0   ← 右爪闭合
1600- 1700        1.1438      5.03    19%    1.0   1.0   ← 左爪闭合（放入）
1800- 1900        0.0677      0.11    21%    1.0   1.0   ← 减速
2200- 2257        0.0020      0.01    33%    1.0   1.0   ← 静止收尾
```

夹爪切换点（归一化）：左 `[0.458, 0.688]`、右 `[0.265, 0.498]`。
逻辑正确：右爪先开 → 右爪闭 → 左爪开（交接）→ 左爪闭。

注意：夹爪值是**二值**（只有 0 和 1，各 episode 只有 2 次切换），
不是连续的百分比。同事的 `9013040` 提交提到 `target_percent grippers`，
但这份数据里是开关量。

---

## 4. 与我们 sdg 方案的对比

| 维度 | 同事 (data-collector-platform) | 我们 (sdg) |
|---|---|---|
| LeRobot 版本 | v2.1 | **v3.0** |
| state | 末端位姿 + gripper + 16 pad = 32 | **32 关节角** |
| action | 末端 **delta** + gripper + 16 pad = 32 | `joint_pos_cmd`（默认）/ **`--action-space delta`** 可切成同语义 |
| fps 声明 | 30 | 20（可配，由 `physics_hz/every` 精确推导） |
| fps 实际 | **约 57Hz 不匀**（15/20ms 混合） | 同一物理步，无抖动 |
| 相邻帧重复 | 运动时 **0%** / 静止段重复属正常 | 运动时 **0%**（逐位比较，650 对中 0 对） |
| 主相机 | 1280×720 | 640×480（可配） |
| `episode_done` | ✅ 有 | 无（用 `session.json`，边采边写防崩溃） |
| intervention 标记 | 有（但全 1，无信息量） | 无 |
| 单条体积 | 15.5MB / 2300 帧 | 实测待补 |

### 关键差异：state/action 空间

- **同事用任务空间**（末端位姿 delta）→ 适合 ACT / Diffusion Policy 直接学末端增量
- **我们用关节空间**（32 关节 + 指令）→ 需要知道机器人结构，但**没有 FK/IK 误差**，
  且 `action` 是控制器的绝对指令，回放时不会累积漂移
- **现在两边可对齐**：`sdg.sh export --action-space delta` 输出
  `joint_pos[t+1] - joint_pos[t]`，实测自洽误差 1.86e-09；
  `--action-space ee_delta` 输出 14 维末端增量，与他们 pose 模式同构

我们文档里记录过这个判断：**EE 回放必须发指令不能发实测**，否则稳态误差叠加两次。
同事的 delta 方案天然规避了这个问题（因为是增量），但反过来在训练时需要
`state[t] + action[t] → state[t+1]` 的积分，**长序列会漂**。

### 诚实的能力对照（不掩饰对方优势）

**他们明显更强的地方**：

| 能力 | 他们 | 我们 | 影响 |
|---|---|---|---|
| 采样率 | 实测约 57Hz（15/20ms） | 20fps 仿真时间 / 13fps 墙钟 | 插拔这类精细任务，时间分辨率差 3~4 倍 |
| 主相机分辨率 | 1280×720 | 640×480 | 细节更多 |
| 人工干预标记 | 有 `intervention` 字段 | **完全没有** | 无法做"只学干预后片段"、DAgger 类训练 |
| 任务空间直接可用 | state/action 都是末端 | 需 `--action-space ee_delta` 转换 | 对齐 ACT 默认用法更顺 |

**我们明显更强的地方**：

| 能力 | 我们 | 他们 | 影响 |
|---|---|---|---|
| fps 自洽性 | `physics_hz/every` 精确推导 | 声明 30，实际 57Hz，视频按 30fps 编码 → 时间轴 2x 错 | 训练时速度语义错，**这是会毁训练的** |
| 维度干净 | 32 维全是真实量 | 32 维里 **16 维全零 pad** | 浪费一半带宽/容量 |
| 语义可配 | `absolute` / `delta` / `ee_delta` 三选 | 固定 pose delta | 研究对比友好 |
| 字段可读性 | 关节**名字**（`left_elbow_joint`） | 只有索引 + `pad_N` | 换机器人/查错成本 |
| 崩溃恢复 | `session.json` 边采边写，崩了能 pack 抢救 | 靠 `episode_done.json`（丢就丢） | 长采集稳定性 |
| 相机内参 | 每帧从 `cameraProjection` 反推，改分辨率自动跟随 | 依赖 ROS `camera_info` 发布 | 改分辨率不会悄悄错 |
| 多机器人/任务隔离 | `configs/<robot>/tasks/<task>.yaml` | 单一 `config.py` | 加任务不改代码 |
| 端到端校验 | `pack`/`diag`/`fk`/`bench`/`export-check` 5 道 | 无 | 数据坏了能定位 |
| 同步精度 | 同一物理步回调内取全部字段 → 0ms | 话题各自带 stamp，需对齐（实测 median 200ms） | 视觉-状态错位 |

**结论**：不是"谁替代谁"。他们赢在**采样率、分辨率、干预标记**（这三项是我们该补的短板），
我们赢在**时间轴正确性、维度干净、可配置性、可诊断性**。而"时间轴 2x 错"这类问题
不修的话，训练结果无法解释 —— 这正是我们平台存在的意义。

---

## 5. 该向同事要什么

按优先级：

1. **`config.py` 的实际运行值 + 启动命令**
   确认这份数据到底是 `STATE_ACTION_SOURCE = "joint"` 还是 pose 模式。
   仓库里 config 写的是 `joint`，但数据是 pose —— 说明运行时被覆盖了。

2. **采集循环的实现代码**（哪个文件、挂在什么回调上）
   搞清楚为什么帧间隔是 15/20ms 而 `FPS=30`。怀疑是渲染回调。

3. **一条 raw 轨迹**（未转 lerobot 的原始数据）+ 转换日志
   对照 raw 的采样率，确认 60Hz 是原始频率还是转换时插值的。

4. **`intervention` 字段的写入逻辑**
   为什么全 1。

5. **他们训 ACT 的 `train_config.json`**
   看 `action` 的 delta 是怎么处理的（是否做了积分/归一化）。

6. **`PI05_MAX_DIM=48` 是为了 PI0.5，那训 ACT 时是否也 pad**
   （如果 ACT 也 pad 到 48，16 维零是纯浪费）

---

## 6. 复现命令

需要 **numpy + pyarrow**。系统 python3 没有 pyarrow，用 `lerobot_ros2/.venv`：

```bash
/home/fiveages/lerobot_ros2/.venv/bin/python analysis_dcp_dataset.py
# 或指定路径
/home/fiveages/lerobot_ros2/.venv/bin/python analysis_dcp_dataset.py <数据集根目录>
```

输出 8 个部分：概览 / 时间戳 / 逐维统计 / 四元数 / action 语义 /
运动时间线（含夹爪状态）/ 跨 episode 相似度 / 静止首段。

`analysis_dcp_dataset.py` 与本文件同目录。
