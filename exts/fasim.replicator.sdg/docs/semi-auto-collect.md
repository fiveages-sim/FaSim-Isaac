# 半自动数采：一键起编排 + 任务完成检测

> 面向 G1D 试管任务（`unitree_g1d` / `sinopec_test_tube`）。
> 判据是任务专用的，不做通用抽象。

---

## 一、解决什么问题

原来一轮采集的操作链：

```
1. ./run.sh                      起 Isaac（运控随 Play 进程内启动）
2. ./sdg.sh randomize            随机化物体（已就绪）
3. ./sdg.sh capture              开始采集
4. cd ~/lerobot_ros2 && motion-generation ...   ← 切仓库、source venv、敲长参数
5. ./sdg.sh stop                 人工判断跑完了，手动停
```

痛点：

- **第 4 步**每次都要切仓库、记参数。最容易踩的坑是忘加 `--no-reset`
  —— 编排会把刚随机化的物体冲回 USD 初始位。
- **第 5 步**得盯着看，而且"编排跑完"≠"任务做成"。编排超时/插偏了
  照样会往下走、照样回 HOME —— 采到的是**失败轨迹**，混进训练集会教坏策略。

现在这两步都收进 sdg 了。

---

## 二、一键起编排

### 用法

```bash
# 预检（只验证仓库/venv/映射，不启动）
./sdg.sh motion --robot unitree_g1d --task sinopec_test_tube --check

# 前台跑（看输出，Ctrl+C 可中断，不影响 Isaac）
./sdg.sh motion --robot unitree_g1d --task sinopec_test_tube

# 后台跑（立即返回，日志在 /tmp/motion_generation.log）
./sdg.sh motion --robot unitree_g1d --task sinopec_test_tube -b
```

菜单：主菜单 →「一键起编排（跑一遍任务）」。不给 `--robot/--task` 时
沿用菜单上次选的上下文。

### 自动带上的两个开关（不要去掉）

| 开关 | 为什么必须 |
|---|---|
| `--no-reset` | 编排默认 reset 环境（timeline STOP+PLAY），会把随机化的物体打回 USD 初始位 |
| `--no-ensure-ros2-stack` | G1D 的 `robot.yaml` 里 `ros2_stack.motion.required: false` —— 运控随 Isaac 的 Play 在**进程内**启动。再加这个会启动第二套控制器抢同一个 domain |

### 实际执行的命令

```bash
cd ~/lerobot_ros2 && source .venv/bin/activate && \
  motion-generation --robot unitree_g1d --task-key pick_tube_full \
    --scene default --no-reset --no-ensure-ros2-stack \
    --workspace ~/lerobot_ros2/examples/IsaacSim
```

### `--workspace` 必须显式传（踩过的坑）

编排的 `--workspace` 默认取 **cwd**。在 `~/lerobot_ros2` 根目录跑会报：

```
ValueError: Unknown robot key: 'unitree_g1d'
```

因为机器人定义在 `examples/IsaacSim/robots/` 下，不在仓库根目录。
更坑的是**根目录也有个 `robots/Dobot/CR5`** —— 从根目录跑能找到"一个
workspace"，但里面没有目标机器人，报错信息完全看不出是路径问题。

所以 `motion_launch.find_workspace()` 会扫 `robots/**/robot.yaml` 找
`key:` 匹配的那个，自动定位 workspace，并显式传 `--workspace`。

### 关于「要不要 source lerobot_ros2 的环境」

**不需要你手动 source**。`motion_launch._wrap_shell()` 生成的命令里已经包含
`source <repo>/.venv/bin/activate`：

- `motion-generation` 是那个 venv 的 console script（依赖 `rclpy`、
  `robot_action_composer` 等），系统 python 没有这些
- sdg 侧的 python 进程**不能**直接 import 编排的包（不同 venv），所以走子进程

你只需要保证 `~/lerobot_ros2/.venv` 存在（`./init.sh all-motion` 建的）。

### 任务名映射

sdg 和编排是**两套独立的任务定义**，名字不一定相同：

| sdg | 编排 |
|---|---|
| `unitree_g1d` / `sinopec_test_tube` | `unitree_g1d` / `pick_tube_full` |

映射表在 `fasim/replicator/sdg/motion_launch.py` 的 `DEFAULT_MAP`。
没配映射时按同名处理。

### 找仓库的顺序

1. `--repo <path>`
2. 环境变量 `LEROBOT_ROS2_DIR`
3. `~/lerobot_ros2`

判定标准是存在 `submodules/robot_action_composer/`。

### 注意：改 `motion_launch.py` 不用重启 Isaac

它是外部 CLI 用的模块。但改 `extension.py` / `live_capture.py` /
`completion.py` **必须重启 Isaac**（进程内模块，import 后缓存在 `sys.modules`）。

---

## 三、任务完成检测

### 判据

`session.json` 里多了 `stop_reason` 和 `done` 两个字段：

```
stop_reason: task_completed | manual | max_frames
done:        {reason, object, rack, d_xy_mm, dz_mm, tilt_deg, frame, ...}
```

判据是三个条件**同时**满足：

1. **xy 已对准**：试管中心与 `hole_3` 世界位置的水平距离 `< 25mm`
2. **z 已插入**：`dz = 试管中心 − 孔底` `<= -60mm`
3. **姿态仍竖直**：试管轴倾角 `< 30°`

### 为什么必须有 z（重要）

`insert` 技能的流程是「悬停孔口上方 → 空中对齐 xy → 垂直下探」。
**悬空对准阶段 xy 就已经接近 0**。实测 run_001：

| 帧 | d_xy | dz | 实际状态 |
|---|---|---|---|
| 300 | 12.2mm | +46.3mm | 悬停上方 |
| 330 | **0.15mm** | +39.8mm | 对准完成，**还悬在空中** |
| 360 | 2.11mm | −15.1mm | 开始下探 |
| 380 | 3.26mm | **−101.4mm** | 真正插到底 ✅ |

只用 xy 判据会在帧 305 就报完成，比真实完成（376）**早 71 帧 / 3.5 秒**，
而且失败的轨迹也会被判成功。

### 关于 xy 阈值的教训（走过一次弯路）

第一版设 `xy_tol_m = 0.020`，依据是「41 条里 39 条 ≤20mm」。**这是错的**：

- 41 条**全部**倾角 ≤13.6°、dz 全部 −98~−102mm —— 都真的插进去了。
- `dxy` 分布是**连续的**：0.65 → 21.82mm，最大相邻间隔只 2.01mm，
  **没有任何自然分界**。20mm 纯粹是从分布尾部硬砍的，没有物理意义。
- 被砍掉的 2 条（`run_011` 21.82mm / `run_023` 20.67mm）倾角仅
  13.6°/12.9°、dz 都是 −98mm，**与其他成功样本同量级**，只是落在孔的边缘
  （孔本身有宽度，试管 Ø13mm 插入时允许偏移）。

**xy 阈值的作用不是「偏多少算失败」，而是「防止把别的孔或架外当成目标孔」**
—— 孔间距约 30mm、架长 200mm，取 25mm 合理。

识别「插歪 / 碰倒」靠 `max_tilt_deg`：正常插入 0.85~13.62°，碰倒 >45°，取 30° 分界。

### 阈值依据（41 个 run，全部真实插入成功）

```
dxy:  0.65 ~ 21.82mm   （连续分布，无自然分界）
dz:   -98 ~ -102mm     （全部到位）
tilt: 0.85 ~ 13.62°    （全部接近竖直）
```

用 `xy<25mm 且 dz<-60mm 且 tilt<30°` → **41/41 全部判定成功**，
命中帧 375~410（中位数 383，与编排实际插入时刻一致）。

反向验证（5 种失败模式全部正确排除）：

| 场景 | 结果 |
|---|---|
| 悬空对准（xy=0, dz=+40） | ✅ 不判完成 |
| 没插进去（横躺 dz=0, tilt=90°） | ✅ 不判完成 |
| 碰倒（dz=−98 但 tilt=60°） | ✅ 不判完成 |
| 偏到别的孔（xy=40mm） | ✅ 不判完成 |
| 掉在架外（xy=179mm） | ✅ 不判完成 |

### 随机化适配

架子被随机平移/旋转后，`hole_3` 的世界位置必须**每帧现算**：

```
hole_world = rack_pos + R(rack_quat) @ hole_offset
```

所以判据天然跟着随机化走，不需要额外配置。已验证：架子平移 3~4cm、
旋转 15°/30° 后判据依然准确。

`hole_offset` 是**资产常量**（`(-0.021, -0.010, 0.110)`，取自编排
`[ObjectBind]` 日志的 `/get_prim_attribute` 实测值），不随随机化变。

### 为什么不在 `_emit_frame` 里直接停采集

`stop_capture()` 会 detach annotator、写 session.json，在物理步回调里做
这些会打断步进。所以：

- 物理步回调只做「numpy 旋转 + 求距离」+ 置 `_S["done_info"]` 标志
- `extension._pump_auto_stop()`（在 `app.update` 侧）检查标志后收尾

### 当前配置：只标注，不自动停

任务 yaml 里 `stop_on_complete: false`。

理由：**要采完整过程**（初始状态 → 完整任务 → 回到 HOME），不截断尾部。
这样 ACT 能学到从待命到归位的全套动作。

如果以后想省时间（实测尾部有 3~4.5s 纯 HOME 画面，占 12~18% 帧），
改成 `true` 即可，届时 `stop_reason` 仍是 `task_completed`。

### 配置项

```yaml
done_when:
  object: test_tube              # payload 里 obj/<label>_pos 的 label
  rack: test_tube_rack           # 目标架
  hole_offset: [-0.021, -0.010, 0.110]   # hole_3 在 rack 局部系（米）
  xy_tol_m: 0.025                # 只用于排除「别的孔」，非精度要求
  dz_inside_mm: -60.0            # 试管中心相对孔底的高度上限（None=不限）
  max_tilt_deg: 30.0             # 倾角上限，识别碰倒/横躺（None=不限）
  consecutive_frames: 8          # 连续多少帧满足才算（防抖）
  stop_on_complete: false        # true=命中即停
```

不写 `done_when` 段则完全不做检测（零影响）。

### 查看检测状态

```bash
./sdg.sh status      # 采集中的话会显示当前 xy/dz 和是否已判完成
```

---

## 四、验证过的内容

```
✅ 41 个历史 run 回放：39/41 正确判定，命中帧 377~410
✅ 随机化适配：架子平移 3~4cm / 旋转 15°30° 后判据依然准确
✅ 反例排除：试管还在右架（xy 208mm）、悬空对准（dz +40mm）都不误判
✅ motion 命令拼串正确（参数用空格不是 &&）
✅ motion-generation --help 接受所有传入参数
✅ 所有改动文件 py_compile 通过
✅ yaml 解析出 done_when + randomize
```

**待实测**（需要重启 Isaac 后真跑一轮）：

- 采集期间 `status` 显示完成检测状态
- 一轮完整流程：randomize → capture → motion → stop，session.json 有
  `stop_reason: task_completed`

---

## 六、一键数采

一轮完整采集收成一个命令：

```bash
# 跑 3 轮（每轮重新随机化）
./sdg.sh auto --robot unitree_g1d --task sinopec_test_tube --rounds 3

# 只预检
./sdg.sh auto --robot unitree_g1d --task sinopec_test_tube --dry-run

# 不裁剪（保留完整原始帧）
./sdg.sh auto --robot unitree_g1d --task sinopec_test_tube --no-trim
```

菜单：主菜单 →「一键数采」。

### 流程

```
[预检]    Isaac 在跑？socket 通？编排仓库找得到？
   ↓
[快照]    没有初始位姿快照就自动记（关键时机：物体还在初始位）
   ↓
[随机化]  socket randomize
   ↓
[开始采集] socket start
   ↓
[跑编排]  子进程 motion-generation（阻塞到跑完）
   ↓
[结束采集] socket stop
   ↓
[裁剪]    trim whole 模式
   ↓
[打包]    pack → episode.npz
```

任一步失败立即停止，**不删数据**（半途失败时最需要看现场），
`failed_at` 字段指出卡在哪。

### 产物

```
~/fasim_data/unitree_g1d/sinopec_test_tube/run_NNN/
├── frames/                 原始帧（全量，含待机段）
├── session.json            含 stop_reason / done
├── episode_trimmed.npz     ★ 训练用：留开头 home + 截到回 home
├── episode.npz             完整原始数据（pack 格式）
└── segments/trim_report.json
```

目录用 `run_NNN` 序号（自动取下一个未用编号），与历史数据一致 ——
早期版本用过时间戳风格（`20261116-175133`），会让排序和批量脚本出错，
已改。

### 为什么每轮独立、不用编排的 `num_runs`

编排支持 `num_runs`，但那会把多轮挤在**一个 session** 里，随机化只能在
开头做一次。而数采的核心诉求恰恰是「每轮换物体位置」。所以外壳循环：
每轮 = 一次完整流程，得到独立的 `run_NNN/`。

---

## 七、待机段裁剪（trim）

### 为什么必须裁

`run_42` 实测：1043 帧 / 52.2s，但只有 28.2s 在动。开头 12.65s 是等
OCS2 就绪的静止，结尾 11.25s 是跑完后的静止。**54.2% 的帧是有效动作**，
其余对 ACT 训练只是噪声（会学到"长期不动"）。

### 但要保留什么（重要）

不能只留"活跃帧" —— 训练要学完整动作：

- **开头保留一小段 home 静止**：让策略知道"从待命状态开始"
- **结尾保留回 home 的完整过程**：这是要学的收尾动作

所以 `--mode whole` 的语义是「整段保留、只裁掉两头多余的待机」，
而不是「切成多个活跃段」（那是 `--mode segments`，用于一个 session
里跑了多个任务的情况）。

### 参数

```bash
./sdg.sh trim <session> [--write]           # 默认 whole 模式
  --head 2.0     开头保留多少秒静止（home 上下文）
  --tail 1.5     最后一次运动后多留多少秒（等停稳）
  --mode segments   旧行为：找所有活跃段各切一条
```

`run_42` 实测：`--head 2.0 --tail 1.5` → 保留帧 253-817（565 帧 / 28.2s），
开头 2.5s 静止 + 完整任务 + 回 home 到接近零位（手臂偏差 0.006 rad）。

---

## 八、踩过的坑

### 编排 `Unknown robot key`

编排的 `--workspace` 默认取 **cwd**。在 `~/lerobot_ros2` 根目录跑会报错，
因为机器人在 `examples/IsaacSim/robots/` 下；而根目录**也有个
`robots/Dobot/CR5`**，让人误以为路径没问题。

已修：`find_workspace()` 扫 `robots/**/robot.yaml` 找 `key:` 匹配的目录，
自动定位并显式传 `--workspace`。

### 快照和数据写到两个不同目录

任务 yaml 的 `output.dir` 原本是 `~/fasim_sdg`，而 `capture` 的默认输出走
`runtime.data_root()` = `~/fasim_data`。结果**快照落在 fasim_sdg，采集数据
落在 fasim_data** —— 随机化读到的是另一份快照。

已修：yaml 的 `output.dir` 改为 `~/fasim_data`，与历史数据同处。

### 回 HOME 拖拽试管架

`insert` 的 `retreat_distance: 0.08` 沿**插入轴**退 8cm，但夹爪是侧面夹的，
退出后仍在试管正上方很近处。直接切 HOME 时手臂向内收会扫过试管架：

- `run_43` 失败：末帧 `tilt=35.8°`、`dxy=57.4mm`（试管被拖出孔并撞歪）
- `run_42` 也轻微拖拽：帧 700→760 试管相对架子移动约 2mm

**修法**：insert 后加一步竖直抬 12cm。

### 抬手步耗满 8 秒超时的真因（不是 movel_duration）

实测日志：

```
TaskQ move_relative timeout: left_arm(pos_dist=0.0101, ori_dist=0.0000, elapsed=8.05s)
```

而 `pose_tol_pos = 0.02` —— **0.0101 < 0.02 本该判"到达"**。矛盾。

真因是**参数污染**：

1. `insert` 的 params 里写了 `pose_tol_pos: 0.0015`（为了让插入更精确）
2. `skill_insert` 末尾有一句 `ctx.task_cfg = replace(ctx.task_cfg, common=qt.common, ...)`
   把含 `0.0015` 的 common **写回了全局 `ctx.task_cfg`**
3. 后续 `move_relative` 用 `ctx.task_cfg.common` 做到位判定 → `0.0101 > 0.0015`
   → **永远判不到** → 耗满 `max_stage_duration: 8.0`

`movel_duration: 0.6` 只让**规划**更快，并不解决判定问题。

**修法**（YAML 侧，不改子模块）：在抬手步显式写 `pose_tol_pos: 0.02`，
把容差重置回正常值。

### 目录命名：`run_NNN` vs 时间戳

`auto` 最初用 `runtime.default_session_dir()`（时间戳风格，如
`20261116-175133`），而本项目历史数据全是 `run_001` / `run_042`。
混在一起会导致：

- 按名字排序时时间戳目录跑到 `run_*` 前面
- 分不清"这是第几条"
- 批量脚本若假设 `run_*` 前缀会漏掉

已修：新增 `runtime.next_run_dir()`，自动扫描已有 `run_NNN` 取下一个序号
（`run_046` 这种）。手动 `capture --out` 仍可任意命名。

早期的 `run_42`（无补零）是手动建的，已统一为三段补零风格。

---

## 十、长时间运行（几十轮）的注意事项

### 实测：RTF 会逐轮下降（重启可恢复）

| 批次 | run | RTF | 相对首轮 |
|---|---|---|---|
| 重启后 | 052 | 138.8Hz | — |
| | 053 | 129.9Hz | −6% |
| | 054 | 116.3Hz | −16% |
| | 055 | 101.8Hz | −27% |
| | 056 | 92.6Hz | **−33%** |

重启 Isaac 后恢复（046=131Hz → 重启 → 052=138Hz 验证过）。

**已排除**：内存（Isaac 仅 16.7G / 125G）、GPU 显存（4.6G / 24.5G）、
物理步订阅泄漏（正确 unsubscribe）、`next_run_dir` 扫描开销（0.14ms）。

**已修复但效果有限**：annotator 累积（每轮新建 9 个对象、只 detach 不销毁）
→ 改为按 key 缓存复用。修复后起点从 131Hz 提升到 138Hz，但**下降速率没变**。

**剩余嫌疑**：render product 不 `destroy`（代码特意保留以避免破坏 ROS 图
writer 的悬空引用，见 `live_capture.stop_capture` 注释）。每轮不新建（按
`_rp_alive` 复用），但 Replicator 内部注册状态可能仍有增长。

**当前策略**：每 5 轮重启 Isaac。`auto` 已加提醒：

```
──────────────────────────────────────────────────────────────
已跑 5 轮，本轮 RTF≈101Hz
建议现在重启 Isaac —— 实测 RTF 逐轮下降（5 轮约 -33%）
  1) 关掉 Isaac 窗口，重新 ./run.sh 并 Play 到待命状态
  2) 再跑剩下的 45 轮:
     sdg.sh auto --robot unitree_g1d --task sinopec_test_tube --rounds 45
  数据目录按 run_NNN 递增，不会覆盖已有包
──────────────────────────────────────────────────────────────
```

汇总里也会打 RTF 趋势，超过 20% 会警告：

```
RTF 趋势: 138Hz → 92Hz（-46Hz / -33.3%）
⚠ 劣化超过 20%，建议重启 Isaac 后再继续采集
```

### 磁盘（已清理，auto 已自动化）

**实测压缩比**（run_046）：

| 内容 | 大小 |
|---|---|
| 原始 frames（790 帧，3 相机 + 深度） | 5.5G |
| 导出后的 LeRobot 数据集 | **17M**（320 倍压缩）|
| 保留 8 帧后的 raw 目录 | 58M |

**auto 的默认流程**（`batch_export`）：

```
每轮：随机化 → 采集 → 编排 → trim → pack        （frames 全部保留）
  ↓  跑完所有轮
批量：把所有轮的 frames 合并导出成**一个** LeRobot 数据集
  ↓  导出成功后
清理：每包只留 8 帧（均匀采样）用于数据质监
```

### 为什么「跑完再批量导出」而不是「每轮导出」

我实测过两者的时间构成：

| 阶段 | 耗时 | 能否批量摊销 |
|---|---|---|
| 启动 python + import lerobot | ~40ms | ✅ |
| 读 frames + 组 features | ~1s | ❌ 每包必做 |
| **视频编码（SVT-AV1）** | **~45s** | ❌ **按 episode 逐个编码，无法合并** |

**视频编码占 95%，而它无法摊销** —— 所以两种方式**时间上几乎没差别**
（只差 40ms × N）。

真正的区别在**产物结构**：

| | 每轮导出 | 批量导出 |
|---|---|---|
| 产物 | 20 个独立数据集 | **1 个数据集 / 20 episodes** |
| 训练 | 要循环加载 20 个再合并 | `LeRobotDataset(root=...)` 直接用 |
| 归一化统计 | 各算各的 | 全局统一 |
| 参数一致性 | 发现不了不一致 | 有守门（不一致会拦下）|

**磁盘代价**：攒着期间每轮占约 5G。N 轮 = 5N GB。
50 轮就是 250G —— 这是必须留够的空间。

想改回每轮导出：加 `--export-each-round`。
完全不想导出（只留 raw）：加 `--no-export`（此时**不会删 frames**）。

### 安全保证：只有导出成功才删 frames

`export_lerobot` **强依赖 `frames/*.npz`**（读 `<cam>_rgb` 编码成 mp4）。
所以清理逻辑有硬性前置条件：

1. 本次导出成功 → 可以删
2. 或目录里已有完整的 LeRobot 数据集（`meta/info.json` 存在）→ 可以删
3. 否则 → **保留全部 frames**，并打 WARN

导出失败时不会丢任何数据，可事后手动重试：

```bash
./sdg.sh export <任务目录> --out ~/lerobot_datasets/xxx_20eps
```

### 保留几帧、怎么保留

`--keep-frames N`（默认 8）：

- **均匀采样**（含首末帧）而不是取前 N 帧 —— 前后帧能看出动作进展，
  只留开头看不到任务中段/结尾的状态
- `session.json` 里会记 `frames_pruned.kept_indices`，并明确标注
  **「frames/ 已不连续，不要再当作完整序列处理」**
- 用途：`sdg.sh shots`（相机联络图）做人工抽检、出问题时回看

设 0 = 全删，设 -1 = 不删。

### 早期数据迁移

在 auto 自动化之前采的包，用一次性脚本补做：

```bash
cd exts/fasim.replicator.sdg && PYTHONPATH=. \
  python3 scripts/diagnostics/migrate_export_prune.py
```

脚本同样保证「先导出成功、再删 frames」，已有完整数据集则跳过导出。

### 清理历史数据（手动，已完成一次）

原始 269G → **99G**（释放 170G）：

- 删 `legacy_v1` 41 条的 `frames/`：−137G
- 删所有 `_idle_prefix/`（trim 切出的待机帧，设计上可删）：−33G
- 完整备份 2 条到 `_legacy_backup/`
- 保留每包 `session.json` + `episode.npz`（后者约 1MB/包，含
  `joint_pos_cmd`/末端/物体，几乎不占空间，值得留）

### 单轮失败自动重试

`--retries 1`（默认）：某轮因 socket 超时/编排异常退出而失败时，
**用新的 run_NNN 目录**重跑一次（失败的目录保留，便于排查）。

### 建议的启动方式

```bash
# 每 5 轮会提醒你重启；一次给足总数即可
cd ~/FaSim-Isaac && ./exts/fasim.replicator.sdg/scripts/sdg.sh auto \
  --robot unitree_g1d --task sinopec_test_tube \
  --rounds 50 --retries 1 2>&1 | tee ~/auto_50rounds.log
```

看到重启提醒时：关 Isaac → `./run.sh` → Play → 重跑剩余轮次。

### 其他风险点（已核查）

| 项 | 状态 |
|---|---|
| render product 累积 | ⚠ 按 key 复用，但可能是劣化主因（待查） |
| annotator 累积 | ✅ 已修（缓存复用） |
| `next_run_dir` 开销 | ✅ 51 目录时 0.14ms |
| socket 超时 | ✅ 60~180s |
| 内存 / 子进程 | ✅ 无泄漏 |
| 磁盘写满 | ✅ 已加预检 |

---

## 十一、数据批次标记

`session.json` 里新增 `batch` / `batch_note` 字段，用来区分采集时期：

| batch | 条数 | 特征 |
|---|---|---|
| `legacy_v1` | 41 | 早期：有抬手但**无 FSM HOME 归位**（结束停在抬手后）、无完成检测字段、每轮约 24s |
| `v2_transition` | 4 | 过渡期：抬手+回 home 已有，但抬手未优化/检测未启用（run_042 有 1043 帧偏长） |
| `v2_full_cycle` | 6 | 完整循环：随机化→采集→编排→裁剪→打包，含抬手+FSM HOME |

**全部 51 条体检结果：插入全部正确**（末帧 dxy<25mm、dz<-60mm、tilt<30°）。

差异只在**结尾**：`legacy_v1` 结束在「抬手后停住」，`v2_*` 结束在
「抬手 → 回 HOME」。ACT 学 `obs→action` 映射，结尾分布不一致会导致策略
在任务完成后行为摇摆，所以**不要混训**（详见下节）。

按 batch 筛选：

```bash
# 只看某个批次
python3 -c "
import json,glob
for d in sorted(glob.glob('run_*')):
    s=json.load(open(f'{d}/session.json'))
    if s.get('batch')=='v2_full_cycle': print(d, s['frames_saved'])
"
```
