# 数采方案对比评测：FaSim sdg（Isaac 内部）vs lerobot_ros2（ROS 话题）

> 状态：**规划中** · 建立于 2026-11-12 · 首个对比对象 lerobot_ros2

本文档用于记录「Isaac 内部直采」与「ROS 话题侧采集」两种方案的客观对比。
结论需要**实测数据支撑**，未测项一律标 `待测`，不做推测性结论。

---

## 1. 两个方案的定义

### A. FaSim sdg（本项目，Isaac 内部）

```
Isaac Kit 进程
├── 物理步进 ──> subscribe_physics_step_events 回调
│                  ├─ 相机：rep.annotators（rgb / depth / camera_params）
│                  ├─ 关节：Articulation.get_dof_positions / get_dof_position_targets
│                  ├─ 末端：RigidPrim(wrist_link) + 固定偏移
│                  └─ 物体：RigidPrim(tracked_prims)
└── 落盘：每 N 个物理步写一个 frame_XXXXXX.npz
```

- 全部数据在**同一个物理步回调**内取，无跨进程传递。
- 不经过 ROS、不经过 DDS。

### B. lerobot_ros2（ROS 话题侧）

```
Isaac Kit 进程                        采集进程（另一个 Python 进程）
├── OmniGraph: ROS2 发布节点   ──DDS──>  ROS2Camera(topic)
│      /joint_states                    ROS2Robot(interface)
│      /left_current_pose               LeRobotDataset.add_frame()
└── 运控节点（OCS2）
```

- `ee_traj.py`：只收 `/joint_states`、`/left|right_current_pose`、
  `/left|right_current_target`、夹爪话题。**不采相机**。
- `demo_record.py` + `lerobot_camera_ros2`：采 `/joint_states` + 相机
  `sensor_msgs/Image`，写入 LeRobot 数据集格式。
- `ros2_robot_interface/record/record_playback.py`：按 Enter 逐点记 关节/末端
  pose，存 JSON。**不采相机**。

### 关键前提差异（影响可比性）

| 项目 | FaSim sdg | lerobot_ros2 |
|---|---|---|
| 运控运行位置 | Isaac 进程内（`ROS=fiveages`） | 可进程内或独立进程 |
| 相机是否需 ROS 发布 | **不需要** | **需要**先建 OmniGraph |
| G1D 场景现成 ROS 相机 | — | **无**（room.usda 只有相机 prim，无发布图） |
| 采集进程数 | 1（Isaac 内） | 2（Isaac + 采集） |

> ⚠ **实测前提**：`room.usda` 里没有任何 `ROS2CameraHelper` 图节点，只有相机
> prim 与曝光设置。要让方案 B 跑起来，必须先给场景加 OmniGraph 发布节点。
> 这是方案 B 的**部署成本**，评测时要计入。

---

## 2. 评测项与测试方法

### 2.1 采集帧数与频率（客观）

| 指标 | 测法 | 单位 |
|---|---|---|
| 仿真时间帧率 | `framerate_sim = 1 / (physics_dt × every)` | fps |
| 墙钟帧率 | `frames_saved / duration_sec` | fps |
| **物理步速率** | `physics_steps / duration_sec` | 步/秒 |
| RTF | `(physics_steps × physics_dt) / duration_sec` | — |
| 帧间抖动 | `max(|Δt − median(Δt)|)` | ms |

**为什么要「物理步速率」**：这是与渲染开销无关的真实仿真速度，也是唯一能
跨方案公平比较的量。话题方式的帧率上限由发布频率决定，本方案的由 `every`
参数决定，直接比 fps 没有意义。

采集命令（A）：`sdg.sh capture --every 10`（20 fps 仿真时间）
采集命令（B）：`demo_record.py`（FPS=30，由订阅侧定时器驱动）

### 2.2 数据包体积与可配置性

| 指标 | 测法 |
|---|---|
| 单帧体积 | `du -sh frames/` ÷ 帧数 |
| 总包体积 | `du -sh <session>` |
| 压缩方式 | A: `np.savez` 无压缩（可切 `savez_compressed`）<br>B: LeRobot 用 `pyarrow` parquet + 图像单独存 |
| 可配采集项 | 是否支持只采关节/只采末端/不采相机 |
| 可配频率 | 是否支持任意 `every` / FPS |
| 可配分辨率 | 是否支持采集时降分辨率 |

**已知信息**：
- A 的 `run_03`：674 帧 × 3 相机 × 640×480 RGB-D ≈ **4.7 GB**（约 7.0 MB/帧）
- A 支持 `--what camera,joint,ee,object` 任意子集、`--every`、`--camera`
- B 的 `demo_record.py` 硬编码 `FPS=30`、`EPISODE_TIME_SEC`、`NUM_EPISODES`，
  需改代码；相机列表在 `camera_config` dict 里手工配

**优化空间（待测）**：
- A: 换 `savez_compressed`（实测 640×480 压缩 52ms/帧，会拖慢物理，不推荐）
- A: 降分辨率 320×240 → 体积约 1/4
- A: `depth` 关掉 → 体积约减半
- A: 只存 `episode.npz`（不含 frames/）→ 体积降到 MB 级，但丢失图像
- B: LeRobot 的图像存储可能有压缩（parquet + 独立视频），需实测

### 2.3 相机参数与内参可信度

| 指标 | A（Isaac 内部） | B（话题） |
|---|---|---|
| 图像来源 | `rep.annotators.get("rgb")` 直读渲染缓冲 | `sensor_msgs/Image`，经 DDS 传输 |
| 内参来源 | `camera_params.cameraProjection` | **`ROS2Camera` 不订阅 `camera_info`** |
| 内参是否落盘 | 是（`<cam>_K`，逐帧） | 否（除非自行加订阅） |
| 相机世界位姿 | 是（`cam/<cam>_pos/_quat`，逐帧） | 否 |
| 相机参数可调 | task.yaml 里配 resolution/annotators | `ROS2CameraConfig` 配 width/height/encoding |

**B 的关键局限（已从代码确认）**：
```python
# lerobot_camera_ros2/camera.py
from sensor_msgs.msg import Image      # ← 只有 Image，没有 CameraInfo
```
没有内参 → 深度图无法转点云、无法做投影校验、无法多相机融合。若需要，
必须自己再加一个 `camera_info` 订阅并落盘。

**内参可信度验证方法（待测）**：
1. 用内参把深度图转点云，检查同一物体的点云尺寸是否等于 USD 里的实际尺寸
   （例如试管直径）。
2. 把 `joint_pos` 经 FK 得到的 tcp 位置投影到图像，检查是否落在夹爪上。

### 2.4 数据对齐（核心优势项）

| 对齐维度 | A（Isaac 内部） | B（话题） |
|---|---|---|
| 图像 ↔ 关节角 | **同一物理步回调**，天然同刻 | 不同回调/不同进程，需靠时间戳插值 |
| 关节 ↔ 末端 | 同上 | 不同话题，需对齐 |
| 关节 ↔ 物体 | 同上 | 物体位姿通常来自 `/get_entity_state` 服务，延迟更大 |
| 时间基准 | `sim_step` + `tl.get_current_time()` | ROS 时间戳（可与仿真时钟不同步） |

**测试方法**：抓取瞬间，检查「图像里夹爪位置」与「`joint_pos` FK 得到的
tcp 位置投影」是否重合。偏差应 < 5 像素（约 1cm）。

**已知信息**：A 的 `run_03` 里图像与关节角在同一 `frame_*.npz` 里，物理步号
`sim_step` 严格单调。B 的对齐取决于发布频率与订阅延迟，需实测。

### 2.5 功能完备度

| 功能 | A（sdg） | B（lerobot_ros2） |
|---|---|---|
| 采集 | ✅ | ✅ |
| 数据体检（7 项） | ✅ `sdg.sh diag` | ❌ |
| 相机联络图 | ✅ `sdg.sh shots` | ⚠️ LeRobot 自带可视化，但非"检查"用途 |
| 切任务片段 | ✅ `sdg.sh trim` | ❌ |
| 同实例回放（关节） | ✅ | ✅（`record_playback.py` 单步/循环） |
| 同实例回放（末端→运控） | ✅ | ✅（`ee_traj replay`） |
| 回放精度量化 | ✅ `sdg.sh fk-diag`（mm 级 + 方向） | ❌（只有 `inspect` 看差值） |
| 采集进度查询 | ✅ `sdg.sh status` | ❌ |
| 交互式菜单 | ✅ | ⚠️ 部分脚本有 input() |
| 崩溃恢复 | ✅（session 边采边写） | ❌ |
| 多机器人配置隔离 | ✅ `configs/<robot>/` | ⚠️ 有 `robot.yaml` + `task_configs/` |

### 2.6 可扩展性

| 维度 | A（sdg） | B（lerobot_ros2） |
|---|---|---|
| 接入新机器人 | 新建 `configs/<robot>/`（配置隔离） | 新建 `robot.yaml` + 采集脚本改代码 |
| 接入新相机 | 改 task.yaml | 改 `camera_config` dict |
| 跟随 Isaac 新特性 | ✅ 直接用 Replicator annotators | ❌ 需 ROS 桥先支持 |
| 自定义数据字段 | 改 `live_capture` payload | 改 LeRobot dataset schema |
| 与其他仿真器对接 | ❌ 绑 Isaac | ✅ 只要发 ROS 话题即可 |

### 2.7 我补充的评测项

**7.1 部署成本**
- A: 注册扩展（`init.sh`），终端无需 source ROS。
- B: 需在场景里建 OmniGraph 发布图 + 起采集进程 + source ROS 环境。
- 测法：从零到第一次采到数据所需步骤数与耗时。

**7.2 对运控的干扰**
- A: 与运控同进程，但只挂 physics step 回调读数据，不写关节。
- B: 独立进程，理论上零干扰；但 DDS 带宽占用可能影响。
- 测法：采集前后 `physics_steps / duration_sec` 的降幅。

**7.3 数据丢失与容错**
- A: 崩溃后 `frames/` 已落盘可抢救（session.json 边写）。
- B: 采集进程崩溃 → 已写入的数据集可恢复（LeRobot 格式有 `save_episode`）。
- 测法：模拟中断，看能恢复多少帧。

**7.4 数据格式与生态**
- A: 自定义 npz，需自己写 DataLoader。
- B: LeRobot 标准格式，可直接 `LeRobotDataset` 加载、直接训练。
- 这是 **B 的重大优势**，也是 A 目前最大的短板（后续可加 `sdg.sh export --format lerobot`）。

**7.5 可复现性**
- A: `session.json` 记 `robot_key`/`task_key`/`code_stamp`/配置快照。
- B: LeRobot 有 `meta/info.json`，但采集参数（FPS、相机话题）不在里面。

**7.6 时间轴正确性**
- A: `dt = every × physics_dt`，`sim_step` 严格等距（实测抖动 0.00ms）。
- B: 由订阅侧定时器驱动，抖动取决于 DDS 与 Python GIL。
- 测法：帧间隔的标准差。

---

## 3. 初步结论（基于代码审查，未实测部分标注）

| 评测项 | 优势方 | 确定度 | 说明 |
|---|---|---|---|
| 帧数/频率 | 待测 | — | 需实测物理步速率 |
| 数据体积 | 待测 | — | 但 A 的 npz 无压缩，B 的 parquet 可能有优势 |
| 采集参数可配 | **A** | 高（代码确认） | A 有 `--what/--every/--camera`；B 硬编码需改代码 |
| 相机内参 | **A** | **高（代码确认）** | B 的 `ROS2Camera` 只订 `Image`，无 `CameraInfo` |
| 图像来源质量 | 待测 | — | A 直读渲染缓冲；B 经 DDS 可能有编码损失 |
| 数据对齐 | **A** | 高（架构决定） | A 同一回调；B 跨进程需插值 |
| 功能完备度 | **A** | 高 | A 多 6+ 个功能（diag/shots/trim/status/崩溃恢复/菜单） |
| 可扩展性（新机器人） | **A** | 中高 | A 纯配置；B 需改代码 |
| 可扩展性（跨仿真器） | **B** | 高 | B 只要 ROS 话题；A 绑死 Isaac |
| 部署成本 | **A** | 中 | A 无需建 OmniGraph；B 必须自己加相机发布 |
| 对运控干扰 | 待测 | — | A 同进程但有实测数据；B 独立进程 |
| 数据格式生态 | **B** | 高 | B 直接 LeRobot 格式可用；A 需自己写加载 |
| 容错 | **A** | 中高 | A 有 session 边写；B 依赖 LeRobot 的 episode 保存 |

**总体判断（待实测验证）**：
- A 在「数据质量 / 对齐 / 功能 / 可配置性」上有结构性优势。
- B 在「生态兼容 / 跨仿真器」上有结构性优势。
- 两者不是替代关系：**A 负责高质量数据生产，B 的格式可以作为 A 的导出目标**。

---

## 4. 评测执行计划

### 阶段 0：准备（预计 0.5 天）
- [ ] 给 G1D 场景加 OmniGraph 相机发布（`/head_camera/image_raw` 等 3 路）
- [ ] 确认 `lerobot_camera_ros2` 环境可跑（`demo_record.py`）
- [ ] 准备统一的测试任务与参数（同一轨迹、同一时长）

### 阶段 1：并行采集（预计 0.5 天）
同一段编排轨迹，分别用两种方式采：
- [ ] A: `sdg.sh capture --what camera,joint,ee,object --every 10`
- [ ] B: `demo_record.py`（FPS=30，三相机）
- 记录：帧数、物理步、墙钟时长、体积

### 阶段 2：客观指标（预计 0.5 天）
- [ ] 计算 2.1 的全部指标（含物理步速率）
- [ ] 体积对比 + 空间优化实验（降分辨率/关深度/压缩）
- [ ] 时间轴抖动

### 阶段 3：数据质量（预计 1 天）
- [ ] 内参验证（点云尺寸 vs USD 实际尺寸）
- [ ] 对齐验证（FK 投影 vs 图像中的夹爪位置）
- [ ] 抓取时刻的姿态一致性

### 阶段 4：汇总（预计 0.5 天）
- [ ] 填写实测数据到下表
- [ ] 更新结论的「确定度」列

---

## 5. 实测数据记录表

> 待阶段 1-3 完成后填写。空白 = 未测。

### 5.1 采集规模

| 指标 | A（sdg） | B（lerobot_ros2） |
|---|---|---|
| 采集命令 | `--every 10` | `demo_record.py` FPS=30 |
| 帧数 | 待测 | 待测 |
| 物理步数 | 待测 | 待测 |
| 墙钟时长 | 待测 | 待测 |
| 仿真时间帧率 | 20 fps（理论） | 待测 |
| 墙钟帧率 | 待测 | 待测 |
| **物理步速率** | 待测 | 待测 |
| RTF | 待测 | 待测 |
| 帧间抖动 | 待测 | 待测 |

### 5.2 体积

| 指标 | A（sdg） | B（lerobot_ros2） |
|---|---|---|
| 总包体积 | 待测 | 待测 |
| 单帧体积 | 待测 | 待测 |
| 关闭深度后 | 待测 | 待测 |
| 320×240 后 | 待测 | 待测 |
| 仅 episode（无机图像） | 待测 | N/A |

### 5.3 相机与对齐

| 指标 | A（sdg） | B（lerobot_ros2） |
|---|---|---|
| 内参是否落盘 | ✅ 已知 | ❌ 已知（代码确认） |
| 内参校验误差 | 待测 | N/A |
| 相机位姿是否落盘 | ✅ 已知 | ❌ 已知 |
| FK 投影 vs 图像偏差 | 待测 | 待测 |

---

## 6. 已知的公平性注意事项

1. **不要在「帧率」上直接比 fps**。A 的帧率由 `every` 参数决定（可任意），
   B 的由订阅定时器决定。要比「物理步速率」或「同一仿真时间下的数据量」。
2. **B 需要自己加相机发布图**，这是额外工作量。评测时应记录"从零到可用"
   的总成本，而不是只比运行期。
3. **两边的 action 语义要统一**。A 用 `joint_pos_cmd`/`<side>_cmd_pos`；
   B 的 `demo_record.py` 用 `sent_action`（指令），`ee_traj.py` 用
   `/left_current_target`。要确认都在比"指令"而非"实测"。
4. **LeRobot 格式本身可能做图像压缩**，比体积时要确认是否无损。

---

## 7. 后续可做

- [ ] `sdg.sh export --format lerobot`：把 A 的数据导出成 LeRobot 格式，
      兼得两者优势（高质量采集 + 生态兼容）
- [ ] 与 data-collector-platform 对比（需先确认其部署与可用性）
- [ ] 与纯 URDF/MuJoCo 离线渲染对比（无运控闭环场景）
