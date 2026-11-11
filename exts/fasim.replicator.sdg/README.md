# FaSim 合成数据采集（sdg）

在**正在运行编排**的 Isaac Sim 会话里旁路采集训练数据：相机 RGB-D、关节角
（实测 + 指令）、末端位姿（实测 + 指令）、交互物体位姿。

采集挂在 `physics step` 回调上，与渲染/控制在同一个 `app.update()` 时钟里，
**不接管控制、不起额外线程**，所以不会干扰运控。

---

## 快速开始

```bash
# 1) 启动仿真（另一个终端，自动带运控）
cd ~/FaSim-Isaac && ./run.sh

# 2) 交互式采集（推荐）
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
./scripts/sdg.sh                # 无参数 = 主菜单
#   → 采集 → 选采集项/相机/频率 → 跑编排 → 回主菜单选「结束采集」

# 3) 转格式 + 体检
./scripts/sdg.sh pack ~/fasim_data/run_01      # 提示缺 --robot 时补上
./scripts/sdg.sh diag ~/fasim_data/run_01/episode.npz
./scripts/sdg.sh shots ~/fasim_data/run_01 --segment   # 相机联络图
```

主菜单会记住上次的机器人/任务/参数（`~/.cache/fasim_sdg/menu_state.json`），
下次回车即沿用。

---

## 部署

```bash
./init.sh          # 或按仓库既有流程注册扩展
```

扩展 `fasim.replicator.sdg` 随 Isaac 启动自动加载，通过
`<runtime>/capture.sock`（Unix domain socket）接受指令。终端侧的
`sdg.sh` 只是在跟这个 socket 说话，**不需要 source 任何环境**
（`fk` / `shots` 等离线命令同理，只依赖 numpy/PyYAML）。

部署检查：

```bash
./scripts/sdg.sh doctor
```

---

## 配置：机器人与任务隔离

**本仓库的代码里不含任何具体机器人名**。所有机器人相关信息都在配置目录下：

```
configs/
├── _default.yaml                 # 全局默认（输出目录、物理参数）
└── <robot_key>/
    ├── robot.yaml                # 机器人级：怎么找到它、怎么读它的位姿
    └── tasks/
        └── <task_key>.yaml       # 任务级：场景、相机、交互物体
```

### `robot.yaml` —— 机器人级

| 字段 | 作用 | 换机器人要改吗 |
|---|---|---|
| `articulation_prim` | Isaac 里 articulation 的根 prim | ✅ 必改 |
| `base_prim` | 底盘 prim（世界→机体换算） | ✅ 必改 |
| `ee_prims` | 名义 tcp frame（诊断对照用） | ✅ 必改 |
| `ee_links` | **末端真值来源**：父刚体 + 固定偏移 | ✅ 必改 |
| `planning` | **末端指令位姿来源**：运控 URDF + ee_frames | ✅ 改 `ee_frames` |
| `gripper_joints` | 夹爪关节名 | 视情况 |
| `expected_dof_count` | 关节总数（自检用） | 视情况 |

### `tasks/<task>.yaml` —— 任务级

| 字段 | 作用 |
|---|---|
| `scene_usda` | 场景文件（相对仓库根或绝对路径） |
| `variant_selection` | 打开场景后要切的 variant（如 `ROS: fiveages`） |
| `cameras` | 相机列表：`name` / `prim` / `resolution` / `depth` |
| `tracked_prims` | 需要记录位姿的物体：`<label>: <prim_path>` |

### 接入一个新机器人 / 新任务

```bash
mkdir -p configs/<new_robot>/tasks
cp configs/unitree_g1d/robot.yaml configs/<new_robot>/robot.yaml
# 改 articulation_prim / base_prim / ee_links / planning.ee_frames
$EDITOR configs/<new_robot>/tasks/<new_task>.yaml
```

**不需要改代码。** 若目录下只有一个候选，`sdg.sh` 会自动选中；有多个才问。

### `ee_links` 与 `planning` 为什么要分开

这是本项目最容易混淆的一点，但必须分开：

| | `ee_links` | `planning` |
|---|---|---|
| 给出什么 | 末端**实测**位姿 | 末端**指令**位姿 |
| 数据来源 | `RigidPrim` 直读 PhysX | 运控 URDF + 关节指令角做 FK |
| 训练用途 | observation | **action** |
| 依赖 | 无 | 需要运控那份 planning URDF |

为什么实测不能直读 tcp：tcp 是纯 Xform（fixed joint 的子 frame），而
newton kit 关了 PhysX→USD/Fabric 同步（`updateToUsd=false` /
`fabricUpdateTransformations=false`），所以 `XformPrim` 任何后端都读到
**authoring 静态值**（实测 765 帧偏离 0.0mm）。必须读父刚体再叠固定偏移。

为什么指令位姿要用 FK：指令位姿只存在于运控 MPC 内部，Isaac 侧只有关节
指令角 `joint_pos_cmd`（`get_dof_position_targets`，与实测无时间滞后）。
用运控**自己加载的那份** planning URDF 做正解，与 `eeFrame` 同源。

### `planning.urdf` 的解析规则

```yaml
planning:
  urdf: auto          # auto=从运控缓存自动找 / none=关闭 / <path>=显式
  cache_dir: ""       # 留空 = $OCS2_PLANNING_URDF_DIR 或 /tmp
  ee_frames:          # 对应运控 .info 里的 eeFrame / eeFrame1
    left: left_tcp
    right: right_tcp
```

运控缓存布局（见 `robot_common_launch/common/robot_utils.py`）：

```
$OCS2_PLANNING_URDF_DIR (默认 /tmp) / ocs2_ros2 / <robot> / planning_urdf / <hash>.urdf
```

hash 由 xacro 映射参数决定、无法复算，所以按 **mtime 取最新**（改过配置会
生成新 hash，最新的就是当前在用的）。

设 `urdf: none` 可关闭 FK —— 此时 EE 回放会退回实测位姿，精度差 7~9mm。

---

## 数据字段：什么是真值，什么是命令

`episode.npz` 里同一个量常常有两份，混用会让训练效果变差：

| 字段 | 语义 | 训练用途 |
|---|---|---|
| `joint_pos` | 实测关节角（PhysX） | **observation** |
| `joint_pos_cmd` | 下发给 drive 的目标关节角 | **action** |
| `<side>_pos` / `_quat` | 末端实测位姿（刚体读） | **observation** |
| `<side>_cmd_pos` / `_cmd_quat` | 末端指令位姿（FK 反算） | **action** |
| `<side>_pos_xform` / `_quat_xform` | Xform 直读的静态对照值 | **忽略**（仅诊断） |
| `obj/<label>_pos` / `_quat` | 交互物体位姿（刚体读） | observation |
| `<cam>_rgb` / `<cam>_depth` | 相机图像 | observation |
| `<cam>_K` / `<cam>_pos` / `<cam>_quat` | 相机内参 + 世界位姿 | 点云/标定 |

**为什么 action 必须是 cmd 不是 actual**：

```
编排:   目标 → 运控 → 实测 = 目标 − 7~9mm      ✅
回放:   实测 → 运控 → 实测 − 7~9mm            ❌ 稳定偏下
```

实测末端比指令低 `mean 7.2mm / max 32mm`、`dz +6.4mm`（实测值）。
用实测当 action，部署时运控会二次叠加同一个稳态误差 —— 这就是
「抓取位置稳定偏下 2-3cm」的成因。

---

## 命令一览

| 命令 | 作用 | 需要 Isaac |
|---|---|---|
| `sdg.sh` | 交互式主菜单 | 否（采集时需 Isaac 在跑） |
| `capture` | 开始采集 | Isaac 需在跑 |
| `status` / `stop` | 查看/结束采集 | Isaac 需在跑 |
| `pack` | 会话 → `episode.npz`（含指令位姿反算） | 否 |
| `diag` | 一键体检（7 项） | 否 |
| `shots` | 相机联络图 | 否 |
| `trim` | 切任务片段 | 否 |
| `fk` / `fk-diag` | FK 对拍 / 回放体检 | 否 |
| `live` | 同实例回放（joints / ee） | 是 |
| `doctor` | 环境自检 | 否 |

`fk` / `diag` / `fk-diag` 会优先找带 pinocchio 的 python
（`SDG_FK_PYTHON` 可指定）；找不到也能跑 —— 本项目的 FK 是**纯 numpy** 实现，
pinocchio 只用于可选的对拍自检。

---

## 数据体检

采完先跑这三条，全绿再拿去训练：

```bash
./scripts/sdg.sh pack <session> --robot <robot>   # 看「末端指令位姿」是否写入
./scripts/sdg.sh diag <episode.npz>               # 7 项体检，末尾给结论
./scripts/sdg.sh shots <session> --segment        # 拼图，人眼确认画面
```

`diag` 的 7 项：

1. 末端位姿是动态值还是死值
2. 末端 vs FK 交叉验证（应 < 5mm）
3. 关节角口径（有没有 `joint_pos_cmd`）+ drive 稳态误差
4. 时间轴（帧间隔来源、抖动）
5. 末端与交互物体的接近量
6. 物体位姿（静止的不该动、被操作的要动）
7. 相机（RGB 统计、深度范围、RGB/深度尺寸一致性）

**注意**：`shots` 只做量化检查（全黑/全白/冻结/尺寸不符），画面内容仍需
人眼看拼图 —— 看得到物体、夹爪、抓取过程才说明采到了有用的图像。

---

## 采集频率

| `every` | 仿真时间帧率 | 说明 |
|---|---|---|
| 5 | 40 fps | 最细，开销最大 |
| 10 | 20 fps | **推荐** |
| 20 | 10 fps | 省开销 |

实际墙钟速度受 RTF 限制（实测本机 3 相机时 RTF≈0.65，约 13 fps 墙钟）。
**跨采集方式比较时用 `physics_steps / duration_sec`**（物理步/秒），
那是与渲染开销无关的真实仿真速度。

---

## 目录结构

```
exts/fasim.replicator.sdg/
├── configs/                      机器人/任务配置（唯一需要改的地方）
├── fasim/replicator/sdg/
│   ├── live_capture.py           ← 实际采集逻辑（由扩展调用）
│   ├── live_replay.py            同实例回放（joints / ee）
│   ├── fk.py                     纯 numpy FK（末端指令位姿 + 离线对拍）
│   ├── pack.py                   会话 → episode
│   ├── episode.py                episode 格式
│   ├── config_io.py              配置加载
│   ├── renderer.py               层2 离线渲染
│   └── scripts/extension.py      随 Isaac 加载的 socket 服务
└── scripts/
    ├── sdg.sh                    统一入口（无参数 = 菜单）
    ├── sdg_menu.py               交互式主菜单
    ├── cli.py                    子命令实现
    ├── camera_shots.py           相机联络图
    └── probe_camera_params.py    相机参数字段探针（升级 Isaac 后自查用）
```

---

## 已知限制 / 后续

- **物体回放**只做了「复位到第 0 帧后交给物理」（`--no-reset-objects` 可关）。
  逐帧强制摆放物体未实现 —— 那份给纯渲染合成数据用，需要时再加。
- **`_xform` 字段**是诊断对照值，训练时必须忽略。
- 相机参数依赖 `camera_params` annotator（Isaac 6.2 实测可用）。升级后跑
  `python.sh scripts/probe_camera_params.py` 复核字段与布局。
