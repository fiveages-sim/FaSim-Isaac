# fasim.replicator.sdg

FaSim 机器人的**离线合成数据采集**扩展（基于 Isaac Replicator）。

## 三层架构

> **两条采集路线并存**：
> - **层2 离线回放**（本 README 主体）：录 npz → 开环重放 → 出图。
>   快、可批量、可做数据增广；但精细操作（如放试管进小孔）做不到，
>   因为缺少闭环反馈。
> - **层2.5 旁路采集**（`scripts/live_capture.py`）：编排正常闭环跑，
>   在同一 Isaac 进程里旁路采相机。成功率 = 编排的成功率，能做精细操作。
>   见 `scripts/README_live_capture.md`。

```
┌──────────────────────────────────────────────────────────────┐
│ 层1  动作生成 + 录制                                           │
│   谁做：motion-generation 编排 + OCS2 运控                     │
│   产出：episode .npz（关节角 + 末端位姿 + 夹爪，同一时刻采样）   │
│   工具：lerobot_ros2/.../tools/ee_traj.py                      │
└──────────────────────────────────────────────────────────────┘
                            ↓  episode .npz
┌──────────────────────────────────────────────────────────────┐
│ 层2  物理重演 + 相机渲染（本扩展）                              │
│   输入：episode .npz                                           │
│   做什么：逐帧 set_dof_position_targets → 物理真正步进 → 渲染    │
│   **不启动控制器、不连 ROS2、不跑 OCS2**；物体由接触力自然跟随   │
│   命令：./scripts/sdg.sh render                                │
└──────────────────────────────────────────────────────────────┘
                            ↓  frames/frame_*.npz
┌──────────────────────────────────────────────────────────────┐
│ 层3  质量检查 + 数据集打包                                      │
│   工具：tools/check_quality.py                                 │
└──────────────────────────────────────────────────────────────┘
```

## 为什么用 DOF 回放，而不是 link 位姿

Isaac 自带的 `ArticulationRecordable`（`recorder.py` 在用）记录的是
**47 个 link 的世界位姿**（源码注释写明 "USD-only link-pose recordable"）。
回放时用 `set_world_poses` 把这些位姿写回去，对关节机器人是**物理非法的**：
关节约束被破坏，夹爪（`AssemblerFixedJoint` 焊接）会被扯开。

层2 改用 `set_dof_position_targets`（走 drive）驱动关节，是合法的运动学重放。
**注意用 drive 而不是 `set_dof_positions`（teleport）** —— 后者会让接触约束
每帧失效，物体抓不住，详见下文「三条必须满足的前提」。

**底盘不驱动**：articulation 的 32 个 DOF 里有 4 个驱动轮 + 8 个脚轮关节。
它们不属于操作轨迹，强行驱动会把整个底座拖走。`skipped_dof_names()` 会把
名字含 `wheel`/`caster` 的全部跳过，保持当前位置不动。

## 关节名优先，不用索引

运控发 `/joint_states` 里按名字的 18 个关节；Isaac 的 `ArticulationView`
有 32 个 DOF 且**顺序不同**。只存索引必然错位。

所以 episode 里同时存 `joint_names`，回放时 `build_dof_mapping()` 按名字
建立映射。任何一个关节名对不上都会报 `missing`，而不是静默用错位置。

## 时间对齐：用帧索引，不用时间戳

```
episode 有 N 帧关节角
层2 逐帧推进：第 i 次循环 = 第 i 帧关节角 + 第 i 组图像
→ 天然一一对应，不存在时间戳漂移
```

因此层2 必须**逐帧步进**（每帧至少 `app.update()` 一次），
不能"实时播放然后按墙钟采样"。

**每帧推进多少由时间累积器决定**（`--steps-per-frame` 默认 0 = 自动）：
按 episode 的真实帧间隔累积，使仿真总时长与录制一致。这一点对接触很重要
—— 回放速度偏离录制速度，接触力就不对。

不要用 fixed time stepping 去对齐：实测那样时间轴会走但 PhysX 不再被
step（关节读回恒等于设定值、画面定格）。

## 渲染阶段有物理吗？

**有。层2 现在是「物理重演」，不是「摆姿势拍照」** —— 这是踩了很多坑之后
确定的唯一可行方案。

| | 正常跑仿真 | 层2 渲染 |
|---|---|---|
| timeline | PLAY | **PLAY**（物理必须真正步进） |
| 物理求解 | 有重力/接触 | **有**（物体靠接触力自然跟随） |
| 关节 | 控制器驱动 | `set_dof_position_targets` + drive |
| 物体 | 受物理影响 | **受物理影响，不手动设位姿** |

为什么不能「摆姿势拍照」：

- 用 `set_dof_positions`（teleport）瞬移关节 → 每帧位移 5~20mm，
  远大于接触厚度（2mm），接触约束每帧被打断，**摩擦求解不出来**，
  物体像「完全光滑地滑过」夹爪。
- 用 `set_world_poses` 硬写物体位姿 → 对刚体非法，要么无效（kinematic）
  要么被物理改写（位置飞到天上）。

正确做法是让物理真正运行：关节走 drive，物体由接触力自然跟随。

## 三条必须满足的前提（缺一条就抓不住物）

这是层2 从「看起来对」到「真的抓住」的关键，每一条都实测量化过：

### 1. 底盘必须锁住

层1 编排全程 **BASE_LOCK**（OCS2 主动控住底盘），所以录制时底盘纹丝不动。
但回放的 USD 里 `base_link` 是**自由浮动刚体**，手臂一挥就把整机带着转。

实测：到 t=28s 已偏航 **-12.5°**、位移 11mm，右末端偏 **76mm**，
夹爪从试管旁擦过去。用 `PhysicsFixedJoint` 锚到世界后，漂移归零。

**必须在构造 `Articulation` 之前加**，否则 physics tensor 失效（关节读写全 NaN）。

### 2. 手臂必须走 drive，不能用 teleport

| 驱动方式 | 关节误差 | 接触 | 结果 |
|---|---|---|---|
| `set_dof_positions` | <1mrad（很准） | 每帧被打断 | 物体光滑滑过 ❌ |
| `set_dof_position_targets` + gain | 稍大 | 稳定 | 正常夹起 ✅ |

实测对比（抓取帧 1050 → 抬手 1350）：
```
teleport: grip -0.0146→-0.0200（空合），tube_z 0.7431→0.7430  ❌
drive:    grip -0.0143→-0.0143（一直夹着），tube_z 0.7431→0.9119 ✅
```

手臂 gain 默认 `K=100000 D=10000`。

### 3. 夹爪力气要够

资产默认 `maxForce=20N`，实测**临界点在 20~25N**：

```
20N → grip 停在 -0.0021（顶不住），试管掉地   ❌
25N → grip 稳定 -0.0147，抬起 186mm，p95=13mm ✅
40N → 抬起 187mm，p95=9mm                     ✅
80N → 抬起 184mm，p95=17mm（过大易抖）        ✅
```

取 **25N**（只比 Dex1 额定 effort 20N 高 5N，尽量贴近现实），
stiffness 提到 50000（决定手指跟随手的速度，太软追不上举手动作）。

## 夹爪开合：开关量，不是位置回放

运控用位置指令驱动夹爪。闭合时 target 继续往里压，手指碰到物体后停住，
于是 `F = K*(target-actual)` 产生夹持力 —— 这才是能把物体提起来的原因。
而 `/joint_states` 记录的是被接触限制后的 **actual**。

若回放也用 actual 作为 target，`target==actual` ⇒ 夹持力恒为 0。
所以回放必须还原「夹住」这个语义：

- **张开** → target = 关节上限（+0.0245）
- **闭合** → target = 关节**下限**（-0.0200，与运控的 `closed_position_` 一致），
  手指被物体挡住停在中间（如 -0.0147），drive 持续出力

判定「开/合」的阈值要注意：录制开头夹爪常停在 **0.0**（两指合拢的中间态，
运控还没下发指令），那一段**不是抓取**。若判成闭合并压到限位，会在真正
抓取前就把物理状态搞坏（实测 293~773 帧被误判，导致后面抓取失败）。
判据用「距张开值超过 60% 行程」。

## 夹爪关节必须纳入 joint_names

`ee_traj` 的 `ARM_JOINTS` 末尾包含 `left_gripper_joint` / `right_gripper_joint`。
它们不在运控优化集合里（运控不管夹爪），但层2 回放必须驱动，
否则夹爪永远停在初始开合状态，手里的试管会与夹爪脱开。

`check_quality.py` 会检查这一项，缺失直接判 FAIL。


## 安装

`init.sh` 的 `install_fasim_kit_extension` 会把 `exts/<name>` 链到
`$ISAACSIM_DIR/extsUser/` 并写入常用 kit 的 `[dependencies]`。

不注册也能用 —— `scripts/sdg.sh` 自带 `PYTHONPATH`，可直接跑。

## 用法

### 路线 A：层2 离线回放（录 npz → 重放 → 出图）

```bash
cd /home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg

# 0) 看配置
./scripts/sdg.sh list
./scripts/sdg.sh describe --robot unitree_g1d --task sinopec_test_tube

# 1) 层1：录制（仿真在跑编排时，另开终端旁听）
cd /home/fiveages/lerobot_ros2/examples/IsaacSim/robots/Unitree/G1D/tools
./ee_traj.sh record -o ~/fa_data/pick_01.npz
./ee_traj.sh inspect ~/fa_data/pick_01.npz

# 2) 检查 episode（不需要 Isaac）
cd /home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg
python3 tools/check_quality.py --episode ~/fa_data/pick_01.npz

# 3) 层2：渲染相机数据（无头，不需要 ROS2）
./scripts/sdg.sh render --robot unitree_g1d --task sinopec_test_tube \
    --episode ~/fa_data/pick_01.npz

# 4) 连同渲染产物一起检查
python3 tools/check_quality.py --episode ~/fa_data/pick_01.npz \
    --render-dir ~/fasim_sdg/unitree_g1d/sinopec_test_tube
```

想肉眼看回放效果，加 `--gui`：

```bash
./scripts/sdg.sh render ... --episode xxx.npz --gui
```

### 路线 B：层2.5 旁路采集（编排闭环跑，顺带采相机）

不需要 npz，也不需要回放。编排正常跑，同一 Isaac 进程内旁路采，
**用控制文件触发，不用 Script Editor**。

```bash
# 1) ./init.sh 安装扩展（只需一次，和 ros2.nodes 一样的安装路径）
cd /home/fiveages/FaSim-Isaac && ./init.sh    # 选 isaac_local

# 2) ./run.sh 启动仿真，做好跑编排的准备

# 3) 新终端：开始采集
echo 'start out=~/fasim_live/pick_01 every=20 camera=head' \
  > ~/fasim_live/control.txt

# 4) 终端跑 motion-generation（和平时一样）

# 5) 结束
echo stop > ~/fasim_live/control.txt
```

详细步骤、控制文件命令、产物格式、实现要点见
**`scripts/README_live_capture.md`**。

### 快速验证工具

```bash
# 无头抽样出图（十几秒到几分钟），出 contact_sheet.png + report.json
/home/fiveages/isaacsim/python.sh tools/verify_replay.py \
    --episode ~/fa_data/pick_10.npz --out /tmp/verify --samples 12

# 带视窗交互回放（能直接看动作对不对），自动切第三人称机位
/home/fiveages/isaacsim/python.sh tools/replay_gui.py \
    --episode ~/fa_data/pick_10.npz --start 900 --end 1700 --speed 0.3
```

`replay_gui.py` 实时打印三个关键量，用来判断抓取是否成功：

```
frame=1200 t=24.0s test_tube=-0.5mm | base 漂移=0.0mm 偏航=+0.000° | 夹爪实际 [-0.0147 -0.0148]
                    ↑ 试管偏差        ↑ 底盘必须恒为 0      ↑ 被物体挡住=夹到了
```

可调参数：`--no-lock-base`（对比漂移）、`--arm-mode teleport`（对比接触失效）、
`--grip-maxforce`（夹持力）、`--close-target`（夹爪闭合目标）、`--no-ros`。

## 输出

```
<output.dir>/<robot_key>/<task_key>/
├── frames/frame_000000.npz       每帧 3 个相机的 rgb + depth
└── manifest.json                 帧范围 / 相机 / 关节跟踪误差
```

`output.dir` 在 `configs/_default.yaml`（当前 `/home/fiveages/fasim_sdg`）。
**数据不进仓库**。

## 关节跟踪误差：核心质量指标

层2 每帧回读 `get_dof_positions()` 与目标比对：

```
joint max error : 0.412 mrad     ← 越小越好
joint p95 error : 0.180 mrad
```

判据：
- `< 5 mrad` —— 相机姿态与录制姿态一致，数据可信
- `5-50 mrad` —— 轻微偏差，检查 `--steps-per-frame`
- `> 50 mrad` —— 回放不忠实，**这帧的图像不能用**

这个数字是回答"数据质量如何"最直接的证据。

## 相机（G1D，已实测）

| 名称 | prim |
|---|---|
| `head` | `/World/G1D/G1_Body/torso_link/d435_link/head_camera` |
| `wrist_left` | `.../left_tcp/Dex1/Geometry/dex1_base_link/dex1_camera_link/dex1_camera_optical_frame/dex1_left_camera` |
| `wrist_right` | `.../right_tcp/Dex1/.../dex1_right_camera` |

## episode 格式

见 `fasim/replicator/sdg/episode.py` 的模块 docstring。要点：

| 字段 | 形状 | 说明 |
|---|---|---|
| `t` | (N,) | 相对秒数，**仅日志用，不参与对齐** |
| `joint_pos` | (N, J) | 实测关节角，列序 = `joint_names` |
| `joint_names` | (J,) | **回放的唯一依据** |
| `left_pos` / `left_quat` | (N,3) / (N,4) | 末端实测（world） |
| `left_cmd_pos` / `left_cmd_quat` | | 末端指令（训练 action） |
| `left_grip` | (N,) | 1=开 0=关 -1=未知 |

## 旧内容说明

| 项 | 状态 |
|---|---|
| `scripts/capturer.py` + `sdg.sh capture` | **legacy**：回放 HDF5 的 47 link 位姿，物理非法。仅保留给历史数据，新数据一律走 `render` |
| `scripts/dof_replay.py` | 早期 DOF 实验脚本。功能已被 `renderer.py` 吸收，保留作参考 |
| `scripts/gui_record.py` | GUI 内录制（Script Editor 用），仍然有效 |
| 已删除的 `test_*.py` / `verify_*.py` | 当时的排查脚本，结论已固化进 `renderer.py` 的注释 |

## 边界：接 VR 遥操

层1 只依赖 `/joint_states` 与末端位姿话题，**不 import 任何 OCS2/lerobot 模块**。
VR 遥操、手动示教、脚本回放都能复用同一个 `record`，层2 完全共享。

## 扩展新机器人

1. 建 `configs/<robot_key>/robot.yaml`（articulation / base / EE / 话题）
2. 建 `configs/<robot_key>/tasks/<task_key>.yaml`（场景 / 相机 / tracked prims）
3. `./scripts/sdg.sh describe --robot <robot_key> --task <task_key>` 自检
4. （可选）`python.sh scripts/probe_cameras.py` 换成新机器人验证相机

不改 Python 代码。
