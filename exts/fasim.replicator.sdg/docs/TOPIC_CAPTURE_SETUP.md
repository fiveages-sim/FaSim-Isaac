# G1D 话题采集 + 外部运控（评测标准流程）

> 为什么不用场景内嵌运控（`ROS=fiveages`）：见
> `exts/fasim.replicator.sdg/docs/DEBUG_LOG.md` 问题 9 ——
> **它会静默破坏腕部相机话题**（`/left_hand_camera/rgb` 等 0 条）。

本流程 = `ROS=enable`（场景只发话题 + 相机）+ **外部 ros2-stack** 起运控。
这也更接近真机部署形态，评测更公平。

---

## 前置检查

```bash
# 1) 场景变体必须是 ROS=enable（不是 fiveages）
grep -A8 'prepend payload.*G1D.usda' \
  ~/FaSim-Isaac/environment/fa-project-usd/Sinopec/2026-10\ Test\ Tube\ Picking/env/G1D/room.usda \
  | grep 'string ROS'
# 期望: string ROS = "enable"
```

```bash
# 2) lerobot_ros2 的配置允许外部运控
grep -A2 'motion:' ~/lerobot_ros2/examples/IsaacSim/robots/Unitree/G1D/robot.yaml
# 期望: required: true
```

```bash
# 3) 环境
cd ~/lerobot_ros2
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash

# **关键：运控 workspace 必须在 AMENT_PREFIX_PATH 首位**
# ros2-stack 的 resolve_setup_bash() 优先用 AMENT_PREFIX_PATH 的第一项；
# 若那里是 isaac_jazzy_ws，会找不到 unitree_g1d_description，报：
#   Failed to get package path for 'unitree_g1d_description'
#   OCS2 requires xacro planning URDF for 'unitree_g1d'
source ~/ros2_ws/install/setup.bash

# **不要** source isaac_jazzy_ws（ros2-stack 不需要，且会抢走 AMENT 首位）

which ros2-stack motion-generation    # 都应指向 .venv/bin
./scripts/setup_uv_env.sh --check     # 全绿

# 确认描述包能解析
ros2 pkg prefix unitree_g1d_description
# 期望: /home/fiveages/ros2_ws/install/unitree_g1d_description
```

---

## 场景/资产要求（用 `ROS=enable` 时必须确认）

`ROS=enable` 只提供基础话题 + 相机，**不提供**以下两项，需要另行确认：

### 1. `/get_prim_attribute` service（motion-generation 的 pick 技能依赖）

`pick` 技能用它读物体上的 grasp 标记（如 `test_tube/tube_grasp`）。
缺失时报：

```
[ERROR] Task queue failed: /get_prim_attribute unavailable within 2.0s
        — add OmniGraph node isaacsim.ros2.bridge.ROS2ServicePrim
```

**已修**：在 `G1D/payloads/ROS/enable.usda` 的 `ROS_JointStates` 图里补了
`ros2_service_prim` 节点（与 FiveAges_W2 / Dobot CR5 同一模式）。

验证：

```bash
ros2 service list | grep get_prim_attribute
```

### 2. 场景变体必须是 `ROS=enable`

```bash
grep 'string ROS' \
  ~/FaSim-Isaac/environment/fa-project-usd/Sinopec/2026-10\ Test\ Tube\ Picking/env/G1D/room.usda
# 期望: string ROS = "enable"
```

---

## 启动顺序（4 个终端）

### 终端 1：Isaac（只开场景，不 Play）

```bash
cd ~/FaSim-Isaac && ./run.sh
```

打开 room 场景，切到 `ROS=enable`（如果 room.usda 已改好，打开即是）。

**先不要点 Play。**

### 终端 2：起外部运控（ros2-stack）

```bash
cd ~/lerobot_ros2
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash

cd examples/IsaacSim
ros2-stack launch --robot unitree_g1d
```

它会 launch `ocs2_arm_controller/full_body.launch.py`，等到
`ocs2_wbc_controller` ready。

**验证**：

```bash
# 另开终端
ros2 topic list | grep -E "joint_states|left_current_pose"
ros2 topic echo /ocs2_wbc_controller/current_state --once
```

### 终端 3：Isaac 点 Play，然后查相机

点 Play 后，在终端 3：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /head_camera/rgb        # 期望 ~20-60Hz
ros2 topic hz /left_hand_camera/rgb   # 期望 ~20-60Hz  ← 关键
ros2 topic hz /right_hand_camera/rgb
```

**三个都要有数据**。这是本次改造的核心验证点。

若腕部仍是 0 条 → 回到 DEBUG_LOG 问题 9 继续排查。

### 终端 4：话题采集（可选，评测用）

```bash
cd ~/lerobot_ros2
source .venv/bin/activate && source /opt/ros/jazzy/setup.bash

# 先自检
python examples/IsaacSim/robots/Unitree/G1D/record_topic.py --check

# 评测采集（旁听，不发控制）
python examples/IsaacSim/robots/Unitree/G1D/bench_record.py \
    --out ~/bench/topic_run02 --duration 40
```

---

## 跑编排

```bash
# 终端 2（运控已在这个终端起了）或新终端
cd ~/lerobot_ros2/examples/IsaacSim
motion-generation
```

选 `pick_tube_full` 任务。

**注意**：`motion-generation` 可能会尝试 `--ensure-ros2-stack`；
因为 `robot.yaml` 里 `motion.required: true`，它会复用已在跑的运控，
不会起第二套。

---

## 与 sdg 内部采集的同轨迹对比

```bash
# 终端 5：sdg 采集（同一条轨迹）
cd ~/FaSim-Isaac/exts/fasim.replicator.sdg
./scripts/sdg.sh capture --what camera,joint,ee,object --out ~/fasim_data/bench02
# 跑一遍编排
./scripts/sdg.sh stop
./scripts/sdg.sh bench ~/fasim_data/bench02
```

得到两个 `bench_metrics.json`，字段同名同义，可直接对比。

---

## 常见问题

| 现象 | 检查 |
|---|---|
| `ros2-stack launch` 报找不到包 | 是否 source 了运控 workspace |
| 运控起来但机器人不动 | Isaac 是否 Play；`/ocs2_wbc_controller/current_state` 是否 ready |
| 腕部相机 0 条 | 场景 ROS 变体是否 `enable`；见 DEBUG_LOG 问题 9 |
| `motion-generation` 起了第二套运控 | 检查 `.meta/ros2_stack.yaml` 是否 `motion.required: true` |
| 话题重复率高（83%） | 正常，见 DEBUG_LOG 问题 11 —— 上游发布率限制 |
