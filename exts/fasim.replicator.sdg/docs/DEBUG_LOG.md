# 调试问题记录（sdg 平台搭建 → 评测完成）

> 从 2026-11-05 开始，到评测文档出炉为止。**所有遇到的问题都记在这里**，
> 包括已解决、已规避、以及判定为"不在范围内"的。
>
> 目的：后续接手的人不用重新踩一遍；评测结论里的每条判断都能追溯到现场证据。

---

## 索引

| # | 日期 | 问题 | 状态 |
|---|---|---|---|
| 1 | 11-06 | 末端位姿一直读到静态值（0.0mm 偏离） | ✅ 已解决 |
| 2 | 11-06 | 回放抓取位置稳定偏下 2-3cm | ✅ 已解决 |
| 3 | 11-07 | 相机内参算错（fy 偏 34%） | ✅ 已解决 |
| 4 | 11-10 | pack 采到相机时崩溃（变量遮蔽） | ✅ 已解决 |
| 5 | 11-11 | 回放 714 帧「秒结束」且机器人不动 | ✅ 已解决 |
| 6 | 11-11 | EE 回放夹爪全程不动 | ✅ 已解决 |
| 7 | 11-11 | 「选全部相机」被当成「不采相机」 | ✅ 已解决 |
| 8 | 11-12 | Isaac 崩溃后残留 socket / 空会话 | ✅ 已缓解 |
| 9 | 11-12 | **`ROS=fiveages` 静默破坏腕部相机话题** | ⚠️ **已规避**（改外部运控） |
| 10 | 11-12 | `ROS=enable` 下出现大量非预期运控话题 | 📌 记录，不在本项目范围 |
| 11 | 11-12 | 话题采集 83.5% 帧是重复数据 | 📌 已量化，见评测 |
| 12 | 11-12 | 诊断脚本用 `sys.exit` 关掉了 Isaac | ✅ 已修复 |
| 13 | 11-12 | conda 停用 → uv 迁移，numpy 版本冲突 | ✅ 已解决 |
| 14 | 11-12 | `ros2-stack` 找不到 `unitree_g1d_description` | ✅ 已解决 |
| 15 | 11-12 | `/get_prim_attribute` service 缺失（enable 变体） | ✅ 已解决 |

---

## 1. 末端位姿一直读到静态值

**现象**：`left_pos` / `right_pos` 在 765 帧里完全不变（偏离 0.0mm），
而且值恰好等于 q≈0 静止姿态的 tcp 位姿。

**根因**：`tcp` 是纯 Xform（fixed joint 的子 frame）。newton kit 里关了
PhysX→USD/Fabric 同步：

```86:94:/home/fiveages/isaacsim/apps/isaacsim.exp.full.newton.kit
updateToUsd = false
fabricUpdateTransformations = false
```

所以 `XformPrim.get_world_poses()` 无论 `usd`/`usdrt`/`fabric` 后端，读到的
都是 authoring stage 的静态值。

**修复**：改读**父刚体**（`wrist_yaw_link` 有 `PhysicsRigidBodyAPI`，
`RigidPrim` 走 tensor 后端直读 PhysX，不经同步管道），再叠 URDF 里的固定
偏移 `[0.0415, 0, 0]`。

**验证**：刚体+偏移 vs Xform 静态值差 0.3mm；vs FK 差 2.1mm。
配置在 `robot.yaml` 的 `ee_links`。

---

## 2. 回放抓取位置稳定偏下 2-3cm

**现象**：编排能抓准，回放总是偏下 2-3cm，且每次偏的量一致。

**根因**：采集只记了**实测**关节角 `q_act`，回放又把它当 drive 目标驱一次，
物理稳态误差 `e` 被叠加两次：

```
编排: q_cmd ──drive──> q_act = q_cmd − e
回放: q_act ──drive──> q_act − e            ← e 第二次
```

实测拟合出 `e ≈ τ·q̇`，`τ ≈ 107.9ms`（与 USD 的 `D/K = 6000/60000 = 100ms`
吻合）。手臂前伸时 `J·e` 主要指向下方。

**修复**：
- 采集增记 `joint_pos_cmd`（`get_dof_position_targets`，与实测**无时间滞后**，
  实测互相关 lag=0）
- 回放默认 `--joint-src cmd`，用指令驱动
- EE 回放同理：目标优先用 `<side>_cmd_pos`（由 `joint_pos_cmd` 经纯 numpy FK 反算）

**验证**：回放 vs 编排末端差 mean 1.0/1.2mm（原来偏 2-3cm）。

---

## 3. 相机内参算错（fy 偏 34%）

**现象**：按 `cameraFocalLength / cameraAperture × 分辨率` 算内参，得到
`fx=465.5 / fy=625.1`，fy 明显不对。

**根因**：本机 `cameraAperture = [24.95, 13.936]` 是 **16:9** 视口，
而渲染分辨率 640×480 是 **4:3**，直接按 aperture 比例算会被拉伸。

**修复**：用投影矩阵反推：

```
fx = 0.5 × |cameraProjection[0]| × W
fy = 0.5 × |cameraProjection[5]| × H
```

实测两者都等于 **465.51**，正确。

**另一个坑**：`cameraViewTransform` 是**世界→相机**（view matrix），
不是相机→世界，相机位姿要取逆。

**验证工具**：`scripts/probe_camera_params.py`（升级 Isaac 后用它复核）。

---

## 4. pack 采到相机时崩溃

**现象**：`TypeError: ufunc 'divide' not supported`，只在采到相机时出现。

**根因**：变量遮蔽。诊断代码里 `d = np.linalg.norm(...)` 覆盖了会话目录变量
`d`，后面 `d / 'frames'` 变成 numpy 数组除以字符串。

**修复**：目录变量改名 `session_dir_p`，诊断变量改名 `xdev` / `cmd_err`。

---

## 5. 回放 714 帧「秒结束」且机器人不动

**现象**：CLI 报 `每次 update = 0.0 ms`，714 帧瞬间发完，机器人纹丝不动。

**根因链**：
1. `_ensure_app_dt()` 靠 timeline 时间差分，暂停时返回 0
2. 但真正原因是 **FSM 不在 OCS2 状态**：只有 `FSMStateName::OCS2` 才接受
   EE 目标（`Ocs2ArmController::syncPoseTargetAcceptance`）

**另一个坑**：FSM 的**话题值**与 C++ 枚举顺序不同！

```
C++ 枚举:   INVALID=0, HOME=1, HOLD=2, MOVEJ=3, OCS2=4
话题值:     HOME=1, HOLD=2, OCS2=3, MOVEJ=4      ← OCS2 是 3
```

而且 `MOVEJ` **只能回 HOLD**（`StateMoveJ::checkChange` 只认 case 2），
不能直接跳 OCS2。正确路径：`MOVEJ →(2) HOLD →(3) OCS2`。

**修复**：EE 回放启动时自动查 `/fsm_state`，不在 OCS2 就走 `2 → 3`；
同时发 `BASE_LOCK`（对齐编排第 0 步）。

---

## 6. EE 回放夹爪全程不动

**根因**：`grip_specs` 只在 `joints` 分支构建。`ee` 分支下夹爪归运控管，
必须发命令话题，而不是 drive 关节。

**修复**：EE 模式发 `/<side>_gripper_controller/target_percent`（`Float64`），
从 `joint_pos_cmd` 提取夹爪时序（按原始值线性映射，不二值化——Dex1 的
`0` 是闭合点、正负两侧都张开）。

---

## 7. 「选全部相机」被当成「不采相机」

**根因**：空字符串在两层语义相反：

```python
# cli.py
camera = "" if sel == "all" else sel     # 选"全部" → 传空串
# live_capture.py
if str(camera or "").strip():            # 空串 → 整个相机分支跳过
```

**修复**：统一语义 —— `None`/`""` = 不采，`"all"` = 全部；CLI 选全部时传 `"all"`。

---

## 8. Isaac 崩溃后残留 socket / 空会话

**现象**：Isaac 崩了，`capture.sock` 还在，CLI 只报 `Errno 111`；
还留下 0 帧的空会话目录。

**修复**：
- `session.json` **边采边写**（挂载时写 `status=capturing`，每 20 帧刷新进度），
  崩溃后 pack 仍能拿到 `physics_dt` 等元信息
- CLI 探测 Isaac 进程，区分「崩了」与「扩展没加载」
- `doctor` 检查残留 socket / `status=capturing` 会话 / 0 帧空目录

---

## 9. `ROS=fiveages` 静默破坏腕部相机话题 ⚠️ 重要

**现象**（2026-11-12 实测，空状态重启 Isaac）：

| ROS 变体 | /head_camera/rgb | /left_hand_camera/rgb | /right_hand_camera/rgb |
|---|---|---|---|
| `enable` | ~56 Hz | **~56 Hz** | **~56 Hz** |
| `fiveages` | 53.6 Hz | **0 条** | **0 条** |

**关键：不报任何错误，静默失效。**

**排查过程（逐层排除）**：

1. 场景变体正确：`Camera=enable`、`EE_*=dex1`、`Sensor=sensors`
2. Dex1 内部变体正确：`ROS=enable`、`Side=left|right`、`Sensor=camera`
3. `Graph/ROS_Dex1Camera` 存在，7 个节点，与头相机结构**完全一致**
4. exec 连线全部解析（`RunOnce → RenderProduct → RGBPublish`）
5. `cameraPrim` 目标 prim 存在，参数正常（`focalLength=18.15`）
6. 图实例存在（`og.get_graph_by_path()` 有效）

→ 静态配置全对，但 RenderProduct 从未创建。

**根因判断**：`fiveages.usda` 比 `enable.usda` 多了两个图：

| 图 | enable | fiveages |
|---|---|---|
| `ROS_Odometry` | ✅ | ✅ |
| `ROS_Differential_Controller` | ✅ | ✅ |
| `ROS_JointStates` | ✅ | ❌ 被删 |
| `ROS_Control`（含 `ROS2ControlLaunch`） | ❌ | ✅ 新增 |
| `ROS_Clock` | ❌ | ✅ 新增 |

`ROS_Control` 里的 `ROS2ControlLaunch`（自定义节点 `fasim.ros2.nodes.ROS2ControlLaunch`）
在 Play 时进程内启动 full_body 运控，**争用了 ROS2 context / 执行资源**，
使后续加载的相机图拿不到有效 context → `IsaacCreateRenderProduct` 不执行。

**规避方案（已采用）**：改用 `ROS=enable` + **外部 ros2-stack** 起运控。

**影响的文件**：
- `environment/.../env/G1D/room.usda`：`ROS` 变体 `fiveages` → `enable`
- `lerobot_ros2/.../G1D/robot.yaml`：`motion.required` → `true`
- `lerobot_ros2/.../test_tube_picking/.meta/ros2_stack.yaml`：允许外部运控

**这个 bug 的评测意义**：话题采集方案**对资产变体配置的正确性要求高，且失败是
静默的**（话题 0 条，无报错）。内部直采不依赖 ROS 发布，无此问题。

---

## 10. `ROS=enable` 下出现大量非预期运控话题 📌 不在本项目范围

**现象**：`ROS=enable` 空状态重启后，`ros2 topic list` 出现：

```
/arms_target_manager/*  /body_current_pose  /fsm_command
/left_current_pose      /left_current_target  /mode_command
/ocs2_wbc_controller/*  /teleop/*  /tf  /odom  ...
```

**预期**：`enable` 只发 `isaac` 前缀的基础话题（`/isaac/joint_states` 等）+
相机话题，用于对接外部运控。

**判断**：这些话题来自外部运控进程（`ros2-stack` 起的 controller_manager /
arms_target_manager / teleop），不是场景图发的。**不属于本项目（数采平台）
的修改范围**，记录备查。

---

## 11. 话题采集 83.5% 帧是重复数据 📌 评测发现

**现象**（`bench_record.py`，采样定时器 20Hz、时长 50s，得 1000 帧）：

```
cam:wrist_left   采样 1000  独立 165  重复 83.5%
joint            采样 1000  独立 165  重复 83.5%
cam:head         采样 1000  独立 165  重复 83.5%
cam:wrist_right  采样 1000  独立 165  重复 83.5%
```

四类字段独立值都是 165（= 3.3Hz），说明被同一个上游瓶颈限制。

**注意**：此前我用「到达时间」算对齐跨度得到 450ms，是**错的**——
字段未更新时 `arrival` 仍是旧值，差值是"多久没更新"而非"对齐误差"。
改用 `header.stamp` 重算，真实对齐跨度是 **median 200ms / max 280ms**。

**结论**：话题方案的「20fps」是定时器频率，**有效数据率只有 3.3Hz**。
已修正 `bench_record.py` 的 `align_spread_ms`（改用 stamp）并新增
`distinct_data` 统计（重复帧占比 + 有效 Hz）。

---

## 12. 诊断脚本用 `sys.exit` 关掉了 Isaac

**现象**：把 `diag_wrist_camera.py` 粘进 Script Editor 执行，`SystemExit: 0`
冒泡到 Kit 主循环 → 触发 `kit_exit` → 仿真被关。

**修复**：所有诊断脚本改为捕获异常、**不调 `sys.exit` / `os._exit`**。

**教训**：给 Isaac 用的脚本一律不能用 `sys.exit`。

---

## 13. conda 停用 → uv 迁移

**背景**：公司停用 conda，统一 uv。

**冲突**：lerobot 0.5.x（PyPI）强制 `numpy>=2`，而系统 `cv_bridge` 是按
numpy 1.x 编译的，升到 2 后 ROS 图像转换会踩 ABI 问题 → **无法采头相机**。

**方案**：环境按用途分离（都走 uv）

| 环境 | numpy | lerobot | 用途 |
|---|---|---|---|
| `.venv` | 1.26.4 | 0.3.4（submodule editable） | 编排 / 采集 / 导出 / 数据处理 |
| `.venv-train` | 2.x | PyPI 0.5.x | 只跑官方训练脚本 |

**另一修正**：venv **去掉 `--system-site-packages`**。实测 `rclpy` /
`cv_bridge` / `cv2` 全部来自 `source /opt/ros/jazzy` 的 `PYTHONPATH`；
加了 `--system-site-packages` 反而混进系统 opencv 4.6，与 pip 的 4.13 打架。

**依赖清单**：`docs/requirements-uv-frozen.txt`（95 个纯 PyPI 包，剔除了
183 个 ROS 系统包 + 10 个 isaac_jazzy_ws 本地包 + opencv/cmake/evdev）。

---

## 14. `ros2-stack` 找不到 `unitree_g1d_description`

**现象**：`ros2-stack launch --robot unitree_g1d` 不报错但也不 ready，
`motion.log` 里反复出现：

```
[ERROR] Failed to get package path for 'unitree_g1d_description':
        package not found, searching: ['.../isaac_jazzy_ws/install/...', '/opt/ros/jazzy']
[ERROR] OCS2 requires xacro planning URDF for 'unitree_g1d' (scope=full)
```

看起来"卡住"，其实是**启动失败后在重试**。

**根因**：`resolve_setup_bash()` 优先用当前 shell 的 `AMENT_PREFIX_PATH`
第一项：:

```39:48:submodules/robot_action_composer/.../ros2_stack/launcher.py
ament = os.environ.get("AMENT_PREFIX_PATH", "")
if ament:
    first = ament.split(os.pathsep)[0]      # ← 优先当前 shell
```

当时首位是 `isaac_jazzy_ws`（不含机器人描述包），而
`unitree_g1d_description` 在 `~/ros2_ws`。

**另外** `.fa-env.local.toml` 里 `[ros2] workspace` 原本写的是
`~/fa_w2_ws`（W2 的空间），也不对。

**修复**：
- `.fa-env.local.toml` → `workspace = "~/ros2_ws"`
- 启动前 **必须** `source ~/ros2_ws/install/setup.bash`，
  且**不要** source `isaac_jazzy_ws`（会抢走 AMENT 首位）

**验证**：`ros2 pkg prefix unitree_g1d_description` 能解析到。

---

## 15. `/get_prim_attribute` service 缺失（`ROS=enable` 变体）

**现象**：`motion-generation` 跑到 pick 技能时报：

```
[ERROR] Task queue failed: /get_prim_attribute unavailable within 2.0s
        — add OmniGraph node isaacsim.ros2.bridge.ROS2ServicePrim
        to the robot ROS payload (not provided by isaacsim.ros2.sim_control)
```

**根因**：这个 service 由 `ROS2ServicePrim` 图节点提供。G1D 原来只在
`ROS=fiveages` 的 `ROS_Control` 图里定义它；切到 `ROS=enable` 后该图不存在
→ service 消失 → pick 技能无法读物体上的 grasp 标记
（`test_tube/tube_grasp`）。

**修复**：在 `G1D/payloads/ROS/enable.usda` 的 `ROS_JointStates` 图里补
`ros2_service_prim` 节点。选这个图是因为它**在 enable 和 fiveages 两种
变体下都存在**，与 FiveAges_W2 / Dobot CR5 的做法一致。

**验证**：`ros2 service list | grep get_prim_attribute` 有输出。

**教训**：切 ROS 变体会连带丢失图节点提供的功能，不只是话题。
`enable` 与 `fiveages` 的能力集不同，迁移时要逐项核对。

---

## 附：sdg 侧的诊断工具

| 脚本 | 用途 |
|---|---|
| `scripts/sdg.sh diag <ep>` | 7 项体检（末端真伪/关节口径/时间轴/物体/相机） |
| `scripts/sdg.sh shots <session>` | 相机联络图，人眼查 RGB |
| `scripts/sdg.sh bench <session>` | 评测指标（与话题方案同口径） |
| `scripts/sdg.sh doctor` | 环境自检 + 崩溃残留检查 |
| `scripts/probe_camera_params.py` | 相机参数字段探针（升级 Isaac 后复核） |
| `scripts/diag_wrist_camera*.py` | 腕部相机排查（问题 9 用） |
| `lerobot_ros2/.../probe_topic_rate.py` | 话题真实频率 / stamp / 批量投递检测 |
| `lerobot_ros2/.../bench_record.py` | 话题侧评测采集 |
