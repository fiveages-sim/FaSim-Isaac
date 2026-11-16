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
| 16 | 11-14 | 闭环推理末端不动（`n_action_steps=1` 撞稳态残差） | ✅ 已解决（改 25） |
| 17 | 11-14 | 推理启动后底盘漂移 / TF 刷屏 / 右手反复伸缩 | 🔍 底盘锁已修；伸缩见 #18 |
| 18 | 11-14 | **action 里没有夹爪通道** → 抓不住、反复抽回 | ✅ 已修（`ee_absolute_gripper` 16 维） |
| 19 | 11-16 | **`n_action_steps` < 合爪帧号** → 合爪动作不在队列里 | ✅ 已修（默认 100，任务已跑通） |

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

## 16. 闭环推理末端不动（800 帧只走 8.5mm）—— `n_action_steps=1` 撞上运控稳态残差

**现象**：L3 闭环推理 800 帧（20Hz / 40 秒），任务需要 25cm 位移，实测：

```
moved_tgt_m = 0.00848   ← 策略目标累计只移动 8.5mm
moved_cur_m = 0.00552   ← 末端下垂 5.5mm
cos = -0.96             ← 方向相反
controller_current_target moved_m = 0.00286  ← 运控确实接受了指令
```

用户观察："末端在发策略的时候不是静止的了，但也只有轻微颤抖和摆动，mm 级别"。

### 诊断过程（全部实验驱动，不靠猜）

**第 1 步：运控侧有没有问题？**

`scripts/probe_ee_tracking.py` 绕开策略，直接发阶跃目标。先读运控的三道闸门
（源码见 `PoseBasedReferenceManager`）：

| 闸门 | 位置 | 含义 |
|---|---|---|
| `accepting_targets_` | `Ocs2ArmController.cpp:560` | **只有 FSM==OCS2 才接受末端绝对目标** |
| `block_<side>_external_ee_` | `WbcCtrlComponent.h:224` | 该臂 enable hold 未回 IDLE 时**丢弃该臂**所有 EE 指令 |
| FSM 状态码 | `Ocs2WbcController.cpp:690` | `1=HOME 2=HOLD 3=OCS2 4=MOVEJ` |

初次 `--check-only` 就读出根因之一：**FSM 在 HOLD（2），闸门1 关闭**。

> 坑：`/fsm_state` 和 `/ocs2_wbc_controller/current_state` 都是
> `rclcpp::QoS(1).transient_local()` 发布。用默认 volatile QoS 订阅**收不到任何消息**，
> 会误报"运控没在跑"。必须用 `DurabilityPolicy.TRANSIENT_LOCAL`。

切到 OCS2 + BASE_LOCK 后，40mm 阶跃（双手 × x/z 全测）：

```
✅ left/x   +40mm → 末端 +37.51mm (跟随 94%)   内部目标 39.07mm 接受=True
✅ left/z   +40mm → 末端 +31.87mm (跟随 80%)
✅ right/x  +40mm → 末端 +37.24mm (跟随 93%)
✅ right/z  +40mm → 末端 +30.99mm (跟随 78%)
```

**决定性实验**：`--replay-dataset` 按 20Hz 回放真实采集的 `ee_absolute` 轨迹：

```
[left]  300 帧：目标位移 153.26mm  实际 154.27mm  跟随 1.007×  cos=0.998
[right] 300 帧：目标位移 208.38mm  实际 236.33mm  跟随 1.134×  cos=0.995
```

**⇒ 运控完全正常，末端精确跟随真实轨迹。问题不在运控。**

**第 2 步：策略输出对不对？**

`online_infer/openloop_check.py` 离线开环（喂数据集真实观测）：

```
MAE 0.002988   量级 真值 0.653482 / 预测 0.653969
逐维相关性 mean=0.998 中位=0.999，14/14 维 > 0.8
相邻帧变化 真值 0.006549 / 预测 0.006426 (0.98x)
```

**⇒ 策略逐帧输出也完全正确，权重没问题。**

**第 3 步：那到底哪里错了？—— 稳态残差 × `n_action_steps=1`**

`--sweep` 扫描"每帧增量步长 vs 实际位移"（40 帧/档，20Hz）：

| 步长 | left 每帧均值 | right 每帧均值 | 趋势 |
|---|---|---|---|
| 0.5mm | **-0.381mm** | -0.305mm | 倒退 |
| 1.0mm | -0.244mm | -0.310mm | 倒退 |
| 2.0mm | -0.079mm | -0.313mm | 倒退 |
| 3.0mm | +0.080mm | -0.075mm | 分界 |
| 5.0mm | +0.352mm | +0.336mm | 前进 |
| 10.0mm | +0.991mm | +0.989mm | 前进 |

**分界精确落在 2.0~3.0mm，与实测稳态残差 2.5mm 吻合。**

零阶跃实验（把当前位姿原样发回）测出稳态残差底：**2.5~3.0mm**。

机制（这是核心）：

```
运控稳态残差 r ≈ 2.5mm     即 current_pose 稳定落后指令目标 2.5mm
策略每帧增量      d ≈ 1.66mm  （训练数据每帧中位移动，实测）

n_action_steps=1 的执行方式：
    每帧:  target_new = current_pose + d        # 以"已经落后 2.5mm 的实测位姿"为基准
         = (target_old - 2.5) + 1.66
         = target_old - 0.84mm                  # ← 目标每帧**倒退**

⇒ 800 帧后目标只净动 8.5mm，且方向随机（被下垂噪声主导）→ cos=-0.96
```

**第 4 步：模型能走完任务吗？**

`scripts/check_chunk_reach.py` 逐 chunk 预测（把上一 chunk 末端写回 state）：

```
chunk 0: 净跨度 101.92mm  每帧中位 1.727mm  首帧 0.806mm
chunk 1: 净跨度 130.03mm
chunk 2: 净跨度  51.23mm
chunk 3: 净跨度 115.01mm
累积 398.2mm（真值同长度 599.4mm）
```

**⇒ 单个 chunk 就能走 100mm+，远超 25cm 任务所需。模型质量没问题，不用重训。**

### 根因结论

**`chunk_size=100` 配 `n_action_steps=1`**：每次预测未来 100 帧却只执行第 0 帧，
然后重新回读观测。而观测（`current_pose`）恒落后目标 2.5mm，导致每帧净增量
`1.66 - 2.5 = -0.84mm` 为负 —— **目标自我倒退，形成死锁**。

ACT 的设计意图是"一次预测 100 步，连续执行其中若干步"，`n_action_steps=1`
把这个优势完全废掉了，还额外引入了残差反馈的负项。

### 修法

**a) 提高 `n_action_steps`（推荐，无需重训）**

推理时设 `n_action_steps=20~50`：一次预测、连续执行 N 帧不回读观测。
`select_action` 内部有 action queue，队列耗尽才重新预测。
这样每帧执行的是 chunk 内的**绝对序列**，不受稳态残差反馈影响。

```python
# 顺序很重要：先改 config，再 reset（reset 里 deque(maxlen=config.n_action_steps)）
policy.config.n_action_steps = 25
if policy.config.temporal_ensemble_coeff is not None:
    policy.config.temporal_ensemble_coeff = None   # 与 n_action_steps>1 互斥
policy.reset()
```

**已落到代码**（默认 25，可用 `--n-action-steps 0` 恢复原值）：
- `online_infer/core.py` `init_inference()` 里做覆盖 + 顺序保证
- `inference.py`、`diag_inference.py` 暴露 `--n-action-steps`

**验证（`scripts/verify_action_queue.py`）**：

```
checkpoint n_action_steps = 1  →  覆盖后 25
reset 后队列 maxlen = 25  ✅
模拟 100 帧闭环：前向次数 4（期望 100/25 = 4.0）
左手净位移 77.42mm
```

**闭环实测对比**（`diag_inference.py --frames 300 --hz 20`，同一模型同一场景）：

| 指标 | `n_action_steps=1` | `n_action_steps=25` |
|---|---|---|
| `tracking_direction` | ❌ FAIL，cos=**-0.96**（方向相反） | ✅ OK，cos=**+0.55** |
| `moved_cur_m` | 0.00552（5.5mm） | **0.1775（177.5mm）** |
| 现象 | 末端只下垂，原地抖动 | 朝目标方向移动 |

**末端位移提升 32 倍**，方向由"相反"变为"一致"。

**b) 或做 action chunk 队列 + 时间集成**

`temporal_ensemble_coeff` 非 None 时自动启用，但要求 `n_action_steps == 1`。
时间集成会把多个 chunk 的预测加权平均，天然抑制抖动，但同样要注意：
若每帧都重新预测、且观测恒有残差，集成也只是把"倒退"平均化，不一定能解决。
**优先用 a)。**

**c) 如果长期要在 20Hz 单帧闭环下跑**

那必须消除稳态残差。可查：
- `movel_max_linear_velocity=0.3 / accel=1.0 / jerk=2.0`（MPC 约束）
- 逆重力能力（z 轴跟随率 78% vs x 轴 93%，说明重力补偿有欠缺）
- 但这是治标，`n_action_steps=1` 本身就该改。

### 教训

1. **先分层验证再归因**：运控（阶跃+回放）、策略（开环）、执行方式（chunk）分开测，
   一次只否定一层。这次是"运控正常 + 策略正常 + 执行方式错"，只看任何一层都会误判。
2. **稳态残差会变成反馈项**：绝对位姿指令以"实测位姿"为基准增量叠加时，
   任何恒定残差都会被逐帧累积成反向漂移。这是位置控制 + 增量策略的通病。
3. **用数据当对照**：训练数据每帧 1.66mm、稳态残差 2.5mm —— 一比较就看出
   1.66 < 2.5 必然倒退，不需要任何猜测。
4. **transient_local 话题**：`/fsm_state`、`current_state` 用默认 QoS 订不到，
   会误判"运控没跑"。查状态类话题一律加 `DurabilityPolicy.TRANSIENT_LOCAL`。

### 相关脚本

| 脚本 | 用途 |
|---|---|
| `scripts/probe_ee_tracking.py --check-only` | 只读三道闸门状态（FSM/使能/底盘/耦合） |
| `scripts/probe_ee_tracking.py --switch-ocs2 --enable` | 开闸门后测阶跃（双手 × 多轴） |
| `scripts/probe_ee_tracking.py --sweep` | 扫描步长 vs 位移，找倒退分界（定位稳态残差） |
| `scripts/probe_ee_tracking.py --replay-dataset <path>` | 回放真实轨迹，验证运控跟随能力 |
| `scripts/check_chunk_reach.py` | 检查 chunk 内累积位移是否够走完任务 |

---

## 17. 推理启动后底盘漂移 / TF 刷屏 / 右手反复伸缩

一次闭环实测暴露的三个问题（2026-11-14）。前两个是功能性的，第三个是前者的连带后果。

### 17.1 TF_OLD_DATA 刷屏 —— 推理脚本自己 reset 仿真触发的

**现象**：推理启动前刷几百行

```
[WARN] TF_OLD_DATA ignoring data from the past for frame base_footprint at time 1.310000
Message clock jumped backwards (16.625s -> 1.215s); simulator restarted?
```

**根因（两段叠加）**：

1. `ros2_robot_interface` 新版（`766b19a`，旧指针 `3c2a731` 没有）加了
   clock-regression 检测：`CLOCK_REGRESSION_THRESHOLD_SEC = 1.0`
   （`ros_interface.py:87`），在 `_joint_state_callback` 里调用
   `_watch_clock_regression`（`ros_interface.py:1499/1532`），
   发现时间戳倒退 >1s 就重建 TF buffer，重建窗口期 tf2 对每条旧 TF 打 `TF_OLD_DATA`。

2. **推理脚本自己在 reset 仿真**：`core.py` 原流程是
   `robot.connect()` → `reset_simulation_state()`，即先连上 ROS 收到上一轮的
   TF（stamp 16.6s），再把仿真 reset 回 0 → 时间戳倒退 → 触发检测 → 刷屏。

所以"重启仿真也没用"：判据比较的是**接口进程内**记住的上一个时间戳，
只要脚本 reset 一次仿真就会触发一次。

**修复**：推理脚本**不再碰仿真**。分工是「人负责开关/复位仿真，
脚本只负责让机器人动」。`core.py` 的 `reset_simulation_state()` 调用
改为只读等待 `_wait_for_scene_ready()`（轮询末端位姿直到有数据，15s 超时）。
`--object-prim` / `--no-reset` 保留兼容但已无作用。

### 17.2 底盘没锁住 —— TF 探针的 2 秒窗口会把 BASE_LOCK 冲掉

**现象**：`BASE_LOCK` 发了、确认了、终检也通过了，**但跑一会儿底盘开始动**。

**根因（真正的那一个）**：运控控制器激活时启动了一个 TF 探针，
2 秒后无论成功失败都会调 `resetHumanoidModeToFsmDefault()`：

```258:300:ros2_ws/src/arms_ros2_control/controller/ocs2_wbc_controller/src/WbcCtrlComponent.cpp
void WbcCtrlComponent::finalizeMobileBaseTfProbe(bool available) {
    ...
    // Install the startup default once (base lock included). Later rounds reset on OCS2 exit.
    resetHumanoidModeToFsmDefault();
```

TF **可用**时（Isaac 会发 `world->base`），这里重置到的默认模式是
`HM_DUAL_RELATIVE_MOBILE_BASE`，其 `baseLocked = **false**`。
于是我们刚发完的 `BASE_LOCK` 在 2 秒后被打回原形。

时间线：

```
t=0.0s  控制器激活 → beginMobileBaseTfProbe()
t=0.3s  我们发 BASE_LOCK → 确认通过 ✅ 终检通过 ✅
t≈2.0s  TF 探针超时/成功 → finalizeMobileBaseTfProbe()
        → resetHumanoidModeToFsmDefault() → baseLocked=false  ❌
t>2.0s  底盘开始动
```

这解释了「终检明明通过、但还是没锁住」的矛盾。

**修复**（`examples/IsaacSim/online_infer/core.py`）：

顺序对齐编排 `pick_tube_full.yaml` 与 `tools/ee_traj.py`（都是验证过的）：

```
ARMS_INDEPENDENT → 切 OCS2 → 等 2.6s（越过 TF 探针窗口）→ BASE_LOCK → ARM_ENABLE
```

- `ARMS_INDEPENDENT` 必须最先：耦合状态下 `/right_target` 会被左臂 leader 覆盖
- `sleep(2.6)` = TF 探针超时 2.0s + 余量；探针只有一次（`mobile_base_tf_decided_`），
  之后不会再重置
- 运行期再每 0.5s 检查一次 `base_state`，发现丢了就补发（限频 1s，不刷爆队列）

**踩过的错误顺序**（都试过，都不够）：
| 顺序 | 结果 |
|---|---|
| `BASE_LOCK → ARM_ENABLE`（不切 OCS2） | 闸门关闭，不生效 |
| `切OCS2 → BASE_LOCK → ARM_ENABLE` | 终检通过，2s 后被 TF 探针冲掉 |
| `ARMS_INDEPENDENT → 切OCS2 → 等2.6s → BASE_LOCK → ARM_ENABLE` | ✅ 正确 |

**另一个坑**：`LEFT_ARM_ENABLE` 和 `BASE_LOCK` **一起发会超时失败**（历史记录实测）：

```
Timeout waiting for modes ['LEFT_ARM_ENABLE', 'BASE_LOCK']
  expect {'left_arm_state': 1, 'base_state': 0}, got {'left_arm_state': 0, 'base_state': 0}
```

所以必须分开发、各自确认。

**不要用 `PhysicsFixedJoint`**：历史记录试过，把 `base_link` 钉到世界后
articulation root 变了，`set_dof_positions` 写到不渲染的物理实体上，画面不动。

**辅助措施：关掉 `send_target` 的自动 FSM 切换**

`ArmHandler.send_target()` 有（`handler/arm_handler.py:188`）：

```python
if self.config.auto_switch_fsm_before_control and self.fsm_command_callback is not None:
    if query_fsm_command() != FSM_OCS2:
        fsm_command_callback(FSM_OCS2)
```

`auto_switch_fsm_before_control` 默认 True（`config.py:198`），callback 绑的是
`send_fsm_command`，而它**强制先发 HOLD**（`ros_interface.py:2626`）。离开 OCS2 会触发
`resetHumanoidModeToFsmDefault()`（`Ocs2WbcController.cpp:100`）→ 锁被清。

这不是本次的主因（主因是 TF 探针），但会形成"跑着跑着锁没了"的隐患，所以也关掉。

**注意**：`ROS2RobotInterface.__init__`（`ros_interface.py:248`）把 config 的
`auto_switch_fsm_before_control` 复制成私有成员 `_auto_switch_fsm_before_control`，
而 `ArmHandler` 读的是私有成员 —— **只改 config 无效**，必须在 `connect()` 之后同步。

### 17.3 右手伸出又抽回、反复调整

**现象**：右手朝试管方向大幅伸出，然后抽回，来回尝试。

**关于底盘与目标的澄清**：末端目标本身是 world 绝对系，写在
`/left_target` / `/right_target` 里，**底盘漂移不会改变目标数值**。
所以 17.2 不是 17.3 的直接原因（早前判断有误，已纠正）。
锁底盘的目的只是「与编排工况一致」，不是修抖动。

**真正可疑的是 chunk 边界**。实测（`/tmp/sdg_chunk_check/chunk_reach.json`）：

```
chunk 0  首帧步长 0.806mm    净跨度 101.92mm
chunk 1  首帧步长 2.591mm    净跨度 130.03mm   ← 边界跳 3 倍
chunk 2  首帧步长 8.111mm    净跨度  51.23mm   ← 边界跳 10 倍
```

`n_action_steps=25` 意味着每 25 帧重新预测一次。新 chunk 的起点依赖**当前观测**，
而运控有 ~2.5mm 稳态残差（`current_pose` 恒落后 target），
所以新 chunk 的第 0 帧位置和旧 chunk 执行到的位置会差一截，
表现为「走出去 → 顿一下 → 换方向」。

**待验证方向**（尚未定论）：
- 降 `n_action_steps` 到 10，看抖动是否减轻
- 或做 action chunk 队列 + 时间集成（`temporal_ensemble_coeff`）
- 或记录每帧 `target` / `current_pose` / 底盘位姿做因果分析

---

## 18. action 里没有夹爪通道 —— 抓不住，表现为「伸出去又抽回」

**现象**：闭环推理时右手朝试管大幅伸出，然后抽回，来回反复，像"不断调整角度抓不到"。

**根因**：`ee_absolute` 的 action 只有 14 维（左右各 pos3+quat4），
`core.py` 里 `gripper_idx = None`，`send_action` 的
`if self.LEFT_GRIPPER_KEY in action` 永远不成立 —— **夹爪一次命令都没发**。
而训练数据里夹爪在 f≈20 开、f≈60 合、f≈220 又开，策略根本没有通道表达它。
于是：位置走对了 → 没夹住 → 画面不变 → 下一 chunk 预测"抬起/撤退" → 看着就是来回试。

### 修法：action = EE 绝对位姿 + **夹爪开关量**（`ee_absolute_gripper`，16 维）

**关键：夹爪必须是 0/1 开关量，不能用采集到的关节位置。**

运控 `AdaptiveGripperController` 是 `target_command` 模式，闭合时回**距初始值最近的
限位**，靠压出夹持力夹紧；给它一个"刚好碰到管子"的连续位置会**夹不紧**（历史踩过）。
0/1 送到 `/<side>_gripper_controller/target_command`（`Int32`），正是编排/VR/RViz 用的
同一条通道。

**sdg 侧怎么拿到开合位**（不走话题）：sdg 采的是 Isaac 内部
`get_dof_position_targets()`，而 `target_command` 只会往位置命令接口写两个限位之一，
所以 `joint_pos_cmd` 的夹爪列天然离散。41 条 run 实测全部只有 3 个值：

| cmd | 含义（run_01 物理结果反推：tube_z） |
|---|---|
| `-0.0200` | 夹住 → 持管抬起 `0.743 → 0.920` |
| `+0.0245` | 张开 → 管子落回 `0.743` |
| `0.0000` | 上电闲置值（两指并拢，属闭合点） |

判据 = **离哪个限位更近**（复刻 `closed_position_ = 距 init 更近的那个限位`），
不是"高于/低于中点"——后者在初始值靠近上限的机器人上会反掉。
dex1 URDF `lower=-0.02 upper=0.0245 init=0` → `closed_position_=-0.02`，与上表吻合。

落地位置：

| 文件 | 改动 |
|---|---|
| `export_lerobot.py` | `_gripper_switch_series()`（开关量推导）+ `ee_absolute_gripper` 16 维 features |
| `_lerobot_export_worker.py` | 该语义也走 `ee_actions` 路径 |
| `scripts/cli.py` | `--action-space` 增加 `ee_absolute_gripper`；`export-check` 加 `GRIPPER_NOT_BINARY` / `GRIPPER_NEVER_CLOSED` |
| `lerobot_ros2/.../core.py` | 识别左右夹爪 idx；`_binarize_gripper()` 二值化；两边都下发 |

实测导出（run_01+02）：`action shape [16]`，夹爪 uniq `[0,1]`，闭合帧 L 64.4% / R 38.7%，
`export-check` ✅。时序与任务语义吻合：`f12 R open → f53 R close(抓住) → f148 L open →
f203 L close(接住) → f233 R open(松开) → f376 L open(放入孔)`。

### 顺带修掉的两个 `export-check` bug

1. **四元数检查索引越界**（14 维时就存在，一直被 `except` 吞掉）：
   `_Q` 的列是 `_qidx` 压缩后的下标，却用 `_an` 的全局下标去索引 →
   `index 13 is out of bounds for axis 1 with size 8`。而且它连带跳过了后面
   的夹爪检查（所以新加的守卫一开始根本没跑）。改成先在局部名字表里定位。

2. **`observation.ee` 的 quat 是 wxyz，而 `action` 是 xyzw**。同一数据集里两种约定并存：
   `action` 的 `*_quat.{x,y,z,w}` 名字明确，`observation.ee` 是 14 维裸数组、**没有
   names**，值来自 `episode.npz` 的 wxyz。直接对比会差 166°（很像"左右臂镜像"，
   极易误判）。当前 ACT 只吃 `state`+图像，`observation.ee` 不在 `input_features`，
   所以**不影响训练**；将来要喂给策略必须先轮转。

### 教训

- **先看动作空间够不够表达任务**，再怀疑策略/运控。抓取任务缺夹爪维，位置再准也抓不住。
- **开关量和连续量必须在导出时就定死语义**，别在推理侧"猜"。运控的 `target_command`
  设计就是要"少给信息、多给它自己决定"，与它对齐比自己算位置稳。
- 诊断脚本里的 `try/except` 会把检查项静默吞掉 —— 加了新检查要先确认它真的跑了。

---

## 附：sdg 侧的诊断工具

| 脚本 | 用途 |
|---|---|
| `scripts/sdg.sh diag <ep>` | 7 项体检（末端真伪/关节口径/时间轴/物体/相机） |
| `scripts/sdg.sh shots <session>` | 相机联络图，人眼查 RGB |
| `scripts/sdg.sh bench <session>` | 评测指标（与话题方案同口径） |
| `scripts/sdg.sh doctor` | 环境自检 + 崩溃残留检查 |
| `scripts/probe_camera_params.py` | 相机参数字段探针（升级 Isaac 后复核） |
| `scripts/probe_ee_tracking.py` | 运控末端跟随探针（闸门状态 / 阶跃 / 步长扫描 / 轨迹回放） |
| `scripts/check_chunk_reach.py` | ACT chunk 累积位移检查（判断是否需要重训） |
| `scripts/verify_action_queue.py` | 验证 `n_action_steps` 覆盖真的生效（队列 maxlen + 前向次数） |
| `scripts/diag_wrist_camera*.py` | 腕部相机排查（问题 9 用） |
| `lerobot_ros2/.../probe_topic_rate.py` | 话题真实频率 / stamp / 批量投递检测 |
| `lerobot_ros2/.../bench_record.py` | 话题侧评测采集 |

---

## 19. `n_action_steps` 小于「合爪帧号」—— 合爪动作根本不在队列里（11-16，决定性根因）

**现象**：闭环推理时右手朝试管伸，但**总是差几厘米就停下合爪**，发现没夹住就
再来，反复试探；夹爪也在反复开合。开环回放（一次算完整 chunk）却能一把抓起。

**为什么之前一直查不到**：所有"上游"都已被逐一排除——图像管线差 0.57mm、
运控跟踪 5.3mm（与训练基线一致）、底盘水平 2.51mm、帧率修好后 20Hz、
state 填 0 只造成 1.3mm 偏差。剩下的只有"执行方式"。

**真正的根因**：`policy.select_action()` 内部是一个长度 = `n_action_steps`
的 action queue。策略在 obs=起点时，**第 49 帧**才合爪；而
`n_action_steps=25` 时队列里只有第 0~24 帧 —— **合爪那一步不在队列里，
永远执行不到**。每 25 帧队列耗尽 → 重新预测 → 因为状态还远离试管，
新 chunk 的合爪点依然在 20+ 帧后 → 死循环。

**实测（同一观测，不同队列长度，看本轮能走到哪）**：

| n_action_steps | 本轮最小 x | 合爪动作 | 结果 |
|---|---|---|---|
| 25 | **2.0647** | ❌ 不在队列内 | 反复够不到 |
| 49 | 1.9869 | ❌ 差一格 | 反复够不到 |
| 50 | 1.9869 | ✅ 第 49 帧 | 可抓 |
| **100** | **1.9842** | ✅ 第 49 帧 | **完整跑通** |

对照训练数据合爪点 x=1.9722：25 帧时手臂永远只走到 2.06（差 ~90mm）。
夹爪"反复开合"也是同一原因——队列边界处夹爪值在"张开(chunk[0])"与
"闭合(chunk[24])"之间跳变。

**修法**：`--n-action-steps` 默认改为 **100**（= chunk_size）。
这不是"把闭环退化成开环"：仍每 100 帧回读一次观测，保留纠偏能力；
只是保证队列覆盖到关键动作。

**启动自检**：`core.py` 新增 `_estimate_grasp_frame()`，用数据集第 0 帧做一次
chunk 预测、找夹爪首次 <0.5 的帧号；若 `>= n_action_steps` 就打印 WARN
（"合爪动作不在队列内，会反复够不到"）。诊断失败静默跳过，不影响启动。

**实测结果**：默认 100 下，G1D 试管任务**完整成功**——右手抓管 → 递给左手 →
左手微调后插入孔位，全程受控、无掉落、无抽搐。这是本任务首次端到端跑通。

### 教训

1. **action queue 的长度就是"一次能看多远"**。凡是"关键动作在序列中段"的任务，
   `n_action_steps` 必须覆盖到那一步，否则那一步永远不存在。判据是
   **策略在该观测下第几帧做关键动作**，不是"经验值 25"。
2. 先分层排除上游（图像/运控/底盘/帧率/state），最后才怀疑执行方式 ——
   但**执行方式一旦出问题，症状看起来和"策略没学好"一模一样**。
3. **开环能成功、闭环失败**时，第一件事应该是对比"开环拿到多少帧、
   闭环一轮拿到多少帧"，而不是怀疑模型。
