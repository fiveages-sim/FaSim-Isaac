"""运行时物体位姿随机化（配置驱动，不与具体机器人/任务强绑定）。

## 要解决什么
自动化采数据时，每轮之间要换一个场景布局，否则 100 条数据都是同一个配置，
策略学不到「泛化到不同物体位置」。手工摆位太慢，所以需要脚本化随机化。

## 设计约束（都是踩过的坑）
1. **先归位再随机化**
   任务执行完物体可能被移动过（拿到手里、插进孔里）。直接对「当前位置」加
   随机偏移会把物体甩到桌子上或穿透架子。所以顺序必须是：
     ① 物体（含挂在夹爪上的）回到**采集开始时**的位姿
     ② 再叠加随机偏移
2. **只随机化配置里声明的轴与范围**
   试管任务只需 xy（物体在桌面上，z 由桌面决定）。z 也随机会让物体悬空或
   陷进桌子。范围一律由配置给，代码里不写死。
3. **不假设物体一定自由**
   「试管架 + 试管」是两个刚体，且试管可能正被夹爪握着。随机化试管架时必须
   把试管一起搬（或先把试管归位到架上），否则试管留在原地、穿模。

## 配置格式（task yaml 的 ``randomize:`` 段）

```yaml
randomize:
  seed: 0                    # 0 = 每次用时间戳
  settle_steps: 30           # 设完位姿后让物理跑多少步（稳定）
  objects:
    # ① 先随机化「被插入的架子」
    - prim: /World/room/scene/test_tube_rack_01
      axes: {x: 0.04, y: 0.03}
    # ② 再把试管**放进那个架子的指定孔里**（不随机化，而是精确放置）
    - prim: /World/room/scene/test_tube
      place_on:
        target: /World/room/scene/test_tube_rack_01
        frame: hole_4            # 目标的子 frame（USD 局部链里的名字）
        align: tube_bottom       # 本物体要落在这个 frame 上的参考点
        offset: [0.0, 0.0, 0.0]  # 在 frame 系内的额外偏移（米）
```

**``place_on`` vs ``follow`` 的区别（关键）**

| | 语义 | 适用 |
|---|---|---|
| `follow: <prim>` | 跟随另一个物体的**平移量** | 两个物体是刚性组合、且已知相对位置 |
| `place_on: {target, frame}` | 从 target 的**子 frame** 读出世界位姿，把本物体精确放过去 | 需要「插进某个孔 / 吸附到某个插槽」——**不依赖相对位置是否稳定** |

试管任务必须用 `place_on`：任务结束时试管已经被拿走了，`follow` 只能算
平移量、还原不出「在孔里」这个状态；而 `place_on` 每轮都重新从
rack_01 的 `hole_4` 读世界位姿，**无论架子被随机到哪，试管都在孔里**。

顺序语义：**先处理 `axes`（随机平移），再处理 `place_on`（精确放置）**。
这样「先随机架子、再按架子新位置放试管」天然成立，不需要额外排序字段。

``follow`` 的语义：该物体不单独随机化，而是**跟着 ``follow`` 指定的物体
平移相同的偏移量**（保持相对布局）。适合「架子上插着试管」这种组合。

## 为什么需要「记录初始位姿」
Isaac 里没有可靠的「USD 原始位姿」读取路径（newton kit 关掉了
PhysX→USD 同步，Xform 读到的可能是 stale 值——见 DEBUG_LOG #1）。
所以策略是：**采集开始时（第一轮之前）抓一次位姿存进内存/meta**，
之后每轮都从这份快照恢复再随机化。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

_LOG_PREFIX = "[randomize]"

#: 存初始位姿的文件名（放在会话/任务数据目录下，跨进程可复用）
SNAPSHOT_NAME = "scene_initial_poses.json"


def _log(msg: str) -> None:
    print(f"{_LOG_PREFIX} {msg}", flush=True)


# ------------------------------------------------------------------ 位姿读写


def _rigid(prim_path: str):
    """创建 RigidPrim（物体必须是刚体才有 get/set_world_poses）。"""
    from isaacsim.core.experimental.prims import RigidPrim

    return RigidPrim(prim_path)


def _to_np(v) -> np.ndarray:
    a = np.asarray(v, dtype=np.float64)
    return a.reshape(a.shape[-1]) if a.ndim else a


def _quat_rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用四元数（w,x,y,z）旋转向量 v。"""
    q = np.asarray(q_wxyz, dtype=np.float64).reshape(-1)[:4]
    n = np.linalg.norm(q)
    if n < 1e-9:
        return np.asarray(v, dtype=np.float64)
    w, x, y, z = q / n
    vv = np.asarray(v, dtype=np.float64).reshape(3)
    # v' = v + 2w(q_vec × v) + 2 q_vec × (q_vec × v)
    qv = np.array([x, y, z], dtype=np.float64)
    t = 2.0 * np.cross(qv, vv)
    return vv + w * t + np.cross(qv, t)


def get_pose(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读刚体世界位姿 → (pos(3), quat_wxyz(4))；失败返回 None。

    只在 Isaac 进程内可用（依赖 isaacsim 的 RigidPrim）。外部进程请用
    :func:`get_pose_any`（会自动退回 ROS 服务）。
    """
    try:
        rp = _rigid(prim_path)
        p, q = rp.get_world_poses()
        pos = _to_np(p).reshape(-1)[:3]
        quat = _to_np(q).reshape(-1)[:4]
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return None
        return pos, quat
    except Exception:  # noqa: BLE001
        # 静默：外部进程（无 isaacsim 模块）会走 get_pose_any 的服务回退
        return None


def _get_pose_xform_fallback(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """Isaac 内 RigidPrim 读不到时的补充路径：USD Xform 世界位姿。

    **仅用于静态/非物理驱动 prim**（如架子本身在 authoring stage 上的位姿）。
    对物理驱动的刚体，USD 值可能是 stale（newton kit 关了同步），
    所以它只作 fallback，且调用方应知道精度可能不足。
    """
    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return None
        xf = UsdGeom.Xformable(prim)
        if not xf:
            return None
        m = xf.ComputeLocalToWorldTransform(0.0)
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        if q is None:
            return None
        pos = np.array([float(t[0]), float(t[1]), float(t[2])], dtype=np.float64)
        im = q.GetImaginary()
        quat = np.array([float(q.GetReal()), float(im[0]),
                         float(im[1]), float(im[2])], dtype=np.float64)
        n = np.linalg.norm(quat)
        if n > 1e-9:
            quat = quat / n
        return pos, quat
    except Exception as exc:  # noqa: BLE001
        _log_once(f"usd_pose_exc:{prim_path}",
                  f"WARN USD 读世界位姿失败 {prim_path}: {type(exc).__name__}: {exc}")
        return None


def set_pose_service(prim_path: str, pos: np.ndarray, quat: np.ndarray,
                     *, zero_velocity: bool = True) -> bool:
    """通过 ``/set_entity_state`` 服务设刚体位姿（**外部进程也能用**）。

    这个服务一次可以同时设 pose 和 twist，正好解决「设了位姿但速度还在 →
    下一帧被甩飞」的问题。

    为什么必须走服务：``set_pose`` 用的 RigidPrim 需要 isaacsim 模块，
    在 ``sdg.sh`` 这种外部进程里 import 失败 → 静默不生效（实测：日志显示
    「2 个物体」但物体根本没动，试管还是崩飞）。
    """
    try:
        from simulation_interfaces.srv import SetEntityState

        req = SetEntityState.Request()
        req.entity = prim_path
        pose = req.state.pose
        pose.position.x = float(pos[0])
        pose.position.y = float(pos[1])
        pose.position.z = float(pos[2])
        pose.orientation.x = float(quat[1])     # quat 传进来是 wxyz
        pose.orientation.y = float(quat[2])
        pose.orientation.z = float(quat[3])
        pose.orientation.w = float(quat[0])
        req.state.header.frame_id = "world"
        if zero_velocity:
            req.state.twist.linear.x = 0.0
            req.state.twist.linear.y = 0.0
            req.state.twist.linear.z = 0.0
            req.state.twist.angular.x = 0.0
            req.state.twist.angular.y = 0.0
            req.state.twist.angular.z = 0.0
        r = _call_service(SetEntityState, "/set_entity_state", req, timeout=5.0)
        if r is None:
            return False
        # 注意：响应的 `result` 字段本身是嵌套的 `Result` 消息
        # （含 result / error_message），不是数字 —— 直接 int() 会抛
        # "int() argument must be ... not 'Result'"。
        res_msg = getattr(r, "result", None)
        code = int(getattr(res_msg, "result", 0) or 0)
        if code == 1:                      # RESULT_OK
            return True
        msg = getattr(res_msg, "error_message", "") or f"result_code={code}"
        _log(f"WARN set_entity_state({prim_path}) 失败: {msg}")
        return False
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN set_entity_state 异常 {prim_path}: {exc}")
        return False


def set_pose_usd_api(prim_path: str, pos: np.ndarray, quat: np.ndarray) -> bool:
    """**在 Isaac 进程内**用 USD API 直接写 prim 的世界位姿。

    与 RigidPrim 的区别：RigidPrim 写的是 PhysX 状态（会被物理改写）；
    USD API 写的是 stage（下一帧物理会读它作为初值）。
    在 randomize 这种「先摆位置再让物理 settle」的场景里，
    写 USD 更符合语义，而且**同步、无阻塞**。
    """
    try:
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return False
        xf = UsdGeom.Xformable(prim)
        if not xf:
            return False

        # 父级世界变换的逆，用于把世界位姿换算成局部
        parent = prim.GetParent()
        parent_inv = Gf.Matrix4d(1.0)
        if parent and parent.IsValid():
            pxf = UsdGeom.Xformable(parent)
            if pxf:
                parent_inv = pxf.ComputeLocalToWorldTransform(0.0).GetInverse()

        m = Gf.Matrix4d(1.0)
        m.SetTranslate(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        m.SetRotateOnly(Gf.Quatd(float(quat[0]),
                                 Gf.Vec3d(float(quat[1]), float(quat[2]), float(quat[3]))))
        local = m * parent_inv

        t = local.ExtractTranslation()
        q = local.ExtractRotationQuat()
        if q is None:
            return False

        ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
        t_op = ops.get("xformOp:translate")
        if t_op is None:
            t_op = xf.AddTranslateOp()
        t_op.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
        q_op = ops.get("xformOp:orient")
        if q_op is None:
            q_op = xf.AddOrientOp()
        im = q.GetImaginary()
        # 用 Quatf 还是 Quatd 取决于 op 的精度；Quatf 最常见
        try:
            q_op.Set(Gf.Quatf(float(q.GetReal()),
                              Gf.Vec3f(float(im[0]), float(im[1]), float(im[2]))))
        except Exception:  # noqa: BLE001
            q_op.Set(Gf.Quatd(float(q.GetReal()),
                              Gf.Vec3d(float(im[0]), float(im[1]), float(im[2]))))
        return True
    except Exception as exc:  # noqa: BLE001
        _log_once(f"usd_set_exc:{prim_path}",
                  f"WARN USD 写位姿失败 {prim_path}: {type(exc).__name__}: {exc}")
        return False


def set_pose(prim_path: str, pos: np.ndarray, quat: np.ndarray,
             *, zero_velocity: bool = True) -> bool:
    """写世界位姿（Isaac 内优先 USD API，外部退回 ROS 服务）。

    ``zero_velocity``：清零速度。只设位姿不清速度时，物体会保留上一轮
    被机器人握着移动的速度，下一帧物理一跑就甩出去（实测试管崩飞）。

    **为什么 Isaac 内不用服务**：randomize 从 app.update() 回调里调用，
    主循环被占用，ROS 服务拿不到响应（实测超时 → 静默失败 → 试管不动）。
    """
    # 1) Isaac 内：USD API（同步、不阻塞）
    if set_pose_usd_api(prim_path, pos, quat):
        if zero_velocity:
            _zero_velocity_isaac(prim_path)
        return True
    # 2) 外部进程：ROS 服务
    return set_pose_service(prim_path, pos, quat, zero_velocity=zero_velocity)


def _zero_velocity_isaac(prim_path: str) -> None:
    """Isaac 内清零刚体速度（RigidPrim 接口，逐版本兼容）。"""
    try:
        rp = _rigid(prim_path)
    except Exception:  # noqa: BLE001
        return
    for meth, args in (
        ("set_linear_velocities", (np.zeros((1, 3)),)),
        ("set_velocities", (np.zeros((1, 6)),)),
    ):
        fn = getattr(rp, meth, None)
        if fn is None:
            continue
        try:
            fn(*args)
            break
        except Exception:  # noqa: BLE001
            continue
    fn = getattr(rp, "set_angular_velocities", None)
    if fn is not None:
        try:
            fn(np.zeros((1, 3)))
        except Exception:  # noqa: BLE001
            pass


def _quat_mul_wxyz(a, b) -> np.ndarray:
    """四元数乘法（w,x,y,z），a ⊗ b。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def _ros_node():
    """取（或创建）一个共享的 ROS 节点，供本模块所有服务调用复用。

    **三个关键点**（都是踩过的）：

    1. ``use_sim_time=False`` —— 必须用**墙钟**。
       Isaac 内创建的节点默认可能带 sim time；而 ``/clock`` 在仿真里由
       Isaac 自己发，节点若不 spin 在 /clock 上，``wait_for_service`` 会
       永远等下去 —— 表现就是「随机化时卡住」。
    2. 只创建一次、共享复用。反复 create/destroy 会导致 Isaac 侧
       sendResponse 报错刷屏（lerobot 侧也有同样注释）。
    3. ``spin_once`` 轮询而不是 ``spin_until_future_complete`` ——
       后者在 Isaac 主循环里会阻塞主线程，服务永远等不到响应（死锁）。

    实测：外部进程（sdg.sh）能读到 hole_4，而 Isaac 进程内读不到 ——
    差别就在这三条。
    """
    node = getattr(_ros_node, "_node", None)
    if node is not None:
        return node
    import rclpy
    from rclpy.node import Node

    if not rclpy.ok():
        rclpy.init()
    # 用节点参数 API 设 use_sim_time（避免手工构造 Parameter/ParameterValue
    # 的 type 字段 —— 那两个消息类型的构造方式在 rclpy 各版本不一致，踩过）。
    node = rclpy.create_node("sdg_randomize_svc")
    try:
        node.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, False)])
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 设置 use_sim_time=False 失败（可能影响服务等待）: {exc}")
    _ros_node._node = node       # type: ignore[attr-defined]
    return node


def _client_for(srv_type, srv_name: str):
    node = _ros_node()
    cache = getattr(_client_for, "_cache", None)
    if cache is None:
        cache = {}
        _client_for._cache = cache     # type: ignore[attr-defined]
    cli = cache.get(srv_name)
    if cli is None:
        cli = node.create_client(srv_type, srv_name)
        cache[srv_name] = cli
    return node, cli


def _call_service(srv_type, srv_name: str, request, *, timeout: float = 5.0):
    """调一个服务，**用墙钟 + spin_once 轮询**（Isaac 内安全）。

    Returns:
        响应对象；失败返回 None（调用方自己决定怎么报）。
    """
    import time as _t

    import rclpy

    try:
        node, cli = _client_for(srv_type, srv_name)
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 创建服务客户端失败 {srv_name}: {exc}")
        return None

    # 等服务可用（墙钟轮询）
    deadline = _t.monotonic() + max(0.5, timeout)
    while _t.monotonic() < deadline:
        if cli.service_is_ready():
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    else:
        if not cli.service_is_ready():
            _log(f"WARN 服务不可用: {srv_name}")
            return None

    fut = cli.call_async(request)
    start = _t.monotonic()
    while not fut.done():
        if (_t.monotonic() - start) > max(0.5, timeout):
            _log(f"WARN 服务超时: {srv_name}")
            return None
        rclpy.spin_once(node, timeout_sec=0.01)   # ← 关键：不用 spin_until_future_complete
    try:
        if fut.exception() is not None:
            _log(f"WARN 服务异常 {srv_name}: {fut.exception()}")
            return None
        return fut.result()
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 取服务结果失败 {srv_name}: {exc}")
        return None


def _read_xform_via_usd_api(prim_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """**在 Isaac 进程内**直接用 USD API 读局部 TRS（不经 ROS，最快最可靠）。

    为什么必须这么做：sdg 的 randomize 是从 ``app.update()`` 回调里调用的，
    此时 **Isaac 主循环正被占用**，任何 ROS 服务调用（``spin_once`` 等）都拿
    不到响应 —— 实测报 ``WARN 服务超时: /get_prim_attribute``，导致 hole_4
    读不到、试管被跳过（看起来就是"试管没跟着架子走、躺在桌子旁边"）。

    直接查 USD stage 没有这个问题：它就是在当前线程同步返回。

    ### 实现要点（踩过）
    ``op.Get()`` 对 translate 返回 ``Gf.Vec3d/Vec3f``、对 orient 返回
    ``Gf.Quatf/Quatd``。**``np.asarray(Gf.Quatf)`` 会抛异常**，于是整个函数
    被 except 吞掉、返回 None，又退回（会超时的）服务路径。
    所以这里对每种类型都显式取值，不做宽松转换。
    """
    try:
        import omni.usd
        from pxr import Gf, UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            _log_once("usd_api", "WARN get_stage() 返回 None")
            return None
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            _log_once(f"usd_api:{prim_path}", f"WARN prim 无效: {prim_path}")
            return None
        xf = UsdGeom.Xformable(prim)
        if not xf:
            _log_once(f"usd_api_xf:{prim_path}", f"WARN 不可 Xformable: {prim_path}")
            return None

        t = np.zeros(3, dtype=np.float64)
        q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        s = np.ones(3, dtype=np.float64)
        for op in xf.GetOrderedXformOps():
            name = op.GetOpName()
            val = op.Get()
            if "translate" in name:
                t = np.array([float(val[0]), float(val[1]), float(val[2])], dtype=np.float64)
            elif "orient" in name:
                # Gf.Quatf / Gf.Quatd：(real, imaginary) → wxyz，与我们要的一致
                if isinstance(val, (Gf.Quatf, Gf.Quatd)):
                    im = val.GetImaginary()
                    v = np.array([float(val.GetReal()), float(im[0]),
                                  float(im[1]), float(im[2])], dtype=np.float64)
                else:
                    v = np.array([float(val[0]), float(val[1]),
                                  float(val[2]), float(val[3])], dtype=np.float64)
                n = np.linalg.norm(v)
                q = v / n if n > 1e-9 else q
            elif "scale" in name:
                s = np.array([float(val[0]), float(val[1]), float(val[2])], dtype=np.float64)
        return t, q, s
    except Exception as exc:  # noqa: BLE001
        _log_once(f"usd_api_exc:{prim_path}",
                  f"WARN USD API 读 {prim_path} 异常: {type(exc).__name__}: {exc}")
        return None


def _log_once(key: str, msg: str) -> None:
    """同类消息只打一次（避免每帧刷屏）。"""
    seen = getattr(_log_once, "_seen", None)
    if seen is None:
        seen = set()
        _log_once._seen = seen          # type: ignore[attr-defined]
    if key in seen:
        return
    seen.add(key)
    _log(msg)


def _read_xform_via_service(prim_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """读一个 prim 的**局部** TRS。优先 USD API，退回 ROS 服务（带缓存）。

    为什么走服务而不是 XformPrim：newton kit 关了 PhysX→USD 同步
    （DEBUG_LOG #1），XformPrim 在本环境读到的是 stale/无效值。

    **缓存**：authoring stage 的局部 TRS 在仿真期间**不会变**（随机化只改
    PhysX，不写 USD），所以按 prim_path 缓存，避免每次读都等服务、spin ——
    那是「随机化卡顿」的来源（每次读 3 个属性 × 若干 prim × 等 100ms）。

    Returns:
        ``(translate(3), orient_wxyz(4), scale(3))``；读不到返回 None。
    """
    cache = getattr(_read_xform_via_service, "_cache", None)
    if cache is None:
        cache = {}
        _read_xform_via_service._cache = cache       # type: ignore[attr-defined]
    if prim_path in cache:
        return cache[prim_path]

    # 1) Isaac 进程内：直接用 USD API（同步、无阻塞）
    out = _read_xform_via_usd_api(prim_path)
    if out is not None:
        cache[prim_path] = out
        return out

    # 2) 外部进程：走 ROS 服务
    try:
        import json as _json

        from isaac_ros2_messages.srv import GetPrimAttribute

        def _ask(attr: str):
            req = GetPrimAttribute.Request()
            req.path = prim_path
            req.attribute = attr
            r = _call_service(GetPrimAttribute, "/get_prim_attribute", req, timeout=2.0)
            if r is None or not getattr(r, "success", False):
                return None
            raw = (getattr(r, "value", "") or "").strip()
            if not raw:
                return None
            try:
                return _json.loads(raw)
            except Exception:  # noqa: BLE001
                return None

        t = _ask("xformOp:translate")
        if t is None:
            cache[prim_path] = None
            return None
        o = _ask("xformOp:orient")
        s = _ask("xformOp:scale")
        tv = np.asarray(t, dtype=np.float64).reshape(-1)[:3]
        ov = (np.asarray(o, dtype=np.float64).reshape(-1)[:4]
              if o is not None else np.array([1.0, 0.0, 0.0, 0.0]))
        sv = (np.asarray(s, dtype=np.float64).reshape(-1)[:3]
              if s is not None else np.ones(3))
        n = np.linalg.norm(ov)
        if n > 1e-9:
            ov = ov / n
        out = (tv, ov, sv)
        cache[prim_path] = out
        return out
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读 {prim_path} 的 xform 失败: {exc}")
        cache[prim_path] = None
        return None


def get_pose_service(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """通过 ``/get_entity_state`` 服务读刚体世界位姿（**外部进程也能用**）。

    与 :func:`get_pose` 的区别：那个用 RigidPrim（只能在 Isaac 进程内跑）。
    本函数走 ROS 服务，所以 ``sdg.sh randomize`` 这种外部调用也能读。
    内部用 :func:`_call_service`（墙钟 + spin_once），Isaac 内也安全。
    """
    try:
        from simulation_interfaces.srv import GetEntityState

        req = GetEntityState.Request()
        req.entity = prim_path
        r = _call_service(GetEntityState, "/get_entity_state", req, timeout=3.0)
        if r is None or not getattr(r, "result", None):
            return None
        st = getattr(r, "state", None)
        pose = getattr(st, "pose", None) if st is not None else None
        if pose is None:
            return None
        p = pose.position
        o = pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        quat = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)   # wxyz
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return None
        n = np.linalg.norm(quat)
        if n > 1e-9:
            quat = quat / n
        return pos, quat
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 服务读位姿失败 {prim_path}: {exc}")
        return None


def get_pose_any(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读世界位姿，按「Isaac 内优先，外部退回服务」的顺序尝试。

    顺序：
      1. RigidPrim（Isaac 内，物理真值，最快）
      2. USD Xform（Isaac 内，静态 prim 的兜底）
      3. ROS /get_entity_state（外部进程）

    **为什么 Isaac 内不能走服务**：sdg 的 randomize 从 ``app.update()``
    回调里调用，主循环被占用，ROS 服务拿不到响应（实测 ``服务超时:
    /get_prim_attribute``，导致 tube 被跳过）。所以在 Isaac 里必须用
    同步的 USD / PhysX 接口。
    """
    got = get_pose(prim_path)
    if got is not None:
        return got
    # Isaac 内：USD 兜底
    got = _get_pose_xform_fallback(prim_path)
    if got is not None:
        return got
    # 外部进程：ROS 服务
    return get_pose_service(prim_path)


def isaac_channel_selftest() -> dict:
    """自检：确认「在 Isaac 回调里」能读/写位姿的各条通道是否可用。

    必须在**真实回调上下文**里跑才有意义 —— 外部进程跑会得到不同结果
    （外部进程里 ROS 服务是可用的，Isaac 回调里不可用）。

    Returns:
        ``{"usd_read": {...}, "usd_write": bool, "rigid": {...}, "service": {...}}``
    """
    out: dict = {"usd_read": None, "usd_write": None,
                 "rigid_read": None, "service_read": None, "stage_ok": None}
    probe = "/World/room/scene/test_tube_rack_01"
    sub = "/World/room/scene/test_tube_rack_01/hole_4"

    try:
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        out["stage_ok"] = stage is not None
    except Exception as exc:  # noqa: BLE001
        out["stage_ok"] = f"exc: {type(exc).__name__}: {exc}"

    # 1) USD API 读局部 TRS
    r = _read_xform_via_usd_api(sub)
    out["usd_read"] = None if r is None else {
        "t": [round(float(x), 5) for x in r[0]],
        "q": [round(float(x), 5) for x in r[1]],
    }

    # 2) USD API 读世界位姿
    w = _get_pose_xform_fallback(probe)
    out["usd_world"] = None if w is None else {
        "pos": [round(float(x), 5) for x in w[0]]}

    # 3) RigidPrim 读
    g = get_pose(probe)
    out["rigid_read"] = None if g is None else {
        "pos": [round(float(x), 5) for x in g[0]]}

    # 4) 服务读（预期在回调里会超时 —— 这正是 bug 的来源）
    s = get_pose_service(probe)
    out["service_read"] = None if s is None else {
        "pos": [round(float(x), 5) for x in s[0]]}

    # 5) USD 写（写到当前位置，等价 no-op）
    if w is not None:
        out["usd_write"] = set_pose_usd_api(probe, w[0], w[1])
    return out


def get_prim_local_trs(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读 prim 的**局部** translate/orient（authoring stage 的常量）。

    与 ``get_pose_any`` 的区别：那个读的是**物理真值**（会随随机化/物理变化），
    这个读的是 **USD 里写死的值**（资产固化的关系，永远不变）。

    典型用途：从 ``room.usda`` 推出「试管相对架子的静止关系」——
    这层关系在资产里是固定的，不依赖任何一次仿真的状态。
    """
    got = _read_xform_via_service(prim_path)
    if got is None:
        return None
    t, q, _s = got
    return np.asarray(t, dtype=np.float64), np.asarray(q, dtype=np.float64)


def get_world_pose_from_usd(prim_path: str,
                            max_depth: int = 8) -> tuple[np.ndarray, np.ndarray] | None:
    """**纯从 USD authoring stage** 合成 prim 的世界位姿（不碰 PhysX）。

    Args:
        prim_path: 目标 prim 的完整路径。
        max_depth: 最多向上回溯几级父节点（防意外无限循环）。

    Returns:
        ``(pos(3), quat_wxyz(4))``；任一级读不到就返回 None。

    ### 用途
    计算「资产里固化的相对关系」。例如：

        rel = get_world_pose_from_usd(tube) − get_world_pose_from_usd(rack)

    这个 rel 是**资产几何常量**，不随随机化/物理变化。于是随机化时：

        试管新位姿 = rack 当前位姿(PhysX) + rel

    就能让试管无论架子搬到哪都保持在孔里。

    ### 为什么必须走这条路
    早期用「快照时的相对偏移」也有类似效果，但快照是在「某一轮物理状态」
    下拍的，试管可能已经歪了一点，误差会被固化进 rel。而 USD 里的是
    authoring 原值，最干净。

    ### 注意 z
    USD 里 ``test_tube`` 的 translate.z 是**自由悬浮**时的值（0.7972），
    而物理落稳后管底会下沉到孔里（实测 0.743）。所以从 USD 推出的 rel.z
    比真实值大 52.7mm。**x/y 是精确的，z 需要按实测校正**（见
    ``place_on.z_source`` 参数）。
    """
    parts = [p for p in str(prim_path).strip("/").split("/") if p]
    if not parts:
        return None
    # 完整祖先链：/World → /World/room → ... → prim（不做深度截断！
    # 早期版本用 max(1, len(parts)-max_depth) 算起点，当层级很少时会
    # 漏掉最上面几级，导致合成结果完全跑偏（实测 rack_01 算出 z=1.47，
    # 正确值是 0.736）。层级最多几十个，全走一遍开销可忽略。
    chain = ["/" + "/".join(parts[:i + 1]) for i in range(len(parts))]
    if max_depth > 0 and len(chain) > max_depth:
        chain = chain[-max_depth:]

    P = np.zeros(3, dtype=np.float64)
    Q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    for p in chain:
        trs = get_prim_local_trs(p)
        if trs is None:
            # 该级没有 xform（如 /World 是 Xform 但可能没有 xformOp）→ 视为恒等
            continue
        t, q = trs
        P = P + _quat_rotate(Q, t)
        Q = _quat_mul_wxyz(Q, q)
        n = np.linalg.norm(Q)
        if n > 1e-9:
            Q = Q / n
    return P, Q


def get_subframe_world_pose(parent_prim: str, frame: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读「parent_prim 下的子 frame」的世界位姿 → (pos(3), quat_wxyz(4))。

    为什么需要它：编排的 grasp_id（如 ``hole_4``）就是 rack USD 内部的子
    frame，代表「孔的中心 + 朝向」。要做「把试管放进这个孔」，就得拿到这个
    frame 的**世界**位姿，而不是靠猜相对偏移。

    ### 为什么不能只合成 USD xform 链

    第一版实现是「把 /World → /World/room → scene → rack → hole_4 的
    xformOp:translate 逐级串起来」。这是**错的**，因为：

      ``set_entity_state``（服务）只改 **PhysX 里的刚体位姿**，
      **不写回 USD 的 xformOp:translate**。

    所以架子被随机平移后，从服务读到的 ``rack_01`` translate 仍是**旧值**
    （实测：架子实际在 y=0.278，服务读到 0.1005）。串出来的 hole_4 世界
    坐标自然也是旧的 → 试管被放到错误位置。

    ### 正确做法

      hole_4 世界位姿 = parent 的**当前**世界位姿 ⊗ hole_4 的**局部** TRS

    其中 parent 当前位姿来自 ``/get_entity_state``（PhysX 真值），
    hole_4 局部 TRS 来自 ``/get_prim_attribute``（authoring stage 里的常量）。
    —— 局部量确实是不变的，只有父级的**世界**位姿会因随机化而变。
    """
    path = f"{parent_prim.rstrip('/')}/{frame}"
    local = _read_xform_via_service(path)
    if local is None:
        _log(f"WARN 子 frame 不存在或读不到: {path}")
        return None
    t_local, q_local, _s_local = local

    # 父级当前世界位姿（PhysX 真值，含随机化后的平移）
    parent = get_pose_any(parent_prim)
    if parent is None:
        _log(f"WARN 读不到父级 {parent_prim} 的世界位姿")
        return None
    p_parent, q_parent = parent

    # world = parent_world ⊗ local
    pos = np.asarray(p_parent, dtype=np.float64) + _quat_rotate(q_parent, t_local)
    quat = _quat_mul_wxyz(np.asarray(q_parent, dtype=np.float64),
                          np.asarray(q_local, dtype=np.float64))
    n = np.linalg.norm(quat)
    if n > 1e-9:
        quat = quat / n

    _log(f"  {path} 世界位姿 = pos {np.round(pos, 4).tolist()} "
         f"quat(wxyz) {np.round(quat, 4).tolist()}  "
         f"(父级 {np.round(p_parent, 4).tolist()} + 局部 {np.round(t_local, 4).tolist()})")
    return pos, quat


# ------------------------------------------------------------------ 配置解析


def load_config(task_config: Any) -> dict | None:
    """从任务配置里取 ``randomize`` 段。没有就返回 None（不做随机化）。"""
    if task_config is None:
        return None
    cfg = getattr(task_config, "randomize", None)
    if cfg is None and isinstance(task_config, dict):
        cfg = task_config.get("randomize")
    if not cfg:
        return None
    return dict(cfg)


def _axes_of(obj: dict) -> dict[str, float]:
    raw = obj.get("axes") or {}
    out: dict[str, float] = {}
    for ax in ("x", "y", "z"):
        if ax in raw:
            try:
                v = float(raw[ax])
            except (TypeError, ValueError):
                continue
            if v > 0:
                out[ax] = v
    return out


# ------------------------------------------------------------------ 快照


def snapshot_path(data_dir: str | Path) -> Path:
    return Path(data_dir) / SNAPSHOT_NAME


def save_snapshot(cfg: dict, data_dir: str | Path) -> dict[str, Any]:
    """记录所有待随机化物体的**当前**世界位姿（第一轮开始前调用一次）。

    ``place_on`` 的物体**也要记录** —— 它需要快照位姿来算「对齐参考 frame
    相对物体原点的局部偏移」（见 restore_and_randomize 里的 align 逻辑）。
    只是它不参与「归位 + 随机偏移」，由 place_on 精确放置。
    """
    snap: dict[str, Any] = {"taken_at": time.time(), "objects": {}}
    for obj in (cfg.get("objects") or []):
        prim = str(obj.get("prim") or "")
        if not prim:
            continue
        got = get_pose_any(prim)
        if got is None:
            snap["objects"][prim] = None
            _log(f"WARN 快照读不到 {prim}")
            continue
        pos, quat = got
        snap["objects"][prim] = {
            "pos": [float(x) for x in pos],
            "quat": [float(x) for x in quat],
            "place_on": isinstance(obj.get("place_on"), dict),
        }
        _log(f"快照 {prim}: pos={np.round(pos, 4).tolist()}")
    p = snapshot_path(data_dir)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(snap, indent=2, ensure_ascii=False), encoding="utf-8")
        _log(f"已保存初始位姿 → {p}")
    except OSError as exc:
        _log(f"WARN 保存快照失败: {exc}")
    return snap


def load_snapshot(data_dir: str | Path) -> dict | None:
    p = snapshot_path(data_dir)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读快照失败: {exc}")
        return None


# ------------------------------------------------------------------ 主流程


def restore_and_randomize(
    cfg: dict,
    *,
    data_dir: str | Path,
    rng: np.random.Generator | None = None,
    settle_fn=None,
) -> dict:
    """把物体恢复到初始位姿，再叠加随机偏移。

    Args:
        cfg: ``randomize`` 配置段（见模块 docstring）。
        data_dir: 存初始位姿快照的目录。
        rng: 随机源；None 时按 ``cfg['seed']`` 或时间戳创建。
        settle_fn: 可选，``settle_fn(n_steps)`` —— 让物理跑 n 步。
            不给就跳过（调用方负责推进）。

    Returns:
        ``{"ok": bool, "applied": [...], "warnings": [...], "seed": int}``
    """
    if rng is None:
        seed = int(cfg.get("seed") or 0) or int(time.time()) & 0x7FFFFFFF
        rng = np.random.default_rng(seed)
    else:
        seed = int(getattr(rng, "_fasim_seed", -1))

    snap = load_snapshot(data_dir)
    warnings: list[str] = []
    applied: list[dict] = []
    if snap is None:
        msg = f"没有初始位姿快照（{snapshot_path(data_dir)}）—— 先调 save_snapshot"
        _log(f"ERROR {msg}")
        return {"ok": False, "applied": [], "warnings": [msg], "seed": seed}

    objects = cfg.get("objects") or []

    # ---- 第一遍：把每个物体搬回初始位姿 ----
    # 先全部归位再做偏移，避免"A 挪了、B 还用旧相对位置去算"的错位。
    offsets: dict[str, np.ndarray] = {}
    #: 本轮**写入**的目标位姿（prim → pos）。第三遍 place_on 必须用它，
    #: 而不是重读 —— 写入 USD 后 PhysX 需要一帧才同步，立刻重读会拿到旧值
    #: （实测：试管按「架子旧位置 + 偏移」放，落在孔边、和架子重叠）。
    written: dict[str, np.ndarray] = {}
    for obj in objects:
        prim = str(obj.get("prim") or "")
        if not prim:
            continue
        # place_on 的物体不走"归位+偏移"，由第三遍精确放置，跳过这里
        if isinstance(obj.get("place_on"), dict):
            continue
        rec = (snap.get("objects") or {}).get(prim)
        if not rec:
            warnings.append(f"{prim}: 快照里没有记录，跳过")
            continue
        axes = _axes_of(obj)
        off = np.zeros(3, dtype=np.float64)
        for i, ax in enumerate(("x", "y", "z")):
            if ax in axes:
                off[i] = float(rng.uniform(-axes[ax], axes[ax]))
        offsets[prim] = off
        target = np.asarray(rec["pos"], dtype=np.float64) + off
        quat = np.asarray(rec["quat"], dtype=np.float64)
        if set_pose(prim, target, quat):
            written[prim] = target
            applied.append({
                "prim": prim,
                "offset": [round(float(v), 4) for v in off],
                "target": [round(float(v), 4) for v in target],
            })

    # ---- 第二遍：follow 关系的物体，跟着目标平移相同偏移 ----
    for obj in objects:
        prim = str(obj.get("prim") or "")
        src = str(obj.get("follow") or "")
        if not prim or not src:
            continue
        if isinstance(obj.get("place_on"), dict):
            continue          # place_on 优先，第三遍处理
        rec = (snap.get("objects") or {}).get(prim)
        if not rec:
            continue
        off = offsets.get(src)
        if off is None:
            warnings.append(f"{prim}: follow={src} 没有随机偏移，按原位不动")
            off = np.zeros(3)
        target = np.asarray(rec["pos"], dtype=np.float64) + off
        quat = np.asarray(rec["quat"], dtype=np.float64)
        if set_pose(prim, target, quat):
            written[prim] = target
            applied.append({
                "prim": prim,
                "follow": src,
                "offset": [round(float(v), 4) for v in off],
            })

    # ---- 第三遍：place_on —— 把物体**精确放进**目标的子 frame ----
    #
    # 这是「插进孔里」的正确做法。与 follow 的区别：
    #   follow  只搬平移量，依赖「相对位置在快照时就正确」
    #   place_on 每轮都重新从 target 的子 frame 读**世界**位姿，所以
    #            无论 target 被随机到哪，本物体都落在目标 frame 上
    #
    # 顺序上天然是「先随机架子（第一遍 axes）→ 再按架子新位置放试管」，
    # 因为本遍在 axes 遍之后执行。
    for obj in objects:
        prim = str(obj.get("prim") or "")
        po = obj.get("place_on")
        if not prim or not isinstance(po, dict):
            continue
        target = str(po.get("target") or "").strip()
        frame = str(po.get("frame") or "").strip()
        if not target or not frame:
            warnings.append(f"{prim}: place_on 需要 target 与 frame 两个字段")
            continue
        # 只校验 frame 存在（读局部 TRS，有缓存，很快），不再合成世界位姿
        # —— 世界位姿由下面的「USD 相对关系 + 目标当前位姿」给出，更准确。
        frame_local = _read_xform_via_service(f"{target.rstrip('/')}/{frame}")
        if frame_local is None:
            warnings.append(
                f"{prim}: 目标 frame 不存在 {target}/{frame} —— 检查配置")
            continue
        got = frame_local       # (t_local, q_local, scale) —— 只是校验存在
        # 目标 frame 的**局部** TRS（用于 offset 的旋转参考）
        pos = np.asarray(got[0], dtype=np.float64)
        quat = np.asarray(got[1], dtype=np.float64)
        # ---- 定位：用**快照的相对偏移** ----
        #
        # rel = 试管快照位姿 − 目标快照位姿
        #
        # ### 为什么不用「每轮从 USD 算」
        # USD 理论上更"干净"（资产原值），但实测有两个坑：
        #   ① get_world_pose_from_usd() 读的是**当前 stage 值**，而
        #      set_pose_usd_api() 会往 stage 写 —— 若首次读取晚于任何一次
        #      写入，rel 就被污染。实测拿到 rel=[0.0036, 0.0174]，
        #      而正确值是 [-0.0208, -0.0095]，差了 24/27mm 且**每轮固定**
        #      → 试管永远停在孔边（正是用户看到的现象）。
        #   ② 进程级 rel 缓存一旦存了错值，之后全部失效。
        #
        # ### 为什么用快照
        # 快照是「第一轮随机化之前、物体处于正确摆放状态」时记录的，
        # 天然就是这个任务的正确相对关系。实测交叉验证：
        #   快照 rel  = [-20.8, -9.5, +8.0] mm
        #   资产推出  = [-20.8, -9.5, +60.7] mm   ← x/y 完全一致，z 差沉降量
        # 所以快照的 rel 更优（且 z 已经是落稳后的正确值）。
        #
        # 前提：快照必须在「物体正确摆放」时记录 —— 菜单第一项就是它。
        rec_self = (snap.get("objects") or {}).get(prim)
        rec_target = (snap.get("objects") or {}).get(target)
        rel = None
        if (isinstance(rec_self, dict) and rec_self.get("pos")
                and isinstance(rec_target, dict) and rec_target.get("pos")):
            rel = (np.asarray(rec_self["pos"], dtype=np.float64)
                   - np.asarray(rec_target["pos"], dtype=np.float64))

        cur_target = get_pose_any(target)
        if cur_target is None:
            warnings.append(f"{prim}: 读不到 {target} 的当前位姿")
            continue

        # **优先用第一遍刚写入的值**：写 USD 后 PhysX 要一帧才同步，
        # 立刻重读是旧值 → 试管错位一个偏移量、落在孔边。
        tgt_pos = written.get(target)
        if tgt_pos is not None:
            cur_target = (np.asarray(tgt_pos, dtype=np.float64), cur_target[1])

        if rel is None:
            # 没快照：退回资产关系（x/y 可靠，z 会偏高约 53mm）
            usd_self = get_world_pose_from_usd(prim)
            usd_target = get_world_pose_from_usd(target)
            if usd_self is not None and usd_target is not None:
                rel = (np.asarray(usd_self[0], dtype=np.float64)
                       - np.asarray(usd_target[0], dtype=np.float64))
                warnings.append(
                    f"{prim}: 没有快照，用资产关系（z 会偏高约 53mm）。"
                    "建议先在物体正确摆放时跑一次 --snapshot")
            else:
                warnings.append(
                    f"{prim}: 既没有快照也读不到资产关系 —— 无法定位，跳过")
                continue

        pos = np.asarray(cur_target[0], dtype=np.float64) + rel
        _log(f"  place_on {prim.split('/')[-1]}: "
             f"{target.split('/')[-1]} {np.round(cur_target[0], 4).tolist()} "
             f"+ rel {np.round(rel, 4).tolist()} = {np.round(pos, 4).tolist()}")

        off = po.get("offset") or [0.0, 0.0, 0.0]
        try:
            off_v = np.asarray([float(x) for x in off][:3], dtype=np.float64)
        except (TypeError, ValueError):
            off_v = np.zeros(3)
        if np.any(off_v):
            # 偏移在 frame 的局部系里，需要旋转到世界系
            pos = pos + _quat_rotate(quat, off_v)
        if set_pose(prim, pos, quat):
            applied.append({
                "prim": prim,
                "place_on": f"{target}/{frame}",
                "target": [round(float(v), 4) for v in pos],
            })
            _log(f"  place_on {prim} → {target}/{frame} "
                 f"pos={np.round(pos, 4).tolist()}")

    # ---- 让物理稳定 ----
    n_settle = int(cfg.get("settle_steps", 0) or 0)
    if n_settle > 0 and settle_fn is not None:
        try:
            settle_fn(n_settle)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"settle 失败: {exc}")

    # ---- 验证：读回位姿，检查是否真的落到了目标附近 ----
    # set_world_poses 对 dynamic 刚体可能被物理改写（历史踩过），
    # 所以这里必须读回确认，不能假定成功。
    #
    # 但**读回偏差不一定是失败**：细长/悬空的物体（如插在孔里的试管）在
    # settle 期间会靠物理沉降几厘米 —— 那是正确行为，不是错误。
    # 所以按「严重程度」分级：
    #   - 偏差 > 3× 容差 → 记 warn（可能真的被撞飞了）
    #   - 偏差 > 容差但 < 3× → 记 note（物理沉降，可接受）
    tol = float(cfg.get("verify_tol_m", 0.05) or 0.05)
    for a in applied:
        want = a.get("target")
        if not want:
            continue
        got = get_pose_any(a["prim"])
        if got is None:
            a["verify"] = "read_failed"
            continue
        err = float(np.linalg.norm(np.asarray(got[0]) - np.asarray(want)))
        a["verify_err_m"] = round(err, 4)
        if err > tol * 3:
            a["verify"] = "bad"
            warnings.append(
                f"{a['prim']}: 实际位姿与目标差 {err*1000:.0f}mm"
                f"（> {tol*3*1000:.0f}mm = 3×容差）—— 可能被撞飞/穿模")
        elif err > tol:
            a["verify"] = "settled"
            _log(f"  note {a['prim']}: 物理沉降 {err*1000:.0f}mm（容差 {tol*1000:.0f}mm 内属正常）")
        else:
            a["verify"] = "ok"

    # 只有「严重偏差」才算失败；沉降/读回失败不影响本轮可用性
    ok = not any(a.get("verify") == "bad" for a in applied)
    _log(f"随机化完成: {len(applied)} 个物体, seed={seed}, "
         f"warnings={len(warnings)}")
    for w in warnings:
        _log(f"  WARN {w}")
    return {"ok": ok, "applied": applied, "warnings": warnings, "seed": seed}
