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


def set_pose(prim_path: str, pos: np.ndarray, quat: np.ndarray,
             *, zero_velocity: bool = True) -> bool:
    """写刚体世界位姿（优先 Isaac 内 RigidPrim，失败退回 ROS 服务）。

    ``zero_velocity``：**必须清零速度**（默认开）。
    只设位姿不清速度时，物体保留着「上一轮被机器人拿着移动」的线速度/角速度，
    下一帧物理一跑就被甩出去 —— 实测试管就是这么崩飞的。
    """
    # 先试 Isaac 内的高效路径
    try:
        rp = _rigid(prim_path)
        rp.set_world_poses(
            positions=np.asarray(pos, dtype=np.float64).reshape(1, 3),
            orientations=np.asarray(quat, dtype=np.float64).reshape(1, 4),
        )
        if zero_velocity:
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
        return True
    except Exception:
        # 外部进程（无 isaacsim）→ 走 ROS 服务
        return set_pose_service(prim_path, pos, quat, zero_velocity=zero_velocity)


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


def _read_xform_via_service(prim_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """通过 ``/get_prim_attribute`` 服务读一个 prim 的**局部** TRS。

    为什么走服务而不是 XformPrim：newton kit 关了 PhysX→USD 同步
    （DEBUG_LOG #1），XformPrim 在本环境读到的是 stale/无效值
    （实测 ``读不到 .../hole_4 的世界位姿``）。而该服务由 Isaac 内的
    ROS2ServicePrim 节点直接查 USD stage，**实测可用**：

        /World/room/scene/test_tube_rack_01/hole_4
          xformOp:translate = [0.021, -0.01, 0.11]
          xformOp:orient    = [1,0,0,0]

    Returns:
        ``(translate(3), orient_wxyz(4), scale(3))``；读不到返回 None。
    """
    try:
        import json as _json

        from isaac_ros2_messages.srv import GetPrimAttribute

        def _ask(attr: str):
            req = GetPrimAttribute.Request()
            req.path = prim_path
            req.attribute = attr
            r = _call_service(GetPrimAttribute, "/get_prim_attribute", req, timeout=3.0)
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
        return tv, ov, sv
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读 {prim_path} 的 xform 失败: {exc}")
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
    """优先 Isaac 内 RigidPrim（快），失败退回 ROS 服务（外部进程可用）。"""
    got = get_pose(prim_path)
    if got is not None:
        return got
    return get_pose_service(prim_path)


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
        got = get_subframe_world_pose(target, frame)
        if got is None:
            warnings.append(
                f"{prim}: 读不到 {target}/{frame} 的世界位姿 —— 检查 frame 名是否存在")
            continue
        pos, quat = got
        # ---- 用「快照时的相对偏移」定位（而不是 align 子 frame）----
        #
        # 为什么不用 align 子 frame：实测 test_tube/tube_bottom 的局部
        # translate 是 [0,0,0]（与管原点重合），拿它对齐毫无作用。
        #
        # 正确公式（纯平移，因为随机化只动 xy 不平移旋转）：
        #
        #   rel          = 试管快照位姿 − 目标快照位姿   ← 正确摆放时的相对位置
        #   试管目标位姿 = 目标当前位姿 + rel            ← 目标搬到哪就跟到哪
        #
        # 这个相对偏移天然包含「管底比孔口低一个管长」，不需要知道
        # hole_4 的语义。而且**只用快照**（不用物体当前位置 —— 它可能正在飞）。
        rec_self = (snap.get("objects") or {}).get(prim)
        rec_target = (snap.get("objects") or {}).get(target)
        if (isinstance(rec_self, dict) and rec_self.get("pos")
                and isinstance(rec_target, dict) and rec_target.get("pos")):
            base_self = np.asarray(rec_self["pos"], dtype=np.float64)
            base_target = np.asarray(rec_target["pos"], dtype=np.float64)
            cur_target = get_pose_any(target)
            if cur_target is None:
                warnings.append(f"{prim}: 读不到 {target} 的当前位姿")
                continue
            rel = base_self - base_target
            pos = np.asarray(cur_target[0], dtype=np.float64) + rel
            _log(f"  place_on {prim}: 快照相对偏移 {np.round(rel, 4).tolist()} "
                 f"+ 目标当前 {np.round(cur_target[0], 4).tolist()} "
                 f"→ 目标 {np.round(pos, 4).tolist()}")
        else:
            warnings.append(
                f"{prim}: 快照缺 {prim} 或 {target} 的位姿 —— "
                "只能按 frame 绝对位置放置（会偏高半个管长）。"
                "请在物体正确摆放时重新「记录初始位姿」")
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
