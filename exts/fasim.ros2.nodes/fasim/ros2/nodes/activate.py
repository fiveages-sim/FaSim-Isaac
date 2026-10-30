"""Activate controllers on Isaac's in-process controller_manager via ROS services."""

from __future__ import annotations

import time
from typing import Iterable

import carb


def _ns(namespace: str) -> str:
    return (namespace or "").strip().strip("/")


def _cm_name(namespace: str) -> str:
    # Absolute service path under the robot namespace (same prefix model as topics).
    ns = _ns(namespace)
    return f"/{ns}/controller_manager" if ns else "/controller_manager"


def _call(node, executor, srv_type, srv_name: str, request, timeout_sec: float = 60.0):
    client = node.create_client(srv_type, srv_name)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not client.service_is_ready():
        executor.spin_once(timeout_sec=0.05)
    if not client.service_is_ready():
        node.destroy_client(client)
        raise RuntimeError(f"service {srv_name!r} not available")
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not future.done():
        executor.spin_once(timeout_sec=0.05)
    node.destroy_client(client)
    if not future.done():
        raise RuntimeError(f"service {srv_name!r} did not respond")
    return future.result()


def activate_controllers(controllers: Iterable[str], namespace: str = "") -> None:
    """Load, configure, then switch-activate the given controllers.

    Uses a private rclpy Context so Isaac's default executor can keep spinning.
    """
    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController
    from rclpy.executors import SingleThreadedExecutor

    names = [str(name) for name in controllers if str(name).strip()]
    if not names:
        raise RuntimeError("activate list is empty")

    ns = _ns(namespace)
    context = rclpy.Context()
    rclpy.init(context=context, args=None)
    # Same short name; uniqueness via namespace → /<ns>/fasim_ros2_control_launch
    node = rclpy.create_node(
        "fasim_ros2_control_launch",
        context=context,
        namespace=ns or "",
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    cm = _cm_name(namespace)
    try:
        list_client = node.create_client(ListControllers, f"{cm}/list_controllers")
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not list_client.service_is_ready():
            executor.spin_once(timeout_sec=0.05)
        if not list_client.service_is_ready():
            raise RuntimeError(f"controller_manager services for {cm!r} not available")
        node.destroy_client(list_client)

        for name in names:
            res = _call(
                node,
                executor,
                LoadController,
                f"{cm}/load_controller",
                LoadController.Request(name=name),
            )
            if not res.ok:
                raise RuntimeError(f"load_controller({name}) failed")
            res = _call(
                node,
                executor,
                ConfigureController,
                f"{cm}/configure_controller",
                ConfigureController.Request(name=name),
            )
            if not res.ok:
                raise RuntimeError(f"configure_controller({name}) failed")
            carb.log_info(f"fasim.ros2: configured {name}")

        # Activate broadcasters first so command controllers can claim interfaces
        # without a single STRICT switch stalling the whole set.
        ordered: list[str] = []
        for name in names:
            if name.endswith("_broadcaster") or name == "joint_state_broadcaster":
                ordered.append(name)
        for name in names:
            if name not in ordered:
                ordered.append(name)

        # Give physics post-step a moment to tick the CM after configure.
        settle = time.monotonic() + 1.0
        while time.monotonic() < settle:
            executor.spin_once(timeout_sec=0.05)

        for name in ordered:
            req = SwitchController.Request(
                activate_controllers=[name],
                strictness=SwitchController.Request.STRICT,
            )
            if hasattr(req, "timeout"):
                req.timeout = Duration(sec=60, nanosec=0)
            res = _call(
                node,
                executor,
                SwitchController,
                f"{cm}/switch_controller",
                req,
                timeout_sec=90.0,
            )
            if not res.ok:
                raise RuntimeError(f"switch_controller(activate=[{name}]) failed")
            carb.log_info(f"fasim.ros2: activated {name}")

    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        if context.ok():
            rclpy.shutdown(context=context)
