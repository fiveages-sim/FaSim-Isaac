#!/usr/bin/env python3
"""Merge quick-start ros2_control YAML for Isaac's in-process ControllerManager.

Runs under the host ROS Python (source ros2_ws first). Stdout is a single JSON
object with yaml_path and activate; diagnostics go to stderr.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import sys


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True, help="Robot name without _description")
    parser.add_argument("--type", default="", dest="robot_type", help="ros2_control type key")
    parser.add_argument(
        "--left-type",
        default="",
        dest="left_type",
        help="Per-side EEF type (robot_common_launch left_type); from Isaac EE_Left",
    )
    parser.add_argument(
        "--right-type",
        default="",
        dest="right_type",
        help="Per-side EEF type (robot_common_launch right_type); from Isaac EE_Right",
    )
    parser.add_argument(
        "--mode",
        default="demo",
        choices=("demo", "split_body", "full_body"),
        help="Which controllers to activate",
    )
    parser.add_argument("--profile", default="", help="Optional robot profile YAML path")
    parser.add_argument(
        "--enable-gripper",
        default="true",
        choices=("true", "false", "1", "0", "yes", "no"),
        help="Include hand/gripper controllers when present",
    )
    parser.add_argument(
        "--enable-body",
        default="true",
        choices=("true", "false", "1", "0", "yes", "no"),
        help="Include body/head controllers in split_body mode",
    )
    parser.add_argument(
        "--namespace",
        default="",
        help="ROS namespace for controller_manager (rewrites YAML key to /ns/controller_manager)",
    )
    parser.add_argument(
        "--update-rate",
        type=int,
        default=0,
        help=(
            "Override controller_manager and controller update_rate (Hz). "
            "Use Isaac physics rate (1/dt). 0 keeps YAML values."
        ),
    )
    parser.add_argument(
        "--enable-arms-target-manager",
        default="true",
        choices=("true", "false", "1", "0", "yes", "no"),
        help="Emit merged params YAML for arms_target_manager_node (VR / marker teleop).",
    )
    parser.add_argument(
        "--arms",
        default="",
        help="robot_common_launch arms xacro key (e.g. piper, h, l, x). Empty: xacro default.",
    )
    return parser.parse_args(argv)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("true", "1", "yes")


def _apply_update_rate(config: dict, rate: int) -> None:
    """Force CM + any controller that declares update_rate to ``rate`` Hz."""
    rate = int(rate)
    if rate <= 0:
        return
    for key, section in config.items():
        if not isinstance(section, dict):
            continue
        params = section.setdefault("ros__parameters", {})
        if not isinstance(params, dict):
            continue
        short = str(key).rstrip("/").split("/")[-1]
        if short == "controller_manager" or "update_rate" in params:
            params["update_rate"] = rate
    print(f"[INFO] Overriding update_rate -> {rate} Hz (Isaac physics aligned)", file=sys.stderr)


def _deep_set_params(config: dict, controller: str, params: dict) -> None:
    section = config.setdefault(controller, {})
    ros_params = section.setdefault("ros__parameters", {})
    ros_params.update(params)


def _resolve_planning_robot_name(
    config: dict,
    controller_key: str,
    robot_name: str,
    robot_variant: str,
    robot_arms: str,
) -> str:
    from robot_common_launch.common.launch_arg_utils import (
        planning_robot_for_arm_family,
        robot_xacro_declares_arg,
    )

    planning_robot_name = robot_name
    try:
        config_robot_name = (
            config.get(controller_key, {}).get("ros__parameters", {}).get("robot_name")
        )
        if config_robot_name:
            planning_robot_name = config_robot_name
    except Exception:
        pass

    if controller_key == "ocs2_arm_controller" and not robot_xacro_declares_arg(
        robot_name, "topology"
    ):
        family_key = str(robot_arms or robot_variant or "").strip()
        variant_robot_name = planning_robot_for_arm_family(family_key)
        if variant_robot_name and variant_robot_name != planning_robot_name:
            print(
                f"[INFO] Arm family '{family_key}' selects planning robot "
                f"'{variant_robot_name}' (config requested: {planning_robot_name})",
                file=sys.stderr,
            )
            planning_robot_name = variant_robot_name
    return planning_robot_name


def _ft_sides_enabled(launch_configurations: dict, profile_path: str) -> tuple[bool, bool]:
    import yaml

    ft: dict[str, str] = {}
    for side in ("left", "right"):
        key = f"{side}_ft"
        val = str(launch_configurations.get(f"hardware_{key}", "") or "").strip()
        if val:
            ft[key] = val

    if profile_path and os.path.isfile(profile_path):
        with open(profile_path, encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        hw = raw.get("hardware")
        if isinstance(hw, dict):
            for side in ("left", "right"):
                key = f"{side}_ft"
                if key not in ft and hw.get(key) is not None:
                    ft[key] = str(hw[key]).strip()

    def enabled(key: str) -> bool:
        val = str(ft.get(key, "") or "").strip().lower()
        return bool(val) and val != "none"

    return enabled("left_ft"), enabled("right_ft")


def _unwrap_ros_params(data: dict) -> dict:
    """Flatten ``/**`` / ``ros__parameters`` YAML shapes into a flat param dict."""
    if not isinstance(data, dict):
        return {}
    if "/**" in data and isinstance(data["/**"], dict):
        nested = data["/**"]
        params = nested.get("ros__parameters")
        return dict(params) if isinstance(params, dict) else dict(nested)
    if "ros__parameters" in data and isinstance(data["ros__parameters"], dict):
        return dict(data["ros__parameters"])
    for value in data.values():
        if isinstance(value, dict) and isinstance(value.get("ros__parameters"), dict):
            return dict(value["ros__parameters"])
    return dict(data)


def _write_arms_target_manager_params(param_sources: list, extras: dict) -> str:
    """Merge prepare_arms_target_manager_parameters sources + extras into one params file."""
    import tempfile

    import yaml

    merged: dict = {}
    for src in param_sources:
        if isinstance(src, str):
            if not os.path.isfile(src):
                continue
            with open(src, encoding="utf-8") as handle:
                merged.update(_unwrap_ros_params(yaml.safe_load(handle) or {}))
        elif isinstance(src, dict):
            merged.update(src)
    merged.update(extras)
    payload = {"/**": {"ros__parameters": merged}}
    fd, path = tempfile.mkstemp(prefix="arms_target_manager_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
        yaml.safe_dump(payload, handle, default_flow_style=False, allow_unicode=True)
    return path


def _prepare_arms_target_manager_yaml(
    *,
    config: dict,
    mode: str,
    planning_robot_name: str,
    hand_controllers: list[str],
) -> str | None:
    """Mirror demo/split_body/full_body.launch ATM Node parameters → temp YAML path."""
    from robot_common_launch import (
        extract_info_file_name_from_config,
        get_robot_package_path,
        parse_task_info,
        prepare_arms_target_manager_parameters,
    )

    info_file_name = extract_info_file_name_from_config(config, launch_mode=mode)
    robot_pkg_path = get_robot_package_path(planning_robot_name)
    if not robot_pkg_path:
        print(
            f"[WARN] ATM: no description package for planning robot "
            f"'{planning_robot_name}'",
            file=sys.stderr,
        )
        return None

    task_file_path = os.path.join(
        robot_pkg_path, "config", "ocs2", f"{info_file_name}.info"
    )
    if not os.path.isfile(task_file_path):
        print(f"[WARN] ATM: task file missing: {task_file_path}", file=sys.stderr)
        return None
    print(f"[INFO] ATM task file: {task_file_path}", file=sys.stderr)

    marker_fixed_frame = None
    wbc_available = False
    # Wall clock: ATM is a host process for VR/markers; do not gate on Isaac /clock.
    extras: dict = {"use_sim_time": False}
    if mode == "full_body":
        cm_params = (
            config.get("controller_manager", {}).get("ros__parameters", {})
            if isinstance(config.get("controller_manager"), dict)
            else {}
        )
        wbc_cfg = cm_params.get("ocs2_wbc_controller", {})
        wbc_type = wbc_cfg.get("type", "") if isinstance(wbc_cfg, dict) else ""
        wbc_available = wbc_type == "ocs2_wbc_controller/Ocs2WbcController"
        if wbc_available:
            # WBC EE targets are world-framed. Force both marker + control base so a
            # wrong .info (e.g. arms-only task.info with baseFrame=torso_link) cannot
            # leave ATM publishing chassis-offset continuous targets (G1D/W2 mobile).
            marker_fixed_frame = "world"
            extras["control_base_frame"] = "world"
        else:
            _dual, control_base_frame, auto_marker = parse_task_info(task_file_path)
            marker_fixed_frame = auto_marker or control_base_frame
        extras["enable_movej_cartesian_markers"] = not wbc_available
        extras["enable_wbc_head_tracking_marker"] = wbc_available
        print(
            f"[INFO] ATM full_body: wbc_available={wbc_available} "
            f"marker_fixed_frame={marker_fixed_frame!r} "
            f"control_base_frame_override={extras.get('control_base_frame')!r}",
            file=sys.stderr,
        )

    arms_params = prepare_arms_target_manager_parameters(
        task_file_path=task_file_path,
        config_file_path=None,
        hand_controllers=hand_controllers or None,
        marker_fixed_frame=marker_fixed_frame,
    )
    if not arms_params:
        print("[WARN] ATM: prepare_arms_target_manager_parameters returned empty", file=sys.stderr)
        return None

    path = _write_arms_target_manager_params(arms_params, extras)
    print(f"[INFO] ATM params YAML: {path}", file=sys.stderr)
    return path


def emit(args: argparse.Namespace) -> dict:
    from robot_common_launch import (
        build_planning_urdf_launch_params,
        detect_controllers,
        get_ros2_control_robot_description,
        load_robot_config,
        load_robot_profile,
        resolve_control_patch,
        resolve_control_sides,
        resolve_robot_arms,
        resolve_robot_variant,
        write_temp_ros2_control_yaml,
    )
    from robot_common_launch.common.launch_arg_utils import (
        planning_robot_for_arm_family,
        robot_xacro_declares_arg,
    )

    robot_name = args.robot.strip()
    robot_type = (args.robot_type or "").strip()
    left_type = (getattr(args, "left_type", None) or "").strip()
    right_type = (getattr(args, "right_type", None) or "").strip()
    mode = args.mode
    profile_path = (args.profile or "").strip()
    enable_gripper = _as_bool(args.enable_gripper)
    enable_body = _as_bool(args.enable_body)
    hardware = "isaac"

    arms = (getattr(args, "arms", None) or "").strip()
    launch_configurations: dict[str, str] = {
        "robot": robot_name,
        "type": robot_type,
        "hardware": hardware,
    }
    if left_type:
        launch_configurations["left_type"] = left_type
    if right_type:
        launch_configurations["right_type"] = right_type
    if arms:
        launch_configurations["arms"] = arms
    if profile_path:
        launch_configurations["robot_profile"] = profile_path

    profile = load_robot_profile(profile_path) if profile_path else {}
    control_left, control_right = resolve_control_sides(launch_configurations, profile)
    control_patch = resolve_control_patch(profile)
    robot_arms = resolve_robot_arms(launch_configurations, profile, robot_name=robot_name)
    if robot_arms and not str(launch_configurations.get("arms") or "").strip():
        launch_configurations["arms"] = robot_arms
        print(f"[INFO] Using robot xacro default arms '{robot_arms}'", file=sys.stderr)
    robot_variant = resolve_robot_variant(
        launch_configurations, profile, robot_name=robot_name
    )

    arm_family = robot_arms or robot_variant
    variant_arm_robot = planning_robot_for_arm_family(arm_family)
    if variant_arm_robot and not robot_xacro_declares_arg(robot_name, "topology"):
        control_patch = copy.deepcopy(control_patch)
        arm_params = control_patch.setdefault("ocs2_arm_controller", {}).setdefault(
            "ros__parameters", {}
        )
        arm_params["robot_name"] = variant_arm_robot

    config, _path, _meta = load_robot_config(
        robot_name,
        "ros2_control",
        robot_type,
        control_left=control_left,
        control_right=control_right,
        control_patch=control_patch,
        robot_variant=robot_variant,
        hardware=hardware,
    )
    if not config:
        raise RuntimeError(f"No ros2_control config for robot '{robot_name}'")

    if mode in ("demo", "split_body"):
        main_controller = "ocs2_arm_controller"
        planning_scope = "arms"
    else:
        main_controller = "ocs2_wbc_controller"
        planning_scope = "full"

    planning_robot_name = _resolve_planning_robot_name(
        config,
        main_controller,
        robot_name,
        robot_variant,
        robot_arms,
    )
    planning_urdf_params = build_planning_urdf_launch_params(
        planning_robot_name,
        launch_configurations,
        hardware,
        profile_path or None,
        planning_scope=planning_scope,
    )
    plan_path = (planning_urdf_params.get("planning_urdf_path") or "").strip()
    if planning_urdf_params.get("planning_urdf_variant") != "xacro" or not plan_path:
        raise RuntimeError(
            f"OCS2 requires xacro planning URDF for '{planning_robot_name}' "
            f"(scope={planning_scope}). path={plan_path!r}"
        )
    if not os.path.isfile(plan_path):
        raise RuntimeError(f"Planning URDF missing: {plan_path}")

    _deep_set_params(config, main_controller, planning_urdf_params)
    print(f"[INFO] OCS2 planning URDF: {plan_path}", file=sys.stderr)

    robot_description = get_ros2_control_robot_description(
        robot_name,
        robot_type=robot_type,
        hardware=hardware,
        launch_configurations=launch_configurations,
        robot_profile=profile_path or None,
    )
    if not robot_description:
        raise RuntimeError(
            f"get_ros2_control_robot_description returned empty for '{robot_name}' "
            f"(type={robot_type!r}, hardware={hardware!r})"
        )

    activate = ["joint_state_broadcaster", main_controller]
    hand_controllers: list[str] = []

    # Mirror robot_common_launch setup_hand_controllers / detect_and_spawn_controllers:
    # patterns ["hand", "gripper"] against merged YAML + xacro robot_description.
    if enable_gripper:
        hands = detect_controllers(
            robot_name,
            robot_type,
            ["hand", "gripper"],
            robot_description=robot_description,
            control_left=control_left,
            control_right=control_right,
            control_patch=control_patch,
            ros2_control_config=config,
        )
        # Skip motion_control_handle (RViz interactive marker, not a gripper).
        hands = [c for c in hands if c.get("name") != "motion_control_handle"]
        hand_controllers = [c["name"] for c in hands]
        for controller in hands:
            print(
                f"[INFO] Hand/gripper controller: {controller['name']} "
                f"({controller.get('type', '')})",
                file=sys.stderr,
            )
        if not hand_controllers:
            print(
                "[WARN] enable_gripper=true but no hand/gripper controller detected "
                f"(robot={robot_name!r}, type={robot_type!r})",
                file=sys.stderr,
            )
        activate.extend(hand_controllers)

    if mode == "split_body" and enable_body:
        body = detect_controllers(
            robot_name,
            robot_type,
            ["body", "head"],
            control_left=control_left,
            control_right=control_right,
            control_patch=control_patch,
            ros2_control_config=config,
        )
        activate.extend(c["name"] for c in body)
    elif mode == "full_body":
        cm_params = config.get("controller_manager", {}).get("ros__parameters", {})
        wbc_cfg = cm_params.get("ocs2_wbc_controller", {})
        wbc_type = wbc_cfg.get("type", "") if isinstance(wbc_cfg, dict) else ""
        if wbc_type == "ocs2_arm_controller/Ocs2ArmController":
            head = detect_controllers(
                robot_name,
                robot_type,
                ["head"],
                control_left=control_left,
                control_right=control_right,
                control_patch=control_patch,
                ros2_control_config=config,
            )
            activate.extend(c["name"] for c in head)

    left_ft, right_ft = _ft_sides_enabled(launch_configurations, profile_path)
    if left_ft or right_ft:
        fts = detect_controllers(
            robot_name,
            robot_type,
            ["ft_broadcaster"],
            ros2_control_config=config,
        )
        for controller in fts:
            name = controller["name"]
            if name.startswith("left_") and left_ft:
                activate.append(name)
            elif name.startswith("right_") and right_ft:
                activate.append(name)

    # Preserve order while dropping duplicates.
    seen: set[str] = set()
    unique_activate: list[str] = []
    for name in activate:
        if name in seen:
            continue
        seen.add(name)
        unique_activate.append(name)

    # When gripper is disabled, drop gripper/hand controllers from the YAML so
    # the param file does not advertise joints absent from a no-EE USD synth.
    if not enable_gripper:
        cm_params = (
            config.get("controller_manager", {}).get("ros__parameters", {})
            if isinstance(config.get("controller_manager"), dict)
            else {}
        )
        for key in list(config.keys()):
            if key == "controller_manager":
                continue
            short = key.split("/")[-1]
            if "gripper" in short.lower() or short.endswith("_hand_controller"):
                config.pop(key, None)
                if isinstance(cm_params, dict):
                    cm_params.pop(short, None)
                print(f"[INFO] Stripped controller '{short}' (enable_gripper=false)", file=sys.stderr)

    # ATM params before CM namespace rewrite (extract_info_file_name needs bare keys).
    atm_params_path = None
    if _as_bool(getattr(args, "enable_arms_target_manager", "true")):
        try:
            atm_params_path = _prepare_arms_target_manager_yaml(
                config=config,
                mode=mode,
                planning_robot_name=planning_robot_name,
                hand_controllers=hand_controllers,
            )
        except Exception as exc:
            print(f"[WARN] ATM params failed: {exc}", file=sys.stderr)

    # Isaac's in-process CM expects /<ns>/controller_manager when namespace is set.
    # Controller parameter blocks must also be namespaced so ocs2_arm_controller
    # finds planning_urdf_path under /<ns>/<controller>.
    update_rate = int(getattr(args, "update_rate", 0) or 0)
    if update_rate > 0:
        _apply_update_rate(config, update_rate)

    ns = (args.namespace or "").strip().strip("/")
    if ns:
        if "controller_manager" in config:
            config[f"/{ns}/controller_manager"] = config.pop("controller_manager")
        for key in list(config.keys()):
            if key.startswith("/") or key == f"/{ns}/controller_manager":
                continue
            config[f"/{ns}/{key}"] = config.pop(key)

    yaml_path = write_temp_ros2_control_yaml(config, quiet=False)

    # Persist xacro robot_description for the OG node to latch on /robot_description
    # (same source as ros2_control_node + robot_state_publisher in quick-start).
    import tempfile

    desc_fd, desc_path = tempfile.mkstemp(prefix="robot_description_", suffix=".urdf")
    with os.fdopen(desc_fd, "w", encoding="utf-8") as handle:
        handle.write(robot_description)
    print(
        f"[INFO] robot_description xacro written: {desc_path} ({len(robot_description)} bytes)",
        file=sys.stderr,
    )

    return {
        "yaml_path": yaml_path,
        "activate": unique_activate,
        "hand_controllers": hand_controllers,
        "namespace": ns,
        "update_rate": update_rate or None,
        "robot_description_path": desc_path,
        "arms_target_manager_params": atm_params_path,
    }


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        payload = emit(args)
    except Exception as exc:
        print(f"[ERROR] emit_control failed: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(payload, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
