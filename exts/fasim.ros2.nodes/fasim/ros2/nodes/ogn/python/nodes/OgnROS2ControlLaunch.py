"""OmniGraph node: merge quick-start YAML, setup in-process CM, activate controllers."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading

import carb
import omni.graph.core as og
import omni.timeline
from isaacsim.ros2.control import Ros2ControlManager

from fasim.ros2.nodes.activate import activate_controllers
from fasim.ros2.nodes.library_path import ensure_isaac_ros2_library_path
from fasim.ros2.nodes.publish_description import RobotDescriptionPublisher

_EMIT_SCRIPT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "emit_control.py")
)


def _read_variant_selection(prim_path: str, variant_set: str) -> str:
    """Return the selection for ``variant_set`` on prim_path, or empty."""
    try:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return ""
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return ""
        vsets = prim.GetVariantSets()
        if not vsets.HasVariantSet(variant_set):
            return ""
        return str(vsets.GetVariantSet(variant_set).GetVariantSelection() or "")
    except Exception as exc:
        carb.log_warn(
            f"ROS2ControlLaunch: read {variant_set!r} variant failed: {exc}"
        )
        return ""


def _ee_type_key(selection: str) -> str:
    """Isaac EE variant name is the ROS type key; ``none``/empty → no EE."""
    name = (selection or "").strip()
    if not name or name.lower() == "none":
        return ""
    return name


def _resolve_ee_launch_types(
    prim_path: str, type_input: str, enable_gripper_input: bool
) -> tuple[str, str, str, bool]:
    """Read USD EE variants as robot_common_launch type / left_type / right_type.

    Isaac variant names must match ROS keys (CR5: AG2F90-C / AG2F120S;
    Zerith: gripper / revo1 / revo2). No remap table.

    Returns (type, left_type, right_type, enable_gripper).
    """
    type_input = (type_input or "").strip()
    ee = _ee_type_key(_read_variant_selection(prim_path, "EE"))
    left = _ee_type_key(_read_variant_selection(prim_path, "EE_Left"))
    right = _ee_type_key(_read_variant_selection(prim_path, "EE_Right"))

    if type_input:
        carb.log_info(
            f"ROS2ControlLaunch: type={type_input!r} enableGripper={bool(enable_gripper_input)} "
            f"(EE={ee!r}, EE_Left={left!r}, EE_Right={right!r})"
        )
        return type_input, "", "", bool(enable_gripper_input)

    # Single-arm ``EE`` → ``type``.
    if ee and not left and not right:
        carb.log_info(f"ROS2ControlLaunch: EE={ee!r} → type={ee!r} enableGripper=True")
        return ee, "", "", True

    # Dual ``EE_Left`` / ``EE_Right`` → left_type / right_type (and type when symmetric).
    if left or right:
        enable = True
        # Symmetric sides: also set type= so xacros that only declare ``type`` work.
        robot_type = left if left and left == right else ""
        carb.log_info(
            f"ROS2ControlLaunch: EE_Left={left!r} EE_Right={right!r} "
            f"→ type={robot_type!r} left_type={left!r} right_type={right!r} "
            f"enableGripper={enable}"
        )
        return robot_type, left, right, enable

    return "", "", "", False


def _find_usd_joint_prim(stage, target_path: str, joint_name: str):
    """Find a Revolute/Prismatic joint prim named ``joint_name`` under target."""
    from pxr import Usd, UsdPhysics

    target = stage.GetPrimAtPath(target_path)
    if not target or not target.IsValid():
        return None
    for prim in Usd.PrimRange(target):
        if prim.GetName() != joint_name:
            continue
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            return prim
    return None


def _is_ee_joint_name(name: str) -> bool:
    """True for AdaptiveGripper / Revo / finger joint names (not arm/chassis)."""
    name_l = (name or "").lower()
    return (
        "gripper" in name_l
        or "finger" in name_l
        or name_l.endswith("_hand_joint")
        or name_l == "hand_joint"
        # Revo / dexhand: left_hand_index_joint, right_hand_thumb_joint1, …
        or "_hand_" in name_l
    )


def _iter_usd_ee_joint_prims(stage, target_path: str):
    """Yield Revolute/Prismatic joint prims that look like EE gripper/hand DOFs.

    Matches robot_common_launch hand/gripper detection: joints under an EE/Hand
    mount, or named like ``*gripper*`` / ``*_hand_*``.

    Note: IsaacRobotAPI ``isaac:physics:robotJoints`` on wheeled humanoids lists
    *every* articulation joint (wheels, body, arms). Only non-joint *mount*
    targets from that rel (Revo2 / Gripper Xforms prepended by EE variants) are
    treated as EE scopes — individual joint targets are ignored.
    """
    from pxr import Usd, UsdPhysics

    target = stage.GetPrimAtPath(target_path)
    if not target or not target.IsValid():
        return

    scopes = [target]
    ee_mount_roots: list[str] = []
    # Only EE/hand mounts from isaac:physics:robotJoints. Wheeled humanoids also
    # list Chassis/Lift/Body/Arms there — those must NOT become EE scan scopes
    # (otherwise every arm/wheel joint is mis-tagged as a hand DOF).
    _ee_mount_markers = (
        "/EE/",
        "/Hand/",
        "/hand/",
        "/DexHand/",
        "/Dex1/",
        "/Dex3/",
        "/gripper/",
        "/Gripper/",
        "/Revo1/",
        "/Revo2/",
        "/RubberHand/",
        "/Zerith_H1_Gripper/",
    )
    try:
        rel = target.GetRelationship("isaac:physics:robotJoints")
        if rel:
            for path in rel.GetTargets():
                prim = stage.GetPrimAtPath(path)
                if not prim or not prim.IsValid():
                    continue
                # Full-body IsaacRobotAPI lists each joint; skip those.
                if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
                    continue
                if prim.IsA(UsdPhysics.Joint):
                    continue
                path_s = str(prim.GetPath())
                if not any(marker in path_s for marker in _ee_mount_markers):
                    continue
                scopes.append(prim)
                ee_mount_roots.append(path_s)
    except Exception:
        pass

    seen: set[str] = set()
    for scope in scopes:
        for prim in Usd.PrimRange(scope):
            if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
                continue
            path = str(prim.GetPath())
            if path in seen:
                continue
            name = prim.GetName()
            under_ee = any(
                token in path
                for token in (
                    "/EE/",
                    "/Hand/",
                    "/hand/",
                    "/DexHand/",
                    "/gripper/",
                    "/Revo1/",
                    "/Revo2/",
                    "/Zerith_H1_Gripper/",
                )
            )
            under_ee_mount = any(path.startswith(root) for root in ee_mount_roots)
            if under_ee or under_ee_mount or _is_ee_joint_name(name):
                seen.add(path)
                yield prim


def _usd_joint_has_drive(joint_prim) -> bool:
    """True if the joint has a PhysicsDriveAPI (commandable)."""
    from pxr import UsdPhysics

    token = "angular" if joint_prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
    return bool(joint_prim.HasAPI(UsdPhysics.DriveAPI, token))


def _usd_joint_parent_child_names(joint_prim) -> tuple[str, str]:
    """Return (parent_link, child_link) basenames from physics:body0/body1."""
    from pxr import UsdPhysics

    joint = UsdPhysics.Joint(joint_prim)
    body0 = joint.GetBody0Rel().GetTargets()
    body1 = joint.GetBody1Rel().GetTargets()
    parent = body0[0].name if body0 else f"{joint_prim.GetName()}_parent"
    child = body1[0].name if body1 else f"{joint_prim.GetName()}_child"
    return str(parent), str(child)


def _usd_revolute_limits(joint_prim) -> tuple[float, float]:
    """Return (lower, upper) rad for a revolute joint (USD stores degrees).

    Non-finite USD limits (±inf for free wheels/casters) fall back to ±π so
    injected kinematic joints stay urdfdom-safe (sanitizer may later promote
    fully-unlimited revolute joints to continuous).
    """
    import math

    from pxr import UsdPhysics

    lower_deg = -180.0
    upper_deg = 180.0
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        rj = UsdPhysics.RevoluteJoint(joint_prim)
        lo = rj.GetLowerLimitAttr().Get()
        up = rj.GetUpperLimitAttr().Get()
        if lo is not None and math.isfinite(float(lo)):
            lower_deg = float(lo)
        if up is not None and math.isfinite(float(up)):
            upper_deg = float(up)
    return math.radians(lower_deg), math.radians(upper_deg)


def _inject_kinematic_joint(
    root,
    *,
    name: str,
    joint_prim,
    joint_names: set[str],
    link_names: set[str],
    default_parent: str | None,
) -> bool:
    """Append a minimal kinematic joint+links for ``name``. Returns True if added."""
    from xml.etree import ElementTree as ET

    from pxr import UsdPhysics

    if name in joint_names:
        return False

    usd_parent, usd_child = _usd_joint_parent_child_names(joint_prim)
    # Prefer USD body0 when that link already exists in the synth tree. Falling
    # back to default_parent first (old behaviour) hung EE DOFs off waist_pitch
    # on assembled mobile manipulators like G1D.
    if usd_parent and usd_parent in link_names:
        parent = usd_parent
    else:
        parent = default_parent or usd_parent or "base_link"
    child = usd_child or f"{name}_link"
    if child in link_names:
        child = f"{name}_link"
    for link in (parent, child):
        if link and link not in link_names:
            ET.SubElement(root, "link", {"name": link})
            link_names.add(link)

    jtype = "revolute" if joint_prim.IsA(UsdPhysics.RevoluteJoint) else "prismatic"
    j_el = ET.SubElement(root, "joint", {"name": name, "type": jtype})
    ET.SubElement(j_el, "parent", {"link": parent})
    ET.SubElement(j_el, "child", {"link": child})
    ET.SubElement(j_el, "axis", {"xyz": "1 0 0"})
    limit = ET.SubElement(j_el, "limit")
    if jtype == "revolute":
        lo, up = _usd_revolute_limits(joint_prim)
        limit.set("lower", str(lo))
        limit.set("upper", str(up))
    else:
        limit.set("lower", "0")
        limit.set("upper", "0.1")
    limit.set("effort", "1000.0")
    limit.set("velocity", "10.0")
    joint_names.add(name)
    return True


def _usd_fixed_joint_origin_xyz(joint_prim) -> str:
    """Return ``xyz`` string from physics:localPos0, or ``0 0 0``."""
    attr = joint_prim.GetAttribute("physics:localPos0")
    if attr and attr.HasAuthoredValueOpinion():
        val = attr.Get()
        if val is not None:
            try:
                return f"{float(val[0])} {float(val[1])} {float(val[2])}"
            except Exception:
                pass
    return "0 0 0"


def _inject_orphan_bridge_fixed_joints(
    stage,
    target_path: str,
    root,
    *,
    joint_names: set[str],
    link_names: set[str],
) -> list[str]:
    """Inject USD FixedJoints that Isaac's exporter dropped, connecting URDF orphans.

    Assembled robots (G1D Chassis+Lift+Body+Arms) weld parts with
    ``AssemblerFixedJoint`` / caster mount fixed joints. UsdToUrdfConverter often
    omits those, leaving multiple roots (``AGV_link`` + ``LZ_ot_Link`` + …) that
    urdfdom rejects. Bridge any FixedJoint whose parent+child links already exist
    and whose child is currently an orphan root.
    """
    from xml.etree import ElementTree as ET

    from pxr import Usd, UsdPhysics

    target = stage.GetPrimAtPath(target_path)
    if not target or not target.IsValid():
        return []

    child_links = {
        j.find("child").get("link")
        for j in root.findall("joint")
        if j.find("child") is not None and j.find("child").get("link")
    }
    injected: list[str] = []

    for prim in Usd.PrimRange(target):
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        if not prim.IsActive():
            continue
        enabled = prim.GetAttribute("physics:jointEnabled")
        if enabled and enabled.HasAuthoredValueOpinion() and enabled.Get() is False:
            continue

        parent, child = _usd_joint_parent_child_names(prim)
        if not parent or not child:
            continue
        if parent not in link_names or child not in link_names:
            continue
        # Only attach currently-orphan children (avoid multi-parent URDF).
        if child in child_links:
            continue

        base_name = prim.GetName() or "fixed_joint"
        # Many assembler welds share the name AssemblerFixedJoint.
        name = base_name
        if name in joint_names or name == "AssemblerFixedJoint":
            name = f"assembler_{parent}_to_{child}"
        if name in joint_names:
            continue

        j_el = ET.SubElement(root, "joint", {"name": name, "type": "fixed"})
        ET.SubElement(j_el, "origin", {"xyz": _usd_fixed_joint_origin_xyz(prim), "rpy": "0 0 0"})
        ET.SubElement(j_el, "parent", {"link": parent})
        ET.SubElement(j_el, "child", {"link": child})
        joint_names.add(name)
        child_links.add(child)
        injected.append(name)

    return injected


def _ensure_hw_joint(rc, name: str, *, commandable: bool) -> bool:
    """Ensure ``name`` exists under a <ros2_control> block. Returns True if added."""
    from xml.etree import ElementTree as ET

    existing = {jel.get("name") for jel in rc.findall("joint") if jel.get("name")}
    if name in existing:
        return False
    jel = ET.SubElement(rc, "joint", {"name": name})
    if commandable:
        for kind in ("position", "velocity", "effort"):
            ET.SubElement(jel, "command_interface", {"name": kind})
    for kind in ("position", "velocity", "effort"):
        ET.SubElement(jel, "state_interface", {"name": kind})
    return True


def _dedupe_urdf_duplicate_names(root) -> list[str]:
    """Make link/joint names unique for urdfdom (G1D hub+chassis both ``base_link``).

    Isaac's USD→URDF exporter uses prim leaf names, so composite robots collide
    (``/G1D/base_link`` vs ``/G1D/G1D_Chassis/base_link``; dual Dex1 fingers).
    Rename later instances to ``name__N``, then retarget joint parent/child by
    walking joints in order and attaching unused instances into the tree.
    """
    from collections import defaultdict

    changed: list[str] = []

    # --- links ---
    by_name: dict[str, list] = defaultdict(list)
    for link in root.findall("link"):
        name = link.get("name")
        if name:
            by_name[name].append(link)

    instances: dict[str, list[str]] = {}
    for name, els in by_name.items():
        if len(els) == 1:
            instances[name] = [name]
            continue
        uniq: list[str] = []
        for i, el in enumerate(els):
            new_name = name if i == 0 else f"{name}__{i}"
            if new_name != el.get("name"):
                el.set("name", new_name)
                changed.append(f"link:{name}→{new_name}")
            uniq.append(new_name)
        instances[name] = uniq

    ambiguous = {n for n, u in instances.items() if len(u) > 1}
    if ambiguous:
        unused: dict[str, list[str]] = {n: list(u) for n, u in instances.items() if n in ambiguous}
        in_tree: set[str] = set()
        attached_from: dict[str, str] = {}
        last_attached: dict[str, str] = {}

        if any(lnk.get("name") == "world" for lnk in root.findall("link")):
            in_tree.add("world")

        def _take_child(orig: str) -> str:
            pool = unused[orig]
            for cand in list(pool):
                if cand not in in_tree:
                    pool.remove(cand)
                    return cand
            if pool:
                return pool.pop(0)
            return instances[orig][-1]

        def _resolve_parent(orig: str, joint_name: str) -> str:
            if orig not in ambiguous:
                return orig
            in_tree_inst = [c for c in instances[orig] if c in in_tree]
            if not in_tree_inst:
                return instances[orig][0]
            if len(in_tree_inst) == 1:
                return in_tree_inst[0]
            jn = (joint_name or "").lower()
            for side in ("left", "right"):
                if not (jn.startswith(side) or f"_{side}_" in f"_{jn}_"):
                    continue
                for cand in in_tree_inst:
                    src = attached_from.get(cand, "")
                    if side in src.lower() or side in cand.lower():
                        return cand
            la = last_attached.get(orig)
            if la and la in in_tree:
                return la
            return in_tree_inst[0]

        for joint in root.findall("joint"):
            parent_el = joint.find("parent")
            child_el = joint.find("child")
            if parent_el is None or child_el is None:
                continue
            p_orig = parent_el.get("link")
            c_orig = child_el.get("link")
            if not p_orig or not c_orig:
                continue
            jname = joint.get("name") or ""

            if p_orig == c_orig and p_orig in ambiguous:
                p_res = _resolve_parent(p_orig, jname)
                c_res = None
                for cand in instances[p_orig]:
                    if cand != p_res and cand not in in_tree:
                        c_res = cand
                        break
                if c_res is None:
                    c_res = _take_child(p_orig)
                elif c_res in unused.get(p_orig, []):
                    unused[p_orig].remove(c_res)
                parent_el.set("link", p_res)
                child_el.set("link", c_res)
                in_tree.add(p_res)
                in_tree.add(c_res)
                attached_from[c_res] = p_res
                last_attached[p_orig] = c_res
                continue

            p_res = _resolve_parent(p_orig, jname) if p_orig in ambiguous else p_orig
            if p_orig in ambiguous and p_res not in in_tree:
                p_res = instances[p_orig][0]
            c_res = _take_child(c_orig) if c_orig in ambiguous else c_orig
            parent_el.set("link", p_res)
            child_el.set("link", c_res)
            in_tree.add(p_res)
            in_tree.add(c_res)
            attached_from[c_res] = p_res
            if c_orig in ambiguous:
                last_attached[c_orig] = c_res

    # --- joints (AssemblerFixedJoint × N, etc.) ---
    seen_joints: dict[str, int] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name:
            continue
        count = seen_joints.get(name, 0)
        seen_joints[name] = count + 1
        if count == 0:
            continue
        new_name = f"{name}__{count}"
        joint.set("name", new_name)
        changed.append(f"joint:{name}→{new_name}")

    return changed


def _fix_synth_urdf(stage, target_path: str, urdf_xml: str) -> tuple[str, list[str], list[str]]:
    """Ensure EE gripper/hand joints exist in both kinematics and <ros2_control>.

    Isaac's USD→URDF path often flattens EE meshes onto the tip link and omits
    seamless EE joints from both the kinematic tree and the hardware block.
    Scan USD for EE/hand DOFs (same spirit as robot_common_launch hand/gripper
    detect), inject missing kinematics, and append missing HW interfaces so
    AdaptiveGripperController / hand controllers can claim joints.

    Returns (xml, injected_or_added, removed).
    """
    from xml.etree import ElementTree as ET

    root = ET.fromstring(urdf_xml)
    joint_names = {j.get("name") for j in root.findall("joint") if j.get("name")}
    link_names = {lnk.get("name") for lnk in root.findall("link") if lnk.get("name")}
    injected: list[str] = []
    removed: list[str] = []

    tip_candidates = ("Link6", "link6", "tcp", "flange", "base_link")
    default_parent = next((n for n in tip_candidates if n in link_names), None)
    if default_parent is None and link_names:
        default_parent = sorted(link_names)[-1]

    rc_blocks = root.findall("ros2_control")
    if not rc_blocks:
        carb.log_warn("ROS2ControlLaunch: synth URDF has no <ros2_control> block")
        return urdf_xml, injected, removed
    rc_main = rc_blocks[0]

    # 1) HW joints listed but missing from kinematics → inject from USD (or drop).
    for rc in rc_blocks:
        for jel in list(rc.findall("joint")):
            name = jel.get("name")
            if not name or name in joint_names:
                continue
            joint_prim = _find_usd_joint_prim(stage, target_path, name)
            if joint_prim is None:
                rc.remove(jel)
                removed.append(str(name))
                continue
            if _inject_kinematic_joint(
                root,
                name=name,
                joint_prim=joint_prim,
                joint_names=joint_names,
                link_names=link_names,
                default_parent=default_parent,
            ):
                injected.append(str(name))

    # 2) USD EE/hand joints missing from HW and/or kinematics → add both.
    #    Actuated *gripper* joints get command interfaces; others are state-only
    #    (PhysX/Newton mimic still drives fingers outside ros2_control).
    usd_ee = list(_iter_usd_ee_joint_prims(stage, target_path))
    if usd_ee:
        carb.log_info(
            "ROS2ControlLaunch: USD EE/hand joints: "
            + ", ".join(p.GetName() for p in usd_ee)
        )
    for joint_prim in usd_ee:
        name = joint_prim.GetName()
        name_l = name.lower()
        commandable = _usd_joint_has_drive(joint_prim) or (
            "gripper" in name_l and "joint" in name_l
        )
        if _ensure_hw_joint(rc_main, name, commandable=commandable):
            injected.append(f"{name}[hw]")
        if _inject_kinematic_joint(
            root,
            name=name,
            joint_prim=joint_prim,
            joint_names=joint_names,
            link_names=link_names,
            default_parent=default_parent,
        ):
            injected.append(str(name))

    # 3) Assembler / mount FixedJoints dropped by UsdToUrdfConverter → re-bridge
    #    orphan roots so urdfdom sees a single kinematic tree (G1D dual-root case).
    bridges = _inject_orphan_bridge_fixed_joints(
        stage,
        target_path,
        root,
        joint_names=joint_names,
        link_names=link_names,
    )
    if bridges:
        injected.extend(bridges)
        carb.log_info(
            "ROS2ControlLaunch: bridged orphan synth roots: " + ", ".join(bridges)
        )

    # 4) urdfdom rejects lower/upper=-inf (caster/wheel unlimited revolute). Isaac's
    #    _fix_joint_limits sometimes leaves them; convert ±inf revolute → continuous.
    sanitized = _sanitize_urdf_joint_limits(root)
    if sanitized:
        injected.extend(sanitized)
        carb.log_info(
            "ROS2ControlLaunch: sanitized non-finite joint limits: "
            + ", ".join(sanitized)
        )

    # 5) Composite robots export duplicate leaf names (hub+chassis base_link,
    #    dual Dex1 fingers). urdfdom: "link 'base_link' is not unique".
    deduped = _dedupe_urdf_duplicate_names(root)
    if deduped:
        injected.extend(deduped)
        carb.log_info(
            "ROS2ControlLaunch: deduped synth names: " + ", ".join(deduped)
        )

    if injected:
        carb.log_info(
            "ROS2ControlLaunch: repaired synth EE joints: " + ", ".join(injected)
        )
    if removed:
        carb.log_warn(
            "ROS2ControlLaunch: dropped unresolved ros2_control joints: " + ", ".join(removed)
        )
    return ET.tostring(root, encoding="unicode"), injected, removed


def _finite_limit(value: str | None) -> float | None:
    """Parse a URDF limit attribute; None if missing or non-finite."""
    import math

    if value is None:
        return None
    try:
        number = float(value)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def _sanitize_urdf_joint_limits(root) -> list[str]:
    """Make joint <limit> parseable by urdfdom; prefer continuous for free spin.

    Returns short labels of joints that were changed (for logging).
    """
    import math
    from xml.etree import ElementTree as ET

    changed: list[str] = []
    pi = math.pi
    for joint in root.findall("joint"):
        jtype = joint.get("type")
        name = joint.get("name") or "?"
        if jtype not in ("revolute", "continuous", "prismatic"):
            continue
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")

        lower = _finite_limit(limit.get("lower"))
        upper = _finite_limit(limit.get("upper"))
        # Exporter often leaves revolute + lower=-inf/upper=inf for wheels/casters.
        if jtype == "revolute" and lower is None and upper is None:
            joint.set("type", "continuous")
            if "lower" in limit.attrib:
                del limit.attrib["lower"]
            if "upper" in limit.attrib:
                del limit.attrib["upper"]
            changed.append(f"{name}→continuous")
        elif jtype == "revolute":
            if lower is None:
                limit.set("lower", str(-pi))
                changed.append(f"{name}.lower")
            if upper is None:
                limit.set("upper", str(pi))
                changed.append(f"{name}.upper")
        elif jtype == "prismatic":
            if lower is None:
                limit.set("lower", "-1.0")
                changed.append(f"{name}.lower")
            if upper is None:
                limit.set("upper", "1.0")
                changed.append(f"{name}.upper")
        elif jtype == "continuous":
            # continuous may omit range; drop non-finite lower/upper if present.
            if "lower" in limit.attrib and _finite_limit(limit.get("lower")) is None:
                del limit.attrib["lower"]
                changed.append(f"{name}.drop_lower")
            if "upper" in limit.attrib and _finite_limit(limit.get("upper")) is None:
                del limit.attrib["upper"]
                changed.append(f"{name}.drop_upper")

        if _finite_limit(limit.get("effort")) is None:
            limit.set("effort", "1000.0")
        if _finite_limit(limit.get("velocity")) is None:
            limit.set("velocity", "10.0")

    return changed


def _isaac_physics_update_rate_hz() -> int:
    """Read physics Hz from the stage PhysicsScene (PhysX / Newton timeStepsPerSecond)."""
    try:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            for prim in stage.Traverse():
                if not prim.IsA(UsdPhysics.Scene):
                    continue
                # PhysX: physxScene:timeStepsPerSecond
                physx_attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
                if physx_attr and physx_attr.HasValue():
                    steps = int(physx_attr.Get() or 0)
                    if steps > 0:
                        carb.log_info(
                            f"ROS2ControlLaunch: PhysicsScene {prim.GetPath()} "
                            f"physxScene:timeStepsPerSecond={steps}"
                        )
                        return steps
                # Newton: newton:timeStepsPerSecond
                newton_attr = prim.GetAttribute("newton:timeStepsPerSecond")
                if newton_attr and newton_attr.HasValue():
                    steps = int(newton_attr.Get() or 0)
                    if steps > 0:
                        carb.log_info(
                            f"ROS2ControlLaunch: PhysicsScene {prim.GetPath()} "
                            f"newton:timeStepsPerSecond={steps}"
                        )
                        return steps
    except Exception as exc:
        carb.log_warn(f"ROS2ControlLaunch: PhysicsScene rate read failed: {exc}")

    # Fallback: SimulationManager (wraps the same PhysicsScene attrs).
    try:
        from isaacsim.core.simulation_manager import SimulationManager

        dt = float(SimulationManager.get_physics_dt())
        if dt > 0.0:
            rate = int(round(1.0 / dt))
            if rate > 0:
                carb.log_info(
                    f"ROS2ControlLaunch: using SimulationManager.get_physics_dt -> {rate} Hz"
                )
                return rate
    except Exception as exc:
        carb.log_warn(f"ROS2ControlLaunch: SimulationManager.get_physics_dt failed: {exc}")

    carb.log_warn("ROS2ControlLaunch: no PhysicsScene rate found, defaulting to 60 Hz")
    return 60


class OgnROS2ControlLaunchInternalState:
    def __init__(self) -> None:
        self._timeline_sub = None
        self._needs_reconfigure = False
        self._failed = False
        self._prim_path = ""
        self._activate_thread: threading.Thread | None = None
        self._activate_error: str | None = None
        self._activate_done = False
        self._description_publisher = RobotDescriptionPublisher()
        self._atm_proc: subprocess.Popen | None = None
        self._atm_params_path: str = ""
        self._ros_setup: str = ""

    def ensure_timeline_subscription(self) -> None:
        if self._timeline_sub is not None:
            return
        timeline = omni.timeline.get_timeline_interface()
        if timeline is None:
            carb.log_warn("ROS2ControlLaunch: omni.timeline unavailable")
            return
        stream = timeline.get_timeline_event_stream()
        if stream is None:
            carb.log_warn("ROS2ControlLaunch: timeline event stream unavailable")
            return
        self._timeline_sub = stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP),
            self._on_stop,
        )

    def stop_arms_target_manager(self) -> None:
        proc = self._atm_proc
        self._atm_proc = None
        if proc is None:
            return
        try:
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except (ProcessLookupError, PermissionError, OSError):
                    proc.terminate()
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except (ProcessLookupError, PermissionError, OSError):
                        proc.kill()
                    proc.wait(timeout=2.0)
            carb.log_info("ROS2ControlLaunch: arms_target_manager stopped")
        except Exception as exc:
            carb.log_warn(f"ROS2ControlLaunch: ATM stop failed: {exc}")

    def start_arms_target_manager(self, params_path: str) -> None:
        """Spawn ATM as a host OS process on wall clock (not a Kit/sim thread).

        Independent of Isaac ``use_sim_time`` / ``/clock`` so VR and markers do
        not ride the physics step. ``nice`` yields CPU to the sim; Stop still
        tears the process down with the ControllerManager.
        """
        self.stop_arms_target_manager()
        if not params_path or not os.path.isfile(params_path):
            raise RuntimeError(f"ATM params missing: {params_path!r}")

        env = os.environ.copy()
        setup = _resolve_ros_setup(self._ros_setup, env)
        # Host bash + sourced workspace: Kit PYTHONPATH must not shadow rclpy.
        # Always wall-clock (override any params-file use_sim_time).
        inner = (
            "exec nice -n 10 ros2 run arms_target_manager arms_target_manager_node "
            "--ros-args -r __node:=arms_target_manager "
            f"--params-file {shlex.quote(params_path)} "
            "-p use_sim_time:=false"
        )
        if setup:
            _prepare_host_ros_env(env)
            cmd = ["bash", "-c", f'source "{setup}" && {inner}']
        else:
            cmd = ["bash", "-c", inner]

        self._atm_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )
        carb.log_info(
            f"ROS2ControlLaunch: arms_target_manager host pid={self._atm_proc.pid} "
            f"(wall-clock, nice=10, not Kit thread) params={params_path}"
        )

    def _on_stop(self, _event) -> None:
        # Ignore Stop while a background activate is in flight — pausing the
        # timeline (or transient Stop events) must not tear down the CM mid-switch.
        if self._activate_thread is not None and not self._activate_done:
            carb.log_warn("ROS2ControlLaunch: ignoring Stop during activate")
            return
        self.stop_arms_target_manager()
        self._description_publisher.stop()
        if self._prim_path:
            try:
                Ros2ControlManager.teardown(self._prim_path)
            except Exception as exc:
                carb.log_warn(f"ROS2ControlLaunch teardown: {exc}")
            self._prim_path = ""
        self._activate_thread = None
        self._activate_error = None
        self._activate_done = False
        self._atm_params_path = ""
        self._needs_reconfigure = True

    def start_activate(self, controllers: list[str], namespace: str) -> None:
        self._activate_error = None
        self._activate_done = False

        def _worker() -> None:
            try:
                activate_controllers(controllers, namespace=namespace)
            except Exception as exc:
                self._activate_error = str(exc)
            finally:
                self._activate_done = True

        self._activate_thread = threading.Thread(
            target=_worker,
            name="fasim_ros2_activate",
            daemon=True,
        )
        self._activate_thread.start()


def _find_workspace_setup(ament_prefix_path: str) -> str:
    """Return install/setup.bash from the first non-/opt/ros ament prefix."""
    for prefix in (ament_prefix_path or "").split(":"):
        prefix = prefix.strip().rstrip("/")
        if not prefix or prefix.startswith("/opt/ros"):
            continue
        parent = os.path.dirname(prefix)
        for candidate in (
            os.path.join(prefix, "setup.bash"),
            os.path.join(parent, "setup.bash") if os.path.basename(parent) == "install" else "",
        ):
            if candidate and os.path.isfile(candidate):
                return candidate
    return ""


def _resolve_ros_setup(ros_setup: str, env: dict) -> str:
    setup = (ros_setup or "").strip()
    if setup and os.path.isfile(setup):
        return setup
    return _find_workspace_setup(env.get("AMENT_PREFIX_PATH", ""))


def _prepare_host_ros_env(env: dict) -> None:
    """Reset path vars so ``source install/setup.bash`` rebuilds overlay order.

    Kit launchers strip ``/opt/ros`` from AMENT (pinocchio/Kit NumPy crash), then
    leave a *partial* prefix list. Re-sourcing setup.bash on top of that can
    insert ``/opt/ros`` *before* some workspace packages. A stub
    ``/opt/ros/.../share/component_models`` (no ``xacro/``) then shadows the real
    ros2_ws package and ``get_ros2_control_robot_description`` returns empty.
    Clearing these lets colcon rebuild workspace-first, underlay-last.
    """
    for key in (
        "PYTHONPATH",
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
    ):
        env.pop(key, None)


def _run_emit(
    *,
    robot: str,
    robot_type: str,
    mode: str,
    profile: str,
    enable_gripper: bool,
    ros_setup: str,
    namespace: str = "",
    update_rate: int = 0,
    left_type: str = "",
    right_type: str = "",
    enable_arms_target_manager: bool = True,
) -> dict:
    if not os.path.isfile(_EMIT_SCRIPT):
        raise RuntimeError(f"emit_control.py not found: {_EMIT_SCRIPT}")

    args = [
        "--robot",
        robot,
        "--type",
        robot_type or "",
        "--mode",
        mode or "demo",
        "--enable-gripper",
        "true" if enable_gripper else "false",
        "--namespace",
        namespace or "",
        "--left-type",
        left_type or "",
        "--right-type",
        right_type or "",
        "--enable-arms-target-manager",
        "true" if enable_arms_target_manager else "false",
    ]
    if profile:
        args.extend(["--profile", profile])
    if update_rate and int(update_rate) > 0:
        args.extend(["--update-rate", str(int(update_rate))])

    env = os.environ.copy()
    # Kit / isaac-sim.sh rewrites PYTHONPATH for Omniverse; AMENT_PREFIX_PATH can
    # still be set from a sourced shell while robot_common_launch is no longer
    # importable. Always source install/setup.bash in a host bash so colcon
    # rebuilds PYTHONPATH for emit_control.py.
    python = shutil.which("python3") or sys.executable
    if "isaacsim" in (python or "") or "kit/python" in (python or ""):
        # Prefer system python3 over Kit's interpreter for ROS packages.
        host_python = shutil.which("python3", path="/usr/bin:/bin") or "/usr/bin/python3"
        if os.path.isfile(host_python):
            python = host_python

    setup = _resolve_ros_setup(ros_setup, env)
    if setup:
        # Drop Kit-polluted / post-guard partial prefixes; setup.bash rebuilds ROS paths.
        _prepare_host_ros_env(env)
        cmd = [
            "bash",
            "-c",
            f'source "{setup}" && exec {subprocess.list2cmdline([python, _EMIT_SCRIPT, *args])}',
        ]
    elif env.get("AMENT_PREFIX_PATH"):
        cmd = [python, _EMIT_SCRIPT, *args]
    else:
        raise RuntimeError(
            "Cannot find ros2_ws install/setup.bash; source the workspace or set "
            "inputs:rosSetup (AMENT_PREFIX_PATH alone is not enough inside Kit)"
        )

    proc = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )
    if proc.stderr:
        for line in proc.stderr.splitlines():
            carb.log_info(f"emit_control: {line}")
    if proc.returncode != 0:
        raise RuntimeError(
            f"emit_control exited {proc.returncode}: {(proc.stderr or proc.stdout or '').strip()}"
        )
    lines = [line.strip() for line in (proc.stdout or "").splitlines() if line.strip()]
    if not lines:
        raise RuntimeError("emit_control produced no stdout JSON")
    try:
        payload = json.loads(lines[-1])
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"emit_control JSON parse failed: {lines[-1]!r}") from exc
    if not isinstance(payload, dict) or "yaml_path" not in payload or "activate" not in payload:
        raise RuntimeError(f"emit_control payload missing keys: {payload!r}")
    return payload


class OgnROS2ControlLaunch:
    @staticmethod
    def internal_state() -> OgnROS2ControlLaunchInternalState:
        return OgnROS2ControlLaunchInternalState()

    @staticmethod
    def compute(db: og.Database) -> bool:
        state = db.per_instance_state
        state.ensure_timeline_subscription()

        if state._needs_reconfigure:
            db.state.configured = False
            state._failed = False
            state._needs_reconfigure = False

        if db.state.configured:
            return True
        if state._failed:
            return False

        # Activation runs off-thread so physics post-step can keep updating the CM.
        if state._activate_thread is not None:
            if not state._activate_done:
                return True
            state._activate_thread = None
            if state._activate_error:
                carb.log_error(
                    f"ROS2ControlLaunch activate failed for {state._prim_path!r}: "
                    f"{state._activate_error}"
                )
                state.stop_arms_target_manager()
                state._description_publisher.stop()
                if state._prim_path:
                    try:
                        Ros2ControlManager.teardown(state._prim_path)
                    except Exception:
                        pass
                    state._prim_path = ""
                state._failed = True
                return False
            # Controllers active — spawn ATM for VR / interactive markers.
            if state._atm_params_path:
                try:
                    state.start_arms_target_manager(state._atm_params_path)
                except Exception as exc:
                    carb.log_error(
                        f"ROS2ControlLaunch: arms_target_manager spawn failed: {exc}"
                    )
                    state.stop_arms_target_manager()
                    state._description_publisher.stop()
                    if state._prim_path:
                        try:
                            Ros2ControlManager.teardown(state._prim_path)
                        except Exception:
                            pass
                        state._prim_path = ""
                    state._failed = True
                    return False
            db.state.configured = True
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED
            return True

        target_prims = db.inputs.targetPrim
        if not target_prims:
            db.log_error("ROS2ControlLaunch: targetPrim is required")
            state._failed = True
            return False
        if len(target_prims) != 1:
            db.log_error(
                f"ROS2ControlLaunch: exactly one targetPrim is required, got {len(target_prims)}"
            )
            state._failed = True
            return False

        robot = str(db.inputs.robot or "").strip()
        if not robot:
            db.log_error("ROS2ControlLaunch: robot is required")
            state._failed = True
            return False

        prim_path = str(target_prims[0])
        try:
            ensure_isaac_ros2_library_path()
            update_rate = _isaac_physics_update_rate_hz()
            carb.log_info(
                f"ROS2ControlLaunch: aligning ros2_control update_rate to Isaac physics "
                f"{update_rate} Hz"
            )
            robot_type, left_type, right_type, enable_gripper = _resolve_ee_launch_types(
                prim_path,
                str(db.inputs.type or ""),
                bool(db.inputs.enableGripper),
            )
            namespace = str(db.inputs.namespace or "")

            # Probe + repair Isaac synth: seamless EE joints (gripper_joint) are
            # often listed in <ros2_control> but omitted from kinematic export.
            # Inject them from USD so CM/JSB/gripper_controller stay consistent.
            import omni.usd
            import isaacsim.ros2.control.ros2_control_manager as _cm_mod
            import isaacsim.ros2.control.urdf_synth as _urdf_synth
            from xml.etree import ElementTree as ET

            stage = omni.usd.get_context().get_stage()
            if stage is None:
                raise RuntimeError("no active USD stage")
            raw_synth = _urdf_synth.build_full_urdf(stage, prim_path)
            fixed_synth, injected_joints, removed_joints = _fix_synth_urdf(
                stage, prim_path, raw_synth
            )
            try:
                dump = f"/tmp/fasim_synth_urdf_{os.getpid()}.urdf"
                with open(dump, "w", encoding="utf-8") as handle:
                    handle.write(fixed_synth)
                carb.log_info(f"ROS2ControlLaunch: wrote fixed synth URDF to {dump}")
            except Exception:
                pass

            hw_joints: set[str] = set()
            try:
                for rc in ET.fromstring(fixed_synth).findall("ros2_control"):
                    for jel in rc.findall("joint"):
                        name = jel.get("name")
                        if name:
                            hw_joints.add(name)
            except Exception:
                pass

            usd_ee_names = [p.GetName() for p in _iter_usd_ee_joint_prims(stage, prim_path)]
            hw_ee = [n for n in hw_joints if _is_ee_joint_name(n)]
            # Align with robot_common_launch: enable_gripper means "detect and
            # spawn hand/gripper controllers". Only fall back to arm-only when
            # the stage itself has no EE DOFs to claim.
            if enable_gripper and not hw_ee and not usd_ee_names:
                carb.log_warn(
                    "ROS2ControlLaunch: enableGripper/EE requested but no USD EE joints "
                    f"under {prim_path!r}; skipping hand/gripper controllers "
                    f"(hw={sorted(hw_joints)}, injected={injected_joints}, "
                    f"removed={removed_joints})"
                )
                enable_gripper = False
            elif enable_gripper and usd_ee_names and not hw_ee:
                missing = sorted(set(usd_ee_names) - hw_joints)
                carb.log_error(
                    "ROS2ControlLaunch: USD EE joints present but still missing from "
                    f"synth HW after repair (usd_ee={usd_ee_names}, missing={missing}, "
                    f"injected={injected_joints}). Hand/gripper activate may fail."
                )
            elif enable_gripper and hw_ee:
                carb.log_info(
                    "ROS2ControlLaunch: EE joints in synth HW: " + ", ".join(sorted(hw_ee))
                )

            enable_atm = bool(getattr(db.inputs, "enableArmsTargetManager", True))
            state._ros_setup = str(db.inputs.rosSetup or "")
            payload = _run_emit(
                robot=robot,
                robot_type=robot_type,
                mode=str(db.inputs.mode or "demo"),
                profile=str(db.inputs.profile or ""),
                enable_gripper=enable_gripper,
                ros_setup=state._ros_setup,
                namespace=namespace,
                update_rate=update_rate,
                left_type=left_type,
                right_type=right_type,
                enable_arms_target_manager=enable_atm,
            )
            yaml_path = payload["yaml_path"]
            activate = list(payload["activate"])
            hand_controllers = list(payload.get("hand_controllers") or [])
            atm_params = str(payload.get("arms_target_manager_params") or "")
            state._atm_params_path = atm_params if enable_atm else ""
            if hand_controllers:
                carb.log_info(
                    "ROS2ControlLaunch: hand/gripper controllers (robot_common_launch detect): "
                    + ", ".join(hand_controllers)
                )
            elif enable_gripper:
                carb.log_warn(
                    "ROS2ControlLaunch: enable_gripper=true but detect_controllers "
                    "found no hand/gripper (check type/xacro robot_description joints)"
                )
            if enable_atm and not atm_params:
                carb.log_warn(
                    "ROS2ControlLaunch: enableArmsTargetManager=true but emit_control "
                    "did not return arms_target_manager_params (VR teleop unavailable)"
                )
            elif atm_params:
                carb.log_info(
                    f"ROS2ControlLaunch: ATM params ready ({atm_params}); "
                    "will spawn after controller activate"
                )
            if not os.path.isfile(yaml_path):
                raise RuntimeError(f"merged YAML missing: {yaml_path}")

            _orig_cm = _cm_mod.build_full_urdf
            _orig_synth = _urdf_synth.build_full_urdf

            def _build_fixed(stage_arg, target, sensor_overlay_urdf_path=None):
                if str(target) == prim_path and sensor_overlay_urdf_path is None:
                    return fixed_synth
                raw = _orig_synth(
                    stage_arg, target, sensor_overlay_urdf_path=sensor_overlay_urdf_path
                )
                fixed, _, _ = _fix_synth_urdf(stage_arg, str(target), raw)
                return fixed

            _cm_mod.build_full_urdf = _build_fixed
            _urdf_synth.build_full_urdf = _build_fixed
            try:
                Ros2ControlManager.setup(
                    prim_path=prim_path,
                    controller_config=yaml_path,
                    urdf_path=None,
                    namespace=namespace,
                    publish_robot_description=False,
                    use_sim_time=bool(db.inputs.useSimTime),
                )
            finally:
                _cm_mod.build_full_urdf = _orig_cm
                _urdf_synth.build_full_urdf = _orig_synth
            state._prim_path = prim_path

            if bool(getattr(db.inputs, "publishRobotDescription", True)):
                desc_path = str(payload.get("robot_description_path") or "")
                if not desc_path or not os.path.isfile(desc_path):
                    raise RuntimeError(
                        "publishRobotDescription is set but emit_control did not "
                        "return robot_description_path"
                    )
                with open(desc_path, encoding="utf-8") as handle:
                    urdf_xml = handle.read()
                topic = state._description_publisher.start(urdf_xml, namespace=namespace)
                carb.log_info(
                    f"ROS2ControlLaunch: latched robot_common_launch description on {topic} "
                    f"({len(urdf_xml)} bytes, enable_gripper={enable_gripper}, "
                    f"hw_joints={sorted(hw_joints)})"
                )

            state.start_activate(activate, namespace=namespace)
        except Exception as exc:
            carb.log_error(f"ROS2ControlLaunch setup failed for {prim_path!r}: {exc}")
            state.stop_arms_target_manager()
            state._description_publisher.stop()
            if state._prim_path:
                try:
                    Ros2ControlManager.teardown(state._prim_path)
                except Exception:
                    pass
                state._prim_path = ""
            state._atm_params_path = ""
            state._failed = True
            return False

        return True
