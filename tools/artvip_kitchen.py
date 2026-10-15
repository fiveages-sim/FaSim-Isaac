#!/usr/bin/env python3
"""下载原版 ArtVIP，并把厨房转成可 payload 的 Newton scene 组件。

镜像在导入 huggingface_hub 之前设置。默认 https://hf-mirror.com。
环境变量 HF_ENDPOINT 或参数 --endpoint 已有值时不覆盖。

    python tools/artvip_kitchen.py download
    python tools/artvip_kitchen.py convert
    python tools/artvip_kitchen.py all
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HF_ENDPOINT = "https://hf-mirror.com"
DATASET_ID = "X-Humanoid/ArtVIP"
OFFICIAL_DIR = REPO_ROOT / "environment" / "ArtVIP"
KITCHEN_REL = Path("Interactive_scene") / "kitchen"
OUTPUT_DIR = REPO_ROOT / "environment" / "ArtVIP-fasim" / "kitchen"
ISAAC_PYTHON = Path.home() / "isaacsim" / "kit" / "python" / "bin" / "python3"
SHELL_THICKNESS_M = 0.002
POT_LID_GAP_M = 0.01
PLATE_STACK_GAP_M = 0.002
BBOX_REL_TOL = 1e-4

# 与具体机器人无关、目前写在 Taku 厨房 env 里的意见。官方没有这些 prim 时 over 不生效。
FASIM_OVERS = [
    (
        ["oven", "oven_E_door_73", "RevoluteJoint_oven_door"],
        [],
        ["float drive:angular:physics:stiffness = 0.8"],
    ),
    (
        ["model_fridge", "fridge_E_Leftdoor_84", "RevoluteJoint_fridge_left"],
        [],
        [
            "float drive:angular:physics:damping = 10",
            "float drive:angular:physics:stiffness = 59.994778",
        ],
    ),
    (
        ["model_fridge", "fridge_E_Rightdoor_73", "RevoluteJoint_fridge_right"],
        [],
        [
            "float drive:angular:physics:damping = 10",
            "float drive:angular:physics:stiffness = 59.996376",
        ],
    ),
    (["kitchen_all_light"], [], ['token visibility = "invisible"']),
    (
        ["Access_control", "Access_control_E_button_09"],
        [],
        ["double3 xformOp:translate = (-0.0019292831420898438, 0.00412750244140625, 0.09463346004486084)"],
    ),
    (
        ["switch", "switch_E_button_09"],
        [],
        ["double3 xformOp:translate = (-0.0019290447235107422, 0.00412750244140625, 0.09463346004486084)"],
    ),
]


def configure_mirror(endpoint: str | None) -> str:
    if endpoint:
        os.environ["HF_ENDPOINT"] = endpoint.rstrip("/")
    elif not os.environ.get("HF_ENDPOINT"):
        os.environ["HF_ENDPOINT"] = DEFAULT_HF_ENDPOINT
    # hf-mirror 不走 xet / hf_transfer，否则大文件下载会失败。
    os.environ.setdefault("HF_HUB_DISABLE_XET", "1")
    os.environ.setdefault("HF_HUB_ENABLE_HF_TRANSFER", "0")
    return os.environ["HF_ENDPOINT"]


def kitchen_src() -> Path:
    return OFFICIAL_DIR / KITCHEN_REL


def ignore_copy(_directory: str, names: list[str]) -> set[str]:
    skipped = set()
    for name in names:
        if name in {".thumbs", "__pycache__", ".git"} or ".bak" in name or name.endswith(".pyc"):
            skipped.add(name)
    return skipped


def ensure_pxr() -> None:
    try:
        import pxr  # noqa: F401
    except ImportError:
        if ISAAC_PYTHON.is_file() and Path(sys.executable).resolve() != ISAAC_PYTHON.resolve():
            os.execv(str(ISAAC_PYTHON), [str(ISAAC_PYTHON), *sys.argv])
        raise SystemExit("转换需要 pxr。请用 Isaac Sim 自带的 python 运行这个脚本。")


def _snapshot_download(snapshot_download, **kwargs) -> None:
    import time

    from huggingface_hub.errors import HfHubHTTPError

    for attempt in range(6):
        try:
            snapshot_download(
                repo_id=DATASET_ID,
                repo_type="dataset",
                local_dir=str(OFFICIAL_DIR),
                max_workers=2,
                **kwargs,
            )
            return
        except HfHubHTTPError as exc:
            if attempt == 5:
                raise
            wait = 8 * (attempt + 1)
            print(f"下载被镜像限流或拒绝 ({exc.__class__.__name__})，{wait}s 后重试 {attempt + 2}/6")
            time.sleep(wait)


def download() -> None:
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        import subprocess

        print("正在安装 huggingface_hub ...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface_hub"])
        from huggingface_hub import snapshot_download

    OFFICIAL_DIR.mkdir(parents=True, exist_ok=True)
    print(f"HF_ENDPOINT={os.environ.get('HF_ENDPOINT')}")
    print(f"下载 {DATASET_ID} -> {OFFICIAL_DIR}")
    ignored = ["**/.DS_Store", "**/.thumbs/**", "**/__MACOSX/**", "**/._*"]
    # 先拿厨房，转换不依赖其余物体；整库接着下，已有文件会跳过。
    _snapshot_download(
        snapshot_download,
        allow_patterns=["Interactive_scene/**", "README.md", ".gitattributes"],
        ignore_patterns=ignored,
    )
    _snapshot_download(snapshot_download, ignore_patterns=ignored)
    entry = kitchen_src() / "Interactive_kitchen.usd"
    if not entry.is_file():
        raise SystemExit(f"下载结束但没有找到 {entry}")
    print(f"原版厨房入口: {entry}")


def _local_matrix(prim, cache):
    from pxr import UsdGeom

    if not prim.IsA(UsdGeom.Xformable):
        return None
    matrix, _resets = cache.GetLocalTransformation(prim)
    return matrix


def _is_rigid_body(stage, path) -> bool:
    from pxr import UsdPhysics

    if not path or str(path) in {"", "/"}:
        return False
    prim = stage.GetPrimAtPath(path)
    return bool(prim and prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI))


def _local_layers(stage):
    from pxr import Sdf

    root = stage.GetRootLayer()
    found = []

    def walk(layer):
        if layer is None or layer in found:
            return
        found.append(layer)
        for path in layer.subLayerPaths:
            walk(Sdf.Layer.FindRelativeToLayer(layer, path))

    walk(root)
    return found


def _xform_authored_here(prim, stage, local_layers) -> bool:
    from pxr import Usd, UsdGeom

    if not prim.IsA(UsdGeom.Xformable):
        return False
    allowed = {layer.identifier for layer in local_layers}
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        stack = op.GetAttr().GetPropertyStack(Usd.TimeCode.Default())
        if stack and stack[0].layer.identifier in allowed:
            return True
    return False


def _has_mesh(prim) -> bool:
    from pxr import Usd, UsdGeom

    if prim.IsA(UsdGeom.Mesh):
        return True
    for child in Usd.PrimRange(prim):
        if child.IsA(UsdGeom.Mesh):
            return True
    return False


def audit_stage(stage) -> dict:
    from pxr import Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    default_prim = stage.GetDefaultPrim()
    root = default_prim.GetPath() if default_prim else stage.GetPseudoRoot().GetPath()

    physics_scenes = []
    scripts = []
    mirrored = []
    flat_colliders = []
    art_roots = []
    orphan_joints = []

    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene" or prim.IsA(UsdPhysics.Scene):
            if str(prim.GetPath()).startswith(str(root)):
                physics_scenes.append(str(prim.GetPath()))
        scripts_attr = prim.GetAttribute("omni:scripting:scripts")
        if scripts_attr and scripts_attr.HasAuthoredValue():
            for asset in scripts_attr.Get() or []:
                scripts.append((str(prim.GetPath()), str(asset.path)))
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_roots.append(str(prim.GetPath()))
        matrix = _local_matrix(prim, cache)
        world = cache.GetLocalToWorldTransform(prim) if matrix is not None else None
        rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        local_negative = matrix is not None and matrix.GetDeterminant() < 0 and _has_mesh(prim)
        world_negative = world is not None and world.GetDeterminant() < 0 and rigid
        if local_negative or world_negative:
            mirrored.append(
                {
                    "path": str(prim.GetPath()),
                    "rigid": rigid,
                    "local_det": float(matrix.GetDeterminant()) if matrix is not None else 0.0,
                    "world_det": float(world.GetDeterminant()) if world is not None else 0.0,
                }
            )
        if prim.IsA(UsdGeom.Mesh) and (
            prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)
        ):
            approx = prim.GetAttribute("physics:approximation").Get()
            points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if points and _is_flat(points) and approx in ("convexDecomposition", "convexHull"):
                flat_colliders.append({"path": str(prim.GetPath()), "approx": str(approx), "n": len(points)})

    covered, joint_rows = _articulated_joint_index(stage)
    world_joints = []
    for path, body0, body1, enabled in joint_rows:
        if not enabled:
            continue
        # body 为空表示钉在世界坐标上。父 prim 平移后，这个世界姿态不会跟着走。
        # PhysX 不允许铰链链接是 kinematic，所以锚点必须留在铰链外面。
        body0_world = body0 in {"", "/"}
        body1_world = body1 in {"", "/"}
        if body0_world or body1_world:
            child = body1 if body0_world else body0
            joint_prim = stage.GetPrimAtPath(path)
            side = "1" if body0_world else "0"
            pos = joint_prim.GetAttribute(f"physics:localPos{side}").Get()
            rot = joint_prim.GetAttribute(f"physics:localRot{side}").Get()
            world_joints.append(
                {
                    "path": path,
                    "child": child,
                    "localPos": _vec3(pos),
                    "localRot": _quat(rot),
                    "xformLines": _local_xform_lines(stage.GetPrimAtPath(child)),
                }
            )
            continue
        if path in covered:
            continue
        bodies = [body0, body1]
        if not all(_is_rigid_body(stage, name) for name in bodies):
            continue
        orphan_joints.append({"path": path, "bodies": bodies})

    groups = _orphan_groups(orphan_joints)
    return {
        "default_prim": str(root),
        "physics_scenes": physics_scenes,
        "scripts": scripts,
        "mirrored": mirrored,
        "flat_colliders": flat_colliders,
        "art_roots": art_roots,
        "orphan_joints": orphan_joints,
        "orphan_groups": groups,
        "world_joints": world_joints,
    }


def _vec3(value) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    return (float(value[0]), float(value[1]), float(value[2]))


def _quat(value) -> tuple[float, float, float, float]:
    if value is None:
        return (1.0, 0.0, 0.0, 0.0)
    imag = value.GetImaginary()
    return (float(value.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))


def _fmt_vec3(value) -> str:
    return f"({value[0]:.17g}, {value[1]:.17g}, {value[2]:.17g})"


def _fmt_quat(value) -> str:
    return f"({value[0]:.17g}, {value[1]:.17g}, {value[2]:.17g}, {value[3]:.17g})"


def _fmt_usd_value(value, type_name) -> str:
    name = str(type_name)
    if name == "matrix4d":
        rows = []
        for row in range(4):
            rows.append("(" + ", ".join(f"{float(value[row][col]):.17g}" for col in range(4)) + ")")
        return "(" + ", ".join(rows) + ")"
    if name.startswith("quat"):
        return _fmt_quat(_quat(value))
    return _fmt_vec3(_vec3(value))


def _local_xform_lines(prim) -> list[str]:
    from pxr import UsdGeom

    ordered = UsdGeom.Xformable(prim).GetOrderedXformOps()
    lines = []
    names = []
    for op in ordered:
        attr = op.GetAttr()
        type_name = attr.GetTypeName()
        lines.append(f"{type_name} {op.GetOpName()} = {_fmt_usd_value(attr.Get(), type_name)}")
        names.append(op.GetOpName())
    if names:
        quoted = ", ".join(f'"{name}"' for name in names)
        lines.append(f"uniform token[] xformOpOrder = [{quoted}]")
    return lines


def _articulated_joint_index(stage):
    import warnings

    from pxr import UsdPhysics

    root = stage.GetPseudoRoot().GetPath()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        report = UsdPhysics.LoadUsdPhysicsFromRange(stage, [root], excludePaths=[])
    covered = set()
    if UsdPhysics.ObjectType.Articulation in report:
        _paths, descs = report[UsdPhysics.ObjectType.Articulation]
        for desc in descs:
            covered.update(str(path) for path in desc.articulatedJoints)
    rows = []
    for key, value in report.items():
        if "Joint" not in str(key):
            continue
        paths, descs = value
        for path, desc in zip(paths, descs, strict=False):
            rows.append((str(path), str(desc.body0), str(desc.body1), bool(desc.jointEnabled)))
    return covered, rows


def _affects_negative_rigid(prim, cache) -> bool:
    from pxr import Usd, UsdPhysics

    for candidate in Usd.PrimRange(prim):
        if not candidate.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if cache.GetLocalToWorldTransform(candidate).GetDeterminant() < 0:
            return True
    return False


def _orphan_groups(orphan_joints: list[dict]) -> list[dict]:
    parent = {}

    def find(name):
        parent.setdefault(name, name)
        while parent[name] != name:
            parent[name] = parent[parent[name]]
            name = parent[name]
        return name

    def union(a, b):
        parent[find(a)] = find(b)

    for joint in orphan_joints:
        union(joint["bodies"][0], joint["bodies"][1])
    buckets: dict[str, list[dict]] = {}
    for joint in orphan_joints:
        buckets.setdefault(find(joint["bodies"][0]), []).append(joint)
    groups = []
    for joints in buckets.values():
        body0_count: dict[str, int] = {}
        bodies = []
        for joint in joints:
            for body in joint["bodies"]:
                if body not in bodies:
                    bodies.append(body)
            body0_count[joint["bodies"][0]] = body0_count.get(joint["bodies"][0], 0) + 1
        children = {joint["bodies"][1] for joint in joints}
        bases = [name for name in bodies if name not in children]
        pool = bases or bodies
        root = max(pool, key=lambda name: body0_count.get(name, 0))
        groups.append({"root": root, "joints": [j["path"] for j in joints], "bodies": bodies})
    return groups


def _is_flat(points, eps: float = 1e-6) -> bool:
    count = len(points)
    if count < 4:
        return True
    mean = [0.0, 0.0, 0.0]
    for point in points:
        mean[0] += point[0]
        mean[1] += point[1]
        mean[2] += point[2]
    mean = [v / count for v in mean]
    cov = [[0.0, 0.0, 0.0] for _ in range(3)]
    for point in points:
        d = [point[i] - mean[i] for i in range(3)]
        for row in range(3):
            for col in range(3):
                cov[row][col] += d[row] * d[col]
    evals = _symmetric_eigvals(cov)
    largest = max(evals)
    if largest <= 1e-20:
        return True
    return min(evals) <= eps * largest


def _symmetric_eigvals(m):
    # 3x3 对称矩阵特征值，解三次方程。
    a, b, c = m[0][0], m[1][1], m[2][2]
    d, e, f = m[0][1], m[0][2], m[1][2]
    p1 = d * d + e * e + f * f
    if p1 <= 1e-30:
        return [a, b, c]
    q = (a + b + c) / 3.0
    p2 = (a - q) ** 2 + (b - q) ** 2 + (c - q) ** 2 + 2.0 * p1
    p = (p2 / 6.0) ** 0.5
    inv_p = 1.0 / p
    b11, b22, b33 = (a - q) * inv_p, (b - q) * inv_p, (c - q) * inv_p
    b12, b13, b23 = d * inv_p, e * inv_p, f * inv_p
    det = (
        b11 * (b22 * b33 - b23 * b23)
        - b12 * (b12 * b33 - b13 * b23)
        + b13 * (b12 * b23 - b22 * b13)
    )
    det = max(-1.0, min(1.0, det / 2.0))
    import math

    angle = math.acos(det) / 3.0
    return [
        q + 2 * p * math.cos(angle),
        q + 2 * p * math.cos(angle + (2 * math.pi / 3)),
        q + 2 * p * math.cos(angle + (4 * math.pi / 3)),
    ]


def print_audit(info: dict) -> None:
    print("—— 原版厨房审计 ——")
    print(f"defaultPrim {info['default_prim']}")
    print(f"PhysicsScene ({len(info['physics_scenes'])})")
    for path in info["physics_scenes"]:
        print(f"  {path}")
    print(f"ArticulationRoot ({len(info['art_roots'])})")
    for path in info["art_roots"][:30]:
        print(f"  {path}")
    if len(info["art_roots"]) > 30:
        print(f"  ... 另有 {len(info['art_roots']) - 30} 个")
    print(f"孤儿关节 ({len(info['orphan_joints'])})，分组 {len(info['orphan_groups'])}")
    for group in info["orphan_groups"]:
        print(f"  root {group['root']} joints {len(group['joints'])}")
    print(f"负行列式且带网格 ({len(info['mirrored'])})")
    for item in info["mirrored"][:40]:
        print(f"  rb={item['rigid']} det={item['local_det']:.4g} {item['path']}")
    if len(info["mirrored"]) > 40:
        print(f"  ... 另有 {len(info['mirrored']) - 40} 个")
    print(f"零厚度凸碰撞 ({len(info['flat_colliders'])})")
    for item in info["flat_colliders"]:
        print(f"  {item['approx']} n={item['n']} {item['path']}")
    print(f"行为脚本 ({len(info['scripts'])})")
    for prim_path, asset in info["scripts"]:
        print(f"  {prim_path} -> {asset}")


def _reflection_scale(matrix):
    linear = [[matrix[row][col] for col in range(3)] for row in range(3)]
    scale = max(abs(linear[r][c]) for r in range(3) for c in range(3)) or 1.0
    off = max(abs(linear[r][c]) for r in range(3) for c in range(3) if r != c)
    if off <= 1e-5 * scale:
        negative = [i for i in range(3) if linear[i][i] < 0]
        if len(negative) == 1:
            axes = [1.0, 1.0, 1.0]
            axes[negative[0]] = -1.0
            return axes
    return [-1.0, 1.0, 1.0]


def _scale_matrix(axes):
    from pxr import Gf

    # Gf.Matrix4d 的 [row][col] 赋值不会写回矩阵，必须用构造函数。
    return Gf.Matrix4d(
        axes[0], 0.0, 0.0, 0.0,
        0.0, axes[1], 0.0, 0.0,
        0.0, 0.0, axes[2], 0.0,
        0.0, 0.0, 0.0, 1.0,
    )


def _set_local_matrix(prim, matrix) -> None:
    from pxr import UsdGeom

    xform = UsdGeom.Xformable(prim)
    ops = list(xform.GetOrderedXformOps())
    op = next((item for item in ops if item.GetOpType() == UsdGeom.XformOp.TypeTransform), None)
    if op is None:
        op = xform.AddTransformOp()
    op.Set(matrix)
    if ops != [op]:
        xform.SetXformOpOrder([op])


def _scale_vec3(value, axes):
    return type(value)(value[0] * axes[0], value[1] * axes[1], value[2] * axes[2])


def _reverse_per_face(counts, values):
    if not counts or not values or len(values) != sum(counts):
        return None
    flipped = []
    cursor = 0
    for count in counts:
        flipped.extend(reversed(list(values[cursor : cursor + count])))
        cursor += count
    return flipped


def _flip_faces(mesh) -> None:
    from pxr import UsdGeom

    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    if not counts or not indices:
        return
    flipped = _reverse_per_face(counts, indices)
    if flipped is not None:
        mesh.GetFaceVertexIndicesAttr().Set(flipped)
    if mesh.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying:
        normals = mesh.GetNormalsAttr().Get()
        flipped_normals = _reverse_per_face(counts, normals) if normals else None
        if flipped_normals is not None:
            mesh.GetNormalsAttr().Set(flipped_normals)
    api = UsdGeom.PrimvarsAPI(mesh)
    for primvar in api.GetPrimvars():
        if primvar.GetInterpolation() != UsdGeom.Tokens.faceVarying:
            continue
        if primvar.IsIndexed():
            flipped_indices = _reverse_per_face(counts, primvar.GetIndices())
            if flipped_indices is not None:
                primvar.SetIndices(flipped_indices)
            continue
        values = primvar.Get()
        flipped_values = _reverse_per_face(counts, values)
        if flipped_values is not None:
            primvar.Set(flipped_values)


def _set_extent(mesh, points) -> None:
    from pxr import Gf, Vt

    low = [min(point[i] for point in points) for i in range(3)]
    high = [max(point[i] for point in points) for i in range(3)]
    extent = Vt.Vec3fArray([Gf.Vec3f(*low), Gf.Vec3f(*high)])
    attr = mesh.GetExtentAttr()
    if attr:
        attr.Set(extent)
    else:
        mesh.CreateExtentAttr(extent)


def _bake_prim(stage, prim, cache) -> None:
    from pxr import Usd, UsdGeom

    matrix = _local_matrix(prim, cache)
    axes = _reflection_scale(matrix)
    scale = _scale_matrix(axes)
    new_matrix = scale * matrix
    descendants = [child for child in Usd.PrimRange(prim) if child != prim]
    child_matrices = []
    for child in descendants:
        if child.IsA(UsdGeom.Xformable):
            child_matrices.append((child, _local_matrix(child, cache)))
    for child in Usd.PrimRange(prim):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        points = mesh.GetPointsAttr().Get()
        if points:
            mesh.GetPointsAttr().Set([_scale_vec3(point, axes) for point in points])
            _set_extent(mesh, mesh.GetPointsAttr().Get())
        normals = mesh.GetNormalsAttr().Get()
        if normals and mesh.GetNormalsInterpolation() != UsdGeom.Tokens.faceVarying:
            mesh.GetNormalsAttr().Set([_scale_vec3(normal, axes) for normal in normals])
        _flip_faces(mesh)
        if mesh.GetNormalsInterpolation() == UsdGeom.Tokens.faceVarying:
            normals = mesh.GetNormalsAttr().Get()
            if normals:
                mesh.GetNormalsAttr().Set([_scale_vec3(normal, axes) for normal in normals])
    for child, child_matrix in child_matrices:
        if child_matrix is None:
            continue
        updated = scale * child_matrix * scale
        if updated == child_matrix and not UsdGeom.Xformable(child).GetOrderedXformOps():
            continue
        _set_local_matrix(child, updated)
    _set_local_matrix(prim, new_matrix)
    cache.Clear()


def _world_size(cache, prim):
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        return None
    size = box.GetSize()
    return (size[0], size[1], size[2])


def bake_mirrors(stage, *, require_authored: bool = True) -> int:
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    bbox_cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    local_layers = _local_layers(stage)
    baked = 0
    for _round in range(8):
        cache.Clear()
        authors = []
        for prim in stage.Traverse():
            matrix = _local_matrix(prim, cache)
            if matrix is None or matrix.GetDeterminant() >= 0 or not _has_mesh(prim):
                continue
            if cache.GetLocalToWorldTransform(prim).GetDeterminant() >= 0:
                continue
            if require_authored and not _xform_authored_here(prim, stage, local_layers):
                continue
            if not _affects_negative_rigid(prim, cache):
                continue
            authors.append(prim)
        if not authors:
            break
        author_paths = {prim.GetPath() for prim in authors}

        def _under_author(prim) -> bool:
            parent = prim.GetParent()
            while parent and parent.IsValid() and not parent.IsPseudoRoot():
                if parent.GetPath() in author_paths:
                    return True
                parent = parent.GetParent()
            return False

        # 只烘焙最外层。父 prim 本轮会改矩阵，子 prim 留到下一轮。
        authors = [prim for prim in authors if not _under_author(prim)]
        for prim in authors:
            bbox_cache.Clear()
            before = _world_size(bbox_cache, prim)
            _bake_prim(stage, prim, cache)
            bbox_cache.Clear()
            after = _world_size(bbox_cache, prim)
            if before and after:
                for old, new in zip(before, after):
                    denom = max(abs(old), 1e-6)
                    if abs(old - new) / denom > BBOX_REL_TOL:
                        raise SystemExit(
                            f"镜像烘焙改变了世界尺寸 {prim.GetPath()}: {before} -> {after}"
                        )
            baked += 1
            print(f"  烘焙镜像 {prim.GetPath()}")
    if baked:
        stage.GetRootLayer().Save()
    return baked


def _plane_normal(points, counts, indices):
    normal = [0.0, 0.0, 0.0]
    cursor = 0
    for count in counts:
        face = [points[indices[cursor + i]] for i in range(count)]
        cursor += count
        for i, point in enumerate(face):
            nxt = face[(i + 1) % len(face)]
            normal[0] += (point[1] - nxt[1]) * (point[2] + nxt[2])
            normal[1] += (point[2] - nxt[2]) * (point[0] + nxt[0])
            normal[2] += (point[0] - nxt[0]) * (point[1] + nxt[1])
    length = (normal[0] ** 2 + normal[1] ** 2 + normal[2] ** 2) ** 0.5
    if length < 1e-12:
        return [0.0, 0.0, 1.0]
    return [v / length for v in normal]


def _world_scale(prim, cache, normal) -> float:
    from pxr import Gf

    matrix = cache.GetLocalToWorldTransform(prim)
    direction = Gf.Vec4d(normal[0], normal[1], normal[2], 0.0) * matrix
    return max((direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2) ** 0.5, 1e-9)


def _extrude(points, counts, indices, normal, distance):
    from pxr import Gf

    half = distance * 0.5
    top = [
        Gf.Vec3f(p[0] + normal[0] * half, p[1] + normal[1] * half, p[2] + normal[2] * half) for p in points
    ]
    bot = [
        Gf.Vec3f(p[0] - normal[0] * half, p[1] - normal[1] * half, p[2] - normal[2] * half) for p in points
    ]
    out_points = list(top) + list(bot)
    out_counts = list(counts)
    out_indices = list(indices)
    offset = len(points)
    cursor = 0
    for count in counts:
        face = list(indices[cursor : cursor + count])
        cursor += count
        out_indices.extend(reversed([index + offset for index in face]))
        out_counts.append(count)
        for i in range(count):
            a = face[i]
            b = face[(i + 1) % count]
            out_indices.extend([a, b, b + offset, a + offset])
            out_counts.append(4)
    return out_points, out_counts, out_indices


# 房间壳体：墙、地面、窗框。网格本身是盒子，SDF 只增加接触开销。
_BOX_SHELL_NAMES = frozenset({"wall", "floor", "window"})


def box_approximate_shells(stage) -> int:
    from pxr import Usd, UsdGeom, UsdPhysics

    root_layer = stage.GetRootLayer()
    done = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)):
            continue
        if not _BOX_SHELL_NAMES.intersection(str(prim.GetPath()).split("/")):
            continue
        approx = prim.GetAttribute("physics:approximation")
        if not approx or approx.Get() == "boundingCube":
            continue
        # 只改写出这个近似值的层。组合后的厨房场景不要再盖一层重复 over。
        stack = approx.GetPropertyStack(Usd.TimeCode.Default())
        if not stack or stack[0].layer != root_layer:
            continue
        approx.Set("boundingCube")
        done += 1
        print(f"  盒体碰撞 {prim.GetPath()}")
    if done:
        root_layer.Save()
    return done


# 左右两扇滑轨门，以及门框 door_E_body_13 的各部件。按部件做包围盒，不把整樘门收成一只盒子。
_SLIDING_DOOR_BODIES = frozenset({"door_E_left_1", "door_E_right_21", "door_E_body_13"})


def _drop_sdf_api(prim, layers: set) -> bool:
    """从原有 apiSchemas 里拿掉 SDF API，保留其余 schema。空的 explicitItems 会把 schema 清空，不能带上。"""
    from pxr import Sdf

    changed = False
    for spec in prim.GetPrimStack():
        current = _schema_op(spec)
        if current is None or _SDF_API not in list(current.prependedItems):
            continue
        kwargs = {}
        prepended = [item for item in current.prependedItems if item != _SDF_API]
        appended = list(current.appendedItems)
        deleted = list(current.deletedItems)
        explicit = list(current.explicitItems)
        if prepended:
            kwargs["prependedItems"] = prepended
        if appended:
            kwargs["appendedItems"] = appended
        if deleted:
            kwargs["deletedItems"] = deleted
        if explicit:
            kwargs["explicitItems"] = explicit
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(**kwargs))
        layers.add(spec.layer)
        changed = True
    return changed


def box_approximate_sliding_doors(stage) -> int:
    """滑轨门和门框各部件的碰撞网格改成包围盒。"""
    from pxr import Sdf, UsdGeom, UsdPhysics

    layers: set = set()
    done = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)):
            continue
        if not _SLIDING_DOOR_BODIES.intersection(str(prim.GetPath()).split("/")):
            continue
        approx = prim.GetAttribute("physics:approximation")
        changed = False
        if not approx or str(approx.Get()) != "boundingCube":
            _set_authored(prim, "physics:approximation", "boundingCube", Sdf.ValueTypeNames.Token, layers)
            changed = True
        if _drop_sdf_api(prim, layers):
            changed = True
        if not changed:
            continue
        done += 1
        print(f"  门部件盒体碰撞 {prim.GetPath()}")
    for layer in layers:
        layer.Save()
    return done


def thicken_colliders(stage) -> int:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    cache = UsdGeom.XformCache()
    done = 0
    meshes = [
        prim
        for prim in stage.Traverse()
        if prim.IsA(UsdGeom.Mesh)
        and (prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI))
    ]
    for prim in meshes:
        approx = prim.GetAttribute("physics:approximation").Get()
        if approx not in ("convexDecomposition", "convexHull"):
            continue
        enabled = prim.GetAttribute("physics:collisionEnabled").Get()
        if enabled is False:
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts or not indices or not _is_flat(points):
            continue
        normal = _plane_normal(points, counts, indices)
        distance = SHELL_THICKNESS_M / _world_scale(prim, cache, normal)
        shell_points, shell_counts, shell_indices = _extrude(points, counts, indices, normal, distance)
        shell_path = prim.GetPath().AppendChild(prim.GetName() + "_newton_shell")
        if stage.GetPrimAtPath(shell_path):
            continue
        shell = UsdGeom.Mesh.Define(stage, shell_path)
        shell.CreatePointsAttr(Vt.Vec3fArray(shell_points))
        shell.CreateFaceVertexCountsAttr(Vt.IntArray(shell_counts))
        shell.CreateFaceVertexIndicesAttr(Vt.IntArray(shell_indices))
        _set_extent(shell, shell_points)
        shell.GetVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(shell.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(shell.GetPrim())
        shell.GetPrim().CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token).Set("convexHull")
        shell.GetPrim().CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        enabled = prim.GetAttribute("physics:collisionEnabled")
        if not enabled:
            enabled = prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
        enabled.Set(False)
        done += 1
        print(f"  加厚碰撞 {prim.GetPath()} local={distance:.4g}")
    if done:
        stage.GetRootLayer().Save()
    return done


def iter_usd_files(root: Path):
    for path in root.rglob("*"):
        if path.suffix.lower() in {".usd", ".usda", ".usdc"} and path.is_file() and ".bak" not in path.name:
            yield path


# Isaac Sim 5.0 脚本在 6.2 上第一行就会失败：omni.kit.scripting 已不存在，
# isaacsim.core.prims.Articulation 变成多 prim 视图，omni.isaac.core 只剩归档。
_ISAAC5_SCRIPT_IMPORTS = """from omni.kit.scripting import BehaviorScript
from isaacsim.core.prims import Articulation
from omni.isaac.core.prims import XFormPrim
from isaacsim.core.utils.types import ArticulationActions
"""

_ISAAC61_SCRIPT_IMPORTS = """# ARTVIP_ISAAC61
try:
    from omni.behavior.scripting.core import BehaviorScript
except ImportError:
    from omni.kit.scripting import BehaviorScript
from isaacsim.core.prims import SingleArticulation as _ArtVIPSingleArticulation
from isaacsim.core.prims import SingleXFormPrim as _ArtVIPSingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction as ArticulationActions


class Articulation(_ArtVIPSingleArticulation):
    def __init__(self, prim_path, *args, **kwargs):
        kwargs.setdefault("reset_xform_properties", False)
        kwargs.setdefault("name", "artvip_" + str(prim_path).strip("/").replace("/", "_"))
        super().__init__(prim_path, *args, **kwargs)
        self._artvip_ready = False

    def _artvip_ready_now(self):
        from isaacsim.core.simulation_manager import SimulationManager

        if SimulationManager.get_physics_sim_view() is None:
            self._artvip_ready = False
            return False
        # 构造时物理句柄可能已经有效，但 ArticulationController 仍要
        # SingleArticulation.initialize() 才接上。只看句柄会让 apply_action 每帧报未初始化。
        # 时间线重置后句柄失效，同样再走一遍 initialize。
        if not self.handles_initialized:
            super().initialize()
        if not self.handles_initialized:
            self._artvip_ready = False
            return False
        self._artvip_ready = True
        return True

    def get_dof_index(self, dof_name):
        if not self._artvip_ready_now():
            return None
        return super().get_dof_index(dof_name)

    def get_joint_positions(self, *args, **kwargs):
        import numpy as np

        if not self._artvip_ready_now():
            return np.zeros(64)
        positions = super().get_joint_positions(*args, **kwargs)
        if positions is None:
            self._artvip_ready = False
            return np.zeros(64)
        return positions

    def apply_action(self, action):
        if not self._artvip_ready_now():
            return None
        return super().apply_action(action)


class XFormPrim(_ArtVIPSingleXFormPrim):
    def __init__(self, prim_path, *args, **kwargs):
        kwargs.setdefault("reset_xform_properties", False)
        kwargs.setdefault("name", "artvip_" + str(prim_path).strip("/").replace("/", "_"))
        super().__init__(prim_path, *args, **kwargs)
"""


def _strip_on_init_visibility(text: str) -> str:
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    in_init = False
    for line in lines:
        if line.startswith("    def on_init("):
            in_init = True
        elif in_init and line.startswith("    def "):
            in_init = False
        if in_init and "set_visibility(" in line:
            continue
        out.append(line)
    return "".join(out)


def patch_scripts(root: Path, scripts: list[tuple[str, str]]) -> int:
    del scripts
    patched = 0
    for path in root.rglob("*.py"):
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        text = path.read_text(encoding="utf-8")
        updated = text.replace(
            'create_simulation_view("numpy", stage_id=sid)',
            'create_simulation_view("numpy", backend="newton", stage_id=sid)',
        ).replace(
            'create_simulation_view("numpy")',
            'create_simulation_view("numpy", backend="newton")',
        )
        if "ARTVIP_ISAAC61" not in updated and _ISAAC5_SCRIPT_IMPORTS in updated:
            updated = updated.replace(_ISAAC5_SCRIPT_IMPORTS, _ISAAC61_SCRIPT_IMPORTS, 1)
        ready_def = "def _artvip_ready_now(self):\n"
        fresh_ready = _ISAAC61_SCRIPT_IMPORTS.split(ready_def, 1)[1].split("\n    def get_dof_index", 1)[0]
        ready_at = updated.find(ready_def)
        dof_at = updated.find("\n    def get_dof_index", ready_at if ready_at >= 0 else 0)
        if ready_at >= 0 and dof_at > ready_at and updated[ready_at + len(ready_def) : dof_at] != fresh_ready:
            updated = updated[: ready_at + len(ready_def)] + fresh_ready + updated[dof_at:]
        # on_play 发生在物理 onResume 之前。这时创建 Articulation 会 flush 场景，
        # 机器人 Action Graph 的关节句柄就绑不上。初始化留到 on_update。
        for call in ("self._initialize_articulation()", "self._initialize_articulations()"):
            early = f"    def on_play(self):\n        {call}\n"
            deferred = "    def on_play(self):\n        return\n"
            if early in updated:
                updated = updated.replace(early, deferred, 1)
        # on_init 在播放前就会改灯光可见性，舞台因此重载，已经选中的 ROS 图不会重建。
        stripped = _strip_on_init_visibility(updated)
        if stripped != updated:
            updated = stripped
        if "_artvip_ready_now" not in updated and "self.initialize()" in updated:
            old_class = """class Articulation(_ArtVIPSingleArticulation):
    def __init__(self, prim_path, *args, **kwargs):
        kwargs.setdefault("reset_xform_properties", False)
        kwargs.setdefault("name", "artvip_" + str(prim_path).strip("/").replace("/", "_"))
        super().__init__(prim_path, *args, **kwargs)
        self.initialize()
"""
            new_class = _ISAAC61_SCRIPT_IMPORTS.split("class Articulation", 1)[1].split("class XFormPrim", 1)[0]
            updated = updated.replace(old_class, "class Articulation" + new_class, 1)
        if updated != text:
            path.write_text(updated, encoding="utf-8")
            patched += 1
            print(f"  脚本改为 Isaac Sim 6.2 {path}")
    return patched


def _usd_quote(name: str) -> str:
    return '"' + name.replace("\\", "\\\\").replace('"', '\\"') + '"'


def _over_tree() -> dict:
    return {}


def _add_over(tree: dict, parts: list[str], meta: list[str], attrs: list[str]) -> None:
    level = tree
    for index, name in enumerate(parts):
        node = level.setdefault(name, {"meta": [], "attrs": [], "kids": {}, "raw": []})
        node.setdefault("raw", [])
        if index == len(parts) - 1:
            for item in meta:
                if item not in node["meta"]:
                    node["meta"].append(item)
            for item in attrs:
                if item not in node["attrs"]:
                    node["attrs"].append(item)
        else:
            level = node["kids"]


def _emit_overs(tree: dict, indent: int) -> list[str]:
    lines = []
    pad = " " * indent
    for name, node in tree.items():
        meta = node["meta"]
        if meta:
            lines.append(f"{pad}over {_usd_quote(name)} (")
            for item in meta:
                lines.append(f"{pad}    {item}")
            lines.append(f"{pad})")
        else:
            lines.append(f"{pad}over {_usd_quote(name)}")
        lines.append(f"{pad}{{")
        for attr in node["attrs"]:
            lines.append(f"{pad}    {attr}")
        for raw in node.get("raw", []):
            for line in raw.splitlines():
                lines.append(f"{pad}    {line}" if line else "")
        lines.extend(_emit_overs(node["kids"], indent + 4))
        lines.append(f"{pad}}}")
    return lines


def write_kitchen_usda(info: dict, payload_name: str) -> None:
    default = info["default_prim"]
    tree = _over_tree()
    for scene in info["physics_scenes"]:
        if not scene.startswith(default):
            continue
        rel = scene[len(default) :].strip("/")
        if not rel:
            continue
        _add_over(tree, rel.split("/"), ["active = false"], [])
    for group in info["orphan_groups"]:
        if not group["root"].startswith(default):
            continue
        rel = group["root"][len(default) :].strip("/")
        _add_over(
            tree,
            rel.split("/"),
            [
                'prepend apiSchemas = ["PhysicsArticulationRootAPI", "NewtonArticulationRootAPI", "PhysxArticulationAPI"]'
            ],
            [
                "bool newton:selfCollisionEnabled = false",
                "bool physxArticulation:enabledSelfCollisions = false",
            ],
        )
    for parts, meta, attrs in FASIM_OVERS:
        _add_over(tree, parts, meta, attrs)
    for joint in info.get("world_joints", []):
        if not joint["path"].startswith(default):
            continue
        child = joint.get("child") or ""
        if not child.startswith(default):
            continue
        child_rel = child[len(default) :].strip("/")
        joint_rel = joint["path"][len(default) :].strip("/")
        if not child_rel or not joint_rel:
            continue
        # 铰链根仍是原来的动态刚体。运动学锚点是它的兄弟，复制同一套局部变换，
        # 固定关节排除在铰链之外，这样 PhysX 才能既固定底座又保留按钮和门。
        parent_parts = child_rel.split("/")[:-1]
        if not parent_parts:
            continue
        anchor_name = "artvip_anchor"
        anchor_path = f"{default}/{'/'.join(parent_parts)}/{anchor_name}"
        _add_over(tree, parent_parts, [], [])
        node = tree
        for name in parent_parts:
            node = node[name]
        xform_lines = "\n".join(f"    {line}" for line in joint["xformLines"])
        node["raw"].append(
            "\n".join(
                [
                    f'def Xform "{anchor_name}" (',
                    '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]',
                    ")",
                    "{",
                    "    bool physics:kinematicEnabled = true",
                    "    bool physics:rigidBodyEnabled = true",
                    "    float physics:mass = 1",
                    xform_lines,
                    "}",
                ]
            )
        )
        _add_over(
            tree,
            joint_rel.split("/"),
            [],
            [
                "bool physics:jointEnabled = true",
                "uniform bool physics:excludeFromArticulation = true",
                f"rel physics:body0 = <{anchor_path}>",
                f"point3f physics:localPos0 = {_fmt_vec3(joint['localPos'])}",
                f"quatf physics:localRot0 = {_fmt_quat(joint['localRot'])}",
            ],
        )
    body = "\n".join(_emit_overs(tree, 4))
    text = f"""#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "World" (
    prepend payload = @./{payload_name}@
)
{{
{body}
}}

def PhysicsScene "PhysicsScene" (
    prepend apiSchemas = ["NewtonSceneAPI", "PhysxSceneAPI"]
)
{{
    vector3f physics:gravityDirection = (0, 0, -1)
    float physics:gravityMagnitude = 9.81
    uniform token physxScene:broadphaseType = "SAP"
    bool physxScene:enableGPUDynamics = 0
    uint physxScene:timeStepsPerSecond = 120
}}
"""
    (OUTPUT_DIR / "kitchen.usda").write_text(text, encoding="utf-8")
    print(f"写出 {OUTPUT_DIR / 'kitchen.usda'}")


def _articulated_body_paths(stage) -> set[str]:
    import warnings

    from pxr import UsdPhysics

    root = stage.GetPseudoRoot().GetPath()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", DeprecationWarning)
        report = UsdPhysics.LoadUsdPhysicsFromRange(stage, [root], excludePaths=[])
    bodies = set()
    if UsdPhysics.ObjectType.Articulation in report:
        _paths, descs = report[UsdPhysics.ObjectType.Articulation]
        for desc in descs:
            bodies.update(str(path) for path in desc.articulatedBodies)
    return bodies


_SDF_API = "PhysxSDFMeshCollisionAPI"


def _schema_op(spec):
    if spec is None or not spec.HasInfo("apiSchemas"):
        return None
    return spec.GetInfo("apiSchemas")


def _prepend_sdf_api(mesh) -> bool:
    from pxr import Sdf

    layer = mesh.GetStage().GetEditTarget().GetLayer()
    path = mesh.GetPath()
    if not layer.GetPrimAtPath(path):
        Sdf.CreatePrimInLayer(layer, path)
    spec = layer.GetPrimAtPath(path)
    current = _schema_op(spec)
    # 只在当前层 prepend。AddAppliedSchema 会把组合结果整表写回，并删掉 MaterialBindingAPI。
    if (
        current
        and list(current.prependedItems) == [_SDF_API]
        and not list(current.deletedItems)
        and not list(current.explicitItems)
        and not list(current.appendedItems)
    ):
        return False
    spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=[_SDF_API]))
    return True


def _mark_sdf(mesh) -> bool:
    from pxr import Sdf

    changed = False
    approx = mesh.GetAttribute("physics:approximation")
    if not approx:
        approx = mesh.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token, True)
    if str(approx.Get()) != "sdf":
        approx.Set("sdf")
        changed = True
    # 只有 token 时，Isaac Sim 6.2 的 PhysX 会把动态网格当成三角网格，再退回凸包。
    if _prepend_sdf_api(mesh):
        changed = True
    return changed


def _is_pot_lid(prim) -> bool:
    parts = str(prim.GetPath()).split("/")
    return any("kitchenware" in part for part in parts) and any("cover" in part.lower() for part in parts)


def relax_cavity_colliders(stage) -> int:
    """凸包把锅、盘子的空腔填实后，叠在上面的物体会在第一帧被顶飞。锅盖保持凸包分解。"""
    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    articulated = _articulated_body_paths(stage)

    def world_points(prim, limit: int):
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not points:
            return np.zeros((0, 3))
        matrix = cache.GetLocalToWorldTransform(prim)
        step = max(1, len(points) // limit)
        out = []
        for point in list(points)[::step]:
            transformed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * matrix
            out.append((transformed[0], transformed[1], transformed[2]))
        return np.asarray(out, dtype=np.float64)

    bodies: dict[str, list] = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)):
            continue
        if prim.GetAttribute("physics:collisionEnabled").Get() is False:
            continue
        owner = prim
        while owner and owner.IsValid() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
            owner = owner.GetParent()
        if not owner or not owner.IsValid():
            continue
        if owner.GetAttribute("physics:kinematicEnabled").Get() is True:
            continue
        if str(owner.GetPath()) in articulated:
            continue
        bodies.setdefault(str(owner.GetPath()), []).append(prim)

    by_asset: dict[str, list[str]] = {}
    for path in bodies:
        by_asset.setdefault("/".join(path.split("/")[:3]), []).append(path)

    done = 0
    seen = set()
    for group in by_asset.values():
        if len(group) < 2:
            continue
        samples = {}
        for path in group:
            chunks = [world_points(mesh, 800) for mesh in bodies[path]]
            chunks = [chunk for chunk in chunks if len(chunk)]
            samples[path] = np.vstack(chunks) if chunks else np.zeros((0, 3))
        for path in group:
            for mesh in bodies[path]:
                if _is_pot_lid(mesh):
                    continue
                approx = mesh.GetAttribute("physics:approximation")
                if not approx or str(approx.Get()) not in ("convexHull", "convexDecomposition", "boundingCube"):
                    continue
                mesh_points = world_points(mesh, 1500)
                if len(mesh_points) < 8:
                    continue
                low, high = mesh_points.min(0), mesh_points.max(0)
                for other in group:
                    if other == path:
                        continue
                    foreign = samples[other]
                    if len(foreign) == 0:
                        continue
                    inside = np.all((foreign >= low) & (foreign <= high), axis=1)
                    if int(inside.sum()) < 80:
                        continue
                    sample = foreign[inside]
                    if len(sample) > 500:
                        sample = sample[:: max(1, len(sample) // 500)]
                    far = 0
                    near = 0
                    for start in range(0, len(sample), 80):
                        chunk = sample[start : start + 80]
                        delta = chunk[:, None, :] - mesh_points[None, :, :]
                        distance = np.sqrt((delta * delta).sum(-1)).min(1)
                        far += int((distance > 0.008).sum())
                        near += int((distance < 0.003).sum())
                    if far < 25 or far <= near:
                        continue
                    mesh_path = str(mesh.GetPath())
                    if mesh_path in seen:
                        continue
                    seen.add(mesh_path)
                    _mark_sdf(mesh)
                    done += 1
                    print(f"  空腔改为 SDF {mesh_path}")
                    break
    for meshes in bodies.values():
        for mesh in meshes:
            if _is_pot_lid(mesh):
                continue
            if str(mesh.GetAttribute("physics:approximation").Get()) != "sdf":
                continue
            if not _prepend_sdf_api(mesh):
                continue
            done += 1
            print(f"  补上 SDF API {mesh.GetPath()}")
    if done:
        stage.GetRootLayer().Save()
    return done


def restore_pot_lid_decomposition(stage) -> int:
    """锅盖不用 SDF，碰撞改回凸包分解，并去掉为此加上的 SDF API。"""
    from pxr import Sdf, UsdGeom, UsdPhysics

    root = stage.GetRootLayer()
    done = 0
    for prim in stage.Traverse():
        if not _is_pot_lid(prim) or not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.MeshCollisionAPI) or prim.HasAPI(UsdPhysics.CollisionAPI)):
            continue
        approx = prim.GetAttribute("physics:approximation")
        if not approx or str(approx.Get()) != "sdf":
            continue
        _set_authored(prim, "physics:approximation", "convexDecomposition", Sdf.ValueTypeNames.Token, set())
        spec = root.GetPrimAtPath(prim.GetPath())
        if spec is not None and spec.HasInfo("apiSchemas"):
            current = spec.GetInfo("apiSchemas")
            if (
                list(current.prependedItems) == [_SDF_API]
                and not list(current.deletedItems)
                and not list(current.explicitItems)
                and not list(current.appendedItems)
            ):
                spec.ClearInfo("apiSchemas")
        done += 1
        print(f"  锅盖改回凸包分解 {prim.GetPath()}")
    if done:
        root.Save()
    return done


def _world_z_extent(prim) -> tuple[float, float] | None:
    from pxr import Gf, Usd, UsdGeom

    low = high = None
    cache = UsdGeom.XformCache()
    for mesh in Usd.PrimRange(prim):
        if not mesh.IsA(UsdGeom.Mesh):
            continue
        points = mesh.GetAttribute("points").Get()
        if not points:
            continue
        matrix = cache.GetLocalToWorldTransform(mesh)
        for point in points:
            transformed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * matrix
            z = float(transformed[2])
            low = z if low is None else min(low, z)
            high = z if high is None else max(high, z)
    if low is None:
        return None
    return low, high


def _lift_local_z(matrix, delta: float):
    from pxr import Gf

    return Gf.Matrix4d(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
        float(matrix[3][0]), float(matrix[3][1]), float(matrix[3][2]) + delta, float(matrix[3][3]),
    )


_CONVEX_HULL_MESHES = (
    "/World/model_kitchenware005/E_seasoning3_13/P_56afdae2f16d8b8e",
    "/World/juicer/E_cover_13/P_b6ec523431b1c731",
)


def restore_seasoning_hull(stage) -> int:
    """这些网格不是盒子，碰撞用凸包。"""
    from pxr import Sdf

    layers: set = set()
    done = 0
    for path in _CONVEX_HULL_MESHES:
        prim = stage.GetPrimAtPath(path)
        if not prim:
            continue
        approx = prim.GetAttribute("physics:approximation")
        if approx and str(approx.Get()) == "convexHull":
            continue
        _set_authored(prim, "physics:approximation", "convexHull", Sdf.ValueTypeNames.Token, layers)
        done += 1
        print(f"  改回凸包 {prim.GetPath()}")
    for layer in layers:
        layer.Save()
    return done


_SINK_DECOMPOSITION = "/World/model_largecabinet/E_body_146/E_parts014_163/P_391d10484cb56706"


def restore_sink_decomposition(stage) -> int:
    """水槽网格用凸包分解，不用 SDF。"""
    from pxr import Sdf

    prim = stage.GetPrimAtPath(_SINK_DECOMPOSITION)
    if not prim:
        return 0
    approx = prim.GetAttribute("physics:approximation")
    if approx and str(approx.Get()) == "convexDecomposition":
        return 0
    layers: set = set()
    _set_authored(prim, "physics:approximation", "convexDecomposition", Sdf.ValueTypeNames.Token, layers)
    for layer in layers:
        layer.Save()
    print(f"  水槽改回凸包分解 {prim.GetPath()}")
    return 1


# 菜叶压在砧板上，抬高后底面离板面 1 cm。动刚体，网格跟着走。
_BOARD_VEGETABLE = "/World/model_kitchenware002/E_board1_4/E_vegetable1_5/E_Component253M2_9"
_BOARD_VEGETABLE_GAP_M = 0.01


def lift_board_vegetable(stage) -> int:
    from pxr import Gf, Sdf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(_BOARD_VEGETABLE)
    board = stage.GetPrimAtPath("/World/model_kitchenware002/E_board1_4")
    if not prim or not board:
        return 0
    piece_z = _world_z_extent(prim)
    if piece_z is None:
        return 0
    low = high = None
    cache = UsdGeom.XformCache()
    for mesh in Usd.PrimRange(board):
        if not mesh.IsA(UsdGeom.Mesh) or _BOARD_VEGETABLE.split("/")[4] in str(mesh.GetPath()).split("/"):
            continue
        if "E_vegetable" in str(mesh.GetPath()):
            continue
        points = mesh.GetAttribute("points").Get()
        if not points:
            continue
        matrix = cache.GetLocalToWorldTransform(mesh)
        for point in points:
            transformed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * matrix
            z = float(transformed[2])
            low = z if low is None else min(low, z)
            high = z if high is None else max(high, z)
    if high is None:
        return 0
    delta_world = high + _BOARD_VEGETABLE_GAP_M - piece_z[0]
    if delta_world <= 1e-4:
        return 0
    parent = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
    scale_z = float((Gf.Vec4d(0.0, 0.0, 1.0, 0.0) * parent)[2])
    if abs(scale_z) < 1e-8:
        raise SystemExit(f"无法把抬高换算到父坐标: {prim.GetPath()}")
    local = UsdGeom.Xformable(prim).GetLocalTransformation()
    layers: set = set()
    _set_authored(
        prim,
        "xformOp:transform",
        _lift_local_z(local, delta_world / scale_z),
        Sdf.ValueTypeNames.Matrix4d,
        layers,
    )
    for layer in layers:
        layer.Save()
    print(f"  菜叶抬高 {prim.GetPath()} {delta_world * 1000:.1f} mm")
    return 1


# 花的碰撞网格贴着台面。抬高后底面离台面 1 cm。
_POTTED_FLOWER = "/World/model_potted_plant017/vase/E_Component171_4"
_POTTED_FLOWER_GAP_M = 0.01


def lift_potted_flower(stage) -> int:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(_POTTED_FLOWER)
    if not prim:
        return 0
    piece_z = _world_z_extent(prim)
    if piece_z is None:
        return 0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    center = cache.ComputeWorldBound(prim).ComputeAlignedBox().GetMidpoint()
    support = None
    for mesh in stage.Traverse():
        if not mesh.IsA(UsdGeom.Mesh) or "model_potted_plant017" in str(mesh.GetPath()):
            continue
        if not (mesh.HasAPI(UsdPhysics.CollisionAPI) or mesh.HasAPI(UsdPhysics.MeshCollisionAPI)):
            continue
        box = cache.ComputeWorldBound(mesh).ComputeAlignedBox()
        if box.IsEmpty():
            continue
        low, high = box.GetMin(), box.GetMax()
        if not (low[0] <= center[0] <= high[0] and low[1] <= center[1] <= high[1]):
            continue
        top = float(high[2])
        if top > piece_z[0] + 0.02:
            continue
        support = top if support is None else max(support, top)
    if support is None:
        return 0
    delta_world = support + _POTTED_FLOWER_GAP_M - piece_z[0]
    if delta_world <= 1e-4:
        return 0
    parent = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
    scale_z = float((Gf.Vec4d(0.0, 0.0, 1.0, 0.0) * parent)[2])
    if abs(scale_z) < 1e-8:
        raise SystemExit(f"无法把抬高换算到父坐标: {prim.GetPath()}")
    local = UsdGeom.Xformable(prim).GetLocalTransformation()
    layers: set = set()
    _set_authored(
        prim,
        "xformOp:transform",
        _lift_local_z(local, delta_world / scale_z),
        Sdf.ValueTypeNames.Matrix4d,
        layers,
    )
    for layer in layers:
        layer.Save()
    print(f"  花抬高 {prim.GetPath()} {delta_world * 1000:.1f} mm")
    return 1


def separate_pot_lids(stage) -> int:
    """把锅盖抬到锅口上方一小段，避免凸包在第一帧和锅嵌在一起被弹飞。"""
    from pxr import Sdf, UsdGeom, UsdPhysics

    layers: set = set()
    done = 0
    world = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    for asset in world.GetChildren():
        if "kitchenware" not in asset.GetName():
            continue
        lids = []
        pots = []
        for child in asset.GetChildren():
            if not child.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            name = child.GetName().lower()
            if "cover" in name:
                lids.append(child)
            elif "pot" in name or name.startswith("e_body"):
                pots.append(child)
        if len(lids) != 1 or len(pots) != 1:
            continue
        pot_extent = _world_z_extent(pots[0])
        lid_extent = _world_z_extent(lids[0])
        if pot_extent is None or lid_extent is None:
            continue
        delta = pot_extent[1] + POT_LID_GAP_M - lid_extent[0]
        if delta <= 1e-4:
            continue
        local = UsdGeom.Xformable(lids[0]).GetLocalTransformation()
        _set_authored(lids[0], "xformOp:transform", _lift_local_z(local, delta), Sdf.ValueTypeNames.Matrix4d, layers)
        done += 1
        print(f"  锅盖抬高 {lids[0].GetPath()} {delta * 1000:.1f} mm")
    for layer in layers:
        layer.Save()
    return done


def separate_stacked_plates(stage) -> int:
    """叠放的碟子保持原来的嵌套，每上一只只比原版再高 2 毫米。"""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    official_path = OFFICIAL_DIR / KITCHEN_REL / "Interactive_kitchen.usd"
    official = Usd.Stage.Open(str(official_path)) if official_path.is_file() else None
    cache = UsdGeom.XformCache()
    grouped: dict[tuple, list] = {}
    for prim in stage.Traverse():
        if "plate" not in prim.GetName().lower() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if prim.GetAttribute("physics:kinematicEnabled").Get() is True:
            continue
        source = official.GetPrimAtPath(prim.GetPath()) if official else None
        if source and source.IsValid():
            baseline = UsdGeom.Xformable(source).GetLocalTransformation()
        else:
            baseline = UsdGeom.Xformable(prim).GetLocalTransformation()
        center = cache.GetLocalToWorldTransform(prim.GetParent()).Transform(baseline.ExtractTranslation())
        key = (str(prim.GetParent().GetPath()), round(float(center[0]), 1), round(float(center[1]), 1))
        grouped.setdefault(key, []).append((float(baseline[3][2]), baseline, prim))

    layers: set = set()
    done = 0
    for group in grouped.values():
        if len(group) < 2:
            continue
        group.sort(key=lambda item: item[0])
        for rank, (_, baseline, prim) in enumerate(group):
            parent = cache.GetLocalToWorldTransform(prim.GetParent())
            scale_z = float((Gf.Vec4d(0.0, 0.0, 1.0, 0.0) * parent)[2])
            if abs(scale_z) < 1e-8:
                raise SystemExit(f"无法把抬高换算到父坐标: {prim.GetPath()}")
            # 缩放已经烤到网格上时，刚体要留拆开后的矩阵。写回原版会把负缩放再乘一次，碟子倒放。
            pose = _body_pose_after_scale_bake(prim, baseline)
            target = _lift_local_z(pose, rank * PLATE_STACK_GAP_M / scale_z)
            current = UsdGeom.Xformable(prim).GetLocalTransformation()
            if _matrix_error(current, target) <= 1e-8:
                continue
            _set_authored(prim, "xformOp:transform", target, Sdf.ValueTypeNames.Matrix4d, layers)
            done += 1
            print(f"  碟子比原版抬高 {prim.GetPath()} {rank * PLATE_STACK_GAP_M * 1000:.0f} mm")
    for layer in layers:
        layer.Save()
    return done


def _body_pose_after_scale_bake(prim, baseline):
    """子网格已经带上从刚体拆出去的缩放时，返回拆开后的刚体矩阵。"""
    import numpy as np
    from pxr import Gf, UsdGeom

    if _pivot_is_identity(baseline):
        return baseline
    clean = _without_pivot(baseline)
    if _pivot_is_identity(clean) and _matrix_error(clean, baseline) < 1e-4:
        return baseline
    transform = Gf.Transform()
    transform.SetMatrix(baseline)
    scale = [float(item) for item in transform.GetScale()]
    linear = np.array([[float(baseline[row][col]) for col in range(3)] for row in range(3)])
    removed = None
    target = None
    if all(abs(item) > 1e-12 for item in scale):
        diagonal = np.diag(scale)
        rotation = np.linalg.inv(diagonal) @ linear
        candidate = _pack_linear(rotation, baseline.ExtractTranslation())
        if _pivot_is_identity(candidate) and np.max(np.abs(diagonal @ rotation - linear)) < 1e-6:
            removed = _scale_matrix(scale)
            target = candidate
    if target is None:
        removed = _pack_linear(linear, Gf.Vec3d(0.0, 0.0, 0.0))
        target = Gf.Matrix4d(1.0)
        target.SetTranslateOnly(baseline.ExtractTranslation())
    for child in prim.GetChildren():
        xformable = UsdGeom.Xformable(child)
        if not xformable or not xformable.GetOrderedXformOps():
            continue
        if _matrix_error(xformable.GetLocalTransformation(), removed) < 1e-5:
            return target
    return baseline


def _matrix_error(left, right) -> float:
    return max(abs(float(left[row][col]) - float(right[row][col])) for row in range(4) for col in range(4))


def _pivot_is_identity(matrix) -> bool:
    """PhysX 用精确比较，枢轴四元数不是 (1,0,0,0) 就会报 ScaleOrientation。"""
    from pxr import Gf

    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    quat = transform.GetPivotOrientation().GetQuat()
    imag = quat.GetImaginary()
    return float(quat.GetReal()) == 1.0 and float(imag[0]) == 0.0 and float(imag[1]) == 0.0 and float(imag[2]) == 0.0


def _without_pivot(matrix):
    from pxr import Gf

    transform = Gf.Transform()
    transform.SetMatrix(matrix)
    clean = Gf.Transform()
    clean.SetTranslation(transform.GetTranslation())
    clean.SetRotation(transform.GetRotation())
    clean.SetScale(transform.GetScale())
    return clean.GetMatrix()


def _pack_linear(linear, translation):
    from pxr import Gf

    return Gf.Matrix4d(
        float(linear[0][0]),
        float(linear[0][1]),
        float(linear[0][2]),
        0.0,
        float(linear[1][0]),
        float(linear[1][1]),
        float(linear[1][2]),
        0.0,
        float(linear[2][0]),
        float(linear[2][1]),
        float(linear[2][2]),
        0.0,
        float(translation[0]),
        float(translation[1]),
        float(translation[2]),
        1.0,
    )


def _scale_matrix(scale):
    from pxr import Gf

    sx, sy, sz = (float(item) for item in scale)
    return Gf.Matrix4d(sx, 0.0, 0.0, 0.0, 0.0, sy, 0.0, 0.0, 0.0, 0.0, sz, 0.0, 0.0, 0.0, 0.0, 1.0)


def _set_authored(prim, name, value, type_name, layers: set) -> None:
    """写到已经有这条意见的层。引用进来的层不在根图层栈里，不能 SetEditTarget。"""
    from pxr import Sdf, Usd

    attr = prim.GetAttribute(name)
    stack = attr.GetPropertyStack(Usd.TimeCode.Default()) if attr else []
    if stack:
        stack[0].default = value
        layers.add(stack[0].layer)
        return
    specs = prim.GetPrimStack()
    if not specs:
        raise SystemExit(f"没有可以写入的层: {prim.GetPath()}.{name}")
    spec = Sdf.AttributeSpec(specs[0], name, type_name)
    spec.default = value
    layers.add(specs[0].layer)


def _write_matrix(prim, matrix, layers: set) -> None:
    from pxr import Sdf, Vt

    _set_authored(prim, "xformOp:transform", matrix, Sdf.ValueTypeNames.Matrix4d, layers)
    current = prim.GetAttribute("xformOpOrder")
    if list(current.Get() or []) == ["xformOp:transform"]:
        return
    _set_authored(
        prim,
        "xformOpOrder",
        Vt.TokenArray(["xformOp:transform"]),
        Sdf.ValueTypeNames.TokenArray,
        layers,
    )


def _capture_world_points(prim, cache) -> list[tuple[str, int, tuple[float, float, float]]]:
    from pxr import Gf, UsdGeom

    samples = []
    for mesh in prim.GetChildren():
        # 只抽这个刚体直接带着的网格，以及再下一层，避免把整棵冰箱都扫进来。
        candidates = [mesh]
        if not mesh.IsA(UsdGeom.Mesh):
            candidates = [child for child in mesh.GetChildren() if child.IsA(UsdGeom.Mesh)]
        for candidate in candidates:
            if not candidate.IsA(UsdGeom.Mesh):
                continue
            points = candidate.GetAttribute("points").Get()
            if not points:
                continue
            world = cache.GetLocalToWorldTransform(candidate)
            for index in {0, len(points) // 2, len(points) - 1}:
                point = points[index]
                transformed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * world
                samples.append((str(candidate.GetPath()), index, (transformed[0], transformed[1], transformed[2])))
            if len(samples) >= 9:
                return samples
    return samples


_ARTICULATION_ROOT_APIS = (
    "PhysicsArticulationRootAPI",
    "PhysxArticulationAPI",
    "NewtonArticulationRootAPI",
)


def strip_kinematic_articulation_roots(stage) -> int:
    """运动学刚体上的关节根会被 PhysX 忽略并报警。删掉这些 API，门和抽屉的关节仍连在底座上。"""
    from pxr import Sdf, UsdPhysics

    layer = stage.GetRootLayer()
    done = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if prim.GetAttribute("physics:kinematicEnabled").Get() is not True:
            continue
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        spec = layer.GetPrimAtPath(prim.GetPath())
        if spec is None:
            spec = Sdf.CreatePrimInLayer(layer, prim.GetPath())
        current = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
        prepended = list(current.prependedItems) if current is not None else []
        appended = list(current.appendedItems) if current is not None else []
        deleted = list(current.deletedItems) if current is not None else []
        explicit = list(current.explicitItems) if current is not None else []
        changed = False
        for token in _ARTICULATION_ROOT_APIS:
            if token not in deleted:
                deleted.append(token)
                changed = True
        if not changed:
            continue
        # 空的 explicitItems 会把整个 apiSchemas 变成替换列表，刚体 API 会被清掉。
        items = {"prependedItems": prepended, "appendedItems": appended, "deletedItems": deleted}
        if explicit:
            items["explicitItems"] = explicit
        spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(**items))
        done += 1
    if done:
        layer.Save()
    return done


def clamp_revolute_limits(stage) -> int:
    """PhysX 把转动关节的上下限夹在 ±180°。写成这个范围后，运行时的行程不变，警告消失。"""
    from pxr import Sdf, UsdPhysics

    layers: set = set()
    done = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        changed = False
        upper = prim.GetAttribute("physics:upperLimit").Get()
        lower = prim.GetAttribute("physics:lowerLimit").Get()
        if upper is not None and float(upper) > 180.0:
            _set_authored(prim, "physics:upperLimit", 180.0, Sdf.ValueTypeNames.Float, layers)
            changed = True
        if lower is not None and float(lower) < -180.0:
            _set_authored(prim, "physics:lowerLimit", -180.0, Sdf.ValueTypeNames.Float, layers)
            changed = True
        if changed:
            done += 1
    for layer in layers:
        layer.Save()
    return done


def author_plate_inertia(stage) -> int:
    """开关和门禁的碰撞是带缩放的 SDF，PhysX 算出来的惯量对角为负。按世界尺寸写成正的盒体惯量。"""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    targets = (
        "/World/switch/switch_E_body_28",
        "/World/Access_control/Access_control_E_body_28",
    )
    cache = UsdGeom.XformCache()
    layers: set = set()
    done = 0
    for path in targets:
        body = stage.GetPrimAtPath(path)
        if not body or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        body_world = cache.GetLocalToWorldTransform(body)
        inverse = body_world.GetInverse()
        low = [1.0e9, 1.0e9, 1.0e9]
        high = [-1.0e9, -1.0e9, -1.0e9]
        found = False
        for child in Usd.PrimRange(body):
            if child == body or not child.HasAPI(UsdPhysics.CollisionAPI) or not child.IsA(UsdGeom.Mesh):
                continue
            points = child.GetAttribute("points").Get()
            if not points:
                continue
            world = cache.GetLocalToWorldTransform(child)
            step = max(1, len(points) // 1500)
            for point in points[::step]:
                placed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * world * inverse
                found = True
                for axis in range(3):
                    low[axis] = min(low[axis], placed[axis])
                    high[axis] = max(high[axis], placed[axis])
        if not found:
            continue
        scale = Gf.Transform(body_world).GetScale()
        # 包围盒在子物体空间，惯量要用缩放后的米制尺寸，轴仍是刚体本地轴。
        size = [max((high[axis] - low[axis]) * abs(float(scale[axis])), 0.001) for axis in range(3)]
        mass = max(0.02, 300.0 * size[0] * size[1] * size[2])
        inertia = (
            mass / 12.0 * (size[1] * size[1] + size[2] * size[2]),
            mass / 12.0 * (size[0] * size[0] + size[2] * size[2]),
            mass / 12.0 * (size[0] * size[0] + size[1] * size[1]),
        )
        center = Gf.Vec3d(0.5 * (low[0] + high[0]), 0.5 * (low[1] + high[1]), 0.5 * (low[2] + high[2]))
        current = body.GetAttribute("physics:diagonalInertia").Get()
        if current is not None and max(abs(float(current[axis]) - inertia[axis]) for axis in range(3)) < 1.0e-8:
            continue
        _prepend_mass_api(body, stage.GetRootLayer())
        _set_authored(body, "physics:mass", mass, Sdf.ValueTypeNames.Float, layers)
        _set_authored(body, "physics:diagonalInertia", Gf.Vec3f(*inertia), Sdf.ValueTypeNames.Float3, layers)
        _set_authored(
            body,
            "physics:centerOfMass",
            Gf.Vec3f(float(center[0]), float(center[1]), float(center[2])),
            Sdf.ValueTypeNames.Point3f,
            layers,
        )
        done += 1
    for layer in layers:
        layer.Save()
    stage.GetRootLayer().Save()
    return done


def _prepend_mass_api(prim, layer) -> None:
    from pxr import Sdf

    spec = layer.GetPrimAtPath(prim.GetPath())
    if spec is None:
        spec = Sdf.CreatePrimInLayer(layer, prim.GetPath())
    current = spec.GetInfo("apiSchemas") if spec.HasInfo("apiSchemas") else None
    prepended = list(current.prependedItems) if current is not None else []
    if "PhysicsMassAPI" in prepended:
        return
    prepended.insert(0, "PhysicsMassAPI")
    explicit = list(current.explicitItems) if current is not None else []
    items = {
        "prependedItems": prepended,
        "appendedItems": list(current.appendedItems) if current is not None else [],
        "deletedItems": list(current.deletedItems) if current is not None else [],
    }
    if explicit:
        items["explicitItems"] = explicit
    spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(**items))


def clear_scale_orientation(stage) -> int:
    """去掉刚体上的缩放朝向。能重写成纯旋转的只改矩阵；其余把缩放挪到子节点。"""
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics

    done = 0
    layers: set = set()
    bodies = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    for prim in bodies:
        matrix = UsdGeom.Xformable(prim).GetLocalTransformation()
        if _pivot_is_identity(matrix):
            continue
        clean = _without_pivot(matrix)
        if _pivot_is_identity(clean) and _matrix_error(clean, matrix) < 1e-4:
            before = _capture_world_points(prim, UsdGeom.XformCache())
            _write_matrix(prim, clean, layers)
            _check_world_points(stage, before)
            done += 1
            print(f"  重写刚体矩阵 {prim.GetPath()}")
            continue
        transform = Gf.Transform()
        transform.SetMatrix(matrix)
        scale = [float(item) for item in transform.GetScale()]
        linear = np.array([[float(matrix[row][col]) for col in range(3)] for row in range(3)])
        diagonal = np.diag(scale)
        removed = None
        target = None
        if all(abs(item) > 1e-12 for item in scale):
            rotation = np.linalg.inv(diagonal) @ linear
            candidate = _pack_linear(rotation, matrix.ExtractTranslation())
            if _pivot_is_identity(candidate) and np.max(np.abs(diagonal @ rotation - linear)) < 1e-6:
                removed = _scale_matrix(scale)
                target = candidate
        diagonal_bake = target is not None
        if target is None:
            removed = _pack_linear(linear, Gf.Vec3d(0.0, 0.0, 0.0))
            target = Gf.Matrix4d(1.0)
            target.SetTranslateOnly(matrix.ExtractTranslation())
            if not _pivot_is_identity(target):
                raise SystemExit(f"无法去掉缩放朝向: {prim.GetPath()}")
        before = _capture_world_points(prim, UsdGeom.XformCache())
        joint_before = _capture_joint_points(stage, str(prim.GetPath())) if diagonal_bake else []
        _push_scale_to_children(prim, removed, layers)
        _write_matrix(prim, target, layers)
        # 对角缩放留下的是纯旋转，关节位置仍在旧的刚体坐标里，要乘上同一组缩放。
        if diagonal_bake:
            _scale_joint_positions(stage, str(prim.GetPath()), scale, layers)
            _check_joint_points(stage, str(prim.GetPath()), joint_before)
        _check_world_points(stage, before)
        done += 1
        print(f"  烘焙刚体缩放 {prim.GetPath()}")
    for layer in layers:
        layer.Save()
    return done


def _push_scale_to_children(prim, removed, layers: set) -> None:
    from pxr import Gf, UsdGeom, UsdPhysics

    for child in prim.GetChildren():
        if child.IsA(UsdPhysics.Joint):
            continue
        xformable = UsdGeom.Xformable(child)
        if not xformable:
            continue
        _write_matrix(child, xformable.GetLocalTransformation() * removed, layers)


def _scale_joint_positions(stage, body_path: str, scale, layers: set) -> None:
    from pxr import Gf, UsdPhysics

    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for index, targets in (
            (0, joint.GetBody0Rel().GetTargets()),
            (1, joint.GetBody1Rel().GetTargets()),
        ):
            if not any(str(target) == body_path for target in targets):
                continue
            name = f"physics:localPos{index}"
            attr = prim.GetAttribute(name)
            pos = attr.Get() if attr else None
            if pos is None:
                continue
            scaled = (float(pos[0]) * scale[0], float(pos[1]) * scale[1], float(pos[2]) * scale[2])
            type_name = attr.GetTypeName()
            value = Gf.Vec3d(*scaled) if str(type_name).endswith("d") else Gf.Vec3f(*scaled)
            _set_authored(prim, name, value, type_name, layers)


def _capture_joint_points(stage, body_path: str):
    from pxr import Gf, UsdGeom, UsdPhysics

    cache = UsdGeom.XformCache()
    world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body_path))
    samples = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        for index, targets in (
            (0, joint.GetBody0Rel().GetTargets()),
            (1, joint.GetBody1Rel().GetTargets()),
        ):
            if not any(str(target) == body_path for target in targets):
                continue
            pos = prim.GetAttribute(f"physics:localPos{index}").Get()
            if pos is None:
                continue
            transformed = Gf.Vec4d(float(pos[0]), float(pos[1]), float(pos[2]), 1.0) * world
            samples.append((str(prim.GetPath()), index, (transformed[0], transformed[1], transformed[2])))
    return samples


def _check_joint_points(stage, body_path: str, samples) -> None:
    from pxr import Gf, UsdGeom

    world = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(body_path))
    for path, index, previous in samples:
        pos = stage.GetPrimAtPath(path).GetAttribute(f"physics:localPos{index}").Get()
        transformed = Gf.Vec4d(float(pos[0]), float(pos[1]), float(pos[2]), 1.0) * world
        error = max(abs(transformed[axis] - previous[axis]) for axis in range(3))
        if error > 1e-4:
            raise SystemExit(f"烘焙后关节位置变了 {path} error={error}")


def _check_world_points(stage, samples) -> None:
    from pxr import Gf, UsdGeom

    cache = UsdGeom.XformCache()
    for path, index, previous in samples:
        mesh = stage.GetPrimAtPath(path)
        points = mesh.GetAttribute("points").Get()
        point = points[index]
        transformed = Gf.Vec4d(float(point[0]), float(point[1]), float(point[2]), 1.0) * cache.GetLocalToWorldTransform(mesh)
        error = max(abs(transformed[axis] - previous[axis]) for axis in range(3))
        if error > 1e-4:
            raise SystemExit(f"烘焙后世界坐标变了 {path} index={index} error={error}")


def convert() -> None:
    from pxr import Usd

    src = kitchen_src()
    entry = src / "Interactive_kitchen.usd"
    if not entry.is_file():
        raise SystemExit(f"没有原版厨房 {entry}。先运行: python tools/artvip_kitchen.py download")

    source_stage = Usd.Stage.Open(str(entry))
    if not source_stage:
        raise SystemExit(f"无法打开 {entry}")
    info = audit_stage(source_stage)
    print_audit(info)
    del source_stage

    if OUTPUT_DIR.exists():
        shutil.rmtree(OUTPUT_DIR)
    OUTPUT_DIR.parent.mkdir(parents=True, exist_ok=True)
    print(f"复制 {src} -> {OUTPUT_DIR}")
    shutil.copytree(src, OUTPUT_DIR, ignore=ignore_copy)

    baked = 0
    thickened = 0
    boxed = 0
    files = sorted(iter_usd_files(OUTPUT_DIR), key=lambda path: (path.name.startswith("Interactive_kitchen"), str(path)))
    for path in files:
        stage = Usd.Stage.Open(str(path))
        if not stage:
            continue
        if path.name.startswith("Interactive_kitchen"):
            baked += bake_mirrors(stage, require_authored=False)
        thickened += thicken_colliders(stage)
        boxed += box_approximate_shells(stage)
        del stage
    print(f"镜像烘焙 {baked} 处，碰撞加厚 {thickened} 处，盒体碰撞 {boxed} 处")

    copied_entry = OUTPUT_DIR / "Interactive_kitchen.usd"
    copied = Usd.Stage.Open(str(copied_entry))
    hollow = relax_cavity_colliders(copied)
    print(f"空腔碰撞改为 SDF {hollow} 处")
    lids = restore_pot_lid_decomposition(copied)
    print(f"锅盖改回凸包分解 {lids} 处")
    lifted = separate_pot_lids(copied)
    print(f"锅盖抬高 {lifted} 处")
    plates = separate_stacked_plates(copied)
    print(f"叠放碟子拉开 {plates} 处")
    seasoning = restore_seasoning_hull(copied)
    print(f"碰撞改回凸包 {seasoning} 处")
    sink = restore_sink_decomposition(copied)
    print(f"水槽碰撞改回凸包分解 {sink} 处")
    oriented = clear_scale_orientation(copied)
    print(f"去掉刚体缩放朝向 {oriented} 处")
    vegetable = lift_board_vegetable(copied)
    print(f"菜叶抬高 {vegetable} 处")
    flower = lift_potted_flower(copied)
    print(f"花抬高 {flower} 处")
    sliding = box_approximate_sliding_doors(copied)
    print(f"门部件盒体碰撞 {sliding} 处")
    stripped = strip_kinematic_articulation_roots(copied)
    print(f"去掉运动学刚体上的关节根 {stripped} 处")
    clamped = clamp_revolute_limits(copied)
    print(f"转动上限收到 180 度 {clamped} 处")
    plated = author_plate_inertia(copied)
    print(f"开关和门禁写成正惯量 {plated} 处")
    copied_info = audit_stage(copied)
    patch_scripts(OUTPUT_DIR, copied_info["scripts"] or info["scripts"])
    write_kitchen_usda(copied_info, "Interactive_kitchen.usd")

    component = Usd.Stage.Open(str(OUTPUT_DIR / "kitchen.usda"))
    check = audit_stage(component)
    active_under_world = []
    for prim in component.Traverse():
        if prim.GetTypeName() != "PhysicsScene" or not prim.IsActive():
            continue
        if str(prim.GetPath()).startswith("/World"):
            active_under_world.append(str(prim.GetPath()))
    print(f"World 下仍活跃的 PhysicsScene: {active_under_world or '无'}")
    print(f"组件内剩余孤儿关节: {len(check['orphan_joints'])}")
    rigid_mirrors = [item for item in check["mirrored"] if item["rigid"] and item["world_det"] < 0]
    print(f"组件内剩余刚体负世界行列式: {len(rigid_mirrors)}")
    for item in rigid_mirrors[:20]:
        print(f"  {item['path']} world={item['world_det']:.6g}")
    if active_under_world:
        raise SystemExit("World 下仍有活跃 PhysicsScene。")
    if check["orphan_joints"]:
        raise SystemExit("转换后仍有孤儿关节，请看上面的审计。")
    if rigid_mirrors:
        raise SystemExit("转换后仍有刚体负世界行列式，请看上面的列表。")


def main() -> None:
    parser = argparse.ArgumentParser(description="用 Hugging Face 镜像下载 ArtVIP，并转换厨房 scene 组件")
    parser.add_argument("--endpoint", help="Hugging Face 端点。默认在未设置 HF_ENDPOINT 时用 https://hf-mirror.com")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("download", help="下载原版数据集到 environment/ArtVIP")
    sub.add_parser("convert", help="审计并转换厨房")
    sub.add_parser("all", help="下载后转换")
    args = parser.parse_args()
    endpoint = configure_mirror(args.endpoint)
    print(f"使用 Hugging Face 端点 {endpoint}")
    if args.cmd in {"convert", "all"}:
        ensure_pxr()
    if args.cmd in {"download", "all"}:
        download()
    if args.cmd in {"convert", "all"}:
        convert()


if __name__ == "__main__":
    main()
