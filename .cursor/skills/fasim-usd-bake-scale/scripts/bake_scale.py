#!/usr/bin/env python3
"""Bake non-unit xformOp:scale into mesh points / child translates; set scale to 1."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path


def _setup_pxr():
    try:
        from pxr import Usd  # noqa: F401
        return
    except ImportError:
        pass
    import os
    home = Path.home()
    matches = sorted(home.glob("isaacsim/extscache/omni.usd.libs-*-lx64.r.cp312"))
    if not matches:
        raise SystemExit(
            "pxr not found. Use Isaac Sim python with omni.usd.libs on PYTHONPATH/LD_LIBRARY_PATH."
        )
    usd_libs = str(matches[-1])
    os.environ["PYTHONPATH"] = usd_libs + ((":" + os.environ["PYTHONPATH"]) if os.environ.get("PYTHONPATH") else "")
    os.environ["LD_LIBRARY_PATH"] = (
        usd_libs + "/bin" + ((":" + os.environ["LD_LIBRARY_PATH"]) if os.environ.get("LD_LIBRARY_PATH") else "")
    )
    if usd_libs not in sys.path:
        sys.path.insert(0, usd_libs)


_setup_pxr()

from pxr import Gf, Usd, UsdGeom, Vt  # noqa: E402

EPS = 1e-6


def get_scale_op(xf):
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            return op
    return None


def get_translate_op(xf):
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return None


def as_xyz(v):
    return float(v[0]), float(v[1]), float(v[2])


def set_vec(op, x, y, z):
    cur = op.Get()
    if isinstance(cur, Gf.Vec3d):
        op.Set(Gf.Vec3d(x, y, z))
    else:
        op.Set(Gf.Vec3f(x, y, z))


def scale_points_attr(attr, sx, sy, sz):
    if not attr or not attr.HasAuthoredValueOpinion():
        return False
    pts = attr.Get()
    if pts is None:
        return False
    attr.Set(Vt.Vec3fArray([Gf.Vec3f(float(p[0]) * sx, float(p[1]) * sy, float(p[2]) * sz) for p in pts]))
    return True


def scale_normals_attr(attr, sx, sy, sz):
    if not attr or not attr.HasAuthoredValueOpinion():
        return
    nrm = attr.Get()
    if nrm is None:
        return
    if abs(sx - sy) < 1e-9 and abs(sy - sz) < 1e-9 and sx > 0:
        return
    invx = 0.0 if abs(sx) < 1e-12 else 1.0 / sx
    invy = 0.0 if abs(sy) < 1e-12 else 1.0 / sy
    invz = 0.0 if abs(sz) < 1e-12 else 1.0 / sz
    out = []
    for n in nrm:
        x, y, z = float(n[0]) * invx, float(n[1]) * invy, float(n[2]) * invz
        length = math.sqrt(x * x + y * y + z * z)
        if length > 1e-12:
            x, y, z = x / length, y / length, z / length
        out.append(Gf.Vec3f(x, y, z))
    attr.Set(Vt.Vec3fArray(out))


def scale_extent_attr(attr, sx, sy, sz):
    if not attr or not attr.HasAuthoredValueOpinion():
        return
    ext = attr.Get()
    if ext is None or len(ext) != 2:
        return
    mn = [float(ext[0][0]) * sx, float(ext[0][1]) * sy, float(ext[0][2]) * sz]
    mx = [float(ext[1][0]) * sx, float(ext[1][1]) * sy, float(ext[1][2]) * sz]
    for i in range(3):
        if mn[i] > mx[i]:
            mn[i], mx[i] = mx[i], mn[i]
    attr.Set(Vt.Vec3fArray([Gf.Vec3f(*mn), Gf.Vec3f(*mx)]))


def bake_mesh_geometry(mesh_prim, sx, sy, sz):
    mesh = UsdGeom.Mesh(mesh_prim)
    scaled = scale_points_attr(mesh.GetPointsAttr(), sx, sy, sz)
    scale_normals_attr(mesh.GetNormalsAttr(), sx, sy, sz)
    scale_extent_attr(mesh.GetExtentAttr(), sx, sy, sz)
    return scaled


def multiply_child_trs(child, sx, sy, sz):
    if not child.IsA(UsdGeom.Xformable):
        return
    xf = UsdGeom.Xformable(child)
    top = get_translate_op(xf)
    if top and top.Get() is not None:
        tx, ty, tz = as_xyz(top.Get())
        set_vec(top, tx * sx, ty * sy, tz * sz)
    sop = get_scale_op(xf)
    if sop and sop.Get() is not None:
        cx, cy, cz = as_xyz(sop.Get())
        set_vec(sop, cx * sx, cy * sy, cz * sz)


def bake_prim(prim, log):
    if not prim.IsA(UsdGeom.Xformable):
        for c in prim.GetChildren():
            bake_prim(c, log)
        return

    xf = UsdGeom.Xformable(prim)
    sop = get_scale_op(xf)
    sx = sy = sz = 1.0
    if sop and sop.Get() is not None:
        sx, sy, sz = as_xyz(sop.Get())

    needs = abs(sx - 1) > EPS or abs(sy - 1) > EPS or abs(sz - 1) > EPS
    if needs:
        log.append(f"bake {prim.GetPath()} scale=({sx:.8g},{sy:.8g},{sz:.8g})")
        if prim.IsA(UsdGeom.Mesh):
            bake_mesh_geometry(prim, sx, sy, sz)
        for child in prim.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                multiply_child_trs(child, sx, sy, sz)
                child_sop = get_scale_op(UsdGeom.Xformable(child))
                if child_sop is None or child_sop.Get() is None:
                    bake_mesh_geometry(child, sx, sy, sz)
            elif child.IsA(UsdGeom.Xformable):
                multiply_child_trs(child, sx, sy, sz)
        set_vec(sop, 1.0, 1.0, 1.0)

    for child in prim.GetChildren():
        bake_prim(child, log)


def world_bbox_size(stage):
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(
        stage.GetPseudoRoot()
    )
    return bbox.GetRange().GetSize()


def count_nonunit_scales(stage):
    n = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Xformable):
            continue
        sop = get_scale_op(UsdGeom.Xformable(prim))
        if not sop or sop.Get() is None:
            continue
        x, y, z = as_xyz(sop.Get())
        if abs(x - 1) > 1e-6 or abs(y - 1) > 1e-6 or abs(z - 1) > 1e-6:
            n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("asset", type=Path, help="Path to .usd / .usda / .usdc")
    ap.add_argument("--dry-run", action="store_true", help="Report only; do not save")
    args = ap.parse_args()
    path = str(args.asset.resolve())
    stage = Usd.Stage.Open(path)
    if not stage:
        raise SystemExit(f"failed to open {path}")

    before = world_bbox_size(stage)
    before_n = count_nonunit_scales(stage)
    log = []
    bake_prim(stage.GetPseudoRoot(), log)
    for line in log:
        print(line)

    if not args.dry_run:
        stage.GetRootLayer().Save()
        stage = Usd.Stage.Open(path)

    after = world_bbox_size(stage)
    after_n = count_nonunit_scales(stage)
    print(f"nonunit scales: {before_n} -> {after_n}")
    print(f"bbox before: ({before[0]:.9g}, {before[1]:.9g}, {before[2]:.9g})")
    print(f"bbox after:  ({after[0]:.9g}, {after[1]:.9g}, {after[2]:.9g})")
    for i, axis in enumerate("xyz"):
        denom = max(abs(before[i]), 1e-12)
        print(f"bbox {axis} rel diff: {abs(after[i] - before[i]) / denom:.3e}")
    if after_n:
        raise SystemExit("remaining non-unit scales")


if __name__ == "__main__":
    main()
