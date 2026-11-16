"""探针：确认 Replicator ``camera_params`` annotator 的实际字段与布局。

为什么需要：``camera_params`` 的 ``cameraViewTransform`` 是 **row-major 还是
column-major**、内参是 ``cameraFocalLength`` 还是 ``cameraAperture`` 形式的
**物理参数**（需要自己换算成像素焦距），不同 Isaac 版本有差异。写错一位就是
整个点云都错。所以不猜，直接打印出来看。

用法（**headless 独立进程**，不影响正在跑的 GUI 仿真）::

    ~/isaacsim/python.sh scripts/probe_camera_params.py

输出：
- ``camera_params`` 的所有 key 和类型
- 由物理参数换算出的像素内参 K
- ``cameraViewTransform`` 的平移部分 vs USD 真实世界变换，判断布局
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})
try:
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom

    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config
    from fasim.replicator.sdg.recorder import _apply_variants

    W, H = 640, 480
    robot = load_robot_config("unitree_g1d")
    task = load_task_config("unitree_g1d", "sinopec_test_tube")

    ctx = omni.usd.get_context()
    ctx.open_stage(str(task.scene_usda))
    for _ in range(40):
        app.update()
    _apply_variants(robot.articulation_prim, task.variant_selection)
    for _ in range(60):
        app.update()

    cam = next((c for c in task.cameras if c.name == "head"), None)
    if cam is None:
        print("找不到 head 相机配置")
        raise SystemExit(1)

    rp = rep.create.render_product(cam.prim_path, (W, H))
    rgb = rep.annotators.get("rgb")
    rgb.attach(rp)
    depth = rep.annotators.get("distance_to_image_plane")
    depth.attach(rp)
    # **camera_params 在 6.2 里叫这个名字**；若不存在会抛错，正是我们要确认的
    try:
        cparams = rep.annotators.get("camera_params")
        cparams.attach(rp)
        has_cp = True
    except Exception as exc:  # noqa: BLE001
        print(f"WARN camera_params annotator 不可用: {exc}")
        has_cp = False

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(60):
        app.update()

    data = cparams.get_data() if has_cp else None
    print("\n" + "=" * 70)
    print("camera_params 顶层 keys")
    print("=" * 70)
    if data is None:
        print("（无数据）")
    else:
        for k, v in data.items():
            if k == "cameraViewTransform":
                arr = np.asarray(v)
                print(f"  {k:24s} shape={arr.shape} dtype={arr.dtype}")
                print(f"    {np.array2string(arr.reshape(-1, 4), precision=4, suppress_small=True)}")
            elif isinstance(v, dict):
                print(f"  {k:24s} dict keys={list(v.keys())}")
                for kk, vv in v.items():
                    print(f"      {kk:20s} = {vv}")
            else:
                print(f"  {k:24s} {type(v).__name__} = {v}")

    # ---- 换算像素内参 ----
    print("\n" + "=" * 70)
    print("换算成像素内参 K（训练/点云需要）")
    print("=" * 70)
    if data is not None:
        focal = float(data.get("cameraFocalLength") or 0.0)
        ap_h = float(data.get("cameraAperture", [0.0, 0.0])[0])   # 水平 aperture
        ap_v = float(data.get("cameraAperture", [0.0, 0.0, 0.0, 0.0])[1]) \
            if len(data.get("cameraAperture", [])) >= 2 else 0.0
        # 若 aperture 是高/宽比例形式，按 USD 约定换算
        if focal > 0 and ap_h > 0:
            fx = focal / ap_h * W
            fy = focal / ap_v * H if ap_v > 0 else fx
            print(f"  cameraFocalLength = {focal}  cameraAperture = "
                  f"{data.get('cameraAperture')}")
            print(f"  → fx = {fx:.2f}  fy = {fy:.2f}  (W={W} H={H})")
            print(f"  → cx = {W/2:.1f}  cy = {H/2:.1f}")
            K = np.array([[fx, 0, W / 2], [0, fy, H / 2], [0, 0, 1]])
            print(f"  K =\n{np.array2string(K, precision=3)}")
        else:
            print("  aperture/focal 字段缺失或为 0，无法换算")

    # ---- 与 USD 真实变换对照 ----
    print("\n" + "=" * 70)
    print("cameraViewTransform 布局判定（对照 USD 真实世界变换）")
    print("=" * 70)
    stage = ctx.get_stage()
    cache = UsdGeom.XformCache()
    cache.SetTime(0)
    prim = stage.GetPrimAtPath(cam.prim_path)
    mat = cache.GetLocalToWorldTransform(prim)
    usd_t = np.array([mat[3][0], mat[3][1], mat[3][2]])
    print(f"  USD 世界平移     : {np.round(usd_t, 4)}")
    if data is not None and "cameraViewTransform" in data:
        arr = np.asarray(data["cameraViewTransform"], dtype=np.float64)
        flat = arr.reshape(-1)
        # candidate A: row-major 4x4, 平移在 [3]/[7]/[11]
        a_t = np.array([flat[3], flat[7], flat[11]])
        # candidate B: column-major（转置），平移在 [12]/[13]/[14]
        b_t = np.array([flat[12], flat[13], flat[14]])
        print(f"  row-major 平移   : {np.round(a_t, 4)}  "
              f"误差 {np.linalg.norm(a_t - usd_t)*1000:.1f}mm")
        print(f"  column-major 平移: {np.round(b_t, 4)}  "
              f"误差 {np.linalg.norm(b_t - usd_t)*1000:.1f}mm")
        best = "row-major" if np.linalg.norm(a_t - usd_t) <= np.linalg.norm(b_t - usd_t) \
            else "column-major（需转置）"
        print(f"  → 判定: {best}")

    print("\n" + "=" * 70)
    print("其它数据健全性")
    print("=" * 70)
    r = rgb.get_data()
    d = depth.get_data()
    print(f"  rgb   shape={np.asarray(r).shape} dtype={np.asarray(r).dtype} "
          f"mean={np.asarray(r)[..., :3].mean():.1f}")
    print(f"  depth shape={np.asarray(d).shape} dtype={np.asarray(d).dtype} "
          f"min={np.nanmin(d):.3f} max={np.nanmax(d):.3f} "
          f"无效={int(np.sum(~np.isfinite(d) | (np.asarray(d) <= 0)))}")

    rp.destroy()
finally:
    app.close()
