# FaSim DexHand Asset — Reference

配套 `SKILL.md`。细节步骤与 o6 实战记录。

## A. 源资产审计命令

```bash
USD_LIBS=$(echo "$HOME"/isaacsim/extscache/omni.usd.libs-*-lx64.r.cp312 | awk '{print $1}')
export PYTHONPATH="$USD_LIBS${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$USD_LIBS/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
PY="$HOME/isaacsim/kit/python/bin/python3"

"$PY" - <<'PY'
from pxr import Usd, UsdGeom
for path, root in [
    (".../LinkerHand_o6_left.usda", "LinkerHand_o6_left"),
    (".../LinkerHand_o6_right.usda", "LinkerHand_o6_right"),
]:
    st = Usd.Stage.Open(path)
    print("====", path)
    for prim in st.Traverse():
        if "/collisions" not in str(prim.GetPath()):
            continue
        if not prim.IsA(UsdGeom.Xformable):
            continue
        for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                s = op.Get()
                if s and min(s) < 0:
                    print("NEG", prim.GetPath(), s)
PY
```

列出驱动关节 / mimic：扫 `payloads/Physics/physics.usda`、`physx.usda`。

## B. 镜像轴（转换前与用户确认）

| 轴 | 例 | Hand/right | 碰撞 | Side/right | 臂右 EE |
|----|-----|------------|------|------------|---------|
| X | o6 | 根 `(1,1,1)`；link/视觉 X | `*_xflip`，局部 `(1,1,1)` | 右手 frames + 改名 | scale `(1,1,1)`；FixedJoint=挂载 TF |
| Y | o7 | 根 `(1,1,1)`；link `Sy R Sy`；视觉局部 Sy=-1 | `*_yflip`，局部 `(1,1,1)` | 改名 + **Y 镜像** joint/mass；**X/Z 轴再 ×Ry(180°) 纠转向** | 同上；**禁止**挂载/根 Sy=-1 |

### Play 后挂载跳变（必查）

| 现象 | 原因 | 修法 |
|------|------|------|
| Play 后整手相对 tcp 转/移（编辑态仍显示原 xform） | `AssemblerFixedJoint` localPos/Rot ≠ 挂载 xform；PhysX 忽略父 prim orient | FixedJoint 与挂载 **同一** T/R |
| 仅右手 Play 跳、左手正常；编辑态右手已镜像 | 根或挂载 `Sy=-1`，PhysX 丢掉负 scale | §2B 烘焙到根 `(1,1,1)` |

### 重力下拇指慢转

主动关节 force 驱动过软时，挂臂后拇指会在重力下缓慢偏离 target=0。可加大该型号拇指 `stiffness` / `maxForce` / `damping`（o7 实测抗下垂：K=50, F=80, D=5）。**不要**为此改 mimic 的 NF/dampingRatio。

## B2. X-flip 烘焙要点（o6）

1. 只烘焙**碰撞**用到的 mesh（常 `mesh_2`/`mesh_5`/`mesh_6` 等）。
2. 新 prim：`/Geometries/{name}_xflip/{name}` —— **子 Mesh 名与源相同**。
3. `instances` 碰撞 instance 改 `@./geometries.usd@</Geometries/{name}_xflip>`。
4. `base` 碰撞 `scale=(1,1,1)`；视觉可留 `(-1,1,1)`。
5. 更新 xflip mesh 的 `extent`，否则 BBoxCache 会偏。

## C. Side=right 引用切换

必须在**入口** `variantSet "Side"` 的 `"right"` 分支：

```usda
delete references = @./payloads/base.usda@
prepend references = @./payloads/Hand/right/base.usda@
prepend payload = @./payloads/Side/right.usda@
```

不要在 `Side/right.usda` 里再 payload 一层右手 base（易与左手 reference 叠双）。

## D. 关节改名模板

```text
JOINTS = [index_joint, index_dip, middle_joint, middle_dip,
          pinky_joint, pinky_dip, ring_joint, ring_dip,
          thumb_joint2, thumb_joint1, thumb_dip]
DRIVEN = {index_joint, middle_joint, pinky_joint, ring_joint,
          thumb_joint2, thumb_joint1}
MIMIC = {
  index_dip: (index_joint, gear),
  ...
  thumb_dip: (thumb_joint1, gear),
}
```

Side 文件内：

1. 全部 JOINTS → `active=false`
2. `def` `{prefix}{name}`，prefix=`left_hand_` / `right_hand_`
3. mimic 的 `referenceJoint` → 同 prefix 的 leader

运控对齐：`config/ros2_control/templates/o6.side.yaml`：

```yaml
joints:
  - {side_}hand_thumb_joint1
  - {side_}hand_thumb_joint2
  - {side_}hand_index_joint
  ...
```

## E. 入口根层禁忌

Kit 切 Side 时若 edit target 在根层，会把 `left_hand_*` / `right_hand_*` 的 over 写进入口 USDA。  
提交前 grep 入口：

```bash
grep -E 'left_hand_|right_hand_' {Name}.usda
```

除文档字符串外不应出现；否则 `Side=default` 会有幽灵 Over prim。

## F. FilteredPairs 操作（Isaac UI）

1. 打开统一入口，`Physics=physx`，先 `enabledSelfCollisions=1`。
2. Robotics → Robot Self-Collision Detector。
3. 静止手型下列出重叠对；对「结构重叠」勾 Filtered Pair。
4. Authoring layer 设为 `payloads/Physics/physics.usda`（或 physx）。
5. Save；左右 Side 各 Play 握拳回归。

常见过滤：掌-近指根、相邻指近端橡胶垫、拇指座与掌。  
不要过滤指尖-物体所需接触。

## G. 与夹爪 / 臂的边界

| 事项 | DexHand skill | 另做 |
|------|---------------|------|
| Hand/Side/Physics | ✓ | |
| 负 scale 烘焙 | ✓ | |
| 臂 EE payload / sticky Side | | Marvin 类挂载 |
| 臂 tcp → flange FixedJoint | | 挂载任务 |
| 挂载 Rz（左 -90 / 右 +90） | | 挂载任务（o6 已实测） |

## H. o6 挂载 TF（记录，非本 skill 产出）

Marvin `link7/tcp/LinkerHand_o6`：

- Left / default：`orient Z = -90°` → quat `(0.7071, 0, 0, -0.7071)`
- Right：`orient Z = +90°` → quat `(0.7071, 0, 0, +0.7071)`
- translate / scale 恒等

### Play 后手瞬间转 90° / 位置跳变（常见坑）

现象：Stage 里手相对 tcp 的 xform 仍是 Rz±90，但 Play 后刚体“跳”到另一朝向，无报错。

原因：`AssemblerFixedJoint` 若 `localRot0/1 = I`，PhysX 约束 `flange_world = tcp_world`，**忽略**父 prim 上的编辑态 orient；仿真接管后手被焊回 tcp 零姿态。

修法：FixedJoint 与挂载 xform **同一旋转与平移**：

```usda
# body0=tcp, body1=flange；要 flange_world = tcp * T * Rz
point3f physics:localPos0 = (0, 0, -0.0022)   # 与 xformOp:translate 一致
quatf physics:localRot0 = (0.70710677, 0, 0, 0.70710677)  # Rz=+90 例
quatf physics:localRot1 = (1, 0, 0, 0)
```

父 prim orient 可保留便于离线预览，但必须以 Joint 为准才能过 Play。

右手若还带根/挂载负 scale：即使 FixedJoint 对齐，Play 仍可能跳 —— 见上文「根负 scale 不可上臂」，先做 Y 安全烘焙。

## J. 接触参数（摘要，以 o7 为准）

| | 四指 | 拇指基线 | Mimic | 碰撞 | Solver |
|--|------|----------|-------|------|--------|
| | D=1 K=5 F=20 | D=1 K=10 F=30 | NF=0 DR=0 | mesh `convexHull` | 64 / 1 |

拇指挂臂抗重力可再加大（例 K=50 F=80 D=5）。详见 SKILL.md §0.1。

## K. PhysX-safe Y-mirror bake（原 `_mirror_right.py`，勿放进资产目录）

**问题：** 根 `Sy=-1` 编辑态正确，PhysX Play 丢掉负 scale → 挂臂跳变。

**公式：** `Sy = diag(1,-1,1)`；`sy_vec(v)=(x,-y,z)`；`sy_quat(q)` ← `Sy·R·Sy`。

### Checklist

1. 从 `Hand/left` 拷到 `Hand/right`；根 scale → `(1,1,1)`。
2. `geometries.usd`：对每个碰撞 mesh 建 `mesh_N_yflip`（点/法线 Y 取反、翻面、更新 extent）。
3. `instances.usda`：碰撞 instance 改引用 `*_yflip`。
4. 每个 link：`t' = sy_vec(t)`，`R' = sy_quat(R)`，scale 取绝对值。
5. 视觉叶节点局部 `Sy=-1`（复用左手 mesh）。
6. 碰撞叶节点局部 `(1,1,1)`。

### Side/right 关节（原 `_build_unified.write_side`）

1. 从共享 `payloads/Physics/{physics,physx}.usda` 取关节，merge PhysX 进 `def`。
2. `localPos/Rot`、COM、`principalAxes` 做 Y 镜像。
3. 前缀 `right_hand_*`；无前缀 `active=false`。
4. **X/Z 轴纠转向：** `axis!=Y` 时 `localRot0/1 ← q ⊗ Ry(180°)`，`Ry180=(0,0,1,0)`（w,x,y,z）。手指多为 Y，不受影响；o7 拇指 `joint2=Z`、`joint3=X` 必须做。

### Isaac 片段

```python
from pxr import Gf, UsdGeom, Vt
SY = Gf.Matrix3d(1, 0, 0, 0, -1, 0, 0, 0, 1)

def sy_vec(v):
    return type(v)(v[0], -v[1], v[2])

def sy_quat(q: Gf.Quatd) -> Gf.Quatd:
    return (SY * Gf.Matrix3d(q) * SY).ExtractRotation().GetQuat()

def bake_yflip_mesh(mesh: UsdGeom.Mesh) -> None:
    pts = [Gf.Vec3f(p[0], -p[1], p[2]) for p in mesh.GetPointsAttr().Get()]
    mesh.GetPointsAttr().Set(Vt.Vec3fArray(pts))
    # normals Y-flip + renormalize; reverse each face winding; update extent
```

## L. 资产目录纪律

**可保留：** `{Name}.usda`、`Textures/`、`payloads/**`

**必须删（勿提交进资产树）：** `*_left/`、`*_right/` 源树、`_build_*.py`、`_mirror_*.py`、`__pycache__/`、`transform_report.json`、临时 `env/test.usda`

## M. o6 对齐 o7 接触（断点续作记录）

验收标准（用户）：轻压桌不飞、无手指吸附、可抓桌上魔方；大力戳指抽搐/拽倒可接受。

| 项 | 曾踩坑 | 应对齐 |
|----|--------|--------|
| 碰撞 | 胶囊/方盒简化；`convexDecomposition` | **mesh + convexHull**（同 o7） |
| Mimic | NF=20 / DR=1 | **0 / 0** |
| Drive | 过软或过硬乱拧 | 四指 5/1/20，拇指 10/1/30 |
| Articulation | solver 96/4；per-link maxVel 夹死 | **64/1**；不要靠限速当主手段 |
| 质量 | 指节抬到几十克 | 保持源克级质量 |

## I. 参考链接

- [Asset Structure](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_02_asset_structure.html)
- [Collider Pairs](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_04_collider_pairs.html)
- [Self-Collision Detector](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup/ext_isaacsim_robot_setup_collision_detector.html)
- 仓库内：`fasim-usd-bake-scale`、`fasim-robot-mujoco-physics`、`fasim-rg75-pad-convert`
