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

## B. X-flip 烘焙要点

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

### Play 后手瞬间转 90°（常见坑）

现象：Stage 里手相对 tcp 的 xform 仍是 Rz±90，但 Play 后刚体“跳”到另一朝向，无报错。

原因：`AssemblerFixedJoint` 若 `localRot0/1 = I`，PhysX 约束 `flange_world = tcp_world`，**忽略**父 prim `LinkerHand_o6` 上的编辑态 orient；仿真接管后手被焊回 tcp 零姿态 → 看起来转了 90°。

修法：FixedJoint 与挂载 xform **同一旋转**：

```usda
# body0=tcp, body1=flange；要 flange_world = tcp * Rz
quatf physics:localRot0 = (0.70710677, 0, 0, -0.70710677)  # 左 Rz=-90
quatf physics:localRot1 = (1, 0, 0, 0)
```

右手用 `localRot0` 的 +Z90。父 prim orient 可保留便于离线预览，但必须以 Joint 为准才能过 Play。

## J. 关节参数铁律（摘要）

主动：type=force, maxForce=10, damping=1, stiffness=3, targets=0。  
Mimic：dampingRatio=0, naturalFrequency=0, offset=0；gearing 用源值。  
详见 SKILL.md §0.1。

## I. 参考链接

- [Asset Structure](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_02_asset_structure.html)
- [Collider Pairs](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_04_collider_pairs.html)
- [Self-Collision Detector](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup/ext_isaacsim_robot_setup_collision_detector.html)
- 仓库内：`fasim-usd-bake-scale`、`fasim-robot-mujoco-physics`、`fasim-rg75-pad-convert`
