---
name: fasim-rg75-pad-convert-reference
---

# RG75 Pad Convert — Reference

## 命名约定

| 术语 | 含义 |
|------|------|
| **Pad** | RG75 根上的 variant：换夹指 tip；**选项名 = 用途/场景**（见 SKILL「Pad variant 命名」） |
| **side** | 关节改名 `left_`/`right_` 前缀 + sticky（相机 mount、pad arm） |
| **arm**（nested） | Pad 内部左右 CAD 选择；由 Side sticky，不进 Pad 下拉 |
| **gripper_base** | 新底座；旧名 **flange** |
| **pad_left / pad_right** | heavy 风格 tip 容器名（固定 tip 可在 base 下） |
| **pad** | type1 / load_type1 风格：两指下同名 tip |

旧 CAD 名 `left_left` / `left_right` / `right_left` / `right_right`：
- 第一个词 = **臂侧**（Left/Right 整爪）
- 第二个词 = **指侧**（该整爪的左/右 tip）
- 映射到新结构时写进 `left/` / `right/` 目录，不要把四套全塞进 Pad 下拉。

**用途名示例：** `heavy_carry`（重载搬运）、`load_type1`（原 PTC 负载指）、`type1`（既有通用指）。文件夹、入口 usda、variant 三者同名。

## 目录模板

### type1-like

```
robots/grippers/Jodell/RG75/payloads/Pad/<name>/
  <name>.usda
  pad.usdc                 # 或 geometries.usd + instances.usda + materials.usda + Textures/
```

### heavy-like + L/R CAD

```
robots/grippers/Jodell/RG75/payloads/Pad/<name>/
  <name>.usda              # mount + nested arm
  left/
    geometries.usd
    instances.usda
    materials.usda
    Textures/pbr.mdl       # 若有
    pad_left.usda
    pad_right.usda
  right/                   # 同上
```

`pad_*.usda` 最小形：

```usda
#usda 1.0
(
    defaultPrim = "root"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "root"
{
    def Xform "mesh" (
        instanceable = true
        prepend references = @./instances.usda@</Instances/<MeshPrim>>
    )
    {
        double3 xformOp:translate = (0, 0, 0)
        float3 xformOp:rotateXYZ = (...)   # 保留源 mesh 本地旋转
        double3 xformOp:scale = (0.001, 0.001, 0.001)  # 常有 mm→m
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
    }
}
```

## Side sticky 模板

追加到已有 `payloads/Side/left.usda`（right 则 `arm = "right"`）：

```usda
over "gripper_base"
{
    over "pad_left" (
        variants = { string arm = "left" }
    ) { }
}

over "right_finger"
{
    over "pad_right" (
        variants = { string arm = "left" }
    ) { }
}
```

Orbbec 等同理：sticky `mount`，且依赖 `side` 在 `Pad`/`Sensors` 前。

## mujoco pad collision 模板

追加到 `payloads/Physics/mujoco.usda`（路径按实际 mount）：

```usda
over "gripper_base"
{
    over "pad_left"
    {
        over "mesh" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
            delete apiSchemas = [
                "PhysxCollisionAPI",
                "PhysxConvexHullCollisionAPI",
                "PhysxConvexDecompositionCollisionAPI"
            ]
        )
        {
            token physics:approximation = "convexDecomposition"
            bool physics:collisionEnabled = 1
        }
    }
}

over "right_finger"
{
    over "pad_right"
    {
        over "mesh" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
            delete apiSchemas = [
                "PhysxCollisionAPI",
                "PhysxConvexHullCollisionAPI",
                "PhysxConvexDecompositionCollisionAPI"
            ]
        )
        {
            token physics:approximation = "convexDecomposition"
            bool physics:collisionEnabled = 1
        }
    }
}
```

Mount 层同时：

```usda
rel physics:simulationOwner   # 空 rel，清掉错误 simulationOwner
prepend rel material:binding:physics = </RG75/Looks/PadMaterials>
```

## 踩坑清单（实战）

### 运动学

1. **勿把所有 pad 当成 type1 mimic**  
   Heavy：真左 tip **固定在 base**，运动 tip 在 `right_finger`；`left_finger` 常为占位。弄错则开合行为与实物不符。

2. **Transformer 常把固定 tip 挂在 `flange` 下**  
   若该 tip 应随 finger 动，必须改 parent；若设计就是固定 tip，应挂 `gripper_base` 并在 mount 中显式表达。

3. **占位 finger 要 invisible**  
   否则视觉双指 + 实际单动，验收必糊。

### 几何与对齐

4. **优先 geometries bake，慎用旧 component usdc**  
   旧 `pad_*.usdc` 与合成层朝向不一致 → 相对 finger 的 mount 矩阵会爆炸；heavy 左臂曾整段错位，回退后用 bake 恢复。

5. **保留 mesh 内部 0.001 scale / 180° 旋转**  
   写在 `pad_*.usda` 的 mesh xform；mount 容器再叠 finger/base 相对 TF。

6. **双臂勿用 `scale=-1` 镜像整套 tip**  
   优先独立 CAD 或旋转。源资产指间偶发负 scale：PhysX 可保留；本 skill 不要求为 Newton bake。

6b. **先比 L/R mesh hash，再决定是否分 `left/`/`right/`**  
   文件级 `md5 geometries.usd` 常因元数据差 1 字节而不同；以 **Mesh points/indices hash** + **pad_* 文本块** 为准。load_type1（原 PTC）：全 mesh 相同、pad 块相同 → 单包，无 arm sticky。

6c. **PhysX 为 Pad 验收标准**  
   `heavy_carry`（`convexDecomposition` + 共享 mujoco collision overs，commit `ee8ed8d`）在 PhysX+ROS2 下可用。  
   **不要**在本流程里为 Newton 改 tip 为 `convexHull`、删 overs 或 chase `nconmax`。

6d. **Pad variant / 文件夹用用途名**  
   见 SKILL「Pad variant 命名」。`ptc` → `load_type1`；勿把裸工程代号留在最终 Pad 下拉。

7. **禁止为单款 pad 改 RG75↔TCP 全局朝向**  
   右臂差 Rz180 时：可试 **该 pad 局部 TF** 或父机 EE 层；改底座会带崩 type1 等所有 Pad。实测 pad 局部 Rz180 效果不好时，**回退并留给父机/EE 策略**，冻结已验证 TF。

8. **「看起来转了 180」不一定是错**  
   入口层与 payload 叠层、viewport 相机、旧整爪根朝向都可能导致观感差；以 **相对 base 的 tip TF + 开合** 为准。

### Variant / UX

9. **`variantSets`：`side` 在 `Pad` 前**  
   否则 Side sticky 压不过 Pad 默认 `arm=left`，右臂仍吃左 CAD。

10. **Pad 下拉只留一个名字**  
    `heavy_carry` 而非 `heavy_carry_left/right`；L/R 用 nested `arm` + Side。

11. **W2/M6 只动 payload/variant**  
    臂上已 sticky `side` 时，Pad 选一次即可；勿改父机非 payload 核心文件。

### 物理

12. **清 `simulationOwner`**  
    对齐 type1；否则碰撞/刚体归属错乱。

13. **Newton：剥 PhysX-only collision schema**  
    保留通用 `Physics*CollisionAPI`；在 mujoco 层 `delete` Physx* collision API。

14. **mujoco overs 路径必须存在于该 Pad**  
    其它 Pad 下 overs 应为无害 no-op；勿引用 type1 的 `soft/Mesh_*` 路径到 heavy。

### 工程卫生

15. **bake 完成并验收后删除临时 `RG75_*_Left/Right` 整爪目录**  
    只保留 `RG75/payloads/Pad/<name>/`。

16. **子模块**  
    改动在 `robots`（及嵌套 `FiveAges` 测试 env）内分别 commit；父仓指针另议。SSH 写权限按 **密钥绑定的 GitHub 账号**，与 `user.name` 无关。

17. **Git 脏文件假象**  
    若曾 push「改名删目录」但未 add 新 `heavy_carry/` 实体，本地会长期 `??`；收尾务必 `git add` 新 Pad 树。

18. **校验别污染金色入口**  
    `SetVariantSelection('load_type1')` 写在 root layer 会把选择存进 `RG75.usda`。用 `Usd.EditContext(stage, stage.GetSessionLayer())`，默认保持 `Pad=none`。

19. **Instanceable tip：Traverse 看不到 Mesh**  
    `instanceable=true` 时 mesh 在 prototype 下；用 `prim.GetPrototype()` / `Usd.PrimRange(proto)` 验收点数，勿误判「没 mesh」。

## 校验命令（可选）

有 `usd-core` / Isaac `python.sh` 时：

```bash
python - <<'PY'
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open('robots/grippers/Jodell/RG75/RG75.usda')
# 设 Pad / side 后检查 prim 是否存在、visibility、局部 translate
for path in ['/RG75/gripper_base/pad_left', '/RG75/right_finger/pad_right',
             '/RG75/left_finger/pad', '/RG75/joints/gripper_joint']:
    p = stage.GetPrimAtPath(path)
    print(path, 'OK' if p and p.IsValid() else 'MISSING')
PY
```

人工：Isaac 中切 `Pad` / `side` / `Physics=mujoco`，看开合与双臂挂载。
