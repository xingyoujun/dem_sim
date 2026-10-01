# smartbox_articulated.usd

由 `smartbox.stp`（Creo 导出的 STEP AP203 装配体 `C6007_SB_ID_M_ASM`）转换而来的
带关节 USD 资产。**单文件、无外部引用**，直接拷走即可使用。

## 基本参数

| 项 | 值 |
|---|---|
| 单位 | 米（`metersPerUnit = 1.0`） |
| 朝向 | Z-up |
| defaultPrim | `/SmartBox` |
| 尺寸 | 0.888 (X) × 0.578 (Y) × 1.405 (Z) m |
| 原点 | 底面在 z=0，XY 居中 |
| 正面朝向 | **-Y**（柜门朝 -Y 方向开） |
| 几何 | 4889 mesh / 约 208 万三角面 |
| 文件大小 | 20.6 MB |

## 关节

articulation root 在 `/SmartBox`，**4 个 DOF**，全是 revolute：

| DOF 名 | 铰链位置 | 限位 |
|---|---|---|
| `door_top_left_hinge`  | 左沿 X = -0.4245 | `[-110°, 0°]` |
| `door_btm_left_hinge`  | 左沿 X = -0.4245 | `[-110°, 0°]` |
| `door_top_right_hinge` | 右沿 X = +0.3345 | `[0°, +110°]` |
| `door_btm_right_hinge` | 右沿 X = +0.3345 | `[0°, +110°]` |

对开式：左列门向 **负向**转开，右列门向 **正向**转开；0° = 关闭。
每个关节已配 `DriveAPI`（stiffness 60 / damping 10 / maxForce 150），
可直接给位置目标驱动，无需另外设增益。

刚体 5 个（柜体 + 4 扇门），柜体经 `base_fixed` 固定关节焊到世界，作固定基座。

## 碰撞

- 门：真实几何的凸包（80 个 collider）
- 柜体：一个不可见的 Cube 近似外壳（`/SmartBox/cabinet/collision_proxy`）

柜体没有逐零件碰撞体（4889 个 mesh 全上碰撞对 PhysX 代价过大）。
若需精确抓取门把手等局部特征，需自行补碰撞体。

## 材质

`UsdPreviewSurface` 两个：柜体/门 `ShellWhite`，支架 `FrameGrey`。
绑定用了 `strongerThanDescendants`，覆盖了 CAD 自带的占位色。
**原始 STEP 无任何贴图**（AP203 格式不支持纹理）。

## 用法

```python
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation

add_reference_to_stage("smartbox_articulated.usd", "/World/box")
box = SingleArticulation(prim_path="/World/box", name="smartbox")
world.scene.add(box); world.reset(); box.initialize()

# box.dof_names ->
#   ['door_top_left_hinge','door_btm_left_hinge',
#    'door_top_right_hinge','door_btm_right_hinge']
```

放到场景里换位置，直接在 `/World/box` 上加 translate / rotateZ 即可
（不要用 `ClearXformOpOrder()` 之外的方式覆盖内部变换）。
