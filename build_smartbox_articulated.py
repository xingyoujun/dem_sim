#!/usr/bin/env python3
"""由 smartbox_m.usd (米制/非实例化/Y-up) 生成带 articulation 的 assets/src/smartbox_articulated_ref.usd
(引用 smartbox_m.usd), 再用 flatten_smartbox.py 展平成 assets/smartbox_articulated.usd。

做两件事:
  1. 材质: 柜体白色, 支架灰色 (用 strongerThanDescendants 盖掉 CAD 自带的占位色)
  2. 关节: 4 扇前门各一个 revolute, 侧开式 —— 左列门铰链在左沿, 右列门在右沿, 对开

坐标说明: 源件是 Y-up(Y 为竖直), 所有 body/joint 都在这个原始系里定义;
/SmartBox 顶层再统一 rotateX 90° 摆正并落地。纯旋转+平移, 不引入缩放, PhysX 安全。
"""
import os
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.join(HERE, "assets", "src")
SRC_NAME = "smartbox_m.usd"
SRC = os.path.join(SRC_DIR, SRC_NAME)
DST = os.path.join(SRC_DIR, "smartbox_articulated_ref.usd")
ASM = "C6007_SB_ID_M_ASM"

# 门: (关节名, 源 prim 名, 铰链侧)  —— L=铰链在门的 -X 沿, R=在 +X 沿
DOORS = [
    ("door_top_left",  "C6007_SB_DOOR_FRONT_TOP_1_M", "L"),
    ("door_btm_left",  "C6007_SB_DOOR_FRONT_BTM_1_M", "L"),
    ("door_top_right", "C6007_SB_DOOR_FRONT_TOP_2_M", "R"),
    ("door_btm_right", "C6007_SB_DOOR_FRONT_BTM_2_M", "R"),
]
OPEN_DEG = 110.0
DOOR_MASS = 2.5

src = Usd.Stage.Open(SRC)
xc = UsdGeom.XformCache()
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

# ---- 在源件里定位 4 扇门, 记录父变换与包围盒 ----
info = {}
for p in src.Traverse():
    for jn, pname, side in DOORS:
        if p.GetName() == pname:
            r = bc.ComputeWorldBound(p).ComputeAlignedRange()
            lo = np.array(r.GetMin(), dtype=float)
            hi = np.array(r.GetMax(), dtype=float)
            Wp = np.array(xc.GetLocalToWorldTransform(p.GetParent()), dtype=float)
            info[jn] = dict(path=str(p.GetPath()), lo=lo, hi=hi, Wp=Wp, side=side)
missing = [d[0] for d in DOORS if d[0] not in info]
assert not missing, f"没找到: {missing}"

# ---- 新 stage ----
stage = Usd.Stage.CreateNew(DST)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

root = UsdGeom.Xform.Define(stage, "/SmartBox")
stage.SetDefaultPrim(root.GetPrim())
rx = UsdGeom.Xformable(root.GetPrim())
rx.ClearXformOpOrder()
# 最后列出的先作用: rotateX 摆正 -> translate 落地
t_root = rx.AddTranslateOp()
rx.AddRotateXOp().Set(90.0)

# ---- 柜体 ----
cab = UsdGeom.Xform.Define(stage, "/SmartBox/cabinet")
cab_geo = UsdGeom.Xform.Define(stage, "/SmartBox/cabinet/geo")
cab_geo.GetPrim().GetReferences().AddReference(f"./{SRC_NAME}")

# 门从柜体里摘掉, 否则会被算进柜体刚体
for jn, d in info.items():
    sub = d["path"].split("/", 2)[2]          # 去掉 /smartbox_m/
    stage.OverridePrim(f"/SmartBox/cabinet/geo/{sub}").SetActive(False)

# ---- 4 扇门各自成体 ----
def mat4(a):
    m = Gf.Matrix4d()
    for i in range(4):
        for j in range(4):
            m[i, j] = float(a[i][j])
    return m

for jn, d in info.items():
    body = UsdGeom.Xform.Define(stage, f"/SmartBox/{jn}")
    bx = UsdGeom.Xformable(body.GetPrim())
    bx.ClearXformOpOrder()
    bx.AddTransformOp().Set(mat4(d["Wp"]))     # 补回父级累积变换
    g = UsdGeom.Xform.Define(stage, f"/SmartBox/{jn}/geo")
    g.GetPrim().GetReferences().AddReference(f"./{SRC_NAME}", Sdf.Path(d["path"]))

# ---- 落地: 量一次再定 root 平移 ----
t_root.Set(Gf.Vec3d(0, 0, 0))
bc2 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
r = bc2.ComputeWorldBound(root.GetPrim()).ComputeAlignedRange()
lo = np.array(r.GetMin(), dtype=float)
hi = np.array(r.GetMax(), dtype=float)
t_root.Set(Gf.Vec3d(-(lo[0] + hi[0]) / 2.0, -(lo[1] + hi[1]) / 2.0, -lo[2]))

# ---- 材质: 柜体白 / 支架灰 ----
def make_mat(path, color, rough, metal):
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m

white = make_mat("/SmartBox/Looks/ShellWhite", (0.902, 0.906, 0.914), 0.42, 0.0)
grey = make_mat("/SmartBox/Looks/FrameGrey", (0.400, 0.412, 0.427), 0.55, 0.15)

def bind_strong(prim_path, mat):
    p = stage.OverridePrim(prim_path)
    api = UsdShade.MaterialBindingAPI.Apply(p)
    # 必须 strongerThanDescendants, 否则各 mesh 自带的 CAD 占位色更强
    api.Bind(mat, UsdShade.Tokens.strongerThanDescendants)

bind_strong(f"/SmartBox/cabinet/geo/{ASM}/C6007_SB_BOX_M", white)
bind_strong(f"/SmartBox/cabinet/geo/{ASM}/C6007_SB_LEG_WHOLE_M", grey)
for jn in info:
    bind_strong(f"/SmartBox/{jn}/geo", white)

# ---- 物理 ----
UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
# 不配 PhysxArticulationAPI: articulation 里直接铰接的两个体本来就自动过滤碰撞,
# 门和柜体不会互相干涉

UsdPhysics.RigidBodyAPI.Apply(cab.GetPrim())
# 柜体焊死到世界, 当固定基座
fixed = UsdPhysics.FixedJoint.Define(stage, "/SmartBox/joints/base_fixed")
fixed.CreateBody1Rel().SetTargets([cab.GetPrim().GetPath()])

# 柜体碰撞: 4889 个 mesh 全上碰撞对 PhysX 太重, 用一组盒体近似。
# 不能用一个实心盒 —— 那样门开了物体也进不去格口。这里做成中空壳体 + 隔板,
# 前面敞开, 四个格口是真的空腔。
# 坐标是原始系(Y 竖直), 柜体相对 /SmartBox 无附加变换, 直接用。
CAB_X0, CAB_X1 = -0.430, 0.430
CAB_Z0, CAB_Z1 = -0.2808, 0.280
CAV_Y0, CAV_Y1 = 0.002, 0.560      # 门的上下沿 = 空腔高度
DOOR_X0, DOOR_X1 = -0.4245, 0.3345  # 门的左右沿
SEAM_X = -0.045                     # 两门中缝 = 竖隔板
SHELF_Y = 0.281                     # 上下两排交界 = 横隔板
BACK_Z = -0.2608

COLLIDERS = {
    # 壳体
    "base_slab": (CAB_X0, -0.029, CAB_Z0,   CAB_X1, CAV_Y0,  CAB_Z1),
    "top_block": (CAB_X0, CAV_Y1, CAB_Z0,   CAB_X1, 0.705,   CAB_Z1),
    "back_wall": (CAB_X0, CAV_Y0, CAB_Z0,   CAB_X1, CAV_Y1,  BACK_Z),
    "left_wall": (CAB_X0, CAV_Y0, BACK_Z,   -0.410, CAV_Y1,  CAB_Z1),
    "panel_col": (DOOR_X1, CAV_Y0, BACK_Z,  CAB_X1, CAV_Y1,  CAB_Z1),
    # 内部隔板
    "divider":   (SEAM_X - 0.011, CAV_Y0, BACK_Z, SEAM_X + 0.011, CAV_Y1, CAB_Z1),
    "shelf":     (DOOR_X0, SHELF_Y - 0.011, BACK_Z, DOOR_X1, SHELF_Y + 0.011, CAB_Z1),
    # 支架: 4 根立柱 + 顶框 + 2 根下横撑
    "leg_fl":    (-0.430, -0.650,  0.248,  -0.398, -0.032,  0.280),
    "leg_fr":    ( 0.398, -0.650,  0.248,   0.430, -0.032,  0.280),
    "leg_bl":    (-0.430, -0.650, -0.280,  -0.398, -0.032, -0.248),
    "leg_br":    ( 0.398, -0.650, -0.280,   0.430, -0.032, -0.248),
    "frame_top": (-0.430, -0.032, -0.280,   0.430,  0.000,  0.280),
    "rail_l":    (-0.430, -0.532, -0.248,  -0.398, -0.500,  0.248),
    "rail_r":    ( 0.398, -0.532, -0.248,   0.430, -0.500,  0.248),
}
prox = UsdGeom.Scope.Define(stage, "/SmartBox/cabinet/collision")
UsdGeom.Imageable(prox.GetPrim()).CreateVisibilityAttr("invisible")
for nm, (x0, y0, z0, x1, y1, z1) in COLLIDERS.items():
    lo = np.array([x0, y0, z0]); hi = np.array([x1, y1, z1])
    cube = UsdGeom.Cube.Define(stage, f"/SmartBox/cabinet/collision/{nm}")
    cube.CreateSizeAttr(1.0)
    cxf = UsdGeom.Xformable(cube.GetPrim())
    cxf.ClearXformOpOrder()
    cxf.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2.0)))
    cxf.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
print(f"[collision] 柜体盒体代理 x{len(COLLIDERS)}")

for jn, d in info.items():
    bp = stage.GetPrimAtPath(f"/SmartBox/{jn}")
    UsdPhysics.RigidBodyAPI.Apply(bp)
    mass = UsdPhysics.MassAPI.Apply(bp)
    mass.CreateMassAttr(DOOR_MASS)
    # 门就是块平板, 凸包足够
    for m in Usd.PrimRange(stage.GetPrimAtPath(f"/SmartBox/{jn}/geo")):
        if m.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(m)
            mc = UsdPhysics.MeshCollisionAPI.Apply(m)
            mc.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

    lo, hi = d["lo"], d["hi"]
    # 铰链在门的外侧竖边, 取门板靠柜体那一面(z 小的一侧), 轴沿 Y(源件竖直方向)
    hx = lo[0] if d["side"] == "L" else hi[0]
    hinge = np.array([hx, (lo[1] + hi[1]) / 2.0, lo[2]])

    j = UsdPhysics.RevoluteJoint.Define(stage, f"/SmartBox/joints/{jn}_hinge")
    j.CreateBody0Rel().SetTargets([cab.GetPrim().GetPath()])
    j.CreateBody1Rel().SetTargets([bp.GetPath()])
    j.CreateAxisAttr(UsdPhysics.Tokens.y)
    j.CreateLocalPos0Attr(Gf.Vec3f(*hinge))                       # 柜体本身无附加变换
    Winv = np.linalg.inv(d["Wp"])
    h4 = np.array([hinge[0], hinge[1], hinge[2], 1.0]) @ Winv
    j.CreateLocalPos1Attr(Gf.Vec3f(*h4[:3]))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    # 左门向 -Y 转开, 右门向 +Y 转开
    if d["side"] == "L":
        j.CreateLowerLimitAttr(-OPEN_DEG); j.CreateUpperLimitAttr(0.0)
    else:
        j.CreateLowerLimitAttr(0.0); j.CreateUpperLimitAttr(OPEN_DEG)

    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(60.0)
    drv.CreateDampingAttr(10.0)
    drv.CreateMaxForceAttr(150.0)
    drv.CreateTargetPositionAttr(0.0)

stage.GetRootLayer().Save()

# ---- 复查 ----
st = Usd.Stage.Open(DST)
bc3 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
r = bc3.ComputeWorldBound(st.GetDefaultPrim()).ComputeAlignedRange()
print("bbox min=", tuple(round(v, 4) for v in r.GetMin()))
print("bbox max=", tuple(round(v, 4) for v in r.GetMax()))
print("size    =", tuple(round(v, 4) for v in r.GetSize()))
nj = [p for p in st.Traverse() if "Joint" in str(p.GetTypeName())]
nb = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
ncol = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
print(f"joints={len(nj)} rigidbodies={len(nb)} colliders={len(ncol)}")
for p in nj:
    print("   ", p.GetName(), p.GetTypeName())
