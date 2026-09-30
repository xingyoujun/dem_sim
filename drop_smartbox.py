"""往 smartbox 上空扔一批物体, 看碰撞, 相机全程环绕。

门先打开, 让物体能真的落进格口里 —— 柜体碰撞是中空壳体, 不是实心块。
"""
import argparse, math, os, random, zlib, struct

parser = argparse.ArgumentParser()
parser.add_argument("--asset", default="/workspace/out/ship/smartbox_articulated.usd")
parser.add_argument("--outdir", default="/workspace/out/drop_frames")
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--fps", type=int, default=30)
parser.add_argument("--substeps", type=int, default=8)      # 物理 240Hz, 小物体接触才稳
parser.add_argument("--seconds", type=float, default=9.0)
parser.add_argument("--spp", type=int, default=12)
parser.add_argument("--seed", type=int, default=7)
args = parser.parse_args()

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True, "width": args.width, "height": args.height,
                         "renderer": "RaytracedLighting"})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf, Sdf

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
try:
    from isaacsim.core.prims import SingleArticulation as Articulation
except ImportError:
    from isaacsim.core.api.robots import Robot as Articulation


def write_png(path, rgb):
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y, :, :3].tobytes() for y in range(h))
    def chunk(tag, data):
        c = tag + data
        return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


world = World(physics_dt=1.0 / (args.fps * args.substeps), rendering_dt=1.0 / args.fps,
              stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
add_reference_to_stage(args.asset, "/World/box")

root_path = None
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.ArticulationRootAPI):
        root_path = str(p.GetPath()); break
print(f"[physics] articulation root = {root_path}", flush=True)

# ---------------------------------------------------------------- 物理材质
pmat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/bouncy")
m = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
m.CreateStaticFrictionAttr(0.5)
m.CreateDynamicFrictionAttr(0.45)
m.CreateRestitutionAttr(0.32)


def bind_phys(prim):
    api = UsdShade.MaterialBindingAPI.Apply(prim)
    api.Bind(pmat, UsdShade.Tokens.weakerThanDescendants, "physics")


# ---------------------------------------------------------------- 地面
gnd = UsdGeom.Cube.Define(stage, "/World/ground")
gnd.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gnd.GetPrim()); gx.ClearXformOpOrder()
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.15))
gx.AddScaleOp().Set(Gf.Vec3f(14.0, 14.0, 0.3))
gnd.CreateDisplayColorAttr([Gf.Vec3f(0.52, 0.53, 0.55)])
UsdPhysics.CollisionAPI.Apply(gnd.GetPrim())
bind_phys(gnd.GetPrim())

# ---------------------------------------------------------------- 掉落物
rng = random.Random(args.seed)
PALETTE = [(0.85,0.30,0.25), (0.95,0.65,0.20), (0.25,0.55,0.80),
           (0.35,0.65,0.40), (0.75,0.70,0.30), (0.55,0.40,0.70)]
objs = []
# 两波: 先近后远, 落地时间错开
WAVES = [(0.9, 2.0, 10), (5.5, 7.2, 8)]
oi = 0
for zlo, zhi, cnt in WAVES:
    for _ in range(cnt):
        kind = rng.choice(["cube", "sphere", "cube", "cylinder"])
        # 撒在柜子正面(-Y)一侧, 让一部分能进敞开的格口
        px = rng.uniform(-0.62, 0.62)
        py = rng.uniform(-0.72, 0.22)
        pz = rng.uniform(zlo, zhi) + 1.45
        path = f"/World/drops/obj_{oi:02d}"
        if kind == "cube":
            s = rng.uniform(0.075, 0.13)
            g = UsdGeom.Cube.Define(stage, path); g.CreateSizeAttr(1.0)
            scale = Gf.Vec3f(s, s * rng.uniform(0.7, 1.3), s)
        elif kind == "sphere":
            s = rng.uniform(0.042, 0.070)
            g = UsdGeom.Sphere.Define(stage, path); g.CreateRadiusAttr(1.0)
            scale = Gf.Vec3f(s, s, s)
        else:
            r_ = rng.uniform(0.035, 0.055)
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(1.0); g.CreateHeightAttr(2.4); g.CreateAxisAttr("Z")
            scale = Gf.Vec3f(r_, r_, r_)
        xf = UsdGeom.Xformable(g.GetPrim()); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(0,360), rng.uniform(0,360), rng.uniform(0,360)))
        xf.AddScaleOp().Set(scale)
        g.CreateDisplayColorAttr([Gf.Vec3f(*rng.choice(PALETTE))])
        pr = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(pr)
        UsdPhysics.RigidBodyAPI.Apply(pr)
        UsdPhysics.MassAPI.Apply(pr).CreateDensityAttr(420.0)
        bind_phys(pr)
        objs.append(path)
        oi += 1
print(f"[drops] 生成 {len(objs)} 个刚体", flush=True)

# ---------------------------------------------------------------- 灯光
dome = UsdLux.DomeLight.Define(stage, "/World/lights/dome")
dome.CreateIntensityAttr(600.0); dome.CreateColorAttr(Gf.Vec3f(0.92,0.94,1.0))
sun = UsdLux.DistantLight.Define(stage, "/World/lights/sun")
sun.CreateIntensityAttr(1900.0); sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-52.0, 0.0, 25.0))

# ---------------------------------------------------------------- 相机
cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * args.height / args.width)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
cx = UsdGeom.Xformable(cam.GetPrim()); cx.ClearXformOpOrder()
cam_op = cx.AddTransformOp()


def look_at(eye, tgt, up=Gf.Vec3d(0,0,1)):
    mm = Gf.Matrix4d(); mm.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*tgt), up)
    return mm.GetInverse()


rp = rep.create.render_product("/World/cam", (args.width, args.height))
annot = rep.AnnotatorRegistry.get_annotator("rgb")
annot.attach([rp])

robot = Articulation(prim_path=root_path, name="smartbox")
world.scene.add(robot)
world.reset()
robot.initialize()
names = list(robot.dof_names)
idx = {nm: i for i, nm in enumerate(names)}
n = len(names)
print(f"[art] dof={n} {names}", flush=True)
robot.get_articulation_controller().set_gains(
    kps=np.full(n, 60.0, dtype=np.float32), kds=np.full(n, 10.0, dtype=np.float32))

SIGN = {nm: (-1.0 if "left" in nm else 1.0) for nm in names}
q_open = np.array([math.radians(105.0) * SIGN[nm] for nm in names], dtype=np.float32)
q_home = np.zeros(n, dtype=np.float32)

TARGET = np.array([0.0, 0.0, 0.72])
RADIUS = 3.9
TOTAL = int(args.seconds * args.fps)
DOOR_FRAMES = int(1.2 * args.fps)

os.makedirs(args.outdir, exist_ok=True)
ctrl = robot.get_articulation_controller()
for f in range(TOTAL):
    t = f / TOTAL
    # 门在最开始 1.2s 打开, 之后保持
    a = min(1.0, f / DOOR_FRAMES)
    a = a * a * (3.0 - 2.0 * a)
    ctrl.apply_action(ArticulationAction(joint_positions=q_home + (q_open - q_home) * a))

    for _ in range(args.substeps):
        world.step(render=False)
    world.render()

    ang = math.radians(-115.0) + 2.0 * math.pi * t
    elev = math.radians(17.0 + 7.0 * math.sin(2.0 * math.pi * t))
    eye = TARGET + np.array([RADIUS * math.cos(ang) * math.cos(elev),
                             RADIUS * math.sin(ang) * math.cos(elev),
                             RADIUS * math.sin(elev)])
    cam_op.Set(look_at(eye, TARGET))

    rgb = annot.get_data()
    if rgb is not None and getattr(rgb, "size", 0):
        write_png(os.path.join(args.outdir, f"f_{f:05d}.png"), np.asarray(rgb))
    if f % 30 == 0:
        print(f"[sim] {f+1}/{TOTAL}", flush=True)

# 落点统计: 看有多少进了柜子
xc = UsdGeom.XformCache()
inside = above = ground = 0
for path in objs:
    p = stage.GetPrimAtPath(path)
    w = xc.GetLocalToWorldTransform(p)
    pos = np.array([w[3][0], w[3][1], w[3][2]])
    if 0.70 < pos[2] < 1.28 and -0.43 < pos[0] < 0.34 and -0.29 < pos[1] < 0.29:
        inside += 1
    elif pos[2] > 1.35:
        above += 1
    else:
        ground += 1
print(f"[result] 落入格口={inside}  停在柜顶={above}  落到地面/其它={ground}", flush=True)
print(f"[done] {TOTAL} frames -> {args.outdir}", flush=True)
sim_app.close()
