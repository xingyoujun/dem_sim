"""四面向 smartbox 发射 cube, 看碰撞, 相机全程环绕。

正面(-Y)那组瞄准 4 个格口内部 —— 格口开口是水平朝前的, 垂直落体进不去,
必须给水平初速度才打得进。其余三面打在柜体/敞开的门板上。
"""
import argparse, math, os, random, zlib, struct

ROOT = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument("--asset", default=os.path.join(ROOT, "assets/smartbox_articulated.usd"))
parser.add_argument("--outdir", default=os.path.join(ROOT, "out/shoot_frames"))
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--fps", type=int, default=30)
parser.add_argument("--substeps", type=int, default=8)
parser.add_argument("--seconds", type=float, default=9.0)
parser.add_argument("--spp", type=int, default=12)
parser.add_argument("--seed", type=int, default=11)
args = parser.parse_args()

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": True, "width": args.width, "height": args.height,
                         "renderer": "RaytracedLighting"})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
try:
    from isaacsim.core.prims import SingleArticulation as Articulation
except ImportError:
    from isaacsim.core.api.robots import Robot as Articulation

G = 9.81


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

pmat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/m")
pm = UsdPhysics.MaterialAPI.Apply(pmat.GetPrim())
pm.CreateStaticFrictionAttr(0.5)
pm.CreateDynamicFrictionAttr(0.45)
pm.CreateRestitutionAttr(0.12)   # 别太弹, 否则射进格口又被后壁弹出来


def bind_phys(prim):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        pmat, UsdShade.Tokens.weakerThanDescendants, "physics")


gnd = UsdGeom.Cube.Define(stage, "/World/ground")
gnd.CreateSizeAttr(1.0)
gx = UsdGeom.Xformable(gnd.GetPrim()); gx.ClearXformOpOrder()
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.15))
gx.AddScaleOp().Set(Gf.Vec3f(16.0, 16.0, 0.3))
gnd.CreateDisplayColorAttr([Gf.Vec3f(0.52, 0.53, 0.55)])
UsdPhysics.CollisionAPI.Apply(gnd.GetPrim())
bind_phys(gnd.GetPrim())

# ---------------------------------------------------------------- 目标点
# 格口中心(世界系): 左列 x=-0.235 右列 x=+0.145; 下排 z=0.841 上排 z=1.121; 进深 y≈0
CELLS = [(-0.235, 0.0, 0.841), (0.145, 0.0, 0.841),
         (-0.235, 0.0, 1.121), (0.145, 0.0, 1.121)]

rng = random.Random(args.seed)
PALETTE = [(0.85,0.30,0.25), (0.95,0.65,0.20), (0.25,0.55,0.80),
           (0.35,0.65,0.40), (0.80,0.72,0.28), (0.58,0.40,0.72)]


def aim(P, T, speed):
    """弹道补偿: 抵消飞行途中的重力下坠"""
    d = np.array(T) - np.array(P)
    t = max(np.linalg.norm(d) / speed, 1e-3)
    v = d / t
    v[2] += 0.5 * G * t
    return v


# (方向名, 发射点基准, 目标列表)
EMITTERS = [
    ("front", np.array([0.0, -1.0, 0.0]), CELLS),                       # -Y, 瞄格口
    ("back",  np.array([0.0,  1.0, 0.0]), [(0.0, 0.0, 1.05), (0.2, 0.0, 0.85)]),
    ("left",  np.array([-1.0, 0.0, 0.0]), [(-0.2, -0.1, 1.10), (-0.2, -0.1, 0.85)]),
    ("right", np.array([ 1.0, 0.0, 0.0]), [(0.15, -0.1, 1.10), (0.15, -0.1, 0.85)]),
]

objs = []
oi = 0
for name, dirv, targets in EMITTERS:
    n_shot = 6 if name == "front" else 4
    for k in range(n_shot):
        T = list(targets[k % len(targets)])
        T[0] += rng.uniform(-0.07, 0.07)
        T[2] += rng.uniform(-0.05, 0.05)
        # 飞行时间越短, 重力补偿的仰角越小, 弹道越平 -> 才射得进水平开口。
        # 正面这组必须走近距高速; 其余方向可以远一点, 到达时间自然错开成两波。
        if name == "front":
            dist = rng.uniform(1.8, 2.4) if k < n_shot // 2 else rng.uniform(3.4, 4.4)
            speed = rng.uniform(6.4, 8.2)
        else:
            dist = rng.uniform(2.6, 3.4) if k < n_shot // 2 else rng.uniform(4.6, 5.8)
            speed = rng.uniform(5.0, 7.0)
        P = np.array(T) + dirv * dist
        P[2] += rng.uniform(-0.10, 0.30)
        P[0] += rng.uniform(-0.20, 0.20) if abs(dirv[0]) < 0.5 else 0.0
        P[1] += rng.uniform(-0.20, 0.20) if abs(dirv[1]) < 0.5 else 0.0
        v = aim(P, T, speed)

        s = rng.uniform(0.062, 0.090) if name == "front" else rng.uniform(0.075, 0.115)
        path = f"/World/shots/cube_{oi:02d}"
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(g.GetPrim()); xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*P))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rng.uniform(0,360), rng.uniform(0,360), rng.uniform(0,360)))
        xf.AddScaleOp().Set(Gf.Vec3f(s, s * rng.uniform(0.8,1.2), s))
        g.CreateDisplayColorAttr([Gf.Vec3f(*rng.choice(PALETTE))])
        pr = g.GetPrim()
        UsdPhysics.CollisionAPI.Apply(pr)
        rb = UsdPhysics.RigidBodyAPI.Apply(pr)
        rb.CreateVelocityAttr(Gf.Vec3f(*v.astype(float)))
        rb.CreateAngularVelocityAttr(Gf.Vec3f(rng.uniform(-180,180), rng.uniform(-180,180), rng.uniform(-180,180)))
        UsdPhysics.MassAPI.Apply(pr).CreateDensityAttr(400.0)
        bind_phys(pr)
        objs.append((path, name))
        oi += 1
print(f"[shots] 发射 {len(objs)} 个 cube: " +
      ", ".join(f"{d}x{sum(1 for _,dd in objs if dd==d)}" for d in ['front','back','left','right']), flush=True)

dome = UsdLux.DomeLight.Define(stage, "/World/lights/dome")
dome.CreateIntensityAttr(600.0); dome.CreateColorAttr(Gf.Vec3f(0.92,0.94,1.0))
sun = UsdLux.DistantLight.Define(stage, "/World/lights/sun")
sun.CreateIntensityAttr(1900.0); sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-52.0, 0.0, 25.0))

cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateFocalLengthAttr(22.0)
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
n = len(names)
print(f"[art] dof={n}", flush=True)
robot.get_articulation_controller().set_gains(
    kps=np.full(n, 60.0, dtype=np.float32), kds=np.full(n, 10.0, dtype=np.float32))
SIGN = {nm: (-1.0 if "left" in nm else 1.0) for nm in names}
q_open = np.array([math.radians(108.0) * SIGN[nm] for nm in names], dtype=np.float32)
# 直接把门摆到全开位再开始, 不做开门动画 —— 否则 cube 到得比门开得快
robot.set_joint_positions(q_open)
world.step(render=False)
print("[art] 门置于全开位", flush=True)

TARGET = np.array([0.0, 0.0, 0.72])
RADIUS = 4.1
TOTAL = int(args.seconds * args.fps)

os.makedirs(args.outdir, exist_ok=True)
ctrl = robot.get_articulation_controller()
for f in range(TOTAL):
    t = f / TOTAL
    ctrl.apply_action(ArticulationAction(joint_positions=q_open))
    for _ in range(args.substeps):
        world.step(render=False)
    world.render()

    ang = math.radians(-100.0) + 2.0 * math.pi * t
    elev = math.radians(15.0 + 7.0 * math.sin(2.0 * math.pi * t))
    eye = TARGET + np.array([RADIUS*math.cos(ang)*math.cos(elev),
                             RADIUS*math.sin(ang)*math.cos(elev),
                             RADIUS*math.sin(elev)])
    cam_op.Set(look_at(eye, TARGET))
    rgb = annot.get_data()
    if rgb is not None and getattr(rgb, "size", 0):
        write_png(os.path.join(args.outdir, f"f_{f:05d}.png"), np.asarray(rgb))
    if f % 30 == 0:
        print(f"[sim] {f+1}/{TOTAL}", flush=True)

xc = UsdGeom.XformCache()
stat = {}
for path, src in objs:
    w = xc.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
    x, y, z = w[3][0], w[3][1], w[3][2]
    if -0.43 < x < 0.34 and -0.28 < y < 0.28 and 0.70 < z < 1.27:
        k = "格口内"
    elif z > 1.35:
        k = "柜顶"
    else:
        k = "地面/其它"
    stat.setdefault(src, {}).setdefault(k, 0)
    stat[src][k] += 1
print("[result] 按发射方向统计落点:", flush=True)
for src in ['front','back','left','right']:
    if src in stat:
        print(f"    {src:6} -> " + "  ".join(f"{k}={v}" for k, v in sorted(stat[src].items())), flush=True)
tot_in = sum(d.get("格口内", 0) for d in stat.values())
print(f"[result] 共 {tot_in}/{len(objs)} 个进入格口", flush=True)
print(f"[done] {TOTAL} frames -> {args.outdir}", flush=True)
sim_app.close()
