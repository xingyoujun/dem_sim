"""机器人 + 重建操作台(notray_tray) + articulated smartbox 的组合场景, 渲几张确认图。

    ./python.sh scene_smartbox.py
"""
import argparse, math, os

ROOT = os.path.dirname(os.path.abspath(__file__))

parser = argparse.ArgumentParser()
parser.add_argument("--robot", default=os.path.join(ROOT, "assets/alphabot2/alphabot2_isaac6_clean.usda"))
parser.add_argument("--scene", default=os.path.join(ROOT, "assets/notray_tray.usd"))
parser.add_argument("--box", default=os.path.join(ROOT, "assets/smartbox_articulated.usd"))
parser.add_argument("--outdir", default=os.path.join(ROOT, "out/scene_views"))
parser.add_argument("--width", type=int, default=1280)
parser.add_argument("--height", type=int, default=720)
parser.add_argument("--spp", type=int, default=16)
parser.add_argument("--views", type=int, default=3)
# smartbox_articulated.usd 已经是 米制/Z-up/底面在原点, 这里只需摆位置
parser.add_argument("--box-x", type=float, default=1.8)
parser.add_argument("--box-y", type=float, default=-0.70)
parser.add_argument("--box-yaw", type=float, default=0.0)
args = parser.parse_args()

from isaacsim import SimulationApp

sim_app = SimulationApp({
    "headless": True, "width": args.width, "height": args.height,
    "renderer": "RaytracedLighting",
})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, Gf

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")

UsdGeom.Xform.Define(stage, "/World/scene").GetPrim().GetReferences().AddReference(args.scene)

robot = UsdGeom.Xform.Define(stage, "/World/robot")
robot.GetPrim().GetReferences().AddReference(args.robot)
rx = UsdGeom.Xformable(robot.GetPrim()); rx.ClearXformOpOrder()
rx.AddTranslateOp().Set(Gf.Vec3d(-0.55, -0.75, 0.0))
rx.AddRotateZOp().Set(90.0)

box = UsdGeom.Xform.Define(stage, "/World/smartbox")
box.GetPrim().GetReferences().AddReference(args.box)
bx = UsdGeom.Xformable(box.GetPrim()); bx.ClearXformOpOrder()
t_op = bx.AddTranslateOp()
bx.AddRotateZOp().Set(args.box_yaw)

k = 0
for p in stage.Traverse():
    if p.IsInstanceable():
        p.SetInstanceable(False); k += 1
print(f"[stage] de-instanced {k} prims", flush=True)

# 先量缩放后的箱子, 再把底面顶到 z=0
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
t_op.Set(Gf.Vec3d(args.box_x, args.box_y, 0.0))
r = bc.ComputeWorldBound(box.GetPrim()).ComputeAlignedRange()
dz = -float(r.GetMin()[2])
t_op.Set(Gf.Vec3d(args.box_x, args.box_y, dz))
bc.Clear()

for name, path in [("scene", "/World/scene"), ("robot", "/World/robot"), ("smartbox", "/World/smartbox")]:
    r = bc.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    print(f"[bbox] {name:9} min={np.round(np.array(r.GetMin()),3)} max={np.round(np.array(r.GetMax()),3)}", flush=True)

world = bc.ComputeWorldBound(stage.GetPrimAtPath("/World")).ComputeAlignedRange()
center = np.array(world.GetMidpoint()); size = np.array(world.GetSize())
radius = float(np.linalg.norm(size)) * 1.15
target = np.array([center[0], center[1], 1.0])

dome = UsdLux.DomeLight.Define(stage, "/World/lights/dome")
dome.CreateIntensityAttr(500.0); dome.CreateColorAttr(Gf.Vec3f(0.90, 0.93, 1.0))
sun = UsdLux.DistantLight.Define(stage, "/World/lights/sun")
sun.CreateIntensityAttr(1500.0); sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 20.0))

cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * args.height / args.width)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10000.0))
cx = UsdGeom.Xformable(cam.GetPrim()); cx.ClearXformOpOrder()
cam_op = cx.AddTransformOp()


def look_at(eye, tgt, up=Gf.Vec3d(0, 0, 1)):
    m = Gf.Matrix4d(); m.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*tgt), up)
    return m.GetInverse()


os.makedirs(args.outdir, exist_ok=True)
rp = rep.create.render_product("/World/cam", (args.width, args.height))
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir=args.outdir, rgb=True)
writer.attach([rp])

# 台前那侧扫三个角度
for i in range(args.views):
    ang = math.radians(-140.0 + 100.0 * i / max(1, args.views - 1))
    elev = math.radians(15.0)
    eye = target + np.array([radius * math.cos(ang) * math.cos(elev),
                             radius * math.sin(ang) * math.cos(elev),
                             radius * math.sin(elev)])
    cam_op.Set(look_at(eye, target))
    rep.orchestrator.step(rt_subframes=args.spp, delta_time=0.0)
    print(f"[render] {i + 1}/{args.views}", flush=True)

rep.orchestrator.wait_until_complete()
print(f"[done] -> {args.outdir}", flush=True)
sim_app.close()
