#!/usr/bin/env python3
"""把带引用的 smartbox_articulated 展平成单体 USD, 便于直接发给别人。

两步:
  1. Stage.Flatten() 把 reference 全部就地合成
  2. 删掉柜体里那 4 个 active=false 的门 prim —— 停用只是不参与合成,
     几何还在文件里白占体积, 展平后它们成了本地 spec 才能真正删除
"""
import sys
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

SRC = sys.argv[1] if len(sys.argv) > 1 else "smartbox_articulated_ref.usd"
DST = sys.argv[2] if len(sys.argv) > 2 else "smartbox_articulated.usd"

stage = Usd.Stage.Open(SRC)
flat = stage.Flatten()

# 递归摘掉 active=false 的 spec
removed = []
def prune(spec):
    for name, child in list(spec.nameChildren.items()):
        if child.GetInfo("active") is False:
            removed.append(child.path)
            del spec.nameChildren[name]
        else:
            prune(child)
prune(flat.pseudoRoot)
print(f"[prune] 删除停用 prim: {len(removed)}")
for p in removed:
    print("   ", p)

flat.Export(DST)
print(f"[flatten] {SRC} -> {DST}")
