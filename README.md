# dem_sim

Isaac Sim 里的 smartbox 仿真：机器人 + 重建操作台 + 带关节的 smartbox。

## 目录

```
assets/
  smartbox_articulated.usd     # smartbox 成品 (单文件, 4 个 revolute 门铰链), 详见 smartbox_articulated.md
  notray_tray.usd              # 扫描重建的操作台 (薄壳视觉网格 + 透明承载面)
  alphabot2/                   # 机器人 alphabot2 (入口 alphabot2_isaac6_clean.usda, payloads/ 为相对引用, 需整体拷贝)
  src/
    smartbox_m.usd             # CAD 转换得到的米制/Y-up 源件, build 脚本输入
    smartbox_articulated_ref.usd  # build 输出 (引用 smartbox_m.usd), flatten 脚本输入
build_smartbox_articulated.py  # smartbox_m.usd -> src/smartbox_articulated_ref.usd (材质 + 关节)
flatten_smartbox.py            # ref 版展平 -> assets/smartbox_articulated.usd
scene_smartbox.py              # 机器人 + 操作台 + smartbox 组合场景, 渲确认图
drop_smartbox.py               # 门打开, 往 smartbox 上扔物体, 环绕渲染
shoot_smartbox.py              # 四面向 smartbox 发射 cube, 环绕渲染
out/                           # 脚本输出 (已 gitignore)
archive/                       # 历史脚本和中间产物 (已 gitignore, 不在仓库里)
```

## 资产

| 资产 | 单位 / 朝向 | defaultPrim | 说明 |
|---|---|---|---|
| `smartbox_articulated.usd` | 米, Z-up, 底面在 z=0 | `/SmartBox` | 4 DOF 对开门, 带 DriveAPI; 细节见 `assets/smartbox_articulated.md` |
| `notray_tray.usd` | 米, Z-up | `/notray_tray` | 由扫描点云 (`notray.ply`) 重建; 资产本身碰撞已关闭, 用一块透明实体平面承载物体 |
| `alphabot2/alphabot2_isaac6_clean.usda` | 米, Z-up | `/alphabot2` | URDF 转换, Physics variant 默认 `physx` (另有 `mujoco`、`physics`) |

## 用法

脚本需要在 Isaac Sim 的 Python 环境里跑 (`./python.sh <script>`)，资产路径均相对脚本所在目录，默认输出到 `out/`。

```bash
./python.sh scene_smartbox.py     # 组合场景确认图 -> out/scene_views
./python.sh drop_smartbox.py      # 扔物体 -> out/drop_frames
./python.sh shoot_smartbox.py     # 发射 cube -> out/shoot_frames
```

重新生成 smartbox 资产 (只需 `pxr`)：

```bash
python3 build_smartbox_articulated.py   # -> assets/src/smartbox_articulated_ref.usd
python3 flatten_smartbox.py             # -> assets/smartbox_articulated.usd
```
