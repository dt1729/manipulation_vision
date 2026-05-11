"""
orange_individual_touch.py

Orange_9 object squeezed between Piper finger pads.
Individual 16×16 <touch> sensors per finger (512 total).

Usage:
  python scripts/orange_individual_touch.py           # benchmark only
  python scripts/orange_individual_touch.py --render  # render + benchmark
Output (with --render): scripts/orange_16x16_touch.mp4
"""

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import time
import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import cv2
import imageio

parser = argparse.ArgumentParser()
parser.add_argument("--render", action="store_true", help="render video (slower)")
args = parser.parse_args()

# ── paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.dirname(SCRIPT_DIR)
ROBOT_XML  = os.path.join(REPO_ROOT,
    "includes/robosuite/robosuite/models/assets/robots/piper_arm/robot.xml")
ORANGE_DIR = os.path.abspath(os.path.join(REPO_ROOT,
    "includes/robocasa/robocasa/models/assets/objects/objaverse/orange/orange_9"))
ORANGE_XML = os.path.join(ORANGE_DIR, "model.xml")
OUTPUT_MP4 = os.path.join(SCRIPT_DIR, "orange_16x16_touch.mp4")
TMP_XML    = os.path.join(os.path.dirname(ROBOT_XML), "_orange_ind.xml")

# ── arm + gripper pose ────────────────────────────────────────────────────────
ARM_POSE = {
    "joint1": 0.0, "joint2": 1.2, "joint3": -1.5,
    "joint4": 0.0, "joint5": 0.4, "joint6": 0.0,
    "joint7":  0.025,   # 25 mm each side → ~50 mm gap (orange ~25 mm radius)
    "joint8": -0.025,
}

# ── orange motion ─────────────────────────────────────────────────────────────
ORANGE_SCALE = 0.040    # ~25 mm radius
SHM_FREQ  = 0.25        # Hz — slow press oscillation
SHM_AMP   = 0.022       # 22 mm → solid press into both pads
SHM_KP    = 80.0
SHM_KD    = 12.0
SWEEP_Z_FREQ = 0.11;  SWEEP_Z_AMP = 0.006
SWEEP_X_FREQ = 0.13;  SWEEP_X_AMP = 0.005

# ── individual sensor grid ────────────────────────────────────────────────────
NX, NY  = 16, 16
SITE_R  = 0.004   # 4 mm — covers ~2 cells with overlap; catches contacts on 1.9×2.8 mm cell pitch
PAD_CY  = -0.0425
PAD_HX  = 0.015
PAD_HY  = 0.0225

x_pos = np.linspace(-PAD_HX + PAD_HX/NX, PAD_HX - PAD_HX/NX, NX)
y_pos = np.linspace(PAD_CY - PAD_HY + PAD_HY/NY,
                    PAD_CY + PAD_HY - PAD_HY/NY, NY)

# ── display constants ─────────────────────────────────────────────────────────
CELL_PX = 22
BORDER  = 1
CMAP    = cv2.COLORMAP_INFERNO

# ── helpers ───────────────────────────────────────────────────────────────────
def jnt_adr(m, n): return m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
def act_id(m, n):  return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
def body_id(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
def site_id(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)
def geom_id(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
def even(x):       return x if x % 2 == 0 else x + 1

def set_arm(m, d):
    for jn, v in ARM_POSE.items():
        d.qpos[jnt_adr(m, jn)] = v
    for i in range(1, 9):
        d.ctrl[act_id(m, f"joint{i}")] = ARM_POSE[f"joint{i}"]

def sensor_panel(readings, label, peak, nx, ny):
    grid = np.abs(readings.reshape(nx, ny)).astype(np.float32)
    vmax = max(peak, 1e-4)
    W = nx * CELL_PX + (nx + 1) * BORDER
    H = ny * CELL_PX + (ny + 1) * BORDER
    img = np.zeros((H, W, 3), np.uint8)
    for ix in range(nx):
        for iy in range(ny):
            val = grid[ix, iy]
            intensity = int(np.clip(val / vmax * 255, 0, 255))
            color = cv2.applyColorMap(
                np.array([[intensity]], dtype=np.uint8), CMAP)[0, 0].tolist()
            x0 = BORDER + ix * (CELL_PX + BORDER)
            y0 = BORDER + iy * (CELL_PX + BORDER)
            cv2.rectangle(img, (x0, y0), (x0+CELL_PX-1, y0+CELL_PX-1), color, -1)
    n_act = int(np.count_nonzero(grid > 0.01 * vmax))
    bar_h = H
    bar = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(np.repeat(bar, 24, 1), CMAP)
    cv2.putText(bar, f"{vmax:.1f}", (1, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255,255,255), 1)
    cv2.putText(bar, "0",           (3, bar_h-5), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (180,180,180), 1)
    row = np.hstack([img, bar])
    hdr = np.zeros((28, row.shape[1], 3), np.uint8)
    cv2.putText(hdr, f"{label}  {n_act}/{nx*ny} active",
                (4, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (210,235,255), 1)
    return np.vstack([hdr, row])

# ── pass 1: finger geometry ───────────────────────────────────────────────────
tree = ET.parse(ROBOT_XML); root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "640")
tree.write(TMP_XML)

m0 = mujoco.MjModel.from_xml_path(TMP_XML); os.remove(TMP_XML)
d0 = mujoco.MjData(m0); set_arm(m0, d0); mujoco.mj_forward(m0, d0)

l7g = d0.geom_xpos[geom_id(m0, "link7_col")].copy()
l8g = d0.geom_xpos[geom_id(m0, "link8_col")].copy()
sl_z = d0.site_xmat[site_id(m0, "tac_pad_left")].reshape(3, 3)[:, 2].copy()
contact_axis = sl_z / np.linalg.norm(sl_z)
center = (l7g + l8g) / 2.0
gap = float(np.dot(l8g - l7g, contact_axis))
print(f"contact_axis : {contact_axis}  gap = {gap*1000:.1f} mm")
print(f"center       : {center}")

# ── build scene ───────────────────────────────────────────────────────────────
tree = ET.parse(ROBOT_XML); root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "640")

size_el = root.find("size")
if size_el is None: size_el = ET.SubElement(root, "size")
size_el.set("nconmax", "500")
size_el.set("njmax",   "2000")

wb  = root.find("worldbody")
snr = root.find("sensor")

# ── inject individual sensor sites into link7 and link8 ──────────────────────
for body_el in root.iter("body"):
    bname = body_el.get("name", "")
    if bname not in ("link7", "link8"):
        continue
    side = "l" if bname == "link7" else "r"
    for ix, x in enumerate(x_pos):
        for iy, y in enumerate(y_pos):
            sname = f"ts_{side}_{ix}_{iy}"
            ET.SubElement(body_el, "site", attrib={
                "name":  sname,
                "type":  "sphere",
                "size":  f"{SITE_R}",
                "pos":   f"{x:.5f} {y:.5f} 0",
                "group": "4",
                "rgba":  "0.2 1 0.2 0.35",
            })
            ET.SubElement(snr, "touch", attrib={
                "name": f"t{side}_{ix}_{iy}",
                "site": sname,
            })

print(f"Injected {NX*NY} sensor sites per finger ({NX*NY*2} total)")

# ── cameras + floor ───────────────────────────────────────────────────────────
ET.SubElement(wb, "geom", attrib={
    "name": "floor", "type": "plane", "size": "2 2 0.1", "pos": "0 0 0",
    "rgba": "0.22 0.22 0.25 1", "contype": "0", "conaffinity": "0",
})

fx, fy, fz = center
ET.SubElement(wb, "camera", attrib={
    "name": "wide_cam", "mode": "fixed",
    "pos":  "0.90 0.0 0.45",
    "xyaxes": "0 1 0  -0.4 0 0.9",
})
ET.SubElement(wb, "camera", attrib={
    "name": "close_cam", "mode": "fixed",
    "pos":  f"{fx+0.22:.4f} {fy:.4f} {fz+0.10:.4f}",
    "xyaxes": "0 1 0  -0.42 0 0.91",
})

# ── embed orange_9 collision meshes ──────────────────────────────────────────
otree = ET.parse(ORANGE_XML)
oroot = otree.getroot()

scene_asset = root.find("asset")
if scene_asset is None:
    scene_asset = ET.SubElement(root, "asset")

orig_scale = None
for mesh_el in oroot.find("asset").findall("mesh"):
    name = mesh_el.get("name", "")
    if "collision" not in name:
        continue
    if orig_scale is None:
        orig_scale = float(mesh_el.get("scale", "0.075 0.075 0.075").split()[0])
    s = ORANGE_SCALE
    new_mesh = ET.SubElement(scene_asset, "mesh")
    new_mesh.set("name", name)
    new_mesh.set("file", os.path.join(ORANGE_DIR, mesh_el.get("file")))
    new_mesh.set("scale", f"{s:.6f} {s:.6f} {s:.6f}")
    if "refquat" in mesh_el.attrib:
        new_mesh.set("refquat", mesh_el.get("refquat"))

cx, cy, cz = center
orange_body = ET.SubElement(wb, "body", attrib={
    "name": "orange_obj",
    "pos":  f"{cx:.5f} {cy:.5f} {cz:.5f}",
})
orange_body.append(ET.fromstring(
    '<inertial pos="0 0 0" mass="0.15" diaginertia="0.0002 0.0002 0.0002"/>'))
orange_body.append(ET.fromstring("<freejoint/>"))

for body_el in oroot.iter("body"):
    if body_el.get("name") == "object":
        for geom_el in body_el.findall("geom"):
            mesh_name = geom_el.get("mesh", "")
            if "collision" in mesh_name:
                ET.SubElement(orange_body, "geom", attrib={
                    "type":        "mesh",
                    "mesh":        mesh_name,
                    "contype":     "2",
                    "conaffinity": "2",
                    "mass":        "0",
                    "rgba":        "1 0.55 0 0.9",
                })

tree.write(TMP_XML)
m = mujoco.MjModel.from_xml_path(TMP_XML); os.remove(TMP_XML)
d = mujoco.MjData(m)
set_arm(m, d)
m.opt.gravity[:] = [0, 0, 0]
m.opt.timestep   = 0.0005
mujoco.mj_forward(m, d)

orange_bid = body_id(m, "orange_obj")
wide_cam   = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "wide_cam")
close_cam  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "close_cam")
print(f"orange body id={orange_bid}  geoms={m.body_geomnum[orange_bid]}")

# collect sensor addresses (NX×NY) per side
def get_sensor_addrs(side):
    addrs = np.zeros((NX, NY), dtype=int)
    for ix in range(NX):
        for iy in range(NY):
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, f"t{side}_{ix}_{iy}")
            addrs[ix, iy] = m.sensor_adr[sid]
    return addrs

l_addrs = get_sensor_addrs("l")
r_addrs = get_sensor_addrs("r")

def read_sensors(addrs):
    return np.array([d.sensordata[a] for a in addrs.flat]).reshape(NX, NY)

# ── simulate + capture ────────────────────────────────────────────────────────
SIM_DT   = m.opt.timestep
FPS      = 30
DURATION = 15.0
STEPS    = int(DURATION / SIM_DT)
EVERY    = max(1, int(1.0 / (FPS * SIM_DT)))

peak_l = peak_r = 1e-4
omega = 2 * np.pi * SHM_FREQ

render_label = "with rendering" if args.render else "no rendering"
print(f"\nSimulating {DURATION}s ({STEPS} steps) — {render_label} …")
if args.render:
    print(f"Output: {OUTPUT_MP4}")

# ── timing accumulators ───────────────────────────────────────────────────────
t_sim    = 0.0
t_sensor = 0.0
t_render = 0.0
t_draw   = 0.0
n_reads  = 0

if args.render:
    r_wide  = mujoco.Renderer(m, 360, 480)
    r_close = mujoco.Renderer(m, 360, 480)
    writer  = imageio.get_writer(OUTPUT_MP4, fps=FPS, quality=8, macro_block_size=None)

wall_start = time.perf_counter()

for step in range(STEPS):
    t = step * SIM_DT

    sweep_z = SWEEP_Z_AMP * np.sin(2 * np.pi * SWEEP_Z_FREQ * t)
    sweep_x = SWEEP_X_AMP * np.sin(2 * np.pi * SWEEP_X_FREQ * t)
    target_pos = (center
                  + contact_axis      * (SHM_AMP * np.sin(omega * t))
                  + np.array([1,0,0]) * sweep_x
                  + np.array([0,0,1]) * sweep_z)

    orange_pos = d.xpos[orange_bid].copy()
    orange_vel = d.cvel[orange_bid, 3:6].copy()
    d.xfrc_applied[orange_bid, :3] = SHM_KP * (target_pos - orange_pos) + SHM_KD * (-orange_vel)

    for i in range(1, 9):
        d.ctrl[act_id(m, f"joint{i}")] = ARM_POSE[f"joint{i}"]

    _t0 = time.perf_counter()
    mujoco.mj_step(m, d)
    t_sim += time.perf_counter() - _t0

    if step % EVERY != 0:
        continue

    _t0 = time.perf_counter()
    sl = read_sensors(l_addrs)
    sr = read_sensors(r_addrs)
    t_sensor += time.perf_counter() - _t0
    peak_l = max(peak_l, float(np.abs(sl).max()))
    peak_r = max(peak_r, float(np.abs(sr).max()))
    n_reads += 1

    if not args.render:
        continue

    _t0 = time.perf_counter()
    r_wide.update_scene(d, camera=wide_cam)
    wide_bgr  = r_wide.render()[:, :, ::-1].copy()
    r_close.update_scene(d, camera=close_cam)
    close_bgr = r_close.render()[:, :, ::-1].copy()
    t_render += time.perf_counter() - _t0

    _t0 = time.perf_counter()
    orange_y_off = float(np.dot(d.xpos[orange_bid] - center, contact_axis))
    cv2.putText(close_bgr,
        f"orange: {orange_y_off*1000:+.1f} mm  ncon={d.ncon}",
        (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 100), 1)

    three_d = np.vstack([wide_bgr, close_bgr])

    sp_l = sensor_panel(sl, "left  (link7)", peak_l, NX, NY)
    sp_r = sensor_panel(sr, "right (link8)", peak_r, NX, NY)

    pw = max(sp_l.shape[1], sp_r.shape[1])
    def hpad(img, w):
        if img.shape[1] < w:
            img = np.hstack([img, np.zeros((img.shape[0], w-img.shape[1], 3), np.uint8)])
        return img
    sp_col = np.vstack([hpad(sp_l, pw), hpad(sp_r, pw)])

    fh = max(three_d.shape[0], sp_col.shape[0])
    def vpad(img, h):
        if img.shape[0] < h:
            img = np.vstack([img, np.zeros((h-img.shape[0], img.shape[1], 3), np.uint8)])
        return img

    div   = np.full((fh, 4, 3), 60, np.uint8)
    frame = np.hstack([vpad(three_d, fh), div, vpad(sp_col, fh)])

    l_act = int(np.count_nonzero(np.abs(sl) > 0.01 * peak_l))
    r_act = int(np.count_nonzero(np.abs(sr) > 0.01 * peak_r))
    cv2.putText(frame,
        f"t={t:.2f}s  ncon={d.ncon}  L={l_act}/{NX*NY}  R={r_act}/{NX*NY}",
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 100), 2)

    fh2 = even(frame.shape[0]); fw2 = even(frame.shape[1])
    frame = cv2.copyMakeBorder(frame, 0, fh2-frame.shape[0],
                               0, fw2-frame.shape[1], cv2.BORDER_CONSTANT)
    writer.append_data(frame[:, :, ::-1].copy())
    t_draw += time.perf_counter() - _t0

wall_total = time.perf_counter() - wall_start

if args.render:
    r_wide.close(); r_close.close()
    writer.close()
    print(f"Done: {OUTPUT_MP4}  ({os.path.getsize(OUTPUT_MP4)//1024} KB)")

t_other = wall_total - t_sim - t_sensor - t_render - t_draw
print()
print(f"── timing breakdown ({render_label}) {'─'*(35 - len(render_label))}")
print(f"  wall total   : {wall_total:7.3f} s")
print(f"  sim (mj_step): {t_sim:7.3f} s  ({t_sim/wall_total*100:.1f}%)  "
      f"  {t_sim/STEPS*1e6:.1f} µs/step   "
      f"  {DURATION/wall_total:.1f}× realtime")
print(f"  sensor read  : {t_sensor:7.3f} s  ({t_sensor/wall_total*100:.1f}%)  "
      f"  {t_sensor/n_reads*1e3:.3f} ms/read  ({NX*NY*2} sensors)")
if args.render:
    print(f"  render       : {t_render:7.3f} s  ({t_render/wall_total*100:.1f}%)  "
          f"  {t_render/n_reads*1e3:.2f} ms/frame  (2 cameras)")
    print(f"  draw+encode  : {t_draw:7.3f} s  ({t_draw/wall_total*100:.1f}%)  "
          f"  {t_draw/n_reads*1e3:.2f} ms/frame")
print(f"  other        : {t_other:7.3f} s  ({t_other/wall_total*100:.1f}%)")
print(f"  peak active  : L={int(np.count_nonzero(sl > 0.01*peak_l))}/{NX*NY}  "
      f"R={int(np.count_nonzero(sr > 0.01*peak_r))}/{NX*NY}  "
      f"  maxL={peak_l:.2f}N  maxR={peak_r:.2f}N")
print("─────────────────────────────────────────────────────────────")
