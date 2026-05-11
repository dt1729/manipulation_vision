"""
individual_touch.py

4×6 = 24 individual MuJoCo <touch> sensors per finger pad.
Inspired by HandManipulateEgg-v1: each site is a small sphere that
independently measures contact force — no contact-point sparsity.

A 3×3 sphere probe is pressed against each pad in turn while a slow
sweep walks the contact patch across different cells.

Usage:  python scripts/individual_touch.py
Output: scripts/individual_touch.mp4
"""

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import cv2
import imageio

# ── paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT  = os.path.dirname(SCRIPT_DIR)
ROBOT_XML  = os.path.join(REPO_ROOT,
    "includes/robosuite/robosuite/models/assets/robots/piper_arm/robot.xml")
OUTPUT_MP4 = os.path.join(SCRIPT_DIR, "individual_touch.mp4")
TMP_XML    = os.path.join(os.path.dirname(ROBOT_XML), "_ind_touch.xml")

# ── arm pose ──────────────────────────────────────────────────────────────────
ARM_POSE = {
    "joint1": 0.0, "joint2": 1.2, "joint3": -1.5,
    "joint4": 0.0, "joint5": 0.4, "joint6": 0.0,
    "joint7":  0.015,   # 15 mm opening — tighter grip for cleaner contact
    "joint8": -0.015,
}

# ── individual sensor grid ────────────────────────────────────────────────────
# 4 columns (across finger width ≈ link7 local X, ±15 mm)
# 6 rows    (along finger length ≈ link7 local Y, centered at −42.5 mm)
# Sphere sites: radius 8 mm — covers 10×9 mm cells with overlap
NX, NY    = 4, 6
SITE_R    = 0.008
PAD_CY    = -0.0425          # pad centre in local Y
PAD_HX    = 0.015            # half-width  in local X (30 mm)
PAD_HY    = 0.0225           # half-length in local Y (45 mm)

x_pos = np.linspace(-PAD_HX + PAD_HX/NX, PAD_HX - PAD_HX/NX, NX)
y_pos = np.linspace(PAD_CY - PAD_HY + PAD_HY/NY,
                    PAD_CY + PAD_HY - PAD_HY/NY, NY)

# ── probe geometry ────────────────────────────────────────────────────────────
SPHERE_R   = 0.006
SPHERE_GAP = 0.012   # 3×3 grid at 12 mm spacing = 24 mm span
offsets_xz = [-SPHERE_GAP, 0.0, SPHERE_GAP]

KP, KD = 100.0, 15.0
PRESS_AMP  = 0.018   # 18 mm press depth from probe_start
PRESS_FREQ = 0.3     # Hz
SWEEP_Z_FREQ = 0.11; SWEEP_Z_AMP = 0.008
SWEEP_X_FREQ = 0.13; SWEEP_X_AMP = 0.006

# ── display constants ─────────────────────────────────────────────────────────
CELL_PX  = 48     # pixels per sensor cell
BORDER   = 3      # border between cells
CMAP     = cv2.COLORMAP_INFERNO

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
    """Render NX×NY individual sensor grid as a spatial heatmap."""
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
            if val > 0.01 * vmax:
                cv2.putText(img, f"{val:.1f}",
                            (x0+4, y0+CELL_PX-6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.28, (220,220,220), 1)
    n_act = int(np.count_nonzero(grid > 0.01 * vmax))
    # colorbar strip
    bar_h = H
    bar = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(-1, 1)
    bar = cv2.applyColorMap(np.repeat(bar, 20, 1), CMAP)
    cv2.putText(bar, f"{vmax:.1f}", (1,12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (255,255,255), 1)
    cv2.putText(bar, "0", (3, bar_h-5), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180,180,180), 1)
    row = np.hstack([img, bar])
    hdr = np.zeros((26, row.shape[1], 3), np.uint8)
    cv2.putText(hdr, f"{label}  {n_act}/{nx*ny} active",
                (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (210,235,255), 1)
    return np.vstack([hdr, row])

# ── pass 1: finger geometry ───────────────────────────────────────────────────
tree = ET.parse(ROBOT_XML); root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "540")
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

# ── build scene ───────────────────────────────────────────────────────────────
tree = ET.parse(ROBOT_XML); root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "540")

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
                "group": "4",        # visual-only group, not rendered by default
            })
            ET.SubElement(snr, "touch", attrib={
                "name": f"t{side}_{ix}_{iy}",
                "site": sname,
            })

# ── cameras + floor ───────────────────────────────────────────────────────────
ET.SubElement(wb, "geom", attrib={
    "type": "plane", "size": "2 2 0.1", "pos": "0 0 0",
    "rgba": "0.2 0.2 0.22 1", "contype": "0", "conaffinity": "0",
})
fx, fy, fz = center
ET.SubElement(wb, "camera", attrib={
    "name": "wide_cam", "mode": "fixed",
    "pos": "0.90 0.0 0.45", "xyaxes": "0 1 0  -0.4 0 0.9",
})
ET.SubElement(wb, "camera", attrib={
    "name": "close_cam", "mode": "fixed",
    "pos":  f"{fx+0.22:.4f} {fy:.4f} {fz+0.10:.4f}",
    "xyaxes": "0 1 0  -0.42 0 0.91",
})

# ── 3×3 sphere probe ─────────────────────────────────────────────────────────
probe_start = l7g + contact_axis * 0.008
pb = ET.SubElement(wb, "body", attrib={
    "name": "probe",
    "pos":  f"{probe_start[0]:.5f} {probe_start[1]:.5f} {probe_start[2]:.5f}",
})
pb.append(ET.fromstring("<freejoint/>"))
for dx in offsets_xz:
    for dz in offsets_xz:
        ET.SubElement(pb, "geom", attrib={
            "type": "sphere", "size": f"{SPHERE_R}",
            "pos":  f"{dx:.4f} 0 {dz:.4f}",
            "contype": "1", "conaffinity": "1",
            "rgba": "1 0.55 0 0.9",
        })

tree.write(TMP_XML)
m = mujoco.MjModel.from_xml_path(TMP_XML)
d = mujoco.MjData(m)
set_arm(m, d)
m.opt.gravity[:] = [0, 0, 0]
m.opt.timestep   = 0.0002
mujoco.mj_forward(m, d)

probe_bid = body_id(m, "probe")
wide_cam  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "wide_cam")
close_cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "close_cam")

# collect sensor addresses — shape (NX, NY) each side
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

print(f"Added {NX*NY} sensors per finger  ({NX*NY*2} total)")
print(f"Sensor grid: {NX} cols × {NY} rows  site radius {SITE_R*1000:.0f} mm")

# ── simulate + capture ────────────────────────────────────────────────────────
SIM_DT   = m.opt.timestep
FPS      = 30
DURATION = 12.0
STEPS    = int(DURATION / SIM_DT)
EVERY    = max(1, int(1.0 / (FPS * SIM_DT)))

W_WIDE, H_WIDE   = 480, 270
W_CLOSE, H_CLOSE = 480, 270
r_wide  = mujoco.Renderer(m, H_WIDE,  W_WIDE)
r_close = mujoco.Renderer(m, H_CLOSE, W_CLOSE)

peak_l = peak_r = 1e-4
omega  = 2 * np.pi * PRESS_FREQ

print(f"\nSimulating {DURATION}s → {STEPS//EVERY} frames …  → {OUTPUT_MP4}")
writer = imageio.get_writer(OUTPUT_MP4, fps=FPS, quality=8, macro_block_size=None)

for step in range(STEPS):
    t = step * SIM_DT

    sweep_z = SWEEP_Z_AMP * np.sin(2 * np.pi * SWEEP_Z_FREQ * t)
    sweep_x = SWEEP_X_AMP * np.sin(2 * np.pi * SWEEP_X_FREQ * t)
    target_pos = (probe_start
                  - contact_axis * (PRESS_AMP * 0.5 * (1 - np.cos(omega * t)))
                  + np.array([1, 0, 0]) * sweep_x
                  + np.array([0, 0, 1]) * sweep_z)

    ppos = d.xpos[probe_bid].copy()
    pvel = d.cvel[probe_bid, 3:6].copy()
    d.xfrc_applied[probe_bid, :3] = KP * (target_pos - ppos) + KD * (-pvel)

    for i in range(1, 9):
        d.ctrl[act_id(m, f"joint{i}")] = ARM_POSE[f"joint{i}"]

    mujoco.mj_step(m, d)

    if step % EVERY != 0:
        continue

    sl = read_sensors(l_addrs)
    sr = read_sensors(r_addrs)
    peak_l = max(peak_l, float(np.abs(sl).max()))
    peak_r = max(peak_r, float(np.abs(sr).max()))

    r_wide.update_scene(d, camera=wide_cam)
    wide_bgr  = r_wide.render()[:, :, ::-1].copy()
    r_close.update_scene(d, camera=close_cam)
    close_bgr = r_close.render()[:, :, ::-1].copy()

    press_mm = float(np.dot(probe_start - d.xpos[probe_bid], contact_axis)) * 1000
    cv2.putText(close_bgr, f"press: {press_mm:.1f} mm  ncon={d.ncon}",
                (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 100), 1)

    three_d = np.vstack([wide_bgr, close_bgr])

    sp_l = sensor_panel(sl, "left  (link7)", peak_l, NX, NY)
    sp_r = sensor_panel(sr, "right (link8)", peak_r, NX, NY)

    # widen panels to equal width
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
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 100), 2)

    fh2 = even(frame.shape[0]); fw2 = even(frame.shape[1])
    frame = cv2.copyMakeBorder(frame, 0, fh2-frame.shape[0],
                               0, fw2-frame.shape[1], cv2.BORDER_CONSTANT)
    writer.append_data(frame[:, :, ::-1].copy())

r_wide.close(); r_close.close()
writer.close()
print(f"Done: {OUTPUT_MP4}  ({os.path.getsize(OUTPUT_MP4)//1024} KB)")
