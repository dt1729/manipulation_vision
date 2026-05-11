"""
tactile_video.py

Full Piper arm, gripper at 25 mm opening.
The orange_9 object (16 convex-decomposition geoms) is oscillated between the
finger pads. Multiple geoms contact each pad simultaneously → multiple sensor
cells fire per frame.

Left panel  — 3D scene (wide arm view + gripper close-up)
Right panel — 16×16 touch_grid heatmaps for each finger

Usage:  python scripts/tactile_video.py
Output: scripts/tactile_video.mp4
"""

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import mujoco
import xml.etree.ElementTree as ET
import cv2
import imageio

# ── paths ────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.dirname(SCRIPT_DIR)
ROBOT_XML   = os.path.join(REPO_ROOT,
    "includes/robosuite/robosuite/models/assets/robots/piper_arm/robot.xml")
ORANGE_DIR  = os.path.abspath(os.path.join(REPO_ROOT,
    "includes/robocasa/robocasa/models/assets/objects/objaverse/orange/orange_9"))
ORANGE_XML  = os.path.join(ORANGE_DIR, "model.xml")
OUTPUT_MP4  = os.path.join(SCRIPT_DIR, "tactile_video.mp4")
TMP_XML     = os.path.join(os.path.dirname(ROBOT_XML), "_tac_scene.xml")

# ── arm + gripper pose ────────────────────────────────────────────────────────
ARM_POSE = {
    "joint1": 0.0,
    "joint2": 1.2,
    "joint3": -1.5,
    "joint4": 0.0,
    "joint5": 0.4,
    "joint6": 0.0,
    "joint7":  0.025,   # 25 mm opening each side → 70 mm gap
    "joint8": -0.025,
}

SHM_FREQ   = 0.25       # Hz
SHM_AMP    = 0.020      # 20 mm → well past the ~10 mm clearance → solid press

SWEEP_Z_FREQ = 0.11
SWEEP_Z_AMP  = 0.006    # ±6 mm vertical
SWEEP_X_FREQ = 0.13
SWEEP_X_AMP  = 0.005    # ±5 mm lateral

SHM_KP = 80.0
SHM_KD = 12.0

# orange_9 mesh scale: 0.050 → half-extents ≈ 22 mm (fits in 35 mm half-gap)
ORANGE_SCALE = 0.040    # ~25 mm radius → 10 mm clearance inside 35 mm half-gap

# tactile display constants
CMAP  = cv2.COLORMAP_INFERNO
TPAD  = 16
CELL  = 22
TGRID = TPAD * CELL     # 352 px

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

def tactile_panel(flat, label, peak):
    grid = np.abs(flat.reshape(TPAD, TPAD)).astype(np.float32)
    vmax = max(peak, 0.0001)
    norm = np.clip(grid / vmax * 255, 0, 255).astype(np.uint8)
    big  = cv2.resize(norm, (TGRID, TGRID), interpolation=cv2.INTER_NEAREST)
    col  = cv2.applyColorMap(big, CMAP)
    for k in range(TPAD + 1):
        cv2.line(col, (k*CELL, 0),   (k*CELL, TGRID),  (45,45,45), 1)
        cv2.line(col, (0, k*CELL),   (TGRID, k*CELL),  (45,45,45), 1)
    n_act = int(np.count_nonzero(grid))
    cv2.putText(col, f"{n_act}/256", (4, TGRID-7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200,245,200), 1)
    bar_h = TGRID
    bar = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(-1,1)
    bar = cv2.applyColorMap(np.repeat(bar, 28, 1), CMAP)
    cv2.putText(bar, f"{vmax:.0f}N", (2,14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255,255,255), 1)
    cv2.putText(bar, "0", (8,bar_h-5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180,180,180), 1)
    row = np.hstack([col, bar])
    hdr = np.zeros((26, row.shape[1], 3), np.uint8)
    cv2.putText(hdr, label, (4,18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (210,235,255), 1)
    return np.vstack([hdr, row])

# ── pass 1: get finger geometry ───────────────────────────────────────────────
tree = ET.parse(ROBOT_XML)
root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "640")
tree.write(TMP_XML)

m0 = mujoco.MjModel.from_xml_path(TMP_XML); os.remove(TMP_XML)
d0 = mujoco.MjData(m0)
set_arm(m0, d0)
mujoco.mj_forward(m0, d0)

l7g = d0.geom_xpos[geom_id(m0, "link7_col")].copy()
l8g = d0.geom_xpos[geom_id(m0, "link8_col")].copy()
sl_z = d0.site_xmat[site_id(m0, "tac_pad_left")].reshape(3, 3)[:, 2].copy()
contact_axis = sl_z / np.linalg.norm(sl_z)   # ≈ world +Y
center = (l7g + l8g) / 2.0
gap = float(np.dot(l8g - l7g, contact_axis))

print(f"contact_axis : {contact_axis}  gap = {gap*1000:.1f} mm")
print(f"center       : {center}")

# ── build full scene ──────────────────────────────────────────────────────────
tree = ET.parse(ROBOT_XML)
root = tree.getroot()
v = root.find("visual")
if v is None: v = ET.SubElement(root, "visual")
gl = v.find("global")
if gl is None: gl = ET.SubElement(v, "global")
gl.set("offwidth", "960"); gl.set("offheight", "640")

size_el = root.find("size")
if size_el is None: size_el = ET.SubElement(root, "size")
size_el.set("nconmax", "500")
size_el.set("njmax",   "2000")

wb = root.find("worldbody")

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
# Load orange model, convert relative file paths to absolute, adjust scale.
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

# Add orange body with freejoint at center
cx, cy, cz = center
orange_body = ET.SubElement(wb, "body", attrib={
    "name": "orange_obj",
    "pos":  f"{cx:.5f} {cy:.5f} {cz:.5f}",
})
orange_body.append(ET.fromstring("<inertial pos=\"0 0 0\" mass=\"0.15\" diaginertia=\"0.0002 0.0002 0.0002\"/>"))
orange_body.append(ET.fromstring("<freejoint/>"))

for body_el in oroot.iter("body"):
    if body_el.get("name") == "object":
        for geom_el in body_el.findall("geom"):
            mesh_name = geom_el.get("mesh", "")
            if "collision" in mesh_name:
                ET.SubElement(orange_body, "geom", attrib={
                    "type":        "mesh",
                    "mesh":        mesh_name,
                    "contype":     "2",    # bit-1: only collides with link7/link8 (conaffinity=3)
                    "conaffinity": "2",    # avoids spurious contact with link6/arm body
                    "mass":        "0",
                    "rgba":        "1 0.55 0 0.9",
                })

tree.write(TMP_XML)
m = mujoco.MjModel.from_xml_path(TMP_XML)
d = mujoco.MjData(m)
set_arm(m, d)
m.opt.gravity[:] = [0, 0, 0]
m.opt.timestep   = 0.0005
mujoco.mj_forward(m, d)

orange_bid = body_id(m, "orange_obj")
s_l_adr   = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_left")]
s_r_adr   = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "touch_right")]
wide_cam  = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "wide_cam")
close_cam = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "close_cam")

print(f"orange body id: {orange_bid}  ngeom on orange: {m.body_geomnum[orange_bid]}")

# ── simulate + capture ────────────────────────────────────────────────────────
SIM_DT   = m.opt.timestep
FPS      = 30
DURATION = 15.0
STEPS    = int(DURATION / SIM_DT)
EVERY    = max(1, int(1.0 / (FPS * SIM_DT)))

W_WIDE, H_WIDE   = 480, 360
W_CLOSE, H_CLOSE = 480, 360
r_wide  = mujoco.Renderer(m, H_WIDE,  W_WIDE)
r_close = mujoco.Renderer(m, H_CLOSE, W_CLOSE)

peak_l = peak_r = 0.0001
omega = 2 * np.pi * SHM_FREQ

print(f"\nSimulating {DURATION}s ({STEPS} steps) → {STEPS//EVERY} frames …")
print(f"Streaming → {OUTPUT_MP4}")
writer = imageio.get_writer(OUTPUT_MP4, fps=FPS, quality=8, macro_block_size=None)

for step in range(STEPS):
    t = step * SIM_DT

    sweep_z = SWEEP_Z_AMP * np.sin(2 * np.pi * SWEEP_Z_FREQ * t)
    sweep_x = SWEEP_X_AMP * np.sin(2 * np.pi * SWEEP_X_FREQ * t)
    target_pos = (center
                  + contact_axis       * (SHM_AMP * np.sin(omega * t))
                  + np.array([1,0,0])  * sweep_x
                  + np.array([0,0,1])  * sweep_z)

    orange_pos = d.xpos[orange_bid].copy()
    orange_vel = d.cvel[orange_bid, 3:6].copy()
    force = SHM_KP * (target_pos - orange_pos) + SHM_KD * (-orange_vel)
    d.xfrc_applied[orange_bid, :3] = force

    for i in range(1, 9):
        d.ctrl[act_id(m, f"joint{i}")] = ARM_POSE[f"joint{i}"]

    mujoco.mj_step(m, d)

    if step % EVERY != 0:
        continue

    sd_l = d.sensordata[s_l_adr : s_l_adr + 256].copy()
    sd_r = d.sensordata[s_r_adr : s_r_adr + 256].copy()
    peak_l = max(peak_l, float(np.abs(sd_l).max()))
    peak_r = max(peak_r, float(np.abs(sd_r).max()))

    r_wide.update_scene(d, camera=wide_cam)
    wide_bgr = r_wide.render()[:, :, ::-1].copy()
    r_close.update_scene(d, camera=close_cam)
    close_bgr = r_close.render()[:, :, ::-1].copy()

    orange_y_off = float(np.dot(d.xpos[orange_bid] - center, contact_axis))
    cv2.putText(close_bgr,
        f"orange: {orange_y_off*1000:+.1f} mm  ncon={d.ncon}",
        (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255,255,100), 1)

    three_d = np.vstack([wide_bgr, close_bgr])

    tp_l = tactile_panel(sd_l, "touch_left  (link7)", peak_l)
    tp_r = tactile_panel(sd_r, "touch_right (link8)", peak_r)
    tac_w = max(tp_l.shape[1], tp_r.shape[1])
    tac_h = tp_l.shape[0] + tp_r.shape[0]
    tac_col = np.zeros((tac_h, tac_w, 3), np.uint8)
    tac_col[:tp_l.shape[0], :tp_l.shape[1]] = tp_l
    tac_col[tp_l.shape[0]:, :tp_r.shape[1]] = tp_r

    fh = max(three_d.shape[0], tac_col.shape[0])
    def vpad(img, h):
        if img.shape[0] < h:
            img = np.vstack([img, np.zeros((h-img.shape[0], img.shape[1], 3), np.uint8)])
        return img
    three_d = vpad(three_d, fh)
    tac_col = vpad(tac_col, fh)

    div   = np.full((fh, 4, 3), 60, np.uint8)
    frame = np.hstack([three_d, div, tac_col])

    l_nz = int(np.count_nonzero(sd_l))
    r_nz = int(np.count_nonzero(sd_r))
    cv2.putText(frame,
        f"t={t:.2f}s   ncon={d.ncon}   L={l_nz} cells   R={r_nz} cells",
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,100), 2)

    fh2 = even(frame.shape[0]); fw2 = even(frame.shape[1])
    frame = cv2.copyMakeBorder(frame, 0, fh2-frame.shape[0],
                               0, fw2-frame.shape[1], cv2.BORDER_CONSTANT)

    writer.append_data(frame[:, :, ::-1].copy())

r_wide.close(); r_close.close()
writer.close()
print(f"Done: {OUTPUT_MP4}  ({os.path.getsize(OUTPUT_MP4)//1024} KB)")
