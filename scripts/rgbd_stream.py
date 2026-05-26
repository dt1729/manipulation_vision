"""
rgbd_stream.py

RGB-D stream from the Piper arm's eye-in-hand camera via robocasa/robosuite.
Demonstrates the full depth pipeline: observation → metric depth → 3D point cloud.

Usage:
    python scripts/rgbd_stream.py
Outputs:
    scripts/rgbd_stream.mp4        — side-by-side RGB | depth colourmap video
    scripts/rgbd_pointcloud.npy    — (H*W, 3) world-frame point cloud at peak depth frame
"""

import os
import numpy as np
import cv2
import imageio
from tqdm import tqdm

import robocasa  # noqa: F401
from robocasa.utils.env_utils import create_env
from robosuite.utils.camera_utils import (
    get_real_depth_map,
    get_camera_intrinsic_matrix,
    get_camera_extrinsic_matrix,
)

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
OUTPUT_MP4  = os.path.join(SCRIPT_DIR, "rgbd_stream.mp4")
OUTPUT_PCD  = os.path.join(SCRIPT_DIR, "rgbd_pointcloud.npy")

CAM_NAME = "robot0_eye_in_hand"
H, W     = 256, 256
FPS      = 20
N_STEPS  = 200

# ── build env ─────────────────────────────────────────────────────────────────
env = create_env(
    env_name="PickPlaceCounterToCabinet",
    robots="PiperOmron",
    camera_names=["robot0_agentview_left", CAM_NAME],
    camera_widths=W,
    camera_heights=H,
    camera_depths=True,
)

obs = env.reset()
action = np.zeros(env.action_spec[0].shape)

# ── precompute pixel grid once ────────────────────────────────────────────────
# pixel coords shaped (H*W, 2): each row is [row_idx, col_idx]
rows, cols = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
pixel_grid = np.stack([rows.ravel(), cols.ravel()], axis=-1)  # (H*W, 2)

# ── video writer ──────────────────────────────────────────────────────────────
writer = imageio.get_writer(OUTPUT_MP4, fps=FPS, quality=8, macro_block_size=None)

best_mean_depth = 0.0
best_pcd        = None

print(f"Running {N_STEPS} steps → {OUTPUT_MP4}")

for step in tqdm(range(N_STEPS)):
    obs, _, _, _ = env.step(action)

    rgb_obs   = obs[f"{CAM_NAME}_image"]   # (H, W, 3) uint8
    depth_obs = obs[f"{CAM_NAME}_depth"]   # (H, W, 1) float32  [0, 1]

    # ── metric depth ──────────────────────────────────────────────────────────
    depth_norm  = depth_obs[..., 0]                          # (H, W)
    depth_m     = get_real_depth_map(env.sim, depth_norm)    # (H, W) metres

    mean_d = float(depth_m[depth_m > 0].mean()) if (depth_m > 0).any() else 0.0
    min_d  = float(depth_m[depth_m > 0].min())  if (depth_m > 0).any() else 0.0
    max_d  = float(depth_m.max())

    if step % 20 == 0:
        print(f"  step {step:3d}  depth min={min_d:.3f}m  mean={mean_d:.3f}m  max={max_d:.3f}m")

    # ── point cloud (save snapshot at frame with most visible depth) ──────────
    if mean_d > best_mean_depth:
        best_mean_depth = mean_d
        K = get_camera_intrinsic_matrix(env.sim, CAM_NAME, H, W)  # (3, 3)
        E = get_camera_extrinsic_matrix(env.sim, CAM_NAME)         # cam→world (4, 4)
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        rows_f = pixel_grid[:, 0].astype(float)
        cols_f = pixel_grid[:, 1].astype(float)
        z = depth_m[pixel_grid[:, 0], pixel_grid[:, 1]]            # (H*W,)
        pts_cam = np.stack([
            (cols_f - cx) * z / fx,
            (rows_f - cy) * z / fy,
            z,
            np.ones_like(z),
        ], axis=-1)                                                  # (H*W, 4)
        best_pcd = (E @ pts_cam.T).T[:, :3]                        # (H*W, 3)

    # ── depth colourmap ───────────────────────────────────────────────────────
    depth_vis = np.clip(depth_m / max(max_d, 0.01), 0, 1)
    depth_u8  = (depth_vis * 255).astype(np.uint8)
    depth_col = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)[:, :, ::-1]  # RGB

    # ── side-by-side frame ────────────────────────────────────────────────────
    left  = rgb_obs.astype(np.uint8)
    right = depth_col.astype(np.uint8)

    div   = np.full((H, 4, 3), 60, np.uint8)
    frame = np.hstack([left, div, right])

    # HUD
    frame_bgr = frame[:, :, ::-1].copy()
    cv2.putText(frame_bgr, f"RGB  eye_in_hand",
                (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1)
    cv2.putText(frame_bgr, f"Depth  min={min_d:.2f}m  max={max_d:.2f}m",
                (W + 8, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 255), 1)

    writer.append_data(frame_bgr[:, :, ::-1])

writer.close()
print(f"\nSaved video  → {OUTPUT_MP4}  ({os.path.getsize(OUTPUT_MP4)//1024} KB)")

# ── save point cloud ──────────────────────────────────────────────────────────
if best_pcd is not None:
    # filter out far/invalid points
    valid = (best_pcd[:, 2] > 0.05) & (best_pcd[:, 2] < 10.0)
    best_pcd = best_pcd[valid]
    np.save(OUTPUT_PCD, best_pcd)
    print(f"Saved cloud  → {OUTPUT_PCD}  shape={best_pcd.shape}")
    print(f"  XYZ min: {best_pcd.min(0)}")
    print(f"  XYZ max: {best_pcd.max(0)}")
