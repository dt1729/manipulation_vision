# Camera System — robocasa / robosuite

Developer reference for how cameras are defined, injected into the simulation, and how
to access RGB and depth observations from Python.

---

## Camera types and where they come from

There are two classes of cameras, set up through different mechanisms:

| Type | Examples | Where defined | How added to sim |
|------|----------|---------------|-----------------|
| **World cameras** | `robot0_agentview_left`, `robot0_agentview_right`, `robot0_agentview_center`, `robot0_frontview` | `robocasa/utils/camera_utils.py` — `CAM_CONFIGS` dict | Added to arena worldbody by `set_cameras()` at `env.reset()` |
| **Body-attached cameras** | `robot0_eye_in_hand` | `robocasa/utils/camera_utils.py` — `CAM_CONFIGS` dict with `parent_body` key | Injected into the robot body's XML element by `kitchen.py:edit_model_xml()` at `env.reset()` |

### Why two mechanisms?

World cameras can be added to the arena's `<worldbody>` directly. Body-attached cameras
must be children of a specific body element (e.g. `robot0_right_hand`) in the compiled
XML tree — they can only be injected after the full scene XML has been assembled, which
happens in `edit_model_xml()`.

---

## Full injection code path

```
CAM_CONFIGS (robocasa/utils/camera_utils.py)
    │
    ├─ set_cameras()                          ← called from kitchen __init__
    │     ├─ adds world cameras to arena worldbody XML
    │     └─ SKIPS cameras with parent_body   ← handled separately
    │
    └─ edit_model_xml()  (kitchen.py:1275–1309)   ← called at every env.reset()
          ├─ iterates CAM_CONFIGS entries with parent_body
          ├─ finds the parent body element in the compiled XML tree
          ├─ creates <camera> element if it doesn't exist
          └─ sets pos / quat from CAM_CONFIGS
```

Relevant files:
- `includes/robocasa/robocasa/utils/camera_utils.py` — `CAM_CONFIGS`, `set_cameras()`
- `includes/robocasa/robocasa/environments/kitchen/kitchen.py:1275` — `edit_model_xml()`
- `includes/robosuite/robosuite/environments/base.py:265` — XML processor chain

---

## Available cameras for PiperOmron

All camera names are prefixed with `robot0_` when accessed through the environment.

| Camera name | Parent body | FOV | Notes |
|-------------|-------------|-----|-------|
| `robot0_agentview_left` | `mobilebase0_support` | 60° | Left third-person view |
| `robot0_agentview_right` | `mobilebase0_support` | 60° | Right third-person view |
| `robot0_agentview_center` | worldbody | — | Overhead center view |
| `robot0_frontview` | `mobilebase0_support` | — | Forward-facing |
| `robot0_eye_in_hand` | `robot0_right_hand` | 75° | Wrist-mounted, moves with EEF |

---

## Getting RGB observations

```python
from robocasa.utils.env_utils import create_env

env = create_env(
    env_name="PickPlaceCounterToCabinet",
    robots="PiperOmron",
    camera_names=["robot0_agentview_left", "robot0_eye_in_hand"],
    camera_widths=256,
    camera_heights=256,
)
obs = env.reset()

rgb = obs["robot0_eye_in_hand_image"]   # np.uint8  (H, W, 3)
```

Observation key format: `{camera_name}_image`

---

## Getting RGB-D observations

Pass `camera_depths=True` to `create_env()`:

```python
env = create_env(
    env_name="PickPlaceCounterToCabinet",
    robots="PiperOmron",
    camera_names=["robot0_agentview_left", "robot0_eye_in_hand"],
    camera_widths=256,
    camera_heights=256,
    camera_depths=True,
)
obs = env.reset()

rgb        = obs["robot0_eye_in_hand_image"]   # np.uint8   (H, W, 3)
depth_norm = obs["robot0_eye_in_hand_depth"]   # np.float32 (H, W, 1)  values in [0, 1]
```

Observation key format: `{camera_name}_depth`

**The depth values are normalized in [0, 1] — they are NOT metric metres.**
See the section below to convert to metres.

---

## Converting normalized depth to metric depth (metres)

```python
from robosuite.utils.camera_utils import get_real_depth_map

depth_norm_2d = obs["robot0_eye_in_hand_depth"][..., 0]   # (H, W)
depth_metres  = get_real_depth_map(env.sim, depth_norm_2d) # (H, W)
```

The conversion formula used internally:

```
near = sim.model.vis.map.znear * sim.model.stat.extent
far  = sim.model.vis.map.zfar  * sim.model.stat.extent
depth_metres = near / (1 - depth_norm * (1 - near/far))
```

Source: `includes/robosuite/robosuite/utils/camera_utils.py:106`

---

## Camera intrinsics and extrinsics

```python
from robosuite.utils.camera_utils import (
    get_camera_intrinsic_matrix,
    get_camera_extrinsic_matrix,
)

H, W = 256, 256
cam  = "robot0_eye_in_hand"

K = get_camera_intrinsic_matrix(env.sim, cam, H, W)   # (3, 3)  — focal length in pixels
E = get_camera_extrinsic_matrix(env.sim, cam)          # (4, 4)  — camera→world transform
```

Intrinsic matrix K uses a pinhole model with square pixels:
```
f = (H/2) / tan(fov/2)
K = [[f, 0, W/2],
     [0, f, H/2],
     [0, 0,   1]]
```

Extrinsic matrix E is the camera→world homogeneous transform (includes a MuJoCo axis
correction so that camera +Z = viewing direction, matching OpenCV convention).

---

## Building a 3D point cloud from depth

```python
import numpy as np

H, W = 256, 256
cam  = "robot0_eye_in_hand"

# precompute pixel grid once
rows, cols = np.meshgrid(np.arange(H), np.arange(W), indexing="ij")
pixel_grid = np.stack([rows.ravel(), cols.ravel()], axis=-1)  # (H*W, 2) — [row, col]

# per-step
depth_m = get_real_depth_map(env.sim, obs[f"{cam}_depth"][..., 0])  # (H, W) metres
K = get_camera_intrinsic_matrix(env.sim, cam, H, W)                  # (3, 3)
E = get_camera_extrinsic_matrix(env.sim, cam)                        # cam→world (4, 4)

fx, fy = K[0, 0], K[1, 1]
cx, cy = K[0, 2], K[1, 2]
rows_f = pixel_grid[:, 0].astype(float)
cols_f = pixel_grid[:, 1].astype(float)
z = depth_m[pixel_grid[:, 0], pixel_grid[:, 1]]   # (H*W,)

pts_cam = np.stack([
    (cols_f - cx) * z / fx,   # X_cam
    (rows_f - cy) * z / fy,   # Y_cam
    z,                         # Z_cam
    np.ones_like(z),
], axis=-1)                                         # (H*W, 4)

points_world = (E @ pts_cam.T).T[:, :3]            # (H*W, 3) world frame metres
```

Note: `transform_from_pixels_to_world` from robosuite is designed for small keypoint batches,
not full H×W grids. Use direct unprojection (above) for point clouds.

---

## Demo script

`scripts/rgbd_stream.py` demonstrates the full pipeline end-to-end:

```bash
python scripts/rgbd_stream.py
# → scripts/rgbd_stream.mp4       (RGB | depth colourmap side by side)
# → scripts/rgbd_pointcloud.npy   (N×3 world-frame point cloud)
```

---

## Enabling depth per-camera vs all cameras

`camera_depths` accepts either a single bool (applies to all cameras) or a list of bools
matching the `camera_names` list:

```python
# depth on all cameras
env = create_env(..., camera_depths=True)

# depth only on eye_in_hand, not agentview_left
env = robosuite.make(...,
    camera_names=["robot0_agentview_left", "robot0_eye_in_hand"],
    camera_depths=[False, True],
)
```

Note: the list form requires using `robosuite.make()` directly — `create_env()` accepts
only a single bool.
