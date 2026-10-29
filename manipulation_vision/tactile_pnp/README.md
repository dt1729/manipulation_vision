# tactile_pnp — standalone MuJoCo pick-and-place with tactile + RGB-D (Piper)

Scripted, tactile-gated pick-and-place of randomized primitive objects with the Piper arm,
logged as a robomimic-style HDF5 dataset. No robosuite/robocasa runtime dependency: it only
reads the Piper MJCF from `includes/robosuite`.

## Run

```bash
export PIP_CACHE_DIR=/data/cache/pip
/data/venvs/manipulation_vision/bin/pip install mujoco numpy h5py imageio imageio-ffmpeg \
    opencv-python-headless pytest
cd /data/repos/manipulation_vision
/data/venvs/manipulation_vision/bin/python -m manipulation_vision.tactile_pnp \
    --episodes 20 --seed 0 --out /data/datasets/tactile_pnp \
    [--no-camera] [--log-hz 100] [--camera-hz 20] [--cam-size 128 128] \
    [--preview /data/datasets/tactile_pnp/preview.mp4]
/data/venvs/manipulation_vision/bin/python -m pytest manipulation_vision/tactile_pnp/tests
```

Writes `<out>/<run>/demos.hdf5` and `summary.json`. Headless rendering uses `MUJOCO_GL=egl`
(set automatically). On this host EGL is Mesa llvmpipe (CPU), so cameras dominate wall time.

## Modules (contracts in `docs/contracts/manipulation_vision.tactile_pnp.*.md`)

| module | job |
| --- | --- |
| `mjutil` | name -> index lookups that raise instead of returning -1 (replaces the copies in `scripts/*touch*.py`) |
| `scene` | `sample_task(rng)` -> `TaskSpec`; `build_scene(task)` -> MJCF with table, object, 2x16x16 taxels, cameras |
| `tactile` | read the `(2,16,16)` grids, active cells, pad contact, physical finger forces |
| `control` | DLS IK on site Jacobians, time-synchronised trapezoid profiles, straight Cartesian segments |
| `pnp` | the phase machine; observers get every control step |
| `cameras` | offscreen RGB + metric depth |
| `recorder` | HDF5 writer (observer) |
| `preview` | MP4 with scene/wrist cams + tactile heatmaps (observer) |
| `cli` | composition root |

## Dataset layout

See `docs/contracts/manipulation_vision.tactile_pnp.recorder.md`. In short:
`data/demo_k/{actions,rewards,dones,states,obs/*}` all of length T (robomimic), images at a
lower rate in `data/demo_k/images/` with `step_index` into `obs`. Episode randomization and
outcome are attributes on `demo_k`.

## Simulation choices worth knowing

- Taxels are physical: each of the 512 sensors sits on its own 0.9 mm collision bump.
  Against the finger's convex-hull mesh, a flat object face only ever produced contacts at
  polygon corners outside the pad, i.e. zero taxels on boxes.
- Finger joints carry 0.5 kg armature (geared drive); without it the fingers chatter.
- Top-down TCP is reachable only up to z ~ 0.11 m at 0.2-0.33 m radius, which bounds the
  workspace and object height.
