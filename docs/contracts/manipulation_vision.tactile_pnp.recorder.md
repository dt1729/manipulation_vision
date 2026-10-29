# Contract: manipulation_vision.tactile_pnp.recorder

**Status:** accepted

## Responsibility

Persist pick-and-place episodes as a robomimic-style HDF5 dataset with tactile and camera streams.

## Interface

```python
class EpisodeRecorder:                       # an pnp.Observer
    def __init__(self, *, log_every: int = 1, rig: CameraRig | None = None,
                 camera_hz: float = 20.0) -> None
    def __call__(self, scene: Scene, info: StepInfo) -> None
    @property
    def num_samples(self) -> int

class DatasetWriter:
    def __init__(self, path: Path, *, env_meta: dict[str, object]) -> None
    def write(self, rec: EpisodeRecorder, task: TaskSpec, result: EpisodeResult,
              *, seed: list[int]) -> str          # returns "demo_<k>"
    def close(self) -> None                       # also via context manager
```

Layout (`T` = logged steps, `F` = camera frames):

```
data/                       attrs: total, env_args (JSON), phase_names, tactile_unit="N",
                                   depth_unit="mm", quat_order="wxyz"
  demo_k/                   attrs: num_samples, success, failure, place_error, sim_time,
                                   seed, task (JSON of TaskSpec), obj_kind, obj_mass,
                                   obj_friction, obj_half_extents, obj_pos, obj_yaw, place_xy,
                                   grasp_force, peak_active (JSON)
    actions        (T, 7)   arm joint targets (rad) + commanded gripper gap (m)
    rewards, dones (T,)     1 on the last step iff success / always 1 on the last step
    states         (T, nq+nv)  full MuJoCo qpos|qvel for replay
    obs/
      t (T,), phase (T,) int8,
      robot0_joint_pos, robot0_joint_vel (T, 6), robot0_gripper_qpos (T, 2),
      robot0_gripper_width (T, 1), robot0_eef_pos (T, 3), robot0_eef_quat (T, 4),
      tactile_left, tactile_right (T, 16, 16) float32 N,
      tactile_active (T, 2) int16, contact_force (T, 2) N,
      object_pos (T, 3), object_quat (T, 4)
    images/                 only when a rig is given
      t (F,), step_index (F,) -> index into obs
      <camera>_rgb (F, H, W, 3) uint8, <camera>_depth (F, H, W) uint16 mm
```

- `obs/*`, `actions`, `rewards`, `dones`, `states` all have length `T`, which is what
  robomimic's `SequenceDataset` requires. Images run at a lower rate and therefore live in
  `images/` with `step_index`; to train robomimic on images, record with
  `camera_hz == log rate` and copy them into `obs/` (they then align 1:1).
- The recorder logs every `log_every`-th control step; cameras fire on the first logged step
  at or after each `1/camera_hz` tick, so the camera rate is capped by the log rate.
- The recorder owns nothing GL-related: the caller owns and closes the `CameraRig`.
- A recorder is single-use (one episode). `DatasetWriter` numbers demos 0, 1, 2 ... in
  write order and flushes after each, so a crash loses at most the current episode.
- Images are gzip-4 compressed with one chunk per frame; low-dim arrays are gzip-4 too.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `log_every` | >= 1 | **reject** - `ValueError` |
| `camera_hz` | > 0 | **reject** - `ValueError` |
| `path` | parent directory writable; file not existing | **reject** - `FileExistsError` (never overwrite a dataset) |
| `rec` | has at least one sample | **reject** - `ValueError` |

## Seam

The observer callback (`pnp.Observer`) is the seam that lets this module exist without the
episode runner knowing HDF5. Storage format is not behind a seam: one format (HDF5,
robomimic layout) is wanted. Rejected: zarr/LeRobot writers now - no consumer yet.

## Dependencies

- **May import:** `scene`, `pnp` (types), `tactile`, `cameras` (types), `h5py`, `numpy`, `mujoco`, stdlib
- **Must not import:** `cli`, `preview`
- **Imported by:** `cli`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `FileExistsError` | the dataset file already exists | choose a new run directory |
| `ValueError` | invalid rates, or writing an empty recorder | programming error |

## Observability

- **DEBUG** `demo.written` - demo key, samples, frames, success.
- Never logged: array contents.

## Tests

- a recorded episode round-trips: every obs key has num_samples rows and actions has 7 columns
- tactile rows equal the StepInfo tactile passed in (no transposition of pads)
- camera frames arrive at ~camera_hz and step_index points at matching obs times
- the dataset refuses to overwrite an existing file
- writing an empty recorder is rejected
- log_every=0 is rejected

## Deletion test

Without it the CLI would build the HDF5 layout inline, and a second consumer (a converter to
robocasa/LeRobot) would have no single definition of the schema to read.
