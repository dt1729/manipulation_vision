# Contract: manipulation_vision.tactile_pnp.tactile

**Status:** accepted

## Responsibility

Turn finger-pad sensor readings and finger contacts into per-pad touch measurements.

## Interface

```python
def read_grids(data: mujoco.MjData, adr: np.ndarray) -> np.ndarray
def active_cells(grids: np.ndarray, threshold: float = ACTIVE_THRESHOLD) -> np.ndarray
def pad_contact(grids: np.ndarray, *, min_cells: int = 1,
                threshold: float = ACTIVE_THRESHOLD) -> np.ndarray
def finger_contact_forces(model: mujoco.MjModel, data: mujoco.MjData,
                          finger_bodies: tuple[int, int], other_geom: int) -> np.ndarray
```

- `read_grids` returns float32 `(2, ny, nx)` in newtons, index 0 = left pad (link7),
  1 = right pad (link8), row = along the finger (row `ny-1` nearest the fingertip),
  column = across it. A fresh array per call.
- Each cell is the normal force on one taxel bump (see `scene.PadLayout`), so a grid sum
  approximates the pad force. `finger_contact_forces` is the authoritative physical force:
  the summed contact-normal force (N) between any geom of each finger body and
  `other_geom`, from `mj_contactForce`.
- `active_cells` counts cells strictly above `threshold` per pad -> int `(2,)`.
- `pad_contact` -> bool `(2,)`: a pad is in contact when at least `min_cells` cells are active.
- All functions read `data` after `mj_step`/`mj_forward`; none mutate it.
- Cost: `read_grids` is one fancy-index gather (~5 us); `finger_contact_forces` is O(ncon).

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `adr` | int array of sensordata offsets, shape `(2, ny, nx)` | **propagate** - built by `scene` |
| `grids` | shape `(2, ny, nx)` | **reject** - `ValueError` with the shape |
| `threshold` | `>= 0` | **reject** - `ValueError` |
| `min_cells` | `>= 1` | **reject** - `ValueError` |
| `finger_bodies`, `other_geom` | valid body / geom ids | **propagate** |

## Seam

None. The sensor is MuJoCo `<touch>` and nothing else exists in this repo's sim path. If a
learned or `touch_grid`-plugin sensor is added it replaces `read_grids` only; callers
depend on the `(2, ny, nx)` newton array, not on the sensor type.

## Dependencies

- **May import:** `mujoco`, `numpy`, stdlib
- **Must not import:** `scene` (receives addresses, not a Scene), `pnp`, `recorder`
- **Imported by:** `pnp`, `recorder`, `preview`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `ValueError` | grid shape or threshold invalid | programming error |

## Observability

No logging.

## Tests

- grids read zero with the gripper open in free space
- squeezing a box between the pads gives nonzero readings on both pads
- `pad_contact` is false for an all-zero grid and true when one cell exceeds threshold
- `finger_contact_forces` is zero with no contact and positive per finger when squeezing
- a `(16, 16)` grid (missing pad axis) is rejected with `ValueError`
- a negative threshold is rejected

## Deletion test

Without it, the grasp state machine, the logger and the preview each re-derive pad contact
from raw sensordata offsets and would disagree on the activity threshold.
