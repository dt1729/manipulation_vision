# Contract: manipulation_vision.tactile_pnp.cameras

**Status:** accepted

## Responsibility

Render RGB and metric depth from named MuJoCo cameras offscreen.

## Interface

```python
@dataclass(frozen=True)
class Frame:
    rgb: np.ndarray     # (H, W, 3) uint8
    depth: np.ndarray   # (H, W) float32, metres along the optical axis

class CameraRig:
    def __init__(self, model: mujoco.MjModel, cameras: Sequence[str] = ("scene_cam", "wrist_cam"),
                 width: int = 128, height: int = 128) -> None
    def capture(self, data: mujoco.MjData) -> dict[str, Frame]
    def close(self) -> None
    def __enter__(self) -> CameraRig
    def __exit__(self, *exc: object) -> None
```

- Needs a GL context: set `MUJOCO_GL=egl` (headless) before `mujoco` is first imported;
  the CLI does this.
- `capture` returns new arrays (callers may keep them); the order of keys is `cameras`.
- The rig is bound to one `MjModel`; a new scene needs a new rig.
- `close()` must be called (or use `with`): it frees the GL context explicitly so the
  renderer's deallocator does not print errors at interpreter exit. Idempotent.
- Cost: ~1-2 ms per camera at 128x128 (two passes: colour, then depth).

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `cameras` | each names a camera in `model` | **reject** - `KeyError` from `mjutil` |
| `width`, `height` | > 0 and within the model's offscreen buffer | **reject** - `ValueError` naming the limit |
| `data` | belongs to the same model | **propagate** |

## Seam

None: MuJoCo's `Renderer` is the only backend. A real-camera adapter in the ROS stack would
produce the same `Frame` and could be swapped at the recorder, which depends only on
`capture` returning `dict[str, Frame]`.

## Dependencies

- **May import:** `mjutil`, `mujoco`, `numpy`, stdlib
- **Must not import:** `scene`, `pnp`, `recorder`, `cli`
- **Imported by:** `recorder`, `preview`, `cli`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `KeyError` | camera name not in model | fix the name |
| `ValueError` | size exceeds the offscreen buffer | build the scene with a larger `offscreen` |

## Observability

No logging.

## Tests

- capture returns both cameras with the requested shapes and dtypes
- scene-camera depth lies within (0.1, 3) m everywhere (it sees the table, not infinity)
- the wrist camera image changes when the arm moves
- a width larger than the offscreen buffer is rejected with `ValueError`
- close is idempotent

## Deletion test

Without it the recorder and the preview each manage renderers, depth toggling and explicit
`close()`; the deallocator noise noted in the repo would return.
