# Contract: manipulation_vision.tactile_pnp.preview

**Status:** accepted

## Responsibility

Render a human-viewable MP4 of an episode: scene and wrist cameras beside both tactile heatmaps.

## Interface

```python
def heatmap(grid: np.ndarray, vmax: float, cell_px: int = 10) -> np.ndarray

class PreviewWriter:                     # a pnp.Observer
    def __init__(self, model: mujoco.MjModel, path: Path, *, fps: int = 30,
                 control_hz: float = 100.0, size: tuple[int, int] = (480, 360)) -> None
    def __call__(self, scene: Scene, info: StepInfo) -> None
    def close(self) -> None              # finalises the MP4 and frees the renderer
```

- `heatmap` maps a `(ny, nx)` grid in N to an RGB uint8 image `(ny*cell_px, nx*cell_px, 3)`
  using the inferno colormap, 0 -> black, `vmax` -> saturated. Rows run along the finger
  (row `ny-1`, drawn at the bottom, is nearest the fingertip), matching the dataset layout.
- Colour scale per pad is the running peak over the episode (floor 0.05 N), as in
  `scripts/orange_individual_touch.py`, so early contact is visible and later saturation is not.
- Frames are emitted every `round(control_hz / fps)` control steps; each frame is annotated
  with sim time, phase name and active-cell counts.
- Output is H.264 MP4 via imageio-ffmpeg; frame size is padded to even dimensions.
- Must be closed (or used with `with`); an unclosed writer leaves a truncated file.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `grid` | 2-D | **reject** - `ValueError` with the shape |
| `vmax` | > 0 | **clamp** to 1e-6 (no WARNING: a zero-contact episode is normal) |
| `fps`, `control_hz` | > 0, `fps <= control_hz` | **reject** - `ValueError` |
| `path` | parent exists | **propagate** - imageio raises |

## Seam

None; it is an observer like the recorder, and that shared protocol is the seam.

## Dependencies

- **May import:** `cameras`, `pnp` (types), `scene` (types), `tactile`, `cv2`, `imageio`, `numpy`, stdlib
- **Must not import:** `recorder`, `cli`
- **Imported by:** `cli`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `ValueError` | bad grid shape or rates | programming error |

## Observability

No logging (the CLI reports the output path).

## Tests

- heatmap of an all-zero grid is black and has shape (ny*cell, nx*cell, 3)
- heatmap puts the hottest colour where the grid peaks
- a 3-D grid is rejected
- fps above control_hz is rejected

## Deletion test

Without it the colormap/panel code would be copied a sixth time from the tactile scripts
into the CLI.
