# Contract: manipulation_vision.tactile_pnp.cli

**Status:** accepted

## Responsibility

Compose scene, episode runner, recorder and preview into a seeded data-collection run.

## Interface

```python
def main(argv: Sequence[str] | None = None) -> int
def collect(cfg: RunConfig) -> RunSummary

@dataclass(frozen=True)
class RunConfig:
    episodes: int; seed: int; out: Path; run_name: str | None
    cameras: bool; log_hz: float; camera_hz: float; cam_size: tuple[int, int]
    preview: Path | None

@dataclass(frozen=True)
class RunSummary:
    run_dir: Path; dataset: Path; episodes: int; successes: int
    failures: dict[str, int]; mean_wall_s: float; realtime_factor: float
    dataset_bytes: int; peak_active: dict[str, dict[str, float]]
```

```
python -m manipulation_vision.tactile_pnp --episodes N --seed S --out DIR [--no-camera]
       [--run-name NAME] [--log-hz 100] [--camera-hz 20] [--cam-size 128 128]
       [--preview PATH] [--preview-episode K]
```

- Writes `<out>/<run_name>/demos.hdf5` and `<out>/<run_name>/summary.json`; default run
  name `run_<UTC timestamp>_s<seed>`. Never overwrites an existing dataset.
- Episode `i` uses `numpy.random.default_rng([seed, i])`, so any single episode is
  reproducible without replaying the earlier ones.
- `--preview PATH [--preview-episode K]` re-runs episode `K` (default 0; any index, even
  beyond `--episodes`) of the same seed into an MP4; physics is deterministic on one machine.
- `realtime_factor` = total simulated seconds / total wall seconds of the episode loops
  (excludes the preview re-run).
- Sets `MUJOCO_GL=egl` if unset before importing mujoco. Exit code 0 when the run
  completes, regardless of the success rate (that is data, not an error).

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `--episodes` | >= 1 | **reject** - argparse error, exit 2 |
| `--log-hz` | divides the 100 Hz control rate | **reject** - argparse error, exit 2 |
| `--camera-hz` | > 0 and <= log rate | **reject** - argparse error, exit 2 |
| `--out` | writable | **propagate** - `OSError` |

## Seam

None: this is the composition root. Everything it wires is injected into `run_episode`
as observers, so nothing below it knows about argparse, files or rates.

## Dependencies

- **May import:** every `tactile_pnp` module, `argparse`, stdlib
- **Must not import:** nothing imports this module except `__main__`
- **Imported by:** `__main__`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `SystemExit(2)` | invalid CLI arguments | fix the command line |
| `FileExistsError` | the run directory already has a dataset | pick another `--run-name` |

## Observability

- **INFO** `run.start` - run dir, episodes, seed, cameras on/off.
- **INFO** `episode.done` (from `pnp`) plus one progress line per episode with wall time.
- **INFO** `run.done` - success rate, realtime factor, dataset size.
- Never logged: array data.

## Tests

- one episode with cameras writes a valid HDF5 (robomimic keys, images, success attr) and summary.json
- `--no-camera` writes no `images` group
- zero episodes is rejected with exit code 2
- a log rate that does not divide the control rate is rejected with exit code 2

## Deletion test

Without it there is no runnable collection; each user would re-wire the observers by hand.
