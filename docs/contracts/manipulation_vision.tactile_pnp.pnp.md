# Contract: manipulation_vision.tactile_pnp.pnp

**Status:** accepted

## Responsibility

Run one scripted, tactile-gated pick-and-place episode on a built scene and judge its outcome.

## Interface

```python
class Phase(IntEnum):
    APPROACH, PREGRASP, DESCEND, GRASP, LIFT, TRANSPORT, PLACE, RELEASE, RETREAT, SETTLE

@dataclass(frozen=True)
class PnpConfig: ...            # rates, speeds, heights, thresholds; all SI units

@dataclass(frozen=True)
class StepInfo:
    t: float                    # s sim time after this control step
    phase: Phase
    action: np.ndarray          # (7,) commanded arm joints (rad) + commanded gripper gap (m)
    tactile: np.ndarray         # (2, 16, 16) N, read after the physics substeps

@dataclass(frozen=True)
class EpisodeResult:
    success: bool
    failure: str | None         # None | "ik" | "missed_grasp" | "slip" | "misplaced" | "timeout"
    place_error: float          # m, object-centre xy distance to the place target
    sim_time: float             # s
    peak_active: dict[str, list[int]]   # phase name -> peak active cells [left, right]
    grasp_force: list[float]    # N per finger, physical contact force at grasp confirmation

Observer = Callable[[Scene, StepInfo], None]

def run_episode(scene: Scene, cfg: PnpConfig | None = None,
                observers: Sequence[Observer] = ()) -> EpisodeResult
```

- Control at `cfg.control_hz` (default 100 Hz) = `1 / (control_hz * timestep)` physics
  substeps per control step. Every observer is called once per control step, in order,
  after the substeps, with `scene.data` reflecting that instant.
- The object target is taken from `scene.task.obj` (a perception stand-in), not from the
  simulator state, so a displaced object produces a genuine miss.
- Phases run in enum order; a failure skips straight to `RETREAT` (gripper opened) and
  then `SETTLE`, so the logged episode always ends at rest with the gripper open.
- `GRASP`: each finger closes at `close_speed` until **its own pad** reports contact
  (`tactile.pad_contact`) and holds there; once **both** pads are in contact, both are
  commanded `squeeze` past their contact positions. The grasp is confirmed only if both
  pads still read contact after `grasp_hold`; a finger reaching the closed limit first is
  `missed_grasp`.
- `LIFT` fails with `slip` if either pad loses contact or the object rises less than half
  the lift height. `TRANSPORT`/`PLACE` fail with `slip` on loss of contact. Loss of
  contact means no active taxel on a pad for `slip_debounce` (50 ms) continuously.
- The grasp yaw is chosen among candidates (box: across its narrow side, both signs;
  round: 8 yaws) that are IK-reachable at both pick and place, since the wrist holds it
  during transport; ties go to the least joint motion.
- Success = after `SETTLE`, object xy within `place_tol` of the target, object resting
  (centre within 1 cm of its rest height), and no finger contact with the object.
- Approach uses a time-synchronised joint profile; every other arm motion is a straight
  Cartesian segment (`Kinematics.plan_linear`).
- Mutates `scene.data` (it is the episode); does not reset it first.
- Cost: ~6-9 s of sim per episode; wall time is dominated by `mj_step` with 512 touch
  sensors (~0.15 ms/step) plus observers.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `scene` | freshly built or `reset_to_home`'d | **propagate** - a mid-motion scene gives a jerky first move |
| `cfg.control_hz` | divides 1/timestep into an integer >= 1 | **reject** - `ValueError` |
| `cfg` speeds, tolerances | > 0 | **reject** - `ValueError` naming the field |
| `observers` | callables that do not mutate `scene.data` | **propagate** |

## Seam

`observers` is the seam: the logger, the preview video and the metrics collector are all
observers, and the episode does not know about any of them. Rejected: the runner owning a
recorder (would force HDF5 on tests and the preview), and a generator yielding steps
(observers would then also need to see failure transitions, which the result already carries).

The grasp policy is not behind a seam: one scripted policy exists. The next policy (learned,
from this data) replaces `run_episode` wholesale rather than plugging into it.

## Dependencies

- **May import:** `scene`, `control`, `tactile`, `mujoco`, `numpy`, stdlib
- **Must not import:** `recorder`, `cameras`, `preview`, `cli`
- **Imported by:** `recorder` (types only), `preview`, `cli`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `ValueError` | `PnpConfig` field invalid | fix the config |

IK failure, missed grasps and slips are outcomes, reported in `EpisodeResult.failure`, never
raised: a data-collection loop must keep going.

## Observability

- **INFO** `episode.done` - success, failure reason, place error, sim time.
- **DEBUG** one line per phase transition with sim time.
- Never logged: per-step tactile arrays.

## Tests

- a seeded episode with a reachable task succeeds and places within tolerance
- observers see phases in non-decreasing enum order and time advancing by 1/control_hz
- the grasp is confirmed only with both pads active (peak active cells > 0 on both in GRASP)
- an object displaced 8 cm from its spec yields `missed_grasp`, not an exception
- the episode ends with the gripper open (last action gap == fully open)
- a control rate that does not divide the physics rate is rejected

## Deletion test

Without it the CLI would hold the phase logic and the tests could not run an episode without
writing HDF5; the tactile grasp rule would have nowhere single to live.
