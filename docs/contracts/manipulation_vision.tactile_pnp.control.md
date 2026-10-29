# Contract: manipulation_vision.tactile_pnp.control

**Status:** accepted

## Responsibility

Convert Cartesian tool-centre-point goals into time-parametrised joint reference trajectories for the Piper arm.

## Interface

```python
def trapezoid_s(distance: float, vmax: float, amax: float, dt: float) -> np.ndarray
def trapezoid_profile(q0: np.ndarray, qf: np.ndarray, vmax: float, amax: float,
                      dt: float) -> np.ndarray
def top_down_rotation(yaw: float) -> np.ndarray

@dataclass(frozen=True)
class IkResult:
    q: np.ndarray            # (6,) rad, inside joint and actuator limits
    pos_err: float           # m
    rot_err: float           # rad
    converged: bool

class Kinematics:
    def __init__(self, model: mujoco.MjModel, handles: Handles) -> None
    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]       # TCP pos (3,), R (3,3)
    def solve(self, pos: np.ndarray, rot: np.ndarray, q_seed: np.ndarray, *,
              pos_tol: float = 1e-3, rot_tol: float = 1e-2, max_iter: int = 300) -> IkResult
    def plan_linear(self, q_start: np.ndarray, pos_goal: np.ndarray, rot_goal: np.ndarray,
                    vmax: float, amax: float, dt: float) -> np.ndarray
```

- `trapezoid_s` returns the normalised path parameter `s` sampled every `dt`, from 0 to
  exactly 1, with |ds/dt| * distance <= `vmax` and |d2s/dt2| * distance <= `amax`.
  A triangular profile is used when `distance` is too short to reach `vmax`.
- `trapezoid_profile` is **time-synchronised**: every joint starts and stops together and
  moves on the straight joint-space line, the limiting joint (largest |qf - q0|) obeying
  `vmax` rad/s and `amax` rad/s^2. Shape `(N, n)`; row 0 = `q0`, row -1 = `qf` exactly.
  (`scripts/send_joint_cmd.py` lets each joint finish on its own clock; this does not.)
- `top_down_rotation(yaw)`: TCP rotation with the approach axis (hand z) pointing down and
  the finger closing axis (hand y) at `yaw` in the table plane.
- `Kinematics` owns a private `MjData`; it never touches the simulation's `data`.
- `solve` is damped least squares on the 6x6 site Jacobian (`mj_jacSite`), with a
  per-iteration step cap and clamping to the intersection of joint range and actuator
  ctrlrange. It tries `q_seed` first, then `HOME_Q`. Unreachable goals are *not* errors:
  the best attempt is returned with `converged=False` and the caller decides.
- `plan_linear` moves the TCP along a straight Cartesian segment with a trapezoidal speed
  profile on the arc length (`vmax` m/s, `amax` m/s^2), rotation held at `rot_goal`; IK is
  solved at 5 mm knots warm-started from the previous knot and joint references are
  interpolated between knots. Shape `(N, 6)`. Raises `IkError` if any knot fails.
- Cost: `solve` ~0.1-1 ms; `plan_linear` ~1 ms per cm of path.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `vmax`, `amax`, `dt` | all > 0 | **reject** - `ValueError` naming the parameter |
| `distance` | >= 0 | **reject** - `ValueError` |
| `q0`, `qf` | same shape | **reject** - `ValueError` with both shapes |
| `rot` | a proper rotation matrix | **propagate** - built by `top_down_rotation` |
| `q_seed` | shape (6,) | **propagate** |

## Seam

None today: one robot, one IK method. `Kinematics` is a class only because it owns the
scratch `MjData` and the cached limit arrays. Rejected: a generic `Controller` Protocol with
OSC/IK variants - only one implementation exists, so the Protocol would be hypothetical.
Execution (writing `data.ctrl`, stepping physics) is deliberately not here: the episode
runner owns the clock, the phase labels and the observers.

## Dependencies

- **May import:** `scene` (Handles, HOME_Q), `mujoco`, `numpy`, stdlib
- **Must not import:** `pnp`, `recorder`, `cameras`, `cli`
- **Imported by:** `pnp`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `ValueError` | profile limits or shapes invalid | programming error |
| `IkError` | a `plan_linear` knot does not converge | abort the episode as unreachable |

## Observability

No logging. Non-convergence is returned (`IkResult.converged`) or raised (`IkError`), and
the episode runner logs it with episode context.

## Tests

- a trapezoid profile starts at q0, ends exactly at qf, and all joints arrive on the same row
- the limiting joint never exceeds vmax or amax (finite differences)
- a distance too short to cruise produces a triangular profile within amax
- zero vmax is rejected with `ValueError`
- mismatched q0/qf shapes are rejected
- IK reaches a reachable top-down target within 1 mm / 0.01 rad
- IK on a target 2 m away returns `converged=False` without raising
- executing a joint profile with the position actuators brings the TCP within 5 mm of target
- `plan_linear` keeps the TCP within 2 mm of the straight segment at every knot

## Deletion test

Without it the state machine would carry Jacobian algebra and profile maths inline, and the
next controller (robocasa port) would copy it a fifth time like the tactile helpers.
