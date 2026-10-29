# Contract: manipulation_vision.tactile_pnp.scene

**Status:** accepted

## Responsibility

Compile one randomized Piper tabletop pick-and-place task into a MuJoCo model with resolved handles.

## Interface

```python
@dataclass(frozen=True)
class ObjectSpec:
    kind: Literal["box", "cylinder", "sphere"]
    half_extents: tuple[float, float, float]   # m, object frame; sphere = (r, r, r)
    mass: float                                # kg
    friction: float                            # sliding coefficient
    pos: tuple[float, float, float]            # m, world, object centre at rest on table
    yaw: float                                 # rad about world z

@dataclass(frozen=True)
class TaskSpec:
    obj: ObjectSpec
    place_xy: tuple[float, float]              # m, world; target for the object centre

@dataclass(frozen=True)
class Workspace: ...                           # sampling bounds; defaults are reachable

def sample_task(rng: np.random.Generator, ws: Workspace | None = None) -> TaskSpec
def build_scene(task: TaskSpec, *, offscreen: tuple[int, int] = (640, 480),
                pad: PadLayout | None = None) -> Scene
def build_xml(task: TaskSpec, *, offscreen: tuple[int, int] = (640, 480),
              pad: PadLayout | None = None) -> str
def reset_to_home(scene: Scene) -> None

@dataclass
class Scene:
    model: mujoco.MjModel
    data: mujoco.MjData
    task: TaskSpec
    handles: Handles          # every index a controller / logger needs
    tactile_adr: np.ndarray   # (2, 16, 16) int sensordata offsets, [left, right]
```

- World frame: robot base at origin, table top at z = 0, +x away from the robot.
- The returned `data` is reset to the home posture with the gripper open and
  `mj_forward` applied, so `data` is valid for reading immediately.
- The object rests on the table: centre z equals its half-height (sphere: radius).
- Physics options follow `scripts/config/piper_behavior.json`: timestep 1 ms,
  IMPLICITFAST, 20 solver iterations.
- 512 `<touch>` sensors (16x16 per pad, link7 = left, link8 = right) on the pad region
  used by `scripts/orange_individual_touch.py`. Each taxel is a 0.9 mm collision sphere
  ("bump") standing 0.6 mm proud of the finger face, with a 1.4 mm touch site enclosing only
  that bump. (Design change found in testing: flat-on-flat contact against the finger's
  convex-hull mesh produces contact points only at the polygon corners, which fall outside
  the pad, so boxes read 0 taxels.) The robot file's `touch_grid` plugin sensors are removed.
- Collision filtering: arm links and finger meshes contype/conaffinity 1; taxel bumps,
  table and object 2. The object touches only the bumps and the table; arm links never hit
  the table. `base`/`link1` contact is excluded (welded parent, meshes overlap 6 mm).
- Finger joints get armature 0.5 (geared drive inertia); without it the 25 g fingers
  chatter against ~256 parallel bump contacts. The object has priority 1 so its friction
  and solref (10 ms) govern its contacts.
- Robot `*_col` geoms are moved to group 3 and shadows are off: under software EGL this
  takes a 128 px frame from ~360 ms to ~42 ms. robot.xml's `nconmax=500` cap is removed
  (a box face on both pads makes ~512 contacts).
- The default `Workspace` keeps objects at radius 0.22-0.35 m and <= 80 mm tall because a
  straight-down TCP is only reachable up to z ~ 0.11 m there (joint2 >= 0, |joint5| <= 1.22).
- Two cameras: `scene_cam` (fixed, overlooks the workspace) and `wrist_cam` (on
  `right_hand`, looking along the approach axis between the fingers).
- A tool-centre-point site `tcp` sits 10 mm behind the fingertips on the hand axis.
- `sample_task` is deterministic given the generator state; it consumes the generator.
- Building takes ~0.2 s (XML parse + compile of 512 bumps/sites); call once per episode.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `rng` | a `numpy.random.Generator` | **propagate** |
| `ws` | each `lo <= hi`; `min_place_dist` achievable inside the bounds | **reject** - `ValueError` naming the field |
| `task.obj.half_extents` | all > 0 | **reject** - `ValueError` |
| `task.obj.mass` | > 0 | **reject** - `ValueError` |
| `offscreen` | both > 0 | **propagate** - MuJoCo rejects at render time |

## Seam

What varies per episode is the `TaskSpec`; sampling and building are split so a caller can
replay a recorded spec (the logger stores it) without the RNG. Object kinds are a closed
`Literal`, not a plugin registry: three primitives exist and a fourth kind is a code change
here anyway (MJCF geom type and grasp axis).

Rejected: mutating a template `MjModel` per episode (`model.geom_size[...] = ...`). Mass,
inertia and primitive type changes need recompilation to stay consistent; patching them
in place produces a model whose inertia does not match its geometry.

## Dependencies

- **May import:** `mjutil`, `mujoco`, `numpy`, stdlib
- **Must not import:** `control`, `pnp`, `recorder`, `cameras`, `cli`
- **Imported by:** `control`, `pnp`, `recorder`, `cameras`, `preview`, `cli`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `ValueError` | a spec or workspace field is out of range | fix the spec |
| `FileNotFoundError` | the robosuite submodule (robot MJCF) is missing | `git submodule update --init` |

## Observability

- **DEBUG** `scene.built` - object kind, size, mass, friction, place target.
- Never logged: the generated XML (tens of KB, 512 sensor lines).

## Tests

- a built scene has exactly 512 touch sensors and a (2,16,16) address array of distinct offsets
- the object body rests on the table: after 0.2 s of simulation it has moved < 2 mm
- two `sample_task` calls with equal seeds give equal specs; different seeds differ
- a zero mass is rejected with `ValueError`
- a workspace whose bounds cannot satisfy `min_place_dist` is rejected
- building does not write any file into the robosuite submodule

## Deletion test

Without it, the CLI, the tests and the preview each re-implement XML injection of the
tactile grid, the cameras and the object, which is exactly how the four copies in
`scripts/` came about.
