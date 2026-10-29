# Contract: manipulation_vision.tactile_pnp.mjutil

**Status:** accepted

## Responsibility

Resolve MuJoCo element names to indices, failing loudly when a name does not exist.

## Interface

```python
def obj_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int
def joint_qpos_adr(model: mujoco.MjModel, name: str) -> int
def joint_dof_adr(model: mujoco.MjModel, name: str) -> int
def actuator_id(model: mujoco.MjModel, name: str) -> int
def body_id(model: mujoco.MjModel, name: str) -> int
def site_id(model: mujoco.MjModel, name: str) -> int
def geom_id(model: mujoco.MjModel, name: str) -> int
def camera_id(model: mujoco.MjModel, name: str) -> int
def sensor_adr(model: mujoco.MjModel, name: str) -> int
```

- Replaces the `jnt_adr/act_id/body_id/site_id/geom_id` one-liners that the four tactile
  scripts in `scripts/` each copy. Those return `-1` (or index with `-1`, which silently
  reads the *last* element) when a name is misspelt; these raise instead.
- Pure lookups on an immutable model; O(1) hash lookups inside MuJoCo, safe to call per
  build but callers should cache indices rather than resolve per physics step.

## Preconditions

| Parameter | Must satisfy | On violation |
| --- | --- | --- |
| `model` | a compiled `MjModel` | **propagate** - the caller built it |
| `name` | names an element of the requested kind in `model` | **reject** - `KeyError` naming kind and name |

## Seam

Nothing varies: there is one MuJoCo name table. A class wrapping the model was rejected:
it would add a lifetime to manage for no state beyond the model the caller already holds.

## Dependencies

- **May import:** `mujoco`, stdlib
- **Must not import:** any other `tactile_pnp` module (this is the leaf)
- **Imported by:** `scene`, `tactile`, `control`, `pnp`, `cameras`

## Errors

| Type | Raised when | Caller response |
| --- | --- | --- |
| `KeyError` | the name is absent for that element kind | fix the name; it is a programming error |

## Observability

No logging.

## Tests

- resolves `joint1` to the same qpos address MuJoCo reports
- an unknown joint name raises `KeyError` carrying the name
- an unknown site name raises `KeyError` (not `-1`)
- `sensor_adr` returns the sensordata offset, not the sensor id

## Deletion test

Deleting this module puts the `-1`-returning one-liners back into every caller, which is
the copy-paste this package exists partly to end.
