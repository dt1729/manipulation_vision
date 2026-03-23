# Adding a Robot to Robosuite + Robocasa

Step-by-step guide for adding a new robot (illustrated with PiperArm / PiperOmron).

---

## 1. Add Mesh Assets

Place all `.STL` mesh files in:
```
robosuite/models/assets/robots/<robot_name>/meshes/
```

MuJoCo only supports STL and OBJ formats — not DAE.

---

## 2. Create the MJCF Robot XML

Create `robosuite/models/assets/robots/<robot_name>/robot.xml`.

### 2a — Declare meshes in `<asset>`
```xml
<asset>
    <mesh name="link1" file="meshes/link1.STL"/>
    ...
</asset>
```

### 2b — Add visual + collision geoms for every link
Robosuite uses MuJoCo geom groups:
- Visual: `group="1" contype="0" conaffinity="0"`
- Collision: no group attribute, add `name="<link>_col"`

```xml
<geom type="mesh" mesh="link1" contype="0" conaffinity="0" group="1"/>
<geom type="mesh" mesh="link1" name="link1_col"/>
```

### 2c — Add `right_hand` body
Robosuite requires a body named `right_hand` as the EEF frame, placed between the last arm link and the gripper fingers. Use identity quaternion:

```xml
<body name="right_hand" pos="0 0 <offset>" quat="1 0 0 0">
    <inertial pos="0 0 0" mass="0" diaginertia="0 0 0"/>
    <!-- gripper finger bodies -->
</body>
```

### 2d — Gripper finger orientations
Source finger body `pos` and `quat` directly from the robot's reference simulation XML — do not guess. For PiperArm, copied from `piper_description.xml` in piper_sim:

```xml
<body name="link7" pos="0 0 0" quat="0.707105 0.707108 0 0"> ... </body>
<body name="link8" pos="0 0 0" quat="-2.59734e-06 -2.59735e-06 -0.707108 -0.707105"> ... </body>
```

---

## 3. Create the Manipulator Model Class

**File:** `robosuite/models/robots/manipulators/<robot>_robot.py`

```python
class PiperArm(ManipulatorModel):
    arms = ["right"]

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/piper_arm/robot.xml"), idn=idn)

    @property
    def default_base(self):
        return "RethinkMount"       # for fixed-base use

    @property
    def default_gripper(self):
        return {"right": None}      # gripper is embedded in the XML, not a separate model

    @property
    def default_controller_config(self):
        return {"right": "default_piperarm"}

    @property
    def init_qpos(self):
        return np.array([0, 0, 0, 0, 0, 0, 0, 0])

    @property
    def base_xpos_offset(self):
        return {"bins": (-0.5, -0.1, 0), "empty": (-0.6, 0, 0),
                "table": lambda table_length: (-0.16 - table_length / 2, 0, 0)}

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"
```

Register it in `robosuite/models/robots/manipulators/__init__.py`:
```python
from .piper_robot import PiperArm
```

---

## 4. Create the Compositional Mobile Robot (optional)

**File:** `robosuite/models/robots/compositional.py`

Subclass the arm and override `default_base` to attach a mobile base:

```python
class PiperOmron(PiperArm):
    @property
    def default_base(self):
        return "OmronMobileBase"

    @property
    def default_arms(self):
        return {"right": "PiperArm"}

    @property
    def default_controller_config(self):
        # "torso" key is required — OmronMobileBase has actuator_torso_height
        return {"right": "default_piperomron", "torso": "default_piperomron"}

    @property
    def init_qpos(self):
        # Tune this so the arm is upright and visible from the kitchen camera
        return np.array([0.0, 1.57, -1.57, 0.0, 1.22, 0.0, 0.0, 0.0])

    @property
    def base_xpos_offset(self):
        return {"bins": (-0.6, -0.1, 0), "empty": (-0.6, 0, 0),
                "table": lambda table_length: (-0.16 - table_length / 2, 0, 0)}
```

---

## 5. Register in Robot Class Mapping

**File:** `robosuite/robots/__init__.py`

```python
ROBOT_CLASS_MAPPING = {
    ...
    "PiperArm":   FixedBaseRobot,
    "PiperOmron": WheeledRobot,    # mobile base robots use WheeledRobot
}
```

---

## 6. Create Controller Config JSON

Filename must match `default_<lowercase_robot_name>.json`.

**Fixed base** (`default_piperarm.json`): use a standard `OSC_POSE` or `JOINT_POSITION` config.

**Mobile base** (`default_piperomron.json`): must use `HYBRID_MOBILE_BASE` with three body parts — `arms`, `torso`, and `base`:

```json
{
    "type": "HYBRID_MOBILE_BASE",
    "body_parts": {
        "arms": {
            "right": {
                "type": "OSC_POSE",
                "input_max": 1, "input_min": -1,
                "output_max": [0.05, 0.05, 0.05, 0.5, 0.5, 0.5],
                "output_min": [-0.05, -0.05, -0.05, -0.5, -0.5, -0.5],
                "kp": 150, "damping_ratio": 1, "impedance_mode": "fixed",
                "kp_limits": [0, 300], "damping_ratio_limits": [0, 10],
                "position_limits": null, "orientation_limits": null,
                "uncouple_pos_ori": true, "input_type": "delta",
                "input_ref_frame": "base", "interpolation": null,
                "ramp_ratio": 0.2,
                "gripper": {"type": "GRIP"}
            }
        },
        "torso": {"type": "JOINT_POSITION", "interpolation": "null", "kp": 2000},
        "base":  {"type": "JOINT_VELOCITY",  "interpolation": "null"}
    }
}
```

Place JSON files in `robosuite/controllers/config/robots/`.

---

## 7. Run the Demo

```bash
conda activate robocasa
python -m robocasa.demos.demo_kitchen_states --task PnPCounterToCab --robot PiperOmron
```
