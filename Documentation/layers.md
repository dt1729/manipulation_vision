# Simulation Stack Layers

Overview of how MuJoCo, Robosuite, Robocasa, and Gymnasium are integrated.

```
MuJoCo (physics)
  └── binding_utils.MjSim
        └── robosuite MujocoEnv / RobotEnv / ManipulationEnv
              └── robocasa Kitchen / KitchenArena / scene_builder
                    └── RoboCasaGymEnv (gymnasium API)
```

---

## Layer 1 — Core MuJoCo Simulation

**`robosuite/utils/binding_utils.py`**
- `MjSim` — direct MuJoCo wrapper
  - `from_xml_string()` / `from_xml_file()` — load physics model from MJCF XML
  - `step()` / `step1()` / `step2()` — advance physics simulation
  - `reset()` / `forward()` / `render()` — state management and offscreen rendering

**`robosuite/models/base.py`**
- `MujocoXML` — parses and merges MJCF XML files
  - `create_default_element()` — manages worldbody/actuator/asset/sensor/tendon/equality/contact sections
  - `resolve_asset_dependency()` — converts relative mesh paths to absolute paths

**`robosuite/models/world.py`** — composes multiple robot/arena/object models into one world XML

---

## Layer 2 — MuJoCo → Robosuite Integration

**`robosuite/environments/base.py`**
- `MujocoEnv` — base class wrapping `MjSim`
  - `_load_model()` — abstract; subclasses build the MJCF model here
  - `_initialize_sim()` — creates `MjSim` from XML string, applies XML processors
  - `reset()` — hard/soft reset, reinitializes observables
  - `step()` — control loop calling `sim.step1()` / `sim.step2()`
  - `_setup_references()` — maps model body/geom names → sim indices
  - `_setup_observables()` — registers observation sensors

**`robosuite/environments/robot_env.py`**
- `RobotEnv` — extends `MujocoEnv`, adds robot loading
  - `_load_model()` — calls `_load_robots()` to inject robot XMLs into the world
  - `_setup_references()` / `_setup_observables()` — aggregates robot-specific data

**`robosuite/environments/manipulation/manipulation_env.py`**
- `ManipulationEnv` — extends `RobotEnv` for task-based manipulation environments

**`robosuite/utils/sim_utils.py`**
- `check_contact()` / `get_contacts()` — contact detection via `sim.data.contact`

---

## Layer 3 — Robosuite → Robocasa Integration

**`robocasa/environments/kitchen/kitchen.py`**
- `Kitchen` — extends `ManipulationEnv`
  - `_load_model()` — calls super then `_setup_model()`
  - `_setup_model()` — loads kitchen arena + fixtures + task objects, handles placement with retry logic (up to 50 attempts)

**`robocasa/models/scenes/kitchen_arena.py`**
- `KitchenArena` — extends robosuite `Arena`
  - Loads layout and style from YAML configs
  - Merges all fixture XMLs into a single worldbody

**`robocasa/models/scenes/scene_builder.py`**
- `create_fixtures()` — factory mapping fixture names → classes (HingeCabinet, Drawer, Stove, Sink, Fridge, Dishwasher, etc.)

**`robocasa/utils/env_utils.py`**
- `create_env()` — high-level environment factory; handles robocasa-specific configs (`layout_and_style_ids`, `obj_instance_split`, `split`) then calls `robosuite.make()` internally

---

## Layer 4 — Robocasa → Gymnasium Integration

**`robocasa/wrappers/gym_wrapper.py`**
- `RoboCasaGymEnv` — Gymnasium-compliant wrapper around `Kitchen`
  - `reset(seed, options)` — Gymnasium-standard reset
  - `step(action_dict)` — returns `(obs, reward, terminated, truncated, info)`
- `PandaOmronKeyConverter` — maps between robosuite and standardized obs/action keys
  - `map_obs()` / `map_obs_in_eval()` — robot state → standardized observation keys
  - `deduce_observation_space()` / `deduce_action_space()` — returns `gymnasium.spaces.Dict`
  - `unmap_action()` — standardized action → robosuite format

**`robosuite/wrappers/gym_wrapper.py`**
- `GymWrapper` — simpler flat-array gym wrapper for any `MujocoEnv`
  - `reset()` / `step()` — gym-compatible; flattens obs to array if `flatten_obs=True`

**`robocasa/utils/robomimic/robomimic_env_wrapper.py`**
- `EnvRobocasa` — alternative wrapper used for dataset recording with robomimic

**`robocasa/utils/eval_utils.py`**
- `create_eval_env()` — eval-specific factory; enables cameras/object observations, disables depth rendering

---

## Key Entry Points

| Goal | Use |
|------|-----|
| Create a training environment | `robocasa/utils/env_utils.py` → `create_env()` |
| Gymnasium-compatible RL loop | `robocasa/wrappers/gym_wrapper.py` → `RoboCasaGymEnv` |
| Evaluation rollouts | `robocasa/utils/eval_utils.py` → `create_eval_env()` + `run_random_rollouts()` |
| Dataset recording (robomimic) | `robocasa/utils/robomimic/robomimic_env_wrapper.py` → `EnvRobocasa` |
| Custom robot in kitchen | Subclass `Kitchen`, override `_load_model()` / `_setup_model()` |
