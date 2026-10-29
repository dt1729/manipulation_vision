"""Name-to-index lookups for MuJoCo models that raise on a missing name.

Contract: docs/contracts/manipulation_vision.tactile_pnp.mjutil.md

``mujoco.mj_name2id`` returns ``-1`` for an unknown name, and ``array[-1]`` then silently
reads the last element. Every lookup here raises ``KeyError`` instead.
"""

from __future__ import annotations

import mujoco

_KIND_LABEL = {
    mujoco.mjtObj.mjOBJ_JOINT: "joint",
    mujoco.mjtObj.mjOBJ_ACTUATOR: "actuator",
    mujoco.mjtObj.mjOBJ_BODY: "body",
    mujoco.mjtObj.mjOBJ_SITE: "site",
    mujoco.mjtObj.mjOBJ_GEOM: "geom",
    mujoco.mjtObj.mjOBJ_CAMERA: "camera",
    mujoco.mjtObj.mjOBJ_SENSOR: "sensor",
}


def obj_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    """Index of the element ``name`` of type ``kind``; ``KeyError`` if absent."""
    idx = int(mujoco.mj_name2id(model, kind, name))
    if idx < 0:
        label = _KIND_LABEL.get(kind, str(kind))
        raise KeyError(f"{label} '{name}' not in model")
    return idx


def joint_qpos_adr(model: mujoco.MjModel, name: str) -> int:
    """Offset of the joint's first coordinate in ``data.qpos``."""
    return int(model.jnt_qposadr[obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])


def joint_dof_adr(model: mujoco.MjModel, name: str) -> int:
    """Offset of the joint's first degree of freedom in ``data.qvel``."""
    return int(model.jnt_dofadr[obj_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    """Index into ``data.ctrl``."""
    return obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def body_id(model: mujoco.MjModel, name: str) -> int:
    return obj_id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def site_id(model: mujoco.MjModel, name: str) -> int:
    return obj_id(model, mujoco.mjtObj.mjOBJ_SITE, name)


def geom_id(model: mujoco.MjModel, name: str) -> int:
    return obj_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)


def camera_id(model: mujoco.MjModel, name: str) -> int:
    return obj_id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)


def sensor_adr(model: mujoco.MjModel, name: str) -> int:
    """Offset of the sensor's first value in ``data.sensordata`` (not the sensor id)."""
    return int(model.sensor_adr[obj_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)])
