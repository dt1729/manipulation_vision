"""Randomized Piper tabletop pick-and-place scene with 2x16x16 finger-pad touch sensors.

Contract: docs/contracts/manipulation_vision.tactile_pnp.scene.md

World frame: robot base at the origin, table top at z = 0, +x away from the robot.
``sample_task`` draws a ``TaskSpec`` from a seeded generator; ``build_scene`` compiles it.
The two are split so a recorded spec can be rebuilt without the generator.
"""

from __future__ import annotations

import logging
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np

from manipulation_vision.tactile_pnp import mjutil

log = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_XML = REPO_ROOT / "includes/robosuite/robosuite/models/assets/robots/piper_arm/robot.xml"

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
FINGER_JOINTS = ("joint7", "joint8")  # left (link7), right (link8)
# Finger slide sign: joint7 opens positive, joint8 opens negative; gap = 2 * opening.
FINGER_SIGN = np.array([1.0, -1.0])
GRIPPER_MAX_OPENING = 0.035  # m per finger -> 70 mm gap (real Piper stroke)
HOME_Q = np.array([0.0, 1.57, -1.57, 0.0, 1.22, 0.0])  # scripts/config/piper_behavior.json
TCP_OFFSET = 0.010  # m behind the fingertips, on the hand approach axis
FINGER_ARMATURE = 0.5  # kg, reflected inertia of each finger's drive
SCENE_CAM_POS = np.array([0.72, -0.42, 0.55])
SCENE_CAM_TARGET = np.array([0.24, 0.0, 0.0])

ObjectKind = Literal["box", "cylinder", "sphere"]
OBJECT_KINDS: tuple[ObjectKind, ...] = ("box", "cylinder", "sphere")


@dataclass(frozen=True)
class ObjectSpec:
    """One primitive object. Units: m, kg, rad."""

    kind: ObjectKind
    half_extents: tuple[float, float, float]  # object frame; cylinder (r, r, hz); sphere (r,r,r)
    mass: float
    friction: float
    pos: tuple[float, float, float]  # world, centre at rest on the table
    yaw: float

    @property
    def half_height(self) -> float:
        return self.half_extents[0] if self.kind == "sphere" else self.half_extents[2]


@dataclass(frozen=True)
class TaskSpec:
    obj: ObjectSpec
    place_xy: tuple[float, float]  # world target for the object centre


@dataclass(frozen=True)
class Workspace:
    """Uniform sampling bounds (m, kg). Defaults lie inside the top-down reachable region.

    The Piper reaches a straight-down TCP only up to z ~= 0.11 m at radius 0.20-0.33 m
    (joint2 >= 0 and joint5 <= 1.22 bind), so positions stay within that radius and objects
    are at most 80 mm tall to leave clearance under a 0.10 m transport height.
    """

    x: tuple[float, float] = (0.22, 0.32)
    y: tuple[float, float] = (-0.12, 0.12)
    min_place_dist: float = 0.10
    box_half_xy: tuple[float, float] = (0.014, 0.024)
    half_height: tuple[float, float] = (0.025, 0.040)
    cyl_radius: tuple[float, float] = (0.014, 0.024)
    sphere_radius: tuple[float, float] = (0.026, 0.030)
    mass: tuple[float, float] = (0.05, 0.25)
    friction: tuple[float, float] = (0.6, 1.2)

    def validate(self) -> None:
        for f in ("x", "y", "box_half_xy", "half_height", "cyl_radius", "sphere_radius",
                  "mass", "friction"):
            lo, hi = getattr(self, f)
            if lo > hi:
                raise ValueError(f"Workspace.{f}: lo {lo} > hi {hi}")
        if math.hypot(self.x[1] - self.x[0], self.y[1] - self.y[0]) < self.min_place_dist:
            raise ValueError(
                f"Workspace.min_place_dist {self.min_place_dist} exceeds the workspace diagonal"
            )


@dataclass(frozen=True)
class PadLayout:
    """Taxel grid on each finger pad, in the finger body frame (pad area as scripts/*touch.py).

    Each taxel is a small collision sphere ("bump") standing ``protrude`` proud of the finger
    face, with a touch site enclosing only that bump. Flat-on-flat contact against the
    finger's convex-hull mesh yields contact points only at the corners of the contact
    polygon, which fall outside the pad; per-taxel bumps make the reading a real pressure
    image. The finger meshes therefore do not collide with the object or table.
    """

    nx: int = 16
    ny: int = 16
    center_y: float = -0.0425
    half_x: float = 0.015
    half_y: float = 0.0225
    bump_radius: float = 0.0009
    protrude: float = 0.0006
    site_radius: float = 0.0014  # < x pitch (1.875 mm): a site sees only its own bump

    def site_positions(self) -> tuple[np.ndarray, np.ndarray]:
        xs = np.linspace(-self.half_x + self.half_x / self.nx, self.half_x - self.half_x / self.nx,
                         self.nx)
        ys = np.linspace(self.center_y - self.half_y + self.half_y / self.ny,
                         self.center_y + self.half_y - self.half_y / self.ny, self.ny)
        return xs, ys


@dataclass(frozen=True)
class Handles:
    """Resolved indices; arrays are ordered as ARM_JOINTS / FINGER_JOINTS."""

    arm_qpos: np.ndarray
    arm_dof: np.ndarray
    arm_act: np.ndarray
    finger_qpos: np.ndarray
    finger_dof: np.ndarray
    finger_act: np.ndarray
    tcp_site: int
    obj_body: int
    obj_qpos: int  # freejoint: 3 pos + 4 quat
    obj_dof: int
    obj_geom: int
    finger_bodies: tuple[int, int]  # link7 (left), link8 (right)
    table_geom: int
    scene_cam: int
    wrist_cam: int


@dataclass
class Scene:
    model: mujoco.MjModel
    data: mujoco.MjData
    task: TaskSpec
    handles: Handles
    tactile_adr: np.ndarray = field(repr=False)  # (2, ny, nx) sensordata offsets, [left, right]
    pad: PadLayout = field(default_factory=PadLayout)


# ── sampling ──────────────────────────────────────────────────────────────────


def _uniform(rng: np.random.Generator, bounds: tuple[float, float]) -> float:
    return float(rng.uniform(bounds[0], bounds[1]))


def sample_task(rng: np.random.Generator, ws: Workspace | None = None) -> TaskSpec:
    """Draw object type, size, mass, friction, pose and a place target. Consumes ``rng``."""
    ws = ws or Workspace()
    ws.validate()
    kind: ObjectKind = OBJECT_KINDS[int(rng.integers(len(OBJECT_KINDS)))]
    if kind == "box":
        half: tuple[float, float, float] = (
            _uniform(rng, ws.box_half_xy), _uniform(rng, ws.box_half_xy),
            _uniform(rng, ws.half_height))
    elif kind == "cylinder":
        r = _uniform(rng, ws.cyl_radius)
        half = (r, r, _uniform(rng, ws.half_height))
    else:
        r = _uniform(rng, ws.sphere_radius)
        half = (r, r, r)
    hz = half[0] if kind == "sphere" else half[2]
    ox, oy = _uniform(rng, ws.x), _uniform(rng, ws.y)
    obj = ObjectSpec(
        kind=kind, half_extents=half, mass=_uniform(rng, ws.mass),
        friction=_uniform(rng, ws.friction), pos=(ox, oy, hz),
        yaw=float(rng.uniform(-math.pi, math.pi)),
    )
    for _ in range(1000):
        px, py = _uniform(rng, ws.x), _uniform(rng, ws.y)
        if math.hypot(px - ox, py - oy) >= ws.min_place_dist:
            return TaskSpec(obj=obj, place_xy=(px, py))
    raise ValueError(f"min_place_dist {ws.min_place_dist} not achievable in 1000 draws")


# ── MJCF construction ─────────────────────────────────────────────────────────


def _validate(task: TaskSpec) -> None:
    o = task.obj
    if o.kind not in OBJECT_KINDS:
        raise ValueError(f"object kind {o.kind!r} not in {OBJECT_KINDS}")
    if min(o.half_extents) <= 0:
        raise ValueError(f"object half_extents must be > 0, got {o.half_extents}")
    if o.mass <= 0:
        raise ValueError(f"object mass must be > 0, got {o.mass}")
    if o.friction <= 0:
        raise ValueError(f"object friction must be > 0, got {o.friction}")


def _child(parent: ET.Element, tag: str) -> ET.Element:
    el = parent.find(tag)
    return el if el is not None else ET.SubElement(parent, tag)


def _fmt(v: np.ndarray | tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{float(x):.6g}" for x in v)


def lookat_xyaxes(pos: np.ndarray, target: np.ndarray, up: np.ndarray | None = None) -> str:
    """MJCF ``xyaxes`` for a camera at ``pos`` looking at ``target`` (cameras look along -z)."""
    up = np.array([0.0, 0.0, 1.0]) if up is None else up
    fwd = np.asarray(target, float) - np.asarray(pos, float)
    fwd /= np.linalg.norm(fwd)
    x = np.cross(fwd, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, fwd)
    return _fmt(np.concatenate([x, y]))


def _strip_touch_grid_plugin(root: ET.Element) -> None:
    ext = root.find("extension")
    if ext is not None:
        root.remove(ext)
    sensor = root.find("sensor")
    if sensor is not None:
        for p in sensor.findall("plugin"):
            sensor.remove(p)


def _inject_pads(root: ET.Element, pad: PadLayout) -> None:
    sensor = _child(root, "sensor")
    xs, ys = pad.site_positions()
    # Finger body -z is the pad normal (towards the object); the face is the z = 0 plane.
    zc = pad.bump_radius - pad.protrude
    for body in root.iter("body"):
        name = body.get("name", "")
        if name not in ("link7", "link8"):
            continue
        side = "l" if name == "link7" else "r"
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                pos = f"{x:.5f} {y:.5f} {zc:.5f}"
                ET.SubElement(body, "geom", name=f"tb_{side}_{iy}_{ix}", type="sphere",
                              size=f"{pad.bump_radius}", pos=pos, contype="2",
                              conaffinity="2", group="3", rgba="0.3 0.3 0.3 1")
                site = f"ts_{side}_{iy}_{ix}"
                ET.SubElement(body, "site", name=site, type="sphere", size=f"{pad.site_radius}",
                              pos=pos, group="4", rgba="0.2 1 0.2 0.35")
                ET.SubElement(sensor, "touch", name=f"t{side}_{iy}_{ix}", site=site)


def _configure_robot(root: ET.Element) -> None:
    """Collision groups, TCP site and wrist camera on the robot tree."""
    for body in root.iter("body"):
        if body.get("name") == "right_hand":
            ET.SubElement(body, "site", name="tcp", pos=f"0 0 {-TCP_OFFSET}", size="0.004",
                          rgba="1 0 0 1", group="3")
            cam_pos = np.array([-0.075, 0.0, -0.115])
            ET.SubElement(body, "camera", name="wrist_cam", mode="fixed", fovy="70",
                          pos=_fmt(cam_pos),
                          xyaxes=lookat_xyaxes(cam_pos, np.array([0.0, 0.0, 0.02]),
                                               up=np.array([-1.0, 0.0, 0.0])))
        for joint in body.findall("joint"):
            if joint.get("name") in FINGER_JOINTS:
                # Reflected inertia of the geared finger drive. Without it the 25 g fingers
                # chatter against the 256-bump pad contact and grasps fail on "slip".
                joint.set("armature", f"{FINGER_ARMATURE}")
        for geom in body.findall("geom"):
            if geom.get("name", "").endswith("_col"):
                # Collision copies of the visual meshes: hide them from cameras. Drawing
                # both doubles the ~180k triangles per frame (llvmpipe has no GPU).
                geom.set("group", "3")
            if geom.get("name") in ("link7_col", "link8_col"):
                # arm-only collisions; the taxel bumps (group 2) carry object/table contact
                geom.set("contype", "1")
                geom.set("conaffinity", "1")
    # `base` is welded to the world, so MuJoCo's parent filter does not apply to base/link1
    # and their meshes overlap by 6 mm: without this, joint1 is jammed and never rotates.
    ET.SubElement(_child(root, "contact"), "exclude", body1="base", body2="link1")


def _object_body(obj: ObjectSpec) -> ET.Element:
    half = obj.half_extents
    if obj.kind == "box":
        gtype, size, rgba = "box", _fmt(half), "0.85 0.35 0.2 1"
    elif obj.kind == "cylinder":
        gtype, size, rgba = "cylinder", _fmt((half[0], half[2])), "0.2 0.5 0.85 1"
    else:
        gtype, size, rgba = "sphere", _fmt((half[0],)), "0.9 0.75 0.15 1"
    q = (math.cos(obj.yaw / 2), 0.0, 0.0, math.sin(obj.yaw / 2))
    body = ET.Element("body", name="object", pos=_fmt(obj.pos), quat=_fmt(q))
    ET.SubElement(body, "freejoint", name="object_free")
    # priority=1 makes the object's friction/solref the ones used in every contact it is part
    # of, so the randomized coefficient is the effective one (MuJoCo otherwise takes the max).
    # solimp is stiffer than default so a 20 N squeeze does not sink the pad into the object;
    # solref 10 ms (not 2 ms) because ~256 parallel bump contacts at 2 ms chatter.
    ET.SubElement(body, "geom", name="object_geom", type=gtype, size=size, rgba=rgba,
                  mass=f"{obj.mass:.6g}", friction=f"{obj.friction:.6g} 0.01 0.0002",
                  solref="0.01 1", solimp="0.95 0.99 0.0005",
                  priority="1", condim="4", contype="2", conaffinity="2")
    return body


def _add_world(root: ET.Element, task: TaskSpec) -> None:
    wb = _child(root, "worldbody")
    ET.SubElement(wb, "light", pos="0.3 -0.3 1.5", dir="0 0.2 -1", diffuse="0.8 0.8 0.8",
                  castshadow="false")
    ET.SubElement(wb, "geom", name="floor", type="plane", size="2 2 0.1", pos="0 0 -0.75",
                  rgba="0.22 0.22 0.25 1", contype="0", conaffinity="0")
    ET.SubElement(wb, "geom", name="table", type="box", size="0.35 0.45 0.02",
                  pos="0.30 0 -0.02", rgba="0.55 0.42 0.3 1", contype="2", conaffinity="2")
    px, py = task.place_xy
    ET.SubElement(wb, "site", name="place_target", type="cylinder", size="0.03 0.0005",
                  pos=f"{px:.5f} {py:.5f} 0.0005", rgba="0.1 0.9 0.2 0.5", group="0")
    ET.SubElement(wb, "camera", name="scene_cam", mode="fixed", fovy="50",
                  pos=_fmt(SCENE_CAM_POS),
                  xyaxes=lookat_xyaxes(SCENE_CAM_POS, SCENE_CAM_TARGET))
    wb.append(_object_body(task.obj))


def build_xml(task: TaskSpec, *, offscreen: tuple[int, int] = (640, 480),
              pad: PadLayout | None = None) -> str:
    """The scene MJCF as a string; never written to disk next to the robot assets."""
    _validate(task)
    pad = pad or PadLayout()
    if not ROBOT_XML.exists():
        raise FileNotFoundError(f"{ROBOT_XML} missing; run `git submodule update --init`")
    root = ET.parse(ROBOT_XML).getroot()
    _child(root, "compiler").set("meshdir", str(ROBOT_XML.parent))
    opt = _child(root, "option")
    opt.attrib.update(timestep="0.001", integrator="implicitfast", iterations="20",
                      cone="elliptic", impratio="10")
    # robot.xml caps nconmax at 500; a box face pressed on both pads makes ~512 bump contacts,
    # and overflowing the cap drops contacts at random, which reads as flickering taxels.
    size = _child(root, "size")
    for k in ("nconmax", "njmax"):
        size.attrib.pop(k, None)
    size.set("memory", "256M")
    visual = _child(root, "visual")
    _child(visual, "global").attrib.update(offwidth=str(offscreen[0]),
                                           offheight=str(offscreen[1]))
    # Shadows cost ~4x per frame under software EGL (llvmpipe) and add nothing to the data.
    _child(visual, "quality").set("shadowsize", "0")
    _strip_touch_grid_plugin(root)
    _configure_robot(root)
    _inject_pads(root, pad)
    _add_world(root, task)
    return ET.tostring(root, encoding="unicode")


def _resolve(model: mujoco.MjModel) -> Handles:
    return Handles(
        arm_qpos=np.array([mjutil.joint_qpos_adr(model, j) for j in ARM_JOINTS]),
        arm_dof=np.array([mjutil.joint_dof_adr(model, j) for j in ARM_JOINTS]),
        arm_act=np.array([mjutil.actuator_id(model, j) for j in ARM_JOINTS]),
        finger_qpos=np.array([mjutil.joint_qpos_adr(model, j) for j in FINGER_JOINTS]),
        finger_dof=np.array([mjutil.joint_dof_adr(model, j) for j in FINGER_JOINTS]),
        finger_act=np.array([mjutil.actuator_id(model, j) for j in FINGER_JOINTS]),
        tcp_site=mjutil.site_id(model, "tcp"),
        obj_body=mjutil.body_id(model, "object"),
        obj_qpos=mjutil.joint_qpos_adr(model, "object_free"),
        obj_dof=mjutil.joint_dof_adr(model, "object_free"),
        obj_geom=mjutil.geom_id(model, "object_geom"),
        finger_bodies=(mjutil.body_id(model, "link7"), mjutil.body_id(model, "link8")),
        table_geom=mjutil.geom_id(model, "table"),
        scene_cam=mjutil.camera_id(model, "scene_cam"),
        wrist_cam=mjutil.camera_id(model, "wrist_cam"),
    )


def _tactile_addresses(model: mujoco.MjModel, pad: PadLayout) -> np.ndarray:
    adr = np.empty((2, pad.ny, pad.nx), dtype=np.int64)
    for k, side in enumerate("lr"):
        for iy in range(pad.ny):
            for ix in range(pad.nx):
                adr[k, iy, ix] = mjutil.sensor_adr(model, f"t{side}_{iy}_{ix}")
    return adr


def reset_to_home(scene: Scene) -> None:
    """Arm at HOME_Q, gripper fully open, object at its spec pose, controls matching."""
    m, d, h = scene.model, scene.data, scene.handles
    mujoco.mj_resetData(m, d)
    d.qpos[h.arm_qpos] = HOME_Q
    d.qpos[h.finger_qpos] = FINGER_SIGN * GRIPPER_MAX_OPENING
    d.ctrl[h.arm_act] = HOME_Q
    d.ctrl[h.finger_act] = FINGER_SIGN * GRIPPER_MAX_OPENING
    mujoco.mj_forward(m, d)


def build_scene(task: TaskSpec, *, offscreen: tuple[int, int] = (640, 480),
                pad: PadLayout | None = None) -> Scene:
    """Compile ``task`` and return a scene reset to the home posture with gripper open."""
    pad = pad or PadLayout()
    model = mujoco.MjModel.from_xml_string(build_xml(task, offscreen=offscreen, pad=pad))
    scene = Scene(model=model, data=mujoco.MjData(model), task=task, handles=_resolve(model),
                  tactile_adr=_tactile_addresses(model, pad), pad=pad)
    reset_to_home(scene)
    log.debug("scene.built kind=%s half=%s mass=%.3f mu=%.2f place=%s", task.obj.kind,
              task.obj.half_extents, task.obj.mass, task.obj.friction, task.place_xy)
    return scene
