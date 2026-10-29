"""Scripted, tactile-gated pick-and-place episode on a built scene.

Contract: docs/contracts/manipulation_vision.tactile_pnp.pnp.md

approach -> pregrasp -> descend -> grasp (each finger closes until its own pad feels contact;
grasp confirmed only when both do) -> lift -> transport -> place -> release -> retreat -> settle.
Observers are called once per control step; the logger and preview are observers.
"""

from __future__ import annotations

import contextlib
import logging
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, fields
from enum import IntEnum

import mujoco
import numpy as np

from manipulation_vision.tactile_pnp import tactile
from manipulation_vision.tactile_pnp.control import (
    IkError,
    Kinematics,
    top_down_rotation,
    trapezoid_profile,
)
from manipulation_vision.tactile_pnp.scene import (
    FINGER_SIGN,
    GRIPPER_MAX_OPENING,
    TCP_OFFSET,
    ObjectSpec,
    Scene,
)

log = logging.getLogger(__name__)

PAD_CENTER_FROM_TIP = 0.0425  # m, pad centre behind the fingertip along the approach axis
MIN_OPENING = 0.001  # m per finger: below this the finger has closed on nothing


class Phase(IntEnum):
    APPROACH = 0
    PREGRASP = 1
    DESCEND = 2
    GRASP = 3
    LIFT = 4
    TRANSPORT = 5
    PLACE = 6
    RELEASE = 7
    RETREAT = 8
    SETTLE = 9


@dataclass(frozen=True)
class PnpConfig:
    """Episode parameters. Units: s, m, rad, N."""

    control_hz: float = 100.0
    joint_vmax: float = 1.5  # rad/s, limiting joint
    joint_amax: float = 3.0  # rad/s^2
    lin_vmax: float = 0.15  # m/s TCP
    lin_amax: float = 0.6  # m/s^2 TCP
    descend_vmax: float = 0.05  # m/s TCP for the last approach and the place descent
    z_travel: float = 0.10  # TCP height for approach and retreat
    tip_clearance: float = 0.006  # fingertip above the table at grasp height
    pregrasp_clearance: float = 0.02  # fingertip above the object top before descending
    close_speed: float = 0.04  # m/s per finger
    squeeze: float = 0.002  # m commanded past first contact (~20 N at kp 1e4)
    contact_min_cells: int = 1
    contact_threshold: float = tactile.ACTIVE_THRESHOLD  # N per taxel
    grasp_hold: float = 0.15  # s to let the squeeze settle before lifting
    slip_debounce: float = 0.05  # s a pad must read no contact before it counts as a slip
    lift_height: float = 0.05
    place_drop: float = 0.003  # object bottom above the table at release
    place_tol: float = 0.02  # m, xy success tolerance
    settle_time: float = 0.4
    max_sim_time: float = 30.0

    def validate(self, timestep: float) -> int:
        """Check fields; return physics substeps per control step."""
        for f in fields(self):
            v = getattr(self, f.name)
            if not v > 0:
                raise ValueError(f"PnpConfig.{f.name} must be > 0, got {v}")
        ratio = 1.0 / (self.control_hz * timestep)
        if abs(ratio - round(ratio)) > 1e-6 or round(ratio) < 1:
            raise ValueError(
                f"PnpConfig.control_hz {self.control_hz} must divide the physics rate "
                f"{1 / timestep:.0f} Hz into an integer number of substeps"
            )
        return round(ratio)


@dataclass(frozen=True)
class StepInfo:
    t: float
    phase: Phase
    action: np.ndarray  # (7,) arm joint command (rad) + commanded gripper gap (m)
    tactile: np.ndarray  # (2, ny, nx) N


@dataclass(frozen=True)
class EpisodeResult:
    success: bool
    failure: str | None
    place_error: float
    sim_time: float
    peak_active: dict[str, list[int]] = field(default_factory=dict)
    grasp_force: list[float] = field(default_factory=lambda: [0.0, 0.0])


Observer = Callable[[Scene, StepInfo], None]


class _Abort(Exception):  # noqa: N818 - internal control flow, not an error type
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def _grasp_yaws(obj: ObjectSpec) -> list[float]:
    """Candidate closing-axis yaws: across the narrow side for a box, any for round objects."""
    if obj.kind == "box":
        base = obj.yaw if obj.half_extents[0] <= obj.half_extents[1] else obj.yaw + math.pi / 2
        return [base, base + math.pi]
    return [k * math.pi / 4 for k in range(-4, 4)]


class _Episode:
    def __init__(self, scene: Scene, cfg: PnpConfig, observers: Sequence[Observer]) -> None:
        self.s, self.cfg, self.observers = scene, cfg, observers
        self.m, self.d, self.h = scene.model, scene.data, scene.handles
        self.substeps = cfg.validate(self.m.opt.timestep)
        self.dt = self.substeps * self.m.opt.timestep
        self.kin = Kinematics(self.m, self.h)
        self.phase = Phase.APPROACH
        self.q_cmd = self.d.qpos[self.h.arm_qpos].copy()
        self.opening = np.abs(self.d.qpos[self.h.finger_qpos]).copy()  # per finger, m
        self.grids = tactile.read_grids(self.d, scene.tactile_adr)
        self.peaks: dict[str, np.ndarray] = {}
        self.grasp_force = np.zeros(2)
        self.require_contact = False
        self.no_contact_time = 0.0
        self.rot: np.ndarray | None = None  # grasp orientation once chosen

    # ── stepping ─────────────────────────────────────────────────────────────
    def tick(self) -> None:
        self.d.ctrl[self.h.arm_act] = self.q_cmd
        self.d.ctrl[self.h.finger_act] = FINGER_SIGN * self.opening
        for _ in range(self.substeps):
            mujoco.mj_step(self.m, self.d)
        self.grids = tactile.read_grids(self.d, self.s.tactile_adr)
        name = self.phase.name
        active = tactile.active_cells(self.grids, self.cfg.contact_threshold)
        self.peaks[name] = np.maximum(self.peaks.get(name, active), active)
        info = StepInfo(t=float(self.d.time), phase=self.phase,
                        action=np.append(self.q_cmd, self.opening.sum()), tactile=self.grids)
        for obs in self.observers:
            obs(self.s, info)
        if self.require_contact and not self.both_in_contact():
            self.no_contact_time += self.dt
            if self.no_contact_time >= self.cfg.slip_debounce - 1e-9:
                raise _Abort("slip")
        else:
            self.no_contact_time = 0.0
        if self.d.time > self.cfg.max_sim_time:
            raise _Abort("timeout")

    def both_in_contact(self) -> bool:
        return bool(tactile.pad_contact(self.grids, min_cells=self.cfg.contact_min_cells,
                                        threshold=self.cfg.contact_threshold).all())

    def follow(self, traj: np.ndarray) -> None:
        for q in traj:
            self.q_cmd = q
            self.tick()

    def hold(self, seconds: float) -> None:
        for _ in range(max(1, round(seconds / self.dt))):
            self.tick()

    # ── motions ──────────────────────────────────────────────────────────────
    def tcp(self) -> np.ndarray:
        return np.array(self.d.site_xpos[self.h.tcp_site], dtype=float)

    def move_joint(self, pos: np.ndarray, rot: np.ndarray) -> None:
        res = self.kin.solve(pos, rot, self.q_cmd)
        if not res.converged:
            raise _Abort("ik")
        c = self.cfg
        self.follow(trapezoid_profile(self.q_cmd, res.q, c.joint_vmax, c.joint_amax, self.dt)[1:])

    def move_linear(self, pos: np.ndarray, rot: np.ndarray, vmax: float) -> None:
        try:
            traj = self.kin.plan_linear(self.q_cmd, pos, rot, vmax, self.cfg.lin_amax, self.dt)
        except IkError as e:
            raise _Abort("ik") from e
        self.follow(traj[1:])

    def set_opening(self, target: float, speed: float) -> None:
        """Ramp both fingers to ``target`` opening (m) at ``speed`` m/s."""
        step = speed * self.dt
        while np.abs(self.opening - target).max() > 1e-9:
            self.opening = self.opening + np.clip(target - self.opening, -step, step)
            self.tick()

    def close_until_contact(self) -> None:
        """Each finger closes until its own pad reports contact and stops there; once both
        pads touch, both squeeze together. (Squeezing on first contact shoves and tips the
        object before the other finger arrives.)"""
        c = self.cfg
        step = c.close_speed * self.dt
        done = np.zeros(2, dtype=bool)
        while not done.all():
            contact = tactile.pad_contact(self.grids, min_cells=c.contact_min_cells,
                                          threshold=c.contact_threshold)
            for k in range(2):
                if done[k]:
                    continue
                if contact[k]:
                    self.opening[k] = abs(float(self.d.qpos[self.h.finger_qpos[k]]))
                    done[k] = True
                elif self.opening[k] <= MIN_OPENING:
                    raise _Abort("missed_grasp")
                else:
                    self.opening[k] = max(0.0, self.opening[k] - step)
            self.tick()
        actual = np.abs(self.d.qpos[self.h.finger_qpos])
        self.opening = np.maximum(0.0, actual - c.squeeze)
        self.hold(c.grasp_hold)
        if not self.both_in_contact():
            raise _Abort("missed_grasp")
        self.grasp_force = tactile.finger_contact_forces(self.m, self.d, self.h.finger_bodies,
                                                         self.h.obj_geom)

    # ── episode ──────────────────────────────────────────────────────────────
    def choose_rotation(self, above: np.ndarray, above_place: np.ndarray) -> np.ndarray:
        """Grasp yaw reachable at both pick and place (the wrist keeps it during transport);
        among those, the one needing the least joint motion from the current posture."""
        best: tuple[float, np.ndarray] | None = None
        for yaw in _grasp_yaws(self.s.task.obj):
            rot = top_down_rotation(yaw)
            res = self.kin.solve(above, rot, self.q_cmd)
            if not res.converged:
                continue
            if not self.kin.solve(above_place, rot, res.q).converged:
                continue
            cost = float(np.abs(res.q - self.q_cmd).max())
            if best is None or cost < best[0]:
                best = (cost, rot)
        if best is None:
            raise _Abort("ik")
        return best[1]

    def obj_pos(self) -> np.ndarray:
        return np.array(self.d.xpos[self.h.obj_body], dtype=float)

    def pick_and_place(self) -> None:
        c, obj = self.cfg, self.s.task.obj
        ox, oy, oz = obj.pos
        tip_grasp = max(c.tip_clearance, oz - PAD_CENTER_FROM_TIP)
        z_grasp = tip_grasp + TCP_OFFSET
        z_pre = min(c.z_travel, oz + obj.half_height + c.pregrasp_clearance + TCP_OFFSET)

        self.phase = Phase.APPROACH
        px, py = self.s.task.place_xy
        rot = self.rot = self.choose_rotation(np.array([ox, oy, c.z_travel]),
                                              np.array([px, py, z_grasp + c.lift_height]))
        self.move_joint(np.array([ox, oy, c.z_travel]), rot)
        self.phase = Phase.PREGRASP
        self.move_linear(np.array([ox, oy, z_pre]), rot, c.lin_vmax)
        self.phase = Phase.DESCEND
        self.move_linear(np.array([ox, oy, z_grasp]), rot, c.descend_vmax)
        self.phase = Phase.GRASP
        self.close_until_contact()

        self.phase = Phase.LIFT
        self.require_contact = True
        z0 = self.obj_pos()[2]
        self.move_linear(self.tcp() + np.array([0.0, 0.0, c.lift_height]), rot, c.lin_vmax)
        self.hold(0.1)
        if self.obj_pos()[2] - z0 < 0.5 * c.lift_height:
            raise _Abort("slip")
        self.phase = Phase.TRANSPORT
        offset = self.obj_pos()[:2] - self.tcp()[:2]  # keep the object, not the TCP, on target
        self.move_linear(np.array([px - offset[0], py - offset[1], self.tcp()[2]]), rot,
                         c.lin_vmax)
        self.phase = Phase.PLACE
        bottom = self.obj_pos()[2] - obj.half_height
        down = np.array([0.0, 0.0, bottom - c.place_drop])
        self.move_linear(self.tcp() - down, rot, c.descend_vmax)
        self.require_contact = False
        self.phase = Phase.RELEASE
        self.set_opening(GRIPPER_MAX_OPENING, 2 * c.close_speed)
        self.retreat(rot)

    def retreat(self, rot: np.ndarray | None) -> None:
        self.require_contact = False
        self.phase = Phase.RETREAT
        self.set_opening(GRIPPER_MAX_OPENING, 2 * self.cfg.close_speed)
        if rot is not None:
            up = self.tcp()
            up[2] = max(up[2], self.cfg.z_travel)
            # retreat is best effort; the outcome is already decided
            with contextlib.suppress(_Abort):
                self.move_linear(up, rot, self.cfg.lin_vmax)
        self.phase = Phase.SETTLE
        self.hold(self.cfg.settle_time)

    def judge(self, failure: str | None) -> EpisodeResult:
        obj, pos = self.s.task.obj, self.obj_pos()
        err = float(np.hypot(*(pos[:2] - np.asarray(self.s.task.place_xy))))
        if failure is None:
            touching = tactile.finger_contact_forces(self.m, self.d, self.h.finger_bodies,
                                                     self.h.obj_geom).any()
            resting = abs(pos[2] - obj.half_height) < 0.01
            if err > self.cfg.place_tol or not resting or touching:
                failure = "misplaced"
        return EpisodeResult(
            success=failure is None, failure=failure, place_error=err,
            sim_time=float(self.d.time),
            peak_active={k: v.astype(int).tolist() for k, v in self.peaks.items()},
            grasp_force=self.grasp_force.round(3).tolist(),
        )


def run_episode(scene: Scene, cfg: PnpConfig | None = None,
                observers: Sequence[Observer] = ()) -> EpisodeResult:
    """Run one episode on ``scene`` (mutating ``scene.data``) and judge it."""
    cfg = cfg or PnpConfig()
    ep = _Episode(scene, cfg, observers)
    failure: str | None = None
    try:
        ep.pick_and_place()
    except _Abort as a:
        failure = a.reason
        log.debug("episode.abort reason=%s phase=%s t=%.2f", a.reason, ep.phase.name,
                  scene.data.time)
        with contextlib.suppress(_Abort):
            ep.retreat(None if a.reason in ("ik", "timeout") else ep.rot)
    result = ep.judge(failure)
    log.info("episode.done success=%s failure=%s place_err=%.4f sim_t=%.2f", result.success,
             result.failure, result.place_error, result.sim_time)
    return result
