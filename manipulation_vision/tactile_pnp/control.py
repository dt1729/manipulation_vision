"""Cartesian TCP goals -> time-parametrised joint reference trajectories for the Piper arm.

Contract: docs/contracts/manipulation_vision.tactile_pnp.control.md

Damped-least-squares IK on MuJoCo site Jacobians, plus time-synchronised trapezoidal
profiles (joint space) and straight-line Cartesian segments (trapezoid on arc length).
Nothing here writes ``data.ctrl`` or steps physics; the episode runner does.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from manipulation_vision.tactile_pnp.scene import HOME_Q, Handles

LINEAR_KNOT = 0.005  # m between IK knots on a Cartesian segment
MAX_IK_STEP = 0.2  # rad, per-iteration joint step cap


class IkError(RuntimeError):
    """A Cartesian segment contains a pose the arm cannot reach."""

    def __init__(self, pos: np.ndarray, pos_err: float, rot_err: float) -> None:
        super().__init__(f"IK failed at {np.round(pos, 4).tolist()}: "
                         f"pos_err={pos_err:.4f} m rot_err={rot_err:.4f} rad")
        self.pos = pos
        self.pos_err = pos_err
        self.rot_err = rot_err


def _positive(**kw: float) -> None:
    for k, v in kw.items():
        if not v > 0:
            raise ValueError(f"{k} must be > 0, got {v}")


def trapezoid_s(distance: float, vmax: float, amax: float, dt: float) -> np.ndarray:
    """Normalised path parameter in [0, 1] sampled every ``dt``; ends exactly at 1."""
    _positive(vmax=vmax, amax=amax, dt=dt)
    if distance < 0:
        raise ValueError(f"distance must be >= 0, got {distance}")
    if distance == 0:
        return np.ones(1)
    t_acc = vmax / amax
    if distance < amax * t_acc**2:  # cannot reach vmax: triangular
        t_acc = math.sqrt(distance / amax)
    v_peak = amax * t_acc
    d_acc = 0.5 * amax * t_acc**2
    t_cruise = (distance - 2 * d_acc) / v_peak
    total = 2 * t_acc + t_cruise
    t = np.arange(math.ceil(total / dt) + 1) * dt
    td = np.clip(t - t_acc - t_cruise, 0.0, t_acc)
    x = np.where(
        t <= t_acc,
        0.5 * amax * t**2,
        d_acc + v_peak * np.minimum(t - t_acc, t_cruise) + v_peak * td - 0.5 * amax * td**2,
    )
    s = np.clip(x / distance, 0.0, 1.0)
    s[-1] = 1.0
    return s


def trapezoid_profile(q0: np.ndarray, qf: np.ndarray, vmax: float, amax: float,
                      dt: float) -> np.ndarray:
    """Time-synchronised joint profile ``(N, n)``; all joints start and stop together."""
    q0, qf = np.asarray(q0, float), np.asarray(qf, float)
    if q0.shape != qf.shape:
        raise ValueError(f"q0 shape {q0.shape} != qf shape {qf.shape}")
    s = trapezoid_s(float(np.abs(qf - q0).max(initial=0.0)), vmax, amax, dt)
    out = q0 + s[:, None] * (qf - q0)
    out[-1] = qf
    return np.asarray(out, dtype=float)


def top_down_rotation(yaw: float) -> np.ndarray:
    """TCP rotation: approach (hand z) straight down, closing axis (hand y) at ``yaw``."""
    y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([np.cross(y, z), y, z])


def rotation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """World-frame axis*angle taking ``current`` to ``target``."""
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, (target @ current.T).ravel())
    vel = np.empty(3)
    mujoco.mju_quat2Vel(vel, quat, 1.0)
    return vel


@dataclass(frozen=True)
class IkResult:
    q: np.ndarray
    pos_err: float
    rot_err: float
    converged: bool


class Kinematics:
    """FK / IK for the TCP site on a private ``MjData`` (the live simulation is untouched)."""

    def __init__(self, model: mujoco.MjModel, handles: Handles) -> None:
        self._m = model
        self._d = mujoco.MjData(model)
        self._h = handles
        jnt = model.dof_jntid[handles.arm_dof]
        ctrl = model.actuator_ctrlrange[handles.arm_act]
        self.lo = np.maximum(model.jnt_range[jnt, 0], ctrl[:, 0])
        self.hi = np.minimum(model.jnt_range[jnt, 1], ctrl[:, 1])
        self._jacp = np.zeros((3, model.nv))
        self._jacr = np.zeros((3, model.nv))

    def _set(self, q: np.ndarray) -> None:
        self._d.qpos[self._h.arm_qpos] = q
        mujoco.mj_kinematics(self._m, self._d)
        mujoco.mj_comPos(self._m, self._d)

    def fk(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """TCP position (3,) m and rotation (3, 3) for arm joints ``q``."""
        self._set(q)
        s = self._h.tcp_site
        return self._d.site_xpos[s].copy(), self._d.site_xmat[s].reshape(3, 3).copy()

    def _attempt(self, pos: np.ndarray, rot: np.ndarray, q: np.ndarray, pos_tol: float,
                 rot_tol: float, max_iter: int) -> IkResult:
        s, cols = self._h.tcp_site, self._h.arm_dof
        q = np.clip(q.astype(float), self.lo, self.hi)
        damping = 1e-2
        for _ in range(max_iter):
            self._set(q)
            e_pos = pos - self._d.site_xpos[s]
            e_rot = rotation_error(rot, self._d.site_xmat[s].reshape(3, 3))
            if np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_rot) < rot_tol:
                return IkResult(q, float(np.linalg.norm(e_pos)), float(np.linalg.norm(e_rot)),
                                converged=True)
            mujoco.mj_jacSite(self._m, self._d, self._jacp, self._jacr, s)
            jac = np.vstack([self._jacp[:, cols], self._jacr[:, cols]])
            err = np.concatenate([e_pos, e_rot])
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping**2 * np.eye(6), err)
            norm = np.linalg.norm(dq)
            if norm > MAX_IK_STEP:
                dq *= MAX_IK_STEP / norm
            q = np.clip(q + dq, self.lo, self.hi)
        p, r = self.fk(q)
        return IkResult(q, float(np.linalg.norm(pos - p)),
                        float(np.linalg.norm(rotation_error(rot, r))), converged=False)

    def solve(self, pos: np.ndarray, rot: np.ndarray, q_seed: np.ndarray, *,
              pos_tol: float = 1e-3, rot_tol: float = 1e-2, max_iter: int = 300) -> IkResult:
        """DLS IK from ``q_seed``, retried from HOME_Q; best attempt if neither converges."""
        best: IkResult | None = None
        for seed in (q_seed, HOME_Q):
            res = self._attempt(np.asarray(pos, float), rot, np.asarray(seed, float),
                                pos_tol, rot_tol, max_iter)
            if res.converged:
                return res
            if best is None or res.pos_err + 0.1 * res.rot_err < best.pos_err + 0.1 * best.rot_err:
                best = res
        assert best is not None
        return best

    def plan_linear(self, q_start: np.ndarray, pos_goal: np.ndarray, rot_goal: np.ndarray,
                    vmax: float, amax: float, dt: float) -> np.ndarray:
        """Straight TCP segment to ``pos_goal`` (trapezoid on arc length), ``(N, 6)``."""
        start = self.fk(q_start)[0]
        goal = np.asarray(pos_goal, float)
        length = float(np.linalg.norm(goal - start))
        n = max(1, math.ceil(length / LINEAR_KNOT))
        knots = [np.asarray(q_start, float)]
        for k in range(1, n + 1):
            p = start + (goal - start) * k / n
            res = self.solve(p, rot_goal, knots[-1])
            if not res.converged:
                raise IkError(p, res.pos_err, res.rot_err)
            knots.append(res.q)
        qk = np.array(knots)
        u = trapezoid_s(length, vmax, amax, dt) * n
        i = np.minimum(u.astype(int), n - 1)
        frac = (u - i)[:, None]
        return np.asarray(qk[i] * (1 - frac) + qk[i + 1] * frac, dtype=float)
