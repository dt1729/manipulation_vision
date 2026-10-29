import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import control
from manipulation_vision.tactile_pnp import scene as sc

DT = 0.01


@pytest.fixture(scope="module")
def built() -> sc.Scene:
    return sc.build_scene(sc.sample_task(np.random.default_rng(0)))


@pytest.fixture(scope="module")
def kin(built) -> control.Kinematics:
    return control.Kinematics(built.model, built.handles)


def test_profile_starts_at_q0_and_ends_exactly_at_qf():
    q0, qf = np.array([0.0, 1.0, -1.0]), np.array([0.5, 0.2, -1.3])
    p = control.trapezoid_profile(q0, qf, vmax=1.0, amax=2.0, dt=DT)
    np.testing.assert_allclose(p[0], q0)
    assert (p[-1] == qf).all()


def test_profile_joints_arrive_on_the_same_row():
    q0, qf = np.zeros(3), np.array([1.0, 0.1, -0.5])
    p = control.trapezoid_profile(q0, qf, vmax=1.0, amax=2.0, dt=DT)
    done = np.isclose(p, qf, atol=1e-9)
    first_done_row = [int(np.argmax(done[:, j])) for j in range(3)]
    assert len(set(first_done_row)) == 1, first_done_row


def test_profile_limiting_joint_respects_vmax_and_amax():
    q0, qf = np.zeros(2), np.array([2.0, -0.3])
    vmax, amax = 0.8, 1.5
    p = control.trapezoid_profile(q0, qf, vmax=vmax, amax=amax, dt=DT)
    v = np.diff(p[:, 0]) / DT
    a = np.diff(v) / DT
    assert np.abs(v).max() <= vmax * 1.001
    assert np.abs(a).max() <= amax * 1.05
    assert np.abs(v).max() >= vmax * 0.99  # long move actually cruises


def test_short_move_is_triangular_and_within_amax():
    s = control.trapezoid_s(0.01, vmax=1.0, amax=2.0, dt=0.001)
    v = np.diff(s) * 0.01 / 0.001
    assert v.max() < 1.0  # never reaches cruise
    assert np.abs(np.diff(v) / 0.001).max() <= 2.0 * 1.05


def test_zero_distance_is_a_single_sample_at_the_goal():
    assert control.trapezoid_s(0.0, vmax=1.0, amax=1.0, dt=DT).tolist() == [1.0]


def test_negative_distance_is_rejected():
    with pytest.raises(ValueError, match="distance"):
        control.trapezoid_s(-1e-9, vmax=1.0, amax=1.0, dt=DT)


def test_zero_vmax_is_rejected():
    with pytest.raises(ValueError, match="vmax"):
        control.trapezoid_s(1.0, vmax=0.0, amax=1.0, dt=DT)


def test_mismatched_shapes_are_rejected():
    with pytest.raises(ValueError, match="shape"):
        control.trapezoid_profile(np.zeros(3), np.zeros(4), 1.0, 1.0, DT)


def test_top_down_rotation_points_approach_axis_down():
    r = control.top_down_rotation(0.7)
    np.testing.assert_allclose(r[:, 2], [0, 0, -1], atol=1e-12)
    np.testing.assert_allclose(r[:, 1], [np.cos(0.7), np.sin(0.7), 0], atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_ik_reaches_reachable_top_down_target(kin):
    pos, rot = np.array([0.30, 0.08, 0.06]), control.top_down_rotation(0.4)
    res = kin.solve(pos, rot, sc.HOME_Q)
    p, r = kin.fk(res.q)
    assert res.converged
    assert np.linalg.norm(p - pos) < 1e-3
    assert np.arccos(np.clip((np.trace(rot.T @ r) - 1) / 2, -1, 1)) < 1e-2


def test_ik_on_unreachable_target_reports_not_converged(kin):
    res = kin.solve(np.array([2.0, 0.0, 0.5]), control.top_down_rotation(0.0), sc.HOME_Q)
    assert not res.converged
    assert res.pos_err > 0.5


def test_executing_joint_profile_brings_tcp_to_target(built, kin):
    m, d, h = built.model, built.data, built.handles
    pos, rot = np.array([0.28, -0.10, 0.10]), control.top_down_rotation(-0.3)
    res = kin.solve(pos, rot, d.qpos[h.arm_qpos].copy())
    profile = control.trapezoid_profile(d.qpos[h.arm_qpos].copy(), res.q, 1.0, 2.0, DT)
    steps = round(DT / m.opt.timestep)
    for q in list(profile) + [res.q] * 50:  # profile, then 0.5 s settle
        d.ctrl[h.arm_act] = q
        for _ in range(steps):
            mujoco.mj_step(m, d)
    assert np.linalg.norm(d.site_xpos[h.tcp_site] - pos) < 0.005


def test_plan_linear_to_unreachable_goal_raises_ik_error_with_the_pose(kin):
    rot = control.top_down_rotation(0.0)
    start = kin.solve(np.array([0.30, 0.0, 0.08]), rot, sc.HOME_Q).q
    with pytest.raises(control.IkError) as caught:
        kin.plan_linear(start, np.array([0.30, 0.0, 0.30]), rot, vmax=0.1, amax=0.5, dt=DT)
    assert caught.value.pos[2] > 0.1  # fails above the reachable band, not at the start
    assert caught.value.pos_err > 1e-3


def test_rotation_error_is_axis_times_angle():
    np.testing.assert_allclose(control.rotation_error(np.eye(3), np.eye(3)), 0.0, atol=1e-12)
    rz = control.top_down_rotation(0.5)
    err = control.rotation_error(control.top_down_rotation(0.8), rz)
    np.testing.assert_allclose(err, [0.0, 0.0, 0.3], atol=1e-9)


def test_converged_ik_result_respects_joint_limits(kin):
    res: control.IkResult = kin.solve(np.array([0.25, -0.1, 0.05]),
                                      control.top_down_rotation(1.0), sc.HOME_Q)
    assert res.converged
    assert (res.q >= kin.lo).all() and (res.q <= kin.hi).all()


def test_plan_linear_stays_on_the_segment(kin):
    rot = control.top_down_rotation(0.2)
    start = kin.solve(np.array([0.30, 0.0, 0.10]), rot, sc.HOME_Q).q
    goal = np.array([0.30, 0.0, 0.03])
    traj = kin.plan_linear(start, goal, rot, vmax=0.1, amax=0.5, dt=DT)
    a, b = kin.fk(start)[0], goal
    u = (b - a) / np.linalg.norm(b - a)
    worst = 0.0
    for q in traj:
        p = kin.fk(q)[0] - a
        worst = max(worst, float(np.linalg.norm(p - u * (p @ u))))
    assert worst < 0.002
    assert np.linalg.norm(kin.fk(traj[-1])[0] - goal) < 1e-3
