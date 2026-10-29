import mujoco
import numpy as np
import pytest

from manipulation_vision.tactile_pnp import pnp
from manipulation_vision.tactile_pnp import scene as sc

SEED = 7


class Collector:
    def __init__(self) -> None:
        self.steps: list[pnp.StepInfo] = []

    def __call__(self, scene: sc.Scene, info: pnp.StepInfo) -> None:
        self.steps.append(info)


@pytest.fixture(scope="module")
def episode() -> tuple[pnp.EpisodeResult, Collector]:
    s = sc.build_scene(sc.sample_task(np.random.default_rng(SEED)))
    col = Collector()
    return pnp.run_episode(s, observers=[col]), col


def test_seeded_episode_succeeds_within_place_tolerance(episode):
    result, _ = episode
    assert result.success, result
    assert result.place_error < pnp.PnpConfig().place_tol


def test_observers_see_phases_in_order(episode):
    _, col = episode
    phases = [int(s.phase) for s in col.steps]
    assert phases == sorted(phases)
    assert set(phases) == {int(p) for p in pnp.Phase}


def test_observer_time_advances_by_one_control_period(episode):
    _, col = episode
    dt = np.diff([s.t for s in col.steps])
    np.testing.assert_allclose(dt, 1.0 / pnp.PnpConfig().control_hz, rtol=1e-6)


def test_grasp_phase_activates_both_pads(episode):
    result, _ = episode
    left, right = result.peak_active["GRASP"]
    assert left > 0 and right > 0


def test_grasp_force_measured_on_both_fingers(episode):
    result, _ = episode
    assert min(result.grasp_force) > 1.0


def test_episode_ends_with_gripper_open(episode):
    _, col = episode
    assert col.steps[-1].action[6] == pytest.approx(2 * sc.GRIPPER_MAX_OPENING)


def test_displaced_object_is_a_missed_grasp():
    s = sc.build_scene(sc.sample_task(np.random.default_rng(SEED)))
    h = s.handles
    s.data.qpos[h.obj_qpos + 1] += 0.08  # perception says it is 8 cm away from where it is
    mujoco.mj_forward(s.model, s.data)
    result = pnp.run_episode(s)
    assert not result.success
    assert result.failure == "missed_grasp"


def test_config_validate_returns_substeps_per_control_step():
    assert pnp.PnpConfig(control_hz=100.0).validate(0.001) == 10
    assert pnp.PnpConfig(control_hz=1000.0).validate(0.001) == 1


def test_non_positive_config_field_is_rejected():
    with pytest.raises(ValueError, match="squeeze"):
        pnp.PnpConfig(squeeze=0.0).validate(0.001)


def test_control_rate_not_dividing_physics_rate_is_rejected():
    s = sc.build_scene(sc.sample_task(np.random.default_rng(SEED)))
    with pytest.raises(ValueError, match="control_hz"):
        pnp.run_episode(s, pnp.PnpConfig(control_hz=300.0))
