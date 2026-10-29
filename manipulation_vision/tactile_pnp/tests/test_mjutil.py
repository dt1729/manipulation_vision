import mujoco
import pytest

from manipulation_vision.tactile_pnp import mjutil

XML = """
<mujoco>
  <worldbody>
    <body name="b">
      <joint name="hinge" type="hinge"/>
      <geom name="g" size="0.1"/>
      <site name="s" size="0.01"/>
      <camera name="cam"/>
    </body>
    <body name="c">
      <freejoint name="free"/>
      <geom size="0.1"/>
      <body name="d">
        <joint name="slide" type="slide"/>
        <geom size="0.1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="m_hinge" joint="hinge"/>
  </actuator>
  <sensor>
    <jointpos name="p_hinge" joint="hinge"/>
    <framepos name="p_site" objtype="site" objname="s"/>
  </sensor>
</mujoco>
"""


@pytest.fixture(scope="module")
def model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_string(XML)


def test_qpos_adr_of_joint_after_freejoint_skips_seven_slots(model):
    assert mjutil.joint_qpos_adr(model, "slide") == 1 + 7


def test_dof_adr_of_joint_after_freejoint_skips_six_slots(model):
    assert mjutil.joint_dof_adr(model, "slide") == 1 + 6


def test_unknown_joint_raises_key_error_naming_it(model):
    with pytest.raises(KeyError) as caught:
        mjutil.joint_qpos_adr(model, "nope")
    assert caught.value.args[0] == "joint 'nope' not in model"


def test_unknown_site_raises_instead_of_returning_minus_one(model):
    with pytest.raises(KeyError):
        mjutil.site_id(model, "missing_site")


@pytest.mark.parametrize(
    ("lookup", "kind", "name"),
    [
        (mjutil.actuator_id, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_hinge"),
        (mjutil.body_id, mujoco.mjtObj.mjOBJ_BODY, "c"),
        (mjutil.geom_id, mujoco.mjtObj.mjOBJ_GEOM, "g"),
        (mjutil.camera_id, mujoco.mjtObj.mjOBJ_CAMERA, "cam"),
        (mjutil.site_id, mujoco.mjtObj.mjOBJ_SITE, "s"),
    ],
)
def test_lookups_agree_with_mujoco_and_raise_on_unknown(model, lookup, kind, name):
    assert lookup(model, name) == mujoco.mj_name2id(model, kind, name)
    with pytest.raises(KeyError):
        lookup(model, name + "_missing")


def test_obj_id_resolves_index_zero_without_raising(model):
    # index 0 is valid; only a negative id means "missing"
    assert mjutil.obj_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "m_hinge") == 0


def test_sensor_adr_is_sensordata_offset_not_sensor_id(model):
    # p_site is sensor id 1 but its data starts after the 1-wide jointpos
    assert mjutil.sensor_adr(model, "p_site") == 1
    assert mjutil.sensor_adr(model, "p_hinge") == 0
