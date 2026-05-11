import robocasa  # noqa: F401
from robocasa.utils.env_utils import create_env
import numpy as np
import imageio
from tqdm import tqdm
from termcolor import colored

env = create_env(
    env_name="PickPlaceCounterToCabinet",
    robots="PiperOmron",
    camera_names=["robot0_agentview_left", "robot0_agentview_right", "robot0_eye_in_hand"],
    camera_widths=256,
    camera_heights=256,
)

writer_left = imageio.get_writer("/tmp/piper_agentview_left.mp4", fps=20)
writer_eih = imageio.get_writer("/tmp/piper_eye_in_hand.mp4", fps=20)

obs = env.reset()

# gripper actuator IDs
j7_id = env.sim.model.actuator_name2id("robot0_joint7")
j8_id = env.sim.model.actuator_name2id("robot0_joint8")

def set_gripper(env, close: bool):
    env.sim.data.ctrl[j7_id] = -0.2 if close else 0.2
    env.sim.data.ctrl[j8_id] = 0.2 if close else -0.2

# close gripper first, then teleport object between closed fingers
set_gripper(env, close=True)
for _ in range(10):
    env.sim.step()

obj = list(env.objects.values())[0]
eef_pos = env.sim.data.get_body_xpos("robot0_right_hand").copy()
jnt_name = obj.joints[0]
qpos_addr = env.sim.model.get_joint_qpos_addr(jnt_name)
env.sim.data.qpos[qpos_addr[0]:qpos_addr[0]+3] = eef_pos
# zero out object velocity so it doesn't fly away
env.sim.data.qvel[qpos_addr[0]:qpos_addr[0]+3] = 0
env.sim.forward()
print(f"Gripper closed. Teleported {obj.name} to EEF at {eef_pos}")

# verify compiled model geom properties
for name in ["robot0_link7_col", "robot0_link8_col"]:
    gid = env.sim.model.geom_name2id(name)
    print(f"{name}: contype={env.sim.model.geom_contype[gid]} conaffinity={env.sim.model.geom_conaffinity[gid]}")
j7_pos = env.sim.data.get_joint_qpos("robot0_joint7")
j8_pos = env.sim.data.get_joint_qpos("robot0_joint8")
obj_pos = env.sim.data.get_body_xpos(obj.root_body)
print(f"joint7={j7_pos:.4f} joint8={j8_pos:.4f} | obj_pos={obj_pos}")

link7_id = env.sim.model.body_name2id("robot0_link7")
link8_id = env.sim.model.body_name2id("robot0_link8")

action = np.zeros(env.action_spec[0].shape)  # hold position

for step_i in tqdm(range(100)):
    obs, _, _, _ = env.step(action)
    set_gripper(env, close=True)
    env.sim.forward()

    tac_left  = np.array([[env.sim.data.get_sensor(f"robot0_tac_left_{r}{c}") for c in range(4)] for r in range(4)])
    tac_right = np.array([[env.sim.data.get_sensor(f"robot0_tac_right_{r}{c}") for c in range(4)] for r in range(4)])
    if step_i % 20 == 0:
        # check finger contacts
        robot_contacts = [(env.sim.model.geom_id2name(env.sim.data.contact[i].geom1),
                           env.sim.model.geom_id2name(env.sim.data.contact[i].geom2))
                          for i in range(env.sim.data.ncon)
                          if "robot" in (env.sim.model.geom_id2name(env.sim.data.contact[i].geom1) or "")
                          or "robot" in (env.sim.model.geom_id2name(env.sim.data.contact[i].geom2) or "")]
        print(f"\nstep {step_i} | ncon={env.sim.data.ncon} robot_contacts={robot_contacts}")
        print(f"left finger (4x4):\n{np.round(tac_left,1)}")
        print(f"right finger (4x4):\n{np.round(tac_right,1)}")

    writer_left.append_data(env.sim.render(height=512, width=512, camera_name="robot0_agentview_left")[::-1])
    writer_eih.append_data(env.sim.render(height=512, width=512, camera_name="robot0_eye_in_hand")[:, ::-1, :])

writer_left.close()
writer_eih.close()
print(colored("Saved agentview_left -> /tmp/piper_agentview_left.mp4", color="yellow"))
print(colored("Saved eye_in_hand   -> /tmp/piper_eye_in_hand.mp4", color="yellow"))
