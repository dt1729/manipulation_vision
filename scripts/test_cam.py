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
# constant EEF linear motion: move forward in x at half max speed, everything else zero
action = np.zeros(env.action_spec[0].shape)
action[0] = 0.5  # EEF +x translation (OSC_POSE index 0)

for step_i in tqdm(range(50)):
    obs, _, _, _ = env.step(action)
    writer_left.append_data(env.sim.render(height=512, width=512, camera_name="robot0_agentview_left")[::-1])
    writer_eih.append_data(env.sim.render(height=512, width=512, camera_name="robot0_eye_in_hand")[:, ::-1, :])

writer_left.close()
writer_eih.close()
print(colored("Saved agentview_left -> /tmp/piper_agentview_left.mp4", color="yellow"))
print(colored("Saved eye_in_hand   -> /tmp/piper_eye_in_hand.mp4", color="yellow"))
