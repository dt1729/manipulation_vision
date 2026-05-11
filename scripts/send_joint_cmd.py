#!/usr/bin/env python3
"""
Send joint angle commands to the Piper arm in MuJoCo/robocasa simulation.

Actuators (position-controlled, set via env.sim.data.ctrl):
  robot0_joint1  : -2.618 to  2.618 rad  (base rotation)
  robot0_joint2  :  0.000 to  3.142 rad
  robot0_joint3  : -2.697 to  0.000 rad
  robot0_joint4  : -1.832 to  1.832 rad
  robot0_joint5  : -1.220 to  1.220 rad
  robot0_joint6  : -3.142 to  3.142 rad  (wrist rotation)
  robot0_joint7  : -0.200 to  0.200 m    (left  finger, slide)
  robot0_joint8  : -0.200 to  0.200 m    (right finger, slide)

Gripper convention (matches SimpleGripController / GRIP action=-1/+1):
  close: joint7 = -0.2,  joint8 = -0.2  (axes are opposed: j7=+z, j8=-z)
  open:  joint7 =  0.2,  joint8 =  0.2

Usage:
  python scripts/send_joint_cmd.py
  python scripts/send_joint_cmd.py --joints 0 0.5 -1.0 0 0.8 0
  python scripts/send_joint_cmd.py --joints 0 0.5 -1.0 0 0.8 0 --gripper close
"""

import argparse
import numpy as np
import robocasa  # noqa: F401
from robocasa.utils.env_utils import create_env

ARM_JOINTS  = [f"robot0_joint{i}" for i in range(1, 7)]
GRIP_JOINTS = ["robot0_joint7", "robot0_joint8"]

# ctrlrange from robot.xml
JOINT_LIMITS = {
    "robot0_joint1": (-2.618,  2.618),
    "robot0_joint2": ( 0.000,  3.14158),
    "robot0_joint3": (-2.697,  0.000),
    "robot0_joint4": (-1.832,  1.832),
    "robot0_joint5": (-1.220,  1.220),
    "robot0_joint6": (-3.14158, 3.14158),
    "robot0_joint7": (-0.200,  0.200),
    "robot0_joint8": (-0.200,  0.200),
}


def set_gripper(env, close: bool):
    val = 0.0 if close else 0.05
    j7 = env.sim.model.actuator_name2id("robot0_joint7")
    j8 = env.sim.model.actuator_name2id("robot0_joint8")
    env.sim.data.ctrl[j7] = val
    env.sim.data.ctrl[j8] = -val


def send_joint_cmd(env, joint_angles_rad: list, gripper_close: bool,
                   max_steps: int = 5000, atol: float = 0.005):
    """
    Set arm joints and step until all joints converge to within atol radians.
    Gripper ctrl is re-asserted every step so nothing can override it.
    joint_angles_rad: list of 6 values [j1..j6] in radians.
    """
    act_ids = []
    cmds = []
    for i, name in enumerate(ARM_JOINTS):
        act_id = env.sim.model.actuator_name2id(name)
        lo, hi = JOINT_LIMITS[name]
        cmd = float(np.clip(joint_angles_rad[i], lo, hi))
        env.sim.data.ctrl[act_id] = cmd
        act_ids.append(act_id)
        cmds.append(cmd)

    jnt_ids = [env.sim.model.joint_name2id(n) for n in ARM_JOINTS]
    qpos_addrs = [env.sim.model.jnt_qposadr[jid] for jid in jnt_ids]

    for step in range(max_steps):
        set_gripper(env, close=gripper_close)
        env.sim.step()
        env.viewer.update()

        qpos = np.array([env.sim.data.qpos[a] for a in qpos_addrs])
        if np.allclose(qpos, cmds, atol=atol):
            print(f"  Converged in {step + 1} steps (atol={atol} rad).")
            return

    print(f"  Warning: did not fully converge after {max_steps} steps.")
    qpos = np.array([env.sim.data.qpos[a] for a in qpos_addrs])
    errs = np.abs(qpos - cmds)
    print(f"  Max error: {errs.max():.4f} rad  (joint {ARM_JOINTS[np.argmax(errs)]})")


def print_joint_state(env):
    print("\nCurrent joint state:")
    for name in ARM_JOINTS + GRIP_JOINTS:
        jnt_name = name.replace("robot0_", "robot0_")
        act_id = env.sim.model.actuator_name2id(name)
        # read actual joint position from qpos
        jnt_id = env.sim.model.joint_name2id(name.replace("robot0_joint", "robot0_joint"))
        qpos_addr = env.sim.model.jnt_qposadr[jnt_id]
        pos = env.sim.data.qpos[qpos_addr]
        print(f"  {name}: {pos:.4f}  (ctrl={env.sim.data.ctrl[act_id]:.4f})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joints", type=float, nargs=6, metavar="RAD",
                        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        help="6 joint angles in radians [j1..j6]")
    parser.add_argument("--gripper", choices=["open", "close"], default="open",
                        help="Gripper state (default: open)")
    parser.add_argument("--steps", type=int, default=5000,
                        help="Max sim steps to run for convergence (default 5000)")
    args = parser.parse_args()

    print("Loading PiperOmron kitchen env...")
    env = create_env(
        env_name="PickPlaceCounterToCabinet",
        robots="PiperOmron",
        render_onscreen=True,
        renderer="mjviewer",
        render_camera=None,
    )
    env.reset()

    # Sync arm ctrl to actual qpos so position actuators start from the real joint state.
    # Gripper is excluded — set_gripper() handles it explicitly below.
    for name in ARM_JOINTS:
        act_id = env.sim.model.actuator_name2id(name)
        jnt_id = env.sim.model.joint_name2id(name)
        qpos_addr = env.sim.model.jnt_qposadr[jnt_id]
        env.sim.data.ctrl[act_id] = env.sim.data.qpos[qpos_addr]

    # Set gripper state early so it holds throughout the arm motion
    set_gripper(env, close=(args.gripper == "close"))

    print_joint_state(env)

    print(f"\nSending command: joints={[round(j,4) for j in args.joints]} rad, gripper={args.gripper}")
    gripper_close = (args.gripper == "close")
    send_joint_cmd(env, args.joints, gripper_close, max_steps=args.steps)

    print_joint_state(env)

    print("\nHolding pose — close the viewer window or Ctrl+C to exit.")
    while True:
        set_gripper(env, close=gripper_close)
        env.sim.step()
        env.viewer.update()


if __name__ == "__main__":
    main()
