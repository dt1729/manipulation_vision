#!/usr/bin/env python3
"""
Send joint angle commands to the Piper arm in MuJoCo/robocasa simulation.

Behavior is driven by a JSON config file (default: scripts/config/piper_behavior.json).
Any config value can be overridden at the CLI.

Usage:
  python scripts/send_joint_cmd.py --joints 0 0.5 -1.0 0 0.8 0
  python scripts/send_joint_cmd.py --config scripts/config/piper_behavior.json --joints 0 0.5 -1.0 0 0.8 0
  python scripts/send_joint_cmd.py --joints 0 0.5 -1.0 0 0.8 0 --max-vel 0.2 --duration 15

Actuators (position-controlled):
  robot0_joint1  : -2.618 to  2.618 rad
  robot0_joint2  :  0.000 to  3.142 rad
  robot0_joint3  : -2.697 to  0.000 rad
  robot0_joint4  : -1.832 to  1.832 rad
  robot0_joint5  : -1.220 to  1.220 rad
  robot0_joint6  : -3.142 to  3.142 rad
  robot0_joint7  : -0.200 to  0.200 m  (left  finger)
  robot0_joint8  : -0.200 to  0.200 m  (right finger)
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np
import robocasa  # noqa: F401
from robocasa.utils.env_utils import create_env

# --------------------------------------------------------------------------- #
_SCRIPT_DIR  = Path(__file__).parent
DEFAULT_CONFIG = _SCRIPT_DIR / "config" / "piper_behavior.json"

INTEGRATOR_MAP = {
    "EULER":        mujoco.mjtIntegrator.mjINT_EULER,
    "IMPLICITFAST": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "IMPLICIT":     mujoco.mjtIntegrator.mjINT_IMPLICIT,
}

ARM_JOINTS  = [f"robot0_joint{i}" for i in range(1, 7)]
GRIP_JOINTS = ["robot0_joint7", "robot0_joint8"]

JOINT_LIMITS = {
    "robot0_joint1": (-2.618,   2.618),
    "robot0_joint2": ( 0.000,   3.14158),
    "robot0_joint3": (-2.697,   0.000),
    "robot0_joint4": (-1.832,   1.832),
    "robot0_joint5": (-1.220,   1.220),
    "robot0_joint6": (-3.14158, 3.14158),
    "robot0_joint7": (-0.200,   0.200),
    "robot0_joint8": (-0.200,   0.200),
}

BASE_JOINTS = [
    "robot0_joint_mobile_forward",
    "robot0_joint_mobile_side",
    "robot0_joint_mobile_yaw",
    "robot0_joint_torso_height",
]


# --------------------------------------------------------------------------- #
# Config loading
# --------------------------------------------------------------------------- #

def load_config(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


def merge_config(cfg: dict, args: argparse.Namespace) -> dict:
    """Apply CLI overrides on top of the JSON config. None means not supplied."""
    overrides = {
        "sim.timestep":    args.timestep,
        "sim.iterations":  args.iterations,
        "motion.duration": args.duration,
        "motion.settle_time": args.settle,
        "motion.max_vel":  args.max_vel,
        "motion.max_acc":  args.max_acc,
    }
    for dotkey, val in overrides.items():
        if val is None:
            continue
        section, key = dotkey.split(".")
        cfg[section][key] = val
    return cfg


def print_config(cfg: dict):
    print("\nActive config:")
    for section, values in cfg.items():
        if isinstance(values, dict):
            print(f"  [{section}]")
            for k, v in values.items():
                print(f"    {k}: {v}")
        elif isinstance(values, list):
            print(f"  [{section}]  ({len(values)} entries)")
            for i, v in enumerate(values):
                print(f"    {i}: {v}")
    print()


# --------------------------------------------------------------------------- #
# Simulation setup
# --------------------------------------------------------------------------- #

def configure_sim(env, cfg: dict):
    sim_cfg = cfg["sim"]
    timestep   = sim_cfg["timestep"]
    integrator = INTEGRATOR_MAP[sim_cfg["integrator"]]
    iterations = sim_cfg["iterations"]

    env.sim.model.opt.timestep   = timestep
    env.sim.model.opt.integrator = integrator
    env.sim.model.opt.iterations = iterations
    print(f"  sim: timestep={timestep}s  integrator={sim_cfg['integrator']}  iterations={iterations}")

    locked = []
    for name in BASE_JOINTS:
        try:
            jid = env.sim.model.joint_name2id(name)
            dof = env.sim.model.jnt_dofadr[jid]
            env.sim.model.dof_damping[dof] = 1e6
            locked.append(name)
        except Exception:
            pass
    if locked:
        print(f"  base locked: {locked}")


def set_gripper(env, close: bool):
    val = 0.0 if close else 0.05
    j7 = env.sim.model.actuator_name2id("robot0_joint7")
    j8 = env.sim.model.actuator_name2id("robot0_joint8")
    env.sim.data.ctrl[j7] =  val
    env.sim.data.ctrl[j8] = -val


# --------------------------------------------------------------------------- #
# Motion
# --------------------------------------------------------------------------- #

def trapezoidal_profile(q0: np.ndarray, qf: np.ndarray,
                        max_vel: float, max_acc: float, dt: float) -> np.ndarray:
    """Time-synchronised trapezoidal profile. Returns (N_steps, n_joints)."""
    dist = np.abs(qf - q0)
    sign = np.sign(qf - q0)

    t_acc  = np.where(dist > 0, max_vel / max_acc, 0.0)
    d_acc  = 0.5 * max_acc * t_acc ** 2
    tri    = dist < 2 * d_acc
    t_acc  = np.where(tri, np.sqrt(np.maximum(dist, 0) / max_acc), t_acc)
    d_acc  = np.where(tri, 0.5 * max_acc * t_acc ** 2, d_acc)
    v_peak = np.where(tri, max_acc * t_acc, max_vel)
    t_cruise = np.where(tri, 0.0, (dist - 2 * d_acc) / max_vel)
    T = float(np.max(2 * t_acc + t_cruise))

    if T < dt:
        return qf.reshape(1, -1)

    steps   = int(np.ceil(T / dt)) + 1
    profile = np.zeros((steps, len(q0)))
    for s in range(steps):
        t = s * dt
        for j in range(len(q0)):
            if dist[j] == 0:
                profile[s, j] = q0[j]
                continue
            ta, tc, vp = t_acc[j], t_cruise[j], v_peak[j]
            da = 0.5 * max_acc * ta ** 2
            if t <= ta:
                q = 0.5 * max_acc * t ** 2
            elif t <= ta + tc:
                q = da + vp * (t - ta)
            elif t <= 2 * ta + tc:
                td = t - ta - tc
                q = da + vp * tc + vp * td - 0.5 * max_acc * td ** 2
            else:
                q = dist[j]
            profile[s, j] = q0[j] + sign[j] * min(q, dist[j])
    return profile


def send_joint_cmd(env, joint_angles_rad: list, gripper_close: bool, cfg: dict,
                   settle: bool = True):
    """
    Settle (first waypoint only) → trapezoidal profile → converge.
    Runs until convergence; timeout = settle + profile_duration + 5s headroom.
    Returns (times, pos_ref, pos_fb).
    """
    m_cfg = cfg["motion"]
    settle_time = m_cfg["settle_time"] if settle else 0.0
    max_vel     = m_cfg["max_vel"]
    max_acc     = m_cfg["max_acc"]
    atol        = m_cfg["atol"]

    jnt_ids    = [env.sim.model.joint_name2id(n) for n in ARM_JOINTS]
    qpos_addrs = [env.sim.model.jnt_qposadr[jid] for jid in jnt_ids]
    act_ids    = [env.sim.model.actuator_name2id(n) for n in ARM_JOINTS]

    lo_hi = [JOINT_LIMITS[n] for n in ARM_JOINTS]
    cmds  = np.array([float(np.clip(joint_angles_rad[i], *lo_hi[i]))
                      for i in range(len(ARM_JOINTS))])

    dt           = env.sim.model.opt.timestep
    settle_steps = int(settle_time / dt)

    times, pos_ref_log, pos_fb_log = [], [], []
    profile      = None
    profile_step = 0
    timeout_steps = None   # set once profile is built

    step = 0
    while True:
        t    = step * dt
        qpos = np.array([env.sim.data.qpos[a] for a in qpos_addrs])

        if step < settle_steps:
            ref = qpos.copy()
        else:
            if profile is None:
                profile = trapezoidal_profile(qpos.copy(), cmds, max_vel, max_acc, dt)
                # timeout = settle + profile + 5s headroom
                timeout_steps = settle_steps + len(profile) + int(5.0 / dt)
                print(f"  profile: {len(profile) * dt:.2f}s  timeout: {timeout_steps * dt:.1f}s total")
            ref = profile[min(profile_step, len(profile) - 1)]
            profile_step += 1

        for aid, val in zip(act_ids, ref):
            env.sim.data.ctrl[aid] = val

        set_gripper(env, close=gripper_close)
        env.sim.step()
        if step % 20 == 0:
            env.viewer.update()

        times.append(t)
        pos_ref_log.append(ref.copy())
        pos_fb_log.append(qpos.copy())

        if step % 500 == 0:
            phase = "settling" if step < settle_steps else "moving"
            print(f"  step {step:5d} ({phase}) | max_err={np.abs(qpos - ref).max():.4f} rad")

        if profile is not None and profile_step >= len(profile):
            if np.allclose(qpos, cmds, atol=atol):
                print(f"  converged at step {step}.")
                break

        if timeout_steps and step >= timeout_steps:
            errs = np.abs(qpos - cmds)
            print(f"  timeout. max_err={errs.max():.4f} rad ({ARM_JOINTS[np.argmax(errs)]})")
            break

        step += 1

    return np.array(times), np.array(pos_ref_log), np.array(pos_fb_log)


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #

def plot_joint_angles(times, pos_ref, pos_fb):
    n    = len(ARM_JOINTS)
    cols = 2
    rows = (n + 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(12, 2.5 * rows), sharex=True)
    fig.suptitle("Joint Trapezoidal Profile — Reference vs Actual", fontsize=13, fontweight="bold")
    axes_flat = np.array(axes).flatten()

    for i, name in enumerate(ARM_JOINTS):
        ax = axes_flat[i]
        ax.plot(times, pos_ref[:, i], "r--", linewidth=1.5, label="Reference")
        ax.plot(times, pos_fb[:, i],  "b-",  linewidth=1.0, label="Actual")
        ax.fill_between(times, pos_ref[:, i], pos_fb[:, i], alpha=0.15, color="red")
        ax.set_title(name, fontsize=10)
        ax.set_ylabel("rad", fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    for j in range(n, len(axes_flat)):
        axes_flat[j].set_visible(False)
    for ax in axes_flat[max(0, n - cols): n]:
        ax.set_xlabel("Time (s)", fontsize=8)

    plt.tight_layout()
    plt.show()


def print_joint_state(env):
    print("\nCurrent joint state:")
    for name in ARM_JOINTS + GRIP_JOINTS:
        act_id    = env.sim.model.actuator_name2id(name)
        jnt_id    = env.sim.model.joint_name2id(name)
        qpos_addr = env.sim.model.jnt_qposadr[jnt_id]
        pos = env.sim.data.qpos[qpos_addr]
        print(f"  {name}: {pos:.4f}  (ctrl={env.sim.data.ctrl[act_id]:.4f})")


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(
        description="Send joint commands to PiperOmron sim, driven by a JSON config."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG,
                        help=f"Path to behavior JSON (default: {DEFAULT_CONFIG})")
    parser.add_argument("--joints", type=float, nargs=6, metavar="RAD", default=None,
                        help="Single target [j1..j6] rad — overrides config waypoints")
    parser.add_argument("--gripper", choices=["open", "close"], default="open")

    # Optional per-run overrides (all default to None = use config value)
    parser.add_argument("--duration",   type=float, default=None, help="Override motion.duration (s)")
    parser.add_argument("--timestep",   type=float, default=None, help="Override sim.timestep (s)")
    parser.add_argument("--settle",     type=float, default=None, help="Override motion.settle_time (s)")
    parser.add_argument("--max-vel",    type=float, default=None, help="Override motion.max_vel (rad/s)")
    parser.add_argument("--max-acc",    type=float, default=None, help="Override motion.max_acc (rad/s²)")
    parser.add_argument("--iterations", type=int,   default=None, help="Override sim.iterations")

    args = parser.parse_args()

    cfg = load_config(args.config)
    cfg = merge_config(cfg, args)
    print_config(cfg)

    print("Loading PiperOmron kitchen env...")
    env = create_env(
        env_name=cfg["robot"]["env_name"],
        robots="PiperOmron",
        render_onscreen=True,
        renderer="mjviewer",
        render_camera=None,
    )
    env.reset()
    configure_sim(env, cfg)

    for name in ARM_JOINTS:
        act_id    = env.sim.model.actuator_name2id(name)
        jnt_id    = env.sim.model.joint_name2id(name)
        qpos_addr = env.sim.model.jnt_qposadr[jnt_id]
        env.sim.data.ctrl[act_id] = env.sim.data.qpos[qpos_addr]

    gripper_close = (args.gripper == "close")
    set_gripper(env, close=gripper_close)
    print_joint_state(env)

    # --joints overrides config waypoints; otherwise use config waypoints
    waypoints = [args.joints] if args.joints is not None else cfg["waypoints"]
    n_wp = len(waypoints)
    print(f"\n{n_wp} waypoint(s) to execute  gripper={args.gripper}")
    for i, wp in enumerate(waypoints):
        print(f"  {i + 1}: {[round(j, 4) for j in wp]}")

    all_times, all_ref, all_fb = [], [], []
    t_offset = 0.0

    for idx, wp in enumerate(waypoints):
        print(f"\n--- Waypoint {idx + 1}/{len(waypoints)}: {[round(j, 4) for j in wp]} ---")
        times, pos_ref, pos_fb = send_joint_cmd(env, wp, gripper_close, cfg,
                                                 settle=(idx == 0))
        all_times.append(times + t_offset)
        all_ref.append(pos_ref)
        all_fb.append(pos_fb)
        t_offset += times[-1] if len(times) else 0.0

    print_joint_state(env)
    plot_joint_angles(
        np.concatenate(all_times),
        np.concatenate(all_ref),
        np.concatenate(all_fb),
    )

    print("\nHolding pose — close the viewer window or Ctrl+C to exit.")
    while True:
        set_gripper(env, close=gripper_close)
        env.sim.step()
        env.viewer.update()


if __name__ == "__main__":
    main()
