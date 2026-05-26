#!/usr/bin/env python3
"""
Piper arm gain tuner.

GUI: kp / kv sliders per joint  →  run step-response sim  →  plot reference vs actual.

Usage:
    python tune_gains.py [path/to/robot.xml]

Defaults to the piper_description.xml inside this repo.
"""

import sys
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt


# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
_HERE = Path(__file__).parent
_REPO_ROOT = _HERE.parent.parent
DEFAULT_XML = (
    _REPO_ROOT
    / "src/piper_ros/src/piper_description/mujoco_model/piper_description.xml"
)

# --------------------------------------------------------------------------- #
# Sim constants  — kept in sync with send_joint_cmd.py
# --------------------------------------------------------------------------- #
SIM_TIMESTEP   = 0.001   # s
SIM_INTEGRATOR = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
SIM_ITERATIONS = 20
SIM_DURATION   = 5.0     # s
RECORD_EVERY   = 5       # record one sample every N physics steps

# PiperOmron init_qpos (from robosuite compositional.py) — 6 arm joints + 2 gripper
# Tuner starts the robot here so the step-response matches the real env
INIT_QPOS = np.array([0.0, 1.57, -1.57, 0.0, 1.22, 0.0, 0.0, 0.0])


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def parse_actuators(xml_path: Path) -> list[dict]:
    """Return list of dicts with name, joint, default_kp, ctrl_lo, ctrl_hi."""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    result = []
    for act in root.findall(".//actuator/position"):
        lo_str, hi_str = act.get("ctrlrange", "0 0").split()
        result.append({
            "name":       act.get("name"),
            "joint":      act.get("joint"),
            "default_kp": float(act.get("kp", 100)),
            "default_kv": float(act.get("kv", 0)),
            "ctrl_lo":    float(lo_str),
            "ctrl_hi":    float(hi_str),
        })
    return result


def joint_qpos_addresses(model, actuators: list[dict]) -> list[int]:
    """Map each actuator's joint name → index into data.qpos."""
    addrs = []
    for act in actuators:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, act["joint"])
        addrs.append(int(model.jnt_qposadr[jid]))
    return addrs



# --------------------------------------------------------------------------- #
# App
# --------------------------------------------------------------------------- #
class GainTunerApp:
    def __init__(self, xml_path: Path):
        self.xml_path  = xml_path
        self.actuators = parse_actuators(xml_path)
        self.n         = len(self.actuators)

        self.root = tk.Tk()
        self.root.title(f"MuJoCo Gain Tuner — {xml_path.name}")
        self.root.resizable(True, True)

        self._sim_running = False
        self._build_gui()

    # ------------------------------------------------------------------ build

    def _build_gui(self):
        # ── left: gain sliders ──────────────────────────────────────────────
        left = ttk.LabelFrame(self.root, text="Gains", padding=8)
        left.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

        # headers
        for col, txt in enumerate(["Joint", "kp", "kp value", "kv", "kv value"]):
            ttk.Label(left, text=txt, font=("", 9, "bold")).grid(
                row=0, column=col, padx=4, pady=(0, 4)
            )

        self.kp_vars: list[tk.DoubleVar] = []
        self.kv_vars: list[tk.DoubleVar] = []

        for i, act in enumerate(self.actuators):
            row = i + 1
            ttk.Label(left, text=act["name"], width=10, anchor="w").grid(
                row=row, column=0, padx=4
            )

            kp_var = tk.DoubleVar(value=act["default_kp"])
            tk.Scale(
                left, variable=kp_var, orient="horizontal",
                from_=0, to=max(20000, act["default_kp"] * 2),
                resolution=50, length=220, showvalue=False,
            ).grid(row=row, column=1, padx=2)
            ttk.Entry(left, textvariable=kp_var, width=8).grid(row=row, column=2, padx=2)
            self.kp_vars.append(kp_var)

            kv_var = tk.DoubleVar(value=act["default_kv"])
            tk.Scale(
                left, variable=kv_var, orient="horizontal",
                from_=0, to=500, resolution=1, length=220, showvalue=False,
            ).grid(row=row, column=3, padx=2)
            ttk.Entry(left, textvariable=kv_var, width=8).grid(row=row, column=4, padx=2)
            self.kv_vars.append(kv_var)

        # ── right: setpoints + controls ─────────────────────────────────────
        right = ttk.LabelFrame(self.root, text="Setpoints", padding=8)
        right.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)

        for col, txt in enumerate(["Joint", "Target", "Value"]):
            ttk.Label(right, text=txt, font=("", 9, "bold")).grid(
                row=0, column=col, padx=4, pady=(0, 4)
            )

        self.sp_vars: list[tk.DoubleVar] = []
        for i, act in enumerate(self.actuators):
            lo, hi = act["ctrl_lo"], act["ctrl_hi"]
            sp_var = tk.DoubleVar(value=round(float(np.clip(INIT_QPOS[i], lo, hi)), 4))
            ttk.Label(right, text=act["name"], width=10, anchor="w").grid(
                row=i + 1, column=0, padx=4
            )
            tk.Scale(
                right, variable=sp_var, orient="horizontal",
                from_=lo, to=hi, resolution=0.001, length=200, showvalue=False,
            ).grid(row=i + 1, column=1, padx=2)
            ttk.Entry(right, textvariable=sp_var, width=8).grid(row=i + 1, column=2, padx=2)
            self.sp_vars.append(sp_var)

        # duration + buttons
        ctrl = ttk.Frame(right)
        ctrl.grid(row=self.n + 2, column=0, columnspan=3, pady=10)

        ttk.Label(ctrl, text="Duration (s):").pack(side="left")
        self.dur_var = tk.DoubleVar(value=SIM_DURATION)
        ttk.Entry(ctrl, textvariable=self.dur_var, width=6).pack(side="left", padx=6)

        self.run_btn = ttk.Button(ctrl, text="Run Simulation", command=self._on_run)
        self.run_btn.pack(side="left", padx=6)

        ttk.Button(ctrl, text="Export XML", command=self._on_export).pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(right, textvariable=self.status_var, foreground="steelblue").grid(
            row=self.n + 3, column=0, columnspan=3
        )

        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=0)

    # --------------------------------------------------------------- run sim

    def _on_run(self):
        if self._sim_running:
            return
        self._sim_running = True
        self.run_btn.state(["disabled"])
        self.status_var.set("Simulating…")
        threading.Thread(target=self._simulate, daemon=True).start()

    def _simulate(self):
        try:
            kp      = np.array([v.get() for v in self.kp_vars])
            kv      = np.array([v.get() for v in self.kv_vars])
            targets = np.array([v.get() for v in self.sp_vars])
            dur     = max(0.1, self.dur_var.get())

            model = mujoco.MjModel.from_xml_path(str(self.xml_path))
            data  = mujoco.MjData(model)

            model.opt.timestep   = SIM_TIMESTEP
            model.opt.integrator = SIM_INTEGRATOR
            model.opt.iterations = SIM_ITERATIONS

            for i in range(self.n):
                model.actuator_gainprm[i, 0] = kp[i]
                model.actuator_biasprm[i, 1] = -kp[i]
                model.actuator_biasprm[i, 2] = -kv[i]

            qpos_ids = joint_qpos_addresses(model, self.actuators)

            # Seed robot at PiperOmron's init_qpos so the starting pose matches the real env
            for idx, qpos_addr in enumerate(qpos_ids):
                data.qpos[qpos_addr] = INIT_QPOS[idx]
            data.ctrl[: self.n] = INIT_QPOS[: self.n]
            mujoco.mj_forward(model, data)

            n_steps  = int(dur / SIM_TIMESTEP)
            t_arr, ref_arr, fb_arr = [], [], []

            for step in range(n_steps):
                data.ctrl[: self.n] = targets

                if step % RECORD_EVERY == 0:
                    t_arr.append(step * SIM_TIMESTEP)
                    ref_arr.append(targets.copy())
                    fb_arr.append(np.array([data.qpos[idx] for idx in qpos_ids]))

                mujoco.mj_step(model, data)

            times   = np.array(t_arr)
            pos_ref = np.array(ref_arr)
            pos_fb  = np.array(fb_arr)

            self.root.after(0, lambda: self._show_plots(times, pos_ref, pos_fb))

        except Exception as exc:
            self.root.after(0, lambda: messagebox.showerror("Simulation error", str(exc)))
        finally:
            self._sim_running = False
            self.root.after(0, lambda: self.run_btn.state(["!disabled"]))
            self.root.after(0, lambda: self.status_var.set("Done."))

    # ------------------------------------------------------------------ plot

    def _show_plots(self, times, pos_ref, pos_fb):
        cols = 2
        rows = (self.n + cols - 1) // cols

        fig, axes = plt.subplots(rows, cols, figsize=(13, 2.8 * rows), sharex=True)
        fig.suptitle("Step Response — Reference vs Actual Position",
                     fontsize=13, fontweight="bold")
        axes_flat = np.array(axes).flatten()

        for i, act in enumerate(self.actuators):
            ax = axes_flat[i]
            ax.plot(times, pos_ref[:, i], "r--", linewidth=1.5, label="Reference")
            ax.plot(times, pos_fb[:, i],  "b-",  linewidth=1.0, label="Actual")
            error = pos_ref[:, i] - pos_fb[:, i]
            ax.fill_between(times, 0, error, alpha=0.15, color="red", label="Error")
            ax.set_title(act["name"], fontsize=10)
            unit = "m" if "joint7" in act["name"] or "joint8" in act["name"] else "rad"
            ax.set_ylabel(unit, fontsize=8)
            ax.legend(fontsize=7, loc="upper right")
            ax.grid(True, alpha=0.3)

        # Hide unused subplots
        for j in range(self.n, len(axes_flat)):
            axes_flat[j].set_visible(False)

        # x-label on bottom row
        for ax in axes_flat[max(0, self.n - cols): self.n]:
            ax.set_xlabel("Time (s)", fontsize=8)

        plt.tight_layout()
        plt.show(block=False)

    # --------------------------------------------------------------- export

    def _on_export(self):
        out = filedialog.asksaveasfilename(
            defaultextension=".xml",
            filetypes=[("XML files", "*.xml")],
            initialfile="piper_tuned.xml",
        )
        if not out:
            return

        tree = ET.parse(self.xml_path)
        root = tree.getroot()
        for i, act_el in enumerate(root.findall(".//actuator/position")):
            act_el.set("kp", str(int(self.kp_vars[i].get())))
            kv = self.kv_vars[i].get()
            if kv > 0:
                act_el.set("kv", str(round(kv, 2)))
            elif "kv" in act_el.attrib:
                del act_el.attrib["kv"]

        try:
            ET.indent(tree, space="    ")   # Python ≥ 3.9
        except AttributeError:
            pass

        tree.write(out, xml_declaration=False)
        self.status_var.set(f"Exported → {Path(out).name}")

    # --------------------------------------------------------------- launch

    def run(self):
        self.root.mainloop()


# --------------------------------------------------------------------------- #
def main():
    xml_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_XML
    if not xml_path.exists():
        print(f"XML not found: {xml_path}")
        print("Usage: python tune_gains.py [path/to/robot.xml]")
        sys.exit(1)
    GainTunerApp(xml_path).run()


if __name__ == "__main__":
    main()
