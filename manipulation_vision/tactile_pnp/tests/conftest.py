import os

# Headless rendering for the camera tests; must be set before mujoco creates a GL context.
os.environ.setdefault("MUJOCO_GL", "egl")
