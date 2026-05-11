import trimesh
import numpy as np

def generate_sphere_mesh(radius=0.0002, subdivisions=3, filename="sphere.stl"):
    # Create an icosphere (more uniform triangles than a UV sphere)
    # Subdivisions: 3 (~1280 faces), 4 (~5120 faces)
    mesh = trimesh.creation.icosphere(subdivisions=subdivisions, radius=radius)
    
    # Export to STL
    mesh.export(filename)
    print(f"Successfully generated {filename} with {len(mesh.faces)} faces.")

if __name__ == "__main__":
    # Change radius to match your {BALL_RADIUS}
    generate_sphere_mesh(radius=0.009, subdivisions=4, filename="/home/dt/manipulation_vision/includes/robosuite/robosuite/models/assets/robots/piper_arm/meshes/mujoco_sphere.stl")
