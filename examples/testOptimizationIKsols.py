import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_ikfast import ur_kinematics
import json

# Initialize the UR3e Kinematics
ur3e_arm = ur_kinematics.URKinematics('ur3e')

def compute_inverse_kinematics(position, roll, pitch, yaw, last_joint=None):
    """Compute all inverse kinematics solutions for a given position and orientation."""
    quaternion = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    ik_input = np.array([*position, *quaternion])
    
    joint_solutions = ur3e_arm.inverse(ik_input, q_guess=last_joint, all_solutions=True)
    return joint_solutions

def generate_path_square(origin=(0, 0, 0), width=0.1, length=0.1, step=0.01):
    """Generate a simple square trajectory."""
    x0, y0, z0 = origin
    path = []
    
    for i in range(int(width / step) + 1):
        x = x0 + i * step
        for j in range(int(length / step) + 1):
            y = y0 + (j * step if i % 2 == 0 else length - j * step)
            path.append((x, y, z0))
    
    return path

def transform_coordinates_to_joint_angles(coordinates, orientation=(0, 179.942, 0)):
    """Transform path coordinates into joint angles using inverse kinematics."""
    joint_trajectories = []
    
    for position in coordinates:
        solutions = compute_inverse_kinematics(position, *orientation)
        print(f"Found {len(solutions)} solutions for position {position}.", solutions)
        joint_trajectories.append(solutions)

    return joint_trajectories

# Generate a simple square path
path = generate_path_square(origin=(0.2, 0.2, 0.2), width=0.15, length=0.15, step=0.05)
print(f"Generated {len(path)} path points.")

# Compute all joint solutions for each path point
joint_trajectory = transform_coordinates_to_joint_angles(path, orientation=(0, 179.942, 0))
print("Computed joint solutions for the trajectory.")

print(joint_trajectory)
