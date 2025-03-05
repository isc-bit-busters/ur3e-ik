import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_ikfast import ur_kinematics
import json

# Initialize the UR3e Kinematics
ur3e_arm = ur_kinematics.URKinematics('ur3e')

def compute_inverse_kinematics(position, roll, pitch, yaw):
    """Compute all inverse kinematics solutions for a given position and orientation."""
    quaternion = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()

    print(f"Computing IK solutions for position {position} and orientation {quaternion}.")
    ik_input = np.array([*position, *quaternion])
    
    joint_solutions = ur3e_arm.inverse(ik_input, all_solutions=True)
    return joint_solutions

def generate_path_square(origin=(0, 0, 0), direction=(1, 0, 0), width=0.1, length=0.1, step=0.01, depth=0.01):
    """
    Generate a zigzag path in a square space with a defined origin and movement direction.

    Parameters:
    - origin (tuple): (x0, y0, z0) starting position.
    - direction (tuple): (dx, dy, dz) vector indicating movement direction.
    - width (float): The width of the zigzag area.
    - height (float): The height of the zigzag area.
    - step (float): The step size for movement.
    - depth (float): How much the path dips when going down.

    Returns:
    - List of (x, y, z) points representing the path.
    """
    x0, y0, z0 = origin
    dx, dy, dz = direction
    
    # Normalize the direction vector to ensure step sizes are correct
    norm = np.sqrt(dx**2 + dy**2 + dz**2)
    dx, dy, dz = dx / norm, dy / norm, dz / norm  # Unit vector

    path = []
    
    # Generate positions along the main movement direction
    steps = np.arange(0, width + step, step)  # Move along width
    for i, s in enumerate(steps):
        # Compute main path movement using direction vector
        x = x0 + s * dx
        y = y0 + s * dy
        z = z0 + s * dz  # Z can change if direction includes dz

        # Zigzag motion along height (perpendicular movement)
        if i % 2 == 0:
            y_positions = np.linspace(y, y + length, num=len(steps))
        else:
            y_positions = np.linspace(y + length, y + depth, num=len(steps))

        for y_pos in y_positions:
            path.append((x, y_pos, z))  # Store the position
    
    return path


def transform_coordinates_to_joint_angles(coordinates, orientation=(0, 179.942, 0)):
    """Transform path coordinates into joint angles using inverse kinematics."""
    joint_trajectories = []
    
    for position in coordinates:
        solutions = compute_inverse_kinematics(position, *orientation)

        if solutions is not None:
            print(f"Found {len(solutions)} solutions for position {position}.", solutions)
            joint_trajectories.append(solutions)
        else:
            print(f"No solutions found for position {position}.")

    return joint_trajectories

# Generate a simple square path
path = generate_path_square(origin=(0.2, 0.2, 0.2), width=0.15, length=0.15, step=0.05)
print(f"Generated {len(path)} path points.")

# Compute all joint solutions for each path point
joint_trajectory = transform_coordinates_to_joint_angles(path, orientation=(0, 179.942, 0))
print("Computed joint solutions for the trajectory.")

print(joint_trajectory)
