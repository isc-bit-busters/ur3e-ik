from ur_ikfast import ur_kinematics
import numpy as np
from scipy.spatial.transform import Rotation as R

import json

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

def transform_coordinates_to_ee_poses(coordinates):
    """Transform path coordinates to IK input format.
    Parameters:
    - coordinates (list): List of (x, y, z) coordinates.
    Returns:
    - List of IK input poses (x, y, z, qx, qy, qz, qw).
    """
    ik_inputs = []
    
    for position in coordinates:
        orientation = [0, 179.942, 0]
        quaternion = R.from_euler('xyz', orientation, degrees=True).as_quat()
        ik_input = np.array([*position, *quaternion])
        ik_inputs.append(ik_input)

    return ik_inputs

def generate_trajectory_file(data, filename="trajectory.json"):
    modTraj = []
    time_step = 2  # Incrément du temps
    time = 4
    
    for arr in data:
        if arr is None:
            continue
        positions = [round(float(x), 4) if abs(x) >= 1e-4 else 0.0 for x in arr]
        velocities = [0.0] * 6  # Vélocités à zéro
        modTraj.append({
            "positions": positions,
            "velocities": velocities,
            "time_from_start": [time, 0]
        })
        time += time_step
    
    with open(filename, "w") as f:
        json.dump({"modTraj": modTraj}, f, indent=4)
    
    print(f"Fichier JSON '{filename}' généré avec succès.")

def main():
    """
    Main function to demonstrate the use of the optimal IK solution.
    From a list of (x, y, z) positions, with orientations fixed to (0, 179.942, 0), we compute a list of ik_inputs and the get the optimal joint trajectory from it.
    """
    print("Running demoOptimalIKSol.py")
    robot = ur_kinematics.URKinematics('ur3e')
    multi_kin = ur_kinematics.MultiURKinematics(robot)

    # To get (x, y, z) positions for the path
    path = generate_path_square(origin=(0.2, 0.2, 0.2), width=0.15, length=0.15, step=0.05)

    # To convert (x, y, z) positions and orientations fixed to (0, 179.942, 0) to joint angles
    ik_inputs = transform_coordinates_to_ee_poses(path)

    # From a list of ee_poses, get the optimal joint trajectory
    joint_trajectory = multi_kin.inverse_optimal(ik_inputs)
    print("Optimal joint trajectory:")
    print(joint_trajectory)

    generate_trajectory_file(joint_trajectory, filename="square_trajectory.json")

if __name__ == "__main__":
    main()