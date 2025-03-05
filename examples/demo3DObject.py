import numpy as np
from ur_ikfast import ur_kinematics
import json

# Initialize the UR3e Kinematics
ur3e_arm = ur_kinematics.URKinematics('ur3e')

def compute_inverse_kinematics(position, quaternion, last_joint=None):
    """Compute all inverse kinematics solutions for a given position and quaternion orientation."""
    ik_input = np.array([*position, *quaternion])
    if last_joint is not None:
        joint_solutions = ur3e_arm.inverse(ik_input, q_guess=last_joint)
    else:
        joint_solutions = ur3e_arm.inverse(ik_input)
    return joint_solutions

def generate_trajectory_file(data, filename="trajectory.json"):
    modTraj = []
    time_step = 2  # Incrément du temps
    time = 4
    
    for arr in data:
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
    
    print(f"Trajectory file '{filename}' generated successfully.")

def load_points_cloud():
    """Load a predefined set of 3D points as a placeholder for point cloud data."""
    # TODO: Implement actual point cloud loading from a file
    return [
        (np.float64(0.2), np.float64(0.2), np.float64(0.2)),
        (np.float64(0.2), np.float64(0.25), np.float64(0.2)),
        (np.float64(0.2), np.float64(0.3), np.float64(0.2)),
        (np.float64(0.2), np.float64(0.35), np.float64(0.2)),
        (np.float64(0.25), np.float64(0.35), np.float64(0.2)),
        (np.float64(0.25), np.float64(0.31666666666666665), np.float64(0.2)),
        (np.float64(0.25), np.float64(0.2833333333333333), np.float64(0.2)),
        (np.float64(0.25), np.float64(0.25), np.float64(0.2)),
        (np.float64(0.30000000000000004), np.float64(0.2), np.float64(0.2)),
        (np.float64(0.30000000000000004), np.float64(0.25), np.float64(0.2)),
        (np.float64(0.30000000000000004), np.float64(0.3), np.float64(0.2)),
        (np.float64(0.30000000000000004), np.float64(0.35), np.float64(0.2)),
        (np.float64(0.35000000000000003), np.float64(0.35), np.float64(0.2)),
        (np.float64(0.35000000000000003), np.float64(0.31666666666666665), np.float64(0.2)),
        (np.float64(0.35000000000000003), np.float64(0.2833333333333333), np.float64(0.2)),
        (np.float64(0.35000000000000003), np.float64(0.25), np.float64(0.2))
    ]

def transform_coordinates_to_joint_angles(coordinates, orientations):
    """Transform path coordinates into joint angles using inverse kinematics, with specific quaternion orientation per point."""
    joint_trajectories = []
    last_joint = None
    
    for position, quaternion in zip(coordinates, orientations):
        solution = compute_inverse_kinematics(position, quaternion, last_joint)
        if solution is not None:
            last_joint = solution
            joint_trajectories.append(solution)
    
    return joint_trajectories

path = load_points_cloud()
print(f"Generated {len(path)} path points.")

# Define specific quaternion orientations for each point in the path
orientations = [(0, 9.9e-1, 0, 5.06e-4) for _ in path]

# Compute all joint solutions for each path point
joint_trajectory = transform_coordinates_to_joint_angles(path, orientations)
print(f"Computed {len(joint_trajectory)} joint solutions for the trajectory.")

print(joint_trajectory)

# Generate trajectory file
generate_trajectory_file(joint_trajectory, filename="3dmodel_trajectory.json")
