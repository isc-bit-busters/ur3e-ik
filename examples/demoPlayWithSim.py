import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_ikfast import ur_kinematics
import pandas as pd
import matplotlib.pyplot as plt
import json

# Initialize the UR3e Kinematics
ur3e_arm = ur_kinematics.URKinematics('ur3e')

def compute_inverse_kinematics(position, roll, pitch, yaw):
    # Convert roll, pitch, yaw to quaternion
    quaternion = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    q_w, q_x, q_y, q_z = quaternion  # Extract quaternion elements
    
    # Construct the 7-element vector format [tx, ty, tz, qw, qx, qy, qz]
    ik_input = np.array([*position, q_w, q_x, q_y, q_z])
    
    # Compute inverse kinematics
    joint_solutions = ur3e_arm.inverse(ik_input)
    
    return joint_solutions

def compute_orientation_towards_target(origin, target):
    direction = np.array(target) - np.array(origin)
    direction = direction / np.linalg.norm(direction)

    # Compute yaw (rotation around z-axis)
    yaw = np.arctan2(direction[1], direction[0])
    
    # Compute pitch (rotation around y-axis)
    pitch = np.arcsin(-direction[2])  # Assuming standard convention where downward is negative
    
    # Roll is assumed to be zero (no rotation around x-axis)
    roll = 0.0
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

def find_reachable_positions(roll=None, pitch=None, yaw=None):
    # Define grid of positions to test
    x_range = np.linspace(-0.8, 0.8, 30)  # UR3e has roughly 0.9m reach
    y_range = np.linspace(-0.8, 0.8, 30)
    z_range = np.linspace(0.0, 0.8, 20)   # UR3e operates above the base

    reachable_positions = []

    # Iterate over grid points
    for x in x_range:
        for y in y_range:
            for z in z_range:
                if roll is None or pitch is None or yaw is None:
                    # roll, pitch, yaw = compute_orientation_towards_target([0.0, 0.0, 0.0], [x, y, z])
                    d_roll, d_pitch, d_yaw = compute_orientation_towards_target([x, y, z], [0.0, 0.0, 0.0])
                else:
                    d_roll, d_pitch, d_yaw = roll, pitch, yaw

                if compute_inverse_kinematics([x, y, z], d_roll, d_pitch, d_yaw) is not None:
                    reachable_positions.append([x, y, z])

    # Convert to DataFrame for visualization
    df_reachable = pd.DataFrame(reachable_positions, columns=["X", "Y", "Z"])
    
    # print("Reachable Positions:")
    # print(df_reachable)
    
    # Save to CSV (optional)
    # df_reachable.to_csv("reachable_positions.csv", index=False)
    return df_reachable

def plot_reachable_positions(df_reachable):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(df_reachable['X'], df_reachable['Y'], df_reachable['Z'], c='blue', marker='o')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Reachable Positions for Given Orientation')
    # plt.show()
    plt.savefig("reachable_positions.png")

def plot_circular_trajectory(circle_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], c='red', marker='o')
    ax.plot(circle_points[:, 0], circle_points[:, 1], circle_points[:, 2], 'b-', alpha=0.6)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    ax.set_title('Circular Trajectory')
    plt.savefig("circular_trajectory.png")

def generate_circular_trajectory(center, radius, normal, num_points=50):
    angles = np.linspace(0, 2 * np.pi, num_points)
    circle_points = []
    
    # Define a rotation matrix to align the circle with the given normal
    normal = np.array(normal) / np.linalg.norm(normal)  # Normalize normal vector
    z_axis = np.array([0, 0, 1])
    if not np.allclose(normal, z_axis):
        rotation_matrix = R.align_vectors([normal], [z_axis])[0].as_matrix()
    else:
        rotation_matrix = np.eye(3)  # Identity matrix if normal is already aligned
    
    for theta in angles:
        local_point = np.array([radius * np.cos(theta), radius * np.sin(theta), 0])
        rotated_point = rotation_matrix @ local_point
        circle_points.append(center + rotated_point)
    
    return np.array(circle_points)

def compute_joint_trajectory_for_circle(center, radius, normal, fixed_orientation=True):
    circle_points = generate_circular_trajectory(center, radius, normal)
    joint_trajectory = []
    for point in circle_points:
        if fixed_orientation:
            roll, pitch, yaw = 90, 0, 0  # Fixed orientation
        else:
            roll, pitch, yaw = compute_orientation_towards_target(point, center)
        joint_angles = compute_inverse_kinematics(point, roll, pitch, yaw)
        if joint_angles is not None:
            joint_trajectory.append(joint_angles)
    return joint_trajectory

def generate_trajectory(data, filename="trajectory.json"):
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
    
    print(f"Fichier JSON '{filename}' généré avec succès.")

# # Example usage
# position = [-0.4, -0.2, 0.3]  # Define target position
# roll, pitch, yaw = 90, 0, 0  # Define desired orientation in degrees
# joint_angles = compute_inverse_kinematics(position, roll, pitch, yaw)

# print("Inverse Kinematics Solutions:")
# print(joint_angles)

# --------------------------------------------

# # Find all reachable positions for a given orientation
# reachable_positions = find_reachable_positions() # Direction is computed towards origin [0, 0, 0]
# # reachable_positions = find_reachable_positions(roll, pitch, yaw) # Direction is fixed to given orientation

# print("Reachable Positions:")
# print(reachable_positions)

# plot_reachable_positions(reachable_positions)

# --------------------------------------------

# Compute joint trajectory for a circular motion
center = [0., -0.4, 0.45]  # Circle center
radius = 0.2  # Circle radius
normal = [0, 1, 0] 

# circle_points = generate_circular_trajectory(center, radius, normal)
# plot_circular_trajectory(np.array(circle_points))

joint_trajectory = compute_joint_trajectory_for_circle(center, radius, normal, fixed_orientation=True)
print("Joint Trajectory for Circle:")
print(joint_trajectory)

# Generate trajectory file
generate_trajectory(joint_trajectory, filename="circle_trajectory.json")

