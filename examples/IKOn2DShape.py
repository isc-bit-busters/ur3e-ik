import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_ikfast import ur_kinematics
import pandas as pd
import matplotlib.pyplot as plt
import json

# Initialize the UR3e Kinematics
ur3e_arm = ur_kinematics.URKinematics('ur3e')


def compute_inverse_kinematics(pos_raw, previous_angles):
    """
    Computes the inverse kinematics from a desired position. Also uses the angles of the previous position to help decide which
    of the returned solutions is the best (the one that generates the least joint movement).

    Parameters
    ----------
    pos_raw : list[float]
        The desired position. it is structured like this: [x, y, z, roll, pitch, yaw] [m, m, m, deg, deg, deg]
    previous_angles : list[float]
        The angles that are currently applied to the robot arm. [rad, rad, rad, rad, rad, rad]

    Returns
    -------
    numpy.ndarray
        The angles [rad] of the 6 joints of the robot that will place the endpoint of the robot in the desired location, from base to wrists (so j1 -> j6).
    """
    # Splits the pos_raw input
    position = pos_raw[0:3]
    roll, pitch, yaw = pos_raw[3:6]

    # Converts roll, pitch, yaw to quaternion
    quaternion = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
    q_w, q_x, q_y, q_z = quaternion  # Extract quaternion elements
    
    # Constructs the 7-element vector format [tx, ty, tz, qw, qx, qy, qz]
    ik_input = np.array([*position, q_w, q_x, q_y, q_z])
    
    # Computes inverse kinematics
    joint_solutions = ur3e_arm.inverse(ik_input, q_guess=previous_angles)
    return joint_solutions



def compute_orientation_towards_target(origin, target):
    """
    [DESC]

    Parameters
    ----------
    origin : 
        
    target : 


    Returns
    -------
    
       
    """
    direction = np.array(target) - np.array(origin)
    direction = direction / np.linalg.norm(direction)

    # Compute yaw (rotation around z-axis)
    yaw = np.arctan2(direction[1], direction[0])
    
    # Compute pitch (rotation around y-axis)
    pitch = np.arcsin(-direction[2])  # Assuming standard convention where downward is negative
    
    # Roll is assumed to be zero (no rotation around x-axis)
    roll = 0.0
    
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)



def compute_joint_trajectory_for_points_tab(points, home):
    """
    Computes the inverse kinematics for a list of points.

    Parameters
    ----------
    points : list[list[float]]
        All the points in the trajectory. each point has to contain a valid position, so [x, y, z, roll, pitch, yaw] [m, m, m, deg, deg, deg]
    home: list[float]
        The home position of the robot. It will considered as the current position of the robot and the one to return to inbetween actions.
        It is mainly used to chose the first inverse kinematics solution, since we use the difference of all angles for our optimization.

    Returns
    -------
    list[[list[float]]]
        A list of all the joint positions to apply to the robot arm to make the endpoints pass by all desired positions. all angles are in [rad]. The values are ordered from base to wrists (so j1 -> j6)
    """
    # variable for the previous position of the arm, the first one being the home position
    previous_point = home

    joint_trajectory = []
    for point in points:
        # for each point in the points list
        # computes the inverse kinematics
        joint_angles = compute_inverse_kinematics(point, previous_point)
        if joint_angles is not None:
            # if a solution was found
            # adds the angles to the trajectory and sets the previous point for the next iteration
            joint_trajectory.append(joint_angles)
            previous_point = joint_angles
        else:
            # if no solution was found
            print(f"[ERROR] Could not find coordinates for the point: {point}")
    return joint_trajectory


def generate_trajectory(data, filename="trajectory.json"):
    """
    [DESC]

    Parameters
    ----------
    data : 
        
    filename : 


    Returns
    -------
    
       
    """
    modTraj = []
    time_step = 1  # Incrément du temps
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



def interpolate_lines(points, threshold):
    """
    Interpolates the lines in a trajectory (list of points) if they exceed a given threshold.

    Parameters
    ----------
    points : list[list[float]]
        A list of points respecting the following format: [x, y, z, roll, pitch, yaw] [m, m, m, deg, deg, deg]
    threshold : 
        The maximum distance that can be done before doing interpolation [m].

    Returns
    -------
    list[list[float]]    
        The input trajectories with added points from interpolation
       
    """
    # the first point can't change
    new_points = [points[0]]

    for i in range(1,len(points)):
        # for each point in the points list (starting from the second element)
        # gets the two points
        p1 = points[i-1]
        p2 = points[i]

        # splits the position
        x1,y1,z1 = p1[0:3]
        x2,y2,z2 = p2[0:3]

        # gets the difference between the two points
        dif_x = x2-x1
        dif_y = y2-y1
        dif_z = z2-z1 

        # computes the distance between the 2 points
        dist = math.sqrt(dif_x * dif_x + dif_y * dif_y + dif_z * dif_z)

        if dist > threshold:
            # if the distance is bigger than the threshold
            # computes how many parts the new division will have
            n_parts = int(dist / threshold)

            for j in range(1, n_parts):
                # for each step of the interpolation to be done
                # computes the next point and adds it to the list
                new_point = [round(x1 + j*dif_x/n_parts,5), round(y1 + j*dif_y/n_parts,5), round(z1 + j*dif_z/n_parts,5), p1[3], p1[4], p1[5]]
                new_points.append(new_point)

        # adds the last point of the interpolated line
        new_points.append(p2)
    
    return new_points


# ============================================================================================
# ===                                       2D shapes                                     ====
# ============================================================================================
# Z shape
#traj_points = [[0.064, 0.303, 0.35650000000000004], [0.064, 0.343, 0.35650000000000004], [0.10400000000000001, 0.303, 0.35650000000000004], [0.10400000000000001, 0.343, 0.35650000000000004], [0.10400000000000001, 0.343, 0.38650000000000007]]
# rectangle shape
traj_points = [[0.094, 0.2931, 0.35650000000000004], [0.114, 0.2931, 0.35650000000000004], [0.114, 0.2931, 0.38650000000000007], [0.094, 0.29460000000000003, 0.38650000000000007], [0.094, 0.29460000000000003, 0.35650000000000004], [0.114, 0.29460000000000003, 0.35650000000000004], [0.114, 0.29460000000000003, 0.38650000000000007], [0.094, 0.29610000000000003, 0.38650000000000007], [0.094, 0.29610000000000003, 0.35650000000000004], [0.114, 0.29610000000000003, 0.35650000000000004], [0.114, 0.29610000000000003, 0.38650000000000007], [0.094, 0.2976, 0.38650000000000007], [0.094, 0.2976, 0.35650000000000004], [0.114, 0.2976, 0.35650000000000004], [0.114, 0.2976, 0.38650000000000007], [0.094, 0.2991, 0.38650000000000007], [0.094, 0.2991, 0.35650000000000004], [0.114, 0.2991, 0.35650000000000004], [0.114, 0.2991, 0.38650000000000007], [0.094, 0.3006, 0.38650000000000007], [0.094, 0.3006, 0.35650000000000004], [0.114, 0.3006, 0.35650000000000004], [0.114, 0.3006, 0.38650000000000007], [0.094, 0.3021, 0.38650000000000007], [0.094, 0.3021, 0.35650000000000004], [0.114, 0.3021, 0.35650000000000004], [0.114, 0.3021, 0.38650000000000007], [0.094, 0.3036, 0.38650000000000007], [0.094, 0.3036, 0.35650000000000004], [0.114, 0.3036, 0.35650000000000004], [0.114, 0.3036, 0.38650000000000007], [0.094, 0.3051, 0.38650000000000007], [0.094, 0.3051, 0.35650000000000004], [0.114, 0.3051, 0.35650000000000004], [0.114, 0.3051, 0.38650000000000007], [0.094, 0.3066, 0.38650000000000007], [0.094, 0.3066, 0.35650000000000004], [0.114, 0.3066, 0.35650000000000004], [0.114, 0.3066, 0.38650000000000007], [0.094, 0.3081, 0.38650000000000007], [0.094, 0.3081, 0.35650000000000004], [0.114, 0.3081, 0.35650000000000004], [0.114, 0.3081, 0.38650000000000007], [0.094, 0.3096, 0.38650000000000007], [0.094, 0.3096, 0.35650000000000004], [0.114, 0.3096, 0.35650000000000004], [0.114, 0.3096, 0.38650000000000007], [0.094, 0.3111, 0.38650000000000007], [0.094, 0.3111, 0.35650000000000004], [0.114, 0.3111, 0.35650000000000004], [0.114, 0.3111, 0.38650000000000007], [0.094, 0.3126, 0.38650000000000007], [0.094, 0.3126, 0.35650000000000004], [0.114, 0.3126, 0.35650000000000004], [0.114, 0.3126, 0.38650000000000007], [0.094, 0.3141, 0.38650000000000007], [0.094, 0.3141, 0.35650000000000004], [0.114, 0.3141, 0.35650000000000004], [0.114, 0.3141, 0.38650000000000007], [0.094, 0.3156, 0.38650000000000007], [0.094, 0.3156, 0.35650000000000004], [0.114, 0.3156, 0.35650000000000004], [0.114, 0.3156, 0.38650000000000007], [0.094, 0.3171, 0.38650000000000007], [0.094, 0.3171, 0.35650000000000004], [0.114, 0.3171, 0.35650000000000004], [0.114, 0.3171, 0.38650000000000007], [0.094, 0.3186, 0.38650000000000007], [0.094, 0.3186, 0.35650000000000004], [0.114, 0.3186, 0.35650000000000004], [0.114, 0.3186, 0.38650000000000007], [0.094, 0.3201, 0.38650000000000007], [0.094, 0.3201, 0.35650000000000004], [0.114, 0.3201, 0.35650000000000004], [0.114, 0.3201, 0.38650000000000007], [0.094, 0.3216, 0.38650000000000007], [0.094, 0.3216, 0.35650000000000004], [0.114, 0.3216, 0.35650000000000004], [0.114, 0.3216, 0.38650000000000007]]
# adds the angles for the 2D shape that will be drawn, 180,0,0 (looking down)
joint_trajectory = []
for i in traj_points:
    joint_trajectory.append([i[0],i[1],i[2],180.1,0,0])

#print("==================================================================================================")
#print(f"Voila les points demandés en entrée: ({len(joint_trajectory)} points)")
#print(joint_trajectory)
#print("==================================================================================================")

# interpolates the trajectory
interpolated_traj = interpolate_lines(joint_trajectory, 0.006)

#print("==================================================================================================")
#print(f"Voila les points demandés en entrée avec des intermediaires pour raccourcir la distance: ({len(interpolated_traj)} points)")
#print(interpolated_traj)
#print("==================================================================================================")

# computes the joint trajectory for all the points we need it to go through
home = [0.950855376, -1.66225158, 0.635299848, -0.597600736, -1.57219259, 0]
traj = compute_joint_trajectory_for_points_tab(interpolated_traj, home)

# adds home position at start and finish
traj = [home] + traj + [home]

# Generate trajectory file
generate_trajectory(traj, filename="traj.json")