from ur_ikfast import ur_kinematics
import numpy as np
from scipy.spatial.transform import Rotation as R

import json

def load_points_cloud(filename="paths.json"):
    """Load the points cloud from the JSON file."""
    with open(filename) as f:
        paths = json.load(f)
    
    return paths

def transform_to_ik_inputs(paths):
    ik_inputs = []
    for path in paths:
        p = path[1]
        for i in p:
            new = [*i[0], *i[1]]
            ik_inputs.append(new)
    
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

    paths = load_points_cloud(filename="paths.json")
    ik_inputs = transform_to_ik_inputs(paths)

    joint_trajectory = multi_kin.inverse_optimal(ik_inputs)
    print("Optimal joint trajectory:")
    print(joint_trajectory)

    generate_trajectory_file(joint_trajectory, filename="square_trajectory.json")

if __name__ == "__main__":
    main()