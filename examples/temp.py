import numpy as np
import ur3e_ikfast  # Import the IKFast module for UR3e

# Initialize kinematics for UR3e robot arm
ur3e_kin = ur3e_ikfast.PyKinematics()
n_joints = ur3e_kin.getDOF()

# Define the desired end-effector pose as a 3x4 transformation matrix
# This example uses an identity rotation and a translation of 0.5 meters along the x-axis
ee_pose = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0.0], [0, 0, 1, 0.0]])

# Compute inverse kinematics to find joint angles that achieve the desired end-effector pose
joint_configs = ur3e_kin.inverse(ee_pose.reshape(-1).tolist())
n_solutions = int(len(joint_configs) / n_joints)
joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)

# Display the number of solutions found
print(f"{n_solutions} solutions found:")

# Iterate through each solution and print the joint angles
for i, joint_config in enumerate(joint_configs):
    print(f"Solution {i + 1}: {joint_config}")
