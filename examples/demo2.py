from ur_ikfast import ur_kinematics

ur3e_arm = ur_kinematics.URKinematics("ur3e")

# pose_matrix = array(
#     [
#         [-0.54030228, 0.84147096, 0.0, 0.0],
#         [0.0, 0.0, 1.0, 0.3],
#         [0.84147096, 0.54030228, -0.0, 0.7],
#     ],
#     dtype=float,
# )

# print("forward() matrix \n", pose_matrix)

# # Try with a reasonable joint angle guess
# joint_guess = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # A common neutral position

# print(
#     "inverse() one from matrix",
#     ur3e_arm.inverse(pose_matrix, False, q_guess=joint_guess),
# )

joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.0]  # in radians
print("joint angles", joint_angles)

pose_quat = ur3e_arm.forward(joint_angles)
pose_matrix = ur3e_arm.forward(joint_angles, "matrix")

print(type(pose_matrix))

print("forward() quaternion \n", pose_quat)
print("forward() matrix \n", pose_matrix)

# print("inverse() all", ur3e_arm.inverse(pose_quat, True))
print(
    "inverse() one from quat", ur3e_arm.inverse(pose_quat, False, q_guess=joint_angles)
)

print(
    "inverse() one from matrix",
    ur3e_arm.inverse(pose_matrix, False, q_guess=joint_angles),
)
