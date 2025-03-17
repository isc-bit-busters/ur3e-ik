import pytest
import math
import numpy as np
from IKOn2DShape import compute_inverse_kinematics, compute_orientation_towards_target, interpolate_lines
from ur_ikfast import ur_kinematics

ur3e_arm = ur_kinematics.URKinematics('ur3e')

@pytest.fixture
def ik_result():
	"""test fixture for the compute_inverse_kinematics function"""
	pos_raw = [0.2, 0.2, 0.5, 1, 0.1, 0.2, 0.3]
	previous_angles = [0, 0, 0, 0, 0, 0]
	result = compute_inverse_kinematics(pos_raw, previous_angles)
	return result, pos_raw

def test_compute_inverse_kinematics_result_type(ik_result):
	"""Test if the result of the compute_inverse_kinematics function is a numpy array of the right size"""
	result, _ = ik_result
	assert isinstance(result, np.ndarray), "The result should be a numpy array"
	assert result.shape == (6,), "The result should be a 6-element array"

def test_compute_inverse_kinematics_forward(ik_result):
	"""Test if the result is correct by comparing the input to the forward kinematics of the output"""
	result, pos_raw = ik_result
	pose_quat = ur3e_arm.forward(result)
	assert np.allclose(pose_quat[:3], pos_raw[:3], atol=1e-3), "The position should be the same"
	assert np.allclose(pose_quat[3:6], pos_raw[4:7], atol=5e-2), "The orientation (qx, qy, qz) should be the same"
	assert np.allclose(pose_quat[6], pos_raw[3], atol=1e-1), "The orientation (w) should be the same"

def test_interpolate_lines():
	"""test if the line interpolation function works as expected"""
	pose1 = np.array([0, 0, 0, 1, 0, 0, 0])
	pose2 = np.array([0, 0, 1, 1, 0, 0, 0])
	poses = interpolate_lines([pose1, pose2], 0.05)
	assert len(poses) == 21, "The result should be a list of 21 poses"
	for i in range(1, len(poses)):
		dist = math.sqrt(math.pow(poses[i][0] - poses[i-1][0], 2) + math.pow(poses[i][1] - poses[i-1][1], 2) + math.pow(poses[i][2] - poses[i-1][2], 2))
		assert dist < 0.5, "The poses should be less than 0.5 apart"
		
	assert np.allclose(poses[0], pose1), "The first pose should be the same as the first input"
	assert np.allclose(poses[-1], pose2), "The last pose should be the same as the second input"

