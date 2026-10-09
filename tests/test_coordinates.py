"""
Unit tests for core.coordinates module.
Validates SE(3) kinematics, Euler rotations, line-of-sight vectors, and angular error math.
"""

import math
import numpy as np
import pytest
from core.coordinates import Pose3D, CoordinateTransform


def test_pose3d_rotation_matrix_identity():
    pose = Pose3D(x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
    R = pose.rotation_matrix()
    assert np.allclose(R, np.eye(3), atol=1e-6)


def test_pose3d_yaw_rotation():
    pose = Pose3D(yaw=math.pi / 2) # 90 deg yaw
    R = pose.rotation_matrix()
    # [1, 0, 0] rotated 90 deg around Z becomes [0, 1, 0]
    p_rot = R @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(p_rot, np.array([0.0, 1.0, 0.0]), atol=1e-6)


def test_pointing_vector_neutral():
    # At pan=0, tilt=0, camera points along +Y_world (North)
    v = CoordinateTransform.compute_pointing_vector(0.0, 0.0)
    assert np.allclose(v, np.array([0.0, 1.0, 0.0]), atol=1e-6)


def test_pointing_vector_elevation_tilt():
    # Tilt up 90 deg -> pointing along +Z_world (Up)
    v = CoordinateTransform.compute_pointing_vector(0.0, math.pi / 2)
    assert np.allclose(v, np.array([0.0, 0.0, 1.0]), atol=1e-6)


def test_pointing_vector_azimuth_pan():
    # Pan 90 deg clockwise/counterclockwise
    v = CoordinateTransform.compute_pointing_vector(math.pi / 2, 0.0)
    assert abs(np.linalg.norm(v) - 1.0) < 1e-6
    assert abs(v[2]) < 1e-6 # Z is zero


def test_target_direction_and_range():
    cam_pos = np.array([0.0, 0.0, 0.0])
    tgt_pos = np.array([300.0, 400.0, 0.0])
    los, r = CoordinateTransform.compute_target_direction_vector(tgt_pos, cam_pos)
    assert math.isclose(r, 500.0, rel_tol=1e-5)
    assert np.allclose(los, np.array([0.6, 0.8, 0.0]), atol=1e-5)


def test_angular_pointing_error_zero_misalignment():
    cam_pos = np.array([0.0, 0.0, 0.0])
    tgt_pos = np.array([0.0, 500.0, 0.0]) # Target directly North
    err = CoordinateTransform.compute_angular_pointing_error(tgt_pos, cam_pos, pan_rad=0.0, tilt_rad=0.0)
    assert math.isclose(err["total_error_deg"], 0.0, abs_tol=1e-4)
    assert math.isclose(err["azimuth_error_deg"], 0.0, abs_tol=1e-4)
    assert math.isclose(err["elevation_error_deg"], 0.0, abs_tol=1e-4)


def test_angular_pointing_error_known_offset():
    cam_pos = np.array([0.0, 0.0, 0.0])
    tgt_pos = np.array([0.0, 500.0, 50.0]) # Elevation angle ~ 5.71 deg
    expected_el = math.degrees(math.atan2(50.0, 500.0))
    err = CoordinateTransform.compute_angular_pointing_error(tgt_pos, cam_pos, pan_rad=0.0, tilt_rad=0.0)
    assert math.isclose(err["elevation_error_deg"], expected_el, rel_tol=1e-3)
    assert math.isclose(err["total_error_deg"], expected_el, rel_tol=1e-3)
