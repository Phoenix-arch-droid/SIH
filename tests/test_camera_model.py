"""
Unit tests for core.camera_model module.
Validates intrinsic calibration, pinhole projection, FOV bounds, and back-projection rays.
"""

import math
import numpy as np
import pytest
from core.camera_model import CameraIntrinsics, VirtualCamera


def test_camera_intrinsics_geometry():
    intrinsics = CameraIntrinsics(width=1280, height=720, hfov_deg=15.0)
    assert intrinsics.cx == 640.0
    assert intrinsics.cy == 360.0
    assert intrinsics.fx > 0.0
    assert intrinsics.fy == intrinsics.fx
    assert 5.0 < intrinsics.vfov_deg < 15.0


def test_camera_projection_on_optical_axis():
    cam = VirtualCamera()
    # Point directly ahead at range z = 100m, x=0, y=0
    p_cam = np.array([0.0, 0.0, 100.0])
    pixel_coord, in_fov, meta = cam.project_point(p_cam)
    assert in_fov is True
    assert pixel_coord is not None
    assert math.isclose(pixel_coord[0], cam.intrinsics.cx, abs_tol=1e-3)
    assert math.isclose(pixel_coord[1], cam.intrinsics.cy, abs_tol=1e-3)
    assert meta["image_space_error_x"] == 0.0
    assert meta["image_space_error_y"] == 0.0


def test_camera_projection_behind_sensor():
    cam = VirtualCamera()
    p_cam = np.array([10.0, 0.0, -50.0]) # Behind camera
    pixel_coord, in_fov, meta = cam.project_point(p_cam)
    assert in_fov is False
    assert pixel_coord is None
    assert meta["in_front"] is False


def test_camera_projection_outside_fov():
    cam = VirtualCamera()
    # At range 100m, HFOV=15 deg means max lateral half-width is 100 * tan(7.5 deg) ~ 13.16m
    # Lateral displacement of 40m is well outside FOV
    p_cam = np.array([40.0, 0.0, 100.0])
    pixel_coord, in_fov, meta = cam.project_point(p_cam)
    assert pixel_coord is not None
    assert in_fov is False
    assert pixel_coord[0] > cam.intrinsics.width


def test_pixel_to_ray_and_back():
    cam = VirtualCamera()
    u, v = 800.0, 500.0
    ray = cam.pixel_to_ray(u, v)
    assert math.isclose(np.linalg.norm(ray), 1.0, abs_tol=1e-6)
    # Re-project ray at depth 100m
    p_cam = ray * (100.0 / ray[2])
    p_proj, in_fov, _ = cam.project_point(p_cam)
    assert math.isclose(p_proj[0], u, abs_tol=1e-3)
    assert math.isclose(p_proj[1], v, abs_tol=1e-3)


def test_apparent_irradiance_attenuation():
    cam = VirtualCamera()
    irr_near = cam.calculate_apparent_beacon_irradiance(10.0, 15.0, 100.0)
    irr_far = cam.calculate_apparent_beacon_irradiance(10.0, 15.0, 500.0)
    # Inverse square distance attenuation: far irradiance must be much lower
    assert irr_near > irr_far
    assert irr_far > 0.0
