"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: simulation.ground_truth

Isolated Ground Truth Telemetry Provider:
- Maintains true physical state of target and camera at every simulation step
- Strictly isolated: Ground truth is NEVER passed to detection, tracking, or control loops
- Solely consumed by verification, metrics logging, comparison analytics, and visualization
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Dict, Any, Optional
from core.coordinates import CoordinateTransform, Pose3D
from core.camera_model import VirtualCamera


@dataclass
class GroundTruthFrame:
    timestamp_s: float
    target_id: str
    true_pos_world: np.ndarray        # [x, y, z] in meters
    true_vel_world: np.ndarray        # [vx, vy, vz] in m/s
    true_accel_world: np.ndarray      # [ax, ay, az] in m/s^2
    camera_pos_world: np.ndarray      # [x, y, z]
    camera_pan_deg: float             # Actual gimbal pan (deg)
    camera_tilt_deg: float            # Actual gimbal tilt (deg)
    true_los_unit: np.ndarray         # Unit line-of-sight vector from camera to target
    true_camera_pointing: np.ndarray  # Unit pointing vector along camera boresight
    slant_range_m: float              # Physical distance between terminal and beacon (m)
    true_pixel_u: Optional[float]     # Exact projection on sensor (None if behind camera)
    true_pixel_v: Optional[float]     # Exact projection on sensor (None if behind camera)
    is_in_fov: bool                   # True if optical line-of-sight falls inside sensor active area
    is_occluded: bool                 # True if line of sight is blocked by obstacle/cloud
    is_optically_visible: bool        # True if in FOV and not occluded
    
    # Precise True Errors
    true_angular_error_deg: float     # Total angular misalignment (deg)
    true_angular_error_mrad: float    # Total angular misalignment (mrad)
    true_azimuth_error_deg: float     # True azimuth error (deg)
    true_elevation_error_deg: float   # True elevation error (deg)
    true_image_error_u: Optional[float] # Pixels from optical crosshair
    true_image_error_v: Optional[float] # Pixels from optical crosshair


class GroundTruthProvider:
    """
    Computes exact reference metrics for evaluation and benchmarking.
    """

    def __init__(self, camera: VirtualCamera):
        self.camera = camera

    def evaluate_frame(
        self,
        timestamp_s: float,
        target_id: str,
        target_pos_world: np.ndarray,
        target_vel_world: np.ndarray,
        target_accel_world: np.ndarray,
        camera_pos_world: np.ndarray,
        camera_pan_rad: float,
        camera_tilt_rad: float,
        is_occluded: bool = False,
        platform_pose: Optional[Pose3D] = None
    ) -> GroundTruthFrame:
        """
        Computes the complete, ground-truth physical state of the simulation step.
        """
        # 1. Coordinate transformation from World to Camera frame
        p_cam = CoordinateTransform.world_to_camera_point(
            target_pos_world, camera_pos_world,
            camera_pan_rad, camera_tilt_rad,
            platform_pose
        )

        # 2. Camera projection
        pixel_coord, in_fov, _ = self.camera.project_point(p_cam)
        u_true = float(pixel_coord[0]) if pixel_coord is not None else None
        v_true = float(pixel_coord[1]) if pixel_coord is not None else None

        # 3. Line of sight and pointing vectors
        los_unit, slant_range = CoordinateTransform.compute_target_direction_vector(
            target_pos_world, camera_pos_world
        )
        cam_pointing = CoordinateTransform.compute_pointing_vector(
            camera_pan_rad, camera_tilt_rad
        )

        # 4. True angular errors
        ang_err_dict = CoordinateTransform.compute_angular_pointing_error(
            target_pos_world, camera_pos_world,
            camera_pan_rad, camera_tilt_rad
        )

        img_err_u = (u_true - self.camera.intrinsics.cx) if u_true is not None else None
        img_err_v = (v_true - self.camera.intrinsics.cy) if v_true is not None else None
        is_visible = in_fov and (not is_occluded)

        return GroundTruthFrame(
            timestamp_s=timestamp_s,
            target_id=target_id,
            true_pos_world=np.array(target_pos_world, copy=True),
            true_vel_world=np.array(target_vel_world, copy=True),
            true_accel_world=np.array(target_accel_world, copy=True),
            camera_pos_world=np.array(camera_pos_world, copy=True),
            camera_pan_deg=math.degrees(camera_pan_rad),
            camera_tilt_deg=math.degrees(camera_tilt_rad),
            true_los_unit=los_unit,
            true_camera_pointing=cam_pointing,
            slant_range_m=slant_range,
            true_pixel_u=u_true,
            true_pixel_v=v_true,
            is_in_fov=in_fov,
            is_occluded=is_occluded,
            is_optically_visible=is_visible,
            true_angular_error_deg=ang_err_dict["total_error_deg"],
            true_angular_error_mrad=ang_err_dict["total_error_mrad"],
            true_azimuth_error_deg=ang_err_dict["azimuth_error_deg"],
            true_elevation_error_deg=ang_err_dict["elevation_error_deg"],
            true_image_error_u=img_err_u,
            true_image_error_v=img_err_v
        )
