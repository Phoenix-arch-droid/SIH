"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: core.coordinates

Rigorous 3D kinematics and SE(3) coordinate transformations connecting:
- World Reference Frame (W): Cartesian [X, Y, Z]
- Platform Frame (P): Mobile host terminal with 6-DOF pose [x, y, z, roll, pitch, yaw]
- Gimbal Frame (G): 2-DOF Pan (Azimuth) and Tilt (Elevation) mount
- Camera Sensor Optical Frame (C): Optical axis pointing along +Z (or +Y forward, +X right, +Z up)
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Dict, Any


@dataclass
class Pose3D:
    """Rigid body 3D position and orientation."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0   # Bank angle (rad)
    pitch: float = 0.0  # Elevation attitude (rad)
    yaw: float = 0.0    # Heading / Azimuth (rad)

    def position_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    def rotation_matrix(self) -> np.ndarray:
        """Computes Euler rotation matrix R_ZYX (Yaw -> Pitch -> Roll)."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)

        Rz = np.array([
            [cy, -sy, 0.0],
            [sy,  cy, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        Ry = np.array([
            [ cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp]
        ], dtype=np.float64)

        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0,  cr, -sr],
            [0.0,  sr,  cr]
        ], dtype=np.float64)

        return Rz @ Ry @ Rx


class CoordinateTransform:
    """
    Manages coordinate transformations for the FSOC tracking terminal.
    
    Frames convention:
    - World Frame (W): Right-handed Cartesian.
      +X: East / Baseline Lateral
      +Y: North / Baseline Range
      +Z: Up / Altitude
    - Camera Sensor Frame (C):
      +X_cam: Image right
      +Y_cam: Image down (standard computer vision convention)
      +Z_cam: Optical axis forward (line-of-sight boresight)
    """

    @staticmethod
    def euler_to_rotation_matrix(pan_rad: float, tilt_rad: float) -> np.ndarray:
        """
        Calculates rotation from gimbal base to camera boresight frame.
        Pan rotates around +Z (azimuth).
        Tilt rotates around rotated +X (elevation).
        """
        c_p, s_p = math.cos(pan_rad), math.sin(pan_rad)
        c_t, s_t = math.cos(tilt_rad), math.sin(tilt_rad)

        # Pan around Z
        R_pan = np.array([
            [ c_p, -s_p, 0.0],
            [ s_p,  c_p, 0.0],
            [ 0.0,  0.0, 1.0]
        ], dtype=np.float64)

        # Tilt around X
        R_tilt = np.array([
            [1.0,  0.0,   0.0],
            [0.0,  c_t,  -s_t],
            [0.0,  s_t,   c_t]
        ], dtype=np.float64)

        # Standard optical axis convention:
        # Camera +Z points along forward line-of-sight (boresight)
        # Camera +X points right
        # Camera +Y points down
        # Alignment mapping from world [East, North, Up] to optical [Right, Down, Forward]:
        # At zero pan and zero tilt:
        # Forward is +Y_world -> +Z_cam
        # Right is +X_world -> +X_cam
        # Down is -Z_world -> +Y_cam
        R_world_to_cam_neutral = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, -1.0],
            [0.0,  1.0,  0.0]
        ], dtype=np.float64)

        # Total rotation from world to camera sensor frame
        R_gimbal = R_pan @ R_tilt
        R_cw = R_world_to_cam_neutral @ R_gimbal.T
        return R_cw

    @staticmethod
    def world_to_camera_point(
        point_world: np.ndarray,
        camera_pos_world: np.ndarray,
        pan_rad: float,
        tilt_rad: float,
        platform_pose: Pose3D = None
    ) -> np.ndarray:
        """
        Transforms a 3D point from World coordinates into Camera Optical coordinates [x_c, y_c, z_c].
        
        z_c is depth along optical boresight.
        x_c is lateral displacement to the right.
        y_c is vertical displacement downwards.
        """
        p_w = np.asarray(point_world, dtype=np.float64)
        c_w = np.asarray(camera_pos_world, dtype=np.float64)
        
        # Translation relative to camera center
        rel_w = p_w - c_w

        # If platform attitude disturbance is present
        if platform_pose is not None:
            R_plat = platform_pose.rotation_matrix()
            rel_w = R_plat.T @ rel_w

        R_cw = CoordinateTransform.euler_to_rotation_matrix(pan_rad, tilt_rad)
        p_cam = R_cw @ rel_w
        return p_cam

    @staticmethod
    def compute_pointing_vector(pan_rad: float, tilt_rad: float) -> np.ndarray:
        """
        Computes the unit pointing vector of the camera boresight in World Frame.
        At pan=0, tilt=0, pointing is along +Y_world.
        """
        c_p, s_p = math.cos(pan_rad), math.sin(pan_rad)
        c_t, s_t = math.cos(tilt_rad), math.sin(tilt_rad)

        # Unit vector in gimbal frame
        # Forward axis under pan (azimuth) and tilt (elevation):
        vx = -s_p * c_t
        vy =  c_p * c_t
        vz =  s_t

        v = np.array([vx, vy, vz], dtype=np.float64)
        norm = np.linalg.norm(v)
        return v / norm if norm > 1e-9 else np.array([0.0, 1.0, 0.0])

    @staticmethod
    def compute_target_direction_vector(
        target_pos_world: np.ndarray,
        camera_pos_world: np.ndarray
    ) -> Tuple[np.ndarray, float]:
        """
        Returns the unit line-of-sight vector from camera to target in World Frame,
        and the slant range distance in meters.
        """
        delta = np.asarray(target_pos_world, dtype=np.float64) - np.asarray(camera_pos_world, dtype=np.float64)
        range_m = float(np.linalg.norm(delta))
        if range_m < 1e-9:
            return np.array([0.0, 1.0, 0.0], dtype=np.float64), 0.0
        unit_dir = delta / range_m
        return unit_dir, range_m

    @staticmethod
    def compute_angular_pointing_error(
        target_pos_world: np.ndarray,
        camera_pos_world: np.ndarray,
        pan_rad: float,
        tilt_rad: float
    ) -> Dict[str, float]:
        """
        Calculates the true angular pointing error between camera boresight and target beacon:
        - Azimuth error (rad & deg)
        - Elevation error (rad & deg)
        - Total angular separation (rad & deg & mrad)
        """
        los_unit, slant_range = CoordinateTransform.compute_target_direction_vector(
            target_pos_world, camera_pos_world
        )
        cam_pointing = CoordinateTransform.compute_pointing_vector(pan_rad, tilt_rad)

        # Dot product for total angular separation
        dot = float(np.clip(np.dot(cam_pointing, los_unit), -1.0, 1.0))
        total_error_rad = math.acos(dot)

        # Target true azimuth and elevation in world
        # Azimuth from +Y (North) towards +X (East): clockwise vs counterclockwise convention
        # Here: pan=0 is +Y, positive pan rotates towards -X or +X
        target_az = math.atan2(-los_unit[0], los_unit[1])
        target_el = math.asin(np.clip(los_unit[2], -1.0, 1.0))

        az_error_rad = target_az - pan_rad
        # Normalize to [-pi, pi]
        az_error_rad = (az_error_rad + math.pi) % (2.0 * math.pi) - math.pi
        el_error_rad = target_el - tilt_rad

        return {
            "total_error_rad": total_error_rad,
            "total_error_deg": math.degrees(total_error_rad),
            "total_error_mrad": total_error_rad * 1000.0,
            "azimuth_error_rad": az_error_rad,
            "azimuth_error_deg": math.degrees(az_error_rad),
            "elevation_error_rad": el_error_rad,
            "elevation_error_deg": math.degrees(el_error_rad),
            "slant_range_m": slant_range,
            "target_azimuth_deg": math.degrees(target_az),
            "target_elevation_deg": math.degrees(target_el),
            "camera_pan_deg": math.degrees(pan_rad),
            "camera_tilt_deg": math.degrees(tilt_rad)
        }
