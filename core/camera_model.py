"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: core.camera_model

Pinhole and radiometric camera model representing the coarse-alignment acquisition sensor:
- Configurable sensor resolution (width, height)
- Horizontal & Vertical Field of View (HFOV, VFOV)
- Intrinsic matrix K (focal length fx, fy, principal point cx, cy)
- Lens distortion (radial k1, k2)
- 3D-to-2D projection with in-FOV validation
- Inverse 2D-to-Ray back-projection
- Radiometric irradiance & Point Spread Function (PSF) intensity rendering
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional, Dict, Any, List


@dataclass
class CameraIntrinsics:
    """Optical and sensor specifications for the FSOC tracking camera."""
    width: int = 1280                 # Sensor horizontal resolution (pixels)
    height: int = 720                 # Sensor vertical resolution (pixels)
    hfov_deg: float = 15.0            # Horizontal Field of View (degrees) - Engineering standard for FSOC coarse alignment
    pixel_pitch_um: float = 3.45      # Sensor physical pixel pitch (micrometers)
    aperture_diameter_mm: float = 50.0# Optical aperture (mm)
    wavelength_nm: float = 850.0      # Optical beacon nominal wavelength (nm) - standard FSOC beacon
    k1: float = 0.0                   # Radial distortion parameter
    k2: float = 0.0                   # Second order radial distortion

    # Derived values
    fx: float = field(init=False)
    fy: float = field(init=False)
    cx: float = field(init=False)
    cy: float = field(init=False)
    vfov_deg: float = field(init=False)

    def __post_init__(self):
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        half_hfov_rad = math.radians(self.hfov_deg / 2.0)
        self.fx = self.cx / math.tan(half_hfov_rad)
        # Assuming square pixels
        self.fy = self.fx
        half_vfov_rad = math.atan(self.cy / self.fy)
        self.vfov_deg = math.degrees(half_vfov_rad * 2.0)

    @property
    def K(self) -> np.ndarray:
        """3x3 Intrinsic calibration matrix."""
        return np.array([
            [self.fx, 0.0,     self.cx],
            [0.0,     self.fy, self.cy],
            [0.0,     0.0,     1.0    ]
        ], dtype=np.float64)


class VirtualCamera:
    """
    Simulates the physical optical camera mounted on the 2-axis gimbal of the FSOC terminal.
    """

    def __init__(self, intrinsics: Optional[CameraIntrinsics] = None):
        self.intrinsics = intrinsics if intrinsics is not None else CameraIntrinsics()
        self.exposure_time_ms: float = 10.0
        self.gain_db: float = 0.0
        self.atmospheric_extinction_coeff: float = 0.0001 # 1/m (clear air)

    def project_point(self, point_camera: np.ndarray) -> Tuple[Optional[np.ndarray], bool, Dict[str, Any]]:
        """
        Projects a 3D point in camera frame [xc, yc, zc] to image plane [u, v].
        
        Returns:
            pixel_coord: np.ndarray [u, v] in pixels, or None if behind camera
            is_visible: True if point is inside active sensor FOV and zc > 0.1m
            metadata: Diagnostic dictionary
        """
        xc, yc, zc = float(point_camera[0]), float(point_camera[1]), float(point_camera[2])

        if zc <= 0.1:  # Behind camera or nearer than near-clipping plane
            return None, False, {
                "in_front": False,
                "in_fov": False,
                "range_zc": zc,
                "pixel_u": None,
                "pixel_v": None
            }

        # Normalized coordinates
        xn = xc / zc
        yn = yc / zc

        # Radial distortion
        r2 = xn * xn + yn * yn
        radial_mult = 1.0 + self.intrinsics.k1 * r2 + self.intrinsics.k2 * (r2 * r2)
        xd = xn * radial_mult
        yd = yn * radial_mult

        # Pixel coordinates
        u = self.intrinsics.fx * xd + self.intrinsics.cx
        v = self.intrinsics.fy * yd + self.intrinsics.cy

        is_in_bounds = (0.0 <= u < self.intrinsics.width) and (0.0 <= v < self.intrinsics.height)

        return np.array([u, v], dtype=np.float64), is_in_bounds, {
            "in_front": True,
            "in_fov": is_in_bounds,
            "range_zc": zc,
            "pixel_u": u,
            "pixel_v": v,
            "image_space_error_x": u - self.intrinsics.cx,
            "image_space_error_y": v - self.intrinsics.cy
        }

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """
        Back-projects a 2D pixel coordinate to a unit line-of-sight ray in camera frame.
        """
        xn = (u - self.intrinsics.cx) / self.intrinsics.fx
        yn = (v - self.intrinsics.cy) / self.intrinsics.fy
        ray = np.array([xn, yn, 1.0], dtype=np.float64)
        return ray / np.linalg.norm(ray)

    def pixel_error_to_angular_offset(self, u: float, v: float) -> Tuple[float, float]:
        """
        Converts pixel position relative to optical center into angular offsets (rad):
        (delta_azimuth, delta_elevation)
        """
        du = u - self.intrinsics.cx
        dv = v - self.intrinsics.cy
        
        # In camera frame: +X is right (negative pan needed to center), +Y is down (negative tilt needed to center)
        delta_az_rad = -math.atan2(du, self.intrinsics.fx)
        delta_el_rad = -math.atan2(dv, self.intrinsics.fy)
        return delta_az_rad, delta_el_rad

    def calculate_apparent_beacon_irradiance(
        self,
        beacon_source_power_w: float,
        beacon_divergence_mrad: float,
        slant_range_m: float
    ) -> float:
        """
        Calculates received beacon optical power density (W/m^2) at camera aperture:
        Beam area at range R: A_beam = pi * (R * tan(theta_div / 2))^2
        Beer-Lambert atmospheric transmission: T_atm = exp(-gamma * R)
        """
        if slant_range_m <= 1.0:
            return 1.0
        
        theta_div_rad = beacon_divergence_mrad * 1e-3
        beam_radius_at_target = max(0.05, slant_range_m * math.tan(theta_div_rad / 2.0))
        beam_area = math.pi * (beam_radius_at_target ** 2)
        
        # Geometric irradiance
        irradiance_geom = beacon_source_power_w / beam_area
        # Atmospheric absorption / scattering
        transmittance = math.exp(-self.atmospheric_extinction_coeff * slant_range_m)
        
        return irradiance_geom * transmittance

    def calculate_psf_sigma_pixels(self, slant_range_m: float) -> float:
        """
        Diffraction-limited optical spot size (Airy disk radius approximated as Gaussian sigma):
        theta_diffraction = 1.22 * lambda / D_aperture
        spot_size_sensor = focal_length * theta_diffraction
        sigma_pix = spot_size_sensor / pixel_pitch
        """
        wavelength_m = self.intrinsics.wavelength_nm * 1e-9
        aperture_m = self.intrinsics.aperture_diameter_mm * 1e-3
        theta_diff = 1.22 * wavelength_m / aperture_m
        
        focal_length_m = (self.intrinsics.fx * self.intrinsics.pixel_pitch_um) * 1e-6
        airy_radius_m = focal_length_m * theta_diff
        sigma_pix = max(1.2, airy_radius_m / (self.intrinsics.pixel_pitch_um * 1e-6))
        return sigma_pix
