"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: core.disturbances

Comprehensive physical and optical disturbance engine:
- Atmospheric Turbulence (Kolmogorov optical scintillation, beam wander / phase tilt)
- Platform Vibration (multi-harmonic base motion from UAV rotors / satellite momentum wheels)
- Gimbal / Camera Jitter (high-frequency servo torque ripple)
- Radiometric Sensor Noise (Gaussian read noise, Shot noise, Salt-and-Pepper impulse noise)
- Optical Defocus & Motion Blur
- Ambient Background Illumination & Solar Glare
- Temporary Line-of-Sight Occlusions (clouds, structures)
- Frame Drop Jitter
- Standard Presets: OFF, LOW, MEDIUM, HIGH, CUSTOM
"""

import math
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, Any, Tuple, Optional


class DisturbanceLevel(str, Enum):
    OFF = "OFF"
    LOW = "LOW"
    MEDIUM = "MEDIUM"
    HIGH = "HIGH"
    CUSTOM = "CUSTOM"


@dataclass
class DisturbanceConfig:
    """Configurable parameters for environmental and platform disturbances."""
    level: DisturbanceLevel = DisturbanceLevel.LOW

    # Atmospheric Turbulence
    scintillation_index: float = 0.15     # sigma_I^2 (0 = none, 0.15 = weak, 0.4 = moderate, 0.8+ = strong)
    beam_wander_std_deg: float = 0.05    # Angular jitter standard deviation (degrees) from thermal eddies
    turbulence_blur_radius: float = 1.0  # Pixel blur radius from turbulence phase distortion

    # Platform Vibration (rotational perturbations applied to gimbal base)
    vibration_enabled: bool = True
    vibration_amplitude_deg: float = 0.15 # Peak vibration amplitude (deg)
    vibration_freq_1_hz: float = 24.0    # Primary harmonic (e.g. UAV rotor / engine fundamental)
    vibration_freq_2_hz: float = 65.0    # Secondary harmonic (gear mesh / aerodynamic flutter)
    vibration_noise_std_deg: float = 0.03 # Broadband random vibration

    # Camera Gimbal Jitter
    jitter_std_deg: float = 0.02         # Servo quantization and bearing micro-jitter

    # Radiometric Sensor Noise
    gaussian_noise_std: float = 8.0      # Readout noise standard deviation (0-255 scale)
    salt_pepper_ratio: float = 0.001     # Dead / hot pixels probability
    shot_noise_factor: float = 0.05      # Photon shot noise scale

    # Optical Blur
    blur_kernel_size: int = 3            # Gaussian blur kernel size (1, 3, 5, 7)

    # Ambient Light & Glare
    ambient_background_level: float = 25.0 # Background grey level (0-255)
    solar_glare_intensity: float = 0.0   # Background glare flare (0-255)

    # Temporary Occlusion
    occlusion_active: bool = False
    occlusion_start_s: float = 8.0
    occlusion_duration_s: float = 2.5    # Duration target is hidden behind cloud/obstacle

    # Frame Drops
    frame_drop_rate: float = 0.0         # Probability of dropping a camera frame [0.0 - 0.5]


class DisturbanceEngine:
    """
    Simulates real-world aerospace, atmospheric, and sensor disturbances.
    """

    PRESETS: Dict[DisturbanceLevel, Dict[str, Any]] = {
        DisturbanceLevel.OFF: {
            "scintillation_index": 0.0,
            "beam_wander_std_deg": 0.0,
            "turbulence_blur_radius": 0.0,
            "vibration_enabled": False,
            "vibration_amplitude_deg": 0.0,
            "vibration_noise_std_deg": 0.0,
            "jitter_std_deg": 0.0,
            "gaussian_noise_std": 0.0,
            "salt_pepper_ratio": 0.0,
            "shot_noise_factor": 0.0,
            "blur_kernel_size": 1,
            "ambient_background_level": 10.0,
            "solar_glare_intensity": 0.0,
            "occlusion_active": False,
            "frame_drop_rate": 0.0
        },
        DisturbanceLevel.LOW: {
            "scintillation_index": 0.12,
            "beam_wander_std_deg": 0.03,
            "turbulence_blur_radius": 0.8,
            "vibration_enabled": True,
            "vibration_amplitude_deg": 0.08,
            "vibration_noise_std_deg": 0.015,
            "jitter_std_deg": 0.01,
            "gaussian_noise_std": 6.0,
            "salt_pepper_ratio": 0.0005,
            "shot_noise_factor": 0.03,
            "blur_kernel_size": 3,
            "ambient_background_level": 20.0,
            "solar_glare_intensity": 0.0,
            "occlusion_active": False,
            "frame_drop_rate": 0.0
        },
        DisturbanceLevel.MEDIUM: {
            "scintillation_index": 0.35,
            "beam_wander_std_deg": 0.08,
            "turbulence_blur_radius": 1.5,
            "vibration_enabled": True,
            "vibration_amplitude_deg": 0.20,
            "vibration_noise_std_deg": 0.04,
            "jitter_std_deg": 0.025,
            "gaussian_noise_std": 14.0,
            "salt_pepper_ratio": 0.002,
            "shot_noise_factor": 0.07,
            "blur_kernel_size": 3,
            "ambient_background_level": 40.0,
            "solar_glare_intensity": 15.0,
            "occlusion_active": False,
            "frame_drop_rate": 0.02
        },
        DisturbanceLevel.HIGH: {
            "scintillation_index": 0.75,
            "beam_wander_std_deg": 0.18,
            "turbulence_blur_radius": 2.5,
            "vibration_enabled": True,
            "vibration_amplitude_deg": 0.45,
            "vibration_noise_std_deg": 0.09,
            "jitter_std_deg": 0.05,
            "gaussian_noise_std": 25.0,
            "salt_pepper_ratio": 0.006,
            "shot_noise_factor": 0.12,
            "blur_kernel_size": 5,
            "ambient_background_level": 60.0,
            "solar_glare_intensity": 45.0,
            "occlusion_active": False,
            "frame_drop_rate": 0.06
        }
    }

    def __init__(self, config: Optional[DisturbanceConfig] = None, seed: int = 101):
        self.config = config if config is not None else DisturbanceConfig()
        self._rng = np.random.RandomState(seed)
        self.apply_preset(self.config.level)

    def apply_preset(self, level: DisturbanceLevel):
        """Applies parameters from predefined disturbance severity tiers."""
        self.config.level = level
        if level in self.PRESETS:
            for k, v in self.PRESETS[level].items():
                setattr(self.config, k, v)

    def compute_angular_disturbances(self, current_time_s: float) -> Tuple[float, float]:
        """
        Computes the net instantaneous angular displacement (d_pan_rad, d_tilt_rad)
        combining platform vibration, camera jitter, and atmospheric beam wander.
        """
        cfg = self.config
        
        # 1. Platform Multi-harmonic Vibration
        vib_pan_deg = 0.0
        vib_tilt_deg = 0.0
        if cfg.vibration_enabled and cfg.vibration_amplitude_deg > 0.0:
            omega1 = 2.0 * math.pi * cfg.vibration_freq_1_hz
            omega2 = 2.0 * math.pi * cfg.vibration_freq_2_hz
            
            amp1 = cfg.vibration_amplitude_deg * 0.7
            amp2 = cfg.vibration_amplitude_deg * 0.3

            vib_pan_deg = (
                amp1 * math.sin(omega1 * current_time_s) +
                amp2 * math.cos(omega2 * current_time_s + 0.4) +
                self._rng.randn() * cfg.vibration_noise_std_deg
            )
            vib_tilt_deg = (
                amp1 * math.cos(omega1 * current_time_s + 0.8) +
                amp2 * math.sin(omega2 * current_time_s) +
                self._rng.randn() * cfg.vibration_noise_std_deg
            )

        # 2. Atmospheric Beam Wander (stochastic Kolmogorov angle-of-arrival fluctuations)
        wander_pan_deg = self._rng.randn() * cfg.beam_wander_std_deg
        wander_tilt_deg = self._rng.randn() * cfg.beam_wander_std_deg

        # 3. Servo Jitter
        jitter_pan_deg = self._rng.randn() * cfg.jitter_std_deg
        jitter_tilt_deg = self._rng.randn() * cfg.jitter_std_deg

        total_pan_rad = math.radians(vib_pan_deg + wander_pan_deg + jitter_pan_deg)
        total_tilt_rad = math.radians(vib_tilt_deg + wander_tilt_deg + jitter_tilt_deg)
        
        return total_pan_rad, total_tilt_rad

    def compute_scintillation_multiplier(self) -> float:
        """
        Computes optical intensity scintillation factor (I / <I>) under log-normal distribution:
        ln(I) ~ N(-sigma_I^2 / 2, sigma_I^2) such that E[I] = 1.0.
        """
        sigma_sq = self.config.scintillation_index
        if sigma_sq <= 0.001:
            return 1.0
        
        mu_ln = -0.5 * sigma_sq
        sigma_ln = math.sqrt(sigma_sq)
        val = math.exp(self._rng.normal(mu_ln, sigma_ln))
        return float(np.clip(val, 0.05, 5.0))

    def is_occluded(self, current_time_s: float) -> bool:
        """Checks if line of sight is temporarily blocked by an occlusion event."""
        if not self.config.occlusion_active:
            return False
        t_start = self.config.occlusion_start_s
        t_end = t_start + self.config.occlusion_duration_s
        return (t_start <= current_time_s <= t_end)

    def is_frame_dropped(self) -> bool:
        """Determines if current sensor frame is dropped due to packet/buffer loss."""
        if self.config.frame_drop_rate <= 0.001:
            return False
        return bool(self._rng.rand() < self.config.frame_drop_rate)

    def apply_sensor_disturbances(self, raw_frame: np.ndarray) -> np.ndarray:
        """
        Applies radiometric noise, background illumination, blur, and salt-pepper noise
        to a 2D synthetic camera frame (grayscale 0-255 uint8 or float32).
        """
        cfg = self.config
        frame = raw_frame.astype(np.float32)

        # 1. Ambient Background & Solar Glare
        if cfg.ambient_background_level > 0.0:
            frame += cfg.ambient_background_level
        if cfg.solar_glare_intensity > 0.0:
            # Radial sun flare in upper portion of frame
            h, w = frame.shape[:2]
            y, x = np.ogrid[:h, :w]
            flare_dist = np.sqrt((x - w * 0.8)**2 + (y - h * 0.2)**2)
            flare = cfg.solar_glare_intensity * np.exp(-flare_dist / (w * 0.3))
            frame += flare

        # 2. Gaussian Readout Noise
        if cfg.gaussian_noise_std > 0.0:
            noise = self._rng.randn(*frame.shape) * cfg.gaussian_noise_std
            frame += noise

        # 3. Photon Shot Noise
        if cfg.shot_noise_factor > 0.0:
            shot_noise = np.sqrt(np.maximum(frame, 0.0)) * self._rng.randn(*frame.shape) * cfg.shot_noise_factor
            frame += shot_noise

        # 4. Salt and Pepper Noise
        if cfg.salt_pepper_ratio > 0.0:
            mask = self._rng.rand(*frame.shape)
            half_p = cfg.salt_pepper_ratio / 2.0
            frame[mask < half_p] = 0.0
            frame[(mask >= half_p) & (mask < cfg.salt_pepper_ratio)] = 255.0

        # Clip and convert to uint8
        clipped = np.clip(frame, 0.0, 255.0).astype(np.uint8)
        return clipped
