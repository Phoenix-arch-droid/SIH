"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: simulation.scenarios

14 Authoritative Standard Evaluation Scenarios:
1. Static Target
2. Slow Target
3. Fast Target
4. Accelerating Target
5. Crossing Target
6. Multiple Targets (Designated Beacon + Distractors)
7. Sensor Noise
8. Platform Vibration
9. Atmospheric Turbulence / Scintillation
10. Camera / Platform Base Motion
11. Temporary Line-of-Sight Occlusion
12. Target Loss & Predictive Reacquisition
13. Combined Multi-Disturbance
14. Extreme Stress Test
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
from core.targets import TrajectoryType, BeaconOpticalSignature, OpticalBeaconTarget
from core.disturbances import DisturbanceLevel, DisturbanceConfig


@dataclass
class ScenarioDefinition:
    scenario_id: int
    name: str
    description: str
    duration_s: float
    seed: int
    initial_camera_pan_deg: float = 0.0
    initial_camera_tilt_deg: float = 0.0
    platform_motion_enabled: bool = False
    targets_config: List[Dict[str, Any]] = field(default_factory=list)
    disturbance_preset: DisturbanceLevel = DisturbanceLevel.LOW
    custom_disturbance: Optional[Dict[str, Any]] = None


class ScenarioCatalog:
    """Provides the library of 14 standard reproducible scenarios."""

    @staticmethod
    def get_all_scenarios() -> List[ScenarioDefinition]:
        return [
            ScenarioCatalog.scenario_1_static_target(),
            ScenarioCatalog.scenario_2_slow_target(),
            ScenarioCatalog.scenario_3_fast_target(),
            ScenarioCatalog.scenario_4_accelerating_target(),
            ScenarioCatalog.scenario_5_crossing_target(),
            ScenarioCatalog.scenario_6_multiple_targets(),
            ScenarioCatalog.scenario_7_sensor_noise(),
            ScenarioCatalog.scenario_8_platform_vibration(),
            ScenarioCatalog.scenario_9_atmospheric_turbulence(),
            ScenarioCatalog.scenario_10_camera_motion(),
            ScenarioCatalog.scenario_11_temporary_occlusion(),
            ScenarioCatalog.scenario_12_loss_and_reacquisition(),
            ScenarioCatalog.scenario_13_combined_disturbance(),
            ScenarioCatalog.scenario_14_stress_test()
        ]

    @staticmethod
    def get_by_id(scenario_id: int) -> ScenarioDefinition:
        for s in ScenarioCatalog.get_all_scenarios():
            if s.scenario_id == scenario_id:
                return s
        return ScenarioCatalog.scenario_1_static_target()

    # 1. Static Target
    @staticmethod
    def scenario_1_static_target() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=1,
            name="Static Target Baseline",
            description="Stationary optical terminal at 500m slant range; benchmarks coarse alignment settling time and steady-state pointing error.",
            duration_s=15.0,
            seed=101,
            initial_camera_pan_deg=2.0,
            initial_camera_tilt_deg=3.5,
            disturbance_preset=DisturbanceLevel.OFF,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [0.0, 500.0, 45.0],
                "velocity": [0.0, 0.0, 0.0],
                "acceleration": [0.0, 0.0, 0.0],
                "trajectory": TrajectoryType.STATIONARY,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=10.0, modulation_frequency_hz=20.0)
            }]
        )

    # 2. Slow Target
    @staticmethod
    def scenario_2_slow_target() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=2,
            name="Slow Target Tracking",
            description="Low-velocity drifting terminal (2.5 m/s); evaluates smooth velocity tracking and steady error convergence.",
            duration_s=18.0,
            seed=102,
            initial_camera_pan_deg=0.0,
            initial_camera_tilt_deg=0.0,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-20.0, 550.0, 50.0],
                "velocity": [2.5, 0.5, 0.2],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 3. Fast Target
    @staticmethod
    def scenario_3_fast_target() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=3,
            name="Fast Target Pursuit",
            description="High-velocity airborne FSOC terminal (15.0 m/s); stress tests maximum gimbal slew rate and velocity feedforward.",
            duration_s=18.0,
            seed=103,
            initial_camera_pan_deg=-4.0,
            initial_camera_tilt_deg=1.0,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-60.0, 450.0, 70.0],
                "velocity": [15.0, -1.0, 1.5],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=15.0, modulation_frequency_hz=20.0)
            }]
        )

    # 4. Accelerating Target
    @staticmethod
    def scenario_4_accelerating_target() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=4,
            name="Accelerating Target Maneuver",
            description="Target undergoing dynamic acceleration (up to 2.2 m/s^2); tests 2nd-order Kalman filter acceleration tracking.",
            duration_s=20.0,
            seed=104,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-30.0, 500.0, 40.0],
                "velocity": [2.0, 0.0, 0.5],
                "trajectory": TrajectoryType.ACCELERATING,
                "params": {"accel": [1.5, -0.2, 0.3], "max_speed": 22.0},
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=14.0, modulation_frequency_hz=20.0)
            }]
        )

    # 5. Crossing Target
    @staticmethod
    def scenario_5_crossing_target() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=5,
            name="Lateral Crossing Fly-By",
            description="Close-range (300m) high angular-rate lateral fly-by across camera boresight; tests dynamic FOV transit.",
            duration_s=18.0,
            seed=105,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-80.0, 320.0, 45.0],
                "velocity": [12.0, 2.0, -0.5],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 6. Multiple Targets + Distractors
    @staticmethod
    def scenario_6_multiple_targets() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=6,
            name="Multiple Targets & Distractor Discrimination",
            description="Primary 850nm (20Hz) optical beacon alongside two distractor beacons (unmodulated glint and 5Hz distractor); evaluates AI candidate discrimination.",
            duration_s=20.0,
            seed=106,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[
                {
                    "id": "BEACON_PRIMARY",
                    "is_designated": True,
                    "position": [-10.0, 500.0, 50.0],
                    "velocity": [4.0, 0.0, 0.0],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0, color_rgb=(180, 240, 255))
                },
                {
                    "id": "DISTRACTOR_1_GLINT",
                    "is_designated": False,
                    "position": [15.0, 520.0, 60.0],
                    "velocity": [1.0, 0.0, -0.2],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=650.0, optical_power_w=16.0, modulation_frequency_hz=0.0, color_rgb=(255, 200, 100)) # Unmodulated brighter glint
                },
                {
                    "id": "DISTRACTOR_2_FALSE_BEACON",
                    "is_designated": False,
                    "position": [-40.0, 480.0, 35.0],
                    "velocity": [5.0, 0.0, 0.3],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=1064.0, optical_power_w=10.0, modulation_frequency_hz=5.0, color_rgb=(255, 120, 120)) # 5Hz false beacon
                }
            ]
        )

    # 7. Sensor Noise
    @staticmethod
    def scenario_7_sensor_noise() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=7,
            name="Severe Sensor Noise & Hot Pixels",
            description="Elevated Gaussian sensor readout noise (std=22.0) and salt-and-pepper impulse hot-pixels; tests morphological and statistical noise filtering.",
            duration_s=18.0,
            seed=107,
            disturbance_preset=DisturbanceLevel.CUSTOM,
            custom_disturbance={
                "gaussian_noise_std": 22.0,
                "salt_pepper_ratio": 0.005,
                "shot_noise_factor": 0.10,
                "ambient_background_level": 35.0,
                "blur_kernel_size": 3
            },
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-15.0, 520.0, 50.0],
                "velocity": [3.5, 0.0, 0.2],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=10.0, modulation_frequency_hz=20.0)
            }]
        )

    # 8. Platform Vibration
    @staticmethod
    def scenario_8_platform_vibration() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=8,
            name="Severe Host Platform Vibration",
            description="Multi-harmonic UAV rotor vibration (24 Hz and 65 Hz) applied directly to gimbal base; evaluates servo disturbance rejection and deadband filtering.",
            duration_s=18.0,
            seed=108,
            disturbance_preset=DisturbanceLevel.CUSTOM,
            custom_disturbance={
                "vibration_enabled": True,
                "vibration_amplitude_deg": 0.35,
                "vibration_freq_1_hz": 24.0,
                "vibration_freq_2_hz": 65.0,
                "vibration_noise_std_deg": 0.06,
                "jitter_std_deg": 0.03
            },
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [0.0, 500.0, 48.0],
                "velocity": [4.0, 0.0, 0.1],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 9. Atmospheric Turbulence
    @staticmethod
    def scenario_9_atmospheric_turbulence() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=9,
            name="Atmospheric Scintillation & Beam Wander",
            description="Strong Kolmogorov atmospheric optical turbulence (scintillation index sigma_I^2 = 0.65, beam wander = 0.15 deg); evaluates AI PSF profile correlation under severe intensity fading.",
            duration_s=20.0,
            seed=109,
            disturbance_preset=DisturbanceLevel.CUSTOM,
            custom_disturbance={
                "scintillation_index": 0.65,
                "beam_wander_std_deg": 0.15,
                "turbulence_blur_radius": 2.2,
                "ambient_background_level": 30.0
            },
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-20.0, 600.0, 55.0],
                "velocity": [3.0, 0.0, 0.0],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 10. Camera / Base Motion
    @staticmethod
    def scenario_10_camera_motion() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=10,
            name="Dynamic Mobile Base Motion",
            description="Continuous attitude oscillations (roll, pitch, yaw) of the host vehicle/platform; validates coordinate transformation compensation.",
            duration_s=20.0,
            seed=110,
            platform_motion_enabled=True,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [10.0, 520.0, 50.0],
                "velocity": [3.0, 0.5, 0.1],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 11. Temporary Occlusion
    @staticmethod
    def scenario_11_temporary_occlusion() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=11,
            name="Temporary Line-of-Sight Occlusion",
            description="Optical beacon completely obscured by cloud/structure for 2.0s (t=2.5s to 4.5s); tests Kalman dead-reckoning forward projection and seamless reacquisition upon re-emergence.",
            duration_s=8.0,
            seed=111,
            initial_camera_pan_deg=-3.0,
            initial_camera_tilt_deg=5.0,
            disturbance_preset=DisturbanceLevel.CUSTOM,
            custom_disturbance={
                "occlusion_active": True,
                "occlusion_start_s": 2.5,
                "occlusion_duration_s": 2.0
            },
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [-30.0, 500.0, 45.0],
                "velocity": [4.0, 0.0, 0.0],
                "trajectory": TrajectoryType.LINEAR,
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=14.0, modulation_frequency_hz=20.0)
            }]
        )

    # 12. Loss & Reacquisition
    @staticmethod
    def scenario_12_loss_and_reacquisition() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=12,
            name="Target Loss & Autonomous Reacquisition",
            description="Target executes high-rate sinusoidal weaving that briefly escapes camera FOV; tests autonomous transition through LOST, REACQUIRING local spiral, and re-LOCK.",
            duration_s=22.0,
            seed=112,
            disturbance_preset=DisturbanceLevel.LOW,
            targets_config=[{
                "id": "BEACON_PRIMARY",
                "is_designated": True,
                "position": [0.0, 480.0, 50.0],
                "velocity": [6.0, 0.0, 0.0],
                "trajectory": TrajectoryType.SINUSOIDAL,
                "params": {"amplitude_x": 45.0, "frequency_x": 0.18, "amplitude_z": 15.0, "frequency_z": 0.12},
                "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
            }]
        )

    # 13. Combined Disturbance
    @staticmethod
    def scenario_13_combined_disturbance() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=13,
            name="Combined Atmospheric & Platform Disturbances",
            description="Simultaneous turbulence (scintillation sigma_I^2=0.45) + platform vibration (0.25 deg) + sensor noise + optical distractor; representative operational environment.",
            duration_s=22.0,
            seed=113,
            disturbance_preset=DisturbanceLevel.MEDIUM,
            targets_config=[
                {
                    "id": "BEACON_PRIMARY",
                    "is_designated": True,
                    "position": [-25.0, 520.0, 50.0],
                    "velocity": [5.0, 0.0, 0.2],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=12.0, modulation_frequency_hz=20.0)
                },
                {
                    "id": "DISTRACTOR_1",
                    "is_designated": False,
                    "position": [20.0, 500.0, 55.0],
                    "velocity": [1.5, 0.0, -0.1],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=650.0, optical_power_w=14.0, modulation_frequency_hz=0.0)
                }
            ]
        )

    # 14. Stress Test
    @staticmethod
    def scenario_14_stress_test() -> ScenarioDefinition:
        return ScenarioDefinition(
            scenario_id=14,
            name="Maximum Operational Stress Test",
            description="Extreme turbulence (sigma_I^2=0.75), severe vibration (0.45 deg), 14 m/s accelerating circular target trajectory, occlusions, and clutter.",
            duration_s=25.0,
            seed=114,
            disturbance_preset=DisturbanceLevel.HIGH,
            custom_disturbance={
                "occlusion_active": True,
                "occlusion_start_s": 10.0,
                "occlusion_duration_s": 2.0,
                "frame_drop_rate": 0.05
            },
            targets_config=[
                {
                    "id": "BEACON_PRIMARY",
                    "is_designated": True,
                    "position": [0.0, 480.0, 50.0],
                    "velocity": [8.0, 0.0, 0.0],
                    "trajectory": TrajectoryType.CIRCULAR,
                    "params": {"center": [0.0, 480.0, 50.0], "radius": 55.0, "period": 18.0},
                    "optical": BeaconOpticalSignature(wavelength_nm=850.0, optical_power_w=14.0, modulation_frequency_hz=20.0)
                },
                {
                    "id": "DISTRACTOR_FLARE",
                    "is_designated": False,
                    "position": [-30.0, 460.0, 40.0],
                    "velocity": [3.0, 0.0, 0.0],
                    "trajectory": TrajectoryType.LINEAR,
                    "optical": BeaconOpticalSignature(wavelength_nm=650.0, optical_power_w=18.0, modulation_frequency_hz=0.0)
                }
            ]
        )
