"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: core.targets

Models optical beacons and remote mobile FSOC terminals:
- Optical characteristics: wavelength (nm), optical power (W), beam divergence (mrad), modulation frequency (Hz)
- Designation: Primary target vs Distractor beacons
- 3D Kinematics and dynamic trajectory generation:
  * Stationary
  * Linear / Constant Velocity
  * Accelerating / Decelerating
  * Sinusoidal (harmonic lateral/vertical motion)
  * Circular / Orbiting
  * Smooth Gauss-Markov random walk
  * Waypoint navigation
"""

import math
import numpy as np
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any


class TrajectoryType(str, Enum):
    STATIONARY = "stationary"
    LINEAR = "linear"
    ACCELERATING = "accelerating"
    SINUSOIDAL = "sinusoidal"
    CIRCULAR = "circular"
    SMOOTH_RANDOM = "smooth_random"
    WAYPOINT = "waypoint"


@dataclass
class BeaconOpticalSignature:
    """Optical and spectral signature of an optical beacon."""
    wavelength_nm: float = 850.0        # Emission wavelength (850nm NIR standard)
    optical_power_w: float = 10.0       # Radiated optical peak power (Watts)
    divergence_mrad: float = 15.0       # Full-angle beam divergence (mrad)
    modulation_frequency_hz: float = 20.0 # Blinking / pulse modulation frequency (0 = CW / unmodulated)
    color_rgb: Tuple[int, int, int] = (180, 240, 255) # Visual false-color representation


@dataclass
class Waypoint:
    time_s: float
    position: np.ndarray  # [x, y, z] in meters


class OpticalBeaconTarget:
    """
    Represents a physical optical communication terminal carrying a pointing beacon.
    """

    def __init__(
        self,
        target_id: str,
        is_designated_target: bool = True,
        initial_position: np.ndarray = None,
        initial_velocity: np.ndarray = None,
        initial_acceleration: np.ndarray = None,
        optical_signature: Optional[BeaconOpticalSignature] = None,
        trajectory_type: TrajectoryType = TrajectoryType.LINEAR,
        trajectory_params: Optional[Dict[str, Any]] = None
    ):
        self.target_id = target_id
        self.is_designated = is_designated_target
        
        # 3D Kinematic state in World Frame [East(X), North(Y), Up(Z)]
        self.position = np.array(initial_position if initial_position is not None else [0.0, 500.0, 50.0], dtype=np.float64)
        self.velocity = np.array(initial_velocity if initial_velocity is not None else [5.0, 0.0, 0.0], dtype=np.float64)
        self.acceleration = np.array(initial_acceleration if initial_acceleration is not None else [0.0, 0.0, 0.0], dtype=np.float64)
        
        self.initial_position = self.position.copy()
        self.initial_velocity = self.velocity.copy()
        self.initial_acceleration = self.acceleration.copy()

        self.optical = optical_signature if optical_signature is not None else BeaconOpticalSignature()
        self.trajectory_type = trajectory_type
        self.params = trajectory_params if trajectory_params is not None else {}

        # Internal state for smooth random walk and waypoints
        self._rng = np.random.RandomState(self.params.get("seed", 42))
        self._random_accel = np.zeros(3, dtype=np.float64)
        self.waypoints: List[Waypoint] = []
        if self.trajectory_type == TrajectoryType.WAYPOINT:
            self._init_waypoints()

    def _init_waypoints(self):
        """Constructs default waypoints if none supplied."""
        raw = self.params.get("waypoints", [
            (0.0, [0.0, 500.0, 50.0]),
            (5.0, [40.0, 520.0, 65.0]),
            (10.0, [80.0, 490.0, 55.0]),
            (15.0, [20.0, 460.0, 45.0]),
            (20.0, [-30.0, 510.0, 60.0]),
            (25.0, [0.0, 500.0, 50.0])
        ])
        self.waypoints = [Waypoint(time_s=t, position=np.array(p, dtype=np.float64)) for t, p in raw]

    def reset(self):
        """Resets target kinematics to initial values."""
        self.position = self.initial_position.copy()
        self.velocity = self.initial_velocity.copy()
        self.acceleration = self.initial_acceleration.copy()
        self._random_accel[:] = 0.0

    def update(self, dt: float, current_time_s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Advances target kinematics by dt seconds according to trajectory mode.
        Returns: (position, velocity, acceleration) in World coordinates.
        """
        if self.trajectory_type == TrajectoryType.STATIONARY:
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0

        elif self.trajectory_type == TrajectoryType.LINEAR:
            self.acceleration[:] = 0.0
            self.position += self.velocity * dt

        elif self.trajectory_type == TrajectoryType.ACCELERATING:
            max_vel = self.params.get("max_speed", 25.0)
            const_accel = np.array(self.params.get("accel", [0.8, -0.2, 0.1]), dtype=np.float64)
            self.acceleration = const_accel
            self.velocity += self.acceleration * dt
            # Speed clamping
            speed = np.linalg.norm(self.velocity)
            if speed > max_vel:
                self.velocity = (self.velocity / speed) * max_vel
            self.position += self.velocity * dt

        elif self.trajectory_type == TrajectoryType.SINUSOIDAL:
            # Weaving motion: e.g. UAV executing searching weave or atmospheric drift
            amp_x = self.params.get("amplitude_x", 30.0)
            freq_x = self.params.get("frequency_x", 0.2) # Hz
            amp_z = self.params.get("amplitude_z", 10.0)
            freq_z = self.params.get("frequency_z", 0.1) # Hz
            base_vel_y = self.params.get("vel_y", 4.0)

            omega_x = 2.0 * math.pi * freq_x
            omega_z = 2.0 * math.pi * freq_z

            self.position[0] = self.initial_position[0] + amp_x * math.sin(omega_x * current_time_s)
            self.position[1] = self.initial_position[1] + base_vel_y * current_time_s
            self.position[2] = self.initial_position[2] + amp_z * math.sin(omega_z * current_time_s)

            self.velocity[0] = amp_x * omega_x * math.cos(omega_x * current_time_s)
            self.velocity[1] = base_vel_y
            self.velocity[2] = amp_z * omega_z * math.cos(omega_z * current_time_s)

            self.acceleration[0] = -amp_x * (omega_x ** 2) * math.sin(omega_x * current_time_s)
            self.acceleration[1] = 0.0
            self.acceleration[2] = -amp_z * (omega_z ** 2) * math.sin(omega_z * current_time_s)

        elif self.trajectory_type == TrajectoryType.CIRCULAR:
            # Orbiting / loitering aircraft: circular path around center
            center = np.array(self.params.get("center", [0.0, 500.0, 50.0]), dtype=np.float64)
            radius = self.params.get("radius", 60.0)
            period = self.params.get("period", 20.0) # seconds per revolution
            omega = (2.0 * math.pi) / period

            angle = omega * current_time_s
            self.position[0] = center[0] + radius * math.cos(angle)
            self.position[1] = center[1] + radius * math.sin(angle)
            self.position[2] = center[2] + 5.0 * math.sin(0.5 * angle)

            self.velocity[0] = -radius * omega * math.sin(angle)
            self.velocity[1] = radius * omega * math.cos(angle)
            self.velocity[2] = 2.5 * omega * math.cos(0.5 * angle)

            self.acceleration[0] = -radius * (omega ** 2) * math.cos(angle)
            self.acceleration[1] = -radius * (omega ** 2) * math.sin(angle)
            self.acceleration[2] = -1.25 * (omega ** 2) * math.sin(0.5 * angle)

        elif self.trajectory_type == TrajectoryType.SMOOTH_RANDOM:
            # 1st-order Gauss-Markov stochastic acceleration (turbulent flight dynamics)
            tau = self.params.get("correlation_time", 2.0) # correlation time constant (s)
            sigma_a = self.params.get("sigma_accel", 1.5) # m/s^2 acceleration standard deviation
            
            beta = 1.0 / tau
            noise = self._rng.randn(3) * sigma_a * math.sqrt(2.0 * beta * dt)
            self._random_accel += (-beta * self._random_accel * dt) + noise
            
            self.acceleration = self._random_accel.copy()
            self.velocity += self.acceleration * dt
            # Bound velocity
            speed = np.linalg.norm(self.velocity)
            if speed > 20.0:
                self.velocity = (self.velocity / speed) * 20.0
            self.position += self.velocity * dt

        elif self.trajectory_type == TrajectoryType.WAYPOINT:
            if not self.waypoints:
                self._init_waypoints()
            total_duration = self.waypoints[-1].time_s
            mod_time = current_time_s % total_duration
            
            # Find interval
            p0, p1 = self.waypoints[0].position, self.waypoints[-1].position
            t0, t1 = 0.0, total_duration
            for i in range(len(self.waypoints) - 1):
                if self.waypoints[i].time_s <= mod_time <= self.waypoints[i+1].time_s:
                    t0 = self.waypoints[i].time_s
                    t1 = self.waypoints[i+1].time_s
                    p0 = self.waypoints[i].position
                    p1 = self.waypoints[i+1].position
                    break
            alpha = (mod_time - t0) / max(1e-4, t1 - t0)
            # Smooth cubic Hermite interpolation
            s = 3.0 * (alpha ** 2) - 2.0 * (alpha ** 3)
            self.position = p0 + (p1 - p0) * s
            ds = 6.0 * alpha - 6.0 * (alpha ** 2)
            self.velocity = ((p1 - p0) / max(1e-4, t1 - t0)) * ds
            self.acceleration[:] = 0.0

        return self.position.copy(), self.velocity.copy(), self.acceleration.copy()

    def get_instantaneous_intensity_factor(self, current_time_s: float) -> float:
        """
        Computes the modulation factor [0.0, 1.0] at current time.
        If modulation_frequency_hz > 0: square wave pulse modulation (active optical beacon).
        """
        freq = self.optical.modulation_frequency_hz
        if freq <= 0.01:
            return 1.0  # Continuous Wave (CW)
        
        # 50% duty cycle square wave modulation
        phase = (current_time_s * freq) % 1.0
        return 1.0 if phase < 0.5 else 0.08  # 8% baseline off-state extinction ratio


class BeaconTargetManager:
    """
    Manages all optical beacons in the simulation, including designated target and distractors.
    """

    def __init__(self):
        self.targets: List[OpticalBeaconTarget] = []
        self.designated_target: Optional[OpticalBeaconTarget] = None

    def clear(self):
        self.targets.clear()
        self.designated_target = None

    def add_target(self, target: OpticalBeaconTarget):
        self.targets.append(target)
        if target.is_designated:
            self.designated_target = target

    def reset_all(self):
        for t in self.targets:
            t.reset()

    def update_all(self, dt: float, current_time_s: float):
        for t in self.targets:
            t.update(dt, current_time_s)

    def get_designated(self) -> Optional[OpticalBeaconTarget]:
        return self.designated_target

    def get_all(self) -> List[OpticalBeaconTarget]:
        return list(self.targets)
