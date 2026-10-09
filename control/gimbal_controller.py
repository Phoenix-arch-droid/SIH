"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: control.gimbal_controller

2-Axis Pan/Tilt Gimbal Controller:
- Discrete-time PID controller with anti-windup clamping
- Low-pass filtered derivative action (noise immune)
- Realistic physical actuator dynamics:
  * Maximum angular velocity / slew rate limit (deg/s)
  * Maximum angular acceleration limit (deg/s^2)
  * Mechanical deadband (prevents limit cycling / servo jitter)
  * Motor inertia and 1st-order response lag
- Feedforward velocity compensation
- Strict isolation: Operates solely on tracker estimates; NEVER ground truth.
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Dict, Any, Optional


@dataclass
class GimbalLimits:
    """Mechanical and electro-mechanical limits for the FSOC tracking gimbal."""
    pan_min_deg: float = -180.0
    pan_max_deg: float = 180.0
    tilt_min_deg: float = -60.0
    tilt_max_deg: float = 85.0
    max_velocity_deg_s: float = 35.0      # Maximum gimbal slew rate (deg/s)
    max_accel_deg_s2: float = 120.0       # Maximum motor acceleration (deg/s^2)
    deadband_deg: float = 0.02            # Servo deadband (deg) ~0.35 mrad
    motor_time_constant_s: float = 0.025  # Actuator electro-mechanical lag tau (s)


@dataclass
class PIDGains:
    kp: float = 3.5                       # Proportional gain
    ki: float = 0.8                       # Integral gain
    kd: float = 0.25                      # Derivative gain
    feedforward_gain: float = 0.9         # Velocity feedforward gain
    integral_limit_deg: float = 5.0       # Anti-windup saturation limit
    derivative_lpf_alpha: float = 0.2     # Derivative low-pass filter coefficient


class AxisPIDController:
    """Discrete PID controller for a single gimbal axis (Pan or Tilt)."""

    def __init__(self, gains: PIDGains, deadband_deg: float = 0.02):
        self.gains = gains
        self.deadband_deg = deadband_deg

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_filtered_deriv = 0.0
        self.last_cmd = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_filtered_deriv = 0.0
        self.last_cmd = 0.0

    def compute(
        self,
        error_deg: float,
        target_velocity_deg_s: float,
        dt: float
    ) -> float:
        """
        Computes the commanded angular rate (deg/s) for this axis.
        """
        if dt <= 1e-4:
            return 0.0

        # Apply deadband
        if abs(error_deg) < self.deadband_deg:
            error_deg = 0.0

        # Proportional term
        P = self.gains.kp * error_deg

        # Integral term with anti-windup clamping
        self.integral += error_deg * dt
        self.integral = float(np.clip(
            self.integral,
            -self.gains.integral_limit_deg,
            self.gains.integral_limit_deg
        ))
        I = self.gains.ki * self.integral

        # Filtered Derivative term: dE/dt
        raw_deriv = (error_deg - self.prev_error) / dt
        alpha = self.gains.derivative_lpf_alpha
        filtered_deriv = alpha * raw_deriv + (1.0 - alpha) * self.prev_filtered_deriv
        self.prev_filtered_deriv = filtered_deriv
        self.prev_error = error_deg
        D = self.gains.kd * filtered_deriv

        # Feedforward term
        FF = self.gains.feedforward_gain * target_velocity_deg_s

        cmd_rate = P + I + D + FF
        self.last_cmd = cmd_rate
        return cmd_rate


class PanTiltGimbalController:
    """
    Simulates the physical closed-loop 2-axis gimbal assembly.
    """

    def __init__(
        self,
        limits: Optional[GimbalLimits] = None,
        pan_gains: Optional[PIDGains] = None,
        tilt_gains: Optional[PIDGains] = None,
        initial_pan_deg: float = 0.0,
        initial_tilt_deg: float = 0.0
    ):
        self.limits = limits if limits is not None else GimbalLimits()
        self.pan_pid = AxisPIDController(
            pan_gains if pan_gains is not None else PIDGains(kp=3.2, ki=0.7, kd=0.22),
            deadband_deg=self.limits.deadband_deg
        )
        self.tilt_pid = AxisPIDController(
            tilt_gains if tilt_gains is not None else PIDGains(kp=3.6, ki=0.8, kd=0.25),
            deadband_deg=self.limits.deadband_deg
        )

        # Actual physical angles
        self.pan_deg: float = initial_pan_deg
        self.tilt_deg: float = initial_tilt_deg

        # Actual velocities
        self.pan_vel_deg_s: float = 0.0
        self.tilt_vel_deg_s: float = 0.0

        # Commanded values (telemetry)
        self.commanded_pan_vel: float = 0.0
        self.commanded_tilt_vel: float = 0.0
        self.last_pan_error_deg: float = 0.0
        self.last_tilt_error_deg: float = 0.0

    def reset(self, initial_pan_deg: float = 0.0, initial_tilt_deg: float = 0.0):
        self.pan_deg = initial_pan_deg
        self.tilt_deg = initial_tilt_deg
        self.pan_vel_deg_s = 0.0
        self.tilt_vel_deg_s = 0.0
        self.commanded_pan_vel = 0.0
        self.commanded_tilt_vel = 0.0
        self.pan_pid.reset()
        self.tilt_pid.reset()

    def update_closed_loop(
        self,
        estimated_angular_error_pan_deg: float,
        estimated_angular_error_tilt_deg: float,
        estimated_target_rate_pan_deg_s: float,
        estimated_target_rate_tilt_deg_s: float,
        dt: float
    ) -> Tuple[float, float, float, float]:
        """
        Advances the physical gimbal closed-loop dynamics.
        
        Args:
            estimated_angular_error_pan_deg: Error from tracker to boresight (deg)
            estimated_angular_error_tilt_deg: Error from tracker to boresight (deg)
            estimated_target_rate_pan_deg_s: Estimated angular velocity of target
            estimated_target_rate_tilt_deg_s: Estimated angular velocity of target
            dt: Time increment (s)
            
        Returns:
            (current_pan_rad, current_tilt_rad, actual_pan_vel_deg_s, actual_tilt_vel_deg_s)
        """
        dt = max(1e-4, dt)
        self.last_pan_error_deg = estimated_angular_error_pan_deg
        self.last_tilt_error_deg = estimated_angular_error_tilt_deg

        # 1. Compute PID commanded angular rates
        cmd_pan_rate = self.pan_pid.compute(
            estimated_angular_error_pan_deg,
            estimated_target_rate_pan_deg_s,
            dt
        )
        cmd_tilt_rate = self.tilt_pid.compute(
            estimated_angular_error_tilt_deg,
            estimated_target_rate_tilt_deg_s,
            dt
        )

        # 2. Velocity limit clamping
        v_max = self.limits.max_velocity_deg_s
        self.commanded_pan_vel = float(np.clip(cmd_pan_rate, -v_max, v_max))
        self.commanded_tilt_vel = float(np.clip(cmd_tilt_rate, -v_max, v_max))

        # 3. Acceleration limits and motor lag (1st order filter)
        a_max = self.limits.max_accel_deg_s2
        tau = max(1e-3, self.limits.motor_time_constant_s)

        # Pan motor response
        target_pan_accel = (self.commanded_pan_vel - self.pan_vel_deg_s) / tau
        actual_pan_accel = float(np.clip(target_pan_accel, -a_max, a_max))
        self.pan_vel_deg_s += actual_pan_accel * dt
        self.pan_vel_deg_s = float(np.clip(self.pan_vel_deg_s, -v_max, v_max))

        # Tilt motor response
        target_tilt_accel = (self.commanded_tilt_vel - self.tilt_vel_deg_s) / tau
        actual_tilt_accel = float(np.clip(target_tilt_accel, -a_max, a_max))
        self.tilt_vel_deg_s += actual_tilt_accel * dt
        self.tilt_vel_deg_s = float(np.clip(self.tilt_vel_deg_s, -v_max, v_max))

        # 4. Integrate positions
        self.pan_deg += self.pan_vel_deg_s * dt
        self.tilt_deg += self.tilt_vel_deg_s * dt

        # 5. Position limit saturation
        self.pan_deg = float(np.clip(self.pan_deg, self.limits.pan_min_deg, self.limits.pan_max_deg))
        self.tilt_deg = float(np.clip(self.tilt_deg, self.limits.tilt_min_deg, self.limits.tilt_max_deg))

        return (
            math.radians(self.pan_deg),
            math.radians(self.tilt_deg),
            self.pan_vel_deg_s,
            self.tilt_vel_deg_s
        )

    def set_open_loop_rate(self, pan_rate_deg_s: float, tilt_rate_deg_s: float, dt: float):
        """Used during autonomous search scan patterns."""
        v_max = self.limits.max_velocity_deg_s
        self.pan_vel_deg_s = float(np.clip(pan_rate_deg_s, -v_max, v_max))
        self.tilt_vel_deg_s = float(np.clip(tilt_rate_deg_s, -v_max, v_max))

        self.pan_deg += self.pan_vel_deg_s * dt
        self.tilt_deg += self.tilt_vel_deg_s * dt

        self.pan_deg = float(np.clip(self.pan_deg, self.limits.pan_min_deg, self.limits.pan_max_deg))
        self.tilt_deg = float(np.clip(self.tilt_deg, self.limits.tilt_min_deg, self.limits.tilt_max_deg))
