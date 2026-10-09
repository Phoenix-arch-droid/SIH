"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracking.predictor

Motion Predictor & Dead-Reckoning Engine:
- Multi-step forward state propagation during occlusions / frame drops
- Covariance uncertainty ellipse calculation (major/minor radii and orientation)
- Latency and gimbal slew lead-angle compensation
- Anticipatory reacquisition trajectory estimation
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any, Optional
from tracking.kalman_tracker import BeaconKalmanTracker, TrackState


@dataclass
class UncertaintyEllipse:
    center_u: float
    center_v: float
    semi_major: float                 # 3-sigma semi-major axis (pixels)
    semi_minor: float                 # 3-sigma semi-minor axis (pixels)
    angle_rad: float                  # Orientation angle of major axis


@dataclass
class PredictedTrajectoryPoint:
    time_offset_s: float
    u: float
    v: float
    semi_major: float
    semi_minor: float


class MotionPredictor:
    """
    Computes dead-reckoning forward projections and uncertainty bounds.
    """

    def __init__(self, latency_compensation_s: float = 0.035):
        self.latency_compensation_s = latency_compensation_s

    def compute_uncertainty_ellipse(
        self,
        P_sub: np.ndarray,
        center_u: float,
        center_v: float,
        confidence_sigma: float = 2.45 # 95% confidence interval for 2D Gaussian
    ) -> UncertaintyEllipse:
        """
        Derives the 2D positional error ellipse from the top-left 2x2 block of covariance P.
        """
        # Eigenvalues and eigenvectors of the 2x2 position covariance
        cov_pos = P_sub[:2, :2]
        eigenvals, eigenvecs = np.linalg.eigh(cov_pos)

        # Sort eigenvalues descending
        order = eigenvals.argsort()[::-1]
        eigenvals = np.maximum(eigenvals[order], 1e-4)
        eigenvecs = eigenvecs[:, order]

        # Semi-major and semi-minor axes
        semi_major = confidence_sigma * math.sqrt(float(eigenvals[0]))
        semi_minor = confidence_sigma * math.sqrt(float(eigenvals[1]))
        
        # Heading angle
        angle_rad = math.atan2(float(eigenvecs[1, 0]), float(eigenvecs[0, 0]))

        return UncertaintyEllipse(
            center_u=center_u,
            center_v=center_v,
            semi_major=semi_major,
            semi_minor=semi_minor,
            angle_rad=angle_rad
        )

    def predict_forward(
        self,
        tracker: BeaconKalmanTracker,
        lookahead_seconds: float = 0.5,
        num_steps: int = 10
    ) -> List[PredictedTrajectoryPoint]:
        """
        Generates a forward trajectory prediction path for visual HUD and predictive control.
        """
        dt_step = lookahead_seconds / max(1, num_steps)
        x_temp = tracker.x.copy()
        P_temp = tracker.P.copy()

        points: List[PredictedTrajectoryPoint] = []
        F = tracker._get_F(dt_step)
        Q = tracker._get_Q(dt_step)

        current_t = 0.0
        for _ in range(num_steps):
            current_t += dt_step
            x_temp = F @ x_temp
            P_temp = F @ P_temp @ F.T + Q

            ellipse = self.compute_uncertainty_ellipse(P_temp, x_temp[0], x_temp[1])
            points.append(PredictedTrajectoryPoint(
                time_offset_s=current_t,
                u=float(x_temp[0]),
                v=float(x_temp[1]),
                semi_major=ellipse.semi_major,
                semi_minor=ellipse.semi_minor
            ))

        return points

    def compute_compensated_target_point(
        self,
        tracker: BeaconKalmanTracker,
        system_latency_s: float
    ) -> Tuple[float, float]:
        """
        Projects target pixel position forward by system processing and gimbal response latency
        to calculate lead-angle gimbal commands.
        """
        state = tracker.get_state()
        dt = max(0.0, system_latency_s)
        # 2nd-order kinematic projection: u_lead = u + v_u*dt + 0.5*a_u*dt^2
        u_lead = state.u + state.velocity_u * dt + 0.5 * state.accel_u * (dt ** 2)
        v_lead = state.v + state.velocity_v * dt + 0.5 * state.accel_v * (dt ** 2)
        return u_lead, v_lead
