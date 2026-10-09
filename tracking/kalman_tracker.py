"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracking.kalman_tracker

Discrete-Time Kalman Filter for Temporal Optical Beacon Tracking:
- Continuous-time 2nd-order kinematic model: Position, Velocity, Acceleration in image coordinates
- State vector: x = [u, v, du/dt, dv/dt, d2u/dt2, d2v/dt2]^T
- Dynamic process noise covariance Q(dt) based on continuous white noise spectral density
- Dynamic measurement noise covariance R(SNR) scaled by candidate signal-to-noise ratio
- Mahalanobis distance gating for outlier rejection
- Covariance propagation and sub-pixel position smoothing
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any
from detection.baseline_cv import DetectionCandidate


@dataclass
class TrackState:
    """Estimated kinematic state of an optical beacon track."""
    track_id: str
    u: float                          # Estimated pixel horizontal coordinate
    v: float                          # Estimated pixel vertical coordinate
    velocity_u: float                 # Estimated velocity (pixels/s)
    velocity_v: float                 # Estimated velocity (pixels/s)
    accel_u: float                    # Estimated acceleration (pixels/s^2)
    accel_v: float                    # Estimated acceleration (pixels/s^2)
    covariance: np.ndarray            # 6x6 State error covariance matrix P
    confidence: float                 # Composite tracking confidence [0.0 - 1.0]
    hits: int = 0                     # Total successful measurement updates
    hit_streak: int = 0               # Consecutive successful updates
    time_since_update_s: float = 0.0  # Elapsed time since last valid measurement
    is_confirmed: bool = False


class BeaconKalmanTracker:
    """
    Kalman filter tracking a single designated beacon or candidate trajectory.
    """

    def __init__(
        self,
        track_id: str,
        initial_u: float,
        initial_v: float,
        initial_dt: float = 0.033,
        process_noise_spectral_density: float = 80.0, # q: acceleration random walk (pixels^2 / s^5)
        base_measurement_noise_std: float = 2.0       # R_std: pixel centroid uncertainty
    ):
        self.track_id = track_id
        self.q = process_noise_spectral_density
        self.base_r_std = base_measurement_noise_std
        
        # 6-state vector: [u, v, u_dot, v_dot, u_ddot, v_ddot]
        self.x = np.array([initial_u, initial_v, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # Initial covariance P
        self.P = np.diag([
            (self.base_r_std * 2.0)**2,
            (self.base_r_std * 2.0)**2,
            100.0,  # Velocity uncertainty (pixels/s)^2
            100.0,
            25.0,   # Accel uncertainty (pixels/s^2)^2
            25.0
        ]).astype(np.float64)

        # Measurement matrix H (observes [u, v])
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        ], dtype=np.float64)

        self.hits = 1
        self.hit_streak = 1
        self.miss_count = 0
        self.time_since_update_s = 0.0
        self.last_confidence = 0.5

    def _get_F(self, dt: float) -> np.ndarray:
        """Constructs state transition matrix for time step dt."""
        dt2 = 0.5 * (dt ** 2)
        F = np.array([
            [1.0, 0.0,  dt, 0.0, dt2, 0.0],
            [0.0, 1.0, 0.0,  dt, 0.0, dt2],
            [0.0, 0.0, 1.0, 0.0,  dt, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0,  dt],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)
        return F

    def _get_Q(self, dt: float) -> np.ndarray:
        """
        Calculates discrete process noise covariance matrix Q(dt)
        for continuous white noise jerk / acceleration disturbance.
        """
        q = self.q
        dt5 = (dt ** 5) / 20.0
        dt4 = (dt ** 4) / 8.0
        dt3 = (dt ** 3) / 6.0
        dt2 = (dt ** 2) / 2.0

        Q1 = np.array([
            [dt5, dt4, dt3],
            [dt4, dt3, dt2],
            [dt3, dt2, dt ]
        ], dtype=np.float64) * q

        # Block diagonal for X and Y dimensions
        Q = np.zeros((6, 6), dtype=np.float64)
        for i in range(3):
            for j in range(3):
                Q[2*i, 2*j] = Q1[i, j]
                Q[2*i + 1, 2*j + 1] = Q1[i, j]
        return Q

    def predict(self, dt: float) -> np.ndarray:
        """
        Propagates state and covariance forward by dt seconds.
        Returns predicted state vector x.
        """
        dt = max(1e-4, dt)
        F = self._get_F(dt)
        Q = self._get_Q(dt)

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.time_since_update_s += dt
        return self.x.copy()

    def update(
        self,
        candidate: DetectionCandidate,
        camera_resolution: Optional[Tuple[int, int]] = None
    ) -> Tuple[np.ndarray, float]:
        """
        Updates filter with a new measurement from detection candidate.
        Dynamically adjusts measurement noise R based on candidate SNR.
        Returns: (updated_state, mahalanobis_distance)
        """
        z = np.array([candidate.centroid_u, candidate.centroid_v], dtype=np.float64)

        # Scale R inversely with SNR (high SNR -> low measurement noise)
        snr_factor = max(0.5, min(4.0, 20.0 / max(1.0, candidate.snr_db)))
        r_std = self.base_r_std * snr_factor
        R = np.diag([r_std**2, r_std**2]).astype(np.float64)

        # Innovation (measurement residual)
        y = z - (self.H @ self.x)

        # Innovation covariance S = H P H^T + R
        S = self.H @ self.P @ self.H.T + R
        S_inv = np.linalg.inv(S)

        # Mahalanobis distance squared: d_M^2 = y^T S^-1 y
        mahalanobis_dist = math.sqrt(float(np.clip(y.T @ S_inv @ y, 0.0, 1e6)))

        # Optimal Kalman Gain K = P H^T S^-1
        K = self.P @ self.H.T @ S_inv

        # State update: x = x + K y
        self.x += K @ y

        # Covariance update (Joseph form for numerical stability):
        # P = (I - K H) P (I - K H)^T + K R K^T
        I = np.eye(6, dtype=np.float64)
        I_KH = I - (K @ self.H)
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        # Update streak metrics
        self.hits += 1
        self.hit_streak += 1
        self.miss_count = 0
        self.time_since_update_s = 0.0

        # Confidence blending: measurement confidence + covariance trace penalty
        pos_cov_trace = float(self.P[0, 0] + self.P[1, 1])
        cov_confidence = math.exp(-pos_cov_trace / 50.0)
        self.last_confidence = float(0.60 * candidate.confidence + 0.40 * cov_confidence)

        return self.x.copy(), mahalanobis_dist

    def mark_miss(self):
        """Records a missing detection frame."""
        self.hit_streak = 0
        self.miss_count += 1
        # Degrade confidence exponentially with missed time
        self.last_confidence = max(0.05, self.last_confidence * 0.88)

    def get_state(self) -> TrackState:
        """Returns the current track estimation snapshot."""
        return TrackState(
            track_id=self.track_id,
            u=float(self.x[0]),
            v=float(self.x[1]),
            velocity_u=float(self.x[2]),
            velocity_v=float(self.x[3]),
            accel_u=float(self.x[4]),
            accel_v=float(self.x[5]),
            covariance=self.P.copy(),
            confidence=self.last_confidence,
            hits=self.hits,
            hit_streak=self.hit_streak,
            time_since_update_s=self.time_since_update_s,
            is_confirmed=(self.hits >= 3)
        )
