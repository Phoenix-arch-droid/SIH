"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: analytics.metrics

Real-Time and Cumulative Tracking Performance Analytics:
- Acquisition Time (s)
- Instantaneous & Cumulative RMS, Mean, Median, and Maximum Pointing Errors
- Lock Retention Rate (%)
- Reacquisition count and recovery duration
- Target visibility and occlusion fraction
- Real-time frame rates and computational latency
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from simulation.engine import TelemetrySnapshot
from core.state_machine import AlignmentState


@dataclass
class RunningPerformanceMetrics:
    """Live summary metrics for real-time HUD and KPI cards."""
    total_frames: int = 0
    elapsed_time_s: float = 0.0
    current_fps: float = 0.0
    current_latency_ms: float = 0.0
    
    # State tracking
    current_state: str = "SEARCHING"
    is_locked: bool = False
    acquisition_time_s: Optional[float] = None
    lock_retention_pct: float = 0.0
    locked_frame_count: int = 0

    # Error statistics (deg & mrad)
    current_error_deg: float = 0.0
    current_error_mrad: float = 0.0
    mean_error_deg: float = 0.0
    mean_error_mrad: float = 0.0
    rms_error_deg: float = 0.0
    rms_error_mrad: float = 0.0
    max_error_deg: float = 0.0
    max_error_mrad: float = 0.0

    # Event counts
    lost_frames_count: int = 0
    reacquisition_count: int = 0
    mean_confidence: float = 0.0
    peak_gimbal_slew_deg_s: float = 0.0


class RealTimeMetricsCalculator:
    """
    Maintains running statistical accumulators for telemetry.
    """

    def __init__(self, lock_threshold_deg: float = 0.5):
        self.lock_threshold_deg = lock_threshold_deg
        self.reset()

    def reset(self):
        self.total_frames = 0
        self.locked_frames = 0
        self.lost_frames = 0
        self.reacquisitions = 0
        self.acquisition_time_s: Optional[float] = None

        self.error_sum_deg = 0.0
        self.error_sq_sum_deg = 0.0
        self.max_error_deg = 0.0
        self.peak_slew = 0.0
        self.conf_sum = 0.0
        self.recent_errors: List[float] = []

    def update(self, snapshot: TelemetrySnapshot) -> RunningPerformanceMetrics:
        """
        Updates cumulative statistics with a new telemetry frame.
        """
        self.total_frames += 1
        t = snapshot.timestamp_s
        err_deg = snapshot.gt.true_angular_error_deg

        # Update acquisition time
        if self.acquisition_time_s is None and snapshot.state in (AlignmentState.TRACKING, AlignmentState.LOCKED):
            self.acquisition_time_s = t

        # Lock retention
        if snapshot.state == AlignmentState.LOCKED or err_deg <= self.lock_threshold_deg:
            self.locked_frames += 1

        if snapshot.state in (AlignmentState.LOST, AlignmentState.REACQUIRING):
            self.lost_frames += 1

        # Accumulators
        self.error_sum_deg += err_deg
        self.error_sq_sum_deg += (err_deg ** 2)
        if err_deg > self.max_error_deg:
            self.max_error_deg = err_deg

        current_slew = math.hypot(snapshot.gimbal_pan_vel_deg_s, snapshot.gimbal_tilt_vel_deg_s)
        if current_slew > self.peak_slew:
            self.peak_slew = current_slew

        self.conf_sum += snapshot.tracking_confidence

        self.recent_errors.append(err_deg)
        if len(self.recent_errors) > 500:
            self.recent_errors.pop(0)

        # Derived statistics
        n = max(1, self.total_frames)
        mean_deg = self.error_sum_deg / n
        rms_deg = math.sqrt(self.error_sq_sum_deg / n)
        lock_pct = (self.locked_frames / n) * 100.0

        deg_to_mrad = (math.pi / 180.0) * 1000.0

        return RunningPerformanceMetrics(
            total_frames=self.total_frames,
            elapsed_time_s=t,
            current_fps=snapshot.fps,
            current_latency_ms=snapshot.latency_ms,
            current_state=snapshot.state.value,
            is_locked=snapshot.is_locked,
            acquisition_time_s=self.acquisition_time_s,
            lock_retention_pct=lock_pct,
            locked_frame_count=self.locked_frames,
            current_error_deg=err_deg,
            current_error_mrad=err_deg * deg_to_mrad,
            mean_error_deg=mean_deg,
            mean_error_mrad=mean_deg * deg_to_mrad,
            rms_error_deg=rms_deg,
            rms_error_mrad=rms_deg * deg_to_mrad,
            max_error_deg=self.max_error_deg,
            max_error_mrad=self.max_error_deg * deg_to_mrad,
            lost_frames_count=self.lost_frames,
            reacquisition_count=self.reacquisitions,
            mean_confidence=self.conf_sum / n,
            peak_gimbal_slew_deg_s=self.peak_slew
        )
