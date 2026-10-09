"""
Unit tests for tracking and control packages:
- BeaconKalmanTracker
- MotionPredictor
- PanTiltGimbalController
- AutonomousSearchScanner
"""

import math
import numpy as np
import pytest
from detection.baseline_cv import DetectionCandidate
from tracking.kalman_tracker import BeaconKalmanTracker
from tracking.predictor import MotionPredictor
from control.gimbal_controller import PanTiltGimbalController, GimbalLimits
from control.scan_patterns import AutonomousSearchScanner


def test_kalman_tracker_velocity_convergence():
    tracker = BeaconKalmanTracker("TEST", initial_u=100.0, initial_v=100.0, initial_dt=0.033)
    # Simulate a target moving with velocity = +60 pixels/s in U and 0 in V
    true_vel_u = 60.0
    dt = 0.033
    curr_u = 100.0

    for _ in range(40):
        curr_u += true_vel_u * dt
        cand = DetectionCandidate(
            centroid_u=curr_u + np.random.normal(0, 0.5),
            centroid_v=100.0 + np.random.normal(0, 0.5),
            bbox=(int(curr_u), 100, 10, 10),
            area=50.0,
            circularity=0.9,
            peak_intensity=200.0,
            mean_intensity=150.0,
            snr_db=20.0,
            confidence=0.85,
            raw_score=0.85
        )
        tracker.predict(dt)
        tracker.update(cand)

    state = tracker.get_state()
    # Estimated velocity should converge close to 60 pixels/s
    assert math.isclose(state.velocity_u, true_vel_u, abs_tol=8.0)
    assert abs(state.velocity_v) < 5.0
    assert state.is_confirmed is True


def test_motion_predictor_forward_extrapolation():
    tracker = BeaconKalmanTracker("TEST", initial_u=200.0, initial_v=200.0)
    tracker.x[2] = 100.0 # 100 pixels/s in U
    predictor = MotionPredictor()
    pred_u, pred_v = predictor.compute_compensated_target_point(tracker, 0.2) # 0.2s ahead
    # u_pred = 200 + 100 * 0.2 = 220
    assert math.isclose(pred_u, 220.0, rel_tol=1e-3)


def test_gimbal_controller_error_reduction():
    gimbal = PanTiltGimbalController()
    gimbal.reset(initial_pan_deg=10.0, initial_tilt_deg=5.0)

    # Closed-loop tracking commanded to drive 10 deg pan error to 0
    dt = 0.033
    for _ in range(60): # ~2 seconds
        gimbal.update_closed_loop(
            estimated_angular_error_pan_deg=-gimbal.pan_deg,
            estimated_angular_error_tilt_deg=-gimbal.tilt_deg,
            estimated_target_rate_pan_deg_s=0.0,
            estimated_target_rate_tilt_deg_s=0.0,
            dt=dt
        )

    # Position should have moved significantly toward 0.0
    assert abs(gimbal.pan_deg) < 1.0
    assert abs(gimbal.tilt_deg) < 1.0


def test_gimbal_slew_rate_clamping():
    limits = GimbalLimits(max_velocity_deg_s=30.0)
    gimbal = PanTiltGimbalController(limits=limits)
    gimbal.reset(0.0, 0.0)

    # Massive 90 deg error requested
    dt = 0.033
    for _ in range(10):
        gimbal.update_closed_loop(90.0, 0.0, 0.0, 0.0, dt)

    # Velocity must not exceed max_velocity_deg_s
    assert abs(gimbal.pan_vel_deg_s) <= limits.max_velocity_deg_s + 0.1


def test_autonomous_scanner_spiral():
    scanner = AutonomousSearchScanner(max_search_radius_deg=10.0)
    p0, t0, _, _ = scanner.compute_scan_angles(0.1)
    p1, t1, _, _ = scanner.compute_scan_angles(1.0)
    # Radii should increase over time
    r0 = math.hypot(p0, t0)
    r1 = math.hypot(p1, t1)
    assert r1 > r0
