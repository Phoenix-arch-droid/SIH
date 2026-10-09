"""
Unit tests for core.targets and core.disturbances modules.
Validates 3D kinematic trajectories, modulation blinking, and disturbance physics.
"""

import math
import numpy as np
import pytest
from core.targets import OpticalBeaconTarget, TrajectoryType, BeaconOpticalSignature, BeaconTargetManager
from core.disturbances import DisturbanceEngine, DisturbanceLevel, DisturbanceConfig


def test_target_stationary_trajectory():
    t = OpticalBeaconTarget("T1", initial_position=[10.0, 500.0, 50.0], trajectory_type=TrajectoryType.STATIONARY)
    pos, vel, accel = t.update(dt=0.1, current_time_s=1.0)
    assert np.allclose(pos, [10.0, 500.0, 50.0])
    assert np.allclose(vel, 0.0)


def test_target_linear_constant_velocity():
    t = OpticalBeaconTarget(
        "T2",
        initial_position=[0.0, 500.0, 0.0],
        initial_velocity=[10.0, 0.0, 0.0],
        trajectory_type=TrajectoryType.LINEAR
    )
    # After 2.0 seconds at 10 m/s in X, position X should be 20.0
    for _ in range(20):
        pos, vel, _ = t.update(dt=0.1, current_time_s=0.1)
    assert math.isclose(pos[0], 20.0, rel_tol=1e-3)
    assert np.allclose(vel, [10.0, 0.0, 0.0])


def test_target_modulation_factor():
    opt = BeaconOpticalSignature(modulation_frequency_hz=10.0) # 10 Hz = 0.1s period (0.05s ON, 0.05s OFF)
    t = OpticalBeaconTarget("T3", optical_signature=opt)
    # At t = 0.02s (inside 1st half of period): ON
    factor_on = t.get_instantaneous_intensity_factor(0.02)
    assert math.isclose(factor_on, 1.0)
    # At t = 0.07s (inside 2nd half of period): OFF/low
    factor_off = t.get_instantaneous_intensity_factor(0.07)
    assert factor_off < 0.2


def test_target_manager_designated_and_distractor():
    mgr = BeaconTargetManager()
    p = OpticalBeaconTarget("PRIMARY", is_designated_target=True)
    d = OpticalBeaconTarget("DISTRACTOR", is_designated_target=False)
    mgr.add_target(p)
    mgr.add_target(d)

    assert mgr.get_designated() == p
    assert len(mgr.get_all()) == 2


def test_disturbance_presets():
    eng = DisturbanceEngine()
    eng.apply_preset(DisturbanceLevel.OFF)
    assert eng.config.vibration_enabled is False
    assert eng.config.scintillation_index == 0.0

    eng.apply_preset(DisturbanceLevel.HIGH)
    assert eng.config.vibration_enabled is True
    assert eng.config.scintillation_index >= 0.70


def test_disturbance_angular_jitter_zero_when_off():
    eng = DisturbanceEngine()
    eng.apply_preset(DisturbanceLevel.OFF)
    d_pan, d_tilt = eng.compute_angular_disturbances(1.0)
    assert math.isclose(d_pan, 0.0, abs_tol=1e-7)
    assert math.isclose(d_tilt, 0.0, abs_tol=1e-7)


def test_disturbance_occlusion_interval():
    eng = DisturbanceEngine()
    eng.config.occlusion_active = True
    eng.config.occlusion_start_s = 5.0
    eng.config.occlusion_duration_s = 2.0

    assert eng.is_occluded(4.0) is False
    assert eng.is_occluded(6.0) is True
    assert eng.is_occluded(8.0) is False
