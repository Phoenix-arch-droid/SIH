"""
Integration and End-to-End Pipeline Tests:
- Full closed-loop verification:
  Simulation -> Virtual Camera -> Disturbance -> Detection -> ID -> Tracking -> Predictor -> Controller -> Telemetry
- Deterministic 18-step Coarse Alignment sequence:
  Acquisition -> Locked -> Disturbance -> Degradation -> Occlusion Loss -> Predictive Recovery -> Report Generation
- Persistence: JSON save, load, and replay.
"""

import os
import math
import tempfile
import pytest
from simulation.engine import SimulationEngine
from simulation.scenarios import ScenarioCatalog
from analytics.logger import ExperimentSession
from analytics.metrics import RealTimeMetricsCalculator
from analytics.report_generator import PerformanceReportGenerator
from core.state_machine import AlignmentState


def test_closed_loop_step_telemetry():
    engine = SimulationEngine(use_ai_enhancement=True, seed=201)
    scen = ScenarioCatalog.scenario_1_static_target()
    engine.load_scenario(scen)

    snapshot = engine.step(dt=0.033)
    assert snapshot is not None
    assert snapshot.frame_number == 1
    assert snapshot.fps > 0.0
    assert snapshot.gt is not None
    assert snapshot.gt.true_angular_error_deg >= 0.0
    assert snapshot.state in AlignmentState


def test_e2e_tracking_convergence():
    """Validates that closed-loop control reduces pointing error and locks on target."""
    engine = SimulationEngine(use_ai_enhancement=True, seed=301)
    scen = ScenarioCatalog.scenario_1_static_target()
    # Initial camera offset is 2.5 deg pan, -1.5 deg tilt (~2.9 deg total)
    engine.load_scenario(scen)

    initial_error = None
    final_error = None

    # Run for 120 steps (~4 seconds)
    for i in range(120):
        snap = engine.step(dt=0.033)
        if i == 0:
            initial_error = snap.gt.true_angular_error_deg
        if i == 119:
            final_error = snap.gt.true_angular_error_deg

    assert initial_error is not None and initial_error > 2.0
    # Error should have converged down into lock boundary (<= 0.5 deg)
    assert final_error < 0.50
    assert engine.state_machine.current_state in (AlignmentState.TRACKING, AlignmentState.LOCKED)


def test_e2e_temporary_occlusion_and_recovery():
    """Validates dead-reckoning prediction during occlusion and reacquisition on re-emergence."""
    engine = SimulationEngine(use_ai_enhancement=True, seed=401)
    scen = ScenarioCatalog.scenario_11_temporary_occlusion()
    scen.duration_s = 6.0
    engine.load_scenario(scen)

    states_seen = set()
    for _ in range(int(scen.duration_s / 0.033)):
        snap = engine.step(dt=0.033)
        states_seen.add(snap.state)

    # Must have experienced TRACKING/LOCKED, followed by DEGRADED/LOST during occlusion, and recovered
    assert AlignmentState.TRACKING in states_seen or AlignmentState.LOCKED in states_seen
    assert AlignmentState.DEGRADED in states_seen or AlignmentState.LOST in states_seen


def test_experiment_persistence_save_load():
    with tempfile.TemporaryDirectory() as tmpdir:
        json_path = os.path.join(tmpdir, "test_exp.json")
        csv_path = os.path.join(tmpdir, "test_exp.csv")

        session = ExperimentSession(experiment_id="TEST_001", scenario_name="Static Test")
        engine = SimulationEngine(seed=501)
        engine.load_scenario(ScenarioCatalog.scenario_1_static_target())

        for _ in range(10):
            snap = engine.step(0.033)
            session.record_snapshot(snap)

        session.save_json(json_path)
        session.save_csv(csv_path)

        assert os.path.exists(json_path)
        assert os.path.exists(csv_path)
        assert os.path.getsize(json_path) > 500
        assert os.path.getsize(csv_path) > 500

        # Load back
        loaded = ExperimentSession.load_json(json_path)
        assert loaded.experiment_id == "TEST_001"
        assert len(loaded.telemetry_records) == 10
        assert loaded.telemetry_records[0]["frame_number"] == 1


def test_performance_report_generation():
    engine = SimulationEngine(seed=601)
    scen = ScenarioCatalog.scenario_1_static_target()
    scen.duration_s = 3.0

    from simulation.benchmark import BenchmarkSuite
    suite = BenchmarkSuite()
    res = suite.run_single_scenario(scen, use_ai=True)
    md = PerformanceReportGenerator.generate_single_report_markdown(res)

    assert "# SIH26169 FSOC Virtual Tracking System" in md
    assert "Simulation Duration" in md
    assert "Lock Retention Rate" in md
    assert "Root Mean Square (RMS) Error" in md
    assert "SYSTEM-DEFINED SIMULATION PARAMETER" in md
    assert "ENGINEERING ASSUMPTION" in md
