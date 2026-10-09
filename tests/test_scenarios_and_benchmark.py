"""
Unit tests for simulation scenarios and automated benchmark engine:
Validates all 14 standard scenarios, reproducibility, identical seed execution,
and comparative statistical metrics calculations.
"""

import math
import pytest
from simulation.scenarios import ScenarioCatalog
from simulation.benchmark import BenchmarkSuite, BenchmarkScenarioResult
from simulation.engine import SimulationEngine


def test_scenario_catalog_all_14_scenarios():
    scenarios = ScenarioCatalog.get_all_scenarios()
    assert len(scenarios) == 14
    for i in range(1, 15):
        s = ScenarioCatalog.get_by_id(i)
        assert s.scenario_id == i
        assert len(s.targets_config) >= 1
        assert s.duration_s > 0.0


def test_benchmark_reproducibility_same_seed():
    suite = BenchmarkSuite()
    scen = ScenarioCatalog.scenario_1_static_target()
    scen.duration_s = 2.0 # Quick test duration

    res1 = suite.run_single_scenario(scen, use_ai=False)
    res2 = suite.run_single_scenario(scen, use_ai=False)

    assert math.isclose(res1.mean_error_deg, res2.mean_error_deg, abs_tol=1e-5)
    assert math.isclose(res1.rms_error_deg, res2.rms_error_deg, abs_tol=1e-5)
    assert res1.total_frames == res2.total_frames


def test_comparative_benchmark_run():
    suite = BenchmarkSuite()
    scen = ScenarioCatalog.scenario_1_static_target()
    scen.duration_s = 2.5

    report = suite.run_comparison(scen)
    assert report.baseline.total_frames > 50
    assert report.enhanced.total_frames > 50
    assert report.baseline.rms_error_deg >= 0.0
    assert report.enhanced.rms_error_deg >= 0.0
    assert report.baseline.fps_mean > 15.0
    assert report.enhanced.fps_mean > 15.0
