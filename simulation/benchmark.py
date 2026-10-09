"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: simulation.benchmark

Automated Benchmark Engine:
Executes rigorous, controlled, side-by-side evaluations of:
  BASELINE (Classical CV) vs ENHANCED (AI/ML Detector)
under 100% identical initial conditions, random seeds, durations, and disturbances.
Calculates authentic mathematical statistics:
- Acquisition Time (s)
- Lock Retention Rate (%)
- Mean, Median, RMS, Maximum, and Standard Deviation of Angular Error (mrad & deg)
- Slew statistics, lost frames, and reacquisition recovery counts
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional, Tuple

from simulation.engine import SimulationEngine, TelemetrySnapshot
from simulation.scenarios import ScenarioDefinition, ScenarioCatalog
from core.state_machine import AlignmentState


@dataclass
class BenchmarkScenarioResult:
    scenario_id: int
    scenario_name: str
    mode: str                         # "BASELINE" or "ENHANCED_AI"
    simulation_duration_s: float
    total_frames: int
    fps_mean: float
    latency_mean_ms: float
    acquisition_time_s: Optional[float]
    lock_retention_rate_pct: float
    lost_frames_count: int
    reacquisition_count: int

    # Angular Pointing Error Statistics (True Error vs Ground Truth)
    mean_error_deg: float
    median_error_deg: float
    rms_error_deg: float
    max_error_deg: float
    std_error_deg: float

    mean_error_mrad: float
    median_error_mrad: float
    rms_error_mrad: float
    max_error_mrad: float
    std_error_mrad: float

    # Raw telemetry snapshots for charting
    telemetry_history: List[TelemetrySnapshot] = field(default_factory=list, repr=False)


@dataclass
class ComparativeBenchmarkReport:
    timestamp_iso: str
    scenario_id: int
    scenario_name: str
    baseline: BenchmarkScenarioResult
    enhanced: BenchmarkScenarioResult
    rms_error_reduction_pct: float
    lock_retention_improvement_pct: float
    latency_delta_ms: float


class BenchmarkSuite:
    """
    Automated benchmark suite for running single or batch evaluations.
    """

    def __init__(self, dt: float = 0.033):
        self.dt = dt

    def run_single_scenario(
        self,
        scenario: ScenarioDefinition,
        use_ai: bool
    ) -> BenchmarkScenarioResult:
        """
        Executes a single benchmark run under specified scenario and mode.
        """
        engine = SimulationEngine(use_ai_enhancement=use_ai, seed=scenario.seed)
        engine.load_scenario(scenario)

        # Run simulation to completion
        num_steps = int(math.ceil(scenario.duration_s / self.dt))
        telemetry: List[TelemetrySnapshot] = []

        for _ in range(num_steps):
            snap = engine.step(self.dt)
            telemetry.append(snap)

        # Compute authentic statistics from real telemetry and ground truth
        errors_deg = np.array([s.gt.true_angular_error_deg for s in telemetry], dtype=np.float64)
        errors_mrad = errors_deg * (math.pi / 180.0) * 1000.0

        # Acquisition time: first frame where state becomes TRACKING or LOCKED
        acq_time_s = None
        for s in telemetry:
            if s.state in (AlignmentState.TRACKING, AlignmentState.LOCKED):
                acq_time_s = s.timestamp_s
                break

        # Lock retention: percentage of frames where true error <= 0.5 deg (or in LOCKED state)
        locked_frames = sum(1 for s in telemetry if s.state == AlignmentState.LOCKED)
        lock_rate = (locked_frames / max(1, len(telemetry))) * 100.0

        lost_frames = sum(1 for s in telemetry if s.state in (AlignmentState.LOST, AlignmentState.REACQUIRING))
        
        # Count reacquisition transitions
        reacq_count = sum(
            1 for i in range(1, len(telemetry))
            if telemetry[i-1].state == AlignmentState.REACQUIRING and telemetry[i].state == AlignmentState.ACQUIRING
        )

        latencies = [s.latency_ms for s in telemetry]
        fps_list = [s.fps for s in telemetry]

        mean_deg = float(np.mean(errors_deg))
        median_deg = float(np.median(errors_deg))
        rms_deg = float(np.sqrt(np.mean(errors_deg ** 2)))
        max_deg = float(np.max(errors_deg))
        std_deg = float(np.std(errors_deg))

        mode_str = "ENHANCED_AI" if use_ai else "BASELINE"

        return BenchmarkScenarioResult(
            scenario_id=scenario.scenario_id,
            scenario_name=scenario.name,
            mode=mode_str,
            simulation_duration_s=scenario.duration_s,
            total_frames=len(telemetry),
            fps_mean=float(np.mean(fps_list)),
            latency_mean_ms=float(np.mean(latencies)),
            acquisition_time_s=acq_time_s,
            lock_retention_rate_pct=lock_rate,
            lost_frames_count=lost_frames,
            reacquisition_count=reacq_count,
            mean_error_deg=mean_deg,
            median_error_deg=median_deg,
            rms_error_deg=rms_deg,
            max_error_deg=max_deg,
            std_error_deg=std_deg,
            mean_error_mrad=mean_deg * 17.4533,
            median_error_mrad=median_deg * 17.4533,
            rms_error_mrad=rms_deg * 17.4533,
            max_error_mrad=max_deg * 17.4533,
            std_error_mrad=std_deg * 17.4533,
            telemetry_history=telemetry
        )

    def run_comparison(self, scenario: ScenarioDefinition) -> ComparativeBenchmarkReport:
        """
        Runs both BASELINE and ENHANCED modes under identical seed and conditions.
        """
        import datetime
        res_baseline = self.run_single_scenario(scenario, use_ai=False)
        res_enhanced = self.run_single_scenario(scenario, use_ai=True)

        rms_reduction = 0.0
        if res_baseline.rms_error_deg > 1e-4:
            rms_reduction = ((res_baseline.rms_error_deg - res_enhanced.rms_error_deg) / res_baseline.rms_error_deg) * 100.0

        lock_improvement = res_enhanced.lock_retention_rate_pct - res_baseline.lock_retention_rate_pct
        latency_delta = res_enhanced.latency_mean_ms - res_baseline.latency_mean_ms

        return ComparativeBenchmarkReport(
            timestamp_iso=datetime.datetime.now().isoformat(),
            scenario_id=scenario.scenario_id,
            scenario_name=scenario.name,
            baseline=res_baseline,
            enhanced=res_enhanced,
            rms_error_reduction_pct=rms_reduction,
            lock_retention_improvement_pct=lock_improvement,
            latency_delta_ms=latency_delta
        )

    def run_all_benchmarks(self) -> List[ComparativeBenchmarkReport]:
        """
        Executes comparison across all 14 standard scenarios.
        """
        reports: List[ComparativeBenchmarkReport] = []
        for scen in ScenarioCatalog.get_all_scenarios():
            rep = self.run_comparison(scen)
            reports.append(rep)
        return reports
