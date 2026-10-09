# Requirements Traceability Matrix: ISRO SIH26169 FSOC Virtual Tracking System

**Project Title:** Development of an AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile Free Space Optical Communication (FSOC) Terminals  
**Problem Statement ID:** SIH26169  
**Organization:** Indian Space Research Organisation (ISRO)  
**Department:** Department of Space / ISRO  
**Theme:** Smart Automation  
**Classification:** Requirements Verification & Compliance Matrix  

---

## 1. Compliance Certification Overview

This document provides complete, line-by-line verification traceability for all mandatory functional, technical, and delivery requirements specified in the official ISRO SIH26169 problem statement. 

Each requirement is mapped directly to its architectural implementation in source code, automated test verification evidence, and interactive user interface manifestation.

### Terminology & Standards Compliance
In accordance with official SIH instructions:
- **SYSTEM-DEFINED SIMULATION PARAMETER:** Any parameter, geometry, or operational threshold established internally to provide a realistic simulation baseline (e.g., $0.50^\circ$ lock threshold, $1280 \times 720$ resolution).
- **ENGINEERING ASSUMPTION:** Physical approximations representing environmental, atmospheric, or actuator behaviors (e.g., $25\,\text{ms}$ electro-mechanical motor lag, Kolmogorov log-normal scintillation model).

---

## 2. Requirements Traceability Matrix

| Req ID | Official Requirement Description | Implementation Module & File | Verification Test Suite | Compliance Status | Interactive / Deliverable Evidence |
| :---: | :--- | :--- | :--- | :---: | :--- |
| **FR-01** | **Generate a configurable virtual environment** | `core/coordinates.py`<br>`simulation/engine.py`<br>`simulation/scenarios.py` | `tests/test_coordinates.py`<br>`tests/test_scenarios_and_benchmark.py` | **100% COMPLIANT** | 3D World View (`ui/static/js/world_view.js`) with mobile platform $\mathcal{F}_B$, remote terminal, coordinate axes, and distance vectors. |
| **FR-02** | **Generate one or more moving targets/optical beacons** | `core/targets.py`<br>`BeaconTargetManager`<br>`OpticalBeaconTarget` | `tests/test_targets_and_disturbances.py` (`TC-15`, `TC-16`, `TC-17`, `TC-18`) | **100% COMPLIANT** | Supports 7 trajectory types (Stationary, Linear, Sinusoidal, Accelerating, Circular, Gauss-Markov, Waypoint). Multi-target distractor support. |
| **FR-03** | **Implement a movable virtual camera** | `core/camera_model.py`<br>`PinholeCameraModel`<br>`core/coordinates.py` | `tests/test_camera_model.py` (`TC-09` through `TC-14`) | **100% COMPLIANT** | Pinhole camera ($1280 \times 720$, $15.0^\circ$ HFOV), 3D-to-2D projection, radial distortion, and Airy Point Spread Function (PSF). |
| **FR-04** | **Automatically detect the target beacon** | `detection/baseline_cv.py`<br>`detection/ai_detector.py`<br>`detection/id_matcher.py` | `tests/test_detectors_and_id.py` (`TC-22` through `TC-25`) | **100% COMPLIANT** | Dual-mode detector: Baseline top-hat morphological filter vs AI 12-D spatio-spectral classifier + 20Hz temporal FFT ID matching. |
| **FR-05** | **Continuously track the beacon using computer vision** | `tracking/kalman_tracker.py`<br>`tracking/predictor.py`<br>`tracking/association.py` | `tests/test_tracker_and_control.py` (`TC-26`, `TC-27`) | **100% COMPLIANT** | 6-state continuous-discrete Kalman filter ($u, v, \dot{u}, \dot{v}, \ddot{u}, \ddot{v}$) with dynamic SNR-scaled covariance $R(\text{SNR})$ and 3-$\sigma$ ellipses. |
| **FR-06** | **Control and reposition the virtual camera** | `control/gimbal_controller.py`<br>`control/scan_patterns.py`<br>`core/state_machine.py` | `tests/test_tracker_and_control.py` (`TC-28`, `TC-29`, `TC-30`) | **100% COMPLIANT** | 2-axis Pan/Tilt PID controller with electro-mechanical lag ($\tau=25\,\text{ms}$), anti-windup, slew rate clamping ($35^\circ/\text{s}$), and spiral search. |
| **FR-07** | **Introduce visual & platform disturbances:**<br>- Atmospheric turbulence<br>- Platform vibrations<br>- Camera motion<br>- Sensor noise<br>- Visual occlusions | `core/disturbances.py`<br>`DisturbanceEngine`<br>`DisturbanceConfig` | `tests/test_targets_and_disturbances.py` (`TC-19`, `TC-20`, `TC-21`) | **100% COMPLIANT** | - Kolmogorov log-normal scintillation & beam wander<br>- Multi-harmonic ($24\,\text{Hz} + 65\,\text{Hz}$) airframe jitter<br>- Dynamic mobile base roll, pitch, yaw<br>- Gaussian readout, Poisson shot, impulse noise<br>- Temporary line-of-sight cloud occlusion |
| **FR-08** | **Display tracking performance and statistics in real time** | `analytics/metrics.py`<br>`ui/server.py`<br>`ui/static/js/charts.js` | `tests/test_e2e_pipeline.py` (`TC-40`, `TC-41`, `TC-42`) | **100% COMPLIANT** | Top KPI bar + 4 synchronized real-time Chart.js scrolling graphs (Pointing error, confidence, FPS/latency, gimbal angles). Zero hardcoded metrics. |
| **DR-01** | **Standalone executable application** | `main.py`<br>`ui/app.py`<br>PyInstaller specification | Standalone build verification (`dist/FSOC_Virtual_Tracker.exe`) | **100% COMPLIANT** | Single double-clickable `.exe` bundling backend, Python runtime, local static assets, and offline models. Zero internet required. |
| **DR-02** | **Complete documented source code** | `core/`, `detection/`, `tracking/`, `control/`, `simulation/`, `analytics/`, `ui/` | Static analysis + `pytest -v` (44 of 44 tests pass) | **100% COMPLIANT** | 100% clean, modular Python codebase with docstrings, type annotations, and clean architectural separation. |
| **DR-03** | **Technical Report (10–15 pages)** | `TECHNICAL_REPORT.md` | Document review & completeness inspection | **100% COMPLIANT** | Comprehensive 21-section formal technical paper covering kinematics, optics, AI/ML models, control theory, benchmarks, and limitations. |
| **DR-04** | **User Manual** | `USER_MANUAL.md` | Verification against live UI controls and workflows | **100% COMPLIANT** | Complete operational manual with system requirements, launch instructions, dashboard guide, scenario tutorials, and troubleshooting FAQ. |
| **DR-05** | **Automatically generated performance report/log** | `analytics/report_generator.py`<br>`FSOC_Performance_Report.md` | `tests/test_e2e_pipeline.py` (`TC-44`) | **100% COMPLIANT** | Emits Markdown & HTML logs with simulation duration, FPS, acquisition time, RMS/mean/max errors, lock retention %, and assumption tags. |

---

## 3. Mandatory Performance Log Parameter Traceability

The problem statement mandates that the automatically generated performance log include specific performance parameters:

| Mandated Parameter | Mathematical Definition | Module Source | Verification Log Reference |
| :--- | :--- | :--- | :--- |
| **Simulation Duration** | $T_{sim} = N_{frames} \times \Delta t$ | `simulation/engine.py` | `FSOC_Performance_Report.md` Section 1 |
| **FPS (Frame Rate)** | $\text{FPS} = \frac{1}{\Delta t}$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 1 |
| **Acquisition Time** | $\min \{t \mid \text{State}(t) \in \{\text{TRACKING}, \text{LOCKED}\}\}$ | `simulation/benchmark.py` | `FSOC_Performance_Report.md` Section 1 |
| **Average Tracking Error** | $\frac{1}{N} \sum_{i=1}^N \theta_{error}(i)$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 2 |
| **Maximum Tracking Error** | $\max_{i=1 \dots N} \theta_{error}(i)$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 2 |
| **Lock Retention Rate** | $\frac{N_{\theta \le 0.50^\circ}}{N_{total}} \times 100\%$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 1 |
| **Processing Time** | Latency per frame $t_{proc} = t_{end} - t_{start}$ (ms) | `simulation/engine.py` | `FSOC_Performance_Report.md` Section 1 |
| **Root Mean Square (RMS) Error** | $\sqrt{\frac{1}{N} \sum_{i=1}^N \theta_{error}^2(i)}$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 2 |
| **Median Tracking Error** | $\text{Median}(\{\theta_{error}(i)\})$ | `analytics/metrics.py` | `FSOC_Performance_Report.md` Section 2 |
| **Lost Frames Count** | Total frames where target state is `LOST` or `REACQUIRING` | `simulation/benchmark.py` | `FSOC_Performance_Report.md` Section 1 |
| **Reacquisition Count** | Count of transitions `REACQUIRING` $\to$ `ACQUIRING` | `simulation/benchmark.py` | `FSOC_Performance_Report.md` Section 1 |

---

## 4. Verification Evidence & Artifact References

1. **Automated Unit & Integration Tests:** 44 passing tests executed by `pytest 9.1.1` (see `TEST_REPORT.md`).
2. **Deterministic Closed-Loop Convergence:** Verified by `tests/test_e2e_pipeline.py::test_e2e_tracking_convergence` ($2.91^\circ \to < 0.45^\circ$).
3. **Dead-Reckoning During Occlusion:** Verified by `tests/test_e2e_pipeline.py::test_e2e_temporary_occlusion_and_recovery`.
4. **Offline Packaging:** Verified by `dist/FSOC_Virtual_Tracker.exe` built via PyInstaller.
5. **Authentic Performance Log:** Emitted at `FSOC_Performance_Report.md` directly from simulation telemetry.
