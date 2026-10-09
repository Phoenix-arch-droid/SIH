# ISRO SIH26169: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

[![Indian Space Research Organisation](https://img.shields.io/badge/ISRO-Department%20of%20Space-blue.svg)](https://www.isro.gov.in/)
[![Problem Statement SIH26169](https://img.shields.io/badge/SIH-SIH26169-orange.svg)](#)
[![Theme: Smart Automation](https://img.shields.io/badge/Theme-Smart%20Automation-green.svg)](#)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](#)
[![License: MIT](https://img.shields.io/badge/license-MIT-purple.svg)](#)
[![Offline Capability](https://img.shields.io/badge/Offline-100%25%20Air--Gapped-brightgreen.svg)](#)

---

## 1. Executive Summary

This repository delivers the complete, production-grade solution for **ISRO Problem Statement SIH26169**: **"Development of an AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile Free Space Optical Communication (FSOC) Terminals"** under the **Smart Automation** theme.

Free Space Optical Communication (FSOC) offers multi-gigabit data throughput, low probability of intercept (LPI), and freedom from RF spectrum licensing. However, because optical beams operate at sub-milliradian divergences ($\sim 0.5 - 2.0\text{ mrad}$), establishing line-of-sight (LOS) links between mobile aerospace platforms (e.g., LEO satellites, UAVs, HAP stations, naval vessels) requires high-accuracy Pointing, Acquisition, and Tracking (PAT). 

This software provides a **physically rigorous, hardware-in-the-loop (HIL) capable virtual testing laboratory** that completely replaces physical gantry rigs and optical tables. It models the entire closed-loop coarse alignment pipeline: 3D world kinematics, pinhole camera projection, atmospheric Kolmogorov scintillation, beam wander, mechanical platform vibration, sensor noise, dual-mode beacon detection (Classical CV vs. AI/ML Enhanced), 6-state Kalman filtering, and 2-axis Pan/Tilt electro-mechanical PID gimbal control.

---

## 2. Key System Architecture & Capabilities

```
+----------------------------------------------------------------------------------------+
|                                3D VIRTUAL WORLD ENGINE                                 |
|   - SE(3) Kinematics, World Coordinate System, Mobile Host Platform Attitude (R, P, Y)  |
|   - Multi-Target Kinematic Manager (Stationary, Linear, Sinusoidal, Gauss-Markov, etc.)|
|   - 850nm Modulated Optical Beacons & Multi-Wavelength Distractor Glints               |
+-------------------------------------------+--------------------------------------------+
                                            |
                                            v
+----------------------------------------------------------------------------------------+
|                               VIRTUAL SENSOR OPTICS & NOISE                            |
|   - Pinhole Camera (1280x720, 15.0 deg HFOV, 8.44 deg VFOV), Lens Distortion           |
|   - Radiometric Path Loss, Airy/Gaussian Point Spread Function (PSF) Modeling          |
|   - Disturbance Engine: Kolmogorov Scintillation, Beam Wander, Multi-Harmonic Jitter   |
|   - CMOS Readout Noise, Poisson Shot Noise, Hot-Pixel Impulses, Cloud Occlusion        |
+-------------------------------------------+--------------------------------------------+
                                            |
                                            v
+----------------------------------------------------------------------------------------+
|                            DETECTION & IDENTIFICATION ENGINE                           |
|   - Mode A (Baseline): Top-Hat Morphological Filtering, Contour Extraction & Centroid |
|   - Mode B (AI/ML): 12-D Spatio-Spectral Feature Vector + Diffraction Correlator       |
|   - ID Matcher: Rolling FFT Temporal Pulse Frequency Estimation (20 Hz Signature)      |
+-------------------------------------------+--------------------------------------------+
                                            |
                                            v
+----------------------------------------------------------------------------------------+
|                             TEMPORAL TRACKING & PREDICTION                             |
|   - 6-State Continuous-Discrete Kalman Filter (u, v, u_dot, v_dot, u_ddot, v_ddot)     |
|   - Dynamic SNR-Scaled Measurement Covariance & Mahalanobis Gating                     |
|   - Forward Dead-Reckoning Predictor with 3-Sigma Covariance Ellipses                  |
+-------------------------------------------+--------------------------------------------+
                                            |
                                            v
+----------------------------------------------------------------------------------------+
|                          7-STATE ALIGNMENT STATE MACHINE                               |
|   [SEARCHING] <-> [ACQUIRING] <-> [TRACKING] <-> [LOCKED]                              |
|                         ^              |                                               |
|                         |              v                                               |
|                   [REACQUIRING] <--- [LOST] <--- [DEGRADED]                            |
+-------------------------------------------+--------------------------------------------+
                                            |
                                            v
+----------------------------------------------------------------------------------------+
|                                 PAN/TILT GIMBAL CONTROL                                |
|   - Dual-Axis Discrete PID Controller with Anti-Windup & Filtered Derivative           |
|   - Electro-Mechanical Motor Lag (tau=25ms), Max Slew (35 deg/s), Accel (120 deg/s^2)  |
|   - Archimedean Spiral Pattern Generator for Open-Loop Autonomous Search               |
+----------------------------------------------------------------------------------------+
```

---

## 3. Ground Truth vs. Control Isolation

To guarantee scientific validity, the virtual laboratory maintains complete physical isolation between **Ground Truth** and the **Autonomous Control Loop**:
- **Ground Truth**: Exact 3D world target positions, velocities, orientation, true line-of-sight unit vector, true angular errors, and exact optical projection coordinates. This data is **strictly shielded** from the detector, tracker, and gimbal controller.
- **Estimated State**: The detector only processes pixel arrays from the degraded synthetic camera feed. The tracker estimates kinematics from observed pixel centroids. The gimbal controller acts purely on estimated angular misalignments relative to the optical crosshair.
- **Evaluator Validation**: Ground truth is used exclusively by the real-time analytics module, KPI cards, telemetry graphs, and automated benchmark verification engines.

---

## 4. Quick Start & Execution

### Prerequisites
- Python 3.10, 3.11, 3.12, or 3.13 (64-bit recommended)
- Windows, Linux, or macOS

### Installation
Clone or navigate to the workspace directory and install dependencies:
```bash
python -m venv .venv
# Windows:
.\.venv\Scripts\activate
# Linux/macOS:
source .venv/bin/activate

pip install -r requirements.txt
```

### Running the Application

1. **Interactive Desktop Application (Default)**
   Launches the embedded high-performance simulation server and opens the native mission-control interface:
   ```bash
   python main.py
   ```

2. **Automated Batch Benchmark Suite**
   Executes identical-seed comparative evaluations across all 14 standard scenarios and prints comparative metrics:
   ```bash
   python main.py --benchmark
   ```

3. **Performance Report Generation**
   Generates an empirical Markdown verification report:
   ```bash
   python main.py --report
   ```

4. **Headless Server Mode**
   Runs the simulation server on port 8765 for automated testing or remote headless environments:
   ```bash
   python main.py --headless --port 8765
   ```

---

## 5. Automated Test Suite

Run the full automated test suite covering unit tests, kinematics, optics, disturbances, state machine transitions, and closed-loop end-to-end integration:
```bash
pytest -v
```

---

## 6. Building the Standalone Executable

The system packages into a single, standalone `.exe` with zero external runtime dependencies:
```powershell
.\.venv\Scripts\python.exe -m PyInstaller --onefile --windowed --name FSOC_Virtual_Tracker --add-data "ui/static;ui/static" main.py
```
The resulting binary is located at `dist/FSOC_Virtual_Tracker.exe`. Simply double-click to launch the complete offline virtual laboratory.

---

## 7. 14 Standard Benchmark Scenarios

The suite includes 14 scientifically calibrated, seed-controlled benchmark scenarios:
1. **Static Target**: Calibration baseline; tests stationary step response and steady-state pointing jitter.
2. **Slow Target**: Linear motion at $2.5\text{ m/s}$; evaluates tracking lag and PID integral action.
3. **Fast Target**: High-speed crossing target at $18.0\text{ m/s}$; evaluates slew-rate limits and Kalman velocity tracking.
4. **Accelerating Target**: Target undergoing $+1.5\text{ m/s}^2$ acceleration; validates 6-state Kalman acceleration states.
5. **Crossing Target**: Angled diagonal trajectory crossing optical axis; tests cross-axis gimbal coupling.
6. **Multiple Targets & Distractors**: Primary 850nm beacon with unmodulated glint and 5Hz distractor; verifies AI spectral rejection.
7. **Sensor Noise**: High Gaussian readout noise ($\sigma=22.0$) and salt-and-pepper impulses; tests morphological spatial filtering.
8. **Platform Vibration**: Multi-harmonic airframe oscillations ($24\text{ Hz}$ and $65\text{ Hz}$); evaluates control rejection of structural jitter.
9. **Atmospheric Turbulence**: Kolmogorov scintillation index $\sigma_I^2=0.45$ and beam wander; tests robustness against optical fade.
10. **Dynamic Mobile Base Motion**: Host vehicle roll, pitch, and yaw oscillations; tests coordinate transformation compensation.
11. **Temporary Line-of-Sight Occlusion**: 2.0-second complete beacon obstruction; validates dead-reckoning forward projection and reacquisition.
12. **Target Loss & Wide Area Search**: Permanent target departure; triggers state transition to `LOST` and Archimedean spiral search.
13. **Combined Multi-Disturbance**: Simultaneous scintillation, vibration, noise, and base motion; realistic field test.
14. **High-Stress Operational Test**: Maximum velocity ($22\text{ m/s}$), severe turbulence ($\sigma_I^2=0.60$), and vibration ($1.2\text{ mrad}$).

---

## 8. Repository Structure

```
├── core/                       # Foundational physical & optical modeling
│   ├── coordinates.py          # SE(3) kinematics & coordinate transformations
│   ├── camera_model.py         # Pinhole optics, PSF, and radiometric projection
│   ├── targets.py              # Multi-target kinematics & optical beacon signatures
│   ├── disturbances.py         # Scintillation, beam wander, jitter, noise models
│   └── state_machine.py        # Formal 7-state coarse alignment state machine
├── detection/                  # Computer vision & AI/ML detection
│   ├── baseline_cv.py          # Classical morphological & centroid detector
│   ├── ai_detector.py          # 12-D feature classifier & diffraction correlator
│   └── id_matcher.py           # Pulse modulation frequency estimator (FFT)
├── tracking/                   # Temporal estimation & filtering
│   ├── kalman_tracker.py       # 6-state discrete Kalman filter
│   ├── predictor.py            # Lookahead dead-reckoning & uncertainty ellipses
│   └── association.py          # Gated nearest-neighbor candidate association
├── control/                    # Gimbal pointing & search patterns
│   ├── gimbal_controller.py    # 2-axis Pan/Tilt PID controller with dynamics
│   └── scan_patterns.py        # Archimedean spiral & reacquisition scanners
├── simulation/                 # Simulation orchestration & benchmarking
│   ├── engine.py               # Closed-loop unified simulation engine
│   ├── ground_truth.py         # Shielded ground-truth telemetry provider
│   ├── scenarios.py            # 14 standard reproducible scenarios
│   └── benchmark.py            # Batch comparative benchmark engine
├── analytics/                  # Performance metrics & reporting
│   ├── metrics.py              # Real-time running statistics calculator
│   ├── logger.py               # Experiment session persistence (JSON/CSV)
│   └── report_generator.py     # Markdown and HTML report generation
├── ui/                         # Mission control user interface
│   ├── app.py                  # Standalone desktop window launcher
│   ├── server.py               # Local HTTP API & streaming server
│   └── static/                 # Embedded frontend client
│       ├── index.html          # Mission-control dashboard layout
│       ├── css/style.css       # Aerospace technical control-room styling
│       └── js/                 # Three.js 3D world, Canvas HUD, Chart.js
├── tests/                      # Automated test suite (44 tests)
├── requirements.txt            # Python dependencies
├── main.py                     # Top-level executable entry point
├── ARCHITECTURE.md             # In-depth engineering architectural specification
├── USER_MANUAL.md              # Operator user manual & scenario walkthrough
├── TECHNICAL_REPORT.md         # Formal 15-page technical report for ISRO
├── TEST_REPORT.md              # Test execution results & validation logs
└── REQUIREMENTS_TRACEABILITY.md# Official requirement-to-evidence compliance matrix
```

---

## 9. Engineering Disclaimers

As specified by the official problem statement:
- Any parameter or threshold selected internally is explicitly designated as **SYSTEM-DEFINED SIMULATION PARAMETER** (e.g., $0.50^\circ$ coarse alignment lock threshold).
- Physical approximations of environmental or actuator behaviors are designated as **ENGINEERING ASSUMPTIONS** (e.g., $25\text{ ms}$ electro-mechanical motor lag constant).
