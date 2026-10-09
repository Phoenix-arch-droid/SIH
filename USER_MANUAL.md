# User Manual: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

**Document Code:** UM-SIH26169-V1.0  
**Target Audience:** Systems Engineers, FSOC Operators, Test Engineers, Evaluation Jury  
**Software Version:** 1.0.0-PROD (ISRO SIH26169)  
**Classification:** Operational Guide & Operator Manual  

---

## 1. System Introduction & Product Concept

The **Virtual FSOC Coarse-Alignment and Tracking Laboratory** is an interactive, hardware-in-the-loop (HIL) capable simulation software for developing, validating, and benchmarking coarse alignment algorithms for mobile Free Space Optical Communication (FSOC) terminals.

In typical aerospace and maritime FSOC deployments, initial platform positional uncertainties ($\approx 1^\circ - 3^\circ$) prevent immediate fine optical coupling. The coarse tracking camera acquires the remote terminal's optical beacon within a moderate field of view ($15.0^\circ$), discriminates it from background clutter and solar glint, tracks its trajectory using a 6-state Kalman filter, and drives a 2-axis Pan/Tilt gimbal to bring the pointing error within the fine-pointing acquisition basket ($\le 0.50^\circ$ or $8.73\,\text{mrad}$).

This software provides a fully integrated, offline, standalone environment featuring:
- A physics-based **3D Tactical World View** modeling mobile host platform kinematics, remote beacon positions, line-of-sight geometry, and gimbal orientation.
- A **Synthetic Optical Camera View** with realistic Point Spread Function (PSF), radiometric beam propagation, and optical reticle HUD overlays.
- A **Configurable Environmental Disturbance Engine** injecting Kolmogorov atmospheric scintillation, beam wander, platform vibrations, sensor readout noise, and cloud occlusions.
- Dual-mode tracking comparing a **Classical Computer Vision Baseline** with an **AI/ML Enhanced Detector**.
- Real-time telemetry, synchronized multi-channel scrolling charts, automated benchmarking, and exportable audit logs.

---

## 2. System Requirements & Installation

### 2.1 Hardware Requirements
- **Processor:** Intel Core i5 / AMD Ryzen 5 or higher (quad-core minimum, 2.0 GHz+).
- **RAM:** 4 GB RAM minimum (8 GB recommended).
- **Display Resolution:** $1366 \times 768$ minimum ($1920 \times 1080$ full HD recommended for optimal multi-panel dashboard display).
- **Graphics:** OpenGL 3.3+ capable integrated or discrete GPU (for Three.js WebGL 3D rendering).
- **Disk Space:** 250 MB free disk space.

### 2.2 Software Environment
- **Operating System:** Windows 10/11 (64-bit), Ubuntu 20.04/22.04 LTS, or macOS 12+.
- **Python Version:** Python 3.10, 3.11, 3.12, or 3.13 (64-bit).
- **Zero Cloud / Air-Gapped Operation:** All client libraries (Three.js, OrbitControls, Chart.js) and detector weights are bundled locally. No active internet connection or cloud service accounts are required.

### 2.3 Installation Steps
```bash
# 1. Open a terminal / command prompt in the project root directory
cd "C:\Users\phoen\Downloads\New folder"

# 2. Activate the existing virtual environment (or create a new one)
.\.venv\Scripts\activate

# 3. Verify installed dependencies
pip install -r requirements.txt
```

---

## 3. Launching the Application

The application provides multiple execution modes to suit interactive demonstrations, headless automated testing, and batch benchmarking:

### 3.1 Mode 1: Interactive Desktop Application (Recommended)
```bash
python main.py
```
- Automatically initializes the simulation engine at 30 FPS.
- Spawns the local simulation server on port 8765.
- Opens your desktop browser to `http://127.0.0.1:8765/index.html` in an aerospace mission-control console.

### 3.2 Mode 2: Automated 14-Scenario Comparative Benchmark
```bash
python main.py --benchmark
```
- Executes identical-seed comparative evaluations across all 14 standard scenarios.
- Compares Baseline Classical CV vs AI/ML Enhanced detector under identical trajectories and disturbances.
- Displays an ASCII benchmark comparison matrix showing RMS error reduction, lock retention rates, and processing latencies.

### 3.3 Mode 3: Official Verification Report Generation
```bash
python main.py --report
```
- Runs Scenario 1 (Static Target Baseline) to completion.
- Evaluates full telemetry and ground-truth vectors.
- Emits a formal verification log to `FSOC_Performance_Report.md`.

### 3.4 Mode 4: Headless Simulation Server
```bash
python main.py --headless --port 9000
```
- Starts the simulation engine and REST API on port 9000 without popping a desktop browser window. Ideal for automated testing pipelines and remote evaluations.

---

## 4. Mission-Control Dashboard Walkthrough

The user interface is designed following professional aerospace ground station aesthetics (dark charcoal background `#0a0d14`, restrained neon accents, glassmorphic cards, high data density).

```
+-------------------------------------------------------------------------------------------------------+
|  ISRO SIH26169: VIRTUAL FSOC COARSE ALIGNMENT & TRACKING LABORATORY                                  |
|  [STATE: LOCKED] | [LOCK: 94.2%] | [FPS: 30.1] | [ERR: 0.18 deg] | [CONF: 96%] | [LATENCY: 4.2 ms]    |
+-------------------------------------------------------------------------------------------------------+
|                                    |                                                                  |
|               PANEL 1:             |                             PANEL 2:                             |
|          3D TACTICAL WORLD         |                       SYNTHETIC CAMERA FEED                      |
|                                    |                                                                  |
|  - Host Platform & Remote Terminal |  - 1280x720 Optical Sensor Frame                                 |
|  - Rotating Pan/Tilt Gimbal        |  - Crosshair Reticle & Boresight Center                          |
|  - Dynamic FOV Frustum Pyramid     |  - Optical Beacon PSF Spot & Distortion                          |
|  - LOS Vector & Boresight Vector   |  - Green Kalman Tracking Bounding Box                            |
|  - Ground Truth & Prediction Trails|  - Purple 3-Sigma Covariance Ellipse                             |
|  - Interactive 3D Orbit Controls   |  - Target ID & Confidence Tag Overlay                            |
|                                    |                                                                  |
+------------------------------------+------------------------------------------------------------------+
|                                    |                                                                  |
|               PANEL 3:             |                             PANEL 4:                             |
|         OPERATOR CONTROLS          |                    REAL-TIME SCROLLING CHARTS                    |
|                                    |                                                                  |
|  - Play / Pause / Step / Reset     |  - Pointing Error (deg & mrad vs 0.50 deg Lock Threshold)        |
|  - Mode: Baseline CV / AI Enhanced |  - Detection & Tracking Confidence History                       |
|  - [ONE-CLICK FSOC ALIGNMENT DEMO] |  - System Processing FPS & Frame Latency (ms)                    |
|  - Scenario Selector (1 to 14)     |  - Pan / Tilt Gimbal Angular Positions (deg)                     |
|  - Disturbance Presets & Sliders   |                                                                  |
|  - Export Telemetry (JSON / CSV)   |                                                                  |
|                                    |                                                                  |
+------------------------------------+------------------------------------------------------------------+
```

### 4.1 Top KPI Telemetry Strip
- **System State:** Displays the current operational mode of the 7-state alignment state machine (`SEARCHING`, `ACQUIRING`, `TRACKING`, `LOCKED`, `DEGRADED`, `LOST`, `REACQUIRING`).
- **Lock Status Badge:** Active green indicator when total angular pointing error is within the coarse alignment boundary ($\le 0.50^\circ$ or $8.73\,\text{mrad}$).
- **Simulation FPS:** Real-time processing frame rate (target: $\ge 30.0\,\text{FPS}$).
- **True Angular Error:** Exact spherical angular separation between the camera boresight and the true target line-of-sight vector (computed from isolated ground truth).
- **Tracking Confidence:** Normalized probability ($0.0 - 1.0$) generated by the 6-state Kalman filter and AI classifier.
- **Processing Latency:** Execution duration of the entire frame loop in milliseconds.

### 4.2 3D Tactical World View (Panel 1)
- **Host Platform:** Silver aerospace vehicle located at the world coordinate origin $(0, 0, 0)$.
- **Remote Terminal:** Blue mobile terminal carrying the active optical beacon.
- **Camera Turret:** Physically rendered 2-axis gimbal assembly that rotates in real time in response to controller commands.
- **FOV Frustum Cone:** Translucent green pyramidal volume visualizing the camera's $15.0^\circ \times 8.44^\circ$ spatial acceptance angle.
- **Pointing Vectors:**
  - *Yellow Line:* True line-of-sight (LOS) vector from the camera optical center to the remote beacon.
  - *Cyan Line:* Actual camera optical boresight pointing vector.
  - *Red Arc:* Instantaneous angular pointing error.
- **Trajectory Trails:** Continuous paths showing true historical flight paths and 100 ms forward dead-reckoning predictions.
- **Interactive Orbit Controls:**
  - *Left Click + Drag:* Orbit camera in 3D around the gimbal platform.
  - *Right Click + Drag:* Pan the 3D tactical perspective.
  - *Scroll Wheel:* Zoom in and out.

### 4.3 Synthetic Optical Camera Sensor Feed (Panel 2)
- High-fidelity optical simulation rendering at $1280 \times 720$ resolution.
- Renders diffraction-limited optical beacon spots with range-dependent Airy disk PSF blur.
- Includes dynamic background illumination and atmospheric scintillation intensity fading.
- **Reticle Crosshairs:** High-precision optical boresight markings centered at $(640, 360)$ pixels.
- **Bounding Box (Green):** Current spatial localization bounding candidate beacon regions.
- **Uncertainty Ellipse (Purple):** 3-sigma spatial error covariance ellipse computed from the Kalman filter covariance matrix $P$.
- **Telemetry HUD:** Displays Target ID, centroid pixel coordinates $(u, v)$, detection confidence %, and estimated angular offset.

### 4.4 Real-Time Analytics Graphs (Panel 4)
- Synchronized multi-channel rolling strip charts updating dynamically at 30 FPS.
- **Graph 1 (Pointing Error):** Live plot of pointing error in degrees and milliradians with a dashed red line marking the $0.50^\circ$ coarse alignment lock threshold.
- **Graph 2 (Confidence):** Detection confidence and Kalman track state stability.
- **Graph 3 (System Dynamics):** Processing FPS and compute latency margin ($< 33.3\,\text{ms}$).
- **Graph 4 (Gimbal Angles):** Commanded vs actual pan and tilt angles demonstrating electro-mechanical motor lag and slew rate clamping.

---

## 5. Operator Controls & Scenario Execution

### 5.1 Primary Simulation Controls
- **RUN / PAUSE:** Starts or freezes the real-time simulation clock.
- **STEP:** Advances the simulation by exactly one discrete time step ($\Delta t = 33.3\,\text{ms}$) for detailed forensic frame inspection.
- **RESET:** Resets simulation time to $0.0\,\text{s}$, centers the gimbal, and clears tracking filters.

### 5.2 Tracking Mode Selection
- **Baseline (Classical CV):** Uses top-hat morphological filtering, Otsu dynamic thresholding, and subpixel moment centroiding. Fast, lightweight, and effective in clean environments.
- **Enhanced (AI/ML Detector):** Uses 12-dimensional spatio-spectral feature classification and 2D Airy pattern diffraction template matching. Exhibits superior noise rejection, glint discrimination, and scintillation robustness.

### 5.3 One-Click FSOC Coarse Alignment Demonstration
Clicking the purple **"FSOC Alignment Demo"** button triggers an automated 18-step demonstration sequence showcasing the entire problem lifecycle:
1. System initializes in `SEARCHING` mode with Archimedean spiral scanning.
2. Optical beacon appears in the camera field of view.
3. Detector acquires beacon candidate; state advances to `ACQUIRING`.
4. 20 Hz temporal pulse modulation frequency is confirmed; state advances to `TRACKING`.
5. 2-axis Pan/Tilt gimbal slews toward target, reducing image-space error.
6. Angular error drops below $0.50^\circ$; state enters `LOCKED`.
7. Target initiates accelerating lateral maneuver; gimbal tracks smoothly with feedforward velocity compensation.
8. Severe platform vibration ($24\,\text{Hz} + 65\,\text{Hz}$) and atmospheric scintillation are injected.
9. System transitions to `DEGRADED`; Kalman filter covariance ellipse expands while maintaining track lock.
10. Target suffers 2.0-second line-of-sight cloud occlusion; detector observations drop to zero.
11. Kalman filter maintains forward dead-reckoning trajectory extrapolation.
12. Target re-emerges; system performs rapid reacquisition; `LOCKED` state is restored.
13. Complete performance log summary is compiled and displayed.

### 5.4 Selecting Standard Scenarios
Select any of the 14 standard scenarios from the dropdown menu:
- `Scenario 1: Static Target Baseline`
- `Scenario 2: Slow Moving Target (2.5 m/s)`
- `Scenario 3: Fast Crossing Target (18.0 m/s)`
- `Scenario 4: Accelerating Target (1.5 m/s^2)`
- `Scenario 5: Crossing Diagonal Target`
- `Scenario 6: Multiple Targets & Distractor Glints`
- `Scenario 7: Severe Sensor Readout Noise`
- `Scenario 8: Multi-Harmonic Platform Vibration`
- `Scenario 9: Atmospheric Scintillation & Beam Wander`
- `Scenario 10: Dynamic Mobile Base Motion (R, P, Y)`
- `Scenario 11: Temporary Line-of-Sight Occlusion (2.0s)`
- `Scenario 12: Permanent Target Loss & Spiral Search`
- `Scenario 13: Combined Multi-Disturbance Mission`
- `Scenario 14: High-Stress Operational Envelope`

### 5.5 Runtime Disturbance Injection
Use the **Disturbance Level** radio buttons to dynamically modify environmental perturbations on the fly:
- **OFF:** Zero noise, perfect optical transmission.
- **LOW:** Light ambient daylight, mild beam wander ($0.2\,\text{mrad}$).
- **MEDIUM:** Realistic aerospace turbulence ($\sigma_I^2 = 0.25$, $0.5\,\text{mrad}$ vibration).
- **HIGH:** Severe turbulence ($\sigma_I^2 = 0.50$, $1.0\,\text{mrad}$ vibration, sensor noise $\sigma = 20.0$).
- **CUSTOM:** Manually tune scintillation index, vibration amplitude, noise std-dev, and occlusion intervals.

### 5.6 Telemetry Persistence & Replay
- **Save Experiment (JSON):** Serializes the complete time-series history, ground truth, detector outputs, and configuration to a timestamped `.json` file.
- **Export Telemetry (CSV):** Dumps frame-by-frame numerical columns (timestamps, coordinates, pointing errors, gimbal angles) for external MATLAB, Python, or Excel analysis.
- **Replay Experiment:** Loads a previously saved `.json` recording to replay the exact optical frames and telemetry sequence deterministically.

---

## 6. Troubleshooting & Frequently Asked Questions

### Q1: The browser page doesn't open automatically on launch.
**A:** If your operating system blocks browser popups, open your web browser manually and navigate to:  
`http://127.0.0.1:8765/index.html`

### Q2: Port 8765 is already in use by another application.
**A:** Launch the application with the `--port` argument to specify an alternate port:
```bash
python main.py --port 8899
```

### Q3: How do I verify that ground truth is not being used for control?
**A:** Inspect `core/state_machine.py` and `control/gimbal_controller.py`. The control loop inputs are derived strictly from `est_pan_err_deg` and `est_tilt_err_deg`, which are calculated from the Kalman filter's pixel centroid estimate relative to $(c_x, c_y)$. Ground truth data (`snapshot.gt`) is populated in a separate struct and is only consumed by the telemetry recorder and analytics module.

### Q4: The 3D world view shows a black screen.
**A:** Ensure your graphics driver supports WebGL. Check that hardware acceleration is enabled in your browser settings (`chrome://settings/system` or `edge://settings/system`).
