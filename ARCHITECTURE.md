# Architectural Specification: AI-Based Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals

**Document ID:** ARCH-SIH26169-V1.0  
**Organization:** Indian Space Research Organisation (ISRO)  
**Department:** Department of Space / ISRO  
**Theme:** Smart Automation  
**Classification:** Technical Architecture Specification  

---

## 1. Executive Architectural Overview

Free Space Optical Communication (FSOC) provides an order-of-magnitude increase in channel capacity compared to conventional RF cross-links, while maintaining sub-milliradian beam divergence to prevent eavesdropping and eliminate spectral contention. However, the extreme spatial narrowness of optical transmission beams imposes strict pointing tolerances: establishing and maintaining an optical link between moving platforms (e.g., LEO satellites, UAVs, airborne sensor craft, maritime vessels) requires a multi-stage Pointing, Acquisition, and Tracking (PAT) architecture.

The **coarse alignment system** bridges the gap between coarse GNSS/ephemeris knowledge ($\sim 1^\circ - 3^\circ$ uncertainty) and fine-tracking fast steering mirrors (FSMs, $\sim 50 - 200\,\mu\text{rad}$ linear range). The coarse tracking camera must detect an incoming beacon within a moderate field of view ($10^\circ - 20^\circ$), identify it against ambient glints, estimate its trajectory, and command a two-axis gimbal mechanism to center the beacon within the capture basket ($\le 0.50^\circ$ or $8.73\,\text{mrad}$) of the optical receiver.

This system implements a completely software-defined virtual testing laboratory that faithfully models the end-to-end electro-optical, environmental, and control dynamics of coarse alignment without requiring specialized physical cleanroom testbeds.

```
+---------------------------------------------------------------------------------------------------+
|                                     CLOSED-LOOP SIMULATION PIPELINE                               |
|                                                                                                   |
|   +-------------------+       +--------------------+       +--------------------+                 |
|   | 3D Virtual World  | ----> | Pinhole Camera     | ----> | Disturbance Engine |                 |
|   | Kinematics SE(3)  |       | Projection & PSF   |       | Noise & Scintill.  |                 |
|   +-------------------+       +--------------------+       +--------------------+                 |
|             |                           |                            |                            |
|             | (Shielded GT)             v                            v                            |
|             |                 +--------------------+       +--------------------+                 |
|             |                 | Optical Frame Gen  | ----> | Dual-Mode Detector |                 |
|             |                 | Radiometric Synth  |       | Baseline CV / AI   |                 |
|             |                 +--------------------+       +--------------------+                 |
|             |                                                        |                            |
|             |                                                        v                            |
|             |                 +--------------------+       +--------------------+                 |
|             |                 | 6-State Kalman     | <---- | ID Matcher         |                 |
|             |                 | Tracker/Predictor  |       | 20Hz Rolling FFT   |                 |
|             |                 +--------------------+       +--------------------+                 |
|             |                           |                                                         |
|             |                           v                                                         |
|             |                 +--------------------+       +--------------------+                 |
|             |                 | 7-State Alignment  | ----> | Pan/Tilt Gimbal    |                 |
|             |                 | State Machine      |       | PID & Slew Limits  |                 |
|             |                 +--------------------+       +--------------------+                 |
|             |                                                        |                            |
|             +---------------------------------------+                | (Gimbal Actuation)         |
|                                                     |                v                            |
|                                                     |      +--------------------+                 |
|                                                     +----> | Camera Orientation |                 |
|                                                            | Update Next Frame  |                 |
|                                                            +--------------------+                 |
+---------------------------------------------------------------------------------------------------+
```

---

## 2. Mathematical Kinematics & Coordinate Transformations

### 2.1 Coordinate Frame Hierarchy
The laboratory establishes four coordinate reference systems:
1. **World Inertial Frame $\mathcal{F}_W$**: Right-handed Cartesian system $(X_W, Y_W, Z_W)$ where $+X$ is East, $+Y$ is North, and $+Z$ is Zenith.
2. **Platform Body Frame $\mathcal{F}_B$**: Fixed to the moving host vehicle, translated by $\mathbf{p}_B(t)$ and rotated by body Euler angles (roll $\phi$, pitch $\theta$, yaw $\psi$).
3. **Gimbal Base Frame $\mathcal{F}_G$**: Positioned at the gimbal pivot center.
4. **Camera Optical Sensor Frame $\mathcal{F}_C$**: Follows standard computer vision convention where $+Z_C$ is along the optical boresight (depth), $+X_C$ points to sensor right, and $+Y_C$ points down.

### 2.2 Rotation Parameterization
Platform attitude is expressed through direction cosine matrices $R_{W \to B}(\phi, \theta, \psi) \in \mathrm{SO}(3)$:
$$
R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & \sin\phi \\ 0 & -\sin\phi & \cos\phi \end{bmatrix},\quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & -\sin\theta \\ 0 & 1 & 0 \\ \sin\theta & 0 & \cos\theta \end{bmatrix},\quad
R_z(\psi) = \begin{bmatrix} \cos\psi & \sin\psi & 0 \\ -\sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
$$
The composite platform rotation is $R_P = R_x(\phi) R_y(\theta) R_z(\psi)$.

For the camera pan/tilt gimbal:
- Pan (azimuth $\alpha$) rotates about the platform vertical axis.
- Tilt (elevation $\beta$) rotates about the horizontal gimbal trunnion.
The rotation from world coordinates into the camera optical frame is:
$$
\mathbf{p}_C = R_{optical} R_{tilt}(\beta) R_{pan}(\alpha) R_P (\mathbf{p}_W - \mathbf{p}_{camera})
$$
where $R_{optical}$ maps aerospace forward $(+Y)$ to optical boresight $(+Z)$ and vertical $(+Z)$ to optical inverted-down $(-Y)$:
$$
R_{optical} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix}
$$

### 2.3 Angular Error Formulation
Given the true unit pointing vector $\hat{\mathbf{v}}_{pointing}$ and target line-of-sight unit vector $\hat{\mathbf{v}}_{target}$:
$$
\theta_{error} = \arccos\left(\text{clamp}(\hat{\mathbf{v}}_{pointing} \cdot \hat{\mathbf{v}}_{target}, -1.0, 1.0)\right)
$$
This total 3D spherical error is decomposed into azimuth and elevation error components:
$$
\Delta\alpha = \arctan2(\hat{v}_{target, x}, \hat{v}_{target, y}) - \alpha,\quad
\Delta\beta = \arcsin(\hat{v}_{target, z}) - \beta
$$

---

## 3. Virtual Camera Optics & Radiometric Projection

### 3.1 Pinhole Intrinsics
The virtual sensor is calibrated with the following system-defined parameters:
- Resolution: $W = 1280$, $H = 720$ pixels.
- Horizontal Field of View (HFOV): $\theta_H = 15.0^\circ$ ($0.2618\,\text{rad}$).
- Vertical Field of View (VFOV): 
  $$\theta_V = 2 \arctan\left(\tan\left(\frac{\theta_H}{2}\right) \times \frac{H}{W}\right) \approx 8.44^\circ\;(0.1473\,\text{rad})$$
- Principal Point: $(c_x, c_y) = (640.0, 360.0)$.
- Focal Lengths:
  $$f_x = \frac{W / 2}{\tan(\theta_H / 2)} = \frac{640}{\tan(7.5^\circ)} \approx 4859.39\,\text{px},\quad f_y = f_x$$

### 3.2 3D-to-2D Projection & Lens Distortion
For a 3D point $\mathbf{p}_C = (X_C, Y_C, Z_C)^T$ in front of the lens ($Z_C > 0$):
$$
x_n = \frac{X_C}{Z_C},\quad y_n = \frac{Y_C}{Z_C},\quad r^2 = x_n^2 + y_n^2
$$
Applying radial distortion ($k_1 = -0.015, k_2 = 0.002$):
$$
x_d = x_n (1 + k_1 r^2 + k_2 r^4),\quad y_d = y_n (1 + k_1 r^2 + k_2 r^4)
$$
Pixel coordinates:
$$
u = f_x x_d + c_x,\quad v = f_y y_d + c_y
$$

### 3.3 Radiometric Irradiance & Airy Point Spread Function (PSF)
An optical beacon with transmitter power $P_{tx}$ and divergence angle $\theta_{div}$ creates an irradiance at range $R$:
$$
E(R) = \frac{4 P_{tx}}{\pi (R \theta_{div})^2} \exp(-\gamma R)
$$
where $\gamma = 10^{-4}\,\text{m}^{-1}$ represents atmospheric Beer-Lambert attenuation.
The optical spot is rendered on the sensor array using a 2D Gaussian approximation of the diffraction-limited Airy disk:
$$
I(x, y) = I_{peak} \exp\left(-\frac{(x - u)^2 + (y - v)^2}{2\sigma_{psf}^2}\right)
$$
where $\sigma_{psf} = \max\left(1.2, \frac{1.22 \lambda R}{D_{aperture} \cdot p_{size}}\right)$ pixels.

---

## 4. Physical Disturbance Engine

The disturbance engine injects physically realistic environmental and mechanical perturbations:

### 4.1 Atmospheric Scintillation & Beam Wander
- **Kolmogorov Scintillation**: Log-normal intensity fluctuations parameterized by scintillation index $\sigma_I^2 \in [0, 0.6]$:
  $$I_{factor} = \exp\left(\mu_x + \sigma_x \mathcal{N}(0, 1)\right),\quad \mu_x = -\frac{\sigma_x^2}{2},\quad \sigma_x = \sqrt{\ln(1 + \sigma_I^2)}$$
- **Atmospheric Beam Wander**: Low-frequency stochastic angular drift ($f \approx 0.5 - 2\,\text{Hz}$) modeled as a continuous Gauss-Markov random process:
  $$\dot{\theta}_{wander} = -\frac{1}{\tau_w} \theta_{wander} + \sigma_w \sqrt{\frac{2}{\tau_w}} w(t)$$

### 4.2 Platform Structural Vibration
Multi-harmonic airframe and gimbal engine vibrations:
$$
\theta_{vib}(t) = \sum_{k=1}^M A_k \sin(2\pi f_k t + \phi_k)
$$
with primary harmonic at $f_1 = 24.0\,\text{Hz}$ (aerodynamic buffeting) and secondary at $f_2 = 65.0\,\text{Hz}$ (gimbal motor harmonics).

### 4.3 Sensor Noise & Optical Degradation
- **Readout Noise**: Additive Gaussian white noise $\mathcal{N}(0, \sigma_{read}^2)$.
- **Shot Noise**: Signal-dependent Poisson noise $\mathcal{P}(\lambda)$.
- **Impulse Noise**: Random hot/dead pixels with salt-and-pepper probability $p_{sp} \in [0, 0.01]$.
- **Cloud/Obstruction Occlusion**: Temporal step attenuation $I_{trans} = 0$ during specified time intervals $[t_{start}, t_{start} + \Delta t]$.

---

## 5. Detection & Identification Architecture

### 5.1 Dual-Mode Detection Pipeline
The system integrates two interchangeable detection architectures to enable rigorous comparative evaluation:

```
                  +--------------------------------+
                  |  Degraded Sensor Frame Image   |
                  +---------------+----------------+
                                  |
            +---------------------+---------------------+
            |                                           |
            v                                           v
+-----------------------+                   +-----------------------+
|  MODE A: BASELINE CV  |                   |  MODE B: AI ENHANCED  |
|  - Morphological Top  |                   |  - Candidate Saliency |
|    Hat Normalization  |                   |  - 12-D Feature Vector|
|  - Otsu Thresholding  |                   |  - Airy Correlator    |
|  - Subpixel Moments   |                   |  - Logistic Regressor |
+-----------+-----------+                   +-----------+-----------+
            |                                           |
            +---------------------+---------------------+
                                  |
                                  v
                    +---------------------------+
                    | Candidate Detections List |
                    +-------------+-------------+
                                  |
                                  v
                    +---------------------------+
                    |  Temporal ID Matcher      |
                    |  20Hz Pulse FFT Analysis  |
                    +-------------+-------------+
                                  |
                                  v
                    +---------------------------+
                    | Verified Target Candidate |
                    +---------------------------+
```

### 5.2 Mode A: Baseline Classical Computer Vision
1. **Morphological Top-Hat Transform**: Eliminates non-uniform background daylight gradients:
   $$T = I - (I \circ B)$$
   where $B$ is a $9\times 9$ flat disk structuring element.
2. **Adaptive Dynamic Thresholding**: Sets threshold $T_{dyn} = \mu_T + 4.5 \sigma_T$.
3. **Contour Extraction & Subpixel Moments**: For each candidate region:
   $$u_c = \frac{m_{10}}{m_{00}},\quad v_c = \frac{m_{01}}{m_{00}},\quad m_{pq} = \sum_x \sum_y x^p y^q I(x, y)$$

### 5.3 Mode B: AI/ML Enhanced Detector
1. **12-Dimensional Spatio-Spectral Feature Extraction**: For every candidate connected component:
   - $f_1$: Normalized peak intensity.
   - $f_2$: Signal-to-Noise Ratio (SNR).
   - $f_3$: Area compactness ($4\pi A / P^2$).
   - $f_4$: Aspect ratio elongation ($b / a$).
   - $f_5$: Equivalent circular diameter.
   - $f_6$: Spot spatial variance $\sigma_{spot}^2$.
   - $f_7$: Skewness of intensity distribution.
   - $f_8$: Kurtosis of intensity distribution.
   - $f_9$: Radial symmetry metric.
   - $f_{10}$: Pearson cross-correlation coefficient with ideal 2D Airy diffraction kernel:
     $$\rho = \frac{\sum (I - \bar{I})(K - \bar{K})}{\sqrt{\sum (I - \bar{I})^2 \sum (K - \bar{K})^2}}$$
   - $f_{11}$: Gradient magnitude boundary sharpness.
   - $f_{12}$: Local background contrast ratio.
2. **Calibrated Logistic Classification**: Evaluates beacon likelihood:
   $$P(\text{Beacon} \mid \mathbf{f}) = \frac{1}{1 + \exp\left(-\mathbf{w}^T \mathbf{f} - b\right)}$$
   Rejecting solar glints, hot pixels, and structural specular reflections.

### 5.4 Temporal Optical ID Matcher
To distinguish the designated optical beacon from false transmitters, the beacon pulses at $f_{mod} = 20.0\,\text{Hz}$. The ID Matcher records the candidate's intensity history over a rolling buffer of 32 frames and computes discrete Fourier transform power:
$$
P(f) = \left|\sum_{n=0}^{N-1} I_n \exp\left(-j \frac{2\pi k n}{N}\right)\right|^2
$$
A detection is accepted only if peak spectral energy aligns within $\pm 2.5\,\text{Hz}$ of $20.0\,\text{Hz}$.

---

## 6. Temporal Tracking & Forward Prediction

### 6.1 6-State Discrete Kalman Filter
The state vector tracks image-plane coordinates, velocities, and accelerations:
$$
\mathbf{x} = \begin{bmatrix} u & v & \dot{u} & \dot{v} & \ddot{u} & \ddot{v} \end{bmatrix}^T \in \mathbb{R}^6
$$
State transition matrix under constant-acceleration motion with time step $\Delta t$:
$$
F = \begin{bmatrix}
1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 & 0 \\
0 & 1 & 0 & \Delta t & 0 & \frac{1}{2}\Delta t^2 \\
0 & 0 & 1 & 0 & \Delta t & 0 \\
0 & 0 & 0 & 1 & 0 & \Delta t \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$
Measurement matrix observing pixel centroids:
$$
H = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 & 0 & 0 \end{bmatrix}
$$
Measurement noise covariance adapts dynamically to detection SNR:
$$
R(\text{SNR}) = \left(\sigma_{base}^2 + \frac{\alpha}{\max(1.0, \text{SNR})}\right) I_2
$$

### 6.2 Forward Dead-Reckoning & Uncertainty Ellipses
During missed frames or occlusions, the filter propagates state estimates forward without measurement updates:
$$
\hat{\mathbf{x}}_{k+m} = F^m \hat{\mathbf{x}}_k,\quad P_{k+m} = F P_{k+m-1} F^T + Q
$$
The 3-sigma spatial uncertainty ellipse on the image plane is computed from the position sub-matrix $P_{pos} \in \mathbb{R}^{2\times 2}$:
$$
\lambda_1, \lambda_2 = \text{eig}(P_{pos}),\quad a = 3\sqrt{\lambda_1},\quad b = 3\sqrt{\lambda_2},\quad \theta = \frac{1}{2}\arctan2(2 P_{12}, P_{11} - P_{22})
$$

---

## 7. 7-State Coarse Alignment State Machine

The tracking state machine governs operational mode transitions based on real-time sensory confidence and pointing errors:

```
              +-------------------------------------------------+
              |                                                 |
              v                                                 |
      [ 1. SEARCHING ]                                          |
              |                                                 |
              | (Candidate Detected, Conf >= 0.3)               |
              v                                                 |
      [ 2. ACQUIRING ]                                          |
              |                                                 |
              | (Confirmed over N_acq frames)                   |
              v                                                 |
      [ 3. TRACKING ] <--------------------+                    |
              |                            |                    |
              | (Error <= 0.50 deg, M_lock)| (Error > 0.50 deg) |
              v                            |                    |
      [ 4. LOCKED ] -----------------------+                    |
              |                                                 |
              | (Target dropped / Conf < 0.35)                  |
              v                                                 |
      [ 5. DEGRADED ]                                           |
              |                                                 |
              | (Lost frames >= N_lost)                         |
              v                                                 |
      [ 6. LOST ]                                               |
              |                                                 |
              | (Autonomous Reacquisition Scan)                 |
              v                                                 |
      [ 7. REACQUIRING ] ---------------------------------------+
              |           (Timeout > 4.0s)
              v
        (Reacquired) -> [ 2. ACQUIRING ]
```

- **SEARCHING**: Wide-area open-loop Archimedean spiral scanning.
- **ACQUIRING**: Candidate validation across consecutive video frames.
- **TRACKING**: Closed-loop gimbal tracking minimizing image-space crosshair error.
- **LOCKED**: Pointing error stabilized within coarse threshold ($\le 0.50^\circ / 8.73\,\text{mrad}$).
- **DEGRADED**: Transient fading or turbulence; dead-reckoning prediction maintained.
- **LOST**: Target departed FOV or obscured beyond persistence window.
- **REACQUIRING**: Local high-speed expanding search pattern centered on last known predicted trajectory.

---

## 8. Pan/Tilt Gimbal Controller Architecture

### 8.1 Dual-Axis Discrete PID Controller
Commanded angular rate for each axis (pan $\alpha$, tilt $\beta$):
$$
\omega_{cmd}(k) = K_p e(k) + K_i \sum_{j=0}^k e(j) \Delta t + K_d \frac{e(k) - e(k-1)}{\Delta t} + \omega_{ff}
$$
- **Anti-Windup**: Integral accumulation clamped to prevent actuator saturation windup:
  $$I(k) = \text{clamp}(I(k-1) + e(k)\Delta t, -I_{max}, I_{max})$$
- **Filtered Derivative**: First-order low-pass filter prevents high-frequency derivative kick from pixel quantization:
  $$D(k) = \alpha_d D(k-1) + (1 - \alpha_d) \frac{e(k) - e(k-1)}{\Delta t}$$
- **Velocity Feedforward**: $\omega_{ff}$ injects estimated target angular velocity from Kalman states to eliminate steady-state tracking lag.

### 8.2 Actuator Physical Constraints
- **Maximum Slew Rate**: $|\omega| \le 35.0^\circ/\text{s}$ ($0.61\,\text{rad/s}$).
- **Maximum Angular Acceleration**: $|\dot{\omega}| \le 120.0^\circ/\text{s}^2$ ($2.09\,\text{rad/s}^2$).
- **Electro-Mechanical Motor Lag**: First-order response with time constant $\tau = 25\,\text{ms}$:
  $$\omega_{actual}(t + \Delta t) = \omega_{actual}(t) + \frac{\Delta t}{\tau} (\omega_{cmd}(t) - \omega_{actual}(t))$$
- **Deadband Filter**: Errors below $0.02^\circ$ ($0.35\,\text{mrad}$) are zeroed to prevent motor dither.

---

## 9. Mission-Control UI Architecture

The user interface delivers a high-density, aerospace mission-control console:
1. **Backend Server (`ui/server.py`)**: Embedded multi-threaded HTTP server providing JSON REST telemetry endpoints (`/api/telemetry`, `/api/control`, `/api/scenarios`) and live JPEG camera frame streaming (`/api/frame.jpg`) at 30-60 FPS.
2. **3D Tactical World View (`ui/static/js/world_view.js`)**: Real-time Three.js WebGL scene visualizing:
   - Host mobile vehicle and remote optical terminal.
   - Dual-axis rotating camera turret.
   - Dynamic camera FOV frustum pyramid.
   - Target line-of-sight vector, camera pointing boresight vector, and angular error arc.
   - Ground truth and predicted trajectory trails.
3. **Sensor HUD View (`ui/static/js/camera_view.js`)**: Canvas optical sensor overlay showing:
   - High-contrast reticle crosshairs.
   - Detection candidate markers with confidence tags.
   - Kalman filter tracking bounding box.
   - 3-sigma covariance uncertainty ellipse.
4. **Live Telemetry Charts (`ui/static/js/charts.js`)**: Real-time multi-channel scrolling charts for:
   - Pointing Error (Degrees & Milliradians).
   - Detection/Tracking Confidence.
   - FPS & Frame Latency.
   - Pan/Tilt Gimbal Angular Dynamics.

---

## 10. Verification & Quality Assurance Summary

- Complete ground truth isolation: control loop never receives ground truth coordinates.
- Zero mock data or fabricated metrics: every calculation is traceable to mathematical formulas and live state.
- 100% offline-first: all assets bundled locally with zero cloud or runtime download dependencies.
- Real standalone executable packaging via PyInstaller.
