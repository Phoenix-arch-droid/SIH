# SIH26169 FSOC Virtual Tracking System - Performance Verification Log

**Generated At:** 2026-09-30 18:39:08 UTC  
**Software Version:** 1.0.0-SIH26169-PROD (ISRO Theme: Smart Automation)  
**Scenario:** Static Target Baseline (ID: 1)  
**Tracking Mode:** `ENHANCED_AI`  

---

## 1. Executive Summary & Key Performance Indicators (KPIs)

| Performance Metric | Numerical Value | Units | Specification / Status |
| :--- | :--- | :--- | :--- |
| **Simulation Duration** | `15.00` | seconds | Complete |
| **Total Frames Processed** | `455` | frames | 100% processed |
| **Mean Frame Rate** | `30.3` | FPS | Real-time requirement >= 30 FPS |
| **Mean Processing Latency** | `109.29` | ms | Latency margin < 33 ms |
| **Acquisition Time** | `0.264 s` | seconds | Coarse alignment acquisition |
| **Lock Retention Rate** | `39.6` | % | Error <= 0.5° (8.7 mrad) |
| **Lost Frames** | `0` | frames | Handled via dead-reckoning |
| **Reacquisition Events** | `0` | count | Autonomous recoveries |

---

## 2. Pointing Accuracy & Angular Error Analysis

*All metrics calculated from authentic ground truth line-of-sight vectors vs actual camera boresight.*

| Statistical Measure | Angular Error (Degrees) | Angular Error (Milliradians) |
| :--- | :--- | :--- |
| **Root Mean Square (RMS) Error** | `0.4486°` | `7.830 mrad` |
| **Mean Absolute Error** | `0.1781°` | `3.109 mrad` |
| **Median Tracking Error** | `0.0378°` | `0.660 mrad` |
| **Maximum (Peak) Error** | `2.6068°` | `45.497 mrad` |
| **Standard Deviation (\sigma)** | `0.4117°` | `7.186 mrad` |

---

## 3. Engineering Assumptions & Simulation Parameters

- **SYSTEM-DEFINED SIMULATION PARAMETER:** Coarse alignment lock threshold = `0.50°` (`8.73 mrad`).
- **SYSTEM-DEFINED SIMULATION PARAMETER:** Sensor resolution = `1280 x 720` pixels, HFOV = `15.0°`.
- **SYSTEM-DEFINED SIMULATION PARAMETER:** Optical beacon wavelength = `850 nm`, divergence = `15.0 mrad`.
- **ENGINEERING ASSUMPTION:** Actuator electro-mechanical lag constant = `25 ms`, max slew rate = `35.0 deg/s`.
- **ENGINEERING ASSUMPTION:** Atmospheric optical transmission follows Beer-Lambert extinction (gamma = 10^-4 m^-1).
