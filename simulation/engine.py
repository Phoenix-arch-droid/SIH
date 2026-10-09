"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: simulation.engine

Closed-Loop Coarse Alignment Simulation Engine:
Integrates physics, kinematics, virtual optics, disturbances, detector, tracker,
state machine, and gimbal control in a unified frame-by-frame loop.
"""

import time
import math
import cv2
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional, Tuple

from core.coordinates import Pose3D, CoordinateTransform
from core.camera_model import CameraIntrinsics, VirtualCamera
from core.targets import OpticalBeaconTarget, BeaconTargetManager, TrajectoryType
from core.disturbances import DisturbanceLevel, DisturbanceConfig, DisturbanceEngine
from core.state_machine import AlignmentState, CoarseAlignmentStateMachine, StateTransitionEvent
from detection.baseline_cv import DetectionCandidate, ClassicalCVDetector
from detection.ai_detector import AIEnhancedBeaconDetector
from detection.id_matcher import BeaconIdentifier
from tracking.kalman_tracker import BeaconKalmanTracker, TrackState
from tracking.predictor import MotionPredictor, UncertaintyEllipse
from control.gimbal_controller import GimbalLimits, PIDGains, PanTiltGimbalController
from control.scan_patterns import ScanPatternType, AutonomousSearchScanner
from simulation.ground_truth import GroundTruthFrame, GroundTruthProvider
from simulation.scenarios import ScenarioDefinition, ScenarioCatalog


@dataclass
class TelemetrySnapshot:
    """Complete, synchronized telemetry record for a single simulation frame."""
    timestamp_s: float
    frame_number: int
    fps: float
    latency_ms: float
    is_ai_enhanced: bool

    # Ground Truth State
    gt: GroundTruthFrame

    # Tracking & State Machine
    state: AlignmentState
    tracking_confidence: float
    is_target_acquired: bool
    is_locked: bool

    # Measurements & Estimates (from CV / Tracker)
    detected_pixel_u: Optional[float]
    detected_pixel_v: Optional[float]
    tracked_pixel_u: Optional[float]
    tracked_pixel_v: Optional[float]
    predicted_pixel_u: Optional[float]
    predicted_pixel_v: Optional[float]
    uncertainty_semi_major: float
    uncertainty_semi_minor: float

    # Estimated Angular Errors (fed to gimbal controller)
    estimated_pan_error_deg: float
    estimated_tilt_error_deg: float
    estimated_total_error_deg: float
    estimated_total_error_mrad: float

    # Gimbal State
    gimbal_pan_deg: float
    gimbal_tilt_deg: float
    gimbal_pan_vel_deg_s: float
    gimbal_tilt_vel_deg_s: float
    commanded_pan_vel_deg_s: float
    commanded_tilt_vel_deg_s: float

    # Disturbances Active
    disturbance_level: str
    is_occluded: bool
    is_frame_dropped: bool
    turbulence_scintillation: float


class SimulationEngine:
    """
    Main closed-loop simulation engine for the FSOC tracking laboratory.
    """

    def __init__(
        self,
        camera_intrinsics: Optional[CameraIntrinsics] = None,
        use_ai_enhancement: bool = True,
        seed: int = 42
    ):
        self.seed = seed
        self.camera = VirtualCamera(camera_intrinsics)
        self.gt_provider = GroundTruthProvider(self.camera)
        self.target_manager = BeaconTargetManager()
        self.disturbances = DisturbanceEngine(seed=seed)
        
        # Detection components
        self.cv_detector = ClassicalCVDetector()
        self.ai_detector = AIEnhancedBeaconDetector(self.cv_detector)
        self.identifier = BeaconIdentifier()
        self.is_ai_enhanced: bool = use_ai_enhancement

        # Tracking & Prediction
        self.tracker: Optional[BeaconKalmanTracker] = None
        self.predictor = MotionPredictor()

        # State Machine
        self.state_machine = CoarseAlignmentStateMachine()

        # Gimbal Control & Search Scanning
        self.gimbal = PanTiltGimbalController()
        self.scanner = AutonomousSearchScanner()

        # Simulation clock and stats
        self.current_time_s: float = 0.0
        self.frame_number: int = 0
        self.current_scenario: Optional[ScenarioDefinition] = None
        self.is_running: bool = False
        self.is_paused: bool = False

        # Synthetic camera image buffer
        self.current_frame_image: Optional[np.ndarray] = None
        self.current_telemetry: Optional[TelemetrySnapshot] = None
        self.history: List[TelemetrySnapshot] = []

        # Platform motion (e.g. ship or UAV attitude oscillations)
        self.platform_pose = Pose3D()
        self.platform_motion_enabled = False

    def load_scenario(self, scenario: ScenarioDefinition):
        """Initializes and loads a standard scenario."""
        self.current_scenario = scenario
        self.seed = scenario.seed
        self.disturbances = DisturbanceEngine(seed=scenario.seed)
        
        if scenario.disturbance_preset != DisturbanceLevel.CUSTOM:
            self.disturbances.apply_preset(scenario.disturbance_preset)
        elif scenario.custom_disturbance:
            self.disturbances.apply_preset(DisturbanceLevel.CUSTOM)
            for k, v in scenario.custom_disturbance.items():
                setattr(self.disturbances.config, k, v)

        # Reset camera / gimbal
        self.gimbal.reset(scenario.initial_camera_pan_deg, scenario.initial_camera_tilt_deg)
        self.scanner.reset(scenario.initial_camera_pan_deg, scenario.initial_camera_tilt_deg)

        # Clear and configure targets
        self.target_manager.clear()
        for tcfg in scenario.targets_config:
            t = OpticalBeaconTarget(
                target_id=tcfg["id"],
                is_designated_target=tcfg.get("is_designated", True),
                initial_position=tcfg["position"],
                initial_velocity=tcfg.get("velocity", [0.0, 0.0, 0.0]),
                initial_acceleration=tcfg.get("acceleration", [0.0, 0.0, 0.0]),
                optical_signature=tcfg.get("optical"),
                trajectory_type=tcfg.get("trajectory", TrajectoryType.LINEAR),
                trajectory_params=tcfg.get("params", {})
            )
            self.target_manager.add_target(t)

        self.platform_motion_enabled = scenario.platform_motion_enabled
        self.reset_state()

    def reset_state(self):
        """Resets simulation time, tracking filters, and state machine."""
        self.current_time_s = 0.0
        self.frame_number = 0
        self.tracker = None
        self.state_machine.reset()
        self.identifier.reset()
        self.target_manager.reset_all()
        self.history.clear()
        self.current_frame_image = None
        self.current_telemetry = None

    def set_ai_mode(self, enabled: bool):
        """Toggles between Baseline Classical CV and AI/ML Enhanced mode."""
        self.is_ai_enhanced = enabled

    def _render_synthetic_frame(
        self,
        camera_pan_rad: float,
        camera_tilt_rad: float,
        is_occluded: bool
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Renders the synthetic optical camera sensor view in real-time.
        Applies radiometric beacon spots, Point Spread Function (PSF),
        modulation, background daylight, and sensor noise.
        """
        w = self.camera.intrinsics.width
        h = self.camera.intrinsics.height
        frame = np.zeros((h, w), dtype=np.float32)

        spot_metadata: Dict[str, Any] = {"spots": []}

        # Render all targets/beacons
        for target in self.target_manager.get_all():
            # If target is primary and occluded, it emits zero line-of-sight light
            if target.is_designated and is_occluded:
                continue

            # Transform target to camera optical frame
            p_cam = CoordinateTransform.world_to_camera_point(
                target.position,
                self.platform_pose.position_vector(),
                camera_pan_rad,
                camera_tilt_rad,
                self.platform_pose if self.platform_motion_enabled else None
            )

            pixel_coord, in_fov, meta = self.camera.project_point(p_cam)
            if not in_fov or pixel_coord is None:
                continue

            u_c = pixel_coord[0]
            v_c = pixel_coord[1]
            slant_range = meta["range_zc"]

            # Radiometric intensity calculation
            mod_factor = target.get_instantaneous_intensity_factor(self.current_time_s)
            scint_factor = self.disturbances.compute_scintillation_multiplier()
            
            # Distance attenuation
            irradiance = self.camera.calculate_apparent_beacon_irradiance(
                target.optical.optical_power_w,
                target.optical.divergence_mrad,
                slant_range
            )
            # Spot peak intensity (0-255 scaled)
            base_intensity = min(255.0, max(20.0, irradiance * 18000.0))
            peak_val = base_intensity * mod_factor * scint_factor

            # Optical Point Spread Function (PSF) sigma
            psf_sigma = self.camera.calculate_psf_sigma_pixels(slant_range)
            # Add atmospheric turbulence blur radius
            psf_sigma += self.disturbances.config.turbulence_blur_radius

            # Render Gaussian intensity profile in sub-window
            radius = int(math.ceil(psf_sigma * 3.5))
            ix1 = max(0, int(u_c - radius))
            ix2 = min(w, int(u_c + radius + 1))
            iy1 = max(0, int(v_c - radius))
            iy2 = min(h, int(v_c + radius + 1))

            if ix2 > ix1 and iy2 > iy1:
                gx, gy = np.meshgrid(np.arange(ix1, ix2), np.arange(iy1, iy2))
                dist_sq = (gx - u_c)**2 + (gy - v_c)**2
                gaussian = peak_val * np.exp(-dist_sq / (2.0 * psf_sigma**2))
                frame[iy1:iy2, ix1:ix2] += gaussian

            spot_metadata["spots"].append({
                "target_id": target.target_id,
                "u": u_c,
                "v": v_c,
                "peak_intensity": peak_val,
                "sigma": psf_sigma
            })

        # Apply environmental disturbances (noise, glare, background)
        degraded_frame = self.disturbances.apply_sensor_disturbances(frame)
        return degraded_frame, spot_metadata

    def step(self, dt: float = 0.033) -> TelemetrySnapshot:
        """
        Executes one full simulation time step of the closed-loop tracking pipeline.
        """
        t_start = time.perf_counter()
        dt = max(1e-4, dt)
        self.current_time_s += dt
        self.frame_number += 1

        # 1. Update 3D target kinematics
        self.target_manager.update_all(dt, self.current_time_s)
        designated = self.target_manager.get_designated()
        if designated is None:
            # Fallback default target if none specified
            designated = OpticalBeaconTarget("BEACON_PRIMARY", initial_position=[0.0, 500.0, 50.0])
            self.target_manager.add_target(designated)

        # 2. Update platform attitude motion if enabled
        if self.platform_motion_enabled:
            omega_p = 0.3 * 2.0 * math.pi
            self.platform_pose.roll = math.radians(1.2 * math.sin(omega_p * self.current_time_s))
            self.platform_pose.pitch = math.radians(0.8 * math.cos(omega_p * self.current_time_s * 0.7))
            self.platform_pose.yaw = math.radians(1.5 * math.sin(omega_p * self.current_time_s * 0.4))

        # 3. Compute angular physical disturbances (vibration, jitter, beam wander)
        vib_pan_rad, vib_tilt_rad = self.disturbances.compute_angular_disturbances(self.current_time_s)
        
        # Effective camera sensor orientation in world
        gimbal_pan_rad = math.radians(self.gimbal.pan_deg)
        gimbal_tilt_rad = math.radians(self.gimbal.tilt_deg)
        effective_camera_pan_rad = gimbal_pan_rad + vib_pan_rad
        effective_camera_tilt_rad = gimbal_tilt_rad + vib_tilt_rad

        is_occluded = self.disturbances.is_occluded(self.current_time_s)
        is_dropped = self.disturbances.is_frame_dropped()

        # 4. Compute Ground Truth (Strictly Isolated)
        gt_frame = self.gt_provider.evaluate_frame(
            timestamp_s=self.current_time_s,
            target_id=designated.target_id,
            target_pos_world=designated.position,
            target_vel_world=designated.velocity,
            target_accel_world=designated.acceleration,
            camera_pos_world=self.platform_pose.position_vector(),
            camera_pan_rad=effective_camera_pan_rad,
            camera_tilt_rad=effective_camera_tilt_rad,
            is_occluded=is_occluded,
            platform_pose=self.platform_pose if self.platform_motion_enabled else None
        )

        # 5. Generate Synthetic Camera Feed
        raw_frame, spot_meta = self._render_synthetic_frame(
            effective_camera_pan_rad,
            effective_camera_tilt_rad,
            is_occluded
        )
        self.current_frame_image = raw_frame

        # 6. Detection Pipeline (Baseline CV vs AI/ML Enhanced)
        detected_candidate: Optional[DetectionCandidate] = None
        matched_candidate: Optional[DetectionCandidate] = None
        match_confidence = 0.0

        if not is_dropped:
            # Execute selected detector
            if self.is_ai_enhanced:
                candidates = self.ai_detector.detect(raw_frame)
            else:
                candidates = self.cv_detector.detect(raw_frame)

            if candidates:
                detected_candidate = candidates[0]
                # Validate against designated beacon optical signature
                for cand in candidates:
                    is_match, conf, _ = self.identifier.match_candidate(
                        cand,
                        track_id=designated.target_id,
                        timestamp_s=self.current_time_s
                    )
                    if is_match:
                        matched_candidate = cand
                        match_confidence = conf
                        break

        # 7. Temporal Kalman Tracking & Prediction
        target_observed = (matched_candidate is not None)
        pred_u, pred_v = None, None
        uncert_a, uncert_b = 0.0, 0.0

        if target_observed:
            cand = matched_candidate
            if self.tracker is None:
                # Initialize new track
                self.tracker = BeaconKalmanTracker(
                    track_id=designated.target_id,
                    initial_u=cand.centroid_u,
                    initial_v=cand.centroid_v,
                    initial_dt=dt
                )
            else:
                self.tracker.predict(dt)
                self.tracker.update(cand)
        elif self.tracker is not None:
            # Dead-reckoning propagation during missed frames
            self.tracker.predict(dt)
            self.tracker.mark_miss()

        # Update Motion Predictor
        if self.tracker is not None:
            track_state = self.tracker.get_state()
            tracked_u = track_state.u
            tracked_v = track_state.v
            tracking_conf = track_state.confidence

            # Forward lookahead prediction (100 ms)
            pred_u, pred_v = self.predictor.compute_compensated_target_point(self.tracker, 0.10)
            ellipse = self.predictor.compute_uncertainty_ellipse(track_state.covariance, tracked_u, tracked_v)
            uncert_a = ellipse.semi_major
            uncert_b = ellipse.semi_minor
        else:
            tracked_u, tracked_v = None, None
            tracking_conf = 0.0

        # 8. Compute Estimated Errors (From Tracker to Optical Crosshair; NEVER ground truth)
        if tracked_u is not None and tracked_v is not None:
            d_az_rad, d_el_rad = self.camera.pixel_error_to_angular_offset(tracked_u, tracked_v)
            est_pan_err_deg = math.degrees(d_az_rad)
            est_tilt_err_deg = math.degrees(d_el_rad)
            est_total_err_deg = math.hypot(est_pan_err_deg, est_tilt_err_deg)
            est_total_err_mrad = est_total_err_deg * (math.pi / 180.0) * 1000.0

            # Estimated velocity rates for feedforward
            track_st = self.tracker.get_state()
            rate_pan_deg_s = math.degrees(track_st.velocity_u / self.camera.intrinsics.fx)
            rate_tilt_deg_s = math.degrees(track_st.velocity_v / self.camera.intrinsics.fy)
        else:
            est_pan_err_deg = 0.0
            est_tilt_err_deg = 0.0
            est_total_err_deg = 99.0
            est_total_err_mrad = 99000.0
            rate_pan_deg_s = 0.0
            rate_tilt_deg_s = 0.0

        # 9. Update 7-State Alignment State Machine
        in_fov_estimate = (tracked_u is not None and 0.0 <= tracked_u < self.camera.intrinsics.width and
                           tracked_v is not None and 0.0 <= tracked_v < self.camera.intrinsics.height)

        current_state = self.state_machine.update(
            dt=dt,
            timestamp_s=self.current_time_s,
            target_detected=target_observed,
            angular_error_deg=est_total_err_deg,
            tracking_confidence=tracking_conf,
            is_target_in_fov=in_fov_estimate,
            target_id=designated.target_id
        )

        # 10. Pan/Tilt Gimbal Controller Execution
        if current_state in (AlignmentState.TRACKING, AlignmentState.LOCKED, AlignmentState.DEGRADED):
            # Closed-loop tracking driving estimated error to zero
            self.gimbal.update_closed_loop(
                estimated_angular_error_pan_deg=est_pan_err_deg,
                estimated_angular_error_tilt_deg=est_tilt_err_deg,
                estimated_target_rate_pan_deg_s=rate_pan_deg_s,
                estimated_target_rate_tilt_deg_s=rate_tilt_deg_s,
                dt=dt
            )
        elif current_state == AlignmentState.SEARCHING:
            # Wide-area autonomous spiral search
            p_cmd, t_cmd, r_pan, r_tilt = self.scanner.compute_scan_angles(dt)
            self.gimbal.set_open_loop_rate(r_pan, r_tilt, dt)
        elif current_state == AlignmentState.REACQUIRING:
            # Local high-speed reacquisition spiral
            p_cmd, t_cmd, r_pan, r_tilt = self.scanner.compute_scan_angles(dt)
            self.gimbal.set_open_loop_rate(r_pan, r_tilt, dt)

        # 11. Compile Telemetry Record
        t_end = time.perf_counter()
        latency_ms = (t_end - t_start) * 1000.0
        fps = 1.0 / max(1e-4, dt)

        snapshot = TelemetrySnapshot(
            timestamp_s=self.current_time_s,
            frame_number=self.frame_number,
            fps=fps,
            latency_ms=latency_ms,
            is_ai_enhanced=self.is_ai_enhanced,
            gt=gt_frame,
            state=current_state,
            tracking_confidence=tracking_conf,
            is_target_acquired=current_state in (AlignmentState.TRACKING, AlignmentState.LOCKED),
            is_locked=(current_state == AlignmentState.LOCKED),
            detected_pixel_u=matched_candidate.centroid_u if matched_candidate else None,
            detected_pixel_v=matched_candidate.centroid_v if matched_candidate else None,
            tracked_pixel_u=tracked_u,
            tracked_pixel_v=tracked_v,
            predicted_pixel_u=pred_u,
            predicted_pixel_v=pred_v,
            uncertainty_semi_major=uncert_a,
            uncertainty_semi_minor=uncert_b,
            estimated_pan_error_deg=est_pan_err_deg,
            estimated_tilt_error_deg=est_tilt_err_deg,
            estimated_total_error_deg=est_total_err_deg,
            estimated_total_error_mrad=est_total_err_mrad,
            gimbal_pan_deg=self.gimbal.pan_deg,
            gimbal_tilt_deg=self.gimbal.tilt_deg,
            gimbal_pan_vel_deg_s=self.gimbal.pan_vel_deg_s,
            gimbal_tilt_vel_deg_s=self.gimbal.tilt_vel_deg_s,
            commanded_pan_vel_deg_s=self.gimbal.commanded_pan_vel,
            commanded_tilt_vel_deg_s=self.gimbal.commanded_tilt_vel,
            disturbance_level=self.disturbances.config.level.value,
            is_occluded=is_occluded,
            is_frame_dropped=is_dropped,
            turbulence_scintillation=self.disturbances.config.scintillation_index
        )

        self.current_telemetry = snapshot
        self.history.append(snapshot)
        return snapshot
