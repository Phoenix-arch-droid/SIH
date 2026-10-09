"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: ui.server

High-Speed Local Simulation Server & REST API:
- Serves 100% offline frontend (HTML/CSS/Three.js/Chart.js)
- Streams real-time 30-60 FPS telemetry, ground truth, and tracking states
- Serves synthetic camera frame streams
- Handles scenario selection, disturbance injection, benchmarks, and report export
- Standard Python standard library implementation (no heavy web framework dependencies)
"""

import os
import io
import time
import json
import base64
import urllib.parse
from http import HTTPStatus
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import Any, Dict, List, Optional
import cv2
import numpy as np

from simulation.engine import SimulationEngine
from simulation.scenarios import ScenarioCatalog, ScenarioDefinition
from simulation.benchmark import BenchmarkSuite
from analytics.logger import ExperimentSession
from analytics.metrics import RealTimeMetricsCalculator
from analytics.report_generator import PerformanceReportGenerator
from core.disturbances import DisturbanceLevel


class SimulationServerState:
    """Shared state between HTTP handler and simulation worker thread."""
    def __init__(self):
        self.engine = SimulationEngine(use_ai_enhancement=True, seed=101)
        self.metrics_calc = RealTimeMetricsCalculator()
        self.experiment = ExperimentSession(scenario_name="Static Target")
        self.benchmark_suite = BenchmarkSuite()
        
        # Load default scenario 1
        default_scen = ScenarioCatalog.scenario_1_static_target()
        self.engine.load_scenario(default_scen)

        self.is_running = True
        self.is_paused = False
        self.demo_mode_active = False
        self.demo_step = 0
        self.demo_timer = 0.0

        self.last_frame_jpeg = b""
        self.last_telemetry_dict = {}


SERVER_STATE = SimulationServerState()


class FSOCRequestHandler(SimpleHTTPRequestHandler):
    """
    HTTP Request Handler serving both static dashboard files and JSON API.
    """

    def __init__(self, *args, **kwargs):
        static_dir = os.path.join(os.path.dirname(__file__), "static")
        super().__init__(*args, directory=static_dir, **kwargs)

    def log_message(self, format, *args):
        # Suppress spammy per-frame HTTP GET log lines
        pass

    def do_GET(self):
        url_parts = urllib.parse.urlparse(self.path)
        path = url_parts.path

        if path == "/api/telemetry":
            self._handle_get_telemetry()
        elif path == "/api/camera_frame":
            self._handle_get_camera_frame()
        elif path == "/api/scenarios":
            self._handle_get_scenarios()
        elif path == "/api/benchmark":
            self._handle_run_benchmark(url_parts.query)
        elif path == "/api/export/report":
            self._handle_export_report()
        elif path == "/api/export/experiment":
            self._handle_export_experiment()
        else:
            # Fall back to serving static files (index.html, style.css, js/*.js)
            super().do_GET()

    def do_POST(self):
        url_parts = urllib.parse.urlparse(self.path)
        path = url_parts.path

        content_length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b"{}"

        try:
            body = json.loads(post_data.decode("utf-8")) if post_data else {}
        except Exception:
            body = {}

        if path == "/api/scenario/load":
            self._handle_load_scenario(body)
        elif path == "/api/sim/control":
            self._handle_sim_control(body)
        elif path == "/api/mode/toggle":
            self._handle_toggle_ai(body)
        elif path == "/api/disturbance/inject":
            self._handle_inject_disturbance(body)
        elif path == "/api/demo/start":
            self._handle_start_demo()
        else:
            self.send_error(HTTPStatus.NOT_FOUND, "Endpoint not found")

    def _send_json(self, data: Any):
        payload = json.dumps(data).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    def _handle_get_scenarios(self):
        scenarios = ScenarioCatalog.get_all_scenarios()
        resp = [{
            "id": s.scenario_id,
            "name": s.name,
            "description": s.description,
            "duration_s": s.duration_s,
            "preset": s.disturbance_preset.value
        } for s in scenarios]
        self._send_json(resp)

    def _handle_load_scenario(self, body: Dict[str, Any]):
        scen_id = int(body.get("scenario_id", 1))
        scen = ScenarioCatalog.get_by_id(scen_id)
        SERVER_STATE.engine.load_scenario(scen)
        SERVER_STATE.metrics_calc.reset()
        SERVER_STATE.experiment = ExperimentSession(scenario_name=scen.name)
        SERVER_STATE.demo_mode_active = False
        self._send_json({"status": "success", "loaded_scenario": scen.name})

    def _handle_sim_control(self, body: Dict[str, Any]):
        action = body.get("action", "pause")
        if action == "pause":
            SERVER_STATE.is_paused = True
        elif action == "resume":
            SERVER_STATE.is_paused = False
        elif action == "reset":
            if SERVER_STATE.engine.current_scenario:
                SERVER_STATE.engine.load_scenario(SERVER_STATE.engine.current_scenario)
            else:
                SERVER_STATE.engine.reset_state()
            SERVER_STATE.metrics_calc.reset()
            SERVER_STATE.demo_mode_active = False
        self._send_json({"status": "success", "action": action, "is_paused": SERVER_STATE.is_paused})

    def _handle_toggle_ai(self, body: Dict[str, Any]):
        enabled = bool(body.get("ai_enhanced", True))
        SERVER_STATE.engine.set_ai_mode(enabled)
        self._send_json({"status": "success", "is_ai_enhanced": SERVER_STATE.engine.is_ai_enhanced})

    def _handle_inject_disturbance(self, body: Dict[str, Any]):
        preset_name = body.get("preset")
        if preset_name in [p.value for p in DisturbanceLevel]:
            SERVER_STATE.engine.disturbances.apply_preset(DisturbanceLevel(preset_name))
        
        # Apply any custom overrides
        for k, v in body.items():
            if hasattr(SERVER_STATE.engine.disturbances.config, k):
                setattr(SERVER_STATE.engine.disturbances.config, k, v)

        self._send_json({
            "status": "success",
            "active_preset": SERVER_STATE.engine.disturbances.config.level.value
        })

    def _handle_start_demo(self):
        """Starts the deterministic 18-step FSOC Coarse Alignment Demo."""
        scen = ScenarioCatalog.scenario_13_combined_disturbance()
        SERVER_STATE.engine.load_scenario(scen)
        SERVER_STATE.metrics_calc.reset()
        SERVER_STATE.demo_mode_active = True
        SERVER_STATE.demo_step = 1
        SERVER_STATE.demo_timer = 0.0
        SERVER_STATE.is_paused = False
        self._send_json({"status": "demo_started", "scenario": scen.name})

    def _handle_get_telemetry(self):
        """Advances simulation step if running and returns telemetry JSON."""
        st = SERVER_STATE
        if not st.is_paused and st.is_running:
            # Advance simulation by dt = 0.033s (30 FPS)
            snapshot = st.engine.step(0.033)
            metrics = st.metrics_calc.update(snapshot)
            st.experiment.record_snapshot(snapshot)

            # Encode synthetic frame to JPEG
            if st.engine.current_frame_image is not None:
                # Downsample or encode to JPEG for HUD
                ret, buf = cv2.imencode(".jpg", st.engine.current_frame_image, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ret:
                    st.last_frame_jpeg = buf.tobytes()

            # Compile telemetry dictionary
            designated = st.engine.target_manager.get_designated()
            all_targets = st.engine.target_manager.get_all()

            target_list = []
            for t in all_targets:
                target_list.append({
                    "id": t.target_id,
                    "is_designated": t.is_designated,
                    "x": round(float(t.position[0]), 2),
                    "y": round(float(t.position[1]), 2),
                    "z": round(float(t.position[2]), 2),
                    "color": t.optical.color_rgb
                })

            st.last_telemetry_dict = {
                "timestamp_s": round(snapshot.timestamp_s, 3),
                "frame_number": snapshot.frame_number,
                "fps": round(snapshot.fps, 1),
                "latency_ms": round(snapshot.latency_ms, 2),
                "is_ai_enhanced": snapshot.is_ai_enhanced,
                "state": snapshot.state.value,
                "confidence": round(snapshot.tracking_confidence, 3),
                "is_locked": snapshot.is_locked,
                
                # Ground Truth
                "gt_total_error_deg": round(snapshot.gt.true_angular_error_deg, 4),
                "gt_total_error_mrad": round(snapshot.gt.true_angular_error_mrad, 3),
                "gt_az_deg": round(snapshot.gt.true_azimuth_error_deg, 4),
                "gt_el_deg": round(snapshot.gt.true_elevation_error_deg, 4),
                "gt_range_m": round(snapshot.gt.slant_range_m, 2),
                "gt_pixel_u": round(snapshot.gt.true_pixel_u, 2) if snapshot.gt.true_pixel_u else None,
                "gt_pixel_v": round(snapshot.gt.true_pixel_v, 2) if snapshot.gt.true_pixel_v else None,
                "is_in_fov": snapshot.gt.is_in_fov,
                "is_occluded": snapshot.gt.is_occluded,

                # Tracking & Estimates
                "detected_u": round(snapshot.detected_pixel_u, 2) if snapshot.detected_pixel_u else None,
                "detected_v": round(snapshot.detected_pixel_v, 2) if snapshot.detected_pixel_v else None,
                "tracked_u": round(snapshot.tracked_pixel_u, 2) if snapshot.tracked_pixel_u else None,
                "tracked_v": round(snapshot.tracked_pixel_v, 2) if snapshot.tracked_pixel_v else None,
                "predicted_u": round(snapshot.predicted_pixel_u, 2) if snapshot.predicted_pixel_u else None,
                "predicted_v": round(snapshot.predicted_pixel_v, 2) if snapshot.predicted_pixel_v else None,
                "uncert_major": round(snapshot.uncertainty_semi_major, 1),
                "uncert_minor": round(snapshot.uncertainty_semi_minor, 1),
                
                # Gimbal
                "gimbal_pan_deg": round(snapshot.gimbal_pan_deg, 4),
                "gimbal_tilt_deg": round(snapshot.gimbal_tilt_deg, 4),
                "gimbal_pan_vel": round(snapshot.gimbal_pan_vel_deg_s, 2),
                "gimbal_tilt_vel": round(snapshot.gimbal_tilt_vel_deg_s, 2),
                "cmd_pan_vel": round(snapshot.commanded_pan_vel_deg_s, 2),
                "cmd_tilt_vel": round(snapshot.commanded_tilt_vel_deg_s, 2),

                # Running Metrics
                "metrics": {
                    "mean_error_deg": round(metrics.mean_error_deg, 4),
                    "rms_error_deg": round(metrics.rms_error_deg, 4),
                    "mean_error_mrad": round(metrics.mean_error_mrad, 3),
                    "rms_error_mrad": round(metrics.rms_error_mrad, 3),
                    "max_error_deg": round(metrics.max_error_deg, 4),
                    "lock_retention_pct": round(metrics.lock_retention_pct, 1),
                    "acquisition_time_s": round(metrics.acquisition_time_s, 3) if metrics.acquisition_time_s else None,
                    "lost_frames": metrics.lost_frames_count,
                    "peak_slew": round(metrics.peak_gimbal_slew_deg_s, 2)
                },

                # 3D Objects in Scene
                "objects": {
                    "targets": target_list,
                    "camera_pos": [0.0, 0.0, 0.0],
                    "camera_pan_deg": round(snapshot.gimbal_pan_deg, 2),
                    "camera_tilt_deg": round(snapshot.gimbal_tilt_deg, 2),
                    "hfov_deg": round(st.engine.camera.intrinsics.hfov_deg, 1),
                    "vfov_deg": round(st.engine.camera.intrinsics.vfov_deg, 1)
                },

                "disturbance_level": snapshot.disturbance_level
            }

        self._send_json(st.last_telemetry_dict)

    def _handle_get_camera_frame(self):
        """Returns the simulated camera sensor feed as JPEG."""
        frame_bytes = SERVER_STATE.last_frame_jpeg
        if not frame_bytes:
            # Generate black placeholder
            dummy = np.zeros((360, 640), dtype=np.uint8)
            ret, buf = cv2.imencode(".jpg", dummy)
            frame_bytes = buf.tobytes()

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame_bytes)))
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(frame_bytes)

    def _handle_run_benchmark(self, query_str: str):
        params = urllib.parse.parse_qs(query_str)
        scen_id = int(params.get("scenario_id", [1])[0])
        scen = ScenarioCatalog.get_by_id(scen_id)

        # Run comparison between Baseline and Enhanced
        comparison = SERVER_STATE.benchmark_suite.run_comparison(scen)
        resp = {
            "scenario_name": comparison.scenario_name,
            "scenario_id": comparison.scenario_id,
            "baseline": {
                "rms_error_deg": round(comparison.baseline.rms_error_deg, 4),
                "rms_error_mrad": round(comparison.baseline.rms_error_mrad, 3),
                "mean_error_deg": round(comparison.baseline.mean_error_deg, 4),
                "max_error_deg": round(comparison.baseline.max_error_deg, 4),
                "lock_retention_pct": round(comparison.baseline.lock_retention_rate_pct, 1),
                "acquisition_time_s": round(comparison.baseline.acquisition_time_s, 3) if comparison.baseline.acquisition_time_s else None,
                "fps": round(comparison.baseline.fps_mean, 1),
                "latency_ms": round(comparison.baseline.latency_mean_ms, 2)
            },
            "enhanced": {
                "rms_error_deg": round(comparison.enhanced.rms_error_deg, 4),
                "rms_error_mrad": round(comparison.enhanced.rms_error_mrad, 3),
                "mean_error_deg": round(comparison.enhanced.mean_error_deg, 4),
                "max_error_deg": round(comparison.enhanced.max_error_deg, 4),
                "lock_retention_pct": round(comparison.enhanced.lock_retention_rate_pct, 1),
                "acquisition_time_s": round(comparison.enhanced.acquisition_time_s, 3) if comparison.enhanced.acquisition_time_s else None,
                "fps": round(comparison.enhanced.fps_mean, 1),
                "latency_ms": round(comparison.enhanced.latency_mean_ms, 2)
            },
            "rms_reduction_pct": round(comparison.rms_error_reduction_pct, 2),
            "lock_improvement_pct": round(comparison.lock_retention_improvement_pct, 2)
        }
        self._send_json(resp)

    def _handle_export_report(self):
        """Generates and downloads the official performance report."""
        scen = SERVER_STATE.engine.current_scenario or ScenarioCatalog.scenario_1_static_target()
        bench_res = SERVER_STATE.benchmark_suite.run_single_scenario(scen, use_ai=SERVER_STATE.engine.is_ai_enhanced)
        md_content = PerformanceReportGenerator.generate_single_report_markdown(bench_res)

        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/markdown; charset=utf-8")
        self.send_header("Content-Disposition", f"attachment; filename=FSOC_Performance_Report_{scen.scenario_id}.md")
        self.send_header("Content-Length", str(len(md_content.encode("utf-8"))))
        self.end_headers()
        self.wfile.write(md_content.encode("utf-8"))

    def _handle_export_experiment(self):
        """Exports the active experiment session JSON."""
        data = {
            "experiment_id": SERVER_STATE.experiment.experiment_id,
            "scenario": SERVER_STATE.experiment.scenario_name,
            "timestamp": SERVER_STATE.experiment.timestamp,
            "records_count": len(SERVER_STATE.experiment.telemetry_records),
            "telemetry": SERVER_STATE.experiment.telemetry_records[-100:] # Last 100 frames
        }
        payload = json.dumps(data, indent=2).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Disposition", "attachment; filename=fsoc_experiment_export.json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


def run_server(port: int = 8765):
    """Starts the HTTP server listening on localhost."""
    server_address = ("127.0.0.1", port)
    httpd = HTTPServer(server_address, FSOCRequestHandler)
    print(f"[FSOC Server] Running at http://127.0.0.1:{port}")
    httpd.serve_forever()
