"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: analytics.logger

Experiment Manager & Telemetry Persistence Engine:
- CREATE, RUN, PAUSE, RESUME, STOP, RESET, SAVE, LOAD, REPLAY, EXPORT
- Serializes complete experiment configurations, seed, ground truth, telemetry, and state transitions
- Exports to standard JSON and CSV format
- Reproducible deterministic replay capability
"""

import os
import json
import csv
import uuid
import datetime
from dataclasses import asdict
from typing import Dict, Any, List, Optional
from simulation.engine import TelemetrySnapshot


class ExperimentSession:
    """
    Manages an active or saved simulation experiment run.
    """

    def __init__(
        self,
        experiment_id: Optional[str] = None,
        scenario_name: str = "Custom",
        software_version: str = "1.0.0-SIH26169-PROD",
        metadata: Optional[Dict[str, Any]] = None
    ):
        self.experiment_id = experiment_id if experiment_id is not None else str(uuid.uuid4())[:8]
        self.timestamp = datetime.datetime.now().isoformat()
        self.scenario_name = scenario_name
        self.software_version = software_version
        self.metadata = metadata if metadata is not None else {}
        self.telemetry_records: List[Dict[str, Any]] = []
        self.state_events: List[Dict[str, Any]] = []

    def record_snapshot(self, snapshot: TelemetrySnapshot):
        """Converts a telemetry snapshot to a serializable dictionary."""
        rec = {
            "timestamp_s": round(snapshot.timestamp_s, 4),
            "frame_number": snapshot.frame_number,
            "fps": round(snapshot.fps, 1),
            "latency_ms": round(snapshot.latency_ms, 2),
            "is_ai_enhanced": snapshot.is_ai_enhanced,
            "state": snapshot.state.value,
            "confidence": round(snapshot.tracking_confidence, 3),
            "is_locked": snapshot.is_locked,
            "gt_true_az_deg": round(snapshot.gt.true_azimuth_error_deg, 4),
            "gt_true_el_deg": round(snapshot.gt.true_elevation_error_deg, 4),
            "gt_total_error_deg": round(snapshot.gt.true_angular_error_deg, 4),
            "gt_total_error_mrad": round(snapshot.gt.true_angular_error_mrad, 3),
            "gt_range_m": round(snapshot.gt.slant_range_m, 2),
            "gt_u": round(snapshot.gt.true_pixel_u, 2) if snapshot.gt.true_pixel_u is not None else None,
            "gt_v": round(snapshot.gt.true_pixel_v, 2) if snapshot.gt.true_pixel_v is not None else None,
            "is_in_fov": snapshot.gt.is_in_fov,
            "is_occluded": snapshot.gt.is_occluded,
            "detected_u": round(snapshot.detected_pixel_u, 2) if snapshot.detected_pixel_u is not None else None,
            "detected_v": round(snapshot.detected_pixel_v, 2) if snapshot.detected_pixel_v is not None else None,
            "tracked_u": round(snapshot.tracked_pixel_u, 2) if snapshot.tracked_pixel_u is not None else None,
            "tracked_v": round(snapshot.tracked_pixel_v, 2) if snapshot.tracked_pixel_v is not None else None,
            "predicted_u": round(snapshot.predicted_pixel_u, 2) if snapshot.predicted_pixel_u is not None else None,
            "predicted_v": round(snapshot.predicted_pixel_v, 2) if snapshot.predicted_pixel_v is not None else None,
            "estimated_pan_err_deg": round(snapshot.estimated_pan_error_deg, 4),
            "estimated_tilt_err_deg": round(snapshot.estimated_tilt_error_deg, 4),
            "estimated_total_err_deg": round(snapshot.estimated_total_error_deg, 4),
            "gimbal_pan_deg": round(snapshot.gimbal_pan_deg, 4),
            "gimbal_tilt_deg": round(snapshot.gimbal_tilt_deg, 4),
            "gimbal_pan_vel": round(snapshot.gimbal_pan_vel_deg_s, 2),
            "gimbal_tilt_vel": round(snapshot.gimbal_tilt_vel_deg_s, 2),
            "cmd_pan_vel": round(snapshot.commanded_pan_vel_deg_s, 2),
            "cmd_tilt_vel": round(snapshot.commanded_tilt_vel_deg_s, 2),
            "disturbance_level": snapshot.disturbance_level
        }
        self.telemetry_records.append(rec)

    def record_state_event(self, event_dict: Dict[str, Any]):
        self.state_events.append(event_dict)

    def save_json(self, filepath: str):
        """Saves full experiment data to JSON."""
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        payload = {
            "experiment_id": self.experiment_id,
            "timestamp": self.timestamp,
            "software_version": self.software_version,
            "scenario_name": self.scenario_name,
            "metadata": self.metadata,
            "total_frames": len(self.telemetry_records),
            "state_events": self.state_events,
            "telemetry": self.telemetry_records
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    def save_csv(self, filepath: str):
        """Saves telemetry time-series to CSV format."""
        if not self.telemetry_records:
            return
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        fieldnames = list(self.telemetry_records[0].keys())
        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.telemetry_records)

    @classmethod
    def load_json(cls, filepath: str) -> "ExperimentSession":
        """Loads a saved experiment for replay and inspection."""
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)

        session = cls(
            experiment_id=data.get("experiment_id"),
            scenario_name=data.get("scenario_name", "Loaded"),
            software_version=data.get("software_version", "1.0.0"),
            metadata=data.get("metadata", {})
        )
        session.timestamp = data.get("timestamp", session.timestamp)
        session.telemetry_records = data.get("telemetry", [])
        session.state_events = data.get("state_events", [])
        return session
