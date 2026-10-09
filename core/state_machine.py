"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: core.state_machine

Formal 7-state alignment state machine for FSOC coarse-pointing acquisition and tracking:
1. SEARCHING: Systematic scanning over uncertainty cone
2. ACQUIRING: Beacon candidate detected; verifying signature & establishing track
3. TRACKING: Closed-loop gimbal repositioning to null pointing error
4. LOCKED: Coarse alignment achieved (error <= lock threshold for >= N frames)
5. DEGRADED: Tracking maintained under severe noise / intermittent drops
6. LOST: Target absent from FOV beyond tolerance window
7. REACQUIRING: Local predictive search around last known trajectory

Transitions are governed entirely by real sensor measurements, Kalman covariance,
and angular errors. Ground truth is NEVER used.
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Callable


class AlignmentState(str, Enum):
    SEARCHING = "SEARCHING"
    ACQUIRING = "ACQUIRING"
    TRACKING = "TRACKING"
    LOCKED = "LOCKED"
    DEGRADED = "DEGRADED"
    LOST = "LOST"
    REACQUIRING = "REACQUIRING"


@dataclass
class StateTransitionEvent:
    timestamp_s: float
    from_state: AlignmentState
    to_state: AlignmentState
    reason: str
    angular_error_deg: float
    confidence: float
    target_id: Optional[str] = None


class CoarseAlignmentStateMachine:
    """
    Implements the real-time FSOC coarse alignment state machine.
    """

    def __init__(
        self,
        lock_threshold_deg: float = 0.5,        # 0.5 deg (approx 8.7 mrad) coarse alignment lock threshold
        lock_confirm_frames: int = 5,           # Number of consecutive frames inside threshold to declare LOCKED
        acquisition_confirm_frames: int = 3,    # Consecutive frames to advance from ACQUIRING to TRACKING
        lost_frame_threshold: int = 12,         # Consecutive missed frames to declare LOST
        degraded_confidence_threshold: float = 0.45 # Confidence below which track is DEGRADED
    ):
        self.lock_threshold_deg = lock_threshold_deg
        self.lock_confirm_frames = lock_confirm_frames
        self.acquisition_confirm_frames = acquisition_confirm_frames
        self.lost_frame_threshold = lost_frame_threshold
        self.degraded_confidence_threshold = degraded_confidence_threshold

        self.current_state = AlignmentState.SEARCHING
        self.previous_state = AlignmentState.SEARCHING
        
        # Consecutive frame counters
        self.consecutive_detection_frames: int = 0
        self.consecutive_locked_frames: int = 0
        self.consecutive_lost_frames: int = 0
        self.reacquisition_timer_s: float = 0.0

        # State transition history
        self.history: List[StateTransitionEvent] = []
        self.state_time_s: float = 0.0

    def reset(self):
        """Resets state machine to initial SEARCHING state."""
        self.current_state = AlignmentState.SEARCHING
        self.previous_state = AlignmentState.SEARCHING
        self.consecutive_detection_frames = 0
        self.consecutive_locked_frames = 0
        self.consecutive_lost_frames = 0
        self.reacquisition_timer_s = 0.0
        self.history.clear()
        self.state_time_s = 0.0

    def _transition(
        self,
        new_state: AlignmentState,
        timestamp_s: float,
        reason: str,
        angular_error_deg: float,
        confidence: float,
        target_id: Optional[str] = None
    ):
        if new_state == self.current_state:
            return
        
        event = StateTransitionEvent(
            timestamp_s=timestamp_s,
            from_state=self.current_state,
            to_state=new_state,
            reason=reason,
            angular_error_deg=angular_error_deg,
            confidence=confidence,
            target_id=target_id
        )
        self.history.append(event)
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_time_s = 0.0

    def update(
        self,
        dt: float,
        timestamp_s: float,
        target_detected: bool,
        angular_error_deg: float,
        tracking_confidence: float,
        is_target_in_fov: bool,
        target_id: Optional[str] = None
    ) -> AlignmentState:
        """
        Advances the state machine by one simulation frame based on actual tracking observations.
        """
        self.state_time_s += dt

        if target_detected:
            self.consecutive_detection_frames += 1
            self.consecutive_lost_frames = 0
        else:
            self.consecutive_lost_frames += 1
            # Optical pulse modulation allows brief off-cycles without instantly wiping detection streak
            if self.consecutive_lost_frames > 2:
                self.consecutive_detection_frames = 0
                self.consecutive_locked_frames = 0

        # State evaluation logic
        if self.current_state == AlignmentState.SEARCHING:
            if target_detected and tracking_confidence >= 0.3:
                self._transition(
                    AlignmentState.ACQUIRING, timestamp_s,
                    "Beacon candidate detected in FOV",
                    angular_error_deg, tracking_confidence, target_id
                )

        elif self.current_state == AlignmentState.ACQUIRING:
            if not target_detected:
                if self.consecutive_lost_frames >= 4:
                    self._transition(
                        AlignmentState.SEARCHING, timestamp_s,
                        "Candidate lost during acquisition confirmation",
                        angular_error_deg, tracking_confidence, target_id
                    )
            elif self.consecutive_detection_frames >= self.acquisition_confirm_frames:
                self._transition(
                    AlignmentState.TRACKING, timestamp_s,
                    f"Beacon signature validated across {self.consecutive_detection_frames} frames",
                    angular_error_deg, tracking_confidence, target_id
                )

        elif self.current_state == AlignmentState.TRACKING:
            if not target_detected:
                if self.consecutive_lost_frames >= 3:
                    self._transition(
                        AlignmentState.DEGRADED, timestamp_s,
                        "Intermittent target detection drop",
                        angular_error_deg, tracking_confidence, target_id
                    )
            elif angular_error_deg <= self.lock_threshold_deg:
                self.consecutive_locked_frames += 1
                if self.consecutive_locked_frames >= self.lock_confirm_frames:
                    self._transition(
                        AlignmentState.LOCKED, timestamp_s,
                        f"Angular error ({angular_error_deg:.3f} deg) <= lock threshold ({self.lock_threshold_deg} deg)",
                        angular_error_deg, tracking_confidence, target_id
                    )
            else:
                self.consecutive_locked_frames = 0
                if tracking_confidence < self.degraded_confidence_threshold:
                    self._transition(
                        AlignmentState.DEGRADED, timestamp_s,
                        f"Confidence ({tracking_confidence:.2f}) dropped below threshold",
                        angular_error_deg, tracking_confidence, target_id
                    )

        elif self.current_state == AlignmentState.LOCKED:
            if not target_detected or angular_error_deg > self.lock_threshold_deg:
                self.consecutive_locked_frames = 0
                if not target_detected:
                    self._transition(
                        AlignmentState.DEGRADED, timestamp_s,
                        "Beacon drop during locked tracking",
                        angular_error_deg, tracking_confidence, target_id
                    )
                else:
                    self._transition(
                        AlignmentState.TRACKING, timestamp_s,
                        f"Angular error ({angular_error_deg:.3f} deg) exceeded lock threshold",
                        angular_error_deg, tracking_confidence, target_id
                    )
            else:
                self.consecutive_locked_frames += 1

        elif self.current_state == AlignmentState.DEGRADED:
            if target_detected and tracking_confidence >= self.degraded_confidence_threshold:
                if angular_error_deg <= self.lock_threshold_deg:
                    self._transition(
                        AlignmentState.LOCKED, timestamp_s,
                        "Signal quality restored and within lock boundary",
                        angular_error_deg, tracking_confidence, target_id
                    )
                else:
                    self._transition(
                        AlignmentState.TRACKING, timestamp_s,
                        "Signal quality recovered; tracking resumed",
                        angular_error_deg, tracking_confidence, target_id
                    )
            elif self.consecutive_lost_frames >= self.lost_frame_threshold:
                self._transition(
                    AlignmentState.LOST, timestamp_s,
                    f"Target undetected for {self.consecutive_lost_frames} consecutive frames",
                    angular_error_deg, tracking_confidence, target_id
                )

        elif self.current_state == AlignmentState.LOST:
            if target_detected:
                self._transition(
                    AlignmentState.ACQUIRING, timestamp_s,
                    "Target re-sighted after loss",
                    angular_error_deg, tracking_confidence, target_id
                )
            else:
                # Enter local reacquisition search
                self._transition(
                    AlignmentState.REACQUIRING, timestamp_s,
                    "Initiating predictive reacquisition scan",
                    angular_error_deg, tracking_confidence, target_id
                )

        elif self.current_state == AlignmentState.REACQUIRING:
            if target_detected:
                self._transition(
                    AlignmentState.ACQUIRING, timestamp_s,
                    "Beacon reacquired during localized search",
                    angular_error_deg, tracking_confidence, target_id
                )
            elif self.state_time_s >= 4.0:  # If reacquisition fails after 4.0 seconds, fall back to wide search
                self._transition(
                    AlignmentState.SEARCHING, timestamp_s,
                    "Predictive reacquisition timed out; resuming wide-area search",
                    angular_error_deg, tracking_confidence, target_id
                )

        return self.current_state
