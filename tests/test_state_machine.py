"""
Unit tests for core.state_machine module.
Validates the 7 formal alignment state transitions:
SEARCHING -> ACQUIRING -> TRACKING -> LOCKED -> DEGRADED -> LOST -> REACQUIRING.
"""

import pytest
from core.state_machine import AlignmentState, CoarseAlignmentStateMachine


def test_state_machine_initial_state():
    sm = CoarseAlignmentStateMachine()
    assert sm.current_state == AlignmentState.SEARCHING


def test_transition_searching_to_acquiring():
    sm = CoarseAlignmentStateMachine()
    st = sm.update(
        dt=0.033, timestamp_s=0.1,
        target_detected=True, angular_error_deg=4.0, tracking_confidence=0.8,
        is_target_in_fov=True
    )
    assert st == AlignmentState.ACQUIRING


def test_transition_acquiring_to_tracking():
    sm = CoarseAlignmentStateMachine(acquisition_confirm_frames=3)
    # Feed 3 consecutive detections
    for i in range(3):
        st = sm.update(
            dt=0.033, timestamp_s=(i + 1) * 0.033,
            target_detected=True, angular_error_deg=3.0, tracking_confidence=0.8,
            is_target_in_fov=True
        )
    assert st == AlignmentState.TRACKING


def test_transition_tracking_to_locked():
    sm = CoarseAlignmentStateMachine(lock_threshold_deg=0.5, lock_confirm_frames=4)
    # Advance to tracking
    for i in range(5):
        sm.update(0.033, i * 0.033, True, 2.0, 0.9, True)
    assert sm.current_state == AlignmentState.TRACKING

    # Now error drops below 0.5 deg for 4 consecutive frames
    for i in range(4):
        st = sm.update(0.033, (i + 5) * 0.033, True, 0.35, 0.95, True)
    assert st == AlignmentState.LOCKED


def test_transition_locked_to_degraded_and_lost():
    sm = CoarseAlignmentStateMachine(lock_threshold_deg=0.5, lost_frame_threshold=5)
    # Lock first
    for i in range(10):
        sm.update(0.033, i * 0.033, True, 0.2, 0.9, True)
    assert sm.current_state == AlignmentState.LOCKED

    # Target drops for 3 frames -> DEGRADED
    for i in range(3):
        st = sm.update(0.033, (i + 10) * 0.033, False, 0.2, 0.2, False)
    assert st == AlignmentState.DEGRADED

    # Target missing for 2 more frames (reaching lost_frame_threshold=5) -> transitions to LOST
    for i in range(2):
        st = sm.update(0.033, (i + 13) * 0.033, False, 0.2, 0.1, False)
    assert st == AlignmentState.LOST

    # Subsequent frame while lost without detection transitions to REACQUIRING
    st = sm.update(0.033, 15 * 0.033, False, 0.2, 0.1, False)
    assert st == AlignmentState.REACQUIRING


def test_transition_lost_to_reacquiring():
    sm = CoarseAlignmentStateMachine()
    sm.current_state = AlignmentState.LOST
    # One update while lost without detection transitions to REACQUIRING
    st = sm.update(0.033, 1.0, False, 5.0, 0.0, False)
    assert st == AlignmentState.REACQUIRING
