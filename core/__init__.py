"""
Core simulation package for SIH26169 FSOC Virtual Tracking System.
"""
from core.coordinates import Pose3D, CoordinateTransform
from core.camera_model import CameraIntrinsics, VirtualCamera
from core.targets import (
    TrajectoryType,
    BeaconOpticalSignature,
    OpticalBeaconTarget,
    BeaconTargetManager
)
from core.disturbances import DisturbanceLevel, DisturbanceConfig, DisturbanceEngine
from core.state_machine import AlignmentState, CoarseAlignmentStateMachine, StateTransitionEvent
