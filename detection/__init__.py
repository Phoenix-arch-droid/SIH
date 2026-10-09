"""
Detection package for SIH26169 FSOC Virtual Tracking System.
"""
from detection.baseline_cv import DetectionCandidate, ClassicalCVDetector
from detection.ai_detector import AIEnhancedBeaconDetector
from detection.id_matcher import BeaconIdentifier
