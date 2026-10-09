"""
Unit tests for detection package:
- ClassicalCVDetector
- AIEnhancedBeaconDetector
- BeaconIdentifier
Validates subpixel centroid accuracy, candidate extraction, and AI feature evaluation.
"""

import math
import numpy as np
import cv2
import pytest
from detection.baseline_cv import ClassicalCVDetector, DetectionCandidate
from detection.ai_detector import AIEnhancedBeaconDetector
from detection.id_matcher import BeaconIdentifier


def _generate_synthetic_spot_frame(cx: float, cy: float, sigma: float = 2.5, peak: float = 200.0) -> np.ndarray:
    """Creates a clean synthetic frame with a 2D Gaussian spot."""
    h, w = 360, 640
    frame = np.full((h, w), 20.0, dtype=np.float32) # Background level 20
    y, x = np.ogrid[:h, :w]
    spot = peak * np.exp(-((x - cx)**2 + (y - cy)**2) / (2.0 * sigma**2))
    frame += spot
    return np.clip(frame, 0.0, 255.0).astype(np.uint8)


def test_classical_detector_subpixel_centroid():
    true_u, true_v = 320.45, 180.72
    frame = _generate_synthetic_spot_frame(true_u, true_v)
    detector = ClassicalCVDetector()
    candidates = detector.detect(frame)

    assert len(candidates) >= 1
    best = candidates[0]
    # Centroid should match true subpixel coordinates within ~0.25 pixels
    assert math.isclose(best.centroid_u, true_u, abs_tol=0.3)
    assert math.isclose(best.centroid_v, true_v, abs_tol=0.3)
    assert best.circularity > 0.70
    assert best.confidence > 0.50


def test_ai_detector_beacon_classification():
    true_u, true_v = 320.0, 180.0
    frame = _generate_synthetic_spot_frame(true_u, true_v)
    ai_det = AIEnhancedBeaconDetector()
    candidates = ai_det.detect(frame)

    assert len(candidates) >= 1
    best = candidates[0]
    # AI detector should evaluate high confidence on a clean Airy/Gaussian spot
    assert best.confidence > 0.60
    assert best.features is not None
    assert len(best.features) == 12


def test_ai_detector_rejection_of_empty_frame():
    # Empty uniform background
    frame = np.full((360, 640), 25, dtype=np.uint8)
    ai_det = AIEnhancedBeaconDetector()
    candidates = ai_det.detect(frame)
    assert len(candidates) == 0


def test_beacon_identifier_matching():
    identifier = BeaconIdentifier(expected_modulation_freq_hz=20.0)
    cand = DetectionCandidate(
        centroid_u=320.0,
        centroid_v=180.0,
        bbox=(310, 170, 20, 20),
        area=100.0,
        circularity=0.92,
        peak_intensity=220.0,
        mean_intensity=150.0,
        snr_db=25.0,
        confidence=0.88,
        raw_score=0.88
    )

    # Feed steady sequence of observations at 20 Hz
    dt = 0.033
    for i in range(25):
        t = i * dt
        # 20 Hz oscillation in intensity
        val = 180.0 + 40.0 * math.sin(2.0 * math.pi * 20.0 * t)
        cand.peak_intensity = val
        is_match, conf, _ = identifier.match_candidate(cand, track_id="BEACON_PRIMARY", timestamp_s=t)

    assert conf > 0.40
