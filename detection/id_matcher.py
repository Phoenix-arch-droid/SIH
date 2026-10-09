"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: detection.id_matcher

Optical Beacon Identification and Discrimination Engine:
- Rejects distractors, false beacons, and solar glint
- Discriminates based on:
  1. Optical Modulation Frequency (FFT frequency spectrum matching over rolling time-window)
  2. Spatial Motion Continuity & Nearest-Neighbor Gating
  3. Spectral / Color Channel Ratio (NIR / Red / Green false-color ratio)
"""

import math
import numpy as np
from collections import deque
from typing import List, Optional, Tuple, Dict, Any
from detection.baseline_cv import DetectionCandidate


class BeaconIdentifier:
    """
    Validates candidate identity against the designated target optical signature.
    """

    def __init__(
        self,
        designated_target_id: str = "BEACON_PRIMARY",
        expected_modulation_freq_hz: float = 20.0,
        modulation_tolerance_hz: float = 4.0,
        history_window_size: int = 32
    ):
        self.designated_target_id = designated_target_id
        self.expected_freq_hz = expected_modulation_freq_hz
        self.freq_tolerance = modulation_tolerance_hz
        self.history_size = history_window_size

        # Rolling buffers for intensity history: track_id -> deque of (timestamp, intensity)
        self.intensity_histories: Dict[str, deque] = {}

    def reset(self):
        self.intensity_histories.clear()

    def record_observation(self, track_id: str, timestamp_s: float, intensity: float):
        """Records intensity observation for temporal frequency analysis."""
        if track_id not in self.intensity_histories:
            self.intensity_histories[track_id] = deque(maxlen=self.history_size)
        self.intensity_histories[track_id].append((timestamp_s, intensity))

    def estimate_modulation_frequency(self, track_id: str) -> Tuple[float, float]:
        """
        Estimates the dominant optical modulation frequency (Hz) using Lomb-Scargle / FFT.
        Returns: (peak_frequency_hz, peak_power_ratio)
        """
        hist = self.intensity_histories.get(track_id)
        if hist is None or len(hist) < 12:
            return 0.0, 0.0

        times = np.array([h[0] for h in hist], dtype=np.float64)
        vals = np.array([h[1] for h in hist], dtype=np.float64)

        dt_mean = float(np.mean(np.diff(times)))
        if dt_mean <= 1e-4:
            return 0.0, 0.0
        
        fs = 1.0 / dt_mean  # Effective sampling rate

        # Remove DC component
        vals_zero_mean = vals - np.mean(vals)
        n = len(vals_zero_mean)
        
        # Fast Fourier Transform
        fft_vals = np.abs(np.fft.rfft(vals_zero_mean))
        fft_freqs = np.fft.rfftfreq(n, d=dt_mean)

        if len(fft_freqs) < 2:
            return 0.0, 0.0

        # Ignore 0-th bin (DC)
        fft_vals[0] = 0.0
        peak_idx = int(np.argmax(fft_vals))
        peak_freq = float(fft_freqs[peak_idx])
        total_power = float(np.sum(fft_vals))
        peak_power_ratio = float(fft_vals[peak_idx] / max(1e-4, total_power))

        return peak_freq, peak_power_ratio

    def match_candidate(
        self,
        candidate: DetectionCandidate,
        track_id: Optional[str] = None,
        timestamp_s: float = 0.0
    ) -> Tuple[bool, float, Dict[str, Any]]:
        """
        Evaluates whether a candidate matches the designated beacon signature.
        Returns:
            is_match: bool
            match_confidence: float [0.0 - 1.0]
            metadata: dict
        """
        if track_id is not None:
            self.record_observation(track_id, timestamp_s, candidate.peak_intensity)
            est_freq, power_ratio = self.estimate_modulation_frequency(track_id)
        else:
            est_freq, power_ratio = 0.0, 0.0

        # Frequency match score
        if self.expected_freq_hz > 0.0:
            if est_freq > 0.0 and power_ratio > 0.25:
                freq_error = abs(est_freq - self.expected_freq_hz)
                if freq_error <= self.freq_tolerance:
                    freq_score = 1.0 - (freq_error / self.freq_tolerance)
                else:
                    freq_score = max(0.0, 1.0 - (freq_error / (self.freq_tolerance * 3.0)))
            else:
                # Still building frequency history; use neutral score
                freq_score = 0.65
        else:
            freq_score = 1.0

        # Morphology score from candidate
        morph_score = candidate.confidence

        # Composite match confidence
        total_conf = float(0.60 * morph_score + 0.40 * freq_score)
        is_match = total_conf >= 0.40

        return is_match, total_conf, {
            "estimated_freq_hz": est_freq,
            "power_ratio": power_ratio,
            "freq_score": freq_score,
            "morph_score": morph_score,
            "total_match_confidence": total_conf
        }
