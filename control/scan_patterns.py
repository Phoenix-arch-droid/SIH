"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: control.scan_patterns

Autonomous Acquisition Search Patterns:
- Archimedean Spiral Scan: sweeps out from center of spatial uncertainty cone
- Localized Reacquisition Spiral: tight high-speed scan around last predicted target state
- Raster Scan: Boustrophedon sweep across rectangular uncertainty boundary
"""

import math
from enum import Enum
from typing import Tuple, Dict, Any


class ScanPatternType(str, Enum):
    ARCHIMEDEAN_SPIRAL = "spiral"
    LOCAL_REACQUISITION = "local_reacquisition"
    RASTER = "raster"


class AutonomousSearchScanner:
    """
    Generates pointing commands for open-loop search and reacquisition modes.
    """

    def __init__(
        self,
        center_pan_deg: float = 0.0,
        center_tilt_deg: float = 0.0,
        max_search_radius_deg: float = 8.0,
        spiral_growth_rate: float = 0.8,     # degrees radius per revolution
        scan_speed_deg_s: float = 12.0       # linear scanning velocity along path
    ):
        self.center_pan_deg = center_pan_deg
        self.center_tilt_deg = center_tilt_deg
        self.max_radius = max_search_radius_deg
        self.growth_rate = spiral_growth_rate
        self.scan_speed = scan_speed_deg_s

        self.elapsed_time_s: float = 0.0
        self.pattern_type = ScanPatternType.ARCHIMEDEAN_SPIRAL

    def reset(self, center_pan_deg: float = 0.0, center_tilt_deg: float = 0.0, pattern: ScanPatternType = ScanPatternType.ARCHIMEDEAN_SPIRAL):
        self.center_pan_deg = center_pan_deg
        self.center_tilt_deg = center_tilt_deg
        self.elapsed_time_s = 0.0
        self.pattern_type = pattern

    def compute_scan_angles(self, dt: float) -> Tuple[float, float, float, float]:
        """
        Advances the scan by dt and returns:
        (target_pan_deg, target_tilt_deg, rate_pan_deg_s, rate_tilt_deg_s)
        """
        self.elapsed_time_s += dt
        t = self.elapsed_time_s

        if self.pattern_type == ScanPatternType.ARCHIMEDEAN_SPIRAL:
            # Constant tangential speed spiral:
            # s(t) = v * t
            # theta(t) = sqrt(2 * v * t / b) where b = growth_rate / (2*pi)
            b = self.growth_rate / (2.0 * math.pi)
            theta = math.sqrt(max(0.0, 2.0 * self.scan_speed * t / max(1e-4, b)))
            r = b * theta

            # Reset or wrap if radius exceeds max uncertainty cone
            if r > self.max_radius:
                self.elapsed_time_s = 0.0
                r = 0.0
                theta = 0.0

            pan_deg = self.center_pan_deg + r * math.cos(theta)
            tilt_deg = self.center_tilt_deg + r * math.sin(theta)

            # Angular rates (derivatives)
            omega = self.scan_speed / max(0.5, r) if r > 0.1 else 2.0
            rate_pan = -r * omega * math.sin(theta)
            rate_tilt = r * omega * math.cos(theta)

            return pan_deg, tilt_deg, rate_pan, rate_tilt

        elif self.pattern_type == ScanPatternType.LOCAL_REACQUISITION:
            # Faster, tighter spiral around last known position (radius <= 2.5 deg)
            max_local_r = 2.5
            b = 0.4 / (2.0 * math.pi)
            theta = math.sqrt(max(0.0, 2.0 * (self.scan_speed * 1.5) * t / max(1e-4, b)))
            r = min(max_local_r, b * theta)
            if r >= max_local_r and t > 3.0:
                self.elapsed_time_s = 0.0

            pan_deg = self.center_pan_deg + r * math.cos(theta)
            tilt_deg = self.center_tilt_deg + r * math.sin(theta)
            return pan_deg, tilt_deg, 0.0, 0.0

        elif self.pattern_type == ScanPatternType.RASTER:
            # Horizontal raster sweeps
            period_h = 3.0 # seconds per line
            half_w = self.max_radius
            phase = (t % period_h) / period_h
            line_idx = int(t / period_h) % 5
            tilt_offset = -self.max_radius + line_idx * (2.0 * self.max_radius / 4.0)

            pan_offset = -half_w + 2.0 * half_w * (phase if line_idx % 2 == 0 else (1.0 - phase))
            return self.center_pan_deg + pan_offset, self.center_tilt_deg + tilt_offset, 0.0, 0.0

        return self.center_pan_deg, self.center_tilt_deg, 0.0, 0.0
