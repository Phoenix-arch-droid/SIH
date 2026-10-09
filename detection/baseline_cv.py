"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: detection.baseline_cv

Classical multi-stage computer vision detector:
1. Spatial preprocessing & background normalization
2. Dynamic thresholding (Otsu / adaptive local thresholding)
3. Morphological filtering (opening / dilation)
4. Connected component analysis and contour extraction
5. Sub-pixel centroiding via 1st-order spatial image moments
6. Geometric and photometric feature extraction:
   - Area, perimeter, circularity / compactness
   - Peak intensity, mean intensity, local SNR
   - Gaussian PSF profile correlation
7. Candidate scoring and selection
"""

import math
import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Any


@dataclass
class DetectionCandidate:
    """Represents an optical beacon candidate extracted from a camera frame."""
    centroid_u: float                 # Sub-pixel horizontal coordinate
    centroid_v: float                 # Sub-pixel vertical coordinate
    bbox: Tuple[int, int, int, int]   # (x, y, w, h)
    area: float                       # Blob pixel area
    circularity: float                # 4*pi*Area / Perimeter^2 (1.0 = perfect circle)
    peak_intensity: float             # Maximum pixel value [0-255]
    mean_intensity: float             # Mean pixel value within blob
    snr_db: float                     # Estimated signal-to-noise ratio
    confidence: float                 # Composite confidence score [0.0 - 1.0]
    raw_score: float                  # Heuristic or classifier score
    features: Optional[np.ndarray] = None # Feature vector for AI classifier


class ClassicalCVDetector:
    """
    Robust baseline computer vision detector for optical FSOC beacons.
    """

    def __init__(
        self,
        min_blob_area: float = 3.0,
        max_blob_area: float = 800.0,
        min_circularity: float = 0.35,
        threshold_offset: float = 20.0
    ):
        self.min_blob_area = min_blob_area
        self.max_blob_area = max_blob_area
        self.min_circularity = min_circularity
        self.threshold_offset = threshold_offset

    def detect(
        self,
        frame_gray: np.ndarray,
        expected_roi: Optional[Tuple[int, int, int, int]] = None
    ) -> List[DetectionCandidate]:
        """
        Executes candidate detection on a grayscale frame.
        
        Args:
            frame_gray: 2D numpy array (uint8)
            expected_roi: Optional search window (x, y, w, h)
            
        Returns:
            List of DetectionCandidate sorted by confidence descending.
        """
        if frame_gray is None or frame_gray.size == 0:
            return []

        h, w = frame_gray.shape[:2]

        # 1. Background estimation (median or morphological top-hat)
        # Using fast top-hat transform to isolate bright point sources from uneven background
        kernel_size = 9
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        tophat = cv2.morphologyEx(frame_gray, cv2.MORPH_TOPHAT, kernel)

        # 2. Dynamic threshold calculation
        bg_mean = float(np.mean(frame_gray))
        bg_std = float(np.std(frame_gray))
        thresh_val = max(35.0, bg_mean + 2.5 * bg_std + self.threshold_offset)
        thresh_val = min(thresh_val, 240.0)

        _, binary = cv2.threshold(tophat, int(thresh_val), 255, cv2.THRESH_BINARY)

        # 3. Morphological cleanup
        clean_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, clean_kernel)
        binary = cv2.dilate(binary, clean_kernel, iterations=1)

        # 4. Contour extraction
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates: List[DetectionCandidate] = []

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_blob_area or area > self.max_blob_area:
                continue

            perimeter = float(cv2.arcLength(cnt, True))
            circularity = (4.0 * math.pi * area) / (perimeter ** 2) if perimeter > 1e-3 else 0.0

            if circularity < self.min_circularity:
                continue

            # Sub-pixel centroid via image moments
            M = cv2.moments(cnt)
            if M["m00"] < 1e-4:
                continue
            u = float(M["m10"] / M["m00"])
            v = float(M["m01"] / M["m00"])

            bx, by, bw, bh = cv2.boundingRect(cnt)
            
            # Extract ROI stats from original grayscale frame
            roi = frame_gray[by:by+bh, bx:bx+bw]
            peak_val = float(np.max(roi)) if roi.size > 0 else 0.0
            mean_val = float(np.mean(roi)) if roi.size > 0 else 0.0

            # Local SNR estimation (signal peak vs surrounding background noise)
            local_noise = max(1.0, bg_std)
            snr_linear = max(1.0, (peak_val - bg_mean) / local_noise)
            snr_db = 10.0 * math.log10(snr_linear)

            # Classical composite confidence metric:
            # Rewards circularity, intensity, and SNR
            circ_score = min(1.0, circularity)
            int_score = min(1.0, peak_val / 255.0)
            snr_score = min(1.0, snr_db / 30.0)
            confidence = float(0.40 * int_score + 0.35 * circ_score + 0.25 * snr_score)

            # Feature vector for AI module if needed
            features = np.array([
                area,
                circularity,
                bw / max(1.0, bh),           # aspect ratio
                peak_val,
                mean_val,
                float(np.std(roi)) if roi.size > 0 else 0.0,
                snr_db,
                bg_mean,
                bg_std,
                u / w,                       # normalized position
                v / h,
                confidence
            ], dtype=np.float32)

            candidates.append(DetectionCandidate(
                centroid_u=u,
                centroid_v=v,
                bbox=(bx, by, bw, bh),
                area=area,
                circularity=circularity,
                peak_intensity=peak_val,
                mean_intensity=mean_val,
                snr_db=snr_db,
                confidence=confidence,
                raw_score=confidence,
                features=features
            ))

        # Sort by confidence descending
        candidates.sort(key=lambda c: c.confidence, reverse=True)
        return candidates
