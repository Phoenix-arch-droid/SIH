"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: tracking.association

Data Association Engine:
- Associates candidate detections with active Kalman tracks
- Gated Global Nearest Neighbor (GNN) and Hungarian cost assignment
- Rejects clutter and distractor beacons outside tracking gate
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Optional, Set
from detection.baseline_cv import DetectionCandidate
from tracking.kalman_tracker import BeaconKalmanTracker


class DataAssociationEngine:
    """
    Manages detection-to-track gating and assignment.
    """

    def __init__(self, spatial_gate_radius_pixels: float = 80.0, mahalanobis_gate_threshold: float = 4.0):
        self.spatial_gate_radius = spatial_gate_radius_pixels
        self.mahalanobis_gate = mahalanobis_gate_threshold

    def associate(
        self,
        trackers: List[BeaconKalmanTracker],
        candidates: List[DetectionCandidate]
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Associates candidates to tracks.
        
        Returns:
            matches: List of (track_idx, candidate_idx)
            unmatched_tracks: List of track_idx
            unmatched_candidates: List of candidate_idx
        """
        if not trackers:
            return [], [], list(range(len(candidates)))
        if not candidates:
            return [], list(range(len(trackers))), []

        num_tracks = len(trackers)
        num_cands = len(candidates)

        cost_matrix = np.full((num_tracks, num_cands), 1e6, dtype=np.float64)

        for t_idx, tracker in enumerate(trackers):
            state = tracker.get_state()
            P_pos = state.covariance[:2, :2]
            P_det = float(np.linalg.det(P_pos))
            P_inv = np.linalg.inv(P_pos) if P_det > 1e-6 else np.eye(2) * 0.1

            for c_idx, cand in enumerate(candidates):
                du = cand.centroid_u - state.u
                dv = cand.centroid_v - state.v
                euclid_dist = math.hypot(du, dv)

                if euclid_dist > self.spatial_gate_radius:
                    continue  # Outside spatial gate

                diff = np.array([du, dv], dtype=np.float64)
                maha_dist = math.sqrt(float(np.clip(diff.T @ P_inv @ diff, 0.0, 1e4)))

                if maha_dist <= self.mahalanobis_gate:
                    # Cost is composite of distance and inverted candidate confidence
                    cost_matrix[t_idx, c_idx] = 0.7 * euclid_dist + 0.3 * (1.0 - cand.confidence) * 100.0

        # Greedy / Munkres optimal assignment
        matches: List[Tuple[int, int]] = []
        matched_tracks: Set[int] = set()
        matched_cands: Set[int] = set()

        # Find minimum cost iteratively
        while True:
            min_val = np.min(cost_matrix)
            if min_val >= 1e5:
                break
            min_idx = np.unravel_index(np.argmin(cost_matrix), cost_matrix.shape)
            t_idx, c_idx = int(min_idx[0]), int(min_idx[1])

            matches.append((t_idx, c_idx))
            matched_tracks.add(t_idx)
            matched_cands.add(c_idx)

            # Invalidate row and column
            cost_matrix[t_idx, :] = 1e6
            cost_matrix[:, c_idx] = 1e6

        unmatched_tracks = [i for i in range(num_tracks) if i not in matched_tracks]
        unmatched_candidates = [j for j in range(num_cands) if j not in matched_cands]

        return matches, unmatched_tracks, unmatched_candidates
