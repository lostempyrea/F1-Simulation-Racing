"""
f1sim/ai/collision.py
=====================
Oriented Bounding Box (OBB) Separating Axis Theorem (SAT) collision detection
(5.63 m x 2.00 m F1 chassis footprint), Time-to-Collision (TTC < 1.2 s)
avoidance & overtaking lane shift (+/- 2.8 m), and aerodynamic slipstream
drafting calculations.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Tuple
import numpy as np


VEHICLE_OBB_LENGTH_M: float = 5.63
VEHICLE_OBB_WIDTH_M: float = 2.00
TTC_AVOIDANCE_THRESHOLD_S: float = 1.2
OVERTAKE_LATERAL_OFFSET_M: float = 2.8


@dataclass
class OBB2D:
    """2D Oriented Bounding Box for an F1 car on the horizontal plane."""
    center_x: float = 0.0
    center_y: float = 0.0
    heading_rad: float = 0.0
    length: float = VEHICLE_OBB_LENGTH_M
    width: float = VEHICLE_OBB_WIDTH_M

    def get_axes(self) -> Tuple[np.ndarray, np.ndarray]:
        c = math.cos(self.heading_rad)
        s = math.sin(self.heading_rad)
        axis_long = np.array([c, s], dtype=np.float64)
        axis_lat = np.array([-s, c], dtype=np.float64)
        return (axis_long, axis_lat)

    def get_corners(self) -> np.ndarray:
        """Returns the (4, 2) array of world-space box corners."""
        ax_l, ax_w = self.get_axes()
        hl = 0.5 * self.length
        hw = 0.5 * self.width
        center = np.array([self.center_x, self.center_y], dtype=np.float64)
        return np.array(
            [
                center + hl * ax_l + hw * ax_w,
                center + hl * ax_l - hw * ax_w,
                center - hl * ax_l - hw * ax_w,
                center - hl * ax_l + hw * ax_w,
            ],
            dtype=np.float64,
        )


def check_obb_intersection(box_a: OBB2D, box_b: OBB2D) -> bool:
    """
    Tests whether two 2D Oriented Bounding Boxes intersect using the
    Separating Axis Theorem (SAT).
    """
    corners_a = box_a.get_corners()
    corners_b = box_b.get_corners()
    axes = list(box_a.get_axes()) + list(box_b.get_axes())

    for axis in axes:
        proj_a = corners_a @ axis
        proj_b = corners_b @ axis
        if float(np.max(proj_a)) < float(np.min(proj_b)) or float(np.max(proj_b)) < float(np.min(proj_a)):
            return False
    return True


def compute_ttc(distance_m: float, v_follower_ms: float, v_leader_ms: float) -> float:
    """
    Computes Time-to-Collision (TTC) in seconds between a trailing car and a leading car.
    When the leader is pulling away (v_rel <= 0.0), returns float('inf') so no
    false collision avoidance braking is triggered.
    """
    dist = max(0.0, float(distance_m))
    v_rel = float(v_follower_ms) - float(v_leader_ms)
    if v_rel <= 0.0:
        return float("inf")
    return dist / v_rel


def compute_overtake_and_avoidance(
    distance_m: float,
    v_follower_ms: float,
    v_leader_ms: float,
    leader_d_m: float = 0.0,
    track_half_width_m: float = 5.5,
    ttc_threshold_s: float = TTC_AVOIDANCE_THRESHOLD_S,
) -> Tuple[float, float, float]:
    """
    Evaluates longitudinal TTC and determines:
        (ttc_sec, target_lateral_offset_m, avoidance_brake_demand)
    If TTC < ttc_threshold_s (1.2s) or distance < 12.0m while closing, commands a
    lateral overtaking lane shift (+/- 2.8m) and proportional emergency brake if blocked.
    """
    ttc = compute_ttc(distance_m, v_follower_ms, v_leader_ms)
    # Strictly disarm when leader is pulling away (negative divergence) or no closing risk exists
    if math.isinf(ttc) or ttc < 0.0:
        return (float("inf"), 0.0, 0.0)
    if ttc >= ttc_threshold_s and distance_m > 10.0:
        return (ttc, 0.0, 0.0)

    # Choose side with more track room (default +2.8m left/inside pass)
    candidate_pos = leader_d_m + OVERTAKE_LATERAL_OFFSET_M
    candidate_neg = leader_d_m - OVERTAKE_LATERAL_OFFSET_M
    margin = max(1.5, float(track_half_width_m) - 1.2)
    can_pass_pos = abs(candidate_pos) <= margin
    can_pass_neg = abs(candidate_neg) <= margin

    if can_pass_pos:
        lateral_shift = candidate_pos
    elif can_pass_neg:
        lateral_shift = candidate_neg
    else:
        # Both sides blocked or track too narrow to shift lanes safely
        lateral_shift = float(np.clip(leader_d_m, -margin, margin))

    brake_demand = 0.0
    is_blocked = (not can_pass_pos and not can_pass_neg) or abs(lateral_shift - leader_d_m) < 1.8
    if ttc < 0.55 and is_blocked:
        brake_demand = min(1.0, (0.55 - ttc) / 0.55)
    return (ttc, float(lateral_shift), float(brake_demand))


def compute_slipstream_drag_multiplier(
    distance_m: float,
    lateral_diff_m: float = 0.0,
    max_distance_m: float = 45.0,
    max_lateral_m: float = 2.5,
) -> float:
    """
    Computes the aerodynamic drag multiplier (< 1.0 in slipstream tow) when
    following within [0.5 m, 45.0 m] behind a leading car.
    """
    dist = float(distance_m)
    lat = abs(float(lateral_diff_m))
    if dist <= 0.5 or dist > max_distance_m or lat > max_lateral_m:
        return 1.0
    long_factor = max(0.0, 1.0 - (dist / max_distance_m))
    lat_factor = max(0.0, 1.0 - (lat / max_lateral_m))
    tow_reduction = 0.18 * long_factor * lat_factor
    return max(0.82, 1.0 - tow_reduction)

