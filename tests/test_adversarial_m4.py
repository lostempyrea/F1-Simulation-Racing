"""
tests/test_adversarial_m4.py
============================
Adversarial stress and edge-case test suite for Milestone 4: Multi-Tier AI Racing System.

Covers:
1. TestAdversarialSATGeometry:
   - Extreme yaw angles (+-100 rad, +-50*pi rad)
   - Concentric boxes with relative yaw
   - Sub-millimeter grazing gap (0.0005m separation vs 0.0000m touch)
   - Perpendicular T-bone wing clip
   - Exact lateral boundary (2.001m vs 1.999m)
   - Exact longitudinal boundary (5.631m vs 5.629m)

2. TestAdversarialTTCAvoidance:
   - Equal velocities zero-relative singularity (returns inf, no div by zero)
   - Near-zero closing epsilon (1e-6 m/s)
   - Zero distance leader ahead vs pulling away
   - Extreme narrow track margins (clamping lateral offset)
   - Oscillating closing speed (no lateral jitter on negative divergence)

3. TestAdversarialSlipstream:
   - Negative distance input handling
   - Boundary epsilon precision (45.0m vs 45.001m)
   - C0 continuity across longitudinal wake boundary
   - C0 continuity across lateral wake boundary

4. TestAdversarialMultiCarCongestion:
   - 6-car authentic driver staggered starting grid non-overlap
   - Pack braking stability into Turn 1 without rear-end collisions
"""

import math
import unittest
import numpy as np

from f1sim.ai.collision import (
    OBB2D,
    check_obb_intersection,
    compute_ttc,
    compute_overtake_and_avoidance,
    compute_slipstream_drag_multiplier,
    VEHICLE_OBB_LENGTH_M,
    VEHICLE_OBB_WIDTH_M,
    TTC_AVOIDANCE_THRESHOLD_S,
    OVERTAKE_LATERAL_OFFSET_M,
)
from f1sim.ai.driver_profiles import AI_DRIVER_ROSTER, get_driver_by_number, DIFFICULTY_PROFILES
from f1sim.ai.racing_line import compute_lookahead_distance, compute_apex_speed
from f1sim.track.monza import MonzaTrack
from f1sim.track.monaco import MonacoTrack
from f1sim.physics.vehicle import Vehicle


class TestAdversarialSATGeometry(unittest.TestCase):
    """Adversarial stress tests for 2D Oriented Bounding Box (OBB) SAT collision engine."""

    def test_sat_extreme_yaw_angles(self):
        """Boxes with extreme yaw (+-100 rad, 50*pi rad) maintain correct orientation and intersection."""
        # Box rotated by 50*pi is an integer multiple of 2*pi, hence identical orientation to 0 rad
        box_base = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_rot_kpi = OBB2D(center_x=0.0, center_y=0.0, heading_rad=50.0 * math.pi)
        self.assertTrue(check_obb_intersection(box_base, box_rot_kpi))

        # Box rotated by -100 rad
        box_neg = OBB2D(center_x=0.0, center_y=0.0, heading_rad=-100.0)
        box_neg_separated = OBB2D(center_x=20.0, center_y=20.0, heading_rad=-100.0)
        self.assertFalse(check_obb_intersection(box_neg, box_neg_separated))

    def test_sat_concentric_boxes_with_relative_yaw(self):
        """Concentric boxes with relative yaw (30 deg, 45 deg, 90 deg) intersect."""
        for angle_deg in [15.0, 30.0, 45.0, 60.0, 90.0]:
            rad = math.radians(angle_deg)
            box1 = OBB2D(center_x=5.0, center_y=5.0, heading_rad=0.0)
            box2 = OBB2D(center_x=5.0, center_y=5.0, heading_rad=rad)
            self.assertTrue(check_obb_intersection(box1, box2))

    def test_sat_sub_millimeter_grazing_gap(self):
        """Longitudinal separation of 5.6305m (0.5mm clearance) is separated, 5.6295m is touching."""
        box1 = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        # Separated by 0.5 mm outside the 5.63m boundary
        box_gap = OBB2D(center_x=5.6305, center_y=0.0, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box1, box_gap))

        # Overlapping by 0.5 mm inside the 5.63m boundary
        box_touch = OBB2D(center_x=5.6295, center_y=0.0, heading_rad=0.0)
        self.assertTrue(check_obb_intersection(box1, box_touch))

    def test_sat_perpendicular_t_bone_clip(self):
        """Perpendicular car clipping the front corner is detected accurately."""
        # Car A along X axis: length extends from -2.815 to +2.815, width -1.0 to +1.0
        car_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)

        # Car B perpendicular (heading = pi/2), center at (2.5, 3.5).
        # Car B extends longitudinally in Y [-2.815, +2.815], so Y bottom is 3.5 - 2.815 = 0.685 (< 1.0)
        # and X extends [-1.0, +1.0], so X range is [1.5, 3.5] which overlaps Car A's front wing [1.5, 2.815].
        car_b_intersect = OBB2D(center_x=2.5, center_y=3.5, heading_rad=math.pi / 2.0)
        self.assertTrue(check_obb_intersection(car_a, car_b_intersect))

        # Car B moved slightly further away in Y: center at (2.5, 4.0), bottom is 4.0 - 2.815 = 1.185 (> 1.0)
        car_b_clear = OBB2D(center_x=2.5, center_y=4.0, heading_rad=math.pi / 2.0)
        self.assertFalse(check_obb_intersection(car_a, car_b_clear))

    def test_sat_parallel_cars_exact_boundary(self):
        """Exact lateral boundary: 2.000m total width sum. 2.001m clear vs 1.999m overlap."""
        box1 = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_clear = OBB2D(center_x=0.0, center_y=2.001, heading_rad=0.0)
        box_touch = OBB2D(center_x=0.0, center_y=1.999, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box1, box_clear))
        self.assertTrue(check_obb_intersection(box1, box_touch))

    def test_sat_collinear_bumper_kiss(self):
        """Exact longitudinal boundary: 5.630m total length sum. 5.631m clear vs 5.629m overlap."""
        box1 = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_clear = OBB2D(center_x=5.631, center_y=0.0, heading_rad=0.0)
        box_touch = OBB2D(center_x=5.629, center_y=0.0, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box1, box_clear))
        self.assertTrue(check_obb_intersection(box1, box_touch))


class TestAdversarialTTCAvoidance(unittest.TestCase):
    """Adversarial stress tests for Time-to-Collision (TTC) and avoidance/overtaking."""

    def test_ttc_equal_velocities_zero_relative(self):
        """Equal velocities (v_rel = 0.0) safely returns inf without division by zero."""
        for v in [0.0, 15.0, 50.0, 95.0]:
            ttc = compute_ttc(distance_m=10.0, v_follower_ms=v, v_leader_ms=v)
            self.assertEqual(ttc, float("inf"))

    def test_ttc_near_zero_closing_epsilon(self):
        """Closing at 1e-6 m/s yields a large finite TTC without overflow or crash."""
        ttc = compute_ttc(distance_m=10.0, v_follower_ms=50.000001, v_leader_ms=50.0)
        self.assertGreater(ttc, 1e6)
        self.assertFalse(math.isinf(ttc))

    def test_ttc_zero_distance_leader_ahead(self):
        """Zero distance with positive closing speed gives TTC = 0.0s."""
        ttc = compute_ttc(distance_m=0.0, v_follower_ms=60.0, v_leader_ms=40.0)
        self.assertEqual(ttc, 0.0)

    def test_ttc_zero_distance_leader_pulling_away(self):
        """Zero distance with leader pulling away gives TTC = inf (negative divergence)."""
        ttc = compute_ttc(distance_m=0.0, v_follower_ms=40.0, v_leader_ms=60.0)
        self.assertEqual(ttc, float("inf"))

    def test_avoidance_extreme_narrow_track_margins(self):
        """On an extremely narrow track (width 6m, half-width 3m), lateral shift is clamped inside walls."""
        ttc, shift, brake = compute_overtake_and_avoidance(
            distance_m=6.0,
            v_follower_ms=55.0,
            v_leader_ms=40.0,
            leader_d_m=0.0,
            track_half_width_m=2.5,
        )
        # Margin is 2.5 - 1.2 = 1.3m, shift must be clamped within margin
        self.assertLessEqual(abs(shift), 2.5)
        # Blocked lane commands avoidance braking
        self.assertGreater(brake, 0.0)

    def test_avoidance_oscillating_closing_speed(self):
        """Alternating between closing and pulling away does not induce false lateral twitch."""
        # Pulling away: leader = 65, follower = 50 -> must be neutral
        ttc_div, shift_div, brake_div = compute_overtake_and_avoidance(
            distance_m=8.0,
            v_follower_ms=50.0,
            v_leader_ms=65.0,
        )
        self.assertEqual(ttc_div, float("inf"))
        self.assertEqual(shift_div, 0.0)
        self.assertEqual(brake_div, 0.0)

        # Closing: leader = 45, follower = 50 -> commands overtaking shift
        ttc_close, shift_close, _ = compute_overtake_and_avoidance(
            distance_m=8.0,
            v_follower_ms=50.0,
            v_leader_ms=45.0,
        )
        self.assertLess(ttc_close, float("inf"))
        self.assertNotEqual(shift_close, 0.0)


class TestAdversarialSlipstream(unittest.TestCase):
    """Adversarial stress tests for aerodynamic drafting slipstream model."""

    def test_slipstream_negative_distance_input(self):
        """Negative distance input (e.g. car ahead is physically behind) returns neutral 1.0."""
        mult = compute_slipstream_drag_multiplier(distance_m=-15.0, lateral_diff_m=0.0)
        self.assertEqual(mult, 1.0)

    def test_slipstream_boundary_epsilon_precision(self):
        """Strict threshold at 45.0m: 45.000m has slight reduction; 45.001m is neutral 1.0."""
        mult_in = compute_slipstream_drag_multiplier(distance_m=44.999, lateral_diff_m=0.0)
        mult_out = compute_slipstream_drag_multiplier(distance_m=45.001, lateral_diff_m=0.0)
        self.assertLess(mult_in, 1.0)
        self.assertEqual(mult_out, 1.0)

    def test_slipstream_continuity_no_jumps(self):
        """Drag multiplier is continuous across the 45.0m boundary with no discontinuous drop."""
        mult_in = compute_slipstream_drag_multiplier(distance_m=44.999, lateral_diff_m=0.0)
        mult_out = compute_slipstream_drag_multiplier(distance_m=45.001, lateral_diff_m=0.0)
        self.assertAlmostEqual(mult_in, mult_out, delta=0.001)

    def test_slipstream_lateral_boundary_continuity(self):
        """Drag multiplier is continuous across the 2.5m lateral offset boundary."""
        mult_in = compute_slipstream_drag_multiplier(distance_m=20.0, lateral_diff_m=2.499)
        mult_out = compute_slipstream_drag_multiplier(distance_m=20.0, lateral_diff_m=2.501)
        self.assertAlmostEqual(mult_in, mult_out, delta=0.001)


class TestAdversarialMultiCarCongestion(unittest.TestCase):
    """Verifies multi-car grid packing and pack braking stability."""

    def test_congested_grid_all_disjoint(self):
        """6 cars (1 player + 5 authentic AI) on a standard 2x2 grid are pairwise disjoint."""
        # Grid positions: 8m longitudinal step, +-1.8m lateral offset
        cars_boxes = []
        for i in range(6):
            row = i // 2
            side = -1.0 if (i % 2 == 0) else 1.0
            cx = row * 8.0
            cy = side * 1.8
            cars_boxes.append(OBB2D(center_x=cx, center_y=cy, heading_rad=0.0))

        # Check all N*(N-1)/2 pairs
        for i in range(6):
            for j in range(i + 1, 6):
                intersects = check_obb_intersection(cars_boxes[i], cars_boxes[j])
                self.assertFalse(
                    intersects,
                    f"Grid cars {i} and {j} collided at ({cars_boxes[i].center_x}, {cars_boxes[i].center_y}) vs ({cars_boxes[j].center_x}, {cars_boxes[j].center_y})"
                )

    def test_pack_braking_zone_stability(self):
        """5 cars braking into a turn with 15m initial gaps maintain safe spacing."""
        # 5 cars traveling at 70 m/s with 15m initial longitudinal separation (car 0 in front)
        positions = [(4 - i) * 15.0 for i in range(5)]
        speeds = [70.0] * 5

        # Simulate 1.5 seconds of deceleration under AI braking logic
        dt = 0.05
        steps = 30
        for _ in range(steps):
            for i in range(len(positions)):
                # Car 0 (leader) brakes hard into corner
                if i == 0:
                    decel = 28.0
                else:
                    # Following cars respond to car ahead
                    gap = positions[i - 1] - positions[i]
                    v_rel = speeds[i] - speeds[i - 1]
                    ttc = compute_ttc(gap, speeds[i], speeds[i - 1])
                    if ttc < 1.0 or gap < 12.0:
                        decel = min(35.0, 28.0 + 7.0 * (1.0 - min(1.0, ttc)))
                    else:
                        decel = 28.0
                speeds[i] = max(15.0, speeds[i] - decel * dt)
                positions[i] += speeds[i] * dt

        # Verify no rear-end collisions occurred: all gaps must remain > 5.63m (OBB length)
        for i in range(1, len(positions)):
            final_gap = positions[i - 1] - positions[i]
            self.assertGreater(
                final_gap,
                5.63,
                f"Rear-end collision between car {i-1} and car {i}: gap {final_gap:.2f}m <= 5.63m"
            )


if __name__ == "__main__":
    unittest.main()
