"""
tests/test_gate_m4_challenger2.py
=================================
Empirical Adversarial Verification Test Suite for Milestone 4: Multi-Tier AI Racing System.
Author: challenger_m4_2 (teamwork_challenger)

Empirically stress-tests:
1. OBB SAT Geometric Collision Detection:
   - Footprint verification: exact length 5.63m and width 2.00m.
   - Rotational invariance: box intersection across arbitrary yaw angles:
     psi_A in [0, pi/6, pi/4, pi/3, pi/2, 3pi/4, pi], psi_B in [0, pi/4, pi/2].
   - Boundary overlap precision:
     * Collinear touching: longitudinal distance 5.630m vs 5.635m.
     * Lateral touching: lateral distance 2.000m vs 2.005m.
     * T-bone collision (90 deg yaw difference) and oblique corner-to-edge penetration.
2. TTC Negative Divergence Protection:
   - Leader pulling away (v_leader > v_follower or v_rel <= 0) across tight gaps:
     distance in [0.1, 0.5, 1.0, 5.0, 8.0, 9.9, 10.0, 10.1, 50.0] m.
   - Assert compute_ttc returns inf and avoidance returns (inf, 0.0, 0.0) - zero swerve, zero brake.
3. TTC Avoidance & Overtake Maneuver:
   - Follower approaching slower leader (v_follower > v_leader) when TTC < 1.2s.
   - Verify avoidance lateral offset is commanded (+-2.8m) and braking is triggered.
   - Boxed-in scenario: when both left and right lanes blocked, emergency deceleration commanded.
4. Slipstream Drafting Tow Cone:
   - Drag multiplier across distances d in [0.0, 5.0, 20.0, 44.9, 45.1, 60.0] m
     and lateral offsets dy in [0.0, 1.0, 2.4, 2.6, 5.0] m (30 points matrix).
   - Continuous decay within cone and exact 1.0 outside cone.
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
from f1sim.physics.aero import Aerodynamics, AeroConfig


class TestOBBSATGeometricCollisionChallenger(unittest.TestCase):
    """
    Task 1: OBB SAT Geometric Collision Detection Adversarial Stress.
    """

    def test_obb_footprint_exact_dimensions(self):
        """Footprint verification: assert exact length 5.63m and width 2.00m."""
        # 1. Module-level constants
        self.assertEqual(VEHICLE_OBB_LENGTH_M, 5.63, "VEHICLE_OBB_LENGTH_M must be exactly 5.63m")
        self.assertEqual(VEHICLE_OBB_WIDTH_M, 2.00, "VEHICLE_OBB_WIDTH_M must be exactly 2.00m")

        # 2. OBB2D default instance attributes
        box = OBB2D()
        self.assertEqual(box.length, 5.63, "OBB2D default length must be 5.63m")
        self.assertEqual(box.width, 2.00, "OBB2D default width must be 2.00m")

        # 3. Geometric corner extent checks
        corners = box.get_corners()
        self.assertEqual(corners.shape, (4, 2))
        x_span = float(np.max(corners[:, 0]) - np.min(corners[:, 0]))
        y_span = float(np.max(corners[:, 1]) - np.min(corners[:, 1]))
        self.assertAlmostEqual(x_span, 5.63, places=6, msg="X span of unrotated box must equal length 5.63m")
        self.assertAlmostEqual(y_span, 2.00, places=6, msg="Y span of unrotated box must equal width 2.00m")

    def test_sat_rotational_invariance_arbitrary_yaw(self):
        """
        Rotational invariance: test box intersection across arbitrary yaw angles:
        psi_A in [0, pi/6, pi/4, pi/3, pi/2, 3pi/4, pi]
        psi_B in [0, pi/4, pi/2]
        """
        angles_a = [0.0, math.pi / 6.0, math.pi / 4.0, math.pi / 3.0, math.pi / 2.0, 3.0 * math.pi / 4.0, math.pi]
        angles_b = [0.0, math.pi / 4.0, math.pi / 2.0]

        total_tested = 0
        for psi_a in angles_a:
            for psi_b in angles_b:
                total_tested += 1
                # Concentric boxes at (0, 0) MUST intersect regardless of yaw combination
                box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=psi_a)
                box_b_concentric = OBB2D(center_x=0.0, center_y=0.0, heading_rad=psi_b)
                self.assertTrue(
                    check_obb_intersection(box_a, box_b_concentric),
                    f"Concentric boxes failed intersection check at psi_a={psi_a:.4f}, psi_b={psi_b:.4f}"
                )

                # Small shift (0.4m, 0.4m) well within both chassis footprints MUST intersect
                box_b_shifted = OBB2D(center_x=0.4, center_y=0.4, heading_rad=psi_b)
                self.assertTrue(
                    check_obb_intersection(box_a, box_b_shifted),
                    f"Overlapping boxes failed intersection at psi_a={psi_a:.4f}, psi_b={psi_b:.4f}"
                )

                # Distant boxes (distance 50m) MUST NOT intersect regardless of yaw combination
                box_b_far = OBB2D(center_x=50.0, center_y=50.0, heading_rad=psi_b)
                self.assertFalse(
                    check_obb_intersection(box_a, box_b_far),
                    f"Separated boxes falsely reported intersection at psi_a={psi_a:.4f}, psi_b={psi_b:.4f}"
                )

        self.assertEqual(total_tested, 21, "Expected 7 x 3 = 21 angle combinations tested")

    def test_sat_boundary_overlap_collinear_precision(self):
        """
        Boundary overlap precision:
        Collinear touching: longitudinal distance 5.630m (touching) vs 5.635m (separated).
        Also stress-test sub-millimeter transitions: 5.629m vs 5.631m.
        """
        box_ref = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)

        # Overlapping / Touching Cases
        # 1. 5.620m: 10mm penetration
        self.assertTrue(check_obb_intersection(box_ref, OBB2D(center_x=5.620, center_y=0.0, heading_rad=0.0)))
        # 2. 5.629m: 1mm penetration
        self.assertTrue(check_obb_intersection(box_ref, OBB2D(center_x=5.629, center_y=0.0, heading_rad=0.0)))
        # 3. 5.630m: exact boundary touching
        self.assertTrue(
            check_obb_intersection(box_ref, OBB2D(center_x=5.630, center_y=0.0, heading_rad=0.0)),
            "Longitudinal distance 5.630m must be detected as touching/intersecting"
        )

        # Separated Cases
        # 4. 5.631m: 1mm separated
        self.assertFalse(check_obb_intersection(box_ref, OBB2D(center_x=5.631, center_y=0.0, heading_rad=0.0)))
        # 5. 5.635m: 5mm separated
        self.assertFalse(
            check_obb_intersection(box_ref, OBB2D(center_x=5.635, center_y=0.0, heading_rad=0.0)),
            "Longitudinal distance 5.635m must be detected as separated"
        )
        # 6. 5.640m: 10mm separated
        self.assertFalse(check_obb_intersection(box_ref, OBB2D(center_x=5.640, center_y=0.0, heading_rad=0.0)))

    def test_sat_boundary_overlap_lateral_precision(self):
        """
        Lateral touching: lateral distance 2.000m (touching) vs 2.005m (separated).
        Also stress-test sub-millimeter transitions: 1.999m vs 2.001m.
        """
        box_ref = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)

        # Overlapping / Touching Cases
        # 1. 1.990m: 10mm penetration
        self.assertTrue(check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=1.990, heading_rad=0.0)))
        # 2. 1.999m: 1mm penetration
        self.assertTrue(check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=1.999, heading_rad=0.0)))
        # 3. 2.000m: exact lateral touching
        self.assertTrue(
            check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=2.000, heading_rad=0.0)),
            "Lateral distance 2.000m must be detected as touching/intersecting"
        )

        # Separated Cases
        # 4. 2.001m: 1mm separated
        self.assertFalse(check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=2.001, heading_rad=0.0)))
        # 5. 2.005m: 5mm separated
        self.assertFalse(
            check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=2.005, heading_rad=0.0)),
            "Lateral distance 2.005m must be detected as separated"
        )
        # 6. 2.010m: 10mm separated
        self.assertFalse(check_obb_intersection(box_ref, OBB2D(center_x=0.0, center_y=2.010, heading_rad=0.0)))

    def test_sat_t_bone_and_oblique_corner_penetration(self):
        """
        T-bone collision (90 deg yaw difference) and oblique corner-to-edge penetration.
        """
        # Car A along X axis: length [-2.815, 2.815], width [-1.0, 1.0]
        car_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)

        # 1. Perpendicular T-Bone (heading = pi/2)
        # Car B extends longitudinally along Y: [y_center - 2.815, y_center + 2.815]
        # Car A side edge is at y = 1.000.
        # Exact touching occurs when y_center - 2.815 = 1.000 => y_center = 3.815m.
        car_b_direct_hit = OBB2D(center_x=0.0, center_y=2.0, heading_rad=math.pi / 2.0)
        self.assertTrue(check_obb_intersection(car_a, car_b_direct_hit), "Direct center T-bone must collide")

        car_b_touching = OBB2D(center_x=0.0, center_y=3.815, heading_rad=math.pi / 2.0)
        self.assertTrue(check_obb_intersection(car_a, car_b_touching), "T-bone nose touching side pod must collide")

        car_b_separated = OBB2D(center_x=0.0, center_y=3.820, heading_rad=math.pi / 2.0)
        self.assertFalse(check_obb_intersection(car_a, car_b_separated), "T-bone 5mm clear must be separated")

        # 2. Oblique corner-to-edge penetration (yaw = pi/4 = 45 deg)
        # Car A front-right corner is at (2.815, 1.000)
        # Place Car B with center positioned so its edge intersects Car A's corner
        car_b_oblique_hit = OBB2D(center_x=3.5, center_y=1.5, heading_rad=math.pi / 4.0)
        self.assertTrue(check_obb_intersection(car_a, car_b_oblique_hit), "Oblique 45-deg corner clip must collide")

        # Back Car B away along diagonal so corners clear
        car_b_oblique_clear = OBB2D(center_x=6.0, center_y=6.0, heading_rad=math.pi / 4.0)
        self.assertFalse(check_obb_intersection(car_a, car_b_oblique_clear), "Oblique separated cars must not collide")


class TestTTCNegativeDivergenceChallenger(unittest.TestCase):
    """
    Task 2: TTC Negative Divergence Protection Stress.
    Leader pulling away (v_leader > v_follower or v_rel <= 0) across tight gaps:
    distance in [0.1, 0.5, 1.0, 5.0, 8.0, 9.9, 10.0, 10.1, 50.0] m.
    Assert compute_ttc returns inf and compute_overtake_and_avoidance returns (inf, 0.0, 0.0).
    """

    def test_negative_divergence_across_nine_tight_gaps(self):
        """Stress-test leader pulling away across all 9 specified gaps and multiple velocity pairs."""
        tight_gaps = [0.1, 0.5, 1.0, 5.0, 8.0, 9.9, 10.0, 10.1, 50.0]
        
        # Test 4 distinct speed regimes where v_leader >= v_follower (v_rel <= 0)
        velocity_regimes = [
            (50.0, 60.0),   # Standard pulling away (v_rel = -10 m/s)
            (20.0, 75.0),   # Rapid departure (v_rel = -55 m/s)
            (0.0, 35.0),    # Follower stationary, leader accelerating (v_rel = -35 m/s)
            (65.0, 65.0),   # Exact speed match zero relative velocity (v_rel = 0.0 m/s)
            (0.0, 0.0),     # Standstill both cars (v_rel = 0.0 m/s)
        ]

        for dist in tight_gaps:
            for v_f, v_l in velocity_regimes:
                # 1. Check compute_ttc
                ttc = compute_ttc(distance_m=dist, v_follower_ms=v_f, v_leader_ms=v_l)
                self.assertTrue(
                    math.isinf(ttc) and ttc > 0,
                    f"compute_ttc({dist}, {v_f}, {v_l}) must return +inf, got {ttc}"
                )

                # 2. Check compute_overtake_and_avoidance (and any alias)
                ttc_res, lat_shift, brake_demand = compute_overtake_and_avoidance(
                    distance_m=dist,
                    v_follower_ms=v_f,
                    v_leader_ms=v_l,
                    leader_d_m=0.0,
                    track_half_width_m=5.5,
                )

                self.assertTrue(
                    math.isinf(ttc_res),
                    f"Avoidance TTC must be inf when leader pulling away at d={dist}m, got {ttc_res}"
                )
                self.assertEqual(
                    lat_shift, 0.0,
                    f"Avoidance lateral shift must be exactly 0.0 (no swerve) at d={dist}m, got {lat_shift}"
                )
                self.assertEqual(
                    brake_demand, 0.0,
                    f"Avoidance brake demand must be exactly 0.0 (no braking) at d={dist}m, got {brake_demand}"
                )

    def test_negative_divergence_non_zero_leader_d(self):
        """Even with non-zero leader lateral position, pulling away must command ZERO swerve."""
        for leader_d in [-2.0, -1.0, 0.0, 1.5, 3.0]:
            ttc, lat_shift, brake = compute_overtake_and_avoidance(
                distance_m=2.0,
                v_follower_ms=45.0,
                v_leader_ms=55.0,
                leader_d_m=leader_d,
            )
            self.assertTrue(math.isinf(ttc))
            self.assertEqual(lat_shift, 0.0)
            self.assertEqual(brake, 0.0)


class TestTTCAvoidanceAndOvertakeChallenger(unittest.TestCase):
    """
    Task 3: TTC Avoidance & Overtake Maneuver Adversarial Verification.
    Follower approaching slower leader (v_follower > v_leader) when TTC < 1.2s.
    Verify lateral shift (+-2.8m) and braking trigger when boxed-in.
    """

    def test_closing_car_triggers_lateral_overtake_shift(self):
        """When closing rapidly with TTC < 1.2s, commands +-2.8m lateral overtake lane shift."""
        # Leader at 40 m/s, Follower at 60 m/s => v_rel = 20 m/s
        # Distance = 16.0 m => TTC = 0.8s (< 1.2s threshold)
        v_f = 60.0
        v_l = 40.0
        dist = 16.0
        expected_ttc = 0.8

        ttc, lat_shift, brake = compute_overtake_and_avoidance(
            distance_m=dist,
            v_follower_ms=v_f,
            v_leader_ms=v_l,
            leader_d_m=0.0,
            track_half_width_m=5.5,
        )

        self.assertAlmostEqual(ttc, expected_ttc, places=4)
        # Default side with room: candidate_pos = 0.0 + 2.8 = 2.8m (<= margin 4.3m)
        self.assertEqual(lat_shift, OVERTAKE_LATERAL_OFFSET_M)
        self.assertEqual(lat_shift, 2.8)
        # Because lateral shift is clear and TTC >= 0.55s, no emergency braking
        self.assertEqual(brake, 0.0)

    def test_closing_car_chooses_right_lane_when_left_blocked(self):
        """When leader is near the left edge, follower shifts right (-2.8m from leader)."""
        # Leader at d = 2.5m on track with half_width = 5.0m (margin = 3.8m)
        # candidate_pos = 2.5 + 2.8 = 5.3m (> 3.8m => BLOCKED)
        # candidate_neg = 2.5 - 2.8 = -0.3m (<= 3.8m => CLEAR)
        ttc, lat_shift, brake = compute_overtake_and_avoidance(
            distance_m=12.0,
            v_follower_ms=55.0,
            v_leader_ms=40.0,
            leader_d_m=2.5,
            track_half_width_m=5.0,
        )
        self.assertAlmostEqual(ttc, 0.8, places=4)
        self.assertAlmostEqual(lat_shift, -0.3, places=4)
        # Shift relative to leader is -2.8m
        self.assertAlmostEqual(lat_shift - 2.5, -2.8, places=4)

    def test_boxed_in_scenario_emergency_deceleration(self):
        """
        Stress-test boxed-in scenario: when both left and right lanes are blocked,
        assert emergency deceleration is commanded.
        """
        # Narrow track (e.g. Monaco street section): half_width = 2.5m => margin = 1.3m
        # Leader is centered at d = 0.0m.
        # candidate_pos = 0.0 + 2.8 = 2.8m (> 1.3m => BLOCKED)
        # candidate_neg = 0.0 - 2.8 = -2.8m (> 1.3m => BLOCKED)
        # Both lanes blocked! Car cannot shift lanes.
        # When closing with TTC < 0.55s, emergency deceleration must be commanded.

        # Test varying distances with v_rel = 20 m/s
        # 1. d = 6.0m => TTC = 0.30s (< 0.55s)
        ttc, shift, brake = compute_overtake_and_avoidance(
            distance_m=6.0,
            v_follower_ms=60.0,
            v_leader_ms=40.0,
            leader_d_m=0.0,
            track_half_width_m=2.5,
        )
        self.assertAlmostEqual(ttc, 0.30, places=4)
        self.assertGreater(brake, 0.0, "Boxed-in car with TTC < 0.55s must command emergency braking")
        expected_brake = (0.55 - 0.30) / 0.55
        self.assertAlmostEqual(brake, expected_brake, places=4)

        # 2. Critical proximity: d = 1.0m => TTC = 0.05s
        ttc_crit, shift_crit, brake_crit = compute_overtake_and_avoidance(
            distance_m=1.0,
            v_follower_ms=60.0,
            v_leader_ms=40.0,
            leader_d_m=0.0,
            track_half_width_m=2.5,
        )
        self.assertAlmostEqual(ttc_crit, 0.05, places=4)
        self.assertGreater(brake_crit, 0.90, "Critical proximity must command >90% braking")
        self.assertLessEqual(brake_crit, 1.0)


class TestSlipstreamDraftingTowConeChallenger(unittest.TestCase):
    """
    Task 4: Slipstream Drafting Tow Cone Adversarial Stress.
    Stress-test drag multiplier across 30 grid points:
    d in [0.0, 5.0, 20.0, 44.9, 45.1, 60.0] m
    dy in [0.0, 1.0, 2.4, 2.6, 5.0] m.
    Assert continuous decay within cone and exact 1.0 outside cone.
    """

    def test_slipstream_grid_30_points_matrix(self):
        """Evaluate the full 6 x 5 = 30 grid points matrix and assert exact aerodynamic boundaries."""
        test_distances = [0.0, 5.0, 20.0, 44.9, 45.1, 60.0]
        test_laterals = [0.0, 1.0, 2.4, 2.6, 5.0]

        evaluated = 0
        for d in test_distances:
            for dy in test_laterals:
                evaluated += 1
                mult = compute_slipstream_drag_multiplier(distance_m=d, lateral_diff_m=dy)

                # Boundary rule 1: d = 0.0m is at or inside leader bumper -> 1.0
                if d <= 0.5:
                    self.assertEqual(
                        mult, 1.0,
                        f"At d={d}m, dy={dy}m: expected neutral 1.0, got {mult}"
                    )
                # Boundary rule 2: d > 45.0m (45.1m, 60.0m) is outside longitudinal wake cone -> exactly 1.0
                elif d > 45.0:
                    self.assertEqual(
                        mult, 1.0,
                        f"At d={d}m, dy={dy}m: expected neutral 1.0 outside cone, got {mult}"
                    )
                # Boundary rule 3: dy > 2.5m (2.6m, 5.0m) is outside lateral wake cone -> exactly 1.0
                elif dy > 2.5:
                    self.assertEqual(
                        mult, 1.0,
                        f"At d={d}m, dy={dy}m: expected neutral 1.0 outside lateral cone, got {mult}"
                    )
                # Boundary rule 4: inside the cone (d in [5.0, 20.0, 44.9], dy in [0.0, 1.0, 2.4]) -> drag reduction
                else:
                    self.assertLess(
                        mult, 1.0,
                        f"At d={d}m, dy={dy}m: expected drag reduction (<1.0), got {mult}"
                    )
                    self.assertGreaterEqual(
                        mult, 0.82,
                        f"At d={d}m, dy={dy}m: drag multiplier cannot fall below maximum 18% reduction (0.82), got {mult}"
                    )

        self.assertEqual(evaluated, 30, "Expected exactly 30 matrix grid points evaluated")

    def test_slipstream_continuous_decay_and_monotonicity(self):
        """
        Assert continuous decay within cone and C0 boundary continuity:
        - Monotonic decay with distance
        - Monotonic decay with lateral offset
        - Continuous transition across 45.0m and 2.5m boundaries
        """
        # 1. Monotonicity along longitudinal centerline (dy = 0.0)
        distances = [1.0, 5.0, 10.0, 20.0, 30.0, 40.0, 44.9]
        multipliers_long = [compute_slipstream_drag_multiplier(d, 0.0) for d in distances]
        for i in range(len(multipliers_long) - 1):
            self.assertLess(
                multipliers_long[i], multipliers_long[i + 1],
                f"Drag multiplier must increase monotonically with distance: {multipliers_long[i]} vs {multipliers_long[i+1]}"
            )

        # 2. Monotonicity across lateral offset at fixed distance d = 15.0m
        laterals = [0.0, 0.5, 1.0, 1.5, 2.0, 2.4]
        multipliers_lat = [compute_slipstream_drag_multiplier(15.0, lat) for lat in laterals]
        for i in range(len(multipliers_lat) - 1):
            self.assertLess(
                multipliers_lat[i], multipliers_lat[i + 1],
                f"Drag multiplier must increase monotonically with lateral offset: {multipliers_lat[i]} vs {multipliers_lat[i+1]}"
            )

        # 3. Sub-millimeter boundary continuity at 45.0m
        mult_44_999 = compute_slipstream_drag_multiplier(44.999, 0.0)
        mult_45_001 = compute_slipstream_drag_multiplier(45.001, 0.0)
        self.assertEqual(mult_45_001, 1.0)
        self.assertAlmostEqual(mult_44_999, 1.0, delta=0.001, msg="Boundary jump at 45m must be < 1e-3")

        # 4. Sub-millimeter boundary continuity at lateral 2.5m
        mult_lat_2_499 = compute_slipstream_drag_multiplier(10.0, 2.499)
        mult_lat_2_501 = compute_slipstream_drag_multiplier(10.0, 2.501)
        self.assertEqual(mult_lat_2_501, 1.0)
        self.assertAlmostEqual(mult_lat_2_499, 1.0, delta=0.001, msg="Boundary jump at 2.5m must be < 1e-3")

    def test_slipstream_aeromodel_coupling(self):
        """
        Verify AeroModel drag force scales directly with slipstream_drag_scalar.
        """
        aero = Aerodynamics(AeroConfig())
        speed = 80.0  # m/s (~288 km/h)

        # Baseline drag without slipstream
        aero.set_slipstream_factors(drag_scalar=1.0)
        _, f_drag_base, _, _ = aero.compute(speed_ms=speed, drs_active=False)

        # In slipstream tow with 15% reduction (scalar = 0.85)
        aero.set_slipstream_factors(drag_scalar=0.85)
        _, f_drag_tow, _, _ = aero.compute(speed_ms=speed, drs_active=False)

        expected_drag = f_drag_base * 0.85
        self.assertAlmostEqual(
            f_drag_tow, expected_drag, places=4,
            msg=f"Aero drag force must scale by exactly 0.85: base={f_drag_base:.2f}N, tow={f_drag_tow:.2f}N"
        )

    def test_slipstream_dense_5000_points_heatmap_bounds(self):
        """
        Dense grid stress test: 100 distance steps x 50 lateral steps = 5,000 points.
        Every point must be bounded in [0.82, 1.00] without NaN or Inf.
        """
        distances = np.linspace(0.0, 60.0, 100)
        laterals = np.linspace(0.0, 5.0, 50)

        for d in distances:
            for lat in laterals:
                val = compute_slipstream_drag_multiplier(distance_m=float(d), lateral_diff_m=float(lat))
                self.assertFalse(math.isnan(val), f"NaN at d={d}, lat={lat}")
                self.assertFalse(math.isinf(val), f"Inf at d={d}, lat={lat}")
                self.assertGreaterEqual(val, 0.82 - 1e-9, f"Below 0.82 floor at d={d}, lat={lat}: {val}")
                self.assertLessEqual(val, 1.0 + 1e-9, f"Above 1.0 ceiling at d={d}, lat={lat}: {val}")

                # Verify outside cone condition
                if d <= 0.5 or d > 45.0 or lat > 2.5:
                    self.assertEqual(val, 1.0, f"Expected 1.0 outside cone at d={d}, lat={lat}, got {val}")
                else:
                    self.assertLess(val, 1.0, f"Expected < 1.0 inside cone at d={d}, lat={lat}, got {val}")


class TestAdversarialSingularitiesChallenger(unittest.TestCase):
    """
    Cross-cutting edge singularities and extreme parameter fuzzing.
    """

    def test_sat_monte_carlo_bounding_circle_invariance(self):
        """
        Monte Carlo test: 1,000 random orientations.
        - Circumscribed circle radius R = sqrt((5.63/2)^2 + (2.0/2)^2) = 2.9876m.
        - Two boxes separated by center-to-center distance > 2*R = 5.9752m can NEVER intersect.
        """
        rng = np.random.RandomState(42)
        r_circum = math.hypot(5.63 / 2.0, 2.0 / 2.0)
        min_safe_dist = 2.0 * r_circum + 0.05  # > 6.025m

        for _ in range(1000):
            psi_a = rng.uniform(-np.pi, np.pi)
            psi_b = rng.uniform(-np.pi, np.pi)
            angle = rng.uniform(0, 2 * np.pi)
            dist = rng.uniform(min_safe_dist, 50.0)

            cx_b = dist * math.cos(angle)
            cy_b = dist * math.sin(angle)

            box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=psi_a)
            box_b = OBB2D(center_x=cx_b, center_y=cy_b, heading_rad=psi_b)

            intersects = check_obb_intersection(box_a, box_b)
            self.assertFalse(
                intersects,
                f"False positive collision at distance {dist:.3f}m > 2*R with yaw_a={psi_a:.2f}, yaw_b={psi_b:.2f}"
            )

    def test_negative_divergence_singularities_and_reverse_motion(self):
        """
        Adversarial cases:
        1. Infinitesimal divergence: v_rel = -1e-12 m/s
        2. Reverse movement: v_follower = -5 m/s, v_leader = 0 m/s
        3. Negative distance input (clamped to 0.0)
        """
        # Infinitesimal divergence
        ttc = compute_ttc(distance_m=10.0, v_follower_ms=50.0, v_leader_ms=50.0 + 1e-12)
        self.assertTrue(math.isinf(ttc))
        ttc_res, lat_shift, brake = compute_overtake_and_avoidance(
            distance_m=10.0, v_follower_ms=50.0, v_leader_ms=50.0 + 1e-12
        )
        self.assertTrue(math.isinf(ttc_res))
        self.assertEqual(lat_shift, 0.0)
        self.assertEqual(brake, 0.0)

        # Reverse velocity
        ttc_rev = compute_ttc(distance_m=10.0, v_follower_ms=-5.0, v_leader_ms=0.0)
        self.assertTrue(math.isinf(ttc_rev))

        # Negative distance
        ttc_neg_d = compute_ttc(distance_m=-10.0, v_follower_ms=60.0, v_leader_ms=40.0)
        self.assertEqual(ttc_neg_d, 0.0)

    def test_ttc_exact_arming_boundary_and_high_speed_closure(self):
        """
        Verifies exact arming threshold at TTC = 1.2s:
        - TTC = 1.2001s: disarmed (no shift)
        - TTC = 1.1999s: armed (commands +-2.8m shift)
        - High-speed closure: 95 m/s (~342 km/h) into 40 m/s
        """
        # 1. Arming threshold at distance > 10m
        # v_rel = 10.0 m/s. Threshold distance for TTC=1.2s is d = 12.0m.
        # At d = 12.01m, TTC = 1.201s (> 1.2s and d > 10m) => disarmed
        ttc_disarmed, shift_disarmed, brake_disarmed = compute_overtake_and_avoidance(
            distance_m=12.01, v_follower_ms=50.0, v_leader_ms=40.0, leader_d_m=0.0
        )
        self.assertAlmostEqual(ttc_disarmed, 1.201, places=3)
        self.assertEqual(shift_disarmed, 0.0)
        self.assertEqual(brake_disarmed, 0.0)

        # At d = 11.99m, TTC = 1.199s (< 1.2s) => armed!
        ttc_armed, shift_armed, brake_armed = compute_overtake_and_avoidance(
            distance_m=11.99, v_follower_ms=50.0, v_leader_ms=40.0, leader_d_m=0.0
        )
        self.assertAlmostEqual(ttc_armed, 1.199, places=3)
        self.assertEqual(shift_armed, 2.8)
        self.assertEqual(brake_armed, 0.0)

        # 2. Extreme closure: 95 m/s vs 40 m/s (v_rel = 55 m/s) at 40m distance => TTC = 0.727s
        ttc_fast, shift_fast, brake_fast = compute_overtake_and_avoidance(
            distance_m=40.0, v_follower_ms=95.0, v_leader_ms=40.0, leader_d_m=0.0
        )
        self.assertAlmostEqual(ttc_fast, 40.0 / 55.0, places=4)
        self.assertEqual(shift_fast, 2.8)
        self.assertEqual(brake_fast, 0.0)


if __name__ == "__main__":
    unittest.main()

