"""
tests/test_tier2_boundaries.py
==============================
Tier 2: Boundary Value Analysis & Corner Case Stress Test Suite.

Verifies extreme values, boundary conditions, edge singularites, and safety limiters:
1. Zero input stability & idle creep prevention
2. Input clamping (negative, excessive, out-of-range throttle/brake/steer)
3. Full simultaneous throttle and brake (left-foot burnout behavior)
4. Extreme steering lock boundaries (+/- 180 deg) & speed attenuation
5. Reverse gear lock inhibitor at forward speeds
6. 8th gear top boundary limit
7. Downshift over-rev safety protection
8. Engine RPM redline rev limiter (15,000 RPM)
9. Engine idle RPM lower floor & anti-stall clutch (4,000 RPM)
10. Brake bias clamping ([0.52, 0.64])
11. Zero-velocity kinematic singularity protection (no ZeroDivisionError)
12. Extreme 90-degree lateral tyre slip angle (finite sliding friction, no NaN)
13. Track distance spline boundary wraparound (s = L -> 0 continuity)
14. Off-track grass/gravel friction and lap invalidation
15. DRS activation/deactivation exact boundary points
16. Time-to-Collision (TTC) negative divergence handling (lead car accelerating away)
"""

import math
import unittest
import numpy as np

from f1sim.physics.vehicle import Vehicle
from tests.benchmark_physics_ai import (
    VEHICLE_MASS,
    WHEELBASE,
    CG_TO_FRONT,
    CG_TO_REAR,
    TYRE_RADIUS,
    IDLE_RPM,
    REDLINE_RPM,
    BASE_MU,
    GEAR_RATIOS,
    FINAL_DRIVE,
)


class TestTier2Boundaries(unittest.TestCase):
    """Boundary value analysis and corner cases."""

    def setUp(self):
        self.car = Vehicle()

    # 1. Zero Input Stability
    def test_zero_throttle_brake_at_rest_no_creep(self):
        """Car at standstill with zero throttle/brake remains at exactly 0.0 speed."""
        self.car.vx = 0.0
        self.car.step(throttle=0.0, brake=0.0, steer=0.0, dt=0.01)
        self.assertEqual(self.car.vx, 0.0)
        self.assertEqual(self.car.speed_kmh, 0.0)

    # 2. Input Clamping & Bounds
    def test_negative_throttle_clamped_to_zero(self):
        """Negative throttle input (-1.0) clamps to 0.0."""
        self.car.step(throttle=-1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertEqual(self.car.throttle, 0.0)

    def test_excessive_throttle_clamped_to_one(self):
        """Excessive throttle input (5.0) clamps to 1.0."""
        self.car.step(throttle=5.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertEqual(self.car.throttle, 1.0)

    def test_negative_brake_clamped_to_zero(self):
        """Negative brake input (-0.5) clamps to 0.0."""
        self.car.step(throttle=0.0, brake=-0.5, steer=0.0, dt=0.001)
        self.assertEqual(self.car.brake, 0.0)

    def test_excessive_brake_clamped_to_one(self):
        """Excessive brake input (10.0) clamps to 1.0."""
        self.car.step(throttle=0.0, brake=10.0, steer=0.0, dt=0.001)
        self.assertEqual(self.car.brake, 1.0)

    def test_excessive_steering_clamped_to_half_pi(self):
        """Steering input beyond mechanical lock (> pi/2 rad) clamps to limit."""
        self.car.step(throttle=0.0, brake=0.0, steer=3.5, dt=0.001)
        self.assertAlmostEqual(self.car.steer, self.car.cfg.max_steer_rad, places=4)

    # 3. Simultaneous Maximum Inputs (Burnout)
    def test_simultaneous_full_throttle_and_brake(self):
        """Simultaneous 100% throttle and 100% brake does not cause instability or numerical overflow."""
        self.car.vx = 0.0
        self.car.gear = 1
        for _ in range(500):
            self.car.step(throttle=1.0, brake=1.0, steer=0.0, dt=0.001)
        self.assertFalse(math.isnan(self.car.vx))
        self.assertFalse(math.isinf(self.car.vx))
        # Carbon brakes overpower 1st gear launch, speed remains low or near standstill
        self.assertLess(self.car.speed_kmh, 30.0)

    # 4. Steering Lock & High-Speed Clamping
    def test_steering_at_extreme_high_speed(self):
        """At 350 km/h, steering input does not produce infinite lateral acceleration."""
        self.car.vx = 350.0 / 3.6
        self.car.gear = 8
        self.car.step(throttle=0.0, brake=0.0, steer=0.10, dt=0.001)
        self.assertFalse(math.isnan(self.car.lateral_g))
        self.assertLess(abs(self.car.lateral_g), 8.0)

    # 5. Reverse Gear Lockout
    def test_reverse_gear_inhibited_at_forward_speed(self):
        """Attempting to shift into Reverse at forward speed > 1 km/h is rejected."""
        self.car.vx = 50.0 / 3.6  # 50 km/h
        self.car.gear = 0
        shifted = self.car.shift_down()
        self.assertFalse(shifted)
        self.assertEqual(self.car.gear, 0)

    def test_reverse_gear_allowed_at_standstill(self):
        """Reverse gear permitted when vehicle is stationary (v <= 1.0 km/h)."""
        self.car.vx = 0.0
        self.car.gear = 0
        shifted = self.car.shift_down()
        self.assertTrue(shifted)
        self.assertEqual(self.car.gear, -1)

    # 6. Top Gear Boundary Limit
    def test_upshift_past_8th_gear_rejected(self):
        """Attempting to shift up past 8th gear remains in 8th gear."""
        self.car.gear = 8
        self.car.shift_up()
        self.assertEqual(self.car.gear, 8)

    # 7. Downshift Over-Rev Protection
    def test_downshift_over_rev_rejected_at_300kmh(self):
        """Downshifting at 300 km/h rejected to prevent engine destruction."""
        self.car.vx = 300.0 / 3.6
        self.car.gear = 6
        shifted = self.car.shift_down()
        self.assertFalse(shifted)
        self.assertEqual(self.car.gear, 6)

    # 8. Engine RPM Redline Floor and Ceiling
    def test_rpm_redline_ceiling_limit(self):
        """Engine RPM cannot exceed 15,000 RPM redline."""
        self.car.vx = 110.0  # Excessive hypothetical speed
        self.car.gear = 8
        self.car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertLessEqual(self.car.rpm, REDLINE_RPM)

    def test_rpm_idle_floor_limit(self):
        """Engine RPM cannot drop below 4,000 RPM idle when engine is running."""
        self.car.vx = 0.0
        self.car.gear = 0  # Neutral
        self.car.step(throttle=0.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertGreaterEqual(self.car.rpm, IDLE_RPM)

    # 9. Brake Bias Clamping
    def test_brake_bias_clamping_low(self):
        """Requesting front brake bias below 52% clamps to 0.52."""
        self.car.brake_bias = 0.45
        self.assertAlmostEqual(self.car.brake_bias, 0.52, places=3)

    def test_brake_bias_clamping_high(self):
        """Requesting front brake bias above 64% clamps to 0.64."""
        self.car.brake_bias = 0.75
        self.assertAlmostEqual(self.car.brake_bias, 0.64, places=3)

    # 10. Zero-Velocity Kinematic Singularity Protection
    def test_zero_velocity_no_zero_division(self):
        """At zero velocity, tyre slip formulas do not raise ZeroDivisionError."""
        self.car.vx = 0.0
        self.car.vy = 0.0
        self.car.yaw_rate = 0.0
        try:
            self.car.step(throttle=0.0, brake=0.0, steer=0.0, dt=0.001)
            no_exception = True
        except ZeroDivisionError:
            no_exception = False
        self.assertTrue(no_exception)
        self.assertFalse(math.isnan(self.car.slip_angle_front))
        self.assertFalse(math.isnan(self.car.slip_angle_rear))

    # 11. Extreme 90-Degree Slip Angle
    def test_extreme_90_degree_lateral_slip_angle(self):
        """Pacejka formula with 90-degree slip (math.pi/2) outputs finite sliding force."""
        fz = 4000.0
        alpha_extreme = math.pi / 2.0  # 90 degrees
        force = self.car.pacejka_lateral_force(alpha_extreme, fz)
        self.assertFalse(math.isnan(force))
        self.assertFalse(math.isinf(force))
        self.assertGreater(force, 0.0)

    # 12. Track Spline Boundary Wraparound
    def test_track_distance_wraparound(self):
        """Track distance s wraps smoothly across finish line without index errors."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        s_current = 5792.95
        ds = 0.20
        s_next = (s_current + ds) % track.length
        self.assertAlmostEqual(s_next, 0.15, places=3)
        self.assertGreaterEqual(s_next, 0.0)
        self.assertLess(s_next, track.length)
        pos_curr, _, _ = track.get_spline_point(s_current)
        pos_next, _, _ = track.get_spline_point(s_next)
        step_dist = float(np.linalg.norm(pos_next - pos_curr))
        self.assertLess(step_dist, 1.0)

    # 13. Off-Track Verge Transition
    def test_off_track_verge_transition(self):
        """Car exceeding track half-width encounters grass/gravel friction drop."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        pos_on_track = 3.2
        pos_off_track = 6.8

        mu_at_pos1 = track.get_surface_mu(s=100.0, d=pos_on_track)
        mu_at_pos2 = track.get_surface_mu(s=100.0, d=pos_off_track)

        self.assertEqual(mu_at_pos1, 1.75)
        self.assertEqual(mu_at_pos2, 0.40)
        self.assertLess(mu_at_pos2, mu_at_pos1)

    # 14. DRS Zone Boundary Precision
    def test_drs_zone_exact_boundary_detection(self):
        """DRS is only available within [s_act, s_deact]."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()

        self.assertFalse(track.is_in_drs_zone(114.99))
        self.assertTrue(track.is_in_drs_zone(115.00))
        self.assertTrue(track.is_in_drs_zone(500.00))
        self.assertTrue(track.is_in_drs_zone(980.00))
        self.assertFalse(track.is_in_drs_zone(980.01))

    # 15. Time-to-Collision (TTC) Negative Divergence
    def test_ttc_negative_divergence_when_leader_pulls_away(self):
        """When leader is traveling faster than follower, TTC is infinity, no collision risk."""
        from f1sim.ai.collision import compute_ttc, compute_overtake_and_avoidance

        v_follower = 50.0  # m/s
        v_leader = 60.0    # m/s (pulling away)
        distance = 25.0    # m

        ttc = compute_ttc(distance, v_follower, v_leader)
        self.assertEqual(ttc, float("inf"))

        # Verify compute_overtake_and_avoidance does NOT trigger even at bumper proximity
        for test_dist in [25.0, 10.0, 5.0, 1.0]:
            ttc_res, lat_shift, brake_demand = compute_overtake_and_avoidance(
                distance_m=test_dist,
                v_follower_ms=v_follower,
                v_leader_ms=v_leader,
                leader_d_m=0.0,
            )
            self.assertEqual(ttc_res, float("inf"))
            self.assertEqual(lat_shift, 0.0)
            self.assertEqual(brake_demand, 0.0)


if __name__ == "__main__":
    unittest.main()
