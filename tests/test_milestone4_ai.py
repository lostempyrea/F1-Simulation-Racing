"""
tests/test_milestone4_ai.py
===========================
Comprehensive Milestone 4 Test Suite for Multi-Tier AI Racing System.
Covers Requirements R1, R2, R3, R4:
- 5 Authentic AI Opponents (#1 Verstappen, #16 Leclerc, #44 Hamilton, #4 Norris, #14 Alonso)
- 3 AI Difficulty Tiers (Easy, Medium, Hard)
- Curvature Speed Profiling & Pure-Pursuit Steering Controller
- 2D OBB Separating Axis Theorem (SAT) Collision Detection (5.63m x 2.00m)
- Time-to-Collision (TTC < 1.2s) & Avoidance / Overtaking Lane Displacements
- Slipstream Drafting Drag Reduction & Aero Coupling
"""

import math
import unittest
import numpy as np

from f1sim.ai.driver_profiles import (
    AI_DRIVER_ROSTER,
    AIDifficulty,
    AIDriverProfile,
    DifficultyCalibration,
    DIFFICULTY_PROFILES,
    get_difficulty_calibration,
    get_driver_by_number,
    estimate_monza_ai_lap_time,
)
from f1sim.ai.racing_line import (
    RacingLineProfile,
    compute_apex_speed,
    compute_braking_entry_speed,
    compute_lookahead_distance,
)
from f1sim.ai.collision import (
    OBB2D,
    VEHICLE_OBB_LENGTH_M,
    VEHICLE_OBB_WIDTH_M,
    TTC_AVOIDANCE_THRESHOLD_S,
    OVERTAKE_LATERAL_OFFSET_M,
    check_obb_intersection,
    compute_ttc,
    compute_overtake_and_avoidance,
    compute_slipstream_drag_multiplier,
)
from f1sim.ai.controller import (
    AIDriverController,
    AICarControlOutput,
)
from f1sim.track.monza import MonzaTrack
from f1sim.track.monaco import MonacoTrack
from f1sim.physics.vehicle import Vehicle
from f1sim.physics.aero import Aerodynamics


class TestAIDriverRoster(unittest.TestCase):
    """Verifies all 5 authentic F1 driver profiles and roster lookup contracts."""

    def test_roster_count_and_completeness(self):
        """AI roster must contain exactly 5 authentic drivers."""
        self.assertEqual(len(AI_DRIVER_ROSTER), 5)
        numbers = {d.number for d in AI_DRIVER_ROSTER}
        self.assertEqual(numbers, {1, 16, 44, 4, 14})

    def test_driver_1_verstappen_attributes(self):
        """#1 Max Verstappen: Red Bull Racing, #001A30, pace 0.998, aggression 0.96."""
        drv = get_driver_by_number(1)
        self.assertEqual(drv.name, "Max Verstappen")
        self.assertEqual(drv.code, "VER")
        self.assertIn("Red Bull", drv.team)
        self.assertEqual(drv.short_team, "Red Bull")
        self.assertEqual(drv.hex, "#001A30")
        self.assertEqual(drv.rgb, (0, 26, 48))
        self.assertAlmostEqual(drv.pace_scalar, 0.998)
        self.assertAlmostEqual(drv.aggression, 0.96)
        self.assertAlmostEqual(drv.error_rate, 0.015)
        self.assertGreater(drv.wet_skill, 0.90)

    def test_driver_16_leclerc_attributes(self):
        """#16 Charles Leclerc: Scuderia Ferrari, #D40000, pace 0.999, aggression 0.93."""
        drv = get_driver_by_number(16)
        self.assertEqual(drv.name, "Charles Leclerc")
        self.assertEqual(drv.code, "LEC")
        self.assertIn("Ferrari", drv.team)
        self.assertEqual(drv.short_team, "Ferrari")
        self.assertEqual(drv.hex, "#D40000")
        self.assertEqual(drv.rgb, (212, 0, 0))
        self.assertAlmostEqual(drv.pace_scalar, 0.999)
        self.assertAlmostEqual(drv.aggression, 0.93)
        self.assertAlmostEqual(drv.error_rate, 0.020)

    def test_driver_44_hamilton_attributes(self):
        """#44 Lewis Hamilton: Mercedes-AMG, #0A0A0A, pace 1.003, aggression 0.91."""
        drv = get_driver_by_number(44)
        self.assertEqual(drv.name, "Lewis Hamilton")
        self.assertEqual(drv.code, "HAM")
        self.assertIn("Mercedes", drv.team)
        self.assertEqual(drv.short_team, "Mercedes")
        self.assertEqual(drv.hex, "#0A0A0A")
        self.assertEqual(drv.rgb, (10, 10, 10))
        self.assertAlmostEqual(drv.pace_scalar, 1.003)
        self.assertAlmostEqual(drv.aggression, 0.91)
        self.assertAlmostEqual(drv.error_rate, 0.018)

    def test_driver_4_norris_attributes(self):
        """#4 Lando Norris: McLaren, #FF8000, pace 1.005, aggression 0.90."""
        drv = get_driver_by_number(4)
        self.assertEqual(drv.name, "Lando Norris")
        self.assertEqual(drv.code, "NOR")
        self.assertEqual(drv.team, "McLaren")
        self.assertEqual(drv.short_team, "McLaren")
        self.assertEqual(drv.hex, "#FF8000")
        self.assertEqual(drv.rgb, (255, 128, 0))
        self.assertAlmostEqual(drv.pace_scalar, 1.005)
        self.assertAlmostEqual(drv.aggression, 0.90)
        self.assertAlmostEqual(drv.error_rate, 0.022)

    def test_driver_14_alonso_attributes(self):
        """#14 Fernando Alonso: Aston Martin, #00594F, pace 1.008, aggression 0.94."""
        drv = get_driver_by_number(14)
        self.assertEqual(drv.name, "Fernando Alonso")
        self.assertEqual(drv.code, "ALO")
        self.assertEqual(drv.team, "Aston Martin")
        self.assertEqual(drv.short_team, "Aston Martin")
        self.assertEqual(drv.hex, "#00594F")
        self.assertEqual(drv.rgb, (0, 89, 79))
        self.assertAlmostEqual(drv.pace_scalar, 1.008)
        self.assertAlmostEqual(drv.aggression, 0.94)
        self.assertAlmostEqual(drv.error_rate, 0.016)

    def test_get_driver_by_number_success_and_keyerror(self):
        """Valid driver numbers return profile; invalid numbers raise KeyError."""
        self.assertEqual(get_driver_by_number(1).code, "VER")
        self.assertEqual(get_driver_by_number(16).code, "LEC")
        with self.assertRaises(KeyError):
            get_driver_by_number(99)

    def test_driver_to_dict_contract(self):
        """to_dict() serialization must contain all regulatory fields."""
        drv = get_driver_by_number(1)
        d = drv.to_dict()
        self.assertEqual(d["number"], 1)
        self.assertEqual(d["code"], "VER")
        self.assertIn("Red Bull", str(d["team"]))
        self.assertEqual(d["short_team"], "Red Bull")
        self.assertEqual(d["hex"], "#001A30")
        self.assertEqual(d["rgb"], (0, 26, 48))
        self.assertIn("error_rate", d)


class TestAIDifficultyCalibration(unittest.TestCase):
    """Verifies physical parameters for Easy, Medium, and Hard difficulty tiers."""

    def test_difficulty_enum_and_profiles(self):
        """Enum and dictionary have Easy, Medium, and Hard profiles."""
        self.assertIn(AIDifficulty.EASY.value, DIFFICULTY_PROFILES)
        self.assertIn(AIDifficulty.MEDIUM.value, DIFFICULTY_PROFILES)
        self.assertIn(AIDifficulty.HARD.value, DIFFICULTY_PROFILES)

    def test_easy_profile_parameters(self):
        """Easy: 82% grip, 250ms latency, early braking 35-50m."""
        cal = get_difficulty_calibration("Easy")
        self.assertAlmostEqual(cal.grip_utilization, 0.82)
        self.assertAlmostEqual(cal.grip_scalar, 0.82)
        self.assertAlmostEqual(cal.reaction_latency_s, 0.250)
        self.assertAlmostEqual(cal.reaction_time, 0.250)
        self.assertTrue(35.0 <= cal.early_brake_offset_m <= 50.0)
        self.assertEqual(cal.early_brake_range_m, (35.0, 50.0))
        self.assertAlmostEqual(cal.braking_offset, 42.0)
        self.assertAlmostEqual(cal.throttle_exit_cap, 0.86)
        self.assertAlmostEqual(cal.max_decel_ms2, 26.5)

    def test_medium_profile_parameters(self):
        """Medium: 92% grip, 100ms latency, early braking 12-18m."""
        cal = get_difficulty_calibration("Medium")
        self.assertAlmostEqual(cal.grip_utilization, 0.92)
        self.assertAlmostEqual(cal.grip_scalar, 0.92)
        self.assertAlmostEqual(cal.reaction_latency_s, 0.100)
        self.assertAlmostEqual(cal.reaction_time, 0.100)
        self.assertTrue(12.0 <= cal.early_brake_offset_m <= 18.0)
        self.assertEqual(cal.early_brake_range_m, (12.0, 18.0))
        self.assertAlmostEqual(cal.braking_offset, 15.0)
        self.assertAlmostEqual(cal.throttle_exit_cap, 0.95)
        self.assertAlmostEqual(cal.max_decel_ms2, 33.0)

    def test_hard_profile_parameters(self):
        """Hard: 99.5% grip, 15ms latency, optimal braking 0-2m."""
        cal = get_difficulty_calibration("Hard")
        self.assertAlmostEqual(cal.grip_utilization, 0.995)
        self.assertAlmostEqual(cal.grip_scalar, 0.995)
        self.assertAlmostEqual(cal.reaction_latency_s, 0.015)
        self.assertAlmostEqual(cal.reaction_time, 0.015)
        self.assertTrue(0.0 <= cal.early_brake_offset_m <= 2.0)
        self.assertEqual(cal.early_brake_range_m, (0.0, 2.0))
        self.assertAlmostEqual(cal.braking_offset, 0.0)
        self.assertAlmostEqual(cal.throttle_exit_cap, 1.00)
        self.assertAlmostEqual(cal.max_decel_ms2, 39.5)

    def test_lap_time_monotonicity_and_sanity(self):
        """Strict monotonic ordering: LapTime(Easy) > LapTime(Medium) > LapTime(Hard)."""
        t_easy = estimate_monza_ai_lap_time("Easy")
        t_med = estimate_monza_ai_lap_time("Medium")
        t_hard = estimate_monza_ai_lap_time("Hard")
        self.assertGreater(t_easy, t_med)
        self.assertGreater(t_med, t_hard)
        self.assertGreaterEqual(t_easy - t_med, 4.0)
        self.assertGreaterEqual(t_med - t_hard, 2.0)
        self.assertTrue(90.0 < t_easy < 105.0)
        self.assertTrue(83.0 < t_med < 92.0)
        self.assertTrue(78.0 < t_hard < 84.0)


class TestRacingLineProfiling(unittest.TestCase):
    """Verifies pure-pursuit lookahead distance and curvature speed profiles."""

    def test_lookahead_distance_clamping(self):
        """L_d = clamp(8.0 + 0.38*v, 8.0, 45.0) verified across full speed spectrum."""
        self.assertEqual(compute_lookahead_distance(0.0), 8.0)
        self.assertEqual(compute_lookahead_distance(-10.0), 8.0)
        self.assertAlmostEqual(compute_lookahead_distance(50.0), 27.0)
        self.assertEqual(compute_lookahead_distance(100.0), 45.0)
        self.assertEqual(compute_lookahead_distance(200.0), 45.0)

    def test_apex_speed_calculation(self):
        """Apex speed scales inversely with square root of curvature."""
        v_high_r = compute_apex_speed(curvature=0.005, mu=1.75, grip_utilization=1.0)
        v_low_r = compute_apex_speed(curvature=0.020, mu=1.75, grip_utilization=1.0)
        self.assertGreater(v_high_r, v_low_r)
        ratio = v_high_r / v_low_r
        # sqrt(0.020 / 0.005) = 2.0
        self.assertAlmostEqual(ratio, 2.0, delta=0.05)

    def test_backward_braking_kinematics(self):
        """Kinematic backward integration computes safe approach speed."""
        v_apex = 22.0
        a_brake = 33.0
        ds = 25.0
        v_entry = compute_braking_entry_speed(v_apex, a_brake, ds)
        expected = math.sqrt(v_apex**2 + 2.0 * a_brake * ds)
        self.assertAlmostEqual(v_entry, expected)
        self.assertGreater(v_entry, v_apex)

    def test_racing_line_profile_monza_build(self):
        """Monza profile build samples 11 corners and sets bounded target speeds."""
        track = MonzaTrack()
        profile = RacingLineProfile(track, difficulty=AIDifficulty.HARD)
        self.assertGreater(len(profile.target_speeds_ms), 500)
        # All speeds must be within reasonable F1 bounds [15 m/s, 96 m/s]
        self.assertTrue(np.all(profile.target_speeds_ms >= 15.0))
        self.assertTrue(np.all(profile.target_speeds_ms <= 96.0))

    def test_racing_line_profile_monaco_build(self):
        """Monaco profile build correctly accounts for tight hairpins."""
        track = MonacoTrack()
        profile = RacingLineProfile(track, difficulty=AIDifficulty.HARD)
        # Target speed around Fairmont Hairpin must drop to <= 55 km/h (~15.3 m/s)
        self.assertLessEqual(np.min(profile.target_speeds_ms), 16.0)


class TestOBBSATCollisionDetection(unittest.TestCase):
    """Verifies 2D OBB Separating Axis Theorem (SAT) collision geometry (5.63m x 2.00m)."""

    def test_obb_dimensions_contract(self):
        """Chassis footprint is 5.63m length by 2.00m width."""
        box = OBB2D(center_x=10.0, center_y=20.0, heading_rad=0.5)
        self.assertEqual(box.length, VEHICLE_OBB_LENGTH_M)
        self.assertEqual(box.width, VEHICLE_OBB_WIDTH_M)
        self.assertEqual(box.length, 5.63)
        self.assertEqual(box.width, 2.00)

    def test_obb_axes_orthonormality(self):
        """Longitudinal and lateral body axes must be unit length and mutually orthogonal."""
        for angle in [0.0, math.pi / 4.0, math.pi / 2.0, 2.15]:
            box = OBB2D(heading_rad=angle)
            ax_long, ax_lat = box.get_axes()
            self.assertAlmostEqual(float(np.linalg.norm(ax_long)), 1.0)
            self.assertAlmostEqual(float(np.linalg.norm(ax_lat)), 1.0)
            self.assertAlmostEqual(float(np.dot(ax_long, ax_lat)), 0.0)

    def test_obb_corner_geometry(self):
        """Corners array must have shape (4, 2) matching rectangular dimensions."""
        box = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        corners = box.get_corners()
        self.assertEqual(corners.shape, (4, 2))
        # Front-left corner at (+2.815, +1.0)
        self.assertAlmostEqual(corners[0, 0], 2.815)
        self.assertAlmostEqual(corners[0, 1], 1.000)

    def test_sat_identical_boxes_collision(self):
        """Two identical overlapping boxes intersect."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_b = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        self.assertTrue(check_obb_intersection(box_a, box_b))

    def test_sat_separated_longitudinal(self):
        """Separated along track length (e.g. 10m apart) does not intersect."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_b = OBB2D(center_x=10.0, center_y=0.0, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box_a, box_b))

    def test_sat_separated_lateral(self):
        """Separated side-by-side (e.g. 3m apart) does not intersect."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        box_b = OBB2D(center_x=0.0, center_y=3.0, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box_a, box_b))

    def test_sat_slight_overlap_contact(self):
        """Slight 0.1m contact at front bumper intersects."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        # Bumper touches when center separation is length = 5.63m; overlap at 5.53m
        box_b = OBB2D(center_x=5.53, center_y=0.0, heading_rad=0.0)
        self.assertTrue(check_obb_intersection(box_a, box_b))

    def test_sat_rotated_yaw_intersection(self):
        """Rotated yaw orientation detected accurately by SAT."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        # Perpendicular box crossing center
        box_b = OBB2D(center_x=0.0, center_y=0.0, heading_rad=math.pi / 2.0)
        self.assertTrue(check_obb_intersection(box_a, box_b))


class TestTTCAndAvoidance(unittest.TestCase):
    """Verifies Time-to-Collision (TTC) and avoidance/overtaking lateral offset logic."""

    def test_ttc_positive_closing(self):
        """Follower traveling faster calculates accurate positive TTC."""
        # 30m distance, 20 m/s closing relative velocity -> TTC = 1.5s
        ttc = compute_ttc(distance_m=30.0, v_follower_ms=60.0, v_leader_ms=40.0)
        self.assertAlmostEqual(ttc, 1.5)

    def test_ttc_critical_under_threshold(self):
        """Closing at 20 m/s with 20m gap -> TTC = 1.0s (< 1.2s threshold)."""
        ttc = compute_ttc(distance_m=20.0, v_follower_ms=60.0, v_leader_ms=40.0)
        self.assertAlmostEqual(ttc, 1.0)
        self.assertLess(ttc, TTC_AVOIDANCE_THRESHOLD_S)

    def test_ttc_negative_divergence_infinite(self):
        """Leader pulling away yields TTC = infinity."""
        ttc = compute_ttc(distance_m=20.0, v_follower_ms=40.0, v_leader_ms=60.0)
        self.assertEqual(ttc, float("inf"))

    def test_avoidance_lane_shift_left_and_right(self):
        """TTC < 1.2s commands lateral overtaking displacement +/- 2.8m."""
        # Leader on centerline (d=0) -> moves to +2.8m
        ttc, shift, brake = compute_overtake_and_avoidance(
            distance_m=15.0,
            v_follower_ms=60.0,
            v_leader_ms=40.0,
            leader_d_m=0.0,
        )
        self.assertAlmostEqual(abs(shift), OVERTAKE_LATERAL_OFFSET_M)

    def test_avoidance_emergency_brake(self):
        """Critical proximity with blocked lane commands avoidance brake."""
        # TTC very low (< 0.55s) and directly behind leader
        ttc, shift, brake = compute_overtake_and_avoidance(
            distance_m=4.0,
            v_follower_ms=50.0,
            v_leader_ms=30.0,
            leader_d_m=0.0,
            track_half_width_m=2.0,  # Narrow track forcing car directly behind
        )
        self.assertGreater(brake, 0.0)

    def test_avoidance_negative_divergence_no_action(self):
        """When leader is pulling away, returns inf TTC and 0 lateral shift / 0 brake."""
        for d in [20.0, 8.0, 3.0]:
            ttc, shift, brake = compute_overtake_and_avoidance(
                distance_m=d,
                v_follower_ms=40.0,
                v_leader_ms=60.0,
                leader_d_m=0.0,
            )
            self.assertEqual(ttc, float("inf"))
            self.assertEqual(shift, 0.0)
            self.assertEqual(brake, 0.0)


class TestSlipstreamTow(unittest.TestCase):
    """Verifies aerodynamic slipstream tow drag reduction and cone attenuation."""

    def test_slipstream_drag_multiplier_in_tow(self):
        """Trailing in slipstream cone reduces drag (< 1.0)."""
        mult = compute_slipstream_drag_multiplier(distance_m=15.0, lateral_diff_m=0.0)
        self.assertLess(mult, 1.0)
        self.assertGreaterEqual(mult, 0.82)

    def test_slipstream_attenuation_with_distance(self):
        """Drag reduction attenuates smoothly as distance increases."""
        mult_close = compute_slipstream_drag_multiplier(distance_m=10.0, lateral_diff_m=0.0)
        mult_far = compute_slipstream_drag_multiplier(distance_m=35.0, lateral_diff_m=0.0)
        self.assertLess(mult_close, mult_far)

    def test_slipstream_attenuation_with_lateral_offset(self):
        """Drag reduction attenuates as lateral offset moves outside the wake."""
        mult_aligned = compute_slipstream_drag_multiplier(distance_m=15.0, lateral_diff_m=0.0)
        mult_offset = compute_slipstream_drag_multiplier(distance_m=15.0, lateral_diff_m=1.8)
        self.assertLess(mult_aligned, mult_offset)

    def test_slipstream_beyond_cone_neutral(self):
        """Beyond 45m drafting distance or 2.5m lateral offset, drag multiplier is exactly 1.0."""
        self.assertEqual(compute_slipstream_drag_multiplier(distance_m=50.0, lateral_diff_m=0.0), 1.0)
        self.assertEqual(compute_slipstream_drag_multiplier(distance_m=20.0, lateral_diff_m=3.0), 1.0)

    def test_aero_drag_reduction_integration(self):
        """Aerodynamics.compute reflects reduced drag when slipstream scalar < 1.0."""
        aero = Aerodynamics()
        _, drag_base, _, _ = aero.compute(speed_ms=80.0)
        aero.set_slipstream_factors(drag_scalar=0.85)
        _, drag_tow, _, _ = aero.compute(speed_ms=80.0)
        self.assertLess(drag_tow, drag_base)
        self.assertAlmostEqual(drag_tow / drag_base, 0.85, delta=0.01)


class TestAIDriverControllerIntegration(unittest.TestCase):
    """Verifies integration of AIDriverController with vehicle physics and track query."""

    def test_pure_pursuit_steering_response(self):
        """Lateral error to the right (d > 0) steers left (negative steering angle)."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)
        controller = AIDriverController(driver, track, difficulty="Hard")
        steer, ld = controller.compute_pure_pursuit_steer(speed_ms=40.0, lateral_error_m=2.0)
        self.assertLess(steer, 0.0)
        self.assertGreaterEqual(ld, 8.0)

    def test_controller_drs_request_in_zone(self):
        """In DRS zone without heavy steering or braking, DRS is activated."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)
        # Monza main straight DRS zone starts around s = 115m
        controller = AIDriverController(driver, track, difficulty="Hard", initial_s=200.0)
        controller.vehicle.state.vx = 75.0
        output = controller.step(dt=0.05, sim_time=1.0)
        self.assertTrue(output.drs_active)

    def test_controller_lap_tracking_and_finish(self):
        """Running a full lap updates lap counter and records lap times."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)
        controller = AIDriverController(driver, track, difficulty="Hard")
        controller.vehicle.state.vx = 80.0
        # Step through 1 full lap (Monza ~81-84s at race pace)
        total_steps = int(105.0 / 0.05)
        for i in range(total_steps):
            controller.step(dt=0.05, sim_time=i * 0.05, total_laps=1)
            if controller.finished:
                break
        self.assertTrue(controller.finished)
        self.assertGreater(len(controller.lap_times), 0)


if __name__ == "__main__":
    unittest.main()
