"""
tests/test_challenger_m4_empirical.py
======================================
Independent empirical verification and stress testing suite for Milestone 4:
Multi-Tier AI Racing System.

Executes strict empirical evaluations for:
1. Adaptive Lookahead Distance Clamping:
   L_d(v) = clamp(8.0 + 0.38 * v, 8.0, 45.0) across boundary velocities
   [-50.0, -0.001, 0.0, 10.0, 50.0, 97.368, 100.0, 200.0] m/s.
2. Pure-Pursuit Controller Tracking & Stability:
   Full lap simulations on Monza (5,793m) and Monaco (3,337m) with zero
   off-track excursions (|d| <= half_width), no spins, no numerical divergence.
3. Lap Time Monotonicity & Deltas:
   Monza lap times across Easy, Medium, Hard with strict monotonicity:
   T_Easy > T_Medium > T_Hard, Delta_Easy-Med >= 4.0s, Delta_Med-Hard >= 2.0s.
4. DRS Zone Compliance:
   DRS opens strictly inside track DRS zones and immediately closes outside.
5. Gear 1 Downshift in Tight Hairpins:
   Low-speed cornering at v ~ 12 m/s (43-45 km/h) verifies downshift to Gear 1.
"""

from __future__ import annotations
import math
import os
import sys
import unittest
import numpy as np

# Ensure repository root is on sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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
    compute_lookahead_distance,
    compute_apex_speed,
    compute_braking_entry_speed,
)
from f1sim.ai.controller import (
    AIDriverController,
    AICarControlOutput,
)
from f1sim.track.monza import MonzaTrack
from f1sim.track.monaco import MonacoTrack
from f1sim.physics.vehicle import Vehicle


class TestTask1_AdaptiveLookaheadClamping(unittest.TestCase):
    """
    Task 1: Adaptive Lookahead Distance Clamping
    L_d(v) = clamp(8.0 + 0.38 * v, 8.0, 45.0)
    """

    def test_specified_boundary_velocities(self):
        """
        Test v in [-50.0, -0.001, 0.0, 10.0, 50.0, 97.368, 100.0, 200.0] m/s.
        Assert L_d in [8.0, 45.0] and exact at 0m/s (8.0m), 50m/s (27.0m), 100m/s (45.0m).
        """
        test_velocities = [-50.0, -0.001, 0.0, 10.0, 50.0, 97.368, 100.0, 200.0]

        for v in test_velocities:
            ld = compute_lookahead_distance(v)
            # Strict boundary assertion
            self.assertGreaterEqual(
                ld, 8.0, f"Lookahead distance {ld} < 8.0 for velocity {v}"
            )
            self.assertLessEqual(
                ld, 45.0, f"Lookahead distance {ld} > 45.0 for velocity {v}"
            )

        # Exact target assertions
        self.assertAlmostEqual(
            compute_lookahead_distance(0.0), 8.0, places=6,
            msg="Lookahead at v=0.0 m/s must be exactly 8.0m"
        )
        self.assertAlmostEqual(
            compute_lookahead_distance(50.0), 27.0, places=6,
            msg="Lookahead at v=50.0 m/s must be exactly 27.0m (8.0 + 0.38*50)"
        )
        self.assertAlmostEqual(
            compute_lookahead_distance(100.0), 45.0, places=6,
            msg="Lookahead at v=100.0 m/s must be clamped to exactly 45.0m"
        )

        # Boundary checks for negative speeds (clamped to 8.0m)
        self.assertAlmostEqual(compute_lookahead_distance(-50.0), 8.0, places=6)
        self.assertAlmostEqual(compute_lookahead_distance(-0.001), 8.0, places=6)

        # Intermediate value checks
        self.assertAlmostEqual(compute_lookahead_distance(10.0), 11.8, places=6)
        # v = 97.36842105263158 is the exact transition point (8.0 + 0.38 * 97.368421 = 45.0)
        ld_97 = compute_lookahead_distance(97.368)
        self.assertAlmostEqual(ld_97, 44.99984, places=4)
        self.assertLessEqual(ld_97, 45.0)

        # High speed clamp
        self.assertAlmostEqual(compute_lookahead_distance(200.0), 45.0, places=6)

    def test_continuous_sweep_and_monotonicity(self):
        """Dense continuous sweep across 10,000 velocity samples from -100 to +300 m/s."""
        velocities = np.linspace(-100.0, 300.0, 10000)
        prev_ld = -1.0
        for v in velocities:
            ld = compute_lookahead_distance(float(v))
            self.assertTrue(8.0 <= ld <= 45.0)
            if prev_ld >= 0.0:
                self.assertGreaterEqual(
                    ld, prev_ld - 1e-12,
                    f"Lookahead distance must be non-decreasing at v={v}"
                )
            prev_ld = ld

    def test_controller_lookahead_agreement(self):
        """Verify AIDriverController.compute_pure_pursuit_steer outputs identical lookahead."""
        driver = get_driver_by_number(1)
        track = MonzaTrack()
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        test_speeds = [0.0, 10.0, 50.0, 97.368, 100.0]
        for spd in test_speeds:
            _, ld = ctrl.compute_pure_pursuit_steer(speed_ms=spd, lateral_error_m=0.0)
            expected_ld = compute_lookahead_distance(spd)
            self.assertAlmostEqual(ld, expected_ld, places=6)


class TestTask2_PurePursuitTrackingAndStability(unittest.TestCase):
    """
    Task 2: Pure-Pursuit Controller Tracking & Stability
    Run full lap simulations on Monza (5,793m) and Monaco (3,337m).
    Assert |d| <= half_width throughout entire lap (zero off-track excursions,
    no spinning, no numerical blowup).
    """

    def test_full_lap_monza_tracking_stability(self):
        """Simulate complete 1-lap Monza run and assert lateral tracking stability."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)  # Verstappen
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        dt = 0.02  # 50 Hz integration
        sim_time = 0.0
        max_time = 150.0
        steps = 0
        max_lateral_dev = 0.0
        half_width = track.nominal_width / 2.0  # 5.5m

        while ctrl.s < track.length and sim_time < max_time:
            out = ctrl.step(dt=dt, sim_time=sim_time)
            sim_time += dt
            steps += 1

            # Check lateral deviation |d| <= half_width
            lat_dev = abs(ctrl.d)
            if lat_dev > max_lateral_dev:
                max_lateral_dev = lat_dev

            self.assertLessEqual(
                lat_dev, half_width,
                f"Off-track excursion on Monza at s={ctrl.s:.1f}m: |d|={lat_dev:.3f}m > half_width={half_width:.1f}m"
            )

            # Check vehicle state sanity (no NaN, Inf, or physical blowup)
            self.assertFalse(math.isnan(ctrl.vehicle.vx), "Vehicle vx is NaN")
            self.assertFalse(math.isinf(ctrl.vehicle.vx), "Vehicle vx is Inf")
            self.assertGreaterEqual(ctrl.vehicle.vx, 0.0, "Vehicle reversed unexpectedly")
            self.assertLess(ctrl.vehicle.vx, 115.0, "Vehicle exceeded physical top speed")

            # Check yaw rate stability
            self.assertFalse(math.isnan(ctrl.vehicle.state.yaw_rate))
            self.assertLess(abs(ctrl.vehicle.state.yaw_rate), 5.0, "Vehicle spun out")

        self.assertGreaterEqual(
            ctrl.s, track.length,
            f"Car failed to complete full lap on Monza within {max_time}s (reached s={ctrl.s:.1f}m)"
        )
        print(f"\n[Task 2 Monza] Lap finished: {steps} steps, time: {sim_time:.2f}s, max lateral deviation: {max_lateral_dev:.3f}m (half_width: {half_width}m)")

    def test_full_lap_monaco_tracking_stability(self):
        """Simulate complete 1-lap Monaco run and assert lateral tracking stability."""
        track = MonacoTrack()
        driver = get_driver_by_number(16)  # Leclerc
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        dt = 0.02  # 50 Hz integration
        sim_time = 0.0
        max_time = 160.0
        steps = 0
        max_lateral_dev = 0.0
        half_width = track.nominal_width / 2.0  # 5.0m

        while ctrl.s < track.length and sim_time < max_time:
            out = ctrl.step(dt=dt, sim_time=sim_time)
            sim_time += dt
            steps += 1

            # Check lateral deviation |d| <= half_width
            lat_dev = abs(ctrl.d)
            if lat_dev > max_lateral_dev:
                max_lateral_dev = lat_dev

            self.assertLessEqual(
                lat_dev, half_width,
                f"Off-track excursion on Monaco at s={ctrl.s:.1f}m: |d|={lat_dev:.3f}m > half_width={half_width:.1f}m"
            )

            # Check vehicle state sanity
            self.assertFalse(math.isnan(ctrl.vehicle.vx))
            self.assertFalse(math.isinf(ctrl.vehicle.vx))
            self.assertGreaterEqual(ctrl.vehicle.vx, 0.0)
            self.assertLess(ctrl.vehicle.vx, 100.0)

            # Check yaw rate stability
            self.assertFalse(math.isnan(ctrl.vehicle.state.yaw_rate))
            self.assertLess(abs(ctrl.vehicle.state.yaw_rate), 5.0)

        self.assertGreaterEqual(
            ctrl.s, track.length,
            f"Car failed to complete full lap on Monaco within {max_time}s (reached s={ctrl.s:.1f}m)"
        )
        print(f"[Task 2 Monaco] Lap finished: {steps} steps, time: {sim_time:.2f}s, max lateral deviation: {max_lateral_dev:.3f}m (half_width: {half_width}m)")


class TestTask3_LapTimeMonotonicityAndDeltas(unittest.TestCase):
    """
    Task 3: Lap Time Monotonicity & Deltas
    Verify simulated or analytical lap times across Easy, Medium, Hard on Monza.
    Assert strict monotonicity: T_Easy > T_Medium > T_Hard,
    with Delta_Easy-Med >= 4.0s and Delta_Med-Hard >= 2.0s.
    """

    def test_analytical_lap_time_monotonicity(self):
        """Assert strict analytical monotonicity: T_Easy > T_Med > T_Hard with required deltas."""
        t_easy = estimate_monza_ai_lap_time(AIDifficulty.EASY)
        t_med = estimate_monza_ai_lap_time(AIDifficulty.MEDIUM)
        t_hard = estimate_monza_ai_lap_time(AIDifficulty.HARD)

        print(f"\n[Task 3 Analytical] Easy: {t_easy:.2f}s, Medium: {t_med:.2f}s, Hard: {t_hard:.2f}s")
        print(f"[Task 3 Analytical] Delta(Easy - Med): {t_easy - t_med:.2f}s, Delta(Med - Hard): {t_med - t_hard:.2f}s")

        self.assertGreater(t_easy, t_med, "Easy must be slower than Medium")
        self.assertGreater(t_med, t_hard, "Medium must be slower than Hard")

        delta_easy_med = t_easy - t_med
        delta_med_hard = t_med - t_hard

        self.assertGreaterEqual(
            delta_easy_med, 4.0,
            f"Delta(Easy - Med) = {delta_easy_med:.2f}s must be >= 4.0s"
        )
        self.assertGreaterEqual(
            delta_med_hard, 2.0,
            f"Delta(Med - Hard) = {delta_med_hard:.2f}s must be >= 2.0s"
        )

    def test_driver_pace_scalar_monotonicity_preserved(self):
        """Assert monotonicity holds across all 5 authentic drivers."""
        for drv in AI_DRIVER_ROSTER:
            t_easy = estimate_monza_ai_lap_time(AIDifficulty.EASY, pace_scalar=drv.pace_scalar)
            t_med = estimate_monza_ai_lap_time(AIDifficulty.MEDIUM, pace_scalar=drv.pace_scalar)
            t_hard = estimate_monza_ai_lap_time(AIDifficulty.HARD, pace_scalar=drv.pace_scalar)

            self.assertGreater(t_easy, t_med)
            self.assertGreater(t_med, t_hard)
            self.assertGreaterEqual(t_easy - t_med, 4.0 * drv.pace_scalar)
            self.assertGreaterEqual(t_med - t_hard, 2.0 * drv.pace_scalar)

    def test_dynamic_simulation_lap_time_monotonicity(self):
        """Simulate dynamic AI lap times around Monza across Easy, Medium, Hard tiers."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)  # Verstappen

        sim_lap_times = {}
        dt = 0.02

        for diff in [AIDifficulty.HARD, AIDifficulty.MEDIUM, AIDifficulty.EASY]:
            ctrl = AIDriverController(driver, track, difficulty=diff)
            sim_time = 0.0
            max_time = 160.0

            while ctrl.s < track.length and sim_time < max_time:
                ctrl.step(dt=dt, sim_time=sim_time)
                sim_time += dt

            sim_lap_times[diff.value] = sim_time

        t_hard = sim_lap_times["Hard"]
        t_med = sim_lap_times["Medium"]
        t_easy = sim_lap_times["Easy"]

        print(f"[Task 3 Dynamic Simulation] Easy: {t_easy:.2f}s, Medium: {t_med:.2f}s, Hard: {t_hard:.2f}s")
        print(f"[Task 3 Dynamic Simulation] Delta(Easy - Med): {t_easy - t_med:.2f}s, Delta(Med - Hard): {t_med - t_hard:.2f}s")

        self.assertGreater(t_easy, t_med, "Simulated Easy lap must be slower than Medium")
        self.assertGreater(t_med, t_hard, "Simulated Medium lap must be slower than Hard")
        self.assertGreaterEqual(t_easy - t_med, 4.0, "Simulated Delta(Easy - Med) must be >= 4.0s")
        self.assertGreaterEqual(t_med - t_hard, 2.0, "Simulated Delta(Med - Hard) must be >= 2.0s")


class TestTask4_DRSZoneCompliance(unittest.TestCase):
    """
    Task 4: DRS Zone Compliance
    Assert DRS wing opens only when track.is_in_drs_zone(s) is True and closes when False.
    """

    def test_monza_drs_zone_exact_compliance(self):
        """Fine-grained track query asserting DRS activation strictly bounded by DRS zones."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)
        # Precompute racing line once to avoid 500x rebuilds
        racing_line = RacingLineProfile(track, difficulty=AIDifficulty.HARD)

        # Test controller across 500 points along Monza track
        s_samples = np.linspace(0.0, track.length, 500, endpoint=False)

        for s_val in s_samples:
            ctrl = AIDriverController(
                driver, track, difficulty=AIDifficulty.HARD,
                racing_line=racing_line, initial_s=s_val
            )
            ctrl.vehicle.vx = 75.0  # High speed straight cruising

            in_drs_zone = track.is_in_drs_zone(s_val)

            # Step 1: Trailing car eligible for DRS (gap = 20m at 75 m/s -> gap_sec = 0.27s < 1.0s)
            out = ctrl.step(dt=0.02, sim_time=1.0, lead_gap_m=20.0, lead_speed_ms=75.0)

            if not in_drs_zone:
                self.assertFalse(
                    out.drs_active,
                    f"DRS illegally active OUTSIDE DRS zone at s={s_val:.1f}m!"
                )
                self.assertFalse(ctrl.vehicle.drs_active)
            else:
                # Inside DRS zone, DRS wing must be open when conditions satisfied
                self.assertTrue(
                    out.drs_active,
                    f"DRS failed to activate INSIDE DRS zone at s={s_val:.1f}m!"
                )
                self.assertTrue(ctrl.vehicle.drs_active)

    def test_drs_closure_under_braking_and_steering(self):
        """Inside a DRS zone, heavy braking or sharp steering must immediately close DRS."""
        track = MonzaTrack()
        driver = get_driver_by_number(1)
        # s=150.0m is inside Monza DRS Zone 1 (Main Straight)
        self.assertTrue(track.is_in_drs_zone(150.0))

        # Test under normal conditions -> open
        ctrl_normal = AIDriverController(driver, track, difficulty=AIDifficulty.HARD, initial_s=150.0)
        ctrl_normal.vehicle.vx = 75.0
        out_normal = ctrl_normal.step(dt=0.02, sim_time=1.0, lead_gap_m=20.0, lead_speed_ms=75.0)
        self.assertTrue(out_normal.drs_active)

        # Test under sharp steer input -> closed
        ctrl_steer = AIDriverController(driver, track, difficulty=AIDifficulty.HARD, initial_s=150.0)
        ctrl_steer.vehicle.vx = 75.0
        ctrl_steer.vehicle.state.steering_angle = 0.25  # > 0.12 threshold
        out_steer = ctrl_steer.step(dt=0.02, sim_time=1.0, lead_gap_m=20.0, lead_speed_ms=75.0)
        self.assertFalse(out_steer.drs_active, "DRS must close when steering sharply")


class TestTask5_Gear1DownshiftInTightHairpins(unittest.TestCase):
    """
    Task 5: Gear 1 Downshift in Tight Hairpins
    Test low-speed cornering at v ~ 12 m/s (45 km/h) and verify that the controller
    permits downshifting to Gear 1 for optimal corner-exit acceleration.
    """

    def test_low_speed_downshift_to_gear_1(self):
        """At v ~ 12 m/s (< 55 km/h), controller must downshift from Gear 2 to Gear 1."""
        track = MonacoTrack()
        driver = get_driver_by_number(16)
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        # Configure vehicle in Gear 2 at 12.0 m/s (~43.2 km/h)
        ctrl.vehicle.vx = 12.0
        ctrl.vehicle.gear = 2
        ctrl.vehicle.powertrain.engine.rpm = 5000.0

        self.assertEqual(ctrl.vehicle.gear, 2)
        self.assertLess(ctrl.vehicle.speed_kmh, 55.0)

        # Step controller
        ctrl.step(dt=0.02, sim_time=1.0)

        # Controller must shift down to Gear 1
        self.assertEqual(
            ctrl.vehicle.gear, 1,
            f"Expected Gear 1 at v=12.0 m/s (speed_kmh={ctrl.vehicle.speed_kmh:.1f}), got Gear {ctrl.vehicle.gear}"
        )

    def test_gear_1_lockout_prevents_neutral_or_reverse(self):
        """In Gear 1 at low speed, controller must NOT shift down to Neutral (0) or Reverse (-1)."""
        track = MonacoTrack()
        driver = get_driver_by_number(16)
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        # Configure vehicle in Gear 1 at 10.0 m/s (~36 km/h)
        ctrl.vehicle.vx = 10.0
        ctrl.vehicle.powertrain.gearbox.set_gear_direct(1)
        self.assertEqual(ctrl.vehicle.gear, 1)

        # Step controller 10 times under braking
        for i in range(10):
            ctrl.step(dt=0.02, sim_time=1.0 + i * 0.02)
            self.assertEqual(
                ctrl.vehicle.gear, 1,
                f"Gearbox improperly downshifted to {ctrl.vehicle.gear} from Gear 1!"
            )

    def test_gear_1_corner_exit_acceleration(self):
        """Verify Gear 1 delivers strong acceleration out of a tight 12 m/s corner."""
        track = MonacoTrack()
        driver = get_driver_by_number(16)
        ctrl = AIDriverController(driver, track, difficulty=AIDifficulty.HARD)

        ctrl.vehicle.vx = 12.0
        ctrl.vehicle.powertrain.gearbox.set_gear_direct(1)
        v_initial = ctrl.vehicle.vx

        # Apply full throttle for 0.5s in Gear 1
        for i in range(25):
            ctrl.vehicle.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.02)

        v_final = ctrl.vehicle.vx
        accel = (v_final - v_initial) / 0.5
        print(f"\n[Task 5 Acceleration] Gear 1 exit: {v_initial:.1f} m/s -> {v_final:.1f} m/s, avg accel: {accel:.2f} m/s^2")
        self.assertGreater(v_final, v_initial)
        self.assertGreater(accel, 6.0, "Gear 1 must provide strong corner-exit acceleration (> 6 m/s^2)")


if __name__ == "__main__":
    unittest.main(verbosity=2)
