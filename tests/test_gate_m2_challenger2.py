"""
tests/test_gate_m2_challenger2.py
=================================
Empirical Adversarial Stress-Testing Suite for Milestone 2:
Track Physical Dynamics, Kerb Rumble, Multi-Wheel Surface Friction,
Starting Gantry Countdown, and DRS Exact Boundaries.

Author: challenger_m2_2 (teamwork_challenger)
Target: f1sim/track/
"""

import math
import random
import unittest
import numpy as np

from f1sim.track import (
    SurfaceType,
    SurfaceProperties,
    CornerInfo,
    DRSZone,
    DistanceMarker,
    PitLane,
    KerbZone,
    RunoffZone,
    GridBox,
    MeshBufferData,
    Track,
    TrackSpline,
    MonzaTrack,
    MonacoTrack,
    TrackBuilder,
    StartingGantry
)
from f1sim.physics.vehicle import Vehicle, VehicleConfig, VehicleState
from f1sim.core.constants import RPM_IDLE, RPM_REDLINE


class TestKerbVibrationFrequencyAdversarial(unittest.TestCase):
    """
    Adversarial Challenge 1: Kerb Rumble Vibration Frequency across velocities v in [10.0, 80.0] m/s.
    Verifies f_kerb = v / lambda_stripe with lambda_stripe = 1.0m (specifically 25.0 Hz at 25.0 m/s / 90 km/h).
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.builder = TrackBuilder(cls.monza)

    def setUp(self):
        self.monza = self.__class__.monza
        self.builder = self.__class__.builder

    def test_rumble_frequency_across_speed_spectrum(self):
        """
        Verify f = v / lambda_stripe for speeds from 10.0 m/s to 80.0 m/s at 5.0 m/s increments.
        Specifically verifies 25.0 Hz at 25.0 m/s (90 km/h).
        """
        # Turn 1 apex kerb at Monza (synchronized with corner distance_s)
        s_kerb = float(self.monza.corners[0].distance_s)
        w_half = self.monza.nominal_width * 0.5
        d_kerb = w_half + 0.5  # 0.5m onto kerb

        test_speeds = np.linspace(10.0, 80.0, 15)  # [10, 15, ..., 80] m/s
        for v in test_speeds:
            surf = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=float(v))
            self.assertTrue(surf.is_kerb, f"Position (s={s_kerb}, d={d_kerb}) should be on kerb")
            self.assertIn(surf.surface_type, (SurfaceType.KERB_RED, SurfaceType.KERB_WHITE))

            expected_freq = float(v) / 1.0  # lambda_stripe = 1.0m
            self.assertAlmostEqual(
                surf.vibration_frequency_hz,
                expected_freq,
                places=3,
                msg=f"At v={v} m/s, expected freq {expected_freq} Hz, got {surf.vibration_frequency_hz} Hz"
            )

        # Explicit target verification: 25.0 Hz at 25.0 m/s (90 km/h)
        surf_25 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=25.0)
        self.assertEqual(surf_25.vibration_frequency_hz, 25.0)
        self.assertEqual(surf_25.friction_mu, 0.85)
        self.assertEqual(surf_25.elevation_step, 0.035)  # 35mm FIA bevel

    def test_rumble_standstill_and_low_speed_cutoff(self):
        """
        Verify vibration frequency drops to 0.0 Hz at standstill or sub-cutoff speed (<= 0.05 m/s).
        """
        s_kerb = float(self.monza.corners[0].distance_s)
        w_half = self.monza.nominal_width * 0.5
        d_kerb = w_half + 0.5

        # Standstill v = 0.0
        surf_0 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.0)
        self.assertEqual(surf_0.vibration_frequency_hz, 0.0)
        self.assertEqual(surf_0.vibration_amplitude, 0.0)

        # Micro creep v = 0.04 m/s (below 0.05 m/s cutoff)
        surf_creep = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.04)
        self.assertEqual(surf_creep.vibration_frequency_hz, 0.0)

    def test_vibration_amplitude_penetration_and_speed_scaling(self):
        """
        Verify kerb vibration amplitude scales monotonically with penetration into kerb and speed.
        amp = 0.85 * min(1.0, pen) * min(1.0, v / 10.0).
        """
        s_kerb = float(self.monza.corners[0].distance_s)
        w_half = self.monza.nominal_width * 0.5

        # Shallow kerb touch (pen = 0.1) vs deep kerb mount (pen = 1.0)
        surf_shallow = self.builder.get_surface_at(s_kerb, w_half + 0.12, speed_ms=20.0)  # 1.2m kerb width -> 10%
        surf_deep = self.builder.get_surface_at(s_kerb, w_half + 1.15, speed_ms=20.0)     # ~95% penetration

        self.assertGreater(surf_deep.vibration_amplitude, surf_shallow.vibration_amplitude)
        self.assertLessEqual(surf_deep.vibration_amplitude, 0.85)

        # Speed scaling: amplitude at 2 m/s vs 10 m/s
        surf_slow = self.builder.get_surface_at(s_kerb, w_half + 0.6, speed_ms=2.0)
        surf_fast = self.builder.get_surface_at(s_kerb, w_half + 0.6, speed_ms=10.0)
        self.assertGreater(surf_fast.vibration_amplitude, surf_slow.vibration_amplitude)

    def test_variable_stripe_length_frequency_response(self):
        """
        Verify f = v / lambda_stripe when kerb zone has non-standard stripe lengths (e.g. 0.5m, 2.0m).
        """
        custom_kerb = KerbZone(start_s=100.0, end_s=200.0, side='both', stripe_length=0.50)
        builder_custom = TrackBuilder(self.monza, kerb_zones=[custom_kerb])

        w_half = self.monza.nominal_width * 0.5
        surf = builder_custom.get_surface_at(150.0, w_half + 0.5, speed_ms=25.0)
        # f = 25.0 / 0.50 = 50.0 Hz
        self.assertAlmostEqual(surf.vibration_frequency_hz, 50.0, places=3)


class TestMultiWheelSurfaceFrictionCouplingAdversarial(unittest.TestCase):
    """
    Adversarial Challenge 2: Multi-Wheel Surface Friction Coupling and Vehicle Dynamics Response.
    Evaluates get_wheel_surface_properties across tarmac, kerb, runoff, gravel, and grass.
    Runs simulated dynamic vehicle steps under split-mu (left tarmac mu=1.0, right grass mu=0.3)
    and verifies yaw moment and traction asymmetry respond physically without crashing or NaNs.
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.builder = TrackBuilder(cls.monza)

    def setUp(self):
        self.monza = self.__class__.monza
        self.builder = self.__class__.builder
        self.vehicle = Vehicle()
        self.vehicle.state.engine_running = True
        self.vehicle.state.rpm = RPM_IDLE
        self.vehicle.state.gear = 1

    def test_contact_patch_surface_classification(self):
        """
        Verify get_wheel_surface_properties on vehicle contact patches across all 5 surface types.
        """
        w_half = self.monza.nominal_width * 0.5
        s_kerb = float(self.monza.corners[0].distance_s)
        c_pos = self.monza.get_spline_point(s_kerb)[0]
        norm, _ = self.monza.spline.get_lateral_normal(s_kerb)

        # 1. Tarmac (centerline)
        p_tarmac = c_pos + 1.0 * norm
        surf_t = self.builder.get_surface_properties(p_tarmac[0], p_tarmac[1], p_tarmac[2], speed_ms=20.0)
        self.assertEqual(surf_t.surface_type, SurfaceType.TARMAC)
        self.assertEqual(surf_t.friction_mu, 1.00)
        self.assertEqual(surf_t.rolling_resistance_mult, 1.00)

        # 2. Kerb (1.2m zone outside tarmac)
        p_kerb = c_pos + (w_half + 0.6) * norm
        surf_k = self.builder.get_surface_properties(p_kerb[0], p_kerb[1], p_kerb[2], speed_ms=20.0)
        self.assertTrue(surf_k.is_kerb)
        self.assertEqual(surf_k.friction_mu, 0.85)

        # 3. Grass Verge (beyond kerb on standard section)
        c_pos_grass = self.monza.get_spline_point(500.0)[0]
        norm_grass, _ = self.monza.spline.get_lateral_normal(500.0)
        p_grass = c_pos_grass + (w_half + 12.0) * norm_grass
        surf_g = self.builder.get_surface_properties(p_grass[0], p_grass[1], p_grass[2], speed_ms=20.0)
        self.assertEqual(surf_g.surface_type, SurfaceType.GRASS_VERGE)
        self.assertEqual(surf_g.friction_mu, 0.30)
        self.assertEqual(surf_g.rolling_resistance_mult, 2.50)

    def test_get_wheel_surface_properties_quad_evaluation(self):
        """
        Verify vectorized 4-wheel query returns exact shape (4,) float64 array and per-wheel props.
        """
        w_half = self.monza.nominal_width * 0.5
        s_kerb = float(self.monza.corners[0].distance_s)
        c_pos = self.monza.get_spline_point(s_kerb)[0]
        norm, _ = self.monza.spline.get_lateral_normal(s_kerb)

        # FL & RL on tarmac, FR & RR on grass
        fl = c_pos + 0.0 * norm
        rl = c_pos + 0.0 * norm
        fr = c_pos + (w_half + 15.0) * norm
        rr = c_pos + (w_half + 15.0) * norm

        wheel_positions = np.array([fl, fr, rl, rr], dtype=np.float64)
        mu_arr, props, vibe = self.builder.get_wheel_surface_properties(wheel_positions, speed_ms=30.0)

        self.assertEqual(mu_arr.shape, (4,))
        self.assertEqual(mu_arr.dtype, np.float64)
        self.assertEqual(mu_arr[0], 1.00)  # FL tarmac
        self.assertEqual(mu_arr[1], 0.30)  # FR grass
        self.assertEqual(mu_arr[2], 1.00)  # RL tarmac
        self.assertEqual(mu_arr[3], 0.30)  # RR grass
        self.assertEqual(len(props), 4)

    def test_dynamic_vehicle_step_split_mu_braking_yaw_moment(self):
        """
        Adversarial Split-Mu Braking:
        Left wheels on tarmac (mu = 1.0), Right wheels on grass (mu = 0.3).
        Vehicle traveling at 50 m/s (180 km/h) applies full brake (brake = 1.0, steer = 0.0).
        Verifies:
        1. Substantial asymmetric braking force (F_brake_left > F_brake_right).
        2. Non-zero induced yaw moment Mz > 0 and positive yaw acceleration pulling towards tarmac.
        3. Stable physical integration without NaNs, infinities, or simulation crashes.
        """
        self.vehicle.vx = 50.0
        self.vehicle.vy = 0.0
        self.vehicle.yaw_rate = 0.0
        self.vehicle.state.gear = 5

        # Split-mu surface array: [FL: 1.0, FR: 0.3, RL: 1.0, RR: 0.3]
        split_mu = np.array([1.0, 0.3, 1.0, 0.3], dtype=np.float64)

        # Baseline: Symmetric tarmac braking
        sym_vehicle = Vehicle()
        sym_vehicle.state.engine_running = True
        sym_vehicle.state.gear = 5
        sym_vehicle.vx = 50.0
        telem_sym = sym_vehicle.step(throttle=0.0, brake=1.0, steer=0.0, dt=1.0 / 120.0, surface_mu=np.ones(4))

        # Split-mu braking step
        telem_split = self.vehicle.step(throttle=0.0, brake=1.0, steer=0.0, dt=1.0 / 120.0, surface_mu=split_mu)

        # 1. No numerical explosion
        self.assertFalse(math.isnan(telem_split.speed_kmh))
        self.assertFalse(math.isnan(self.vehicle.state.yaw_accel))
        self.assertFalse(math.isinf(self.vehicle.state.yaw_accel))

        # 2. Asymmetric yaw moment generated:
        # In symmetric braking, yaw_accel is ~0.0
        self.assertAlmostEqual(sym_vehicle.state.yaw_accel, 0.0, places=3)

        # In split-mu braking (left high mu, right low mu), left wheels produce much higher
        # longitudinal braking drag than right wheels, generating a strong positive yaw moment
        # (pulling the car counter-clockwise towards the left tarmac side).
        self.assertGreater(
            self.vehicle.state.yaw_accel,
            1.5,
            f"Expected positive yaw acceleration under split-mu braking, got {self.vehicle.state.yaw_accel}"
        )

        # 3. Overall deceleration is lower on split-mu than on full tarmac
        self.assertGreater(telem_split.longitudinal_g, telem_sym.longitudinal_g)  # Less negative Gs

    def test_dynamic_vehicle_step_split_mu_acceleration_traction_asymmetry(self):
        """
        Adversarial Split-Mu Launch / Acceleration:
        Left wheels on tarmac (mu = 1.0), Right wheels on grass (mu = 0.3).
        Vehicle traveling at 20 m/s in gear 2 accelerates at 100% throttle (throttle = 1.0, steer = 0.0).
        Verifies:
        1. Traction asymmetry produces a net yaw acceleration.
        2. Dynamic vehicle state remains strictly bounded and physically valid.
        """
        self.vehicle.vx = 20.0
        self.vehicle.vy = 0.0
        self.vehicle.yaw_rate = 0.0
        self.vehicle.state.gear = 2
        self.vehicle.state.rpm = 8000.0

        split_mu = np.array([1.0, 0.3, 1.0, 0.3], dtype=np.float64)

        # Single step
        telem = self.vehicle.step(throttle=1.0, brake=0.0, steer=0.0, dt=1.0 / 120.0, surface_mu=split_mu)

        self.assertFalse(math.isnan(telem.speed_kmh))
        self.assertGreater(telem.longitudinal_g, 0.0)

        # Multi-step 120 Hz rollout (60 steps = 0.5s) under split-mu
        for _ in range(60):
            telem = self.vehicle.step(throttle=0.8, brake=0.0, steer=0.0, dt=1.0 / 120.0, surface_mu=split_mu)
            self.assertFalse(math.isnan(self.vehicle.state.vx))
            self.assertFalse(math.isnan(self.vehicle.state.yaw_rate))

        # Vehicle must maintain physical bounds
        self.assertGreater(self.vehicle.state.vx, 20.0)
        self.assertLess(abs(self.vehicle.state.yaw_rate), 20.0)  # No rotational runaway


class TestDRSExactBoundariesAdversarial(unittest.TestCase):
    """
    Adversarial Challenge 3: DRS Exact Boundaries on Monza & Monaco.
    Tests step-by-step traversal across DRS activation points (Monza Main Straight s = 115.0m,
    Serraglio s = 2360.0m; Monaco s = 18.0m). Confirms exact boundary detection without false triggers.
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.monaco = MonacoTrack()

    def setUp(self):
        self.monza = self.__class__.monza
        self.monaco = self.__class__.monaco

    def test_monza_main_straight_drs_exact_boundaries(self):
        """
        Monza Zone 1 (Main Straight): activation_s = 115.0m, deactivation_s = 980.0m.
        Test with micro-increments epsilon = 0.05m.
        """
        eps = 0.05

        # Before activation point
        self.assertFalse(self.monza.is_in_drs_zone(115.0 - eps))
        # Exact activation point
        self.assertTrue(self.monza.is_in_drs_zone(115.0))
        # Immediately inside activation
        self.assertTrue(self.monza.is_in_drs_zone(115.0 + eps))

        # Midpoint of DRS zone
        self.assertTrue(self.monza.is_in_drs_zone(500.0))

        # Immediately before deactivation point
        self.assertTrue(self.monza.is_in_drs_zone(980.0 - eps))
        # Exact deactivation point
        self.assertTrue(self.monza.is_in_drs_zone(980.0))
        # Immediately after deactivation point
        self.assertFalse(self.monza.is_in_drs_zone(980.0 + eps))

    def test_monza_serraglio_straight_drs_exact_boundaries(self):
        """
        Monza Zone 2 (Serraglio Straight): activation_s = 2360.0m, deactivation_s = 3950.0m.
        """
        eps = 0.05

        # Before activation
        self.assertFalse(self.monza.is_in_drs_zone(2360.0 - eps))
        # Exact activation
        self.assertTrue(self.monza.is_in_drs_zone(2360.0))
        # Inside
        self.assertTrue(self.monza.is_in_drs_zone(2360.0 + eps))
        self.assertTrue(self.monza.is_in_drs_zone(3000.0))

        # At deactivation
        self.assertTrue(self.monza.is_in_drs_zone(3950.0))
        # Past deactivation
        self.assertFalse(self.monza.is_in_drs_zone(3950.0 + eps))

    def test_monaco_pit_straight_drs_exact_boundaries(self):
        """
        Monaco Zone 1 (Pit Straight): activation_s = 18.0m, deactivation_s = 528.0m (length 510m).
        """
        eps = 0.05

        # Before activation
        self.assertFalse(self.monaco.is_in_drs_zone(18.0 - eps))
        # Exact activation
        self.assertTrue(self.monaco.is_in_drs_zone(18.0))
        # Inside
        self.assertTrue(self.monaco.is_in_drs_zone(18.0 + eps))
        self.assertTrue(self.monaco.is_in_drs_zone(250.0))

        # At deactivation
        self.assertTrue(self.monaco.is_in_drs_zone(528.0))
        # Past deactivation
        self.assertFalse(self.monaco.is_in_drs_zone(528.0 + eps))

    def test_drs_false_trigger_absence_across_corners(self):
        """
        Verify DRS is strictly FALSE in all braking zones, corners, chicanes, and tunnels.
        """
        # On Monza: 0..115 is false, 980..2360 is false, 3950..5793 is false
        for s in [50.0, 100.0, 1000.0, 1500.0, 2000.0, 4200.0, 5000.0, 5500.0]:
            self.assertFalse(
                self.monza.is_in_drs_zone(s),
                f"Monza distance s={s}m must NOT trigger DRS"
            )

        # Monaco key corners outside Pit Straight (18m..528m)
        monaco_non_drs = [
            600.0,   # Massenet
            700.0,   # Casino Square
            1150.0,  # Fairmont Hairpin
            1400.0,  # Portier
            1750.0,  # Tunnel
            2250.0,  # Nouvelle Chicane
            2470.0,  # Tabac
            2700.0,  # Swimming Pool
            3090.0,  # La Rascasse
            3200.0,  # Antony Noghes
        ]
        for s in monaco_non_drs:
            self.assertFalse(
                self.monaco.is_in_drs_zone(s),
                f"Monaco distance s={s}m must NOT trigger DRS"
            )

    def test_drs_wraparound_invariance(self):
        """
        Verify DRS state is invariant under full circuit wraparound (s + k * L).
        """
        # Monza
        for k in [-2, -1, 1, 2, 5]:
            s_active = 500.0 + k * self.monza.length
            s_inactive = 1500.0 + k * self.monza.length
            self.assertTrue(self.monza.is_in_drs_zone(s_active))
            self.assertFalse(self.monza.is_in_drs_zone(s_inactive))

        # Monaco
        for k in [-2, -1, 1, 2, 5]:
            s_active = 200.0 + k * self.monaco.length
            s_inactive = 1000.0 + k * self.monaco.length
            self.assertTrue(self.monaco.is_in_drs_zone(s_active))
            self.assertFalse(self.monaco.is_in_drs_zone(s_inactive))


class TestStartingGantryCountdownAndLaunchAdversarial(unittest.TestCase):
    """
    Adversarial Challenge 4: Starting Gantry Countdown & Launch State Machine.
    Runs 100 iterations of StartingGantry.step(dt).
    Verifies:
    1. 5 red lights illuminate sequentially at exact 1.0s intervals.
    2. Random hold falls strictly within [0.2, 3.0]s.
    3. All 5 lights extinguish simultaneously (lights == [False, False, False, False, False]).
    4. Zero intermediate flickering or partial light extinguishment.
    """

    def test_100_iterations_starting_gantry_countdown_and_hold(self):
        """
        Executes 100 independent trials of the starting sequence.
        """
        hold_durations = []

        for trial in range(100):
            gantry = StartingGantry(s_location=0.0)
            self.assertEqual(gantry.state, "IDLE")
            self.assertEqual(gantry.get_light_states(), [False] * 5)
            self.assertFalse(gantry.is_lights_out)

            # Start countdown with default randomized hold
            gantry.start_countdown()
            self.assertEqual(gantry.state, "COUNTDOWN")
            hold_durations.append(gantry.hold_duration)

            # Check hold duration range requirement: [0.2, 3.0]s
            self.assertGreaterEqual(
                gantry.hold_duration,
                0.2,
                f"Trial {trial}: hold_duration {gantry.hold_duration} < 0.2s"
            )
            self.assertLessEqual(
                gantry.hold_duration,
                3.0,
                f"Trial {trial}: hold_duration {gantry.hold_duration} > 3.0s"
            )

            # Advance by dt = 0.05s increments
            dt = 0.05
            while gantry.state == "COUNTDOWN":
                gantry.step(dt)
                if gantry.state == "COUNTDOWN":
                    expected_count = min(5, int(gantry.timer // 1.0))
                    expected_lights = [(i < expected_count) for i in range(5)]
                    self.assertEqual(
                        gantry.get_light_states(),
                        expected_lights,
                        f"Trial {trial} at timer={gantry.timer:.3f}s: expected {expected_lights}, got {gantry.get_light_states()}"
                    )

            # Reached HOLD state: all 5 lights MUST be ON
            self.assertEqual(gantry.state, "HOLD")
            self.assertEqual(gantry.get_light_states(), [True, True, True, True, True])
            self.assertFalse(gantry.is_lights_out)

            # Step through entire HOLD phase
            while gantry.state == "HOLD":
                gantry.step(0.01)
                if gantry.state == "HOLD":
                    self.assertEqual(
                        gantry.get_light_states(),
                        [True, True, True, True, True],
                        f"Trial {trial}: Light dropped during HOLD"
                    )

            # LIGHTS OUT: all 5 lights extinguish simultaneously!
            self.assertEqual(gantry.state, "LIGHTS_OUT")
            self.assertEqual(
                gantry.get_light_states(),
                [False, False, False, False, False],
                f"Trial {trial}: Lights failed to extinguish simultaneously!"
            )
            self.assertTrue(gantry.is_lights_out)

        # Statistical verification across 100 trials:
        mean_hold = np.mean(hold_durations)
        std_hold = np.std(hold_durations)
        self.assertGreater(std_hold, 0.1, "Hold durations must exhibit authentic pseudo-random variance")
        self.assertAlmostEqual(mean_hold, 1.5, delta=0.5)

    def test_gantry_step_method_compatibility(self):
        """
        Verify StartingGantry responds to step(dt) and provides proper introspection.
        """
        gantry = StartingGantry()
        self.assertTrue(callable(getattr(gantry, "step", None)))


if __name__ == "__main__":
    unittest.main()
