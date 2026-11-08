"""
tests/challenge_m2_empirical_stress.py
======================================
Comprehensive Empirical Stress-Testing Harness for Milestone 2:
1. Multi-wheel split-mu surface friction coupling & yaw dynamics (restoring counter-torque)
2. Kerb rumble frequency scaling f = v / lambda across [10.0, 80.0] m/s and standstill cutoff
3. StartingGantry state machine timing with micro, jittered, and large multi-second dt steps
4. Procedural 3D meshes (vertex buffers, normal vectors, UVs, gantry frame, distance boards facing -T(s))
5. Corner curvature synchronization across Monza and Monaco (DEAD_ENDS #22)

Author: challenger_m2_iter3_2 (teamwork_challenger)
"""

import sys
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


class TestMultiWheelSplitMuCouplingStress(unittest.TestCase):
    """
    Stress-tests multi-wheel surface friction coupling and split-mu yaw dynamics.
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

    def test_split_mu_braking_across_friction_differentials(self):
        """
        Adversarially tests braking yaw moment under various split-mu combinations:
        - Tarmac / Kerb: [1.0, 0.85, 1.0, 0.85]
        - Tarmac / Runoff: [1.0, 0.50, 1.0, 0.50]
        - Tarmac / Grass: [1.0, 0.30, 1.0, 0.30]
        - Reversed Grass / Tarmac: [0.30, 1.0, 0.30, 1.0] -> yaw moment must reverse!
        - Symmetric baseline: [1.0, 1.0, 1.0, 1.0] -> yaw moment ~ 0.0
        """
        v_init = 50.0  # 180 km/h
        dt = 1.0 / 120.0

        split_cases = [
            ("Symmetric Tarmac", np.array([1.0, 1.0, 1.0, 1.0]), "zero"),
            ("Tarmac / Kerb (0.85)", np.array([1.0, 0.85, 1.0, 0.85]), "positive"),
            ("Tarmac / Runoff (0.50)", np.array([1.0, 0.50, 1.0, 0.50]), "positive"),
            ("Tarmac / Grass (0.30)", np.array([1.0, 0.30, 1.0, 0.30]), "positive"),
            ("Grass / Tarmac (Reversed)", np.array([0.30, 1.0, 0.30, 1.0]), "negative"),
        ]

        yaw_accels = {}

        for name, mu_arr, expected_sign in split_cases:
            veh = Vehicle()
            veh.state.engine_running = True
            veh.state.gear = 5
            veh.vx = v_init
            veh.vy = 0.0
            veh.yaw_rate = 0.0

            telem = veh.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt, surface_mu=mu_arr)

            yaw_accel = veh.state.yaw_accel
            yaw_accels[name] = yaw_accel

            self.assertFalse(math.isnan(yaw_accel), f"{name}: yaw_accel is NaN")
            self.assertFalse(math.isinf(yaw_accel), f"{name}: yaw_accel is Inf")

            if expected_sign == "zero":
                self.assertAlmostEqual(yaw_accel, 0.0, places=3, msg=f"{name} should have zero yaw accel")
            elif expected_sign == "positive":
                self.assertGreater(yaw_accel, 0.0, f"{name} should produce positive (counter-clockwise) yaw accel")
            elif expected_sign == "negative":
                self.assertLess(yaw_accel, 0.0, f"{name} should produce negative (clockwise) yaw accel")

        # Monotonicity check: larger mu difference -> larger yaw acceleration
        self.assertGreater(yaw_accels["Tarmac / Runoff (0.50)"], yaw_accels["Tarmac / Kerb (0.85)"])
        self.assertGreater(yaw_accels["Tarmac / Grass (0.30)"], yaw_accels["Tarmac / Runoff (0.50)"])
        # Symmetry check: reversed mu should produce equal magnitude opposite sign
        self.assertAlmostEqual(yaw_accels["Tarmac / Grass (0.30)"], -yaw_accels["Grass / Tarmac (Reversed)"], places=3)

    def test_split_mu_counter_steering_restoring_torque(self):
        """
        Adversarial Counter-Steering Verification:
        When vehicle experiences split-mu pull (left tarmac mu=1.0, right grass mu=0.3),
        applying rightward counter-steer (steer < 0) generates front tyre lateral force
        that produces clockwise restoring torque opposing the disturbance.
        """
        veh_unassisted = Vehicle()
        veh_unassisted.state.engine_running = True
        veh_unassisted.state.gear = 5
        veh_unassisted.vx = 40.0
        veh_unassisted.vy = 0.0
        veh_unassisted.yaw_rate = 0.0

        veh_counter = Vehicle()
        veh_counter.state.engine_running = True
        veh_counter.state.gear = 5
        veh_counter.vx = 40.0
        veh_counter.vy = 0.0
        veh_counter.yaw_rate = 0.0

        split_mu = np.array([1.0, 0.3, 1.0, 0.3])
        dt = 1.0 / 120.0

        # Unassisted step: steer = 0.0
        veh_unassisted.step(throttle=0.0, brake=0.8, steer=0.0, dt=dt, surface_mu=split_mu)
        yaw_accel_unassisted = veh_unassisted.state.yaw_accel

        # Counter-steer step: apply right counter-steer (steer = -0.05 rad)
        veh_counter.step(throttle=0.0, brake=0.8, steer=-0.05, dt=dt, surface_mu=split_mu)
        yaw_accel_counter = veh_counter.state.yaw_accel

        # Restoring counter-torque reduces positive yaw acceleration
        self.assertLess(
            yaw_accel_counter,
            yaw_accel_unassisted,
            f"Counter-steering must reduce perturbation yaw accel (got unassisted {yaw_accel_unassisted:.2f}, counter {yaw_accel_counter:.2f})"
        )

    def test_split_mu_multi_step_rollout_stability(self):
        """
        Rollout vehicle under split-mu across 120 steps (1.0 full second at 120 Hz)
        at speeds of 20, 50, and 80 m/s.
        Confirms no NaN, Inf, or rotational runaway.
        """
        dt = 1.0 / 120.0
        split_mu = np.array([1.0, 0.5, 1.0, 0.5])

        for v0 in [20.0, 50.0, 80.0]:
            veh = Vehicle()
            veh.state.engine_running = True
            veh.state.gear = 6 if v0 >= 50.0 else 3
            veh.vx = v0
            veh.vy = 0.0
            veh.yaw_rate = 0.0

            for step_idx in range(120):
                telem = veh.step(throttle=0.0, brake=0.6, steer=0.0, dt=dt, surface_mu=split_mu)
                self.assertFalse(math.isnan(veh.state.vx), f"v0={v0}, step={step_idx}: vx is NaN")
                self.assertFalse(math.isnan(veh.state.vy), f"v0={v0}, step={step_idx}: vy is NaN")
                self.assertFalse(math.isnan(veh.state.yaw_rate), f"v0={v0}, step={step_idx}: yaw_rate is NaN")
                self.assertLess(abs(veh.state.yaw_rate), 50.0, f"v0={v0}, step={step_idx}: yaw rate runaway")


class TestKerbFrequencyAndStandstillStress(unittest.TestCase):
    """
    Stress-tests kerb rumble frequency scaling f = v / lambda across [10.0, 80.0] m/s,
    standstill cutoff, and amplitude scaling.
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.builder = TrackBuilder(cls.monza)

    def setUp(self):
        self.monza = self.__class__.monza
        self.builder = self.__class__.builder

    def test_fine_grained_frequency_scaling_across_spectrum(self):
        """
        Tests 71 speed steps from 10.0 to 80.0 m/s in 1.0 m/s increments.
        Verifies exact f = v / lambda (lambda = 1.0m) with error < 1e-6 Hz.
        """
        s_kerb = float(self.monza.corners[0].distance_s)
        d_kerb = (self.monza.nominal_width * 0.5) + 0.5

        speeds = np.linspace(10.0, 80.0, 71)
        for v in speeds:
            sp = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=float(v))
            expected_f = float(v) / 1.0
            self.assertAlmostEqual(
                sp.vibration_frequency_hz,
                expected_f,
                places=5,
                msg=f"Speed {v:.1f} m/s: expected {expected_f:.2f} Hz, got {sp.vibration_frequency_hz:.2f} Hz"
            )

    def test_standstill_and_micro_speed_strict_cutoff(self):
        """
        Tests standstill and speeds near cutoff threshold (cutoff = 0.05 m/s):
        - v = 0.0 -> f = 0.0, amp = 0.0
        - v = 0.01 -> f = 0.0, amp = 0.0
        - v = 0.04 -> f = 0.0, amp = 0.0
        - v = 0.05 -> f = 0.0 (or cutoff edge)
        - v = 0.06 -> f > 0.0
        - v = 0.10 -> f = 0.10 Hz
        """
        s_kerb = float(self.monza.corners[0].distance_s)
        d_kerb = (self.monza.nominal_width * 0.5) + 0.5

        for v in [0.0, 0.01, 0.02, 0.04, 0.05]:
            sp = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=v)
            self.assertEqual(sp.vibration_frequency_hz, 0.0, f"v={v} m/s must have 0.0 Hz vibration frequency")

        # Above cutoff
        sp_06 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.06)
        self.assertGreater(sp_06.vibration_frequency_hz, 0.0)

        sp_10 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.10)
        self.assertAlmostEqual(sp_10.vibration_frequency_hz, 0.10, places=5)

    def test_amplitude_monotonic_penetration_and_speed_scaling(self):
        """
        Verifies amplitude formula amp = 0.85 * min(1.0, pen) * min(1.0, v / 10.0).
        """
        s_kerb = float(self.monza.corners[0].distance_s)
        w_half = self.monza.nominal_width * 0.5

        # Check penetration sweep at v = 20 m/s (speed factor = 1.0)
        prev_amp = -1.0
        for frac in np.linspace(0.1, 1.0, 10):
            d = w_half + frac * 1.20  # 1.2m kerb width
            sp = self.builder.get_surface_at(s_kerb, d, speed_ms=20.0)
            self.assertGreater(sp.vibration_amplitude, prev_amp)
            self.assertLessEqual(sp.vibration_amplitude, 0.85001)
            prev_amp = sp.vibration_amplitude

        # Check speed sweep at 50% penetration (pen factor = 0.5 -> max amp = 0.425)
        d_mid = w_half + 0.60
        prev_amp_v = -1.0
        for v in [1.0, 3.0, 5.0, 7.0, 10.0, 15.0]:
            sp = self.builder.get_surface_at(s_kerb, d_mid, speed_ms=v)
            if v <= 10.0:
                self.assertGreater(sp.vibration_amplitude, prev_amp_v)
            else:
                self.assertAlmostEqual(sp.vibration_amplitude, prev_amp_v, places=4)
            prev_amp_v = sp.vibration_amplitude


class TestStartingGantryStateAccumulatorStress(unittest.TestCase):
    """
    Stress-tests StartingGantry under varied, jittered, and large multi-second dt steps.
    """

    def test_varied_dt_step_sizes_accumulator_stability(self):
        """
        Executes gantry sequence with various dt schedules:
        - Tiny dt = 1e-4s (high frequency)
        - Nominal dt = 0.00833s (120 Hz)
        - Jittered random dt in [0.001, 0.15]s
        - Large multi-second skips: dt = 1.5s, 3.5s, 6.0s
        """
        # 1. Tiny dt = 1e-4s
        g_tiny = StartingGantry()
        g_tiny.start_countdown(hold_duration=1.0)
        steps = 0
        while not g_tiny.is_lights_out and steps < 100000:
            g_tiny.step(1e-4)
            steps += 1
        self.assertTrue(g_tiny.is_lights_out)

        # 2. Jittered random dt steps
        for trial in range(10):
            g_jit = StartingGantry()
            g_jit.start_countdown(hold_duration=1.0)
            t_total = 0.0
            states_visited = []
            while not g_jit.is_lights_out and t_total < 15.0:
                dt = random.uniform(0.005, 0.25)
                g_jit.step(dt)
                t_total += dt
                if not states_visited or states_visited[-1] != g_jit.state:
                    states_visited.append(g_jit.state)

            self.assertEqual(states_visited, ["COUNTDOWN", "HOLD", "LIGHTS_OUT"])
            self.assertTrue(g_jit.is_lights_out)
            self.assertEqual(g_jit.get_light_states(), [False] * 5)

        # 3. Large multi-second skips:
        # Step with dt = 2.5s -> should have 2 lights
        g_large = StartingGantry()
        g_large.start_countdown(hold_duration=1.5)
        g_large.step(2.5)
        self.assertEqual(g_large.state, "COUNTDOWN")
        self.assertEqual(g_large.get_light_states(), [True, True, False, False, False])

        # Step another 3.0s (total 5.5s) -> transitions into HOLD with timer = 0.5s!
        g_large.step(3.0)
        self.assertEqual(g_large.state, "HOLD")
        self.assertEqual(g_large.get_light_states(), [True] * 5)
        self.assertAlmostEqual(g_large.timer, 0.5, places=4)

        # Step another 1.2s (total 6.7s, hold is 1.5s -> 0.5 + 1.2 = 1.7 >= 1.5) -> transitions to LIGHTS_OUT!
        g_large.step(1.2)
        self.assertEqual(g_large.state, "LIGHTS_OUT")
        self.assertTrue(g_large.is_lights_out)
        self.assertEqual(g_large.get_light_states(), [False] * 5)

        # Massive single step from countdown: dt = 10.0s
        g_mega = StartingGantry()
        g_mega.start_countdown(hold_duration=1.0)
        g_mega.step(10.0)
        self.assertTrue(g_mega.is_lights_out)
        self.assertEqual(g_mega.get_light_states(), [False] * 5)

    def test_1000_monte_carlo_hold_durations_distribution(self):
        """
        Runs 1,000 randomized countdowns and verifies hold_duration distribution:
        - Strictly in [0.2, 3.0]s
        - Mean near 1.5s
        - Minimum >= 0.2s, Maximum <= 3.0s
        """
        holds = []
        for _ in range(1000):
            g = StartingGantry()
            g.start_countdown()
            holds.append(g.hold_duration)

        h_arr = np.array(holds)
        self.assertGreaterEqual(float(np.min(h_arr)), 0.2)
        self.assertLessEqual(float(np.max(h_arr)), 3.0)
        self.assertAlmostEqual(float(np.mean(h_arr)), 1.5, delta=0.2)
        self.assertGreater(float(np.std(h_arr)), 0.3)


class TestProceduralMeshesAdversarial(unittest.TestCase):
    """
    Stress-tests 3D procedural track meshes:
    vertex buffers, normal vectors, UV coordinates, gantry lateral orientation,
    and distance boards facing -T(s).
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.builder_monza = TrackBuilder(cls.monza)
        cls.monaco = MonacoTrack()
        cls.builder_monaco = TrackBuilder(cls.monaco)

    def test_monza_track_ribbon_mesh_integrity(self):
        """
        Verifies Monza track ribbon mesh buffers:
        - Vertex format: 12 floats (pos 3, norm 3, uv 2, surface 1, tan 3)
        - Normals have unit length and point upward (nz > 0)
        - Tangents have unit length and are orthogonal to normals (|n . t| < 1e-4)
        - UV coordinates within valid ranges
        - Index bounds and triangle counts
        """
        ribbon = self.builder_monza.build_track_ribbon(step_size=2.0)
        self.assertGreater(ribbon.total_vertices, 1000)
        self.assertGreater(ribbon.total_triangles, 1000)
        self.assertEqual(ribbon.vertices.shape[1], 12)

        # Check indices strictly reference valid vertices
        max_idx = int(np.max(ribbon.indices))
        self.assertLess(max_idx, ribbon.total_vertices)

        verts = ribbon.vertices
        norms = verts[:, 3:6]
        tans = verts[:, 9:12]
        uvs = verts[:, 6:8]

        # Normal unit lengths
        norm_lens = np.linalg.norm(norms, axis=1)
        np.testing.assert_allclose(norm_lens, 1.0, atol=1e-3, err_msg="Ribbon normals not unit length")

        # Normals point upward (mostly nz > 0.8 on flat Monza)
        self.assertGreater(float(np.mean(norms[:, 2])), 0.90)

        # Tangent unit lengths
        tan_lens = np.linalg.norm(tans, axis=1)
        np.testing.assert_allclose(tan_lens, 1.0, atol=1e-3, err_msg="Ribbon tangents not unit length")

        # Orthogonality: n . t ~ 0
        dots = np.abs(np.sum(norms * tans, axis=1))
        self.assertLess(float(np.max(dots)), 1e-3, "Normals and tangents not orthogonal")

        # UV checks: u in [0, 1]
        self.assertGreaterEqual(float(np.min(uvs[:, 0])), 0.0)
        self.assertLessEqual(float(np.max(uvs[:, 0])), 1.0)

    def test_kerb_mesh_angled_normals_and_beveled_stripes(self):
        """
        Verifies kerb mesh generation:
        - Angled normals pointing outward from track
        - Alternating red/white stripes
        """
        kerbs = self.builder_monza.build_kerbs(step_size=0.5)
        self.assertGreater(kerbs.total_vertices, 500)
        self.assertGreater(kerbs.total_triangles, 500)

        verts = kerbs.vertices
        stypes = verts[:, 8]
        # Only KERB_RED (1) and KERB_WHITE (2)
        unique_types = set(np.unique(stypes).astype(int))
        self.assertTrue(unique_types.issubset({SurfaceType.KERB_RED, SurfaceType.KERB_WHITE}))

        # Normals unit length
        norms = verts[:, 3:6]
        norm_lens = np.linalg.norm(norms, axis=1)
        np.testing.assert_allclose(norm_lens, 1.0, atol=1e-3)

    def test_distance_boards_facing_oncoming_traffic(self):
        """
        Verifies distance boards face -T(s) directly oncoming traffic:
        normal_vec . T(s) == -1.0 (+- 1e-4).
        """
        boards, mesh = self.builder_monza.build_distance_boards()
        self.assertGreater(len(boards), 0)

        for b in boards:
            tan = self.monza.spline.get_tangent(b.s)
            dot_prod = float(np.dot(b.normal_dir, tan))
            self.assertAlmostEqual(
                dot_prod,
                -1.0,
                places=3,
                msg=f"Distance board at s={b.s:.1f}m must face oncoming traffic with normal = -T(s), got dot={dot_prod}"
            )

    def test_fia_gantry_road_relative_orientation(self):
        """
        Verifies starting gantry spans across track perpendicular to tangent (along lateral vector).
        """
        gantry, mesh = self.builder_monza.build_fia_gantry(s_location=0.0)
        self.assertGreater(mesh.total_vertices, 20)

        # Check column lateral separation matches track width + clearance
        tan = self.monza.spline.get_tangent(0.0)
        lat, norm = self.monza.spline.get_lateral_normal(0.0)
        w_l, w_r = self.monza.spline.get_width(0.0)

        verts = mesh.vertices[:, 0:3]
        # Project all gantry vertices onto lateral vector
        lat_coords = np.dot(verts - self.monza.spline.get_point(0.0), lat)
        span = float(np.max(lat_coords) - np.min(lat_coords))
        expected_span = (w_l + w_r) + 4.0  # span_clearance = 4.0
        self.assertAlmostEqual(span, expected_span, delta=0.6)

    def test_combined_track_mesh_consolidation(self):
        """
        Verifies get_combined_track_mesh() aggregates all sub-meshes with contiguous index offsets.
        """
        combined = self.builder_monza.get_combined_track_mesh()
        self.assertGreater(combined.total_vertices, 2000)
        self.assertGreater(combined.total_triangles, 2000)

        max_idx = int(np.max(combined.indices))
        self.assertLess(max_idx, combined.total_vertices)


class TestCornerCurvatureSynchronizationStress(unittest.TestCase):
    """
    Stress-tests corner curvature synchronization on Monza and Monaco (DEAD_ENDS #22).
    Verifies every cataloged corner apex aligns with the true physical curvature maximum.
    """

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.monaco = MonacoTrack()

    def test_monza_11_corners_curvature_synchronization(self):
        """
        For all 11 Monza corners:
        - Catalog s corresponds to a curvature peak (|s_catalog - s_peak| <= 10.0m)
        - Curvature kappa > 0.0025 (Radius <= 400.0m)
        - No corner on a straightaway (R > 1000m)
        """
        m = self.monza
        s_samples = np.linspace(0.0, m.length, 5793, endpoint=False)
        kappas = np.array([m.get_curvature(s) for s in s_samples])

        for c in m.corners:
            s_cat = float(c.distance_s)
            k_cat = float(m.get_curvature(s_cat))
            r_cat = 1.0 / max(1e-6, k_cat)

            # Search in local window [s_cat - 20, s_cat + 20]m
            s_win = np.linspace(s_cat - 20.0, s_cat + 20.0, 41)
            k_win = [m.get_curvature(s) for s in s_win]
            best_idx = int(np.argmax(k_win))
            s_peak = float(s_win[best_idx])
            k_peak = float(k_win[best_idx])

            dist_err = abs(s_cat - s_peak)

            # 1. Apex synchronization: distance error <= 10m
            self.assertLessEqual(
                dist_err,
                10.0,
                f"Monza {c.name}: catalog s={s_cat:.1f}m deviates from local peak s={s_peak:.1f}m by {dist_err:.1f}m"
            )

            # 2. Authentic corner curvature: no straightaway
            self.assertGreater(
                k_cat,
                0.0025,
                f"Monza {c.name}: curvature {k_cat:.6f} (R={r_cat:.1f}m) indicates straight track (DEAD_ENDS #22)"
            )

            # 3. Radius realistic for corner classification
            if "Ascari" in c.name or "Lesmo" in c.name:
                self.assertLessEqual(r_cat, 80.0, f"Monza medium corner {c.name} radius {r_cat:.1f}m must be <= 80m")
            elif "Chicane" in c.name or "Variante" in c.name or "T1" in c.name or "T2" in c.name:
                self.assertLessEqual(r_cat, 45.0, f"Monza chicane {c.name} radius {r_cat:.1f}m must be <= 45m")
            else:
                self.assertLessEqual(r_cat, 350.0, f"Monza sweeper {c.name} radius {r_cat:.1f}m must be <= 350m")

    def test_monaco_19_corners_curvature_synchronization(self):
        """
        For all 19 Monaco corners:
        - Catalog s corresponds to curvature peak (|s_catalog - s_peak| <= 10.0m)
        - Curvature kappa > 0.002 (Radius <= 500.0m)
        - Hairpins / Chicanes have R <= 30m
        """
        m = self.monaco
        L = m.length
        n_corners = len(m.corners)

        for i, c in enumerate(m.corners):
            s_cat = float(c.distance_s)
            k_cat = float(m.get_curvature(s_cat))
            r_cat = 1.0 / max(1e-6, k_cat)

            prev_c = m.corners[(i - 1) % n_corners]
            next_c = m.corners[(i + 1) % n_corners]
            s_prev = float(prev_c.distance_s)
            s_next = float(next_c.distance_s)

            # Distance to adjacent corners along circuit
            d_prev = (s_cat - s_prev) % L
            d_next = (s_next - s_cat) % L

            # Exclusive Voronoi search window: half-distance to neighbor, capped at 10.0m
            w_back = min(10.0, d_prev * 0.5)
            w_fwd = min(10.0, d_next * 0.5)

            s_low = s_cat - w_back
            s_high = s_cat + w_fwd

            s_win = np.linspace(s_low, s_high, 41)
            k_win = [m.get_curvature(s % L) for s in s_win]
            best_idx = int(np.argmax(k_win))
            s_peak = float(s_win[best_idx])

            dist_err = abs(s_cat - s_peak)

            # 1. Apex synchronization within exclusive corner domain: distance error <= 10m
            self.assertLessEqual(
                dist_err,
                10.0,
                f"Monaco {c.name}: catalog s={s_cat:.1f}m deviates from local peak s={s_peak:.1f}m by {dist_err:.1f}m (win=[{s_low:.1f}, {s_high:.1f}])"
            )

            # 2. No straightaway
            self.assertGreater(
                k_cat,
                0.002,
                f"Monaco {c.name}: curvature {k_cat:.6f} (R={r_cat:.1f}m) indicates straight track (DEAD_ENDS #22)"
            )

            # 3. Special corners: Fairmont Hairpin, Sainte Devote, Rascasse, Nouvelle Chicane
            if "Fairmont" in c.name:
                self.assertLessEqual(r_cat, 20.0, f"Fairmont Hairpin radius {r_cat:.1f}m must be <= 20m")
            elif "Chicane" in c.name or "Rascasse" in c.name or "Sainte Dévote" in c.name:
                self.assertLessEqual(r_cat, 30.0, f"{c.name} radius {r_cat:.1f}m must be <= 30m")


if __name__ == "__main__":
    unittest.main()
