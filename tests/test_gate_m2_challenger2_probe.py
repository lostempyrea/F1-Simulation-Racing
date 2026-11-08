"""
tests/test_gate_m2_challenger2_probe.py
=======================================
Empirical Challenge & Adversarial Stress Probe for Milestone 2 Iteration 2.
Directly probes the 5 adversarial challenge vectors mandated in DISPATCH.md:
1. Multi-wheel surface coupling & split-mu dynamics
2. Kerb rumble vibration frequency spectrum & low-speed cutoff
3. Starting gantry multi-step timer & extreme dt carry-over
4. 3D procedural meshes (gantry lateral span, distance boards facing -tan, angled kerb normals)
5. Off-track verge deceleration & dual-mode friction contract
"""

import math
import unittest
import numpy as np

from f1sim.track import (
    SurfaceType,
    SurfaceProperties,
    MonzaTrack,
    MonacoTrack,
    TrackBuilder,
    StartingGantry,
    MeshBufferData
)
from f1sim.physics.vehicle import Vehicle
from f1sim.core.constants import RPM_IDLE


class TestAdversarialVector1MultiWheel(unittest.TestCase):
    """Vector 1: Multi-wheel surface coupling and split-mu dynamics."""

    def setUp(self):
        self.monza = MonzaTrack()
        self.builder = TrackBuilder(self.monza)

    def test_quad_wheel_surface_properties_and_split_mu(self):
        w_half = self.monza.nominal_width * 0.5
        c_pos = self.monza.get_spline_point(1180.0)[0]
        norm, _ = self.monza.spline.get_lateral_normal(1180.0)

        # FL & RL on tarmac, FR & RR on grass verge
        fl = c_pos + 0.0 * norm
        rl = c_pos + 0.0 * norm
        fr = c_pos + (w_half + 12.0) * norm
        rr = c_pos + (w_half + 12.0) * norm

        wheel_pos = np.array([fl, fr, rl, rr], dtype=np.float64)
        mu_arr, props, max_vibe = self.builder.get_wheel_surface_properties(wheel_pos, speed_ms=25.0)

        self.assertEqual(mu_arr.shape, (4,))
        self.assertEqual(mu_arr[0], 1.00)
        self.assertEqual(mu_arr[1], 0.30)
        self.assertEqual(mu_arr[2], 1.00)
        self.assertEqual(mu_arr[3], 0.30)

        # Vehicle step with split-mu
        veh = Vehicle()
        veh.state.engine_running = True
        veh.state.gear = 5
        veh.vx = 50.0

        telem = veh.step(throttle=0.0, brake=1.0, steer=0.0, dt=1.0 / 120.0, surface_mu=mu_arr)
        self.assertFalse(math.isnan(telem.speed_kmh))
        self.assertFalse(math.isnan(veh.state.yaw_accel))
        self.assertGreater(veh.state.yaw_accel, 1.5, f"Yaw accel was {veh.state.yaw_accel}")

    def test_split_mu_extreme_traction_launch(self):
        veh = Vehicle()
        veh.state.engine_running = True
        veh.state.gear = 2
        veh.state.rpm = 8000.0
        veh.vx = 15.0

        split_mu = np.array([1.0, 0.3, 1.0, 0.3], dtype=np.float64)
        for _ in range(120):  # 1.0s at 120 Hz
            telem = veh.step(throttle=1.0, brake=0.0, steer=0.0, dt=1.0 / 120.0, surface_mu=split_mu)
            self.assertFalse(math.isnan(veh.state.vx))
            self.assertFalse(math.isnan(veh.state.yaw_rate))
            self.assertLess(abs(veh.state.yaw_rate), 25.0)


class TestAdversarialVector2KerbRumble(unittest.TestCase):
    """Vector 2: Kerb rumble vibration frequency spectrum & low-speed cutoff."""

    def setUp(self):
        self.monza = MonzaTrack()
        self.builder = TrackBuilder(self.monza)

    def test_frequency_spectrum_10_to_80_ms(self):
        s_kerb = 1180.0
        w_half = self.monza.nominal_width * 0.5
        d_kerb = w_half + 0.6

        # Check speeds from 10 to 80 m/s
        for v in range(10, 81, 5):
            surf = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=float(v))
            expected_f = float(v) / 1.0  # lambda = 1.0m
            self.assertAlmostEqual(surf.vibration_frequency_hz, expected_f, places=3)
            self.assertTrue(surf.is_kerb)
            self.assertEqual(surf.friction_mu, 0.85)

        # Exactly 25 Hz at 25 m/s (90 km/h)
        surf_25 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=25.0)
        self.assertEqual(surf_25.vibration_frequency_hz, 25.0)

    def test_standstill_and_low_speed_cutoff(self):
        s_kerb = 1180.0
        w_half = self.monza.nominal_width * 0.5
        d_kerb = w_half + 0.6

        surf_0 = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.0)
        self.assertEqual(surf_0.vibration_frequency_hz, 0.0)
        self.assertEqual(surf_0.vibration_amplitude, 0.0)

        surf_creep = self.builder.get_surface_at(s_kerb, d_kerb, speed_ms=0.04)
        self.assertEqual(surf_creep.vibration_frequency_hz, 0.0)


class TestAdversarialVector3StartingGantry(unittest.TestCase):
    """Vector 3: Starting gantry multi-step timer with extreme dt carry-over."""

    def test_extreme_step_dt_6_5s(self):
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=1.0)
        self.assertEqual(gantry.state, "COUNTDOWN")

        # Step 6.5s: 5.0s countdown + 1.0s hold leaves 0.5s into LIGHTS_OUT
        gantry.step(6.5)
        self.assertEqual(gantry.state, "LIGHTS_OUT")
        self.assertTrue(gantry.is_lights_out)
        self.assertEqual(gantry.get_light_states(), [False] * 5)
        self.assertAlmostEqual(gantry.lights_out_time, 0.5, places=3)

    def test_extreme_step_dt_12s_transitions_to_racing(self):
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=1.0)

        # Step 12.0s: 5.0s countdown + 1.0s hold + 5.0s lights_out leaves 1.0s -> RACING
        gantry.step(12.0)
        self.assertEqual(gantry.state, "RACING")
        self.assertTrue(gantry.is_lights_out)
        self.assertEqual(gantry.get_light_states(), [False] * 5)

    def test_micro_step_dt_0_001s(self):
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=0.5)

        dt = 0.001
        # Advance through 5.0s countdown (5000 steps)
        for step_i in range(5000):
            gantry.step(dt)
            if gantry.state == "COUNTDOWN":
                expected_on = min(5, int(gantry.timer // 1.0))
                self.assertEqual(sum(gantry.get_light_states()), expected_on)

        self.assertEqual(gantry.state, "HOLD")
        self.assertEqual(gantry.get_light_states(), [True] * 5)

        # Advance through 0.5s hold (500 steps)
        for _ in range(500):
            gantry.step(dt)

        self.assertEqual(gantry.state, "LIGHTS_OUT")
        self.assertEqual(gantry.get_light_states(), [False] * 5)

    def test_arbitrary_variable_dt_sequence(self):
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=1.2)

        # Sequence of irregular steps summing to 5.0 + 1.2 + 0.3 = 6.5s
        dts = [1.2, 0.8, 1.5, 0.5, 1.0, 0.7, 0.8]  # Sum = 6.5s
        for step_dt in dts:
            gantry.step(step_dt)

        self.assertEqual(gantry.state, "LIGHTS_OUT")
        self.assertAlmostEqual(gantry.lights_out_time, 0.3, places=3)


class TestAdversarialVector4Meshes(unittest.TestCase):
    """Vector 4: 3D procedural meshes, gantry lateral orientation, distance boards."""

    def setUp(self):
        self.monza = MonzaTrack()
        self.monaco = MonacoTrack()
        self.builder_monza = TrackBuilder(self.monza)
        self.builder_monaco = TrackBuilder(self.monaco)

    def test_fia_gantry_crossbeam_lateral_span_orientation(self):
        """Confirm crossbeam extends along lateral basis and NOT along tangent."""
        for name, builder, track in [("Monza", self.builder_monza, self.monza), ("Monaco", self.builder_monaco, self.monaco)]:
            _, gantry_mesh = builder.build_fia_gantry(s_location=0.0)
            self.assertGreater(gantry_mesh.total_vertices, 0)
            self.assertEqual(gantry_mesh.vertices.shape[1], 12)

            # Vertex format: x, y, z, nx, ny, nz, u, v, stype, tx, ty, tz
            verts = gantry_mesh.vertices
            # Box 2 is the crossbeam: vertices 16 to 23
            cb_verts = verts[16:24, :3]
            lat, norm = track.spline.get_lateral_normal(0.0)
            tan = track.spline.get_tangent(0.0)

            # Project crossbeam bounding box onto lat, tan, norm
            cb_lat_proj = [float(np.dot(p, lat)) for p in cb_verts]
            cb_tan_proj = [float(np.dot(p, tan)) for p in cb_verts]

            span_lat = max(cb_lat_proj) - min(cb_lat_proj)
            span_tan = max(cb_tan_proj) - min(cb_tan_proj)

            # Span along lat should be ~ (width + 4.0m) >= 14m
            # Span along tan should be ~ 0.80m (depth)
            self.assertGreater(span_lat, 10.0, f"{name}: Crossbeam span along lat was {span_lat}")
            self.assertLess(span_tan, 2.0, f"{name}: Crossbeam depth along tan was {span_tan}")

    def test_distance_marker_boards_face_oncoming_traffic(self):
        """Confirm distance boards have surface normal pointing along -tan."""
        boards, boards_mesh = self.builder_monza.build_distance_boards()
        self.assertEqual(len(boards), 9)  # 3 corners * 3 boards (150, 100, 50)
        self.assertEqual(boards_mesh.total_vertices, 36)  # 9 boards * 4 vertices

        verts = boards_mesh.vertices
        for i, b in enumerate(boards):
            tan = self.monza.spline.get_tangent(b.s)
            expected_norm = -tan

            b_verts = verts[i * 4 : (i + 1) * 4]
            # Normal is cols 3, 4, 5
            for v_row in b_verts:
                v_norm = v_row[3:6]
                dot_prod = float(np.dot(v_norm, expected_norm))
                self.assertAlmostEqual(dot_prod, 1.0, places=3, msg=f"Board {i} normal did not match -tan")

    def test_kerb_angled_surface_normals(self):
        """Confirm kerb cross-sections have angled normals for specular highlights."""
        kerb_mesh = self.builder_monza.build_kerbs()
        self.assertGreater(kerb_mesh.total_vertices, 0)

        # Check that kerb vertices have non-purely-vertical normals (nx or ny is non-zero)
        norms = kerb_mesh.vertices[:, 3:6]
        # At least some normals must have lateral tilt (|nx| + |ny| > 0.05)
        tilted_count = sum(1 for n in norms if (n[0]**2 + n[1]**2)**0.5 > 0.1)
        self.assertGreater(tilted_count, 100)

    def test_combined_mesh_packaging(self):
        """Confirm get_combined_track_mesh packs ribbon, kerbs, grid, boards."""
        combined = self.builder_monza.get_combined_track_mesh()
        self.assertIsNotNone(combined)
        self.assertGreater(combined.total_vertices, 1000)
        self.assertEqual(len(combined.indices) % 3, 0)


class TestAdversarialVector5VergeFriction(unittest.TestCase):
    """Vector 5: Off-track verge deceleration & dual-mode friction contract."""

    def setUp(self):
        self.monza = MonzaTrack()
        self.monaco = MonacoTrack()

    def test_dual_mode_friction_contract(self):
        # Monza normalized (default is normalized=False in Track.get_surface_mu, wait!)
        # Let's verify both modes on Monza
        mu_tarmac_abs = self.monza.get_surface_mu(100.0, 0.0, normalized=False)
        mu_tarmac_norm = self.monza.get_surface_mu(100.0, 0.0, normalized=True)
        self.assertEqual(mu_tarmac_abs, 1.75)
        self.assertEqual(mu_tarmac_norm, 1.00)

        mu_verge_abs = self.monza.get_surface_mu(100.0, 20.0, normalized=False)
        mu_verge_norm = self.monza.get_surface_mu(100.0, 20.0, normalized=True)
        self.assertEqual(mu_verge_abs, 0.40)
        self.assertEqual(mu_verge_norm, 0.30)

        # Monaco normalized and absolute
        mu_m_tarmac_abs = self.monaco.get_surface_mu(100.0, 0.0, normalized=False)
        mu_m_tarmac_norm = self.monaco.get_surface_mu(100.0, 0.0, normalized=True)
        self.assertEqual(mu_m_tarmac_abs, 1.75)
        self.assertEqual(mu_m_tarmac_norm, 1.00)

        mu_m_verge_abs = self.monaco.get_surface_mu(100.0, 20.0, normalized=False)
        mu_m_verge_norm = self.monaco.get_surface_mu(100.0, 20.0, normalized=True)
        self.assertEqual(mu_m_verge_abs, 0.40)
        self.assertEqual(mu_m_verge_norm, 0.30)

    def test_kerb_zone_gating_straight_vs_corner(self):
        """Verify is_on_kerb is FALSE on straightaways and TRUE at corner apexes."""
        w_half = self.monza.nominal_width * 0.5
        c_pos_straight = self.monza.get_spline_point(500.0)[0]
        norm_straight, _ = self.monza.spline.get_lateral_normal(500.0)

        # On Monza main straight (s=500m), lateral offset at road edge + 0.5m:
        p_straight_edge = c_pos_straight + (w_half + 0.5) * norm_straight
        self.assertFalse(
            self.monza.is_on_kerb(p_straight_edge),
            "Straightaway road edge must NOT be flagged as kerb!"
        )

        # At Turn 1 apex (s=1180m), lateral offset at road edge + 0.5m:
        c_pos_corner = self.monza.get_spline_point(1180.0)[0]
        norm_corner, _ = self.monza.spline.get_lateral_normal(1180.0)
        p_corner_kerb = c_pos_corner + (w_half + 0.5) * norm_corner
        self.assertTrue(
            self.monza.is_on_kerb(p_corner_kerb),
            "Turn 1 apex kerb MUST be flagged as kerb!"
        )


if __name__ == "__main__":
    unittest.main()
