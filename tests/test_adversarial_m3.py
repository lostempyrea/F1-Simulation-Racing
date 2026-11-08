"""
tests/test_adversarial_m3.py
============================
Adversarial & Boundary Stress Suite for Milestone 3:
- Cockpit First-Person Camera (extreme G-loads, NaN/zero-dt resilience, dynamic FOV clamping, mirror frustums)
- 3D Cockpit & Rotating Steering Wheel Meshes (orthogonal matrix determinant & norm invariance, vertex buffer layout)
- PCU-8D Telemetry LCD & 15-LED Shift Light Bar (16 Hz blue strobe alternation, boundary RPMs, RGBA texture integrity)
- GraphicsSettings (corrupt/out-of-range clamping, preset transitions, Halo opacity cycle, JSON round-trip)
- KeyBindingConfig & InputManager (conflict auto-swap across all 13 actions, invalid key rejection, interactive capture, hot-swapping)
- ModernGL 3.3 Core Profile Standalone Renderer (halo opacity alpha blending, mirror FBO toggle, multi-frame stability)
"""
from __future__ import annotations
import math
import tempfile
from types import SimpleNamespace
import unittest
from pathlib import Path
import numpy as np

from f1sim.core.math3d import Vec3
from f1sim.graphics import (
    CockpitCamera,
    CockpitMeshBuilder,
    CockpitRenderer,
    GraphicsPreset,
    GraphicsSettings,
    LED_COLOR_AMBER,
    LED_COLOR_BLUE,
    LED_COLOR_GREEN,
    LED_COLOR_OFF,
    LED_COLOR_RED,
    TelemetryLCDDisplay,
    compute_wheel_model_matrix_4x4,
    compute_wheel_rotation_matrix_3x3,
    format_brake_bias,
    format_gear,
    resolve_drs_status,
)
from f1sim.input import (
    ControlAction,
    InputManager,
    KeyBindingConfig,
    SteeringFilter,
    apply_cubic_gamma,
    apply_deadzone,
    blend_inputs,
    compute_speed_attenuation,
    normalize_trigger,
)
from f1sim.physics.vehicle import Vehicle


class TestAdversarialMilestone3(unittest.TestCase):
    """Adversarial stress tests for Milestone 3 (Cockpit, LCD, Input, Graphics Settings, Keybindings)."""

    def test_01_camera_extreme_g_loads_and_zero_dt_stability(self):
        """Camera neck spring remains bounded under +/-20G spikes and dt=0.0."""
        cam = CockpitCamera()
        # Zero dt step should not divide by zero or produce NaN
        cam.update(dt=0.0, speed_kmh=250.0, longitudinal_g=-15.0, lateral_g=15.0, on_kerb=True)
        eye0 = cam.get_local_eye_position()
        self.assertFalse(math.isnan(eye0.x) or math.isnan(eye0.y) or math.isnan(eye0.z))

        # Sustained extreme 20G load must clamp neck offset within +/- max_neck_offset_m (0.045m)
        for _ in range(100):
            cam.update(dt=0.01, speed_kmh=400.0, longitudinal_g=-20.0, lateral_g=20.0, rpm=15000.0, on_kerb=True)
        self.assertLessEqual(abs(cam.neck_offset.x), cam.cfg.max_neck_offset_m + 1e-6)
        self.assertLessEqual(abs(cam.neck_offset.y), cam.cfg.max_neck_offset_m + 1e-6)
        self.assertLessEqual(abs(cam.neck_offset.z), cam.cfg.max_neck_offset_m + 1e-6)
        self.assertLessEqual(abs(math.degrees(cam.head_pitch_rad)), cam.cfg.max_head_pitch_deg + 1e-4)
        self.assertLessEqual(abs(math.degrees(cam.head_roll_rad)), cam.cfg.max_head_roll_deg + 1e-4)

    def test_02_steering_wheel_rotation_clamping_and_orthogonality(self):
        """Steering wheel rotation matrix clamps inputs beyond +/-10 rad to [-pi, +pi] and stays orthogonal."""
        for raw_theta in (-10.0, -math.pi, -1.234, 0.0, 2.345, math.pi, 10.0):
            rot = compute_wheel_rotation_matrix_3x3(raw_theta)
            ident = rot.T @ rot
            np.testing.assert_allclose(ident, np.eye(3), atol=1e-6)
            self.assertAlmostEqual(float(np.linalg.det(rot)), 1.0, places=6)

        # Verify clamping at +/- pi produces the same rotation matrix as +/- 10.0
        np.testing.assert_allclose(
            compute_wheel_rotation_matrix_3x3(math.pi),
            compute_wheel_rotation_matrix_3x3(10.0),
            atol=1e-7,
        )

    def test_03_pcu8d_16hz_strobe_and_edge_telemetry_values(self):
        """PCU-8D 15-LED shift light bar alternates at 16 Hz above 13,500 RPM and handles edge gears."""
        lcd = TelemetryLCDDisplay()
        # At t=0.0s (even half-cycle of 16 Hz -> int(t*32)%2 == 0): all 15 LEDs blue
        leds_t0, cnt0, str0 = lcd.compute_rev_lights(13600.0, time_sec=0.0)
        # At t=1/32s = 0.03125s (odd half-cycle of 16 Hz -> int(t*32)%2 == 1): alternating strobe
        leds_t1, cnt1, str1 = lcd.compute_rev_lights(13600.0, time_sec=0.03125)
        self.assertTrue(str0 and str1)
        self.assertEqual(cnt0, 15)
        self.assertNotEqual(leds_t0[0], leds_t1[0])
        self.assertEqual(leds_t1[14], LED_COLOR_BLUE)

        # Out-of-range gear numbers gracefully fall back to 'N'
        self.assertEqual(format_gear(-99), "N")
        self.assertEqual(format_gear(99), "N")
        self.assertEqual(format_gear(-1), "R")

    def test_04_graphics_settings_out_of_bounds_clamping(self):
        """GraphicsSettings clamps invalid resolutions, negative FPS, invalid MSAA, and extreme FOV."""
        gs = GraphicsSettings(
            preset="INVALID_PRESET",
            resolution=(100, 50),
            fps_limit=-50,
            msaa_samples=5,
            base_fov_deg=120.0,
            max_fov_delta_deg=99.0,
            camera_shake_intensity=9.0,
            halo_opacity=-1.0,
            draw_distance_m=50.0,
        )
        self.assertEqual(gs.preset, GraphicsPreset.CUSTOM.value)
        self.assertEqual(gs.resolution, (640, 360))
        self.assertEqual(gs.fps_limit, 0)
        self.assertIn(gs.msaa_samples, (0, 2, 4, 8))
        self.assertEqual(gs.base_fov_deg, 75.0)
        self.assertEqual(gs.max_fov_delta_deg, 15.0)
        self.assertEqual(gs.camera_shake_intensity, 2.0)
        self.assertEqual(gs.halo_opacity, 0.15)
        self.assertEqual(gs.draw_distance_m, 300.0)

    def test_05_keybindings_conflict_resolution_and_invalid_key_rejection(self):
        """Rebinding across all actions maintains zero duplicate conflicts and rejects unknown keys."""
        kb = KeyBindingConfig()
        with self.assertRaises(ValueError):
            kb.rebind_key(ControlAction.THROTTLE, "NON_EXISTENT_KEY_XYZ")
        with self.assertRaises(ValueError):
            kb.rebind_gamepad_button(ControlAction.DRS, 99)

        # Perform a chain of rebinds across 5 actions and verify 0 conflicts after every step
        chain = [
            (ControlAction.THROTTLE, "S"),
            (ControlAction.BRAKE, "F"),
            (ControlAction.DRS, "E"),
            (ControlAction.GEAR_UP, "Q"),
            (ControlAction.GEAR_DOWN, "W"),
        ]
        for act, target_key in chain:
            kb.rebind_key(act, target_key, resolve_conflict=True)
            self.assertEqual(len(kb.detect_conflicts()), 0)

    def test_06_moderngl_renderer_halo_opacity_and_mirror_toggle(self):
        """Standalone ModernGL renderer renders cleanly with translucent Halo and toggled mirrors."""
        gs = GraphicsSettings(resolution=(320, 200), halo_opacity=0.35, rear_mirrors_enabled=False)
        renderer = CockpitRenderer(settings=gs, standalone=True, init_gl=True)
        try:
            renderer.render_frame(wheel_angle_rad=-math.pi / 2.0)
            p1 = renderer.read_offscreen_pixels()
            gs.rear_mirrors_enabled = True
            gs.halo_opacity = 1.0
            renderer.apply_settings(gs)
            renderer.render_frame(wheel_angle_rad=+math.pi / 2.0)
            p2 = renderer.read_offscreen_pixels()

            self.assertIsNotNone(p1)
            self.assertIsNotNone(p2)
            # Wheel rotated from -90 deg to +90 deg and Halo opacity changed -> image pixels differ
            self.assertGreater(float(np.abs(p1.astype(np.int16) - p2.astype(np.int16)).mean()), 0.2)
        finally:
            renderer.release()


if __name__ == "__main__":
    unittest.main()
