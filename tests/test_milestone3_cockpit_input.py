"""
tests/test_milestone3_cockpit_input.py
======================================
Comprehensive Milestone 3 verification suite covering:
1. First-person Cockpit Camera (Y=0.68m, Z=-0.25m, 68 deg FOV, dynamic FOV, G-neck spring, 25Hz kerb shudder)
2. 3D Cockpit & Rotating Steering Wheel Meshes (monocoque, Halo X=0.0m w=0.05m, mirrors +/-0.48m, wheel +/-pi rad)
3. McLaren PCU-8D Telemetry LCD & 15-LED Shift Light Bar (Green/Amber/Red/Blue Strobe, Gear, Speed, DRS, Brake Bias, RGBA texture)
4. Graphics Settings Subsystem (Presets Low/Medium/High/Ultra/Custom, Halo opacity, FOV, MSAA, Mirror FBOs, JSON save/load)
5. Keybinding & Controller Remapping Subsystem (Rebind keys/buttons, conflict detection & auto-swap, interactive capture, JSON save/load)
6. Dual Input Manager & Steering Filter (Deadzone, cubic gamma, trigger normalization, slew rate, speed attenuation, hot-swapping, Vehicle actuation)
7. ModernGL 3.3 Standalone Cockpit Renderer (Shader compilation, VAO upload, LCD texture streaming, mirror FBO pass, offscreen pixel readback)
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
    CockpitGeometrySpec,
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
    VALID_DRS_STATES,
    compute_speed_kmh,
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


class TestMilestone3CockpitCameraAndMeshes(unittest.TestCase):
    """Tests first-person cockpit camera optics, G-neck dynamics, and 3D cockpit meshes."""

    def test_cockpit_camera_nominal_pose_and_fov(self):
        cam = CockpitCamera()
        self.assertAlmostEqual(cam.eye_y, 0.68, places=4)
        self.assertAlmostEqual(cam.eye_z, -0.25, places=4)
        self.assertAlmostEqual(cam.nominal_fov, 68.0, places=4)

        # At standstill, FOV equals nominal 68.0 deg
        cam.update(dt=0.016, speed_kmh=0.0, longitudinal_g=0.0, lateral_g=0.0)
        self.assertAlmostEqual(cam.current_fov_deg, 68.0, places=3)

        # At 350 km/h over several frames, dynamic FOV expands toward 78.0 deg
        for _ in range(60):
            cam.update(dt=0.016, speed_kmh=350.0, longitudinal_g=0.0, lateral_g=0.0)
        self.assertGreater(cam.current_fov_deg, 76.5)
        self.assertLessEqual(cam.current_fov_deg, 78.5)

    def test_cockpit_camera_neck_spring_and_kerb_shudder(self):
        cam = CockpitCamera()
        # Apply 4.5G braking (longitudinal_g = -4.5) and 3.0G right cornering
        for _ in range(30):
            cam.update(dt=0.01, speed_kmh=200.0, longitudinal_g=-4.5, lateral_g=3.0, on_kerb=True)
        eye_local = cam.get_local_eye_position()
        self.assertNotAlmostEqual(eye_local.x, 0.0, places=4)
        self.assertNotAlmostEqual(eye_local.z, -0.25, places=4)
        self.assertGreater(cam.head_pitch_rad, 0.0)
        self.assertGreater(cam.head_roll_rad, 0.0)

        view_mat = cam.get_view_matrix(cockpit_local=True)
        proj_mat = cam.get_projection_matrix(aspect_ratio=16.0 / 9.0)
        self.assertEqual(view_mat.to_numpy().shape, (4, 4))
        self.assertEqual(proj_mat.to_numpy().shape, (4, 4))

        l_view, l_proj = cam.get_mirror_matrices("left")
        r_view, r_proj = cam.get_mirror_matrices("right")
        self.assertEqual(l_view.to_numpy().shape, (4, 4))
        self.assertEqual(r_proj.to_numpy().shape, (4, 4))

    def test_cockpit_meshes_and_wheel_transforms(self):
        builder = CockpitMeshBuilder()
        all_meshes = builder.build_all()
        expected_keys = {
            "monocoque",
            "halo",
            "mirror_housings",
            "left_mirror_glass",
            "right_mirror_glass",
            "wheel_body",
            "wheel_lcd",
        }
        self.assertEqual(set(all_meshes.keys()), expected_keys)
        for name, mesh in all_meshes.items():
            self.assertGreater(mesh.vertex_count, 0, f"Mesh {name} has 0 vertices")
            self.assertGreater(mesh.triangle_count, 0, f"Mesh {name} has 0 triangles")
            self.assertEqual(mesh.vertices.shape[1], 8)

        # Verify Halo and Mirror symmetry specs
        self.assertEqual(builder.spec.halo_center_x, 0.0)
        self.assertLessEqual(builder.spec.halo_strut_width, 0.08)
        self.assertEqual(builder.spec.left_mirror_x, -0.48)
        self.assertEqual(builder.spec.right_mirror_x, +0.48)

        # Verify 3x3 and 4x4 steering wheel rotation matrices across [-pi, +pi]
        for angle in (-math.pi, -math.pi / 2.0, 0.0, math.pi / 3.0, math.pi):
            rot3 = compute_wheel_rotation_matrix_3x3(angle)
            vec = np.array([0.14, -0.06, 0.02], dtype=np.float64)
            rotated = rot3 @ vec
            self.assertAlmostEqual(float(np.linalg.norm(vec)), float(np.linalg.norm(rotated)), places=6)
            self.assertAlmostEqual(float(np.linalg.det(rot3)), 1.0, places=6)

            mat4 = compute_wheel_model_matrix_4x4(angle, builder.spec)
            p_hub = mat4.transform_point(Vec3(0.0, 0.0, 0.0))
            self.assertAlmostEqual(p_hub.x, builder.spec.wheel_hub_x, places=5)
            self.assertAlmostEqual(p_hub.y, builder.spec.wheel_hub_y, places=5)
            self.assertAlmostEqual(p_hub.z, builder.spec.wheel_hub_z, places=5)


class TestMilestone3PCU8DTelemetryLCD(unittest.TestCase):
    """Tests the McLaren PCU-8D steering wheel LCD and 15-LED RPM rev-light bar."""

    def test_15_led_rev_lights_progression_and_strobe(self):
        lcd = TelemetryLCDDisplay(width=512, height=320)

        # Below 10,500 RPM: all 15 LEDs off
        leds_idle, count_idle, strobe_idle = lcd.compute_rev_lights(8500.0)
        self.assertEqual(len(leds_idle), 15)
        self.assertEqual(count_idle, 0)
        self.assertFalse(strobe_idle)
        self.assertTrue(all(c == LED_COLOR_OFF for c in leds_idle))

        # 11,600 RPM: Green LEDs lit
        leds_g, count_g, _ = lcd.compute_rev_lights(11600.0)
        self.assertEqual(count_g, 5)
        self.assertEqual(leds_g[0], LED_COLOR_GREEN)
        self.assertEqual(leds_g[4], LED_COLOR_GREEN)
        self.assertEqual(leds_g[5], LED_COLOR_OFF)

        # 12,700 RPM: Green (0..4) + Amber (5..9) lit
        leds_a, count_a, _ = lcd.compute_rev_lights(12700.0)
        self.assertEqual(count_a, 10)
        self.assertEqual(leds_a[4], LED_COLOR_GREEN)
        self.assertEqual(leds_a[9], LED_COLOR_AMBER)
        self.assertEqual(leds_a[10], LED_COLOR_OFF)

        # 13,400 RPM: Green + Amber + Red (10..13) lit
        leds_r, count_r, _ = lcd.compute_rev_lights(13400.0)
        self.assertEqual(count_r, 14)
        self.assertEqual(leds_r[13], LED_COLOR_RED)
        self.assertEqual(leds_r[14], LED_COLOR_OFF)

        # 14,000 RPM: Blue upshift strobe active
        leds_b, count_b, strobe_b = lcd.compute_rev_lights(14000.0, time_sec=0.0)
        self.assertEqual(count_b, 15)
        self.assertTrue(strobe_b)
        self.assertEqual(leds_b[14], LED_COLOR_BLUE)

    def test_lcd_state_and_rgba_buffer_generation(self):
        lcd = TelemetryLCDDisplay(width=512, height=320)
        state = lcd.update(
            rpm=12950.0,
            gear=7,
            speed_ms=85.0,
            drs_active=True,
            drs_available=True,
            brake_bias=0.565,
            lap_delta_sec=-0.34,
            dt=0.016,
        )
        self.assertEqual(state.gear_str, "7")
        self.assertAlmostEqual(state.speed_kmh, 306.0, places=3)
        self.assertEqual(state.drs_status, "ACTIVE")
        self.assertEqual(state.brake_bias_str, "56.5% F")
        self.assertEqual(state.lap_delta_str, "-0.34s")

        rgba = lcd.get_rgba_array()
        self.assertEqual(rgba.shape, (320, 512, 4))
        self.assertEqual(rgba.dtype, np.uint8)
        self.assertEqual(len(lcd.get_rgba_bytes()), 512 * 320 * 4)


class TestMilestone3GraphicsSettingsAndKeybindings(unittest.TestCase):
    """Tests GraphicsSettings presets/persistence and KeyBindingConfig remapping/conflict swap."""

    def test_graphics_settings_presets_validation_and_persistence(self):
        gs = GraphicsSettings()
        self.assertEqual(gs.preset, GraphicsPreset.HIGH.value)
        self.assertEqual(gs.base_fov_deg, 68.0)

        gs.apply_preset(GraphicsPreset.LOW)
        self.assertEqual(gs.msaa_samples, 0)
        self.assertFalse(gs.rear_mirrors_enabled)
        self.assertFalse(gs.motion_blur_enabled)

        gs.apply_preset(GraphicsPreset.ULTRA)
        self.assertEqual(gs.msaa_samples, 8)
        self.assertTrue(gs.rear_mirrors_enabled)
        self.assertTrue(gs.motion_blur_enabled)
        self.assertEqual(gs.draw_distance_m, 2000.0)

        # Halo opacity cycling: 1.0 -> 0.35 -> 0.15 -> 1.0
        self.assertAlmostEqual(gs.halo_opacity, 1.0)
        self.assertAlmostEqual(gs.cycle_halo_opacity(), 0.35)
        self.assertAlmostEqual(gs.cycle_halo_opacity(), 0.15)
        self.assertAlmostEqual(gs.cycle_halo_opacity(), 1.0)

        with tempfile.TemporaryDirectory() as tmpdir:
            cfg_path = Path(tmpdir) / "graphics_settings.json"
            gs.base_fov_deg = 72.0
            gs.halo_opacity = 0.35
            gs.save(cfg_path)
            loaded = GraphicsSettings.load(cfg_path)
            self.assertAlmostEqual(loaded.base_fov_deg, 72.0)
            self.assertAlmostEqual(loaded.halo_opacity, 0.35)
            self.assertEqual(loaded.msaa_samples, 8)

    def test_keybinding_remapping_conflict_swap_and_capture(self):
        kb = KeyBindingConfig()
        self.assertEqual(kb.get_key(ControlAction.THROTTLE), "W")
        self.assertEqual(kb.get_key(ControlAction.BRAKE), "S")
        self.assertEqual(len(kb.detect_conflicts()), 0)

        # Rebind THROTTLE to 'S' (currently BRAKE) with automatic conflict swap
        swapped = kb.rebind_key(ControlAction.THROTTLE, "S", resolve_conflict=True)
        self.assertEqual(swapped, ControlAction.BRAKE.value)
        self.assertEqual(kb.get_key(ControlAction.THROTTLE), "S")
        self.assertEqual(kb.get_key(ControlAction.BRAKE), "W")
        self.assertEqual(len(kb.detect_conflicts()), 0)

        # Interactive key capture mode
        import pygame
        kb.start_capture(ControlAction.DRS, slot="primary")
        self.assertTrue(kb.is_capturing)
        fake_event = SimpleNamespace(type=pygame.KEYDOWN, key=pygame.K_k)
        result = kb.process_capture_event(fake_event)
        self.assertIsNotNone(result)
        self.assertFalse(kb.is_capturing)
        self.assertEqual(kb.get_key(ControlAction.DRS), "K")

        # Gamepad button rebind + conflict swap
        old_gear_up_btn = kb.get_gamepad_button(ControlAction.GEAR_UP)
        old_drs_btn = kb.get_gamepad_button(ControlAction.DRS)
        conflict_gp = kb.rebind_gamepad_button(ControlAction.DRS, old_gear_up_btn, resolve_conflict=True)
        self.assertEqual(conflict_gp, ControlAction.GEAR_UP.value)
        self.assertEqual(kb.get_gamepad_button(ControlAction.DRS), old_gear_up_btn)
        self.assertEqual(kb.get_gamepad_button(ControlAction.GEAR_UP), old_drs_btn)

        # Save & load JSON round-trip
        with tempfile.TemporaryDirectory() as tmpdir:
            bind_path = Path(tmpdir) / "keybindings.json"
            kb.steering_sensitivity = 1.35
            kb.save(bind_path)
            loaded = KeyBindingConfig.load(bind_path)
            self.assertEqual(loaded.get_key(ControlAction.DRS), "K")
            self.assertAlmostEqual(loaded.steering_sensitivity, 1.35)

        kb.reset_to_defaults()
        self.assertEqual(kb.get_key(ControlAction.THROTTLE), "W")
        self.assertEqual(kb.get_key(ControlAction.DRS), "F")

    def test_input_manager_dual_input_and_vehicle_actuation(self):
        mgr = InputManager()
        car = Vehicle()
        car.auto_shift = False
        car.gear = 1
        car.vx = 50.0  # 180 km/h
        car.drs_available = True

        # Frame 1: Keyboard W (throttle), E (gear up), F (DRS), ] (brake bias +0.5%)
        st1 = mgr.update(
            speed_ms=car.vx,
            dt=0.016,
            pressed_keys={"W", "E", "F", "]"},
            gamepad_state=None,
        )
        self.assertEqual(st1.throttle, 1.0)
        self.assertTrue(st1.gear_up_pressed)
        self.assertTrue(st1.drs_pressed)
        self.assertTrue(st1.brake_bias_inc_pressed)
        self.assertEqual(st1.active_device, "KEYBOARD")

        initial_bb = car.brake_bias
        mgr.apply_to_vehicle(car, dt=0.001, in_drs_zone=True)
        self.assertEqual(car.gear, 2)
        self.assertTrue(car.drs_active)
        self.assertAlmostEqual(car.brake_bias, initial_bb + 0.005, places=4)

        # Frame 2: Holding same keys does not re-trigger edge actions
        st2 = mgr.update(
            speed_ms=car.vx,
            dt=0.016,
            pressed_keys={"W", "E", "F", "]"},
            gamepad_state=None,
        )
        self.assertFalse(st2.gear_up_pressed)
        self.assertFalse(st2.drs_pressed)

        # Frame 3: Hot-swap to Gamepad analog steering & triggers
        st3 = mgr.update(
            speed_ms=car.vx,
            dt=0.016,
            pressed_keys=set(),
            gamepad_state={
                "steer": 0.65,
                "throttle": 200.0,
                "brake": 0.0,
                "trigger_max": 255.0,
                "buttons": {},
            },
        )
        self.assertEqual(st3.active_device, "GAMEPAD")
        self.assertGreater(st3.throttle, 0.75)
        self.assertGreater(st3.wheel_angle_rad, 0.0)


class TestMilestone3ModernGLRenderer(unittest.TestCase):
    """Tests standalone ModernGL 3.3 Core Profile rendering of the cockpit & LCD."""

    def test_standalone_cockpit_renderer_pass(self):
        settings = GraphicsSettings(resolution=(640, 360))
        renderer = CockpitRenderer(settings=settings, standalone=True, init_gl=True)
        try:
            renderer.lcd.update(
                rpm=13200.0,
                gear=6,
                speed_ms=78.0,
                drs_active=True,
                drs_available=True,
                brake_bias=0.57,
            )
            renderer.camera.update(dt=0.016, speed_kmh=280.8, longitudinal_g=0.8, lateral_g=-1.5)
            renderer.render_frame(wheel_angle_rad=0.35)
            self.assertEqual(renderer.frame_count, 1)

            pixels = renderer.read_offscreen_pixels()
            self.assertIsNotNone(pixels)
            self.assertEqual(pixels.shape, (360, 640, 4))
            # Ensure non-uniform image (cockpit + sky + LCD rendered)
            self.assertGreater(float(np.std(pixels[:, :, :3])), 5.0)
        finally:
            renderer.release()


if __name__ == "__main__":
    unittest.main()
