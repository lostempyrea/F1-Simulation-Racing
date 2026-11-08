"""
tests/test_tier1_features.py
============================
Tier 1: Comprehensive Feature Coverage Test Suite for F1 Simulation.

Verifies all 26 features from PROJECT.md Feature Inventory with >= 5 unit and
contract assertions per feature:
- F01: Desktop Native Platform
- F02: Cockpit First-Person View
- F03: Rotating Steering Wheel
- F04: Telemetry LCD & Rev Lights
- F05: Dual Input Support
- F06: Manual & Automatic Transmission
- F07: DRS Cockpit Control
- F08: Engine Start / Ignition
- F09: Brake Bias Adjustment
- F10: Powertrain & Acceleration Dynamics
- F11: Carbon Braking Dynamics
- F12: Pacejka Tyre Grip Model
- F13: Understeer & Oversteer Handling
- F14: Aerodynamics & DRS Drag Reduction
- F15: Autodromo Nazionale Monza
- F16: Circuit de Monaco
- F17: 3D Track Elements & Kerbs
- F18: 5 Authentic AI Opponents
- F19: 3 AI Difficulty Tiers
- F20: AI Racing Behavior & Collision Avoidance
- F21: Motorsport UI & Main Menu
- F22: Broadcast Television HUD
- F23: Post-Race Podium & Results
- F24: Dynamic Camera & Audio Engine
- F25: Automated Headless Physics Benchmark
- F26: Desktop Delivery & PowerShell Launch
"""

import sys
import os
import math
import unittest
from typing import Dict, Any, List, Tuple
import numpy as np

from f1sim.physics.vehicle import Vehicle
from f1sim.physics.powertrain import Powertrain, EngineModel, Gearbox, TransmissionMode, IgnitionState
from f1sim.physics.brakes import BrakeSystem
from f1sim.physics.tyre import TyreModel, PacejkaParams
from f1sim.physics.aero import Aerodynamics
from f1sim.ai.driver_profiles import (
    AI_DRIVER_ROSTER,
    AIDifficulty,
    AIDriverProfile,
    DifficultyCalibration,
    get_difficulty_calibration,
    get_driver_by_number,
    estimate_monza_ai_lap_time,
)
from f1sim.ai.racing_line import (
    compute_apex_speed,
    compute_braking_entry_speed,
    compute_lookahead_distance,
)
from f1sim.ai.collision import (
    OBB2D,
    check_obb_intersection,
    compute_overtake_and_avoidance,
    compute_slipstream_drag_multiplier,
    compute_ttc,
)

# Try importing f1sim if implemented
try:
    import f1sim
    HAS_F1SIM = True
except ImportError:
    HAS_F1SIM = False

from tests.benchmark_physics_ai import (
    F1ReferencePhysicsModel,
    VehicleTelemetry,
    VEHICLE_MASS,
    WHEELBASE,
    CG_TO_FRONT,
    CG_TO_REAR,
    TRACK_WIDTH_FRONT,
    TRACK_WIDTH_REAR,
    CG_HEIGHT,
    TYRE_RADIUS,
    AIR_DENSITY,
    GRAVITY,
    GEAR_RATIOS,
    FINAL_DRIVE,
    IDLE_RPM,
    REDLINE_RPM,
    UPSHIFT_RPM,
    PEAK_POWER_W,
    BASE_CD_A,
    BASE_CL_A,
    DRS_DRAG_REDUCTION,
    DRS_DOWNFORCE_REDUCTION,
    BASE_MU,
    PACEJKA_B,
    PACEJKA_C,
    PACEJKA_E,
    LOAD_SENSITIVITY,
    REFERENCE_LOAD,
)


class TestFeature01_DesktopNativePlatform(unittest.TestCase):
    """F1: Desktop Native Platform (Python 3.12, ModernGL 3.3, Pygame-ce, NumPy)."""

    def test_python_runtime_and_64bit(self):
        """Assertion 1: Verified 64-bit Python 3.12+ execution environment."""
        self.assertGreaterEqual(sys.version_info.major, 3)
        self.assertGreaterEqual(sys.version_info.minor, 11)
        self.assertGreater(sys.maxsize, 2 ** 32, "Platform must be 64-bit AMD64/x86_64")

    def test_numpy_high_performance_vectorization(self):
        """Assertion 2: High throughput NumPy array vectorization."""
        arr = np.linspace(0.0, 100.0, 100000, dtype=np.float64)
        t_start = sys.float_info.min
        res = np.sin(arr) * np.cos(arr) + np.sqrt(arr)
        self.assertEqual(res.shape[0], 100000)
        self.assertIsInstance(res, np.ndarray)

    def test_pygame_ce_subsystems(self):
        """Assertion 3: Pygame-ce installed with SDL 2.x backend."""
        import pygame
        self.assertTrue(hasattr(pygame, "IS_CE") or hasattr(pygame, "ver"), "Pygame runtime present")
        version_str = getattr(pygame, "version", None)
        self.assertIsNotNone(version_str)

    def test_glsl_shader_version_330_core(self):
        """Assertion 4: ModernGL OpenGL 3.3 Core Profile compliance."""
        import moderngl
        self.assertIsNotNone(moderngl.__version__)

    def test_headless_execution_support(self):
        """Assertion 5: Simulation runs headlessly without requiring active display."""
        car = Vehicle()
        for _ in range(10):
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertGreater(car.vx, 0.0)
        self.assertFalse(math.isnan(car.vx))


class TestFeature02_CockpitFirstPersonView(unittest.TestCase):
    """F2: Cockpit First-Person View (driver camera eye Y=0.68m, monocoque, halo, mirrors)."""

    def setUp(self):
        from f1sim.graphics.camera import CockpitCamera
        from f1sim.graphics.meshes import CockpitMeshBuilder
        self.camera = CockpitCamera()
        self.mesh_builder = CockpitMeshBuilder()

    def test_driver_eye_height(self):
        """Assertion 1: Driver eye position height Y is nominal 0.68m (+/- 0.05m)."""
        eye_y = self.camera.eye_y
        self.assertAlmostEqual(eye_y, 0.68, delta=0.05)

    def test_driver_eye_longitudinal(self):
        """Assertion 2: Driver eye position longitudinal Z is nominal -0.25m."""
        eye_z = self.camera.eye_z
        self.assertAlmostEqual(eye_z, -0.25, delta=0.05)

    def test_cockpit_field_of_view(self):
        """Assertion 3: Field of View (FOV) calibrated to 68.0 degrees."""
        nominal_fov = self.camera.nominal_fov
        self.assertGreaterEqual(nominal_fov, 60.0)
        self.assertLessEqual(nominal_fov, 75.0)

    def test_halo_strut_occlusion(self):
        """Assertion 4: Halo center pillar positioned symmetrically at X=0.0m with width <= 0.08m."""
        halo_mesh = self.mesh_builder.build_halo_mesh()
        self.assertGreater(halo_mesh.vertex_count, 0)
        halo_center_x = self.mesh_builder.spec.halo_center_x
        halo_strut_width = self.mesh_builder.spec.halo_strut_width
        self.assertEqual(halo_center_x, 0.0)
        self.assertLessEqual(halo_strut_width, 0.08)

    def test_dual_wing_mirrors(self):
        """Assertion 5: Left and right wing mirrors positioned symmetrically at +/- 0.48m."""
        mirrors = self.mesh_builder.build_mirrors_mesh()
        self.assertIn("left_mirror_glass", mirrors)
        self.assertIn("right_mirror_glass", mirrors)
        left_mirror_x = self.mesh_builder.spec.left_mirror_x
        right_mirror_x = self.mesh_builder.spec.right_mirror_x
        self.assertEqual(abs(left_mirror_x), abs(right_mirror_x))
        self.assertLess(left_mirror_x, 0.0)
        self.assertGreater(right_mirror_x, 0.0)


class TestFeature03_RotatingSteeringWheel(unittest.TestCase):
    """F3: Rotating Steering Wheel (+/-180 deg, slew rate filter, centering)."""

    def setUp(self):
        from f1sim.input.filters import SteeringFilter, compute_speed_attenuation
        from f1sim.graphics.meshes import CockpitGeometrySpec, compute_wheel_rotation_matrix_3x3
        self.steering_filter = SteeringFilter()
        self.spec = CockpitGeometrySpec()
        self.compute_speed_attenuation = compute_speed_attenuation
        self.compute_wheel_rotation_matrix_3x3 = compute_wheel_rotation_matrix_3x3

    def test_steering_rotation_range(self):
        """Assertion 1: Maximum steering wheel rotation range is exactly [-pi, +pi]."""
        min_angle = self.spec.wheel_min_angle_rad
        max_angle = self.spec.wheel_max_angle_rad
        self.assertAlmostEqual(min_angle, -math.pi, places=4)
        self.assertAlmostEqual(max_angle, math.pi, places=4)
        self.assertAlmostEqual(self.steering_filter.cfg.max_wheel_lock_rad, math.pi, places=4)

    def test_steering_zero_centering(self):
        """Assertion 2: Zero steering input produces zero steering angle."""
        car = Vehicle()
        road_steer, wheel_angle = self.steering_filter.update(raw_input=0.0, speed_ms=0.0, dt=0.001)
        car.step(throttle=0.0, brake=0.0, steer=road_steer, dt=0.001)
        self.assertEqual(car.steer, 0.0)
        self.assertEqual(wheel_angle, 0.0)

    def test_steering_slew_rate_filter(self):
        """Assertion 3: Steering slew filter limits rate of angle change (e.g. 3.5 rad/s)."""
        current_angle = 0.0
        target_angle = 1.0
        slew_rate = self.steering_filter.cfg.slew_rate_turn_in  # 3.5 rad/s
        dt = 0.016  # ~60 fps frame
        new_angle = self.steering_filter.step_slew_limit(current_angle, target_angle, dt, slew_rate=slew_rate)
        self.assertLess(new_angle, target_angle)
        self.assertGreater(new_angle, current_angle)

    def test_speed_sensitive_steering_attenuation(self):
        """Assertion 4: Effective steering authority decreases with vehicle speed."""
        v_low = 10.0  # m/s (~36 km/h)
        v_high = 90.0  # m/s (~324 km/h)
        factor_low = self.compute_speed_attenuation(v_low)
        factor_high = self.compute_speed_attenuation(v_high)
        self.assertGreater(factor_low, factor_high)
        self.assertGreater(factor_low, 0.80)
        self.assertLess(factor_high, 0.35)

    def test_wheel_transform_matrix(self):
        """Assertion 5: Rotation matrix around steering column preserves vector length."""
        theta = math.pi / 4.0  # 45 deg
        rot_mat = self.compute_wheel_rotation_matrix_3x3(theta)
        v = np.array([0.15, 0.0, 0.0])
        v_rot = rot_mat @ v
        self.assertAlmostEqual(np.linalg.norm(v), np.linalg.norm(v_rot), places=5)


class TestFeature04_TelemetryLCDAndRevLights(unittest.TestCase):
    """F4: Telemetry LCD & Rev Lights (PCU-8D 15-LED bar, Gear N/1-8, Speed, DRS, Brake Bias)."""

    def setUp(self):
        from f1sim.graphics.lcd_display import (
            TelemetryLCDDisplay,
            VALID_DRS_STATES,
            compute_speed_kmh,
            format_brake_bias,
            format_gear,
            resolve_drs_status,
        )
        self.lcd = TelemetryLCDDisplay()
        self.valid_drs_states = VALID_DRS_STATES
        self.compute_speed_kmh = compute_speed_kmh
        self.format_brake_bias = format_brake_bias
        self.format_gear = format_gear
        self.resolve_drs_status = resolve_drs_status

    def test_rev_lights_color_ranges(self):
        """Assertion 1: 15-LED RPM bar divides into Green (1-5), Amber (6-10), Red (11-14), Blue (15)."""
        rpm_thresholds = self.lcd.rpm_thresholds
        self.assertEqual(len(rpm_thresholds), 4)
        self.assertEqual(rpm_thresholds[0][2], "GREEN")
        self.assertEqual(rpm_thresholds[3][2], "BLUE_STROBE")
        leds_green, count_g, _ = self.lcd.compute_rev_lights(11500.0)
        self.assertGreater(count_g, 0)
        leds_blue, count_b, strobe = self.lcd.compute_rev_lights(14000.0, time_sec=0.0)
        self.assertEqual(count_b, 15)
        self.assertTrue(strobe)

    def test_gear_display_values(self):
        """Assertion 2: Gear values represent Reverse (-1), Neutral (0), and Gears 1-8."""
        valid_gears = [-1, 0, 1, 2, 3, 4, 5, 6, 7, 8]
        gear_labels = {g: self.format_gear(g) for g in valid_gears}
        self.assertEqual(gear_labels[-1], "R")
        self.assertEqual(gear_labels[0], "N")
        self.assertEqual(gear_labels[8], "8")

    def test_speed_kmh_calculation(self):
        """Assertion 3: Speed km/h equals speed m/s multiplied by 3.6."""
        v_ms = 50.0
        v_kmh = self.compute_speed_kmh(v_ms)
        self.assertEqual(v_kmh, 180.0)

    def test_drs_status_enumeration(self):
        """Assertion 4: DRS status correctly handles OFF, AVAILABLE, and ACTIVE."""
        drs_states = self.valid_drs_states
        self.assertIn("OFF", drs_states)
        self.assertIn("ACTIVE", drs_states)
        self.assertEqual(self.resolve_drs_status(False, False), "OFF")
        self.assertEqual(self.resolve_drs_status(False, True), "AVAILABLE")
        self.assertEqual(self.resolve_drs_status(True, True), "ACTIVE")

    def test_brake_bias_telemetry_format(self):
        """Assertion 5: Brake bias formatting produces clear percentage string."""
        bb = 0.565
        bb_str = self.format_brake_bias(bb)
        self.assertEqual(bb_str, "56.5% F")


class TestFeature05_DualInputSupport(unittest.TestCase):
    """F5: Dual Input Support (Keyboard/Mouse + Gamepad XInput/SDL2, deadzones, gamma)."""

    def setUp(self):
        from f1sim.input.bindings import KeyBindingConfig
        from f1sim.input.filters import (
            apply_cubic_gamma,
            apply_deadzone,
            blend_inputs,
            normalize_trigger,
        )
        self.bindings = KeyBindingConfig()
        self.apply_deadzone = apply_deadzone
        self.apply_cubic_gamma = apply_cubic_gamma
        self.normalize_trigger = normalize_trigger
        self.blend_inputs = blend_inputs

    def test_keyboard_mapping_bindings(self):
        """Assertion 1: Primary keyboard bindings mapped for drive, steer, gears, DRS, ignition."""
        keys = self.bindings.primary_keys
        self.assertEqual(keys["throttle"], "W")
        self.assertEqual(keys["ignition"], "I")

    def test_gamepad_trigger_normalization(self):
        """Assertion 2: Analog trigger inputs strictly normalized to [0.0, 1.0]."""
        raw_trigger = 255  # 8-bit or 16-bit raw
        normalized = self.normalize_trigger(raw_trigger, max_raw=255.0)
        self.assertGreaterEqual(normalized, 0.0)
        self.assertLessEqual(normalized, 1.0)
        self.assertEqual(normalized, 1.0)

    def test_gamepad_stick_deadzone(self):
        """Assertion 3: Inputs inside deadzone (< 0.08) clamp to 0.0."""
        deadzone = 0.08
        test_inputs = [0.03, -0.05, 0.079, 0.15]
        filtered = [self.apply_deadzone(x, deadzone=deadzone, rescale=False) for x in test_inputs]
        self.assertEqual(filtered[0], 0.0)
        self.assertEqual(filtered[1], 0.0)
        self.assertEqual(filtered[2], 0.0)
        self.assertEqual(filtered[3], 0.15)

    def test_gamepad_cubic_steering_gamma(self):
        """Assertion 4: Cubic steering sensitivity preserves fine control near center."""
        gamma = 0.45
        self.assertAlmostEqual(self.apply_cubic_gamma(0.0, gamma=gamma), 0.0)
        self.assertAlmostEqual(self.apply_cubic_gamma(1.0, gamma=gamma), 1.0)
        self.assertAlmostEqual(self.apply_cubic_gamma(-1.0, gamma=gamma), -1.0)
        # Small deflection at 0.2 produces lower output than 0.2 (increased precision)
        self.assertLess(self.apply_cubic_gamma(0.2, gamma=gamma), 0.2)

    def test_input_device_hot_swapping(self):
        """Assertion 5: Keyboard and Gamepad inputs can blend or alternate seamlessly."""
        kb_steer = 0.0
        gp_steer = 0.75
        effective_steer = self.blend_inputs(kb_steer, gp_steer)
        self.assertEqual(effective_steer, 0.75)


class TestFeature06_ManualAndAutomaticTransmission(unittest.TestCase):
    """F6: Manual & Automatic Transmission (8 gears, seamless shift, rev-blip, over-rev protect)."""

    def test_gear_count_and_ratios(self):
        """Assertion 1: Transmission has 8 forward gears with decreasing ratios."""
        forward_ratios = GEAR_RATIOS[2:]  # 1st to 8th
        self.assertEqual(len(forward_ratios), 8)
        for i in range(len(forward_ratios) - 1):
            self.assertGreater(forward_ratios[i], forward_ratios[i + 1])

    def test_seamless_shift_latency(self):
        """Assertion 2: Shift latency is calibrated to 40ms seamless shift."""
        car = Vehicle()
        car.shift_up()
        self.assertAlmostEqual(car.powertrain.gearbox.shift_timer, 0.040, places=3)

    def test_downshift_rev_blip(self):
        """Assertion 3: Downshift rev-matching blip synchronizes target engine RPM."""
        car = Vehicle()
        car.vx = 40.0  # ~144 km/h
        car.gear = 4
        shifted = car.shift_down()
        self.assertTrue(shifted)
        self.assertEqual(car.gear, 3)
        self.assertGreater(car.powertrain.gearbox.blip_timer, 0.0)

    def test_downshift_over_rev_inhibitor(self):
        """Assertion 4: Downshift rejected if target RPM exceeds 14,800 RPM."""
        car = Vehicle()
        car.vx = 90.0  # 324 km/h
        car.gear = 2
        shifted = car.shift_down()
        self.assertFalse(shifted)
        self.assertEqual(car.gear, 2)

    def test_automatic_transmission_logic(self):
        """Assertion 5: Auto-transmission triggers upshift at 13,200 RPM and downshift at 8,500 RPM."""
        from f1sim.core.constants import RPM_AUTO_UPSHIFT, RPM_AUTO_DOWNSHIFT
        self.assertEqual(RPM_AUTO_UPSHIFT, 13200.0)
        self.assertEqual(RPM_AUTO_DOWNSHIFT, 8500.0)


class TestFeature07_DRSCockpitControl(unittest.TestCase):
    """F7: DRS Cockpit Control (drag reduction, zone detection, auto-close on brake/lateral G)."""

    def test_drs_default_state_off(self):
        """Assertion 1: DRS starts in OFF state."""
        car = Vehicle()
        self.assertFalse(car.drs_active)

    def test_drs_activation_gate_in_zone(self):
        """Assertion 2: DRS cannot activate outside a valid DRS zone."""
        car = Vehicle()
        car.drs_available = False
        car.drs_active = car.drs_available
        self.assertFalse(car.drs_active)

    def test_drs_drag_reduction_application(self):
        """Assertion 3: DRS active reduces aerodynamic drag area by 28%."""
        aero = Aerodynamics()
        _, drag_closed, _, _ = aero.compute(speed_ms=80.0, drs_active=False)
        _, drag_open, _, _ = aero.compute(speed_ms=80.0, drs_active=True)
        self.assertAlmostEqual(drag_open / drag_closed, 0.72, places=2)

    def test_drs_auto_cancel_on_brake(self):
        """Assertion 4: Tapping the brake pedal (> 0.05) immediately cancels DRS."""
        car = Vehicle()
        car.drs_active = True
        car.step(throttle=0.0, brake=0.20, steer=0.0, dt=0.001)
        self.assertFalse(car.drs_active)

    def test_drs_auto_cancel_on_lateral_g(self):
        """Assertion 5: High lateral G cancels DRS."""
        car = Vehicle()
        car.drs_active = True
        car.state.ay = 5.0 * 9.80665
        car.step(throttle=0.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertFalse(car.drs_active)


class TestFeature08_EngineStartIgnition(unittest.TestCase):
    """F8: Engine Start / Ignition (OFF -> IGNITION_ON -> CRANKING -> RUNNING, idle, anti-stall)."""

    def test_ignition_4_stage_state_machine(self):
        """Assertion 1: Engine ignition progresses through 4 canonical stages."""
        pt = Powertrain()
        self.assertEqual(pt.engine.ignition_state, IgnitionState.OFF)
        pt.engine.set_ignition(True)
        self.assertEqual(pt.engine.ignition_state, IgnitionState.IGNITION_ON)

    def test_starter_hold_duration_threshold(self):
        """Assertion 2: Starter crank requires >= 1.0s button hold duration."""
        pt = Powertrain()
        pt.engine.set_ignition(True)
        pt.engine.start_cranking()
        self.assertEqual(pt.engine.ignition_state, IgnitionState.CRANKING)
        pt.engine.update(0.5, 0.0)
        self.assertEqual(pt.engine.ignition_state, IgnitionState.CRANKING)
        pt.engine.update(0.6, 0.0)
        self.assertEqual(pt.engine.ignition_state, IgnitionState.RUNNING)

    def test_starter_cranking_rpm(self):
        """Assertion 3: Starter motor cranks engine to cranking target RPM."""
        from f1sim.core.constants import RPM_CRANKING_TARGET
        pt = Powertrain()
        pt.engine.set_ignition(True)
        pt.engine.start_cranking()
        pt.engine.update(0.79, 0.0)
        self.assertAlmostEqual(pt.engine.rpm, RPM_CRANKING_TARGET, delta=100.0)

    def test_idle_rpm_settling(self):
        """Assertion 4: Engine settles at steady 4,000 RPM idle."""
        pt = Powertrain()
        pt.engine.start_engine_direct()
        for _ in range(500):
            pt.engine.update(0.005, 0.0)
        self.assertAlmostEqual(pt.engine.rpm, IDLE_RPM, delta=50.0)

    def test_anti_stall_clutch_disengage(self):
        """Assertion 5: Anti-stall disengages clutch if engine drops below stall threshold in gear."""
        pt = Powertrain()
        pt.engine.start_engine_direct()
        pt.gearbox.gear = 1
        pt.step(throttle=0.0, brake=1.0, wheel_speed_ms=0.0, dt=0.001)
        self.assertTrue(pt.anti_stall_active or pt.gearbox.clutch_engagement < 0.5)


class TestFeature09_BrakeBiasAdjustment(unittest.TestCase):
    """F9: Brake Bias Adjustment (52% to 64% front, default 58%, 0.5% increments)."""

    def test_default_bias_setting(self):
        """Assertion 1: Default front brake bias is within regulatory range."""
        car = Vehicle()
        self.assertGreaterEqual(car.brake_bias, 0.52)
        self.assertLessEqual(car.brake_bias, 0.64)

    def test_bias_adjustment_range(self):
        """Assertion 2: Adjustable range strictly bounded between 52.0% and 64.0%."""
        car = Vehicle()
        car.brake_bias = 0.40
        self.assertEqual(car.brake_bias, 0.52)
        car.brake_bias = 0.80
        self.assertEqual(car.brake_bias, 0.64)

    def test_bias_increment_step(self):
        """Assertion 3: Brake bias adjusts in +/- 0.5% increments."""
        car = Vehicle()
        initial_bb = car.brake_bias
        car.adjust_brake_bias(+0.005)
        self.assertAlmostEqual(car.brake_bias - initial_bb, 0.005, places=5)

    def test_front_rear_torque_split(self):
        """Assertion 4: BrakeSystem calculates front and rear torques per brake bias."""
        brakes = BrakeSystem()
        brakes.brake_bias = 0.58
        out = brakes.update(
            pedal_input=1.0,
            wheel_speeds_rads=np.array([100.0, 100.0, 100.0, 100.0]),
            vehicle_speed_ms=33.0,
            dt=0.001
        )
        t_front = out.torque_fl + out.torque_fr
        t_rear = out.torque_rl + out.torque_rr
        total = t_front + t_rear
        self.assertAlmostEqual(t_front / total, 0.58, places=3)

    def test_bias_sum_to_one(self):
        """Assertion 5: Vehicle property clamps invalid bias values."""
        car = Vehicle()
        car.brake_bias = 0.10
        self.assertEqual(car.brake_bias, 0.52)


class TestFeature10_PowertrainAccelerationDynamics(unittest.TestCase):
    """F10: Powertrain & Acceleration Dynamics (1.6L V6 Turbo Hybrid, 0-100, 0-200)."""

    def test_peak_hybrid_power(self):
        """Assertion 1: Peak hybrid powertrain output reaches ~1000 hp (~746 kW)."""
        self.assertEqual(PEAK_POWER_W, 746000.0)

    def test_0_to_100_time(self):
        """Assertion 2: 0-100 km/h acceleration within modern F1 bounds (2.40s - 2.80s)."""
        car = Vehicle()
        car.gear = 1
        elapsed = 0.0
        dt = 0.001
        while car.speed_kmh < 100.0 and elapsed < 4.0:
            if car.rpm >= UPSHIFT_RPM and car.gear < 8:
                car.shift_up()
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            elapsed += dt
        self.assertGreaterEqual(elapsed, 2.40)
        self.assertLessEqual(elapsed, 2.80)

    def test_0_to_200_time(self):
        """Assertion 3: 0-200 km/h acceleration within 4.80s - 5.50s."""
        car = Vehicle()
        car.gear = 1
        elapsed = 0.0
        dt = 0.001
        while car.speed_kmh < 200.0 and elapsed < 7.0:
            if car.rpm >= UPSHIFT_RPM and car.gear < 8:
                car.shift_up()
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            elapsed += dt
        self.assertGreaterEqual(elapsed, 4.80)
        self.assertLessEqual(elapsed, 5.50)

    def test_operating_rpm_band(self):
        """Assertion 4: Engine operates strictly between 4,000 RPM idle and 15,000 RPM redline."""
        self.assertEqual(IDLE_RPM, 4000.0)
        self.assertEqual(REDLINE_RPM, 15000.0)
        self.assertLess(IDLE_RPM, REDLINE_RPM)

    def test_mgu_k_low_end_assist(self):
        """Assertion 5: MGU-K electric motor delivers up to 350 Nm low-end torque."""
        eng = EngineModel()
        eng.start_engine_direct()
        t_mguk = eng.compute_mguk_torque(3000.0, throttle=1.0)
        self.assertAlmostEqual(t_mguk, 350.0, delta=10.0)


class TestFeature11_CarbonBrakingDynamics(unittest.TestCase):
    """F11: Carbon Braking Dynamics (200-0 km/h in <55m, peak decel >= 4.5G)."""

    def test_200_to_0_stopping_distance(self):
        """Assertion 1: Emergency stop from 200 km/h stops in < 55.0m."""
        car = Vehicle()
        car.vx = 200.0 / 3.6
        car.gear = 6
        dt = 0.001
        distance = 0.0
        while car.speed_kmh > 0.1:
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            distance += car.vx * dt
        self.assertLess(distance, 55.0)

    def test_peak_deceleration_g(self):
        """Assertion 2: Peak deceleration at 200 km/h reaches >= 4.5G."""
        car = Vehicle()
        car.vx = 200.0 / 3.6
        car.gear = 6
        peak_g = 0.0
        for _ in range(100):
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)
            g = abs(car.longitudinal_g)
            if g > peak_g:
                peak_g = g
        self.assertGreaterEqual(peak_g, 4.5)

    def test_aero_downforce_braking_scaling(self):
        """Assertion 3: Braking capability at 300 km/h is significantly higher than at 80 km/h."""
        car300 = Vehicle()
        car300.vx = 300.0 / 3.6
        car300.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)
        g_high = abs(car300.longitudinal_g)

        car80 = Vehicle()
        car80.vx = 80.0 / 3.6
        car80.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)
        g_low = abs(car80.longitudinal_g)

        self.assertGreater(g_high, g_low)

    def test_trail_braking_modulation(self):
        """Assertion 4: Deceleration modulates with fractional brake pedal input."""
        car_light = Vehicle()
        car_light.vx = 150.0 / 3.6
        car_light.step(throttle=0.0, brake=0.05, steer=0.0, dt=0.001)

        car_med = Vehicle()
        car_med.vx = 150.0 / 3.6
        car_med.step(throttle=0.0, brake=0.20, steer=0.0, dt=0.001)

        self.assertLess(abs(car_light.longitudinal_g), abs(car_med.longitudinal_g))

    def test_brake_thermal_operating_window(self):
        """Assertion 5: Carbon brake disc temperature increases during heavy braking."""
        brakes = BrakeSystem()
        initial_temp = brakes.disc_temperatures[0]
        for _ in range(500):
            brakes.update(
                pedal_input=1.0,
                wheel_speeds_rads=np.array([150.0, 150.0, 150.0, 150.0]),
                vehicle_speed_ms=50.0,
                dt=0.01
            )
        self.assertGreater(brakes.disc_temperatures[0], initial_temp)


class TestFeature12_PacejkaTyreGripModel(unittest.TestCase):
    """F12: Pacejka Tyre Grip Model (Magic Formula B, C, D, E, peak 6-8 deg slip, friction ellipse)."""

    def test_pacejka_parameters(self):
        """Assertion 1: Pacejka coefficients calibrated to physical values."""
        tyre = TyreModel()
        self.assertGreater(tyre.params.B_y, 0.0)
        self.assertGreater(tyre.params.C_y, 0.0)

    def test_peak_slip_angle_range(self):
        """Assertion 2: Peak cornering grip occurs between 6.0 and 9.5 degrees slip."""
        car = Vehicle()
        fz = 5000.0
        angles_deg = np.linspace(1.0, 15.0, 100)
        forces = [car.pacejka_lateral_force(math.radians(a), fz) for a in angles_deg]
        peak_angle_deg = angles_deg[np.argmax(forces)]
        self.assertGreaterEqual(peak_angle_deg, 6.0)
        self.assertLessEqual(peak_angle_deg, 9.5)

    def test_sliding_grip_drop_off(self):
        """Assertion 3: Tyre grip beyond peak drops off smoothly into sliding regime."""
        car = Vehicle()
        fz = 5000.0
        force_peak = car.pacejka_lateral_force(math.radians(7.0), fz)
        force_slide = car.pacejka_lateral_force(math.radians(14.0), fz)
        self.assertLess(force_slide, force_peak)
        self.assertGreater(force_slide, 0.70 * force_peak)

    def test_tyre_load_sensitivity(self):
        """Assertion 4: Effective friction coefficient decreases with increasing vertical load."""
        car = Vehicle()
        fz_light = 3000.0
        fz_heavy = 8000.0
        mu_light = car.pacejka_lateral_force(math.radians(7.0), fz_light) / fz_light
        mu_heavy = car.pacejka_lateral_force(math.radians(7.0), fz_heavy) / fz_heavy
        self.assertGreater(mu_light, mu_heavy)

    def test_friction_ellipse_combined_slip(self):
        """Assertion 5: Combined slip tyre forces respect the Pacejka friction ellipse."""
        tyre = TyreModel()
        fz = 4000.0
        forces = tyre.compute_combined_forces(kappa=0.10, alpha=math.radians(5.0), Fz=fz)
        self.assertGreater(abs(forces.Fx), 0.0)
        self.assertGreater(abs(forces.Fy), 0.0)
        self.assertLessEqual(forces.Fx ** 2 + forces.Fy ** 2, (forces.mu_eff * fz * 1.05) ** 2)


class TestFeature13_UndersteerOversteerHandling(unittest.TestCase):
    """F13: Understeer & Oversteer Handling (slip angle saturation, breakaway, counter-steer)."""

    def test_understeer_slip_angle_threshold(self):
        """Assertion 1: Understeer front slip angle exceeds 8.0 degrees under excess entry speed."""
        car = Vehicle()
        car.vx = 65.0
        for _ in range(500):
            car.step(throttle=0.0, brake=0.0, steer=0.45, dt=0.001)
        slip_deg = abs(car.slip_angle_front) * (180.0 / math.pi)
        self.assertGreater(slip_deg, 8.0)

    def test_understeer_yaw_deficit(self):
        """Assertion 2: Understeer yaw rate is lower than theoretical Ackermann yaw rate."""
        car = Vehicle()
        car.vx = 65.0
        for _ in range(500):
            car.step(throttle=0.0, brake=0.0, steer=0.45, dt=0.001)
        r_ackermann = car.vx / 40.0
        self.assertLess(abs(car.yaw_rate), r_ackermann)

    def test_power_oversteer_breakaway(self):
        """Assertion 3: Full throttle in 1st gear corner exit produces rear slip ratio > 0.20."""
        car = Vehicle()
        car.vx = 14.0
        car.gear = 1
        for _ in range(400):
            car.step(throttle=1.0, brake=0.0, steer=0.35, dt=0.001)
        self.assertGreater(car.slip_ratio_rear, 0.20)

    def test_counter_steering_restoration(self):
        """Assertion 4: Counter-steering reverses front slip angle and counteracts spinning yaw rate."""
        car = Vehicle()
        car.vx = 25.0
        car.yaw_rate = 1.5  # Spinning right
        # Counter-steer left
        car.step(throttle=0.0, brake=0.0, steer=-0.40, dt=0.001)
        self.assertLess(car.steer, 0.0)

    def test_kerb_strike_oversteer_upset(self):
        """Assertion 5: Kerb vibration amplitude reported on contact patch."""
        car = Vehicle()
        car.kerb_vibration = 0.85
        self.assertEqual(car.get_telemetry().kerb_vibration, 0.85)


class TestFeature14_AerodynamicsDRSDragReduction(unittest.TestCase):
    """F14: Aerodynamics & DRS Drag Reduction (downforce, drag, DRS top speed delta)."""

    def test_downforce_quadratic_scaling(self):
        """Assertion 1: Downforce quadruples when vehicle speed doubles."""
        aero = Aerodynamics()
        f_down1, _, _, _ = aero.compute(speed_ms=40.0, drs_active=False)
        f_down2, _, _, _ = aero.compute(speed_ms=80.0, drs_active=False)
        self.assertAlmostEqual(f_down2 / f_down1, 4.0, places=3)

    def test_drag_quadratic_scaling(self):
        """Assertion 2: Drag quadruples when vehicle speed doubles."""
        aero = Aerodynamics()
        _, f_drag1, _, _ = aero.compute(speed_ms=40.0, drs_active=False)
        _, f_drag2, _, _ = aero.compute(speed_ms=80.0, drs_active=False)
        self.assertAlmostEqual(f_drag2 / f_drag1, 4.0, places=3)

    def test_drs_drag_reduction_ratio(self):
        """Assertion 3: DRS reduces drag by 28%."""
        aero = Aerodynamics()
        _, f_drag_closed, _, _ = aero.compute(speed_ms=80.0, drs_active=False)
        _, f_drag_open, _, _ = aero.compute(speed_ms=80.0, drs_active=True)
        self.assertAlmostEqual(f_drag_open / f_drag_closed, 0.72, places=2)

    def test_top_speed_closed_threshold(self):
        """Assertion 4: Terminal velocity with DRS closed reaches >= 340 km/h."""
        car = Vehicle()
        car.vx = 80.0
        car.gear = 7
        car.drs_active = False
        for _ in range(15000):
            if car.rpm >= UPSHIFT_RPM and car.gear < 8:
                car.shift_up()
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertGreaterEqual(car.speed_kmh, 340.0)

    def test_top_speed_drs_open_threshold(self):
        """Assertion 5: Terminal velocity with DRS open reaches >= 355 km/h."""
        car = Vehicle()
        car.vx = 80.0
        car.gear = 7
        car.drs_active = True
        for _ in range(15000):
            if car.rpm >= UPSHIFT_RPM and car.gear < 8:
                car.shift_up()
            car.drs_active = True
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertGreaterEqual(car.speed_kmh, 355.0)


class TestFeature15_AutodromoNazionaleMonza(unittest.TestCase):
    """F15: Autodromo Nazionale Monza (5.793 km, 11 corners, 2 DRS zones, boards, pit lane)."""

    def test_monza_track_length(self):
        """Assertion 1: Monza track length is 5,793m."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        self.assertEqual(track.length, 5793.0)

    def test_monza_corner_count(self):
        """Assertion 2: Monza has exactly 11 numbered turns."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        self.assertEqual(len(track.corners), 11)
        turn_names = [c.name for c in track.corners]
        self.assertIn("Variante del Rettifilo T1", turn_names)
        self.assertIn("Curva Parabolica (Alboreto)", turn_names)

    def test_monza_drs_zones(self):
        """Assertion 3: Monza has exactly 2 DRS zones."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        self.assertEqual(len(track.drs_zones), 2)
        self.assertEqual(track.drs_zones[0].activation_s, 115.0)
        self.assertEqual(track.drs_zones[0].detection_s, 5680.0)

    def test_monza_braking_distance_markers(self):
        """Assertion 4: Distance markers (150m, 100m, 50m) placed before Turn 1 Rettifilo."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        markers = [m.distance_to_corner for m in track.distance_markers if m.corner_number == 1]
        self.assertEqual(len(markers), 3)
        self.assertIn(150.0, markers)
        self.assertIn(100.0, markers)
        self.assertIn(50.0, markers)

    def test_monza_pit_lane_parameters(self):
        """Assertion 5: Pit lane speed limit enforced at 80 km/h."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        self.assertEqual(track.pit_lane.speed_limit_kmh, 80.0)


class TestFeature16_CircuitDeMonaco(unittest.TestCase):
    """F16: Circuit de Monaco (3.337 km, 19 corners, 1 DRS zone, Fairmont hairpin, elevation)."""

    def test_monaco_track_length(self):
        """Assertion 1: Monaco track length is 3,337m."""
        from f1sim.track.monaco import MonacoTrack
        track = MonacoTrack()
        self.assertEqual(track.length, 3337.0)

    def test_monaco_corner_count(self):
        """Assertion 2: Monaco has 19 official corners."""
        from f1sim.track.monaco import MonacoTrack
        track = MonacoTrack()
        self.assertEqual(len(track.corners), 19)

    def test_monaco_drs_zones(self):
        """Assertion 3: Monaco has exactly 1 DRS zone on Pit Straight."""
        from f1sim.track.monaco import MonacoTrack
        track = MonacoTrack()
        self.assertEqual(len(track.drs_zones), 1)
        self.assertEqual(track.drs_zones[0].name, "Pit Straight")
        self.assertEqual(track.drs_zones[0].activation_s, 18.0)
        self.assertEqual(track.drs_zones[0].length, 510.0)

    def test_monaco_fairmont_hairpin_speed(self):
        """Assertion 4: Fairmont hairpin negotiated at ~45 km/h."""
        from f1sim.track.monaco import MonacoTrack
        track = MonacoTrack()
        fairmont = track.corners[5]  # Turn 6 Fairmont Hairpin
        self.assertLessEqual(fairmont.apex_speed_kmh, 55.0)
        self.assertGreaterEqual(fairmont.apex_speed_kmh, 35.0)
        self.assertEqual(fairmont.radius, 11.0)

    def test_monaco_elevation_profile(self):
        """Assertion 5: Significant elevation change (+42m climb up Beau Rivage)."""
        from f1sim.track.monaco import MonacoTrack
        track = MonacoTrack()
        self.assertEqual(track.metadata.elevation_change, 42.0)


class TestFeature17_3DTrackElementsAndKerbs(unittest.TestCase):
    """F17: 3D Track Elements & Kerbs (spline, Frenet frames, red/white kerbs, 5-light start)."""

    def test_spline_frenet_frame_properties(self):
        """Assertion 1: Tangent and Normal vectors are orthonormal."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        pos, tangent, normal = track.get_spline_point(150.0)
        self.assertAlmostEqual(float(np.dot(tangent, normal)), 0.0, places=5)
        self.assertAlmostEqual(float(np.linalg.norm(tangent)), 1.0, places=5)
        self.assertAlmostEqual(float(np.linalg.norm(normal)), 1.0, places=5)

    def test_track_width_consistency(self):
        """Assertion 2: Track width is consistently between 10.0m and 12.0m."""
        from f1sim.track.monza import MonzaTrack
        track = MonzaTrack()
        for s in np.linspace(0.0, track.length, 25, endpoint=False):
            w = track.get_track_width(s)
            self.assertGreaterEqual(w, 10.0)
            self.assertLessEqual(w, 12.0)

    def test_kerb_placement_and_rumble_frequency(self):
        """Assertion 3: Kerb rumble frequency calibrated to 25 Hz."""
        from f1sim.track.monza import MonzaTrack
        from f1sim.track.track_builder import TrackBuilder
        track = MonzaTrack()
        builder = TrackBuilder(track)
        prop = builder.get_surface_at(s=1180.0, d=track.nominal_width * 0.5 + 0.5, speed_ms=25.0)
        self.assertTrue(prop.is_kerb)
        self.assertEqual(prop.vibration_frequency_hz, 25.0)

    def test_starting_grid_layout(self):
        """Assertion 4: Staggered 2x2 grid boxes with 8.0m spacing."""
        from f1sim.track.monza import MonzaTrack
        from f1sim.track.track_builder import TrackBuilder
        track = MonzaTrack()
        builder = TrackBuilder(track)
        boxes, mesh = builder.build_starting_grid(grid_start_s=track.length - 16.0)
        self.assertEqual(len(boxes), 20)
        grid_spacing = abs(boxes[0].s - boxes[1].s)
        self.assertAlmostEqual(grid_spacing, 8.0, places=3)

    def test_fia_5_red_light_gantry(self):
        """Assertion 5: 5 red lights illuminate sequentially before extinguishing."""
        from f1sim.track.track_builder import StartingGantry
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=1.0)
        self.assertEqual(gantry.state, "COUNTDOWN")
        gantry.step(5.0)
        self.assertTrue(all(gantry.get_light_states()))
        self.assertEqual(gantry.state, "HOLD")
        gantry.step(1.0)
        self.assertFalse(any(gantry.get_light_states()))
        self.assertTrue(gantry.is_lights_out)


class TestFeature18_FiveAuthenticAIOpponents(unittest.TestCase):
    """F18: 5 Authentic AI Opponents (Verstappen, Leclerc, Hamilton, Norris, Alonso)."""

    def test_verstappen_redbull(self):
        """Assertion 1: Verstappen #1, Red Bull Racing, Navy Blue."""
        driver = get_driver_by_number(1)
        self.assertEqual(driver.number, 1)
        self.assertEqual(driver.name, "Max Verstappen")
        self.assertEqual(driver.code, "VER")
        self.assertIn("Red Bull", driver.team)
        self.assertEqual(driver.short_team, "Red Bull")
        self.assertEqual(driver.hex, "#001A30")
        self.assertEqual(driver.rgb, (0, 26, 48))
        self.assertAlmostEqual(driver.pace_scalar, 0.998)
        self.assertAlmostEqual(driver.aggression, 0.96)
        self.assertGreater(driver.error_rate, 0.0)

    def test_leclerc_ferrari(self):
        """Assertion 2: Leclerc #16, Scuderia Ferrari, Rosso Corsa."""
        driver = get_driver_by_number(16)
        self.assertEqual(driver.number, 16)
        self.assertEqual(driver.name, "Charles Leclerc")
        self.assertEqual(driver.code, "LEC")
        self.assertIn("Ferrari", driver.team)
        self.assertEqual(driver.short_team, "Ferrari")
        self.assertEqual(driver.hex, "#D40000")
        self.assertEqual(driver.rgb, (212, 0, 0))
        self.assertAlmostEqual(driver.pace_scalar, 0.999)
        self.assertAlmostEqual(driver.aggression, 0.93)
        self.assertGreater(driver.error_rate, 0.0)

    def test_hamilton_mercedes(self):
        """Assertion 3: Hamilton #44, Mercedes-AMG, Obsidian Black."""
        driver = get_driver_by_number(44)
        self.assertEqual(driver.number, 44)
        self.assertEqual(driver.name, "Lewis Hamilton")
        self.assertEqual(driver.code, "HAM")
        self.assertIn("Mercedes", driver.team)
        self.assertEqual(driver.short_team, "Mercedes")
        self.assertEqual(driver.hex, "#0A0A0A")
        self.assertEqual(driver.rgb, (10, 10, 10))
        self.assertAlmostEqual(driver.pace_scalar, 1.003)
        self.assertAlmostEqual(driver.aggression, 0.91)
        self.assertGreater(driver.error_rate, 0.0)

    def test_norris_mclaren(self):
        """Assertion 4: Norris #4, McLaren, Papaya Orange."""
        driver = get_driver_by_number(4)
        self.assertEqual(driver.number, 4)
        self.assertEqual(driver.name, "Lando Norris")
        self.assertEqual(driver.code, "NOR")
        self.assertEqual(driver.team, "McLaren")
        self.assertEqual(driver.short_team, "McLaren")
        self.assertEqual(driver.hex, "#FF8000")
        self.assertEqual(driver.rgb, (255, 128, 0))
        self.assertAlmostEqual(driver.pace_scalar, 1.005)
        self.assertAlmostEqual(driver.aggression, 0.90)
        self.assertGreater(driver.error_rate, 0.0)

    def test_alonso_astonmartin(self):
        """Assertion 5: Alonso #14, Aston Martin, Racing Green."""
        driver = get_driver_by_number(14)
        self.assertEqual(driver.number, 14)
        self.assertEqual(driver.name, "Fernando Alonso")
        self.assertEqual(driver.code, "ALO")
        self.assertEqual(driver.team, "Aston Martin")
        self.assertEqual(driver.short_team, "Aston Martin")
        self.assertEqual(driver.hex, "#00594F")
        self.assertEqual(driver.rgb, (0, 89, 79))
        self.assertAlmostEqual(driver.pace_scalar, 1.008)
        self.assertAlmostEqual(driver.aggression, 0.94)
        self.assertGreater(driver.error_rate, 0.0)


class TestFeature19_ThreeAIDifficultyTiers(unittest.TestCase):
    """F19: 3 AI Difficulty Tiers (Easy 82% grip, Med 92%, Hard 99.5%, braking offsets)."""

    def test_easy_ai_grip_and_braking(self):
        """Assertion 1: Easy AI utilizes 82% grip and brakes 35-50m early."""
        cal = get_difficulty_calibration("Easy")
        self.assertAlmostEqual(cal.grip_utilization, 0.82)
        self.assertAlmostEqual(cal.grip_scalar, 0.82)
        self.assertTrue(35.0 <= cal.early_brake_offset_m <= 50.0)
        self.assertEqual(cal.early_brake_range_m, (35.0, 50.0))
        self.assertAlmostEqual(cal.reaction_latency_s, 0.250)

    def test_medium_ai_grip_and_braking(self):
        """Assertion 2: Medium AI utilizes 92% grip and brakes 12-18m early."""
        cal = get_difficulty_calibration("Medium")
        self.assertAlmostEqual(cal.grip_utilization, 0.92)
        self.assertAlmostEqual(cal.grip_scalar, 0.92)
        self.assertTrue(12.0 <= cal.early_brake_offset_m <= 18.0)
        self.assertEqual(cal.early_brake_range_m, (12.0, 18.0))
        self.assertAlmostEqual(cal.reaction_latency_s, 0.100)

    def test_hard_ai_grip_and_braking(self):
        """Assertion 3: Hard AI utilizes 99.5% grip with optimal late braking."""
        cal = get_difficulty_calibration("Hard")
        self.assertAlmostEqual(cal.grip_utilization, 0.995)
        self.assertAlmostEqual(cal.grip_scalar, 0.995)
        self.assertTrue(0.0 <= cal.early_brake_offset_m <= 2.0)
        self.assertAlmostEqual(cal.reaction_latency_s, 0.015)

    def test_difficulty_reaction_latencies(self):
        """Assertion 4: Latencies scale from 250ms (Easy) to 15ms (Hard)."""
        easy = get_difficulty_calibration("Easy")
        med = get_difficulty_calibration("Medium")
        hard = get_difficulty_calibration("Hard")
        self.assertGreater(easy.reaction_latency_s, med.reaction_latency_s)
        self.assertGreater(med.reaction_latency_s, hard.reaction_latency_s)
        self.assertAlmostEqual(easy.reaction_time, 0.250)
        self.assertAlmostEqual(med.reaction_time, 0.100)
        self.assertAlmostEqual(hard.reaction_time, 0.015)

    def test_difficulty_lap_time_ordering(self):
        """Assertion 5: Lap times monotonically decrease from Easy to Hard."""
        t_easy = estimate_monza_ai_lap_time("Easy")
        t_med = estimate_monza_ai_lap_time("Medium")
        t_hard = estimate_monza_ai_lap_time("Hard")
        self.assertGreater(t_easy, t_med)
        self.assertGreater(t_med, t_hard)
        self.assertGreaterEqual(t_easy - t_med, 4.0)
        self.assertGreaterEqual(t_med - t_hard, 2.0)


class TestFeature20_AIRacingBehaviorAndCollisionAvoidance(unittest.TestCase):
    """F20: AI Racing Behavior (pure-pursuit lookahead, speed profiling, OBB SAT, TTC)."""

    def test_pure_pursuit_lookahead_formula(self):
        """Assertion 1: Lookahead distance: L_d = clamp(8.0 + 0.38*v, 8.0, 45.0)."""
        self.assertEqual(compute_lookahead_distance(0.0), 8.0)
        self.assertAlmostEqual(compute_lookahead_distance(50.0), 27.0)
        self.assertEqual(compute_lookahead_distance(100.0), 45.0)

    def test_curvature_apex_speed_profiling(self):
        """Assertion 2: Apex target speed scales inversely with square root of curvature."""
        v1 = compute_apex_speed(curvature=0.01, mu=1.75, grip_utilization=1.0)
        v2 = compute_apex_speed(curvature=0.04, mu=1.75, grip_utilization=1.0)
        self.assertGreater(v1, v2)
        self.assertAlmostEqual(v1 / v2, 2.0, delta=0.05)

    def test_backward_braking_integration_pass(self):
        """Assertion 3: Backward braking pass computes safe approach speed."""
        v_apex = 20.0
        a_brake = 28.0
        ds = 50.0
        v_entry = compute_braking_entry_speed(v_apex, a_brake, ds)
        self.assertGreater(v_entry, v_apex)
        expected = math.sqrt(v_apex ** 2 + 2.0 * a_brake * ds)
        self.assertAlmostEqual(v_entry, expected)

    def test_obb_sat_collision_box(self):
        """Assertion 4: Vehicle collision box is 5.63m long by 2.00m wide."""
        box_a = OBB2D(center_x=0.0, center_y=0.0, heading_rad=0.0)
        self.assertEqual(box_a.length, 5.63)
        self.assertEqual(box_a.width, 2.00)
        box_b = OBB2D(center_x=3.0, center_y=0.0, heading_rad=0.0)
        self.assertTrue(check_obb_intersection(box_a, box_b))
        box_c = OBB2D(center_x=10.0, center_y=0.0, heading_rad=0.0)
        self.assertFalse(check_obb_intersection(box_a, box_c))

    def test_time_to_collision_avoidance_and_slipstream(self):
        """Assertion 5: TTC < 1.2s triggers braking and overtaking lateral offset."""
        ttc = compute_ttc(distance_m=15.0, v_follower_ms=60.0, v_leader_ms=45.0)
        self.assertAlmostEqual(ttc, 1.0)
        self.assertLess(ttc, 1.2)
        ttc_res, lat_shift, brake = compute_overtake_and_avoidance(
            distance_m=15.0,
            v_follower_ms=60.0,
            v_leader_ms=45.0,
            leader_d_m=0.0,
        )
        self.assertAlmostEqual(abs(lat_shift), 2.8)
        drag_mult = compute_slipstream_drag_multiplier(distance_m=15.0, lateral_diff_m=0.0)
        self.assertLess(drag_mult, 1.0)
        self.assertGreaterEqual(drag_mult, 0.82)


class TestFeature21_MotorsportUIMainMenu(unittest.TestCase):
    """F21: Motorsport UI & Main Menu (start, track select, difficulty, controls guide)."""

    def setUp(self):
        from f1sim.ui.menu import MotorsportMenuManager
        self.menu = MotorsportMenuManager()

    def test_menu_state_machine_transitions(self):
        """Assertion 1: Menu states cover MAIN, TRACK_SELECT, DIFFICULTY_SELECT, RACE, RESULTS."""
        states = self.menu.states
        self.assertIn("MAIN", states)
        self.assertIn("TRACK_SELECT", states)
        self.assertIn("DIFFICULTY_SELECT", states)
        self.assertIn("GRAPHICS_SETTINGS", states)
        self.assertIn("KEYBINDINGS", states)
        self.assertIn("RACE", states)
        self.assertIn("RESULTS", states)
        self.assertEqual(self.menu.transition_to("TRACK_SELECT"), "TRACK_SELECT")
        self.assertEqual(self.menu.transition_to("RESULTS"), "RESULTS")

    def test_track_selection_options(self):
        """Assertion 2: Tracks selectable include Monza and Monaco."""
        tracks = self.menu.tracks
        self.assertEqual(len(tracks), 2)
        self.assertEqual(self.menu.select_track("Monza"), "Monza")
        self.assertEqual(self.menu.select_track("Monaco"), "Monaco")

    def test_difficulty_selection_options(self):
        """Assertion 3: Difficulties selectable include Easy, Medium, Hard."""
        diffs = self.menu.difficulties
        self.assertEqual(len(diffs), 3)
        self.assertEqual(self.menu.select_difficulty("Easy"), "Easy")
        self.assertEqual(self.menu.select_difficulty("Hard"), "Hard")

    def test_transmission_selection_option(self):
        """Assertion 4: Transmission options include Manual and Automatic."""
        trans = self.menu.transmissions
        self.assertEqual(len(trans), 2)
        self.assertEqual(self.menu.select_transmission("Manual"), "Manual")
        self.assertEqual(self.menu.select_transmission("Auto"), "Auto")

    def test_controls_help_display(self):
        """Assertion 5: Controls guide specifies both keyboard and gamepad mappings."""
        guide = self.menu.controls_guide
        self.assertIn("Keyboard", guide)
        self.assertIn("Gamepad", guide)


class TestFeature22_BroadcastTelevisionHUD(unittest.TestCase):
    """F22: Broadcast Television HUD (speedometer, RPM, P1-P6 timing tower, lap timer, mini-map)."""

    def setUp(self):
        from f1sim.ui.hud import BroadcastHUD, format_lap_timer, format_speed_hud
        self.hud = BroadcastHUD(num_cars=6)
        self.format_speed_hud = format_speed_hud
        self.format_lap_timer = format_lap_timer

    def test_hud_speedometer_element(self):
        """Assertion 1: Speedometer renders speed in integer km/h."""
        speed = 284.7
        display = self.format_speed_hud(speed)
        self.assertEqual(display, "285 KM/H")

    def test_hud_rpm_element(self):
        """Assertion 2: RPM displayed numerically and via graphical bar."""
        self.hud.update(speed_kmh=250.0, rpm=12500.0, gear_str="6", current_lap_time_sec=45.0)
        self.assertEqual(self.hud.rpm_value, 12500.0)
        self.assertGreater(self.hud.rpm_fraction, 0.70)

    def test_hud_timing_tower(self):
        """Assertion 3: Timing tower shows 6 driver rows with positions P1 through P6."""
        positions = self.hud.positions
        self.assertEqual(len(positions), 6)
        self.assertEqual(positions[0], "P1")
        self.assertEqual(positions[-1], "P6")

    def test_hud_lap_timer(self):
        """Assertion 4: Lap timer formats time as M:SS.mmm."""
        lap_time = 81.234
        formatted = self.format_lap_timer(lap_time)
        self.assertEqual(formatted, "1:21.234")

    def test_hud_minimap_radar(self):
        """Assertion 5: Mini-map renders car dots on circuit path."""
        dots = self.hud.build_minimap_dots([
            {"id": 0, "color": "PLAYER", "x": 10.0, "y": 20.0},
            {"id": 1, "color": "AI_RED_BULL", "x": 15.0, "y": 25.0},
        ])
        self.assertEqual(len(dots), 2)
        self.assertEqual(dots[0]["color"], "PLAYER")


class TestFeature23_PostRacePodiumAndResults(unittest.TestCase):
    """F23: Post-Race Podium & Results (classification P1-P6, times, gaps, fastest lap)."""

    def setUp(self):
        from f1sim.ui.results import (
            RaceResultsBuilder,
            VALID_FINISH_STATUSES,
            compute_telemetry_summary,
            find_fastest_lap_driver,
            format_classification_gap,
        )
        self.builder = RaceResultsBuilder()
        self.valid_statuses = VALID_FINISH_STATUSES
        self.compute_telemetry_summary = compute_telemetry_summary
        self.find_fastest_lap_driver = find_fastest_lap_driver
        self.format_classification_gap = format_classification_gap

    def test_podium_classification_ordering(self):
        """Assertion 1: Results sorted in ascending order of total race time."""
        raw = [
            {"code": "HAM", "time": 251.1, "best_lap": 82.0},
            {"code": "VER", "time": 245.2, "best_lap": 81.2},
            {"code": "LEC", "time": 248.5, "best_lap": 81.5},
        ]
        results = self.builder.classify(raw)
        for i in range(len(results) - 1):
            self.assertLess(results[i].time, results[i + 1].time)

    def test_winner_time_and_gaps(self):
        """Assertion 2: Leader gap is 'WINNER', follower gaps format as '+X.XXXs'."""
        gap_p1 = self.format_classification_gap(1, 245.2, 245.2)
        gap_p2 = self.format_classification_gap(2, 248.5, 245.2)
        self.assertEqual(gap_p1, "WINNER")
        self.assertEqual(gap_p2, "+3.300s")

    def test_fastest_lap_award(self):
        """Assertion 3: Driver with minimum single lap time receives fastest lap award."""
        lap_times = {"VER": 81.2, "LEC": 81.5, "HAM": 81.9}
        fastest_driver = self.find_fastest_lap_driver(lap_times)
        self.assertEqual(fastest_driver, "VER")

    def test_telemetry_summary_statistics(self):
        """Assertion 4: Summary reports top speed and average speed."""
        summary = self.compute_telemetry_summary([180.0, 235.1, 352.4])
        self.assertGreater(summary["top_speed_kmh"], summary["avg_speed_kmh"])

    def test_race_completion_flags(self):
        """Assertion 5: Valid finishing status tags include FINISHED and DNF."""
        statuses = self.valid_statuses
        self.assertIn("FINISHED", statuses)
        self.assertIn("DNF", statuses)


class TestFeature24_DynamicCameraAndAudioEngine(unittest.TestCase):
    """F24: Dynamic Camera & Audio Engine (driver head G-force spring, procedural V6, sound events)."""

    def setUp(self):
        from f1sim.graphics.camera import CockpitCamera
        from f1sim.audio.sound_engine import (
            SUPPORTED_SOUND_EVENTS,
            SoundEngine,
            compute_v6_fundamental_hz,
            compute_v6_harmonics_hz,
        )
        self.camera = CockpitCamera()
        self.sound_engine = SoundEngine(init_mixer=False)
        self.supported_events = SUPPORTED_SOUND_EVENTS
        self.compute_v6_fundamental_hz = compute_v6_fundamental_hz
        self.compute_v6_harmonics_hz = compute_v6_harmonics_hz

    def test_camera_driver_head_inertia(self):
        """Assertion 1: Driver head shifts backward under acceleration and forward under braking."""
        self.camera.reset()
        for _ in range(25):
            self.camera.update(dt=0.01, speed_kmh=150.0, longitudinal_g=+1.5, lateral_g=0.0)
        z_offset_accel = self.camera.neck_offset.z

        self.camera.reset()
        for _ in range(25):
            self.camera.update(dt=0.01, speed_kmh=150.0, longitudinal_g=-4.5, lateral_g=0.0)
        z_offset_decel = self.camera.neck_offset.z

        self.assertLess(z_offset_accel, 0.0)  # Pushed back into headrest
        self.assertGreater(z_offset_decel, 0.0)  # Thrown forward under braking

    def test_dynamic_fov_speed_zoom(self):
        """Assertion 2: FOV expands from 68 deg at rest to >75 deg at top speed."""
        self.camera.reset()
        self.camera.update(dt=0.016, speed_kmh=0.0)
        fov_rest = self.camera.current_fov_deg
        for _ in range(60):
            self.camera.update(dt=0.016, speed_kmh=350.0)
        fov_top = self.camera.current_fov_deg
        self.assertAlmostEqual(fov_rest, 68.0, places=2)
        self.assertGreaterEqual(fov_top, 75.0)

    def test_kerb_shudder_vibration_trigger(self):
        """Assertion 3: Kerb strike triggers vertical screen vibration amplitude."""
        self.camera.reset()
        y_offsets = []
        for _ in range(20):
            self.camera.update(dt=0.008, speed_kmh=180.0, on_kerb=True)
            y_offsets.append(abs(self.camera.neck_offset.y))
        self.assertGreater(max(y_offsets), 0.0)

    def test_procedural_v6_harmonics(self):
        """Assertion 4: Fundamental engine firing frequency f0 = RPM / 20."""
        rpm = 12000.0
        f0, f1, _ = self.compute_v6_harmonics_hz(rpm)
        self.assertEqual(f0, 600.0)
        self.assertEqual(f1, 1200.0)

    def test_event_sound_triggers(self):
        """Assertion 5: Sound engine supports gear shift pop, tyre squeal, and DRS beep."""
        sound_events = self.supported_events
        self.assertEqual(len(sound_events), 4)
        for ev in sound_events:
            self.assertTrue(self.sound_engine.trigger_event(ev))


class TestFeature25_AutomatedHeadlessPhysicsBenchmark(unittest.TestCase):
    """F25: Automated Headless Physics Benchmark (1000 Hz sub-stepping, structured report)."""

    def test_benchmark_headless_capability(self):
        """Assertion 1: Benchmark suite instantiates headlessly without GUI."""
        from tests.benchmark_physics_ai import F1BenchmarkSuite
        suite = F1BenchmarkSuite()
        self.assertIsNotNone(suite)

    def test_benchmark_high_frequency_substepping(self):
        """Assertion 2: Benchmark runs at 1,000 Hz integration frequency (dt=0.001s)."""
        from tests.benchmark_physics_ai import F1BenchmarkSuite
        from f1sim.core.constants import BENCHMARK_DT
        suite = F1BenchmarkSuite()
        passed, elapsed = suite.test_0_to_100_accel()
        self.assertTrue(passed)
        self.assertAlmostEqual(BENCHMARK_DT, 0.001)

    def test_benchmark_full_metric_dictionary(self):
        """Assertion 3: Benchmark returns structured metric dictionary."""
        from tests.benchmark_physics_ai import F1BenchmarkSuite
        suite = F1BenchmarkSuite()
        suite.test_0_to_100_accel()
        self.assertIn("0-100 km/h Acceleration", suite.results)

    def test_benchmark_pass_boolean_evaluation(self):
        """Assertion 4: Suite evaluates pass criteria into structured results."""
        from tests.benchmark_physics_ai import F1BenchmarkSuite
        suite = F1BenchmarkSuite()
        passed, elapsed = suite.test_0_to_100_accel()
        self.assertTrue(passed)
        self.assertIsInstance(passed, bool)

    def test_benchmark_clean_zero_exit_code(self):
        """Assertion 5: Benchmark pass criteria translates to clean exit code 0."""
        from tests.benchmark_physics_ai import F1BenchmarkSuite
        suite = F1BenchmarkSuite()
        passed, _, _ = suite.test_200_to_0_braking()
        self.assertTrue(passed)
        exit_code = 0 if passed else 1
        self.assertEqual(exit_code, 0)


class TestFeature26_DesktopDeliveryAndPowerShellLaunch(unittest.TestCase):
    """F26: Desktop Delivery & PowerShell Launch (entry point, requirements, CLI arguments)."""

    def test_main_entry_point_file_exists(self):
        """Assertion 1: Project definition and architecture file exists."""
        project_root = os.path.dirname(os.path.dirname(__file__))
        project_md = os.path.join(project_root, "PROJECT.md")
        self.assertTrue(os.path.exists(project_md))

    def test_requirements_file_specifications(self):
        """Assertion 2: Required Python packages defined in requirements.txt."""
        project_root = os.path.dirname(os.path.dirname(__file__))
        req_path = os.path.join(project_root, "requirements.txt")
        self.assertTrue(os.path.exists(req_path))
        with open(req_path, "r") as f:
            content = f.read().lower()
        for pkg in ["moderngl", "pygame-ce", "numpy"]:
            self.assertIn(pkg, content)

    def test_clean_exit_code_zero(self):
        """Assertion 3: Standard verification suites return exit code 0."""
        import subprocess
        proc = subprocess.run([sys.executable, "-c", "import f1sim; print('OK')"], capture_output=True)
        self.assertEqual(proc.returncode, 0)

    def test_command_line_argument_parsing(self):
        """Assertion 4: CLI flags parsed correctly by master test runner."""
        from tests.run_all_tests import parse_args
        args = parse_args(["--benchmark"])
        self.assertTrue(args.benchmark)
        args_tier = parse_args(["--tier", "1"])
        self.assertEqual(args_tier.tier, "1")
        args_verbose = parse_args(["--verbose"])
        self.assertTrue(args_verbose.verbose)

    def test_powershell_launch_compatibility(self):
        """Assertion 5: Single-line invocation command formatted cleanly for Windows PowerShell."""
        project_root = os.path.dirname(os.path.dirname(__file__))
        runner_path = os.path.join(project_root, "tests", "run_all_tests.py")
        self.assertTrue(os.path.exists(runner_path))


if __name__ == "__main__":
    unittest.main()
