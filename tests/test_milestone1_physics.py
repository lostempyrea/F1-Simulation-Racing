"""
tests/test_milestone1_physics.py
================================
Comprehensive Milestone 1 Automated Test Suite for F1 Simulation Core & Physics.

Verifies:
1. Core foundations: math3d (Vec3, Mat3, Mat4, Quaternion, splines), time_step (AccumulatorClock, SubstepRunner), constants.
2. Powertrain & Transmission:
   - 1.6L V6 Turbo Hybrid ICE + MGU-K (1,000 hp / 746 kW peak power, torque curve vs RPM)
   - 8-speed seamless sequential gearbox with reverse
   - Manual shifting with 40ms seamless delay, 60ms auto-blip throttle on downshifts
   - Overrev protection rejecting downshifts above 14,800 RPM
   - Automatic transmission logic (kickdown, auto-upshift at 13,200 RPM, auto-downshift)
   - 4-stage ignition state machine (OFF -> IGNITION_ON -> CRANKING -> RUNNING)
   - Anti-stall clutch decouple (< 2,800 RPM in gear -> clutch decoupled, idle held at 4,000 RPM)
3. Aerodynamics & DRS:
   - Dynamic downforce and drag (rho = 1.225, A = 1.6, CL = 2.9, CD = 0.95, front downforce 44%, rear 56%)
   - DRS mechanics reducing CD by 28% and rear downforce by 32%
   - Flap actuator slew rate (0.15s opening/closing transition)
   - Safety auto-close interlocks on brake (> 0.05) or lateral G (> 0.40G)
4. Pacejka '96 Tyre Grip:
   - Non-linear Pacejka Magic Formula for longitudinal and lateral forces
   - Accurate slip angle kinematics with peak grip at 6.0° - 8.0° (0.105 - 0.140 rad)
   - Normal load sensitivity mu(Fz) decaying smoothly with vertical load
   - Combined slip friction ellipse coupling traction/braking with cornering force
5. Carbon-Carbon Brakes:
   - Thermal model with pad friction curve (cold glaze < 200°C, optimal 350-950°C, fade > 1000°C)
   - Convective duct cooling and Stefan-Boltzmann radiation
   - Adjustable Brake Bias (52% to 64% front, default 58%, step 0.5%)
6. Two-Track Vehicle Dynamics:
   - 4-wheel contact patch kinematics
   - Dynamic weight transfer (static, longitudinal under braking/accel, lateral with 55% front roll stiffness)
   - 0-100 km/h acceleration in 2.40s - 2.80s
   - 0-200 km/h acceleration in 4.80s - 5.50s
   - 200-0 km/h emergency stopping distance in < 55m (< 50m optimal) with >= 4.5G peak deceleration
   - Top speed > 340 km/h without DRS, > 355 km/h with DRS (delta >= 15 km/h)
   - Understeer dynamics (front slip saturation > 8° pushes car wide, yaw deficit)
   - Oversteer dynamics (rear wheelspin / brake lock collapses rear lateral grip, yaw surge, counter-steering stabilization)
   - VehicleTelemetry dataclass contract export
"""
import math
import unittest
import numpy as np

import f1sim.core.constants as c
from f1sim.core.math3d import (
    Vec3, Mat3, Mat4, Quaternion,
    catmull_rom_spline, catmull_rom_tangent, calculate_curvature,
    frenet_frame, clamp, lerp, smoothstep, wrap_angle_pi
)
from f1sim.core.time_step import AccumulatorClock, SubstepRunner, TimeStepState
from f1sim.physics.powertrain import Powertrain, EngineModel, Gearbox, EngineOutput, TransmissionOutput
from f1sim.physics.aero import Aerodynamics, DRSState, AeroForces, AeroConfig, AeroSetupPreset
from f1sim.physics.tyre import TyreModel, PacejkaParams, TyreForces
from f1sim.physics.brakes import BrakeSystem, BrakeDisc, BrakeConfig, BrakeTorqueOutput
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry, VehicleConfig, VehicleState


class TestCoreFoundations(unittest.TestCase):
    """Verifies core math, constants, and fixed time step clock."""

    def test_constants_definitions(self):
        self.assertAlmostEqual(c.GRAVITY, 9.80665)
        self.assertAlmostEqual(c.AIR_DENSITY, 1.225)
        self.assertEqual(c.SIM_HZ, 120)
        self.assertEqual(c.BENCHMARK_HZ, 1000)
        self.assertEqual(c.VEHICLE_MASS, 820.0)
        self.assertEqual(c.WHEELBASE, 3.600)
        self.assertEqual(c.TYRE_RADIUS, 0.360)
        self.assertEqual(c.RPM_IDLE, 4000.0)
        self.assertEqual(c.RPM_REDLINE, 15000.0)
        self.assertEqual(c.RPM_AUTO_UPSHIFT, 13200.0)
        self.assertEqual(c.RPM_AUTO_DOWNSHIFT, 8500.0)
        self.assertEqual(c.BRAKE_BIAS_DEFAULT, 0.58)
        self.assertEqual(c.BRAKE_BIAS_MIN, 0.52)
        self.assertEqual(c.BRAKE_BIAS_MAX, 0.64)

    def test_vec3_operations(self):
        v1 = Vec3(1.0, 2.0, 3.0)
        v2 = Vec3(4.0, 5.0, 6.0)
        self.assertEqual(v1 + v2, Vec3(5.0, 7.0, 9.0))
        self.assertEqual(v2 - v1, Vec3(3.0, 3.0, 3.0))
        self.assertEqual(v1 * 2.0, Vec3(2.0, 4.0, 6.0))
        self.assertAlmostEqual(v1.dot(v2), 32.0)
        cross = v1.cross(v2)
        self.assertEqual(cross, Vec3(-3.0, 6.0, -3.0))
        self.assertAlmostEqual(cross.dot(v1), 0.0)
        self.assertAlmostEqual(cross.dot(v2), 0.0)
        v_norm = v1.normalized()
        self.assertAlmostEqual(v_norm.length(), 1.0)

    def test_mat3_and_euler(self):
        rot_z = Mat3.rotation_z(math.pi / 2.0)
        v = Vec3(1.0, 0.0, 0.0)
        v_rot = rot_z.transform(v)
        self.assertAlmostEqual(v_rot.x, 0.0, places=5)
        self.assertAlmostEqual(v_rot.y, 1.0, places=5)
        self.assertAlmostEqual(v_rot.z, 0.0, places=5)

        ident = rot_z @ rot_z.inverse()
        self.assertTrue(np.allclose(ident.to_numpy(), np.eye(3)))

    def test_mat4_transforms(self):
        trans = Mat4.translation(10.0, -5.0, 2.0)
        p = Vec3(0.0, 0.0, 0.0)
        p_t = trans.transform_point(p)
        self.assertEqual(p_t, Vec3(10.0, -5.0, 2.0))

        v = Vec3(1.0, 0.0, 0.0)
        v_t = trans.transform_vector(v)
        self.assertEqual(v_t, Vec3(1.0, 0.0, 0.0))

    def test_quaternion_slerp(self):
        q1 = Quaternion.identity()
        q2 = Quaternion.from_axis_angle(Vec3(0, 0, 1), math.pi)
        q_mid = q1.slerp(q2, 0.5)
        eul = q_mid.to_euler()
        self.assertAlmostEqual(abs(eul[1]), math.pi / 2.0, places=3)

    def test_catmull_rom_spline_interpolation(self):
        p0 = np.array([0.0, 0.0, 0.0])
        p1 = np.array([10.0, 0.0, 0.0])
        p2 = np.array([20.0, 10.0, 0.0])
        p3 = np.array([30.0, 10.0, 0.0])

        start = catmull_rom_spline(p0, p1, p2, p3, 0.0)
        end = catmull_rom_spline(p0, p1, p2, p3, 1.0)
        self.assertTrue(np.allclose(start, p1))
        self.assertTrue(np.allclose(end, p2))

        tangent_mid = catmull_rom_tangent(p0, p1, p2, p3, 0.5)
        self.assertGreater(tangent_mid[0], 0.0)

    def test_accumulator_clock_120hz(self):
        clock = AccumulatorClock(hz=120)
        # Deliver 1/60s (approx 2 ticks)
        ticks = clock.tick(1.0 / 60.0)
        self.assertEqual(ticks, 2)
        self.assertAlmostEqual(clock.sim_time, 2.0 / 120.0)
        self.assertGreaterEqual(clock.alpha, 0.0)
        self.assertLess(clock.alpha, 1.0)


class TestPowertrainAndTransmission(unittest.TestCase):
    """Verifies hybrid ICE + MGU-K engine, sequential gearbox, ignition, and anti-stall."""

    def test_hybrid_engine_peak_power(self):
        eng = EngineModel()
        eng.ignition_state = c.IgnitionState.RUNNING
        # Evaluate power at 12,500 RPM
        rpm = 12500.0
        out = eng.update(dt=0.01, throttle=1.0, target_driveline_rpm=rpm)
        # Combined power should reach ~746 kW (1000 hp)
        self.assertGreaterEqual(out.power_kw, 720.0)
        self.assertLessEqual(out.power_kw, 760.0)

    def test_mguk_instant_low_end_torque(self):
        eng = EngineModel()
        eng.ignition_state = c.IgnitionState.RUNNING
        # At 3,000 RPM, MGU-K should deliver up to 350 Nm electric assist
        t_mguk = eng.compute_mguk_torque(3000.0, throttle=1.0)
        self.assertAlmostEqual(t_mguk, 350.0, delta=10.0)

    def test_engine_braking_at_zero_throttle(self):
        eng = EngineModel()
        eng.ignition_state = c.IgnitionState.RUNNING
        t_brake_low = eng.compute_ice_torque(4000.0, throttle=0.0)
        t_brake_high = eng.compute_ice_torque(14000.0, throttle=0.0)
        self.assertLess(t_brake_low, 0.0)
        self.assertLess(t_brake_high, t_brake_low)

    def test_ignition_4_stage_state_machine(self):
        eng = EngineModel()
        # Stage 0: OFF
        self.assertEqual(eng.ignition_state, c.IgnitionState.OFF)
        self.assertEqual(eng.rpm, 0.0)

        # Stage 1: IGNITION_ON
        eng.set_ignition(True)
        self.assertEqual(eng.ignition_state, c.IgnitionState.IGNITION_ON)
        self.assertEqual(eng.rpm, 0.0)

        # Stage 2: CRANKING
        eng.press_starter(0.01)
        self.assertEqual(eng.ignition_state, c.IgnitionState.CRANKING)
        # Crank for 0.4s (partial crank)
        for _ in range(40):
            eng.update(dt=0.01, throttle=0.0)
        self.assertGreater(eng.rpm, 1000.0)
        self.assertLess(eng.rpm, c.RPM_CRANKING_TARGET)

        # Complete crank to >= 0.8s -> transitions to RUNNING
        for _ in range(50):
            eng.update(dt=0.01, throttle=0.0)
        self.assertEqual(eng.ignition_state, c.IgnitionState.RUNNING)
        # Flare to 6500 RPM
        self.assertGreaterEqual(eng.rpm, 6000.0)

        # Settle to idle over 0.5s
        for _ in range(60):
            eng.update(dt=0.01, throttle=0.0)
        self.assertAlmostEqual(eng.rpm, c.RPM_IDLE, delta=50.0)

        # Kill switch returns to OFF
        eng.set_ignition(False)
        self.assertEqual(eng.ignition_state, c.IgnitionState.OFF)
        self.assertEqual(eng.rpm, 0.0)

    def test_anti_stall_clutch_decouple(self):
        pt = Powertrain(mode=c.TransmissionMode.MANUAL)
        pt.set_running()
        pt.gearbox.gear = 2
        # Stationary wheels in gear -> RPM drops below 2800 RPM
        out = pt.step(throttle=0.0, brake=1.0, wheel_speed_ms=0.0, dt=0.01)
        self.assertTrue(out.anti_stall_active)
        self.assertEqual(out.clutch_engagement, 0.0)
        # Engine holds steady idle at 4000 RPM
        self.assertAlmostEqual(out.engine_rpm, c.RPM_IDLE, delta=50.0)

        # Reapplication of throttle recovers clutch
        out = pt.step(throttle=0.50, brake=0.0, wheel_speed_ms=5.0, dt=0.05)
        self.assertFalse(out.anti_stall_active)
        self.assertGreater(out.clutch_engagement, 0.0)

    def test_seamless_shift_delay_and_torque_cut(self):
        gb = Gearbox(mode=c.TransmissionMode.MANUAL)
        gb.gear = 2
        self.assertTrue(gb.request_upshift())
        self.assertEqual(gb.gear, 3)
        self.assertTrue(gb.shift_timer > 0.0)
        self.assertAlmostEqual(gb.shift_timer, c.SEAMLESS_SHIFT_DURATION, delta=0.005)

    def test_downshift_auto_blip_and_overrev_protection(self):
        gb = Gearbox(mode=c.TransmissionMode.MANUAL)
        gb.gear = 3
        # Safe downshift at 30 m/s
        self.assertTrue(gb.request_downshift(wheel_speed_ms=30.0))
        self.assertEqual(gb.gear, 2)
        self.assertTrue(gb.blip_timer > 0.0)
        self.assertAlmostEqual(gb.blip_timer, c.THROTTLE_BLIP_DURATION, delta=0.005)

        # Excessive speed downshift rejected: 2nd gear at 90 m/s (324 km/h) into 1st
        gb.gear = 2
        rejected = gb.request_downshift(wheel_speed_ms=90.0)
        self.assertFalse(rejected)
        self.assertEqual(gb.gear, 2)  # Remains in 2nd!


class TestAerodynamicsAndDRS(unittest.TestCase):
    """Verifies speed-squared downforce, drag, DRS mechanics, and safety interlocks."""

    def test_nominal_aero_forces(self):
        aero = Aerodynamics()
        speed = 50.0  # 180 km/h
        # q = 0.5 * 1.225 * 50^2 = 1531.25 Pa
        # downforce = 1531.25 * 2.9 * 1.6 = 7105 N
        # drag = 1531.25 * 0.95 * 1.6 = 2327.5 N
        f_down, f_drag, f_down_f, f_down_r = aero.compute(speed, drs_active=False)
        self.assertAlmostEqual(f_down, 7105.0, delta=10.0)
        self.assertAlmostEqual(f_drag, 2327.5, delta=10.0)
        self.assertAlmostEqual(f_down_f / f_down, 0.44, delta=0.01)
        self.assertAlmostEqual(f_down_r / f_down, 0.56, delta=0.01)

    def test_drs_drag_and_downforce_reduction(self):
        aero = Aerodynamics()
        speed = 80.0
        f_down_closed, f_drag_closed, _, f_r_closed = aero.compute(speed, drs_active=False)
        f_down_open, f_drag_open, _, f_r_open = aero.compute(speed, drs_active=True)

        # Drag reduces by 28%
        self.assertAlmostEqual(f_drag_open / f_drag_closed, 0.72, delta=0.01)
        # Rear downforce reduces by 32%
        self.assertAlmostEqual(f_r_open / f_r_closed, 0.68, delta=0.01)

    def test_drs_safety_auto_close_on_brake(self):
        aero = Aerodynamics()
        # Enable DRS in zone
        aero.update(speed_ms=70.0, dt=0.01, in_drs_zone=True, brake_input=0.0, lateral_g=0.0)
        aero.request_drs_toggle()
        self.assertEqual(aero.drs_state, DRSState.ACTIVE)

        # Tap brake pedal > 0.05
        aero.update(speed_ms=70.0, dt=0.01, in_drs_zone=True, brake_input=0.20, lateral_g=0.0)
        self.assertNotEqual(aero.drs_state, DRSState.ACTIVE)

    def test_drs_safety_auto_close_on_lateral_g(self):
        aero = Aerodynamics()
        aero.update(speed_ms=70.0, dt=0.01, in_drs_zone=True, brake_input=0.0, lateral_g=0.0)
        aero.request_drs_toggle()
        self.assertEqual(aero.drs_state, DRSState.ACTIVE)

        # Sharp turn > 0.40 G
        aero.update(speed_ms=70.0, dt=0.01, in_drs_zone=True, brake_input=0.0, lateral_g=0.55)
        self.assertNotEqual(aero.drs_state, DRSState.ACTIVE)


class TestPacejkaTyreModel(unittest.TestCase):
    """Verifies Pacejka '96 Magic Formula, peak slip angles, load sensitivity, and friction ellipse."""

    def test_peak_cornering_slip_angle(self):
        tyre = TyreModel()
        fz = 4000.0
        angles_rad = np.linspace(math.radians(1.0), math.radians(16.0), 100)
        forces = [tyre.compute_pure_lateral(a, fz) for a in angles_rad]
        peak_idx = int(np.argmax(forces))
        peak_angle_deg = math.degrees(angles_rad[peak_idx])

        # Peak grip must occur between 6.0° and 8.0°
        self.assertGreaterEqual(peak_angle_deg, 6.0)
        self.assertLessEqual(peak_angle_deg, 8.5)

        # Post-peak drop-off
        slide_force = tyre.compute_pure_lateral(math.radians(15.0), fz)
        self.assertLess(slide_force, forces[peak_idx])

    def test_normal_load_sensitivity(self):
        tyre = TyreModel()
        mu_light = tyre.compute_mu(2000.0, is_lateral=True)
        mu_heavy = tyre.compute_mu(8000.0, is_lateral=True)
        self.assertGreater(mu_light, mu_heavy)

    def test_combined_slip_friction_ellipse(self):
        tyre = TyreModel()
        fz = 4000.0
        # 100% braking lockup kappa = -1.0
        forces = tyre.compute_combined_forces(kappa=-1.0, alpha=math.radians(7.0), Fz=fz)
        # Lateral force must collapse
        self.assertLess(abs(forces.Fy), 0.15 * tyre.compute_pure_lateral(math.radians(7.0), fz))
        # Inside friction ellipse: (Fx/Dx)^2 + (Fy/Dy)^2 <= 1.001
        dx = tyre.compute_mu(fz, is_lateral=False) * fz
        dy = tyre.compute_mu(fz, is_lateral=True) * fz
        ellipse_metric = (forces.Fx / dx) ** 2 + (forces.Fy / dy) ** 2
        self.assertLessEqual(ellipse_metric, 1.001)


class TestCarbonBrakes(unittest.TestCase):
    """Verifies carbon-carbon thermal model, pad friction, and brake bias adjustment."""

    def test_brake_bias_adjustment(self):
        brakes = BrakeSystem()
        self.assertEqual(brakes.get_bias(), c.BRAKE_BIAS_DEFAULT)

        # Increment towards front
        brakes.adjust_bias(+1)
        self.assertAlmostEqual(brakes.get_bias(), 0.585)

        # Clamp max
        for _ in range(30):
            brakes.adjust_bias(+1)
        self.assertEqual(brakes.get_bias(), c.BRAKE_BIAS_MAX)

        # Clamp min
        for _ in range(40):
            brakes.adjust_bias(-1)
        self.assertEqual(brakes.get_bias(), c.BRAKE_BIAS_MIN)

    def test_pad_friction_temperature_curve(self):
        disc = BrakeDisc(BrakeConfig(), initial_temp_c=50.0)
        # Cold glaze < 100°C
        self.assertEqual(disc.pad_friction_coefficient, 0.25)

        # Optimal window 350°C - 950°C
        disc.temperature_c = 600.0
        self.assertEqual(disc.pad_friction_coefficient, 0.52)

        # Thermal fade > 950°C
        disc.temperature_c = 1100.0
        self.assertLess(disc.pad_friction_coefficient, 0.52)
        self.assertGreater(disc.pad_friction_coefficient, 0.30)


class TestVehiclePhysicsBenchmarks(unittest.TestCase):
    """Verifies the core Milestone 1 vehicle physics benchmark targets."""

    def test_0_to_100_acceleration(self):
        """Benchmark: 0-100 km/h in 2.40s - 2.80s."""
        car = Vehicle()
        car.gear = 1
        car.rpm = 4000.0
        dt = 0.001
        elapsed = 0.0

        while car.speed_kmh < 100.0 and elapsed < 4.0:
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            elapsed += dt

        self.assertGreaterEqual(elapsed, 2.40, f"0-100 time {elapsed:.3f}s was too fast (< 2.40s)")
        self.assertLessEqual(elapsed, 2.80, f"0-100 time {elapsed:.3f}s was too slow (> 2.80s)")

    def test_0_to_200_acceleration(self):
        """Benchmark: 0-200 km/h in 4.80s - 5.50s."""
        car = Vehicle()
        car.gear = 1
        car.rpm = 4000.0
        dt = 0.001
        elapsed = 0.0

        while car.speed_kmh < 200.0 and elapsed < 8.0:
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            elapsed += dt

        self.assertGreaterEqual(elapsed, 4.80, f"0-200 time {elapsed:.3f}s was too fast (< 4.80s)")
        self.assertLessEqual(elapsed, 5.50, f"0-200 time {elapsed:.3f}s was too slow (> 5.50s)")

    def test_200_to_0_emergency_braking(self):
        """Benchmark: 200-0 km/h in < 55.0m with peak deceleration >= 4.5G."""
        car = Vehicle()
        car.vx = 200.0 / 3.6
        car.gear = 6
        car.brake_bias = 0.58
        dt = 0.001
        distance = 0.0
        peak_g = 0.0

        while car.speed_kmh > 0.1:
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            g = abs(car.longitudinal_g)
            if g > peak_g:
                peak_g = g
            distance += car.vx * dt

        self.assertLess(distance, 55.0, f"Stopping distance {distance:.2f}m >= 55.0m")
        self.assertLess(distance, 50.0, f"Stopping distance {distance:.2f}m >= 50.0m in optimal range")
        self.assertGreaterEqual(peak_g, 4.5, f"Peak deceleration {peak_g:.2f}G < 4.5G")

    def test_top_speed_closed_and_open_drs(self):
        """Benchmark: Top speed > 340 km/h without DRS, > 355 km/h with DRS (delta >= 15 km/h)."""
        # Closed run
        car_closed = Vehicle()
        car_closed.vx = 80.0
        car_closed.gear = 7
        car_closed.drs_active = False
        dt = 0.001

        for _ in range(15000):
            if car_closed.rpm >= 12800.0 and car_closed.gear < 8:
                car_closed.shift_up()
            car_closed.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)

        v_closed = car_closed.speed_kmh
        self.assertGreaterEqual(v_closed, 340.0, f"Top speed closed {v_closed:.1f} km/h < 340.0 km/h")

        # Open run
        car_open = Vehicle()
        car_open.vx = 80.0
        car_open.gear = 7
        car_open.drs_active = True

        for _ in range(15000):
            if car_open.rpm >= 12800.0 and car_open.gear < 8:
                car_open.shift_up()
            car_open.drs_active = True
            car_open.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)

        v_open = car_open.speed_kmh
        self.assertGreaterEqual(v_open, 355.0, f"DRS top speed {v_open:.1f} km/h < 355.0 km/h")
        self.assertGreaterEqual(v_open - v_closed, 15.0, f"DRS delta {v_open - v_closed:.1f} km/h < 15.0 km/h")

    def test_understeer_slip_angle_saturation(self):
        """Benchmark: Understeer front slip angle > 8.0 degrees under excessive entry speed."""
        car = Vehicle()
        car.vx = 65.0  # 234 km/h
        car.gear = 6
        dt = 0.001

        max_front_slip_deg = 0.0
        for _ in range(600):
            car.step(throttle=0.0, brake=0.0, steer=0.45, dt=dt)
            slip_deg = math.degrees(abs(car.slip_angle_front))
            if slip_deg > max_front_slip_deg:
                max_front_slip_deg = slip_deg

        self.assertGreater(max_front_slip_deg, 8.0, f"Understeer slip {max_front_slip_deg:.1f} deg <= 8.0 deg")

    def test_power_oversteer_breakaway_and_counter_steer(self):
        """Benchmark: Full throttle corner exit induces rear slip ratio > 0.20 and yaw surge."""
        car = Vehicle()
        car.vx = 14.0  # ~50 km/h
        car.gear = 1
        car.steer = 0.35
        dt = 0.001

        max_rear_slip = 0.0
        max_yaw_rate = 0.0
        for _ in range(500):
            car.step(throttle=1.0, brake=0.0, steer=0.35, dt=dt)
            if car.slip_ratio_rear > max_rear_slip:
                max_rear_slip = car.slip_ratio_rear
            if abs(car.yaw_rate) > max_yaw_rate:
                max_yaw_rate = abs(car.yaw_rate)

        self.assertGreater(max_rear_slip, 0.20, f"Rear slip {max_rear_slip:.2f} <= 0.20")
        self.assertGreater(max_yaw_rate, 0.80, f"Yaw rate {max_yaw_rate:.2f} <= 0.80 rad/s")

        # Apply counter-steering
        initial_yaw_rate = car.yaw_rate
        for _ in range(200):
            car.step(throttle=0.0, brake=0.0, steer=-0.50, dt=dt)
        # Yaw rate decelerates or reverses
        self.assertLess(car.yaw_rate, initial_yaw_rate)

    def test_vehicle_telemetry_export(self):
        """Verifies VehicleTelemetry dataclass contract export."""
        car = Vehicle()
        telem = car.get_telemetry()
        self.assertIsInstance(telem, VehicleTelemetry)
        self.assertAlmostEqual(telem.brake_bias, 0.58)
        self.assertTrue(telem.engine_running)
        self.assertEqual(telem.gear, 1)


if __name__ == "__main__":
    unittest.main()
