"""
tests/test_challenger_m1_iter2_handling.py
==========================================
Empirical Adversarial Challenge Suite 2:
Milestone 1 Handling Dynamics, Standstill Stability, 120Hz Zero-Crossing Latching,
Reverse Kinematics & Lockout, and Emergent Power-Oversteer Breakaway.

Author: Challenger 2 (Empirical Challenger)
Milestone: Milestone 1 Gate Re-evaluation
"""

import math
import unittest
import numpy as np

from f1sim.core.constants import (
    RPM_IDLE, RPM_AUTO_UPSHIFT, TransmissionMode,
    BRAKE_BIAS_DEFAULT, BRAKE_BIAS_MIN, BRAKE_BIAS_MAX
)
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry, VehicleConfig, VehicleState
from f1sim.physics.powertrain import Powertrain
from f1sim.physics.tyre import TyreModel, PacejkaParams


class TestStandstillZeroInputStability(unittest.TestCase):
    """
    Empirical Challenge 1:
    Standstill zero-input stability (Anti-creep).
    At 0 throttle and 0 brake in Gear 1 (and all gears), speed must remain
    identically 0.00 km/h (speed < 0.05 km/h).
    """

    def test_standstill_gear1_zero_input_multi_dt(self):
        """Verify 0 throttle, 0 brake in Gear 1 produces zero creep across dt = 1/120s, 1/60s, 1/1000s."""
        dts = [1.0 / 120.0, 1.0 / 60.0, 0.001, 0.01]
        for dt in dts:
            car = Vehicle()
            car.vx = 0.0
            car.gear = 1
            car.rpm = RPM_IDLE
            car.powertrain.set_running()

            max_speed_kmh = 0.0
            for _ in range(int(3.0 / dt)):  # 3 seconds duration
                telem = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)
                if telem.speed_kmh > max_speed_kmh:
                    max_speed_kmh = telem.speed_kmh

            print(f"[Anti-Creep Gear 1 dt={dt:.5f}s] Max Speed: {max_speed_kmh:.6f} km/h (Target < 0.05)")
            self.assertLess(max_speed_kmh, 0.05, f"Creep detected in Gear 1 at dt={dt}: {max_speed_kmh} km/h")
            self.assertEqual(max_speed_kmh, 0.0, f"Expected exactly 0.0 km/h creep at dt={dt}, got {max_speed_kmh}")

    def test_standstill_all_gears_zero_input(self):
        """Verify zero creep across Neutral, Gear 1, Gear 2, and Reverse at standstill."""
        gears = [0, 1, 2, -1]
        dt = 1.0 / 120.0

        for g in gears:
            car = Vehicle()
            car.vx = 0.0
            car.gear = g
            car.rpm = RPM_IDLE
            car.powertrain.set_running()

            max_speed = 0.0
            for _ in range(240):  # 2.0s
                telem = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)
                if telem.speed_kmh > max_speed:
                    max_speed = telem.speed_kmh

            print(f"[Anti-Creep Gear {g}] Max Speed: {max_speed:.6f} km/h")
            self.assertLess(max_speed, 0.05, f"Creep detected in gear {g}: {max_speed} km/h")
            self.assertEqual(max_speed, 0.0)

    def test_standstill_long_duration_drift(self):
        """Verify 30 seconds (3,600 steps at 120Hz) of standstill produces zero cumulative position displacement."""
        car = Vehicle()
        car.vx = 0.0
        car.gear = 1
        car.rpm = RPM_IDLE
        car.powertrain.set_running()
        dt = 1.0 / 120.0

        for _ in range(3600):
            car.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)

        disp = math.sqrt(car.state.x ** 2 + car.state.y ** 2)
        print(f"[Anti-Creep 30s 120Hz] Cumulative Displacement: {disp:.6f} m, Final Speed: {car.speed_kmh:.6f} km/h")
        self.assertEqual(disp, 0.0, f"Cumulative drift over 30s: {disp} m")
        self.assertEqual(car.speed_kmh, 0.0)

    def test_standstill_small_perturbation_latching(self):
        """Verify that micro-perturbation below standstill threshold (< 0.05 m/s) is latched to 0."""
        car = Vehicle()
        car.state.vx = 0.03  # 0.108 km/h perturbation
        car.gear = 1
        car.rpm = RPM_IDLE
        car.powertrain.set_running()
        dt = 1.0 / 120.0

        telem = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)
        print(f"[Micro-Perturbation Latch] After 1 step: speed = {telem.speed_kmh:.6f} km/h, vx = {car.state.vx:.6f}")
        self.assertEqual(car.state.vx, 0.0, "Micro-perturbation was not latched to 0.0")
        self.assertEqual(telem.speed_kmh, 0.0)


class Test120HzZeroCrossingLatching(unittest.TestCase):
    """
    Empirical Challenge 2:
    120 Hz zero-crossing latching: stopping from 200 km/h at dt=1/120s does not
    result in reverse velocity or runaway oscillation.
    """

    def test_200_to_0_emergency_braking_120hz_latching(self):
        """
        Verify stopping from 200 km/h at dt=1/120s:
        - Distance < 55m
        - Peak decel >= 4.5G
        - Min vx >= -0.0001 m/s (no negative velocity overshoot)
        - Latches at 0.00 km/h and remains motionless under continuous full braking.
        """
        car = Vehicle()
        car.vx = 200.0 / 3.6  # 55.556 m/s
        car.gear = 6
        car.brake_bias = 0.58
        dt = 1.0 / 120.0

        min_vx = 0.0
        max_decel_g = 0.0
        dist = 0.0
        stopped_step = None
        vx_history_near_stop = []

        total_steps = 480  # 4.0 seconds (sufficient to stop and observe standstill under brake)
        for step_idx in range(total_steps):
            vx_before = car.state.vx
            telem = car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            vx_after = car.state.vx

            if telem.longitudinal_g < -max_decel_g:
                max_decel_g = abs(telem.longitudinal_g)

            if vx_after < min_vx:
                min_vx = vx_after

            if vx_after > 0.0:
                dist += vx_after * dt

            if vx_before > 0.0 and vx_after == 0.0 and stopped_step is None:
                stopped_step = step_idx

            if stopped_step is not None and step_idx - stopped_step < 10:
                vx_history_near_stop.append((step_idx, vx_after))

        print(f"\n[120Hz Braking Latch] Stopping Distance: {dist:.2f} m (Target < 55.0 m)")
        print(f"[120Hz Braking Latch] Peak Decel: {max_decel_g:.2f} G (Target >= 4.5 G)")
        print(f"[120Hz Braking Latch] Min vx encountered: {min_vx:.6f} m/s (Target >= -0.0001 m/s)")
        print(f"[120Hz Braking Latch] Stopped at step: {stopped_step} ({stopped_step * dt:.3f} s)")
        print(f"[120Hz Braking Latch] Final speed after 4.0s full brake: {car.speed_kmh:.6f} km/h")

        self.assertLess(dist, 55.0, f"Stopping distance {dist:.2f}m exceeded 55.0m")
        self.assertGreaterEqual(max_decel_g, 4.50, f"Peak decel {max_decel_g:.2f}G below 4.5G")
        self.assertGreaterEqual(min_vx, -0.0001, f"Negative velocity overshoot observed: min_vx = {min_vx}")
        self.assertEqual(car.speed_kmh, 0.0, f"Final speed not 0.0: {car.speed_kmh}")

    def test_latching_across_framerate_spectrum(self):
        """Stress-test zero-crossing latch across 60Hz, 120Hz, 144Hz, 240Hz, 500Hz, 1000Hz."""
        frequencies = [60, 120, 144, 240, 500, 1000]
        for f in frequencies:
            dt = 1.0 / f
            car = Vehicle()
            car.vx = 55.556  # 200 km/h
            car.gear = 6
            min_vx = 0.0

            for _ in range(int(3.5 / dt)):
                car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
                if car.state.vx < min_vx:
                    min_vx = car.state.vx

            print(f"[Braking Latch @ {f} Hz] Final speed: {car.speed_kmh:.6f} km/h, Min vx: {min_vx:.6f} m/s")
            self.assertGreaterEqual(min_vx, -0.0001, f"Overshoot at {f}Hz: {min_vx}")
            self.assertEqual(car.speed_kmh, 0.0, f"Did not stop at {f}Hz: {car.speed_kmh}")

    def test_low_speed_partial_braking_latch(self):
        """Verify stopping smoothly from slow entry speeds (5 km/h, 15 km/h, 30 km/h) under varying brake levels."""
        entry_speeds_kmh = [5.0, 15.0, 30.0]
        brake_levels = [0.25, 0.50, 0.75, 1.00]
        dt = 1.0 / 120.0

        for v_kmh in entry_speeds_kmh:
            for b in brake_levels:
                car = Vehicle()
                car.vx = v_kmh / 3.6
                car.gear = 1
                min_vx = 0.0

                for _ in range(240):  # 2.0s
                    car.step(throttle=0.0, brake=b, steer=0.0, dt=dt)
                    if car.state.vx < min_vx:
                        min_vx = car.state.vx

                self.assertGreaterEqual(min_vx, -0.0001, f"Overshoot from {v_kmh} km/h at brake {b}: {min_vx}")
                self.assertEqual(car.speed_kmh, 0.0, f"Car failed to stop cleanly from {v_kmh} km/h at brake {b}")


class TestReverseKinematicsAndLockout(unittest.TestCase):
    """
    Empirical Challenge 3:
    Reverse kinematics:
    1. Reverse gear acceleration gives negative forward velocity (vx < 0).
    2. Reverse braking opposes motion (decelerates towards 0, latches at 0).
    3. Reverse lockout prevents downshift into R at forward speed > 1 km/h.
    """

    def test_reverse_acceleration_kinematics(self):
        """Throttle in Reverse gear (gear=-1) accelerates vehicle backward (negative vx)."""
        car = Vehicle()
        car.vx = 0.0
        car.gear = -1
        car.powertrain.set_running()
        dt = 1.0 / 120.0

        vx_samples = []
        for _ in range(120):  # 1.0s
            telem = car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            vx_samples.append(car.state.vx)

        final_vx = car.state.vx
        final_speed = telem.speed_kmh
        print(f"\n[Reverse Accel] Final vx: {final_vx:.3f} m/s, Speed: {final_speed:.2f} km/h")

        self.assertLess(final_vx, -3.0, f"Reverse acceleration failed: vx = {final_vx} m/s")
        self.assertGreater(final_speed, 10.0, f"Reported scalar speed {final_speed} km/h too low")
        # Ensure vx monotonically decreases (becoming more negative)
        self.assertLess(vx_samples[-1], vx_samples[0])
        # Ensure rear wheel omega is negative
        self.assertLess(car.state.wheel_omega[2], 0.0, "Rear left wheel omega must be negative in reverse")
        self.assertLess(car.state.wheel_omega[3], 0.0, "Rear right wheel omega must be negative in reverse")

    def test_reverse_braking_opposes_motion(self):
        """Braking while moving in reverse opposes motion (accelerates vx toward 0) and latches at 0."""
        car = Vehicle()
        car.vx = -12.0  # moving backward at 43.2 km/h
        car.gear = -1
        car.powertrain.set_running()
        dt = 1.0 / 120.0

        max_vx = car.state.vx
        for _ in range(180):  # 1.5s braking
            vx_before = car.state.vx
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            vx_after = car.state.vx
            if vx_after > max_vx:
                max_vx = vx_after

        print(f"[Reverse Braking] Initial vx: -12.00 m/s -> Final vx: {car.state.vx:.6f} m/s (Speed: {car.speed_kmh:.6f} km/h)")
        self.assertGreater(max_vx, -1.0, "Braking in reverse did not decelerate vehicle toward zero")
        self.assertEqual(car.state.vx, 0.0, "Reverse braking failed to latch at standstill 0.0 m/s")
        self.assertEqual(car.speed_kmh, 0.0, "Final speed not 0.0 km/h")

    def test_reverse_lockout_sweep(self):
        """
        Reverse lockout boundary sweep:
        Downshifting from Neutral to Reverse must be locked out at speeds > 1.0 km/h (0.2778 m/s),
        and permitted at speeds <= 1.0 km/h.
        """
        car = Vehicle()

        # Speeds above 1 km/h: MUST BE REJECTED
        speeds_rejected_kmh = [1.2, 2.0, 5.0, 20.0, 72.0, 200.0]
        for v_kmh in speeds_rejected_kmh:
            car.vx = v_kmh / 3.6
            car.gear = 0  # Neutral
            success = car.shift_down()
            self.assertFalse(success, f"Reverse lockout failed at {v_kmh} km/h: shift was accepted!")
            self.assertEqual(car.gear, 0, f"Gear shifted from 0 to {car.gear} at {v_kmh} km/h!")

        # Speeds at or below 1 km/h: MUST BE PERMITTED
        speeds_permitted_kmh = [0.0, 0.2, 0.5, 0.9]
        for v_kmh in speeds_permitted_kmh:
            car.vx = v_kmh / 3.6
            car.gear = 0  # Neutral
            car.powertrain.gearbox.shift_timer = 0.0
            success = car.shift_down()
            self.assertTrue(success, f"Reverse shift incorrectly rejected at low speed {v_kmh} km/h!")
            self.assertEqual(car.gear, -1, f"Gear did not transition to -1 at {v_kmh} km/h!")

    def test_shift_from_gear1_requires_passing_through_neutral(self):
        """Direct shift down from Gear 1 must enter Neutral (0), not Reverse (-1)."""
        car = Vehicle()
        car.vx = 0.0
        car.gear = 1
        car.powertrain.gearbox.shift_timer = 0.0

        # First downshift -> 0 (Neutral)
        s1 = car.shift_down()
        self.assertTrue(s1)
        self.assertEqual(car.gear, 0, "Downshift from Gear 1 should go to Neutral (0)")

        # Second downshift at 0 km/h -> -1 (Reverse)
        car.powertrain.gearbox.shift_timer = 0.0
        s2 = car.shift_down()
        self.assertTrue(s2)
        self.assertEqual(car.gear, -1, "Downshift from Neutral should go to Reverse (-1)")


class TestPowerOversteerBreakaway(unittest.TestCase):
    """
    Empirical Challenge 4:
    Emergent power-oversteer breakaway:
    Full throttle in gear 1/2 corner exit induces rear slip ratio > 0.20
    and peak yaw rate > 0.80 rad/s without synthetic yaw moment multipliers.
    """

    def test_power_oversteer_gear1_breakaway_metrics(self):
        """
        Verify in Gear 1 corner exit:
        - Rear slip ratio > 0.20
        - Peak yaw rate > 0.80 rad/s
        - Peak yaw acceleration > 1.50 rad/s^2
        """
        car = Vehicle()
        car.vx = 14.0  # 50.4 km/h apex speed
        car.gear = 1
        car.steer = 0.35  # ~20 deg road wheel steer
        dt = 1.0 / 120.0

        peak_rear_slip = 0.0
        peak_yaw_rate = 0.0
        peak_yaw_accel = 0.0

        for _ in range(60):  # 0.5s full throttle stomp
            telem = car.step(throttle=1.0, brake=0.0, steer=0.35, dt=dt)
            if car.slip_ratio_rear > peak_rear_slip:
                peak_rear_slip = car.slip_ratio_rear
            if abs(car.state.yaw_rate) > peak_yaw_rate:
                peak_yaw_rate = abs(car.state.yaw_rate)
            if abs(car.state.yaw_accel) > peak_yaw_accel:
                peak_yaw_accel = abs(car.state.yaw_accel)

        print(f"\n[Power-Oversteer Gear 1] Peak Rear Slip Ratio: {peak_rear_slip:.4f} (Criterion > 0.20)")
        print(f"[Power-Oversteer Gear 1] Peak Yaw Rate: {peak_yaw_rate:.4f} rad/s (Criterion > 0.80)")
        print(f"[Power-Oversteer Gear 1] Peak Yaw Accel: {peak_yaw_accel:.4f} rad/s^2 (Criterion > 1.50)")

        self.assertGreater(peak_rear_slip, 0.20, f"Rear slip ratio {peak_rear_slip} <= 0.20")
        self.assertGreater(peak_yaw_rate, 0.80, f"Yaw rate {peak_yaw_rate} <= 0.80 rad/s")
        self.assertGreater(peak_yaw_accel, 1.50, f"Yaw accel {peak_yaw_accel} <= 1.50 rad/s^2")

    def test_power_oversteer_gear2_breakaway_metrics(self):
        """
        Verify in Gear 2 corner exit (higher speed ~80 km/h):
        - Rear slip ratio > 0.20
        - Peak yaw rate > 0.80 rad/s
        """
        car = Vehicle()
        car.vx = 22.0  # 79.2 km/h
        car.gear = 2
        car.steer = 0.30
        dt = 1.0 / 120.0

        peak_rear_slip = 0.0
        peak_yaw_rate = 0.0

        for _ in range(60):
            car.step(throttle=1.0, brake=0.0, steer=0.30, dt=dt)
            if car.slip_ratio_rear > peak_rear_slip:
                peak_rear_slip = car.slip_ratio_rear
            if abs(car.state.yaw_rate) > peak_yaw_rate:
                peak_yaw_rate = abs(car.state.yaw_rate)

        print(f"\n[Power-Oversteer Gear 2] Peak Rear Slip Ratio: {peak_rear_slip:.4f} (Criterion > 0.20)")
        print(f"[Power-Oversteer Gear 2] Peak Yaw Rate: {peak_yaw_rate:.4f} rad/s (Criterion > 0.80)")

        self.assertGreater(peak_rear_slip, 0.20, f"Gear 2 rear slip {peak_rear_slip} <= 0.20")
        self.assertGreater(peak_yaw_rate, 0.80, f"Gear 2 yaw rate {peak_yaw_rate} <= 0.80 rad/s")

    def test_high_gear_traction_stability_contrast(self):
        """
        Traction contrast:
        In 5th gear at 200 km/h, full throttle cornering should NOT induce violent slip breakaway
        because transmission torque multiplication is lower.
        """
        car = Vehicle()
        car.vx = 55.556  # 200 km/h
        car.gear = 5
        car.steer = 0.15
        dt = 1.0 / 120.0

        peak_rear_slip = 0.0
        for _ in range(60):
            car.step(throttle=1.0, brake=0.0, steer=0.15, dt=dt)
            if car.slip_ratio_rear > peak_rear_slip:
                peak_rear_slip = car.slip_ratio_rear

        print(f"[Traction Contrast Gear 5] Peak Rear Slip Ratio: {peak_rear_slip:.4f}")
        self.assertLess(peak_rear_slip, 0.15, "5th gear corner exit should not experience severe wheelspin breakaway")

    def test_no_synthetic_multiplier_in_vehicle_dynamics(self):
        """
        Source code inspection & verification:
        Ensure no synthetic multiplier (e.g. '* 2.45') or artificial trigger flag
        overrides Mz or lateral tyre forces in vehicle.py.
        """
        import inspect
        import f1sim.physics.vehicle as vmod

        src = inspect.getsource(vmod.Vehicle.step)
        self.assertNotIn("* 2.45", src, "Found synthetic '* 2.45' multiplier in Vehicle.step!")
        self.assertNotIn("is_power_oversteer", src, "Found synthetic 'is_power_oversteer' trigger in Vehicle.step!")
        self.assertNotIn("ax_target", src, "Found synthetic 'ax_target' piecewise power clamp in Vehicle.step!")


if __name__ == "__main__":
    unittest.main()
