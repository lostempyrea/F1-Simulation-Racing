"""
tests/test_gate_m1_challenger1.py
=================================
Empirical Stress Test & Verification Suite by Challenger 1
Milestone 1 Gate Re-evaluation: Vehicle Physics, Acceleration Continuity,
Emergency Braking, DRS Terminal Speeds, and Standstill/Reverse Robustness.
"""

import math
import unittest
import numpy as np

from f1sim.core.constants import (
    TransmissionMode, RPM_IDLE, RPM_REDLINE, RPM_AUTO_UPSHIFT,
    BRAKE_BIAS_MIN, BRAKE_BIAS_MAX, BRAKE_BIAS_DEFAULT, GRAVITY
)
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry, VehicleConfig, VehicleState
from f1sim.physics.aero import Aerodynamics, DRSState, AeroConfig
from f1sim.physics.brakes import BrakeSystem, BrakeConfig
from f1sim.physics.powertrain import Powertrain, EngineModel


class EmpiricalChallenger1PhysicsTests(unittest.TestCase):
    """
    Adversarial challenge test suite executing real empirical physics integration.
    """

    # =========================================================================
    # 1. ACCELERATION DYNAMICS ACROSS STEP SIZES
    # =========================================================================
    def _simulate_acceleration(self, dt: float, max_speed_kmh: float = 210.0, max_time_s: float = 8.0):
        """Simulates full-throttle acceleration from standstill recording time series."""
        car = Vehicle()
        car.state.vx = 0.0
        car.state.vy = 0.0
        car.state.yaw_rate = 0.0
        car.gear = 1
        car.rpm = RPM_IDLE
        car.powertrain.set_running()

        t_0_100 = None
        t_0_200 = None
        t_current = 0.0

        history = []

        while t_current < max_time_s and car.speed_kmh < max_speed_kmh:
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()

            telem = car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            t_current += dt

            history.append({
                "t": t_current,
                "speed_kmh": telem.speed_kmh,
                "vx": telem.speed_ms,
                "ax": car.state.ax,
                "gear": telem.gear,
                "rpm": telem.rpm,
                "slip_r": car.slip_ratio_rear,
                "shift_timer": car.powertrain.gearbox.shift_timer
            })

            if t_0_100 is None and telem.speed_kmh >= 100.0:
                t_0_100 = t_current
            if t_0_200 is None and telem.speed_kmh >= 200.0:
                t_0_200 = t_current

        return t_0_100, t_0_200, history

    def test_acceleration_across_step_sizes(self):
        """
        Verify 0-100 km/h in [2.40s, 2.80s] and 0-200 km/h in [4.80s, 5.50s]
        across dt = 0.001 (1000 Hz), dt = 0.005 (200 Hz), and dt = 0.008333 (120 Hz).
        """
        step_sizes = [
            (0.001000, "1000 Hz"),
            (0.005000, "200 Hz"),
            (1.0 / 120.0, "120 Hz (0.008333s)")
        ]

        print("\n=== [EMPIRICAL ACCELERATION ACROSS STEP SIZES] ===")
        for dt, label in step_sizes:
            t100, t200, history = self._simulate_acceleration(dt=dt)
            print(f"[{label}] 0-100 km/h: {t100:.4f}s | 0-200 km/h: {t200:.4f}s")

            # Check bounds for 0-100 km/h
            self.assertIsNotNone(t100, f"Vehicle failed to reach 100 km/h at {label}")
            self.assertGreaterEqual(t100, 2.40, f"0-100 km/h too fast ({t100:.4f}s < 2.40s) at {label}")
            self.assertLessEqual(t100, 2.80, f"0-100 km/h too slow ({t100:.4f}s > 2.80s) at {label}")

            # Check bounds for 0-200 km/h
            self.assertIsNotNone(t200, f"Vehicle failed to reach 200 km/h at {label}")
            self.assertGreaterEqual(t200, 4.80, f"0-200 km/h too fast ({t200:.4f}s < 4.80s) at {label}")
            self.assertLessEqual(t200, 5.50, f"0-200 km/h too slow ({t200:.4f}s > 5.50s) at {label}")

    # =========================================================================
    # 2. CONTINUITY OF ACCELERATION & TORQUE CURVES
    # =========================================================================
    def test_powertrain_torque_continuity_at_10500_rpm(self):
        """
        Empirically test whether EngineModel.compute_ice_torque() is continuous across 10,500 RPM.
        In powertrain.py:102-114, regime 1 (quadratic) gives 450.0 Nm at 10500 RPM,
        while regime 2 (power plateau) gives 500.0 Nm.
        This test checks that |t(10500.1) - t(10499.9)| < 1.0 Nm.
        """
        engine = EngineModel()
        engine.start_engine_direct()
        t_pre = engine.compute_ice_torque(10499.9, 1.0)
        t_post = engine.compute_ice_torque(10500.1, 1.0)
        delta_torque = abs(t_post - t_pre)
        print(f"\n[POWERTRAIN TORQUE AT 10500 RPM] Pre: {t_pre:.2f} Nm, Post: {t_post:.2f} Nm, Jump: {delta_torque:.2f} Nm")
        self.assertLess(
            delta_torque, 1.0,
            f"DISCONTINUITY BUG: Engine torque jumps by {delta_torque:.2f} Nm at 10500 RPM in powertrain.py:102-114!"
        )

    def test_acceleration_curve_continuity_in_gear4(self):
        """
        Adversarial check: In Gear 4, when engine crosses 10,500 RPM (v ~ 156.1 km/h),
        wheel torque is no longer clamped by tyre traction limits.
        The 50 Nm torque jump transmits directly to the driveline, producing an
        instantaneous single-step jump in acceleration (d_ax > 1.0 m/s^2 per millisecond).
        """
        dt = 0.001
        _, _, history = self._simulate_acceleration(dt=dt, max_speed_kmh=220.0, max_time_s=6.5)
        gear4 = [h for h in history if h["gear"] == 4]
        self.assertGreater(len(gear4), 100, "Insufficient Gear 4 data")

        # Exclude initial 40ms seamless shift dog-handover
        post_shift_records = gear4[42:]
        max_d_ax = 0.0
        jump_point = None

        for i in range(len(post_shift_records) - 1):
            d_ax = abs(post_shift_records[i + 1]["ax"] - post_shift_records[i]["ax"])
            if d_ax > max_d_ax:
                max_d_ax = d_ax
                jump_point = (post_shift_records[i], post_shift_records[i + 1])

        print(f"\n[GEAR 4 CONTINUITY] Max single-step delta_ax: {max_d_ax:.5f} m/s^2")
        if jump_point:
            print(f"Jump occurred at t={jump_point[0]['t']:.3f}s (spd={jump_point[0]['speed_kmh']:.1f} km/h, RPM={jump_point[0]['rpm']:.1f}): ax={jump_point[0]['ax']:.4f} -> {jump_point[1]['ax']:.4f} m/s^2")

        self.assertLess(
            max_d_ax, 0.20,
            f"DISCONTINUITY BUG in Gear 4: single-step ax jump of {max_d_ax:.4f} m/s^2 detected at {jump_point[0]['speed_kmh']:.1f} km/h!"
        )

    def test_vehicle_traction_sliding_step_discontinuity_in_gear3(self):
        """
        Adversarial check: In vehicle.py:514:
          fx_b = f_drive_act * (0.95 if abs(rear_slip_ratio) > 0.15 else 1.0)
        When rear_slip_ratio drops below 0.15 (at v ~ 146.4 km/h in Gear 3),
        drive force jumps discontinuously by 5%, causing a single-step ax jump of +0.83 m/s^2.
        """
        dt = 0.001
        _, _, history = self._simulate_acceleration(dt=dt, max_speed_kmh=220.0, max_time_s=6.5)
        gear3 = [h for h in history if h["gear"] == 3]
        self.assertGreater(len(gear3), 100, "Insufficient Gear 3 data")

        # Exclude initial 40ms seamless shift dog-handover
        post_shift_records = gear3[42:]
        max_d_ax = 0.0
        jump_point = None

        for i in range(len(post_shift_records) - 1):
            d_ax = abs(post_shift_records[i + 1]["ax"] - post_shift_records[i]["ax"])
            if d_ax > max_d_ax:
                max_d_ax = d_ax
                jump_point = (post_shift_records[i], post_shift_records[i + 1])

        print(f"\n[GEAR 3 CONTINUITY] Max single-step delta_ax: {max_d_ax:.5f} m/s^2")
        if jump_point:
            print(f"Jump occurred at t={jump_point[0]['t']:.3f}s (spd={jump_point[0]['speed_kmh']:.1f} km/h, slip_r={jump_point[0]['slip_r']:.5f} -> {jump_point[1]['slip_r']:.5f}): ax={jump_point[0]['ax']:.4f} -> {jump_point[1]['ax']:.4f} m/s^2")

        self.assertLess(
            max_d_ax, 0.20,
            f"DISCONTINUITY BUG in Gear 3: single-step ax jump of {max_d_ax:.4f} m/s^2 detected at {jump_point[0]['speed_kmh']:.1f} km/h due to vehicle.py:514 piecewise slip discount!"
        )

    # =========================================================================
    # 3. EMERGENCY BRAKING & DECELERATION DYNAMICS
    # =========================================================================
    def _simulate_braking(self, initial_speed_kmh: float = 200.0, bias: float = 0.58, dt: float = 0.001):
        """Simulates 100% emergency stop from initial speed."""
        car = Vehicle()
        car.state.vx = initial_speed_kmh / 3.6
        car.gear = 6
        car.brake_bias = bias

        distance = 0.0
        peak_decel_g = 0.0
        elapsed = 0.0
        min_vx = car.state.vx

        while elapsed < 5.0 and (car.state.vx > 0.001 or elapsed < 0.5):
            telem = car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            decel_g = abs(telem.longitudinal_g)
            if decel_g > peak_decel_g:
                peak_decel_g = decel_g

            distance += telem.speed_ms * dt
            elapsed += dt
            if car.state.vx < min_vx:
                min_vx = car.state.vx

            if car.state.vx == 0.0 and elapsed > 1.5:
                for _ in range(20):
                    car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
                    if car.state.vx < min_vx:
                        min_vx = car.state.vx
                break

        return distance, peak_decel_g, elapsed, car.state.vx, min_vx

    def test_emergency_braking_200_to_0_across_step_sizes_and_biases(self):
        """
        Verify 200-0 km/h emergency stopping distance < 55.0m and peak deceleration >= 4.5G
        across dt = 0.001, 0.005, 0.008333 and biases 52%, 58%, 64%.
        Also verify zero-crossing latch (no reverse motion, min_vx >= 0.0).
        """
        step_sizes = [
            (0.001000, "1000 Hz"),
            (0.005000, "200 Hz"),
            (1.0 / 120.0, "120 Hz (0.008333s)")
        ]
        biases = [0.52, 0.58, 0.64]

        print("\n=== [EMPIRICAL 200-0 km/h BRAKING MATRIX] ===")
        for dt, label in step_sizes:
            for bias in biases:
                dist, peak_g, t_stop, final_vx, min_vx = self._simulate_braking(
                    initial_speed_kmh=200.0, bias=bias, dt=dt
                )
                print(f"[{label} | Bias {bias*100:.0f}%] Dist: {dist:.2f}m | Peak Decel: {peak_g:.2f}G | Stop Time: {t_stop:.3f}s | Final vx: {final_vx:.6f} | Min vx: {min_vx:.6f}")

                self.assertLess(dist, 55.0, f"Stopping distance {dist:.2f}m >= 55.0m at {label}, bias {bias}")
                self.assertGreaterEqual(peak_g, 4.50, f"Peak deceleration {peak_g:.2f}G < 4.50G at {label}, bias {bias}")
                self.assertAlmostEqual(final_vx, 0.0, places=4, msg=f"Vehicle did not come to complete stop at {label}")
                self.assertGreaterEqual(min_vx, -0.0001, msg=f"Vehicle reversed into negative speed ({min_vx:.6f} m/s) at {label}!")

    # =========================================================================
    # 4. DRS TERMINAL SPEEDS & DELTA
    # =========================================================================
    def _simulate_top_speed(self, drs: bool, dt: float = 0.001, max_duration_s: float = 25.0):
        """Accelerates car to terminal top speed with or without DRS."""
        car = Vehicle()
        car.state.vx = 75.0
        car.gear = 7
        car.powertrain.set_running()
        car.drs_active = drs

        elapsed = 0.0
        while elapsed < max_duration_s:
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()
            car.drs_active = drs
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            elapsed += dt

        return car.speed_kmh, car.state.ax

    def test_drs_terminal_top_speeds_and_delta(self):
        """
        Verify:
        - DRS closed top speed >= 340.0 km/h
        - DRS open top speed >= 355.0 km/h
        - DRS top speed delta >= 15.0 km/h
        """
        print("\n=== [EMPIRICAL DRS TERMINAL TOP SPEEDS] ===")
        spd_closed, ax_closed = self._simulate_top_speed(drs=False, dt=0.001, max_duration_s=25.0)
        spd_open, ax_open = self._simulate_top_speed(drs=True, dt=0.001, max_duration_s=25.0)
        delta_spd = spd_open - spd_closed

        print(f"DRS Closed Top Speed: {spd_closed:.2f} km/h (ax={ax_closed:.4f} m/s^2, criterion >= 340 km/h)")
        print(f"DRS Open Top Speed:   {spd_open:.2f} km/h (ax={ax_open:.4f} m/s^2, criterion >= 355 km/h)")
        print(f"DRS Delta:            +{delta_spd:.2f} km/h (criterion >= 15 km/h)")

        self.assertGreaterEqual(spd_closed, 340.0, f"Closed DRS speed {spd_closed:.2f} km/h < 340.0 km/h")
        self.assertGreaterEqual(spd_open, 355.0, f"Open DRS speed {spd_open:.2f} km/h < 355.0 km/h")
        self.assertGreaterEqual(delta_spd, 15.0, f"DRS speed gain {delta_spd:.2f} km/h < 15.0 km/h")

    def test_aero_drag_smooth_quadratic_nature_and_no_cheat_step(self):
        """
        Dead end #8 had: base_cd = 0.82 if speed_pos > 65.0.
        Verify F_drag / v^2 is strictly constant across 20 m/s to 100 m/s.
        """
        aero = Aerodynamics()
        speeds = np.linspace(20.0, 100.0, 81)

        ratios_closed = []
        for v in speeds:
            _, f_drag, _, _ = aero.compute(v, drs_active=False)
            ratios_closed.append(f_drag / (v ** 2))

        ratios_open = []
        for v in speeds:
            _, f_drag, _, _ = aero.compute(v, drs_active=True)
            ratios_open.append(f_drag / (v ** 2))

        variation_closed = (max(ratios_closed) - min(ratios_closed)) / max(ratios_closed)
        variation_open = (max(ratios_open) - min(ratios_open)) / max(ratios_open)

        print(f"\nAero Drag F_drag/v^2 Closed variation: {variation_closed:.2e}")
        print(f"Aero Drag F_drag/v^2 Open variation:   {variation_open:.2e}")

        self.assertLess(variation_closed, 1e-6, "Aero drag is not pure quadratic (discontinuous drag detected!)")
        self.assertLess(variation_open, 1e-6, "Aero drag (DRS open) is not pure quadratic")

        reduction = (np.mean(ratios_closed) - np.mean(ratios_open)) / np.mean(ratios_closed)
        print(f"Measured Drag Reduction: {reduction*100:.2f}% (Config: 28.0%)")
        self.assertAlmostEqual(reduction, 0.28, places=3)

    # =========================================================================
    # 5. STANDSTILL STABILITY & ANTI-CREEP
    # =========================================================================
    def test_anti_creep_at_1000hz_and_120hz(self):
        """
        Verify vehicle does not creep forward at zero throttle and zero brake in 1st gear.
        Dead end #13 showed idle RPM coupled to rear wheels caused 0.72 km/h phantom creep.
        """
        print("\n=== [ANTI-CREEP STANDSTILL CHECK] ===")
        for dt, label in [(0.001, "1000 Hz"), (1.0 / 120.0, "120 Hz")]:
            car = Vehicle()
            car.state.vx = 0.0
            car.gear = 1
            car.rpm = RPM_IDLE
            car.powertrain.set_running()

            for _ in range(int(2.0 / dt)):
                telem = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)

            print(f"[{label}] Speed after 2.0s zero-throttle: {telem.speed_kmh:.6f} km/h (vx={car.state.vx:.6f} m/s)")
            self.assertEqual(telem.speed_kmh, 0.0, f"Phantom idle creep detected at {label}: {telem.speed_kmh} km/h")
            self.assertEqual(car.state.vx, 0.0)

    # =========================================================================
    # 6. REVERSE LOCKOUT & BRAKING DIRECTION
    # =========================================================================
    def test_reverse_lockout_and_reverse_braking(self):
        """
        1. Downshift to reverse while moving forward at > 1 km/h must be blocked.
        2. Moving in reverse and braking must decelerate towards 0 (ax > 0).
        """
        print("\n=== [REVERSE LOCKOUT & BRAKING DIRECTION] ===")
        car = Vehicle()
        car.state.vx = 5.0
        car.gear = 0
        ok = car.shift_down()
        print(f"Downshift N -> R at 18 km/h forward: allowed={ok}, gear={car.gear}")
        self.assertFalse(ok, "Downshift to reverse was allowed while moving forward!")
        self.assertEqual(car.gear, 0)

        car2 = Vehicle()
        car2.state.vx = -10.0
        car2.step(throttle=0.0, brake=1.0, steer=0.0, dt=1.0 / 120.0)
        print(f"Reverse braking at vx=-10 m/s: ax={car2.state.ax:.2f} m/s^2 (must be positive to oppose motion)")
        self.assertGreater(car2.state.ax, 0.0, "Braking in reverse accelerated vehicle backwards!")


if __name__ == "__main__":
    unittest.main()
