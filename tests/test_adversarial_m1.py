"""
tests/test_adversarial_m1.py
============================
Independent Adversarial Stress & Verification Suite for Milestone 1:
Acceleration, Powertrain, DRS & Braking Dynamics.

Authored by Empirical Challenger (teamwork_challenger).
Tests:
1. Standing launch under varying grip conditions (mu = 0.70 .. 1.20) at 1000 Hz and 120 Hz.
   Confirms 0-100 km/h is strictly in [2.40, 2.80]s and 0-200 km/h is strictly in [4.80, 5.50]s on optimal track (mu=1.0).
2. Emergency braking 200 -> 0 km/h across varying brake bias settings (52%, 58%, 64%) and clamped boundaries.
   Confirms stopping distance < 55.0m and peak deceleration > 4.5G across all bias settings.
3. DRS aerodynamics & safety interlocks:
   Confirms drag reduction is ~28% (27% - 29%), top speed > 355 km/h on straightaway,
   and instant auto-close on braking (> 0.05) and lateral G (> 0.40G).
4. Adversarial stress & boundary robustness:
   - Standstill stability (zero drift/creep)
   - Input clamping robustness (NaN/Inf resistance, negative & excessive inputs)
   - Left-foot braking / simultaneous throttle + brake
   - Multi-cycle hard braking thermal progression
   - Reverse motion braking direction
   - Reverse gear downshift lockout at forward speed
"""

import math
import unittest
import numpy as np

from f1sim.core.constants import (
    TransmissionMode, RPM_IDLE, RPM_AUTO_UPSHIFT,
    BRAKE_BIAS_MIN, BRAKE_BIAS_MAX, BRAKE_BIAS_DEFAULT
)
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry
from f1sim.physics.aero import Aerodynamics, DRSState, AeroConfig
from f1sim.physics.brakes import BrakeSystem, BrakeConfig
from f1sim.physics.powertrain import Powertrain, TransmissionMode


class TestAdversarialAcceleration(unittest.TestCase):
    """Adversarial testing of standing launch & acceleration dynamics."""

    def _run_launch(self, surface_mu: float = 1.0, dt: float = 0.001) -> tuple[float, float, list[float]]:
        """Simulates standing launch to 200 km/h."""
        car = Vehicle()
        car.state.vx = 0.0
        car.state.vy = 0.0
        car.state.yaw_rate = 0.0
        car.gear = 1
        car.rpm = RPM_IDLE
        car.powertrain.set_running()

        t_0_100 = None
        t_0_200 = None
        elapsed = 0.0
        max_time = 10.0
        speeds = []
        mu_vec = np.full(4, surface_mu, dtype=np.float64)

        while elapsed < max_time and (t_0_200 is None):
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()
            telem = car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt, surface_mu=mu_vec)
            elapsed += dt
            speeds.append(telem.speed_kmh)

            if t_0_100 is None and telem.speed_kmh >= 100.0:
                t_0_100 = elapsed
            if t_0_200 is None and telem.speed_kmh >= 200.0:
                t_0_200 = elapsed

        return t_0_100 or elapsed, t_0_200 or elapsed, speeds

    def test_optimal_launch_strict_bounds_1000hz(self):
        """Confirm 0-100 km/h in [2.40, 2.80]s and 0-200 km/h in [4.80, 5.50]s at 1000 Hz."""
        t100, t200, _ = self._run_launch(surface_mu=1.0, dt=0.001)
        print(f"\n[EMPIRICAL 1000Hz Launch] 0-100 km/h: {t100:.4f}s, 0-200 km/h: {t200:.4f}s")
        self.assertGreaterEqual(t100, 2.40, f"0-100 too fast: {t100:.4f}s < 2.40s")
        self.assertLessEqual(t100, 2.80, f"0-100 too slow: {t100:.4f}s > 2.80s")
        self.assertGreaterEqual(t200, 4.80, f"0-200 too fast: {t200:.4f}s < 4.80s")
        self.assertLessEqual(t200, 5.50, f"0-200 too slow: {t200:.4f}s > 5.50s")

    def test_optimal_launch_strict_bounds_120hz(self):
        """Confirm 0-100 km/h in [2.40, 2.80]s and 0-200 km/h in [4.80, 5.50]s at game tick 120 Hz."""
        dt = 1.0 / 120.0
        t100, t200, _ = self._run_launch(surface_mu=1.0, dt=dt)
        print(f"[EMPIRICAL 120Hz Launch] 0-100 km/h: {t100:.4f}s, 0-200 km/h: {t200:.4f}s")
        self.assertGreaterEqual(t100, 2.40, f"120Hz 0-100 too fast: {t100:.4f}s < 2.40s")
        self.assertLessEqual(t100, 2.80, f"120Hz 0-100 too slow: {t100:.4f}s > 2.80s")
        self.assertGreaterEqual(t200, 4.80, f"120Hz 0-200 too fast: {t200:.4f}s < 4.80s")
        self.assertLessEqual(t200, 5.50, f"120Hz 0-200 too slow: {t200:.4f}s > 5.50s")

    def test_varying_grip_sensitivity(self):
        """Adversarial grip sweep (mu = 0.70 .. 1.20): verify that lower grip produces lower acceleration."""
        mu_values = [0.70, 0.80, 0.90, 1.00, 1.10, 1.20]
        results = {}
        for mu in mu_values:
            t100, t200, _ = self._run_launch(surface_mu=mu, dt=0.001)
            results[mu] = (t100, t200)
            print(f"[Grip mu={mu:.2f}] 0-100 km/h: {t100:.4f}s | 0-200 km/h: {t200:.4f}s")

        # Low grip must be physically slower than optimal grip
        self.assertGreater(results[0.70][0], results[1.00][0],
                           f"Bug: 0.70 grip ({results[0.70][0]:.4f}s) was not slower than 1.00 grip ({results[1.00][0]:.4f}s). surface_mu is ignored!")

    def test_standstill_stability_zero_throttle_zero_brake(self):
        """At zero throttle and zero brake on flat surface, car must remain stationary (no creeping)."""
        car = Vehicle()
        car.state.vx = 0.0
        car.state.vy = 0.0
        car.state.yaw_rate = 0.0
        car.gear = 1
        car.rpm = RPM_IDLE

        for _ in range(120):  # 1 second of simulation
            telem = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=1.0 / 120.0)

        self.assertAlmostEqual(telem.speed_kmh, 0.0, places=2,
                               msg=f"Bug: Car crept from standstill to {telem.speed_kmh:.2f} km/h with zero throttle!")


class TestAdversarialBraking(unittest.TestCase):
    """Adversarial testing of emergency braking dynamics across brake biases."""

    def _run_emergency_braking(self, bias: float, dt: float = 0.001) -> tuple[float, float, float]:
        """Brakes from 200 km/h to 0 km/h with 100% brake demand."""
        car = Vehicle()
        car.state.vx = 200.0 / 3.6  # 55.556 m/s
        car.gear = 6
        car.brake_bias = bias
        distance = 0.0
        peak_g = 0.0
        elapsed = 0.0

        while car.speed_kmh > 0.05 and elapsed < 5.0:
            telem = car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)
            decel_g = abs(telem.longitudinal_g)
            if decel_g > peak_g:
                peak_g = decel_g
            distance += telem.speed_ms * dt
            elapsed += dt

        return distance, peak_g, elapsed

    def test_braking_across_all_bias_settings_1000hz(self):
        """Verify stopping distance < 55m and peak decel > 4.5G across 52%, 58%, and 64% bias at 1000 Hz."""
        biases = [0.52, 0.58, 0.64]
        for bias in biases:
            dist, peak_g, t_stop = self._run_emergency_braking(bias=bias, dt=0.001)
            print(f"\n[EMPIRICAL Braking Bias {bias*100:.0f}%] Distance: {dist:.2f}m, Peak Decel: {peak_g:.2f}G, Time: {t_stop:.3f}s")
            self.assertLess(dist, 55.0, f"Stopping distance {dist:.2f}m >= 55.0m at bias {bias}")
            self.assertGreater(peak_g, 4.50, f"Peak decel {peak_g:.2f}G <= 4.50G at bias {bias}")

    def test_braking_bias_120hz_frequency(self):
        """Verify stopping distance < 55m and vehicle comes to rest without reversing at 120 Hz game loop."""
        dt = 1.0 / 120.0
        for bias in [0.52, 0.58, 0.64]:
            dist, peak_g, t_stop = self._run_emergency_braking(bias=bias, dt=dt)
            print(f"[EMPIRICAL 120Hz Braking Bias {bias*100:.0f}%] Distance: {dist:.2f}m, Peak Decel: {peak_g:.2f}G, Time: {t_stop:.3f}s")
            self.assertLess(dist, 55.0, f"120Hz Stopping distance {dist:.2f}m >= 55.0m at bias {bias}. Car reversed or failed to stop!")
            self.assertGreater(peak_g, 4.50, f"120Hz Peak decel {peak_g:.2f}G <= 4.50G at bias {bias}")

    def test_reverse_motion_braking_direction(self):
        """When vehicle is moving in reverse (vx < 0), applying brakes must decelerate towards zero, not accelerate backwards."""
        car = Vehicle()
        car.state.vx = -5.0  # -18 km/h in reverse
        dt = 1.0 / 120.0
        car.step(throttle=0.0, brake=1.0, steer=0.0, dt=dt)

        self.assertGreater(car.state.ax, 0.0,
                           f"Bug: Longitudinal acceleration {car.state.ax:.2f} m/s^2 was negative while reversing with brake applied! Brakes acted as reverse accelerator.")

    def test_brake_bias_clamping_robustness(self):
        """Setting extreme out-of-range brake biases must clamp both vehicle state and brake system to [0.52, 0.64]."""
        car = Vehicle()
        car.brake_bias = 0.30
        self.assertAlmostEqual(car.state.brake_bias, BRAKE_BIAS_MIN, places=2,
                               msg=f"Bug: car.state.brake_bias {car.state.brake_bias} was not clamped to {BRAKE_BIAS_MIN}!")
        self.assertAlmostEqual(car.brake_bias, BRAKE_BIAS_MIN, places=2)

        car.brake_bias = 0.85
        self.assertAlmostEqual(car.state.brake_bias, BRAKE_BIAS_MAX, places=2)
        self.assertAlmostEqual(car.brake_bias, BRAKE_BIAS_MAX, places=2)

    def test_repeated_hard_braking_thermal_dissipation(self):
        """Multiple 200-0 stops heat up carbon discs; test thermal accumulation."""
        car = Vehicle()
        initial_temps = car.brakes.get_temperatures().copy()

        for stop_idx in range(3):
            car.state.vx = 200.0 / 3.6
            for _ in range(2500):  # 2.5s hard stop
                car.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)

        post_temps = car.brakes.get_temperatures()
        print(f"[Thermal Stress] Initial disc temp: {initial_temps[0]:.1f}°C -> Post 3-stops temp: {post_temps[0]:.1f}°C")
        self.assertGreater(post_temps[0], initial_temps[0] + 50.0, "Discs did not heat up after 3 emergency stops")
        self.assertLess(post_temps[0], 1200.0, "Discs suffered unphysical thermal runaway")


class TestAdversarialDRSAerodynamics(unittest.TestCase):
    """Adversarial testing of DRS drag reduction, top speed, and safety auto-close."""

    def test_drag_reduction_is_approximately_28_percent(self):
        """Empirically measure drag reduction across speed spectrum (100 to 350 km/h)."""
        aero = Aerodynamics()
        speeds_kmh = [120.0, 180.0, 240.0, 300.0, 350.0]

        for spd_kmh in speeds_kmh:
            v_ms = spd_kmh / 3.6
            _, f_drag_closed, _, _ = aero.compute(v_ms, drs_active=False)
            _, f_drag_open, _, _ = aero.compute(v_ms, drs_active=True)

            drag_reduction_pct = (f_drag_closed - f_drag_open) / f_drag_closed * 100.0
            print(f"[DRS Drag at {spd_kmh:.0f} km/h] Closed: {f_drag_closed:.1f}N | Open: {f_drag_open:.1f}N | Reduction: {drag_reduction_pct:.2f}%")
            self.assertAlmostEqual(drag_reduction_pct, 28.0, delta=1.0,
                                   msg=f"DRS drag reduction {drag_reduction_pct:.2f}% not within ~28%")

    def test_top_speed_exceeds_355_kmh_with_drs(self):
        """Simulate straightaway top speed run with DRS open; verify v > 355 km/h."""
        car = Vehicle()
        car.vx = 80.0  # 288 km/h entry speed in 7th gear
        car.gear = 7
        car.drs_active = True
        dt = 0.001

        # Accelerate for 15 seconds with DRS forced open
        for _ in range(15000):
            if car.rpm >= 12800.0 and car.gear < 8:
                car.shift_up()
            car.drs_active = True
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)

        top_speed_kmh = car.speed_kmh
        print(f"\n[EMPIRICAL Top Speed DRS Open] Terminal speed: {top_speed_kmh:.2f} km/h")
        self.assertGreater(top_speed_kmh, 355.0, f"DRS top speed {top_speed_kmh:.2f} km/h did not exceed 355 km/h")

    def test_drs_auto_close_on_brake_threshold(self):
        """DRS must remain active at brake <= 0.05 and close immediately at brake > 0.05."""
        car = Vehicle()
        car.vx = 80.0
        car.gear = 7
        car.drs_active = True

        # Test brake just below threshold (0.04) -> DRS remains active
        car.step(throttle=0.0, brake=0.04, steer=0.0, dt=0.001, drs_active=True)
        self.assertTrue(car.drs_active, "DRS closed prematurely at brake = 0.04 (<= 0.05)")

        # Test brake at threshold (0.05) -> DRS remains active
        car.step(throttle=0.0, brake=0.05, steer=0.0, dt=0.001, drs_active=True)
        self.assertTrue(car.drs_active, "DRS closed at brake = 0.05 (threshold is strictly > 0.05)")

        # Test brake just above threshold (0.051) -> DRS MUST immediately auto-close
        car.step(throttle=0.0, brake=0.051, steer=0.0, dt=0.001, drs_active=True)
        self.assertFalse(car.drs_active, "DRS failed to auto-close at brake = 0.051 (> 0.05)!")

    def test_drs_auto_close_on_lateral_g(self):
        """DRS must auto-close if lateral acceleration exceeds 0.40G."""
        car = Vehicle()
        car.vx = 60.0  # 216 km/h
        car.drs_active = True

        # Sharp steering creating high lateral acceleration
        car.step(throttle=0.0, brake=0.0, steer=0.35, dt=0.001, drs_active=True)
        if abs(car.lateral_g) > 0.40:
            # Next step must have auto-closed DRS
            car.step(throttle=0.0, brake=0.0, steer=0.35, dt=0.001)
            self.assertFalse(car.drs_active, f"DRS remained active under high lateral G ({car.lateral_g:.2f}G > 0.40G)!")


class TestAdversarialPowertrainAndGearbox(unittest.TestCase):
    """Stress tests for gearbox shifting, reverse lockout, and input boundary protection."""

    def test_reverse_gear_lockout_at_forward_speed(self):
        """Downshifting from Neutral into Reverse while vehicle is moving forward (v > 1 km/h) must be rejected."""
        car = Vehicle()
        car.state.vx = 20.0  # 72 km/h forward
        car.gear = 0         # Neutral
        shift_ok = car.shift_down()
        self.assertFalse(shift_ok, f"Bug: Downshift from Neutral to Reverse was accepted at {car.speed_kmh:.1f} km/h forward speed!")
        self.assertEqual(car.gear, 0, f"Bug: Gear changed from 0 to {car.gear} while moving forward!")

    def test_negative_and_overflow_inputs(self):
        """Verify physics engine does not crash or generate NaN/Inf with hostile inputs."""
        car = Vehicle()
        car.state.vx = 30.0

        for thr, brk, str_val in [(-1.0, -1.0, -5.0), (5.0, 5.0, 10.0)]:
            telem = car.step(throttle=thr, brake=brk, steer=str_val, dt=0.001)
            self.assertFalse(math.isnan(telem.speed_kmh))
            self.assertFalse(math.isinf(telem.speed_kmh))
            self.assertGreaterEqual(telem.throttle, 0.0)
            self.assertLessEqual(telem.throttle, 1.0)
            self.assertGreaterEqual(telem.brake, 0.0)
            self.assertLessEqual(telem.brake, 1.0)

    def test_simultaneous_full_throttle_and_full_brake(self):
        """100% throttle + 100% brake (left-foot braking / burnout): must produce finite valid telemetry."""
        car = Vehicle()
        car.state.vx = 40.0
        car.gear = 3

        for _ in range(100):
            telem = car.step(throttle=1.0, brake=1.0, steer=0.0, dt=0.001)

        self.assertFalse(math.isnan(telem.speed_kmh))
        self.assertFalse(math.isinf(telem.speed_kmh))
        self.assertLess(telem.speed_kmh, 40.0 * 3.6)


if __name__ == "__main__":
    unittest.main()
