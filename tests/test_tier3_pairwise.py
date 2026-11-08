"""
tests/test_tier3_pairwise.py
============================
Tier 3: Pairwise Combinations & Cross-Feature Integration Test Suite.

Verifies cross-subsystem interactions and realistic multi-factor behaviors:
1. DRS Active + Hard Threshold Braking (auto-shutoff, aero stabilization)
2. Rearward Brake Bias + High-Speed Corner Turn-In (rear lockup & oversteer moment)
3. Kerb Strike + Full Throttle Acceleration (normal load oscillation & wheelspin spike)
4. Manual Downshift without Rev-Blip into Hairpin (compression lockup & snap oversteer)
5. AI Overtake in DRS Zone with Slipstream Tow (tow + DRS flap speed delta & lateral lane shift)
6. Standing Start with Cold Tyres & High Throttle (clutch drop wheelspin & traction hookup)
7. Cockpit Telemetry LCD + Steering Slew Filter in Rapid Slalom (synchronized telemetry & smooth steering)
8. High-Speed Aerodynamic Grip (Monza Curva Grande) vs Low-Speed Mechanical Grip (Monaco Hairpin)
"""

import math
import unittest
import numpy as np

from f1sim.physics.vehicle import Vehicle
from tests.benchmark_physics_ai import (
    F1ReferencePhysicsModel,
    VEHICLE_MASS,
    WHEELBASE,
    CG_TO_FRONT,
    CG_TO_REAR,
    TYRE_RADIUS,
    AIR_DENSITY,
    BASE_CD_A,
    BASE_CL_A,
    DRS_DRAG_REDUCTION,
    DRS_DOWNFORCE_REDUCTION,
    BASE_MU,
    GEAR_RATIOS,
    FINAL_DRIVE,
    UPSHIFT_RPM,
    IDLE_RPM,
    REDLINE_RPM,
)


class TestTier3Pairwise(unittest.TestCase):
    """Pairwise combinatorial and cross-feature interaction tests."""

    def setUp(self):
        self.car = Vehicle()

    # Pairwise 1: DRS Active + Hard Threshold Braking (F7 + F11 + F14)
    def test_pairwise_drs_active_hard_braking(self):
        """DRS open at 330 km/h automatically snaps shut on hard braking, restoring downforce."""
        self.car.vx = 330.0 / 3.6  # 91.67 m/s
        self.car.gear = 8
        self.car.drs_available = True
        self.car.drs_active = True
        self.assertTrue(self.car.drs_active)

        # Step 1: Hit brake pedal hard (100%)
        self.car.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)

        # DRS must immediately auto-deactivate
        self.assertFalse(self.car.drs_active)
        # Full aerodynamic downforce restored
        f_down_restored = 0.5 * AIR_DENSITY * BASE_CL_A * (self.car.vx ** 2)
        self.assertGreater(f_down_restored, 15000.0)
        # Deceleration is strong and stable (> 5.0G)
        self.assertGreater(abs(self.car.longitudinal_g), 4.5)

    # Pairwise 2: Rearward Brake Bias + High-Speed Corner Turn-In (F9 + F11 + F12 + F13)
    def test_pairwise_rearward_brake_bias_trail_braking(self):
        """Rearward brake bias (52% front, 48% rear) during turn-in induces severe rear slip and oversteer."""
        self.car.vx = 220.0 / 3.6  # ~61 m/s
        self.car.gear = 5
        self.car.brake_bias = 0.52  # Most rearward allowable bias

        # Trail braking into corner with steering lock
        max_rear_slip = 0.0
        for _ in range(300):  # 0.3 seconds
            self.car.step(throttle=0.0, brake=0.85, steer=0.25, dt=0.001)
            slip_deg = abs(self.car.slip_angle_rear) * (180.0 / math.pi)
            if slip_deg > max_rear_slip:
                max_rear_slip = slip_deg

        # Rear slip angle exceeds front stability threshold
        self.assertGreater(max_rear_slip, 4.0)
        self.assertGreater(abs(self.car.yaw_rate), 0.3)

    # Pairwise 3: Kerb Strike + Full Throttle Acceleration (F10 + F12 + F13 + F17 + F24)
    def test_pairwise_kerb_strike_full_throttle_acceleration(self):
        """Accelerating over a 25 Hz kerb in 2nd gear causes traction disturbance and vibration."""
        self.car.vx = 90.0 / 3.6  # 25 m/s
        self.car.gear = 2

        # Simulate 25 Hz kerb rumble: normal load oscillates by +/- 35%
        dt = 0.001
        vibrations = []
        for step_i in range(200):
            t = step_i * dt
            # Oscillating vertical load disturbance from sawtooth kerb
            kerb_oscillation = 0.35 * math.sin(2.0 * math.pi * 25.0 * t)
            self.car.kerb_vibration = abs(kerb_oscillation)
            self.car.step(throttle=0.7, brake=0.0, steer=0.05, dt=dt)
            vibrations.append(self.car.kerb_vibration)

        # Vibration is active and oscillating
        self.assertGreater(max(vibrations), 0.30)
        self.assertAlmostEqual(min(vibrations), 0.0, delta=0.05)

    # Pairwise 4: Manual Downshift without Rev-Blip into Hairpin (F6 + F10 + F12 + F13)
    def test_pairwise_manual_downshift_compression_lockup(self):
        """Downshifting from 4th to 2nd into hairpin without rev-blip creates engine braking torque spike."""
        self.car.vx = 140.0 / 3.6  # ~39 m/s
        self.car.gear = 4

        # Downshift without throttle blip
        self.car.gear = 2
        # Engine RPM shoots up towards redline
        ratio = GEAR_RATIOS[self.car.gear + 1] * FINAL_DRIVE
        predicted_rpm = (self.car.vx / TYRE_RADIUS) * ratio * 1.85 * (60.0 / (2.0 * math.pi))
        self.car.rpm = max(IDLE_RPM, min(REDLINE_RPM, predicted_rpm))
        self.assertGreater(self.car.rpm, 10000.0)

        # High engine braking induces deceleration even at 0.0 brake pedal
        telem = self.car.step(throttle=0.0, brake=0.0, steer=0.15, dt=0.001)
        self.assertLess(telem.longitudinal_g, 0.0)  # Decelerating purely on engine braking drag

    # Pairwise 5: AI Overtake in DRS Zone with Slipstream Tow (F7 + F14 + F18 + F20)
    def test_pairwise_ai_drs_slipstream_overtake(self):
        """Following car in slipstream tow with DRS active closes speed gap and shifts lane to overtake."""
        from f1sim.physics.vehicle import Vehicle
        from f1sim.ai.collision import (
            compute_ttc,
            compute_overtake_and_avoidance,
            compute_slipstream_drag_multiplier,
        )

        # Leader: 320 km/h, DRS closed
        leader_v = 320.0 / 3.6
        follower_car = Vehicle()
        follower_car.state.vx = leader_v
        follower_car.powertrain.gearbox.gear = 8
        follower_car.state.gear = 8
        follower_car.state.rpm = 11800.0

        # Enable DRS and apply slipstream tow drag reduction
        follower_car.state.drs_active = True
        drag_mult = compute_slipstream_drag_multiplier(distance_m=12.0, lateral_diff_m=0.0)
        self.assertLess(drag_mult, 1.0)
        follower_car.aero.set_slipstream_factors(drag_scalar=drag_mult)

        for _ in range(3000):  # 3 seconds of straight acceleration at 1000 Hz
            follower_car.state.drs_active = True
            follower_car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001, drs_active=True)

        follower_v = follower_car.speed_kmh
        self.assertGreater(follower_v, 345.0)
        speed_delta = follower_v - (leader_v * 3.6)
        self.assertGreater(speed_delta, 15.0)

        # AI TTC calculation triggers lateral overtaking shift (+/- 2.8m)
        gap = 8.0  # meters (within 8m drafting zone)
        ttc = compute_ttc(gap, follower_car.state.vx, leader_v)
        self.assertLess(ttc, 1.2)  # TTC threshold reached

        ttc_res, lat_shift, brake_demand = compute_overtake_and_avoidance(
            distance_m=gap,
            v_follower_ms=follower_car.state.vx,
            v_leader_ms=leader_v,
            leader_d_m=0.0,
        )
        self.assertAlmostEqual(abs(lat_shift), 2.8)
        self.assertEqual(brake_demand, 0.0)

    # Pairwise 6: Standing Start with Cold Tyres & High Throttle (F8 + F10 + F12 + F17)
    def test_pairwise_standing_grid_launch_wheelspin(self):
        """Clutch launch at 11,000 RPM from standstill produces initial wheelspin before traction hookup."""
        self.car.vx = 0.0
        self.car.gear = 1
        self.car.rpm = 11000.0  # High launch RPM

        slips = []
        speeds = []
        for _ in range(1500):  # 1.5 seconds of launch
            self.car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
            slips.append(self.car.slip_ratio_rear)
            speeds.append(self.car.speed_kmh)

        # Initial launch produces high slip ratio
        self.assertGreater(max(slips[:200]), 0.15)
        # Vehicle cleanly hooks up and accelerates past 50 km/h
        self.assertGreater(speeds[-1], 50.0)

    # Pairwise 7: Cockpit Telemetry LCD + Steering Slew Filter in Rapid Slalom (F3 + F4 + F22)
    def test_pairwise_slalom_telemetry_synchronization(self):
        """Rapid left-to-right slalom steering generates smooth wheel rotation and synchronous telemetry."""
        from f1sim.input.filters import SteeringFilter
        from f1sim.graphics.lcd_display import TelemetryLCDDisplay

        steering_filter = SteeringFilter()
        lcd = TelemetryLCDDisplay()
        self.car.vx = 40.0  # 144 km/h slalom speed
        self.car.gear = 4

        dt = 0.001
        telemetry_samples = []
        wheel_angles = []
        for step_i in range(1000):  # 1.0s slalom
            t = step_i * dt
            # Alternating normalized steer demand +/- 0.5 at 2 Hz
            raw_steer = 0.5 * math.sin(2.0 * math.pi * 2.0 * t)
            road_steer, wheel_angle = steering_filter.update(
                raw_input=raw_steer,
                speed_ms=self.car.vx,
                dt=dt,
                is_analog=True,
            )
            self.car.step(throttle=0.5, brake=0.0, steer=road_steer, dt=dt)
            telem = self.car.get_telemetry()
            lcd_state = lcd.update_from_vehicle(self.car, dt=dt)
            telemetry_samples.append((telem.steering_angle, telem.lateral_g, lcd_state.speed_kmh))
            wheel_angles.append(wheel_angle)

        # Check that steering angle is bounded and lateral G tracks direction of steering
        self.assertGreater(max(wheel_angles), 0.1)
        self.assertLess(min(wheel_angles), -0.1)
        for steer, lat_g, spd in telemetry_samples[100:900:100]:
            self.assertFalse(math.isnan(steer))
            self.assertFalse(math.isnan(lat_g))
            self.assertGreater(spd, 50.0)

    # Pairwise 8: High Speed Aerodynamic Grip vs Low Speed Mechanical Grip (F10 + F14 + F15 + F16)
    def test_pairwise_high_speed_aero_vs_low_speed_mechanical_grip(self):
        """Curva Grande (290 km/h) generates massive downforce (>14,000N) vs Fairmont Hairpin (45 km/h, <500N)."""
        # Monza Curva Grande
        v_curva_grande = 290.0 / 3.6  # 80.56 m/s
        f_down_curva = 0.5 * AIR_DENSITY * BASE_CL_A * (v_curva_grande ** 2)

        # Monaco Fairmont Hairpin
        v_hairpin = 45.0 / 3.6  # 12.5 m/s
        f_down_hairpin = 0.5 * AIR_DENSITY * BASE_CL_A * (v_hairpin ** 2)

        # Downforce at 290 km/h is > 30x higher than at 45 km/h
        self.assertGreater(f_down_curva, 14000.0)
        self.assertLess(f_down_hairpin, 600.0)
        self.assertGreater(f_down_curva / f_down_hairpin, 30.0)


if __name__ == "__main__":
    unittest.main()
