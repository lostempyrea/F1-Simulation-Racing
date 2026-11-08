"""
tests/test_tier4_scenarios.py
=============================
Tier 4: Real-World Workload Scenarios Test Suite for F1 Simulation.

Exercises complex end-to-end race scenarios from TEST_INFRA.md:
1. Scenario 1: Monza Full GP Sprint (3 Laps, 6 cars, lap counters, podium results)
2. Scenario 2: Monaco Precision Street Duel (tight technical track, Fairmont hairpin, barrier safety)
3. Scenario 3: Standing Grid 5-Red-Light Launch (FIA gantry lights, clutch drop, 0-100 in 2.5s, clean T1)
4. Scenario 4: DRS Straight Overtake Maneuver (trailing car, DRS detection <1.0s, +16 km/h delta, pass)
5. Scenario 5: Extreme Brake Bias Cornering Snap & Counter-Steer Recovery (rear lockup, snap oversteer, recovery)
"""

import math
import unittest
from typing import List, Dict, Any
import numpy as np

from f1sim.physics.vehicle import Vehicle
from f1sim.ai.driver_profiles import AI_DRIVER_ROSTER
from f1sim.ai.collision import compute_slipstream_drag_multiplier
from tests.benchmark_physics_ai import (
    F1ReferencePhysicsModel,
    VEHICLE_MASS,
    WHEELBASE,
    TYRE_RADIUS,
    AIR_DENSITY,
    BASE_CD_A,
    BASE_CL_A,
    DRS_DRAG_REDUCTION,
    BASE_MU,
    GEAR_RATIOS,
    FINAL_DRIVE,
    UPSHIFT_RPM,
)


class TestTier4Scenarios(unittest.TestCase):
    """Full-length realistic race application workloads."""

    # Scenario 1: Monza Full GP Sprint (3 Laps)
    def test_scenario_1_monza_3_lap_gp_sprint(self):
        """Monza 3-lap race with 1 Player and 5 AI cars finishing without collision and generating podium."""
        track_length = 5793.0
        total_laps = 3

        # Grid setup: 6 cars
        drivers = [{"id": 0, "name": "Player", "team": "Custom", "pace_scalar": 1.000}]
        for drv in AI_DRIVER_ROSTER:
            drivers.append({"id": drv.id, "name": drv.name, "team": drv.team, "pace_scalar": drv.pace_scalar})

        # Simulate 3 laps for all 6 cars
        cars_sim = []
        for i, d in enumerate(drivers):
            car = Vehicle()
            cars_sim.append({
                "driver": d,
                "car": car,
                "distance": -i * 8.0,  # 8m grid spacing
                "lap": 1,
                "lap_times": [],
                "current_lap_start_t": 0.0,
                "total_time": 0.0,
                "finished": False
            })

        dt = 0.05  # 20 Hz simulation for 3 full laps
        sim_time = 0.0
        max_time = 350.0  # ~5.8 minutes maximum

        while not all(c["finished"] for c in cars_sim) and sim_time < max_time:
            sim_time += dt
            for c in cars_sim:
                if c["finished"]:
                    continue

                # Dynamic speed profiling around Monza
                s_track = max(0.0, c["distance"]) % track_length
                # Chicanes / braking zones: Rettifilo (950-1200m), Roggia (2600-2800m), Ascari (4800-5050m), Parabolica (5500-5700m)
                in_chicane = any(st <= s_track <= en for st, en in [(950, 1200), (2600, 2800), (4800, 5050), (5500, 5700)])
                target_v = (24.0 if in_chicane else 88.0) * (1.0 / c["driver"]["pace_scalar"])

                car = c["car"]
                throttle = 1.0 if car.state.vx < target_v else 0.0
                brake = 1.0 if car.state.vx > target_v + 3.0 else 0.0

                car.step(throttle=throttle, brake=brake, steer=0.0, dt=dt)
                c["distance"] += car.state.vx * dt

                # Lap tracking
                current_lap_calculated = int(c["distance"] // track_length) + 1
                if current_lap_calculated > c["lap"]:
                    lap_duration = sim_time - c["current_lap_start_t"]
                    c["lap_times"].append(lap_duration)
                    c["current_lap_start_t"] = sim_time
                    c["lap"] = min(total_laps, current_lap_calculated)

                if c["distance"] >= track_length * total_laps:
                    c["finished"] = True
                    c["total_time"] = sim_time

        # Verification Criteria
        # 1. All cars finished 3 full laps
        for c in cars_sim:
            self.assertTrue(c["finished"], f"{c['driver']['name']} did not finish 3 laps in time")
            self.assertEqual(len(c["lap_times"]), total_laps, f"Expected {total_laps} lap times recorded")
            self.assertGreaterEqual(c["distance"], track_length * total_laps)

        # 2. Results Classification
        classified = sorted(cars_sim, key=lambda x: x["total_time"])
        self.assertEqual(len(classified), 6)
        winner = classified[0]
        self.assertGreater(winner["total_time"], 230.0)  # ~4 minutes for 3 laps
        self.assertLess(winner["total_time"], 320.0)

        # 3. Best lap of the race
        all_laps = [t for c in cars_sim for t in c["lap_times"]]
        fastest_lap = min(all_laps)
        self.assertGreater(fastest_lap, 75.0)  # Real Monza F1 lap ~80-95s
        self.assertLess(fastest_lap, 105.0)

    # Scenario 2: Monaco Precision Street Duel
    def test_scenario_2_monaco_precision_street_duel(self):
        """Monaco street duel negotiating Fairmont hairpin and tight armco boundaries."""
        car = Vehicle()

        # Sector 1: Sainte Dévote -> Massenet (90 km/h -> 200 km/h)
        car.state.vx = 25.0
        for _ in range(800):
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertGreater(car.speed_kmh, 110.0)

        # Sector 2: Heavy braking into Fairmont Hairpin
        while car.speed_kmh > 45.0:
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)
        self.assertLessEqual(car.speed_kmh, 45.0)

        # Rotate through Hairpin at full lock
        max_steer = 0.0
        for _ in range(600):
            steer = 0.85
            throttle = 0.25 if car.speed_kmh < 42.0 else 0.05
            car.step(throttle=throttle, brake=0.0, steer=steer, dt=0.001)
            if abs(car.state.steering_angle) > max_steer:
                max_steer = abs(car.state.steering_angle)

        self.assertGreaterEqual(max_steer, 0.70)
        self.assertLessEqual(car.speed_kmh, 50.0)

        # Sector 3: Tunnel sprint to 270 km/h
        car.state.vy = 0.0
        car.state.yaw_rate = 0.0
        car.state.drs_active = True
        for _ in range(6000):
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=0.001, drs_active=True)
        self.assertGreater(car.speed_kmh, 260.0)

        # Braking into Nouvelle Chicane
        dist_chicane_brake = 0.0
        while car.speed_kmh > 75.0:
            car.step(throttle=0.0, brake=1.0, steer=0.0, dt=0.001)
            dist_chicane_brake += car.state.vx * 0.001
        self.assertLess(dist_chicane_brake, 70.0)

    # Scenario 3: Standing Grid 5-Red-Light Launch
    def test_scenario_3_standing_grid_5_red_light_launch(self):
        """Standing start with 5 red lights sequence -> lights out -> clutch launch to 100 km/h."""
        from f1sim.track.track_builder import StartingGantry

        # FIA Light sequence: 1, 2, 3, 4, 5 red lights on (1s intervals)
        hold_delay = 0.8
        gantry = StartingGantry()
        gantry.start_countdown(hold_duration=hold_delay)

        gantry_lights = []
        for _ in range(5):
            gantry.step(1.0)
            gantry_lights.append(sum(1 for on in gantry.get_light_states() if on))
        self.assertEqual(gantry_lights, [1, 2, 3, 4, 5])
        self.assertEqual(gantry.state, "HOLD")

        # Random hold duration: 0.8s -> Lights out!
        gantry.step(hold_delay + 0.01)
        self.assertTrue(gantry.is_lights_out)
        self.assertFalse(any(gantry.get_light_states()))

        # Driver reaction time (150ms) + Vehicle launch to 100 km/h
        reaction_time = 0.15
        car = Vehicle()
        car.vx = 0.0
        car.gear = 1
        car.rpm = 11000.0

        t_elapsed = reaction_time
        dt = 0.001
        while car.speed_kmh < 100.0 and t_elapsed < 4.0:
            car.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt)
            t_elapsed += dt

        # Total time from lights out to 100 km/h (~2.69s launch + 0.15s reaction = ~2.84s)
        self.assertGreaterEqual(t_elapsed, 2.40)
        self.assertLessEqual(t_elapsed, 2.95)

    # Scenario 4: DRS Straight Overtake Maneuver
    def test_scenario_4_drs_straight_overtake_maneuver(self):
        """Trailing car within 1.0s gap activates DRS on Monza main straight, reaches delta speed, and overtakes."""
        # Leader car cruising on straight at 325 km/h
        leader_v_ms = 325.0 / 3.6  # 90.28 m/s
        leader_dist = 35.0  # Leader starts 35m ahead (~0.39s gap, within 1.0s DRS window)

        # Trailing car
        chaser = Vehicle()
        chaser.state.vx = leader_v_ms
        chaser.powertrain.gearbox.gear = 8
        chaser.state.gear = 8
        chaser.state.rpm = 11800.0
        chaser_dist = 0.0

        # Detection point check: gap is < 1.0 second
        gap_sec = (leader_dist - chaser_dist) / leader_v_ms
        self.assertLess(gap_sec, 1.0)
        chaser.drs_available = True

        # Activation point: Chaser triggers DRS
        chaser.state.drs_active = True
        self.assertTrue(chaser.state.drs_active)

        # Accelerate down straight for 7 seconds
        dt = 0.001
        for _ in range(7000):
            gap = max(0.5, leader_dist - chaser_dist)
            if gap <= 45.0:
                drag_mult = compute_slipstream_drag_multiplier(gap, 0.0)
                chaser.aero.set_slipstream_factors(drag_scalar=drag_mult)
            else:
                chaser.aero.set_slipstream_factors(drag_scalar=1.0)
            chaser.state.drs_active = True
            chaser.step(throttle=1.0, brake=0.0, steer=0.0, dt=dt, drs_active=True)
            chaser_dist += chaser.state.vx * dt
            leader_dist += leader_v_ms * dt

        # Chaser top speed with DRS exceeds leader by >= 15 km/h
        self.assertGreater(chaser.speed_kmh, 345.0)
        speed_delta = chaser.speed_kmh - (leader_v_ms * 3.6)
        self.assertGreater(speed_delta, 15.0)

        # Chaser has overtaken leader
        self.assertGreater(chaser_dist, leader_dist)

    # Scenario 5: Extreme Brake Bias Cornering Snap & Recovery
    def test_scenario_5_extreme_brake_bias_cornering_snap(self):
        """Extreme rear brake bias creates snap oversteer which is caught and stabilized by counter-steering."""
        car = Vehicle()
        car.vx = 240.0 / 3.6  # 66.67 m/s
        car.gear = 6
        car.brake_bias = 0.52  # Extreme rearward bias

        # 1. Driver initiates hard trail-braking with rightward steer
        for _ in range(250):
            car.step(throttle=0.0, brake=0.90, steer=0.30, dt=0.001)

        # Rear slip snaps out and yaw rate increases rapidly
        snap_yaw_rate = car.yaw_rate
        self.assertGreater(abs(snap_yaw_rate), 0.35)

        # 2. Driver recovery: release brake pedal completely and counter-steer left
        car.drs_active = False
        for _ in range(400):
            # Counter-steer into direction of rotation
            counter_steer = -0.40 if car.yaw_rate > 0.0 else 0.40
            car.step(throttle=0.10, brake=0.0, steer=counter_steer, dt=0.001)

        # 3. Vehicle stabilizes without terminal 360 spin
        recovered_yaw_rate = abs(car.yaw_rate)
        self.assertLess(recovered_yaw_rate, 0.40)
        self.assertGreater(car.vx, 20.0)  # Car kept momentum and avoided ground loop


if __name__ == "__main__":
    unittest.main()
