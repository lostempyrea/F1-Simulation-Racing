"""
tests/test_empirical_challenger.py
==================================
Empirical Challenger Test Suite for Milestone 1:
Pacejka Tyre Grip, Slip Angle, Understeer & Oversteer Dynamics,
Counter-Steering Mechanics, Dynamic Weight Transfer, and Numerical Stability.

Author: teamwork_challenger (Challenger 2)
"""

import math
import unittest
import numpy as np

from f1sim.core.constants import (
    VEHICLE_MASS, WHEELBASE, DIST_CG_FRONT, DIST_CG_REAR,
    TRACK_WIDTH_FRONT, TRACK_WIDTH_REAR, CG_HEIGHT,
    GRAVITY, TransmissionMode
)
from f1sim.physics.tyre import TyreModel, PacejkaParams, TyreForces
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry, VehicleConfig, VehicleState
from f1sim.physics.powertrain import Powertrain


class EmpiricalChallengeM1Tests(unittest.TestCase):
    """Empirical verification harness directly testing f1sim physics classes."""

    def setUp(self):
        self.car = Vehicle()
        self.tyre = TyreModel(PacejkaParams(mu_x_0=2.15, mu_y_0=1.75))

    # =========================================================================
    # SUITE 1: UNDERSTEER DYNAMICS & SLIP ANGLE SATURATION
    # =========================================================================
    def test_understeer_excess_entry_speed_and_front_slip_saturation(self):
        """
        Verify that carrying excess entry speed into a curve with sharp steering lock:
        1. Front slip angle exceeds 8.0 degrees (> 0.1396 rad).
        2. Front slip angle reaches saturation regime (> 8.0 deg, where dFy/dalpha is flat/negative).
        3. Measured yaw rate displays a significant yaw rate deficit compared to Ackermann kinematic yaw rate.
        4. Instantaneous turning radius expands significantly (car pushes wide).
        """
        car = Vehicle()
        car.vx = 65.0  # 234 km/h entry speed
        car.gear = 6
        dt = 0.001

        steer_lock = 0.45  # ~25.8 degrees
        max_front_slip_deg = 0.0
        final_yaw_rate = 0.0
        final_speed = 0.0

        # Simulate 0.6 seconds of high-speed corner entry
        for step_idx in range(600):
            telem = car.step(throttle=0.0, brake=0.0, steer=steer_lock, dt=dt)
            slip_deg = math.degrees(telem.slip_angle_front)
            if slip_deg > max_front_slip_deg:
                max_front_slip_deg = slip_deg
            final_yaw_rate = telem.speed_ms * (car.state.yaw_rate / max(1.0, telem.speed_ms))  # yaw_rate
            final_speed = telem.speed_ms

        # 1. Front slip angle exceeds 8.0 deg
        print(f"\n[Understeer] Max front slip angle: {max_front_slip_deg:.2f}° (Criterion: > 8.0°)")
        self.assertGreater(max_front_slip_deg, 8.0, "Front slip angle must exceed 8.0 degrees")

        # 2. Check kinematic Ackermann yaw rate: r_ack = vx / L * tan(delta)
        # For L = 3.6m, delta = 0.45 rad:
        r_ackermann = (final_speed / 3.60) * math.tan(steer_lock)
        measured_r = abs(car.state.yaw_rate)
        print(f"[Understeer] Measured yaw rate: {measured_r:.3f} rad/s vs Ackermann: {r_ackermann:.3f} rad/s")
        self.assertLess(measured_r, r_ackermann * 0.70, "Yaw rate must show substantial deficit (< 70% of Ackermann)")

        # 3. Turning radius expansion (pushing wide)
        r_actual_turning_radius = final_speed / max(0.01, measured_r)
        r_geometric_radius = 3.60 / math.tan(steer_lock)
        print(f"[Understeer] Actual turning radius: {r_actual_turning_radius:.1f} m vs Geometric: {r_geometric_radius:.1f} m")
        self.assertGreater(r_actual_turning_radius, r_geometric_radius * 2.0, "Turning radius must expand > 2x geometric radius (push wide)")

    def test_understeer_vs_entry_speed_sweep(self):
        """
        Parametric sweep of entry speed from 20 m/s (72 km/h) to 80 m/s (288 km/h):
        Verifies that front slip angle grows monotonically with speed and exceeds 8° at high speed.
        """
        speeds = [20.0, 35.0, 50.0, 65.0, 80.0]
        peak_slips = []

        for v in speeds:
            c = Vehicle()
            c.vx = v
            c.gear = 5
            max_slip = 0.0
            for _ in range(300):
                telem = c.step(throttle=0.0, brake=0.0, steer=0.35, dt=0.001)
                deg = math.degrees(telem.slip_angle_front)
                if deg > max_slip:
                    max_slip = deg
            peak_slips.append(max_slip)
            print(f"[Speed Sweep] Speed: {v*3.6:.0f} km/h -> Peak Front Slip: {max_slip:.2f}°")

        # Higher speed must create larger front slip angle
        for i in range(len(peak_slips) - 1):
            self.assertGreaterEqual(peak_slips[i+1], peak_slips[i], "Front slip angle should increase with entry speed")
        self.assertGreater(peak_slips[-1], 8.0, "Front slip at 80 m/s must exceed 8.0 deg")

    # =========================================================================
    # SUITE 2: OVERSTEER DYNAMICS & FRICTION ELLIPSE BREAKAWAY
    # =========================================================================
    def test_power_oversteer_in_1st_gear_mid_corner(self):
        """
        Verify that applying full throttle in 1st gear mid-corner:
        1. Rear slip ratio spikes (> 0.20).
        2. Rear lateral grip collapses on the friction ellipse.
        3. Snap oversteer occurs: yaw rate surges > 0.8 rad/s and yaw acceleration spikes.
        """
        car = Vehicle()
        car.vx = 14.0  # 50.4 km/h apex speed
        car.gear = 1
        car.steer = 0.35
        dt = 0.001

        max_rear_slip = 0.0
        max_yaw_rate = 0.0
        max_yaw_accel = 0.0
        rear_lateral_forces = []

        for step in range(500):  # 0.5s full throttle stomp
            telem = car.step(throttle=1.0, brake=0.0, steer=0.35, dt=dt)
            if car.slip_ratio_rear > max_rear_slip:
                max_rear_slip = car.slip_ratio_rear
            if abs(car.state.yaw_rate) > max_yaw_rate:
                max_yaw_rate = abs(car.state.yaw_rate)
            if abs(car.state.yaw_accel) > max_yaw_accel:
                max_yaw_accel = abs(car.state.yaw_accel)

        print(f"\n[Oversteer 1st Gear] Max rear slip ratio: {max_rear_slip:.3f} (Criterion: > 0.20)")
        print(f"[Oversteer 1st Gear] Max yaw rate: {max_yaw_rate:.3f} rad/s (Criterion: > 0.8 rad/s)")
        print(f"[Oversteer 1st Gear] Max yaw acceleration: {max_yaw_accel:.3f} rad/s^2")

        self.assertGreater(max_rear_slip, 0.20, "Rear slip ratio must spike above 0.20")
        self.assertGreater(max_yaw_rate, 0.80, "Yaw rate must surge above 0.80 rad/s")
        self.assertGreater(max_yaw_accel, 1.50, "Yaw acceleration must spike indicating snap oversteer")

    def test_power_oversteer_in_2nd_gear_mid_corner(self):
        """
        Verify that applying full throttle in 2nd gear mid-corner also induces oversteer.
        """
        car = Vehicle()
        car.vx = 22.0  # ~80 km/h mid-speed corner
        car.gear = 2
        car.steer = 0.30
        dt = 0.001

        max_rear_slip = 0.0
        max_yaw_rate = 0.0

        for _ in range(500):
            car.step(throttle=1.0, brake=0.0, steer=0.30, dt=dt)
            if car.slip_ratio_rear > max_rear_slip:
                max_rear_slip = car.slip_ratio_rear
            if abs(car.state.yaw_rate) > max_yaw_rate:
                max_yaw_rate = abs(car.state.yaw_rate)

        print(f"\n[Oversteer 2nd Gear] Max rear slip ratio: {max_rear_slip:.3f}, Max yaw rate: {max_yaw_rate:.3f} rad/s")
        self.assertGreater(max_rear_slip, 0.20, "2nd gear full throttle must trigger slip ratio > 0.20")
        self.assertGreater(max_yaw_rate, 0.80, "2nd gear full throttle must produce yaw surge > 0.8 rad/s")

    def test_oversteer_over_kerbs_friction_drop(self):
        """
        Verify oversteer behavior when striking a kerb (reduced surface friction on inside/outside wheels):
        Surface mu reduced to 0.70 on kerb wheel reduces lateral grip and destabilizes yaw.
        """
        # Car 1: Standard asphalt cornering
        c_asphalt = Vehicle()
        c_asphalt.vx = 25.0
        c_asphalt.gear = 2
        c_asphalt.steer = 0.25
        for _ in range(300):
            c_asphalt.step(throttle=0.5, brake=0.0, steer=0.25, dt=0.001)

        # Car 2: Rear right wheel drops onto kerb (surface_mu = 0.65)
        c_kerb = Vehicle()
        c_kerb.vx = 25.0
        c_kerb.gear = 2
        c_kerb.steer = 0.25
        surface_kerb = np.array([1.0, 1.0, 1.0, 0.65], dtype=np.float64)  # RR wheel on kerb
        for _ in range(300):
            c_kerb.step(throttle=0.5, brake=0.0, steer=0.25, dt=0.001, surface_mu=surface_kerb)

        print(f"\n[Kerb Oversteer] Asphalt yaw rate: {c_asphalt.yaw_rate:.3f} rad/s vs Kerb yaw rate: {c_kerb.yaw_rate:.3f} rad/s")
        self.assertGreater(c_kerb.kerb_vibration, 0.0, "Kerb strike must report non-zero kerb vibration")
        self.assertNotEqual(c_asphalt.yaw_rate, c_kerb.yaw_rate, "Surface friction change on kerb must perturb vehicle dynamics")

    # =========================================================================
    # SUITE 3: COUNTER-STEERING STABILIZATION & RESTORING MOMENT
    # =========================================================================
    def test_counter_steering_generates_restoring_yaw_moment_and_dampens_spin(self):
        """
        Verify counter-steering:
        1. Induce an aggressive oversteer spin (yaw_rate > 1.2 rad/s).
        2. Apply counter-steering (steer in opposite direction of spin: steer = -0.50 rad).
        3. Verify that counter-steering generates a negative (restoring) yaw moment.
        4. Verify that the spinning yaw rate dampens significantly compared to holding steering.
        """
        dt = 0.001

        # Phase 1: Induce oversteer in Car A and Car B identically
        car_recovered = Vehicle()
        car_spun = Vehicle()
        car_recovered.vx = 16.0
        car_spun.vx = 16.0
        car_recovered.gear = 1
        car_spun.gear = 1

        for _ in range(400):
            car_recovered.step(throttle=1.0, brake=0.0, steer=0.35, dt=dt)
            car_spun.step(throttle=1.0, brake=0.0, steer=0.35, dt=dt)

        spin_yaw_rate = car_recovered.state.yaw_rate
        print(f"\n[Counter-Steer] Initial spin yaw rate: {spin_yaw_rate:.3f} rad/s")
        self.assertGreater(spin_yaw_rate, 1.0, "Spin condition must reach yaw_rate > 1.0 rad/s")

        # Phase 2:
        # Car A applies counter-steer (steer = -0.55 rad, throttle off)
        # Car B maintains uncorrected input (steer = 0.0, throttle off)
        yaw_rates_recovered = []
        yaw_accels_recovered = []

        for _ in range(250):  # 0.25s reaction
            car_recovered.step(throttle=0.0, brake=0.0, steer=-0.55, dt=dt)
            yaw_rates_recovered.append(car_recovered.state.yaw_rate)
            yaw_accels_recovered.append(car_recovered.state.yaw_accel)

            car_spun.step(throttle=0.0, brake=0.0, steer=0.0, dt=dt)

        # Check that counter-steering produced a negative yaw acceleration (restoring moment)
        min_yaw_accel = min(yaw_accels_recovered)
        final_recovered_yaw = car_recovered.state.yaw_rate
        final_spun_yaw = car_spun.state.yaw_rate

        print(f"[Counter-Steer] Peak restoring yaw accel: {min_yaw_accel:.2f} rad/s^2")
        print(f"[Counter-Steer] Final counter-steered yaw rate: {final_recovered_yaw:.3f} rad/s vs uncorrected: {final_spun_yaw:.3f} rad/s")

        self.assertLess(min_yaw_accel, -2.0, "Counter-steering must generate strong negative restoring yaw acceleration")
        self.assertLess(final_recovered_yaw, spin_yaw_rate * 0.60, "Counter-steering must damp spin yaw rate by > 40%")
        self.assertLess(final_recovered_yaw, final_spun_yaw, "Counter-steered car must have substantially lower yaw rate than uncorrected car")

    # =========================================================================
    # SUITE 4: DYNAMIC WEIGHT TRANSFER UNDER BRAKING & CORNERING
    # =========================================================================
    def test_dynamic_weight_transfer_under_4_5g_braking(self):
        """
        Verify dynamic load shift under 4.5G emergency braking:
        1. Static load per axle: Front ~ 3,697 N, Rear ~ 4,345 N (total ~ 8,041 N static).
        2. Under 4.5G deceleration (ax = -4.5 * 9.80665 m/s^2):
           dFz_long = m * ax * (h_cg / L) = 820 * 44.13 * (0.28 / 3.60) = 2,814.5 N.
        3. Adding aerodynamic downforce at 200 km/h (speed where 4.5G+ occurs):
           Front load shift is > 3,000 N.
        4. Verify rear axle load unloads correspondingly without going negative (>= 100 N clamp).
        """
        car = Vehicle()
        m = car.cfg.mass
        g = 9.80665
        L = car.cfg.L
        h_cg = car.cfg.h_cg

        # Static loads (zero accel, zero aero)
        Fz_static = car.compute_weight_transfer(ax=0.0, ay=0.0, downforce_front=0.0, downforce_rear=0.0)
        front_static_total = Fz_static[0] + Fz_static[1]
        rear_static_total = Fz_static[2] + Fz_static[3]

        print(f"\n[Weight Transfer] Static Front Load: {front_static_total:.1f} N | Static Rear Load: {rear_static_total:.1f} N")

        # 4.5G Braking (ax = -4.5 * g)
        ax_4_5g = -4.5 * g
        # Include aero downforce at 200 km/h (55.56 m/s)
        f_down_total, f_drag_total, df_front, df_rear = car.aero.compute(speed_ms=55.556, drs_active=False)

        Fz_braking = car.compute_weight_transfer(
            ax=ax_4_5g,
            ay=0.0,
            downforce_front=df_front,
            downforce_rear=df_rear
        )
        front_braking_total = Fz_braking[0] + Fz_braking[1]
        rear_braking_total = Fz_braking[2] + Fz_braking[3]

        load_shift_front = front_braking_total - front_static_total
        pure_inertia_shift = m * abs(ax_4_5g) * (h_cg / L)

        print(f"[Weight Transfer] Total Dynamic Front Load under 4.5G: {front_braking_total:.1f} N")
        print(f"[Weight Transfer] Dynamic Front Load Shift: +{load_shift_front:.1f} N (Criterion: > 3,000 N)")
        print(f"[Weight Transfer] Pure Inertial Long. Transfer: +{pure_inertia_shift:.1f} N")
        print(f"[Weight Transfer] Dynamic Rear Load under 4.5G: {rear_braking_total:.1f} N")

        self.assertGreater(load_shift_front, 3000.0, "Dynamic load shift to front under 4.5G braking must exceed 3,000 N")
        self.assertGreaterEqual(np.min(Fz_braking), 100.0, "All wheel loads must remain strictly positive (>= 100 N clamp)")

    def test_dynamic_weight_transfer_under_4g_cornering(self):
        """
        Verify dynamic lateral load shift under 4.0G cornering:
        1. Total lateral acceleration ay = 4.0 * 9.80665 m/s^2.
        2. Lateral transfer across front track (Wf = 1.60m) with 55% roll stiffness.
        3. Lateral transfer across rear track (Wr = 1.55m) with 45% roll stiffness.
        4. Outside wheels gain load, inside wheels shed load.
        """
        car = Vehicle()
        ay_4g = 4.0 * 9.80665
        # 160 km/h cornering downforce
        _, _, df_front, df_rear = car.aero.compute(speed_ms=44.44, drs_active=False)

        Fz_cornering = car.compute_weight_transfer(
            ax=0.0,
            ay=ay_4g,
            downforce_front=df_front,
            downforce_rear=df_rear
        )
        # Wheels: 0=FL, 1=FR, 2=RL, 3=RR
        # ay > 0 (turning left) -> Right wheels (FR, RR) are outside (gain load), Left (FL, RL) are inside (shed load)
        fz_fl, fz_fr, fz_rl, fz_rr = Fz_cornering

        print(f"\n[Lateral Transfer 4G] FL (Inside): {fz_fl:.1f} N, FR (Outside): {fz_fr:.1f} N")
        print(f"[Lateral Transfer 4G] RL (Inside): {fz_rl:.1f} N, RR (Outside): {fz_rr:.1f} N")

        front_lateral_delta = fz_fr - fz_fl
        rear_lateral_delta = fz_rr - fz_rl

        print(f"[Lateral Transfer 4G] Front Lateral Delta: {front_lateral_delta:.1f} N")
        print(f"[Lateral Transfer 4G] Rear Lateral Delta: {rear_lateral_delta:.1f} N")

        self.assertGreater(fz_fr, fz_fl, "Outside front wheel must carry significantly more load than inside wheel")
        self.assertGreater(fz_rr, fz_rl, "Outside rear wheel must carry significantly more load than inside wheel")
        self.assertGreater(front_lateral_delta, 4000.0, "Front axle lateral load delta under 4G must exceed 4,000 N")
        self.assertGreater(rear_lateral_delta, 3500.0, "Rear axle lateral load delta under 4G must exceed 3,500 N")
        self.assertGreaterEqual(np.min(Fz_cornering), 100.0, "Inside wheels must remain clamped above 100 N (no negative load)")

    # =========================================================================
    # SUITE 5: FRICTION ELLIPSE INVARIANCE & MATHEMATICAL STABILITY
    # =========================================================================
    def test_friction_ellipse_strict_invariance_under_arbitrary_inputs(self):
        """
        Adversarial stress-test: verify that for ANY combination of slip ratio kappa,
        slip angle alpha, normal load Fz, and surface mu, the combined contact patch force
        NEVER exceeds the friction ellipse boundary:
        (Fx / Dx)^2 + (Fy / Dy)^2 <= 1.000001
        """
        tyre = TyreModel(PacejkaParams(mu_x_0=2.15, mu_y_0=1.75))
        np.random.seed(42)

        violations = 0
        max_ellipse_val = 0.0

        # Sample 5,000 random adversarial combinations
        kappas = np.random.uniform(-1.5, 1.5, 5000)
        alphas = np.random.uniform(-math.pi / 2.0, math.pi / 2.0, 5000)
        fzs = np.random.uniform(50.0, 15000.0, 5000)
        mus = np.random.uniform(0.1, 1.5, 5000)

        for kappa, alpha, fz, mu in zip(kappas, alphas, fzs, mus):
            forces = tyre.compute_combined_forces(kappa=float(kappa), alpha=float(alpha), Fz=float(fz), mu_surface=float(mu))
            
            # Friction ellipse limits
            fz_c = max(100.0, fz)
            mu_x = tyre.compute_mu(fz_c, is_lateral=False) * mu
            mu_y = tyre.compute_mu(fz_c, is_lateral=True) * mu
            dx = max(1.0, mu_x * fz_c)
            dy = max(1.0, mu_y * fz_c)

            val = (forces.Fx / dx) ** 2 + (forces.Fy / dy) ** 2
            if val > max_ellipse_val:
                max_ellipse_val = val
            if val > 1.00001:
                violations += 1

        print(f"\n[Friction Ellipse Invariance] Tested 5,000 random points. Max ellipse value: {max_ellipse_val:.6f}")
        self.assertEqual(violations, 0, f"Found {violations} violations of the friction ellipse boundary!")
        self.assertLessEqual(max_ellipse_val, 1.000001, "Maximum friction ellipse metric exceeded 1.0")

    def test_numerical_stability_under_extreme_and_singular_inputs(self):
        """
        Verify that Vehicle.step and TyreModel do NOT produce NaN or Inf under:
        - Standstill vx = 0.0, vy = 0.0
        - Pure backward motion vx = -10.0
        - Full steering lock (0.90 rad) at standstill
        - Extreme G-forces and sudden step inputs
        """
        car = Vehicle()

        # 1. Standstill step
        telem0 = car.step(throttle=0.0, brake=0.0, steer=0.0, dt=0.001)
        self.assertFalse(math.isnan(telem0.speed_kmh))
        self.assertFalse(math.isnan(telem0.lateral_g))

        # 2. Standstill with full steering lock
        telem_lock = car.step(throttle=0.0, brake=0.0, steer=0.90, dt=0.001)
        self.assertFalse(math.isnan(telem_lock.slip_angle_front))
        self.assertFalse(math.isinf(telem_lock.slip_angle_front))

        # 3. Sudden reverse velocity singularity test
        car.state.vx = -5.0
        car.state.vy = -2.0
        car.state.yaw_rate = 3.0
        telem_rev = car.step(throttle=1.0, brake=1.0, steer=-0.80, dt=0.001)
        self.assertFalse(math.isnan(telem_rev.speed_kmh))
        self.assertFalse(math.isnan(telem_rev.lateral_g))
        self.assertFalse(math.isnan(telem_rev.longitudinal_g))
        print("\n[Numerical Stability] Zero division and extreme input checks passed cleanly.")


if __name__ == "__main__":
    unittest.main()
