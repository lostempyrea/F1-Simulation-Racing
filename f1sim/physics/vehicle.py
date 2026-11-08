"""
f1sim/physics/vehicle.py
Two-track (4-wheel) Formula 1 vehicle dynamics engine with dynamic weight
transfer, roll stiffness distribution, understeer/oversteer yaw mechanics,
and fixed-step numerical integration.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
import numpy as np

from f1sim.core.constants import (
    VEHICLE_MASS, WHEELBASE, DIST_CG_FRONT, DIST_CG_REAR,
    TRACK_WIDTH_FRONT, TRACK_WIDTH_REAR, CG_HEIGHT,
    MOMENT_INERTIA_YAW, WHEEL_INERTIA, TYRE_RADIUS,
    RPM_IDLE, RPM_REDLINE, RPM_AUTO_UPSHIFT, TransmissionMode,
    BRAKE_BIAS_DEFAULT, BRAKE_BIAS_MIN, BRAKE_BIAS_MAX, GRAVITY
)
from f1sim.physics.tyre import TyreModel, TyreForces, PacejkaParams
from f1sim.physics.powertrain import Powertrain
from f1sim.physics.aero import Aerodynamics, DRSState, AeroConfig
from f1sim.physics.brakes import BrakeSystem, BrakeConfig


@dataclass(frozen=True)
class VehicleConfig:
    """Fixed geometric and physical vehicle properties."""
    mass: float = VEHICLE_MASS             # Total vehicle mass (kg)
    L: float = WHEELBASE                   # Wheelbase (m)
    Lf: float = DIST_CG_FRONT              # CG to front axle (m) (54% rear weight bias)
    Lr: float = DIST_CG_REAR               # CG to rear axle (m) (46% front weight bias)
    Wf: float = TRACK_WIDTH_FRONT          # Front track width (m)
    Wr: float = TRACK_WIDTH_REAR           # Rear track width (m)
    h_cg: float = CG_HEIGHT                # Center of gravity height (m)
    Iz: float = MOMENT_INERTIA_YAW         # Yaw moment of inertia (kg*m^2)
    Iw: float = WHEEL_INERTIA              # Wheel moment of inertia (kg*m^2)
    Rw: float = TYRE_RADIUS                # Wheel rolling radius (m)
    K_roll_front: float = 0.55             # Front anti-roll bar stiffness distribution (55%)
    max_steer_rad: float = 0.900           # Max road wheel steer angle (~51 deg for hairpin)
    steer_ratio: float = 14.0              # Steering wheel to road wheel ratio


@dataclass
class VehicleState:
    """Dynamic vehicle state vector."""
    # World Coordinates
    x: float = 0.0               # World position X (m)
    y: float = 0.0               # World position Y (m)
    z: float = 0.0               # World position Z (m)
    yaw: float = 0.0             # Heading angle psi (rad)

    # Body Frame Velocities
    vx: float = 0.0              # Forward velocity (m/s)
    vy: float = 0.0              # Lateral velocity (m/s)
    yaw_rate: float = 0.0        # Angular velocity d(psi)/dt (rad/s)

    # Accelerations (Body Frame)
    ax: float = 0.0              # Longitudinal acceleration (m/s^2)
    ay: float = 0.0              # Lateral acceleration (m/s^2)
    yaw_accel: float = 0.0       # Angular acceleration d^2(psi)/dt^2 (rad/s^2)

    # Wheel Angular Velocities (FL, FR, RL, RR) (rad/s)
    wheel_omega: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64))

    # Controls & Inputs
    throttle: float = 0.0        # 0.0 .. 1.0
    brake: float = 0.0           # 0.0 .. 1.0
    steering_angle: float = 0.0  # Road wheel steer angle delta (rad)
    gear: int = 1                # Current gear (-1: R, 0: N, 1-8)
    rpm: float = RPM_IDLE        # Engine RPM
    drs_active: bool = False
    drs_available: bool = False
    brake_bias: float = BRAKE_BIAS_DEFAULT


@dataclass
class VehicleTelemetry:
    """Exact interface contract required by PROJECT.md & UI/Renderer."""
    speed_kmh: float
    speed_ms: float
    rpm: float
    gear: int
    throttle: float
    brake: float
    steering_angle: float
    drs_available: bool
    drs_active: bool
    brake_bias: float
    engine_running: bool
    slip_angle_front: float
    slip_angle_rear: float
    lateral_g: float
    longitudinal_g: float
    kerb_vibration: float


class Vehicle:
    """
    Two-track 4-wheel Formula 1 vehicle dynamics simulator.
    """
    def __init__(
        self,
        config: VehicleConfig | None = None,
        tyre_model: TyreModel | None = None,
        powertrain: Powertrain | None = None,
        aero: Aerodynamics | None = None,
        brakes: BrakeSystem | None = None
    ) -> None:
        self.cfg: VehicleConfig = config if config is not None else VehicleConfig()
        self.tyre: TyreModel = tyre_model if tyre_model is not None else TyreModel(PacejkaParams(mu_x_0=2.15, mu_y_0=1.75))
        self.powertrain: Powertrain = powertrain if powertrain is not None else Powertrain(mode=TransmissionMode.AUTOMATIC)
        if powertrain is None:
            self.powertrain.set_running()
            self.powertrain.gearbox.gear = 1

        self.aero: Aerodynamics = aero if aero is not None else Aerodynamics(AeroConfig(c_drag_nominal=0.79))
        self.brakes: BrakeSystem = brakes if brakes is not None else BrakeSystem(BrakeConfig(max_system_torque=14800.0))

        self.state: VehicleState = VehicleState()
        self.state.gear = self.powertrain.gear
        self.state.rpm = self.powertrain.rpm
        self.state.brake_bias = self.brakes.get_bias()

        # Wheel contact patch positions relative to CG (FL, FR, RL, RR)
        self.wheel_pos = np.array([
            [+self.cfg.Lf, +self.cfg.Wf / 2.0],  # FL (0)
            [+self.cfg.Lf, -self.cfg.Wf / 2.0],  # FR (1)
            [-self.cfg.Lr, +self.cfg.Wr / 2.0],  # RL (2)
            [-self.cfg.Lr, -self.cfg.Wr / 2.0],  # RR (3)
        ], dtype=np.float64)

        # Telemetry tracking buffers
        self.last_slip_angles = np.zeros(4, dtype=np.float64)
        self.last_slip_ratios = np.zeros(4, dtype=np.float64)
        self.last_normal_loads = np.zeros(4, dtype=np.float64)
        self.kerb_vibration_amplitude: float = 0.0

    @property
    def speed_ms(self) -> float:
        return math.sqrt(self.state.vx ** 2 + self.state.vy ** 2)

    @property
    def speed_kmh(self) -> float:
        return self.speed_ms * 3.6

    @property
    def vx(self) -> float:
        return self.state.vx

    @vx.setter
    def vx(self, val: float) -> None:
        self.state.vx = val
        wheel_w = val / self.cfg.Rw
        self.state.wheel_omega = np.full(4, wheel_w, dtype=np.float64)

    @property
    def vy(self) -> float:
        return self.state.vy

    @vy.setter
    def vy(self, val: float) -> None:
        self.state.vy = val

    @property
    def yaw_rate(self) -> float:
        return self.state.yaw_rate

    @yaw_rate.setter
    def yaw_rate(self, val: float) -> None:
        self.state.yaw_rate = val

    @property
    def rpm(self) -> float:
        return self.state.rpm

    @rpm.setter
    def rpm(self, val: float) -> None:
        self.state.rpm = val

    @property
    def gear(self) -> int:
        return self.state.gear

    @gear.setter
    def gear(self, val: int) -> None:
        self.state.gear = val
        self.powertrain.gearbox.gear = val

    @property
    def throttle(self) -> float:
        return self.state.throttle

    @property
    def brake(self) -> float:
        return self.state.brake

    @property
    def steering_angle(self) -> float:
        return self.state.steering_angle

    @steering_angle.setter
    def steering_angle(self, val: float) -> None:
        self.state.steering_angle = val

    @property
    def steer(self) -> float:
        return self.state.steering_angle

    @steer.setter
    def steer(self, val: float) -> None:
        self.state.steering_angle = val

    @property
    def drs_active(self) -> bool:
        return self.state.drs_active

    @drs_active.setter
    def drs_active(self, val: bool) -> None:
        self.state.drs_active = val
        if val:
            self.aero.drs_state = DRSState.ACTIVE
            self.aero.flap_position = 1.0
        else:
            self.aero.force_close_drs()
            self.aero.flap_position = 0.0

    @property
    def drs_available(self) -> bool:
        return self.state.drs_available

    @drs_available.setter
    def drs_available(self, val: bool) -> None:
        self.state.drs_available = val

    @property
    def brake_bias(self) -> float:
        return self.state.brake_bias

    @brake_bias.setter
    def brake_bias(self, val: float) -> None:
        clamped = float(np.clip(val, BRAKE_BIAS_MIN, BRAKE_BIAS_MAX))
        self.state.brake_bias = clamped
        self.brakes.set_bias(clamped)

    def adjust_brake_bias(self, delta: float) -> float:
        self.brake_bias = self.brake_bias + delta
        return self.brake_bias

    @property
    def lateral_g(self) -> float:
        return self.state.ay / 9.80665

    @property
    def longitudinal_g(self) -> float:
        return self.state.ax / 9.80665

    @property
    def slip_angle_front(self) -> float:
        return float(0.5 * (abs(self.last_slip_angles[0]) + abs(self.last_slip_angles[1])))

    @property
    def slip_angle_rear(self) -> float:
        return float(0.5 * (abs(self.last_slip_angles[2]) + abs(self.last_slip_angles[3])))

    @property
    def slip_ratio_rear(self) -> float:
        return float(0.5 * (self.last_slip_ratios[2] + self.last_slip_ratios[3]))

    @property
    def kerb_vibration(self) -> float:
        return self.kerb_vibration_amplitude

    @kerb_vibration.setter
    def kerb_vibration(self, val: float) -> None:
        self.kerb_vibration_amplitude = val

    def shift_up(self) -> bool:
        success = self.powertrain.gearbox.request_upshift()
        if success:
            self.state.gear = self.powertrain.gearbox.gear
        return success

    def shift_down(self) -> bool:
        if self.powertrain.gearbox.gear == 0 and self.state.vx > 0.28:
            return False
        success = self.powertrain.gearbox.request_downshift(self.state.vx)
        if success:
            self.state.gear = self.powertrain.gearbox.gear
        return success

    def pacejka_lateral_force(self, alpha_rad: float, fz_n: float) -> float:
        return float(self.tyre.compute_pure_lateral(alpha_rad, fz_n, 1.0))

    def compute_weight_transfer(
        self,
        ax: float,
        ay: float,
        downforce_front: float,
        downforce_rear: float
    ) -> np.ndarray:
        """
        Computes 4-wheel normal loads Fz (FL, FR, RL, RR) accounting for
        static distribution, longitudinal transfer, lateral transfer with
        roll stiffness, and aerodynamic downforce.
        """
        g = 9.80665
        m = self.cfg.mass
        L = self.cfg.L
        h_cg = self.cfg.h_cg

        # Static Axle Loads
        Fz_front_stat = m * g * (self.cfg.Lr / L)
        Fz_rear_stat = m * g * (self.cfg.Lf / L)

        # Longitudinal Weight Transfer
        dFz_long = m * ax * (h_cg / L)

        # Lateral Weight Transfer (Left/Right)
        dFz_lat_front = m * ay * (h_cg / self.cfg.Wf) * self.cfg.K_roll_front
        dFz_lat_rear = m * ay * (h_cg / self.cfg.Wr) * (1.0 - self.cfg.K_roll_front)

        # 4 Wheels: FL, FR, RL, RR
        Fz_fl = 0.5 * Fz_front_stat - 0.5 * dFz_long + 0.5 * downforce_front - dFz_lat_front
        Fz_fr = 0.5 * Fz_front_stat - 0.5 * dFz_long + 0.5 * downforce_front + dFz_lat_front
        Fz_rl = 0.5 * Fz_rear_stat + 0.5 * dFz_long + 0.5 * downforce_rear - dFz_lat_rear
        Fz_rr = 0.5 * Fz_rear_stat + 0.5 * dFz_long + 0.5 * downforce_rear + dFz_lat_rear

        Fz_array = np.array([Fz_fl, Fz_fr, Fz_rl, Fz_rr], dtype=np.float64)
        return np.maximum(Fz_array, 100.0)

    def step(
        self,
        throttle: float,
        brake: float,
        steer: float,
        dt: float = 0.001,
        drs_active: bool | None = None,
        surface_mu: np.ndarray | None = None,
        shift_cmd: int = 0
    ) -> VehicleTelemetry:
        """
        Integrates vehicle dynamics for one physics time-step dt.
        steer: steering input in radians or normalized [-1.0 .. +1.0].
        """
        dt = max(0.0001, dt)
        has_surface_mu = surface_mu is not None
        if surface_mu is None:
            surface_mu = np.ones(4, dtype=np.float64)

        if drs_active is not None:
            self.state.drs_active = drs_active

        # Auto-cancel DRS on brake, high lateral G, or any steering input
        if brake > 0.05 or abs(self.state.ay / 9.80665) > 0.40 or abs(steer) > 0.035:
            self.state.drs_active = False
            self.aero.force_close_drs()

        # 1. Update steering geometry with speed-sensitive limiter
        # At high speed, max steering angle decreases to prevent snap oversteer
        speed_factor = 1.0 / (1.0 + 0.0018 * self.state.vx * self.state.vx)
        effective_max_steer = self.cfg.max_steer_rad * speed_factor
        road_steer = float(np.clip(steer, -effective_max_steer, effective_max_steer))
        steer_angles = np.array([road_steer, road_steer, 0.0, 0.0], dtype=np.float64)
        self.state.steering_angle = road_steer
        self.state.throttle = float(np.clip(throttle, 0.0, 1.0))
        self.state.brake = float(np.clip(brake, 0.0, 1.0))

        # 2. Aerodynamic Forces (Monza high-speed aero trim)
        f_down_total, f_drag_total, f_down_f, f_down_r = self.aero.compute(self.state.vx, self.state.drs_active)

        # 3. Dynamic Normal Loads
        Fz = self.compute_weight_transfer(
            self.state.ax,
            self.state.ay,
            f_down_f,
            f_down_r
        )
        self.last_normal_loads = Fz.copy()

        # 4. Wheel Kinematics & Contact Patch Velocities
        r = self.state.yaw_rate
        vx_cg = self.state.vx
        vy_cg = self.state.vy

        vx_patches = vx_cg - r * self.wheel_pos[:, 1]
        vy_patches = vy_cg + r * self.wheel_pos[:, 0]

        cos_d = np.cos(steer_angles)
        sin_d = np.sin(steer_angles)
        vx_wheels = vx_patches * cos_d + vy_patches * sin_d
        vy_wheels = -vx_patches * sin_d + vy_patches * cos_d

        # 5. Non-singular Slip Angles and Slip Ratios
        v_eff = np.maximum(np.abs(vx_wheels), 0.50)
        slip_angles = -np.arctan2(vy_wheels, v_eff)

        # 6. Powertrain Drive & Shifting
        tx_out = self.powertrain.step(
            throttle=self.state.throttle,
            brake=self.state.brake,
            wheel_speed_ms=self.state.vx,
            dt=dt,
            shift_cmd=shift_cmd
        )
        self.state.rpm = tx_out.engine_rpm
        self.state.gear = tx_out.gear

        # Auto upshift when RPM >= 12,800 in forward gears
        if self.powertrain.gearbox.mode == TransmissionMode.AUTOMATIC and self.state.gear < 8 and self.state.rpm >= RPM_AUTO_UPSHIFT:
            self.shift_up()

        # 7. Drive Traction & Power Oversteer Modeling
        # Drive directionality (-1 in Reverse, +1 forward)
        drive_dir = -1.0 if self.state.gear == -1 else 1.0

        # Calculate dynamic rear traction limit incorporating surface_mu
        fz_rear_total = Fz[2] + Fz[3]
        mu_rear_surface = 0.5 * (surface_mu[2] + surface_mu[3])
        mu_rear_eff = 1.82 * (1.0 - 0.075 * ((fz_rear_total / 2.0 - 4000.0) / 4000.0)) * mu_rear_surface
        f_traction_max_rear = mu_rear_eff * fz_rear_total

        # Wheel drive torque through transmission
        wheel_drive_torque = tx_out.wheel_drive_torque
        if self.state.gear == -1:
            wheel_drive_torque = -abs(wheel_drive_torque)

        f_drive_wheel = (wheel_drive_torque / self.cfg.Rw) if (self.state.gear != 0 and self.state.throttle > 0.01) else 0.0

        # Continuous natural slip ratio calculation
        if self.state.throttle > 0.01 and self.state.gear != 0:
            f_demand_mag = abs(f_drive_wheel)
            if f_demand_mag <= f_traction_max_rear:
                # Elastic adhesion regime
                rear_slip_mag = 0.10 * (f_demand_mag / max(1.0, f_traction_max_rear))
            else:
                # Sliding / breakaway regime
                excess_ratio = (f_demand_mag - f_traction_max_rear) / max(1.0, f_traction_max_rear)
                rear_slip_mag = min(0.38, 0.10 + 0.23 * min(1.5, excess_ratio))
            rear_slip_ratio = drive_dir * rear_slip_mag
        else:
            rear_slip_ratio = 0.0

        # Rear wheel rotational speed: decoupled at standstill with zero throttle (no idle creep)
        if self.state.throttle <= 0.01 or self.state.gear == 0:
            omega_rear = vx_cg / self.cfg.Rw
        else:
            omega_rear = (vx_cg * (1.0 + rear_slip_ratio)) / self.cfg.Rw if abs(vx_cg) > 0.2 else (rear_slip_ratio * 0.5 / self.cfg.Rw)

        self.state.wheel_omega[2] = omega_rear
        self.state.wheel_omega[3] = omega_rear

        # 8. Braking Forces & Thermal Disc Updates
        if self.state.brake > 0.0:
            # Step brake system with pad thermal states and retrieve applied spindle torques
            brake_out = self.brakes.update(self.state.brake, self.state.wheel_omega, abs(vx_cg), dt)
            brake_torques = np.array([brake_out.torque_fl, brake_out.torque_fr, brake_out.torque_rl, brake_out.torque_rr], dtype=np.float64)

            # Wheel braking forces calculated directly from disc torque and clamped by tyre normal load & surface_mu
            f_brake_wheels = np.zeros(4, dtype=np.float64)
            for i in range(4):
                f_demand = brake_torques[i] / self.cfg.Rw
                mu_tyre = self.tyre.compute_mu(float(Fz[i]), is_lateral=False) * float(surface_mu[i])
                f_grip = mu_tyre * float(Fz[i])
                f_brake_wheels[i] = min(f_demand, f_grip)

            # Wheel rotational slip under braking
            front_slip = -0.10 * self.state.brake
            rear_slip = -0.09 * self.state.brake
            self.state.wheel_omega[0] = (vx_cg * (1.0 + front_slip)) / self.cfg.Rw
            self.state.wheel_omega[1] = (vx_cg * (1.0 + front_slip)) / self.cfg.Rw
            self.state.wheel_omega[2] = (vx_cg * (1.0 + rear_slip)) / self.cfg.Rw
            self.state.wheel_omega[3] = (vx_cg * (1.0 + rear_slip)) / self.cfg.Rw
        else:
            f_brake_wheels = np.zeros(4, dtype=np.float64)
            self.state.wheel_omega[0] = vx_cg / self.cfg.Rw
            self.state.wheel_omega[1] = vx_cg / self.cfg.Rw

        slip_ratios = np.array([
            (self.state.wheel_omega[0] * self.cfg.Rw - vx_wheels[0]) / v_eff[0],
            (self.state.wheel_omega[1] * self.cfg.Rw - vx_wheels[1]) / v_eff[1],
            (self.state.wheel_omega[2] * self.cfg.Rw - vx_wheels[2]) / v_eff[2],
            (self.state.wheel_omega[3] * self.cfg.Rw - vx_wheels[3]) / v_eff[3],
        ], dtype=np.float64)

        self.last_slip_angles = slip_angles.copy()
        self.last_slip_ratios = slip_ratios.copy()

        # 9. Contact Patch Tyre Forces & Yaw Moments
        Fx_body_total = 0.0
        Fy_body_total = 0.0
        Mz_total = 0.0

        k_blend_lat = float(np.clip(abs(vx_cg) / 0.20, 0.0, 1.0))

        for i in range(4):
            forces = self.tyre.compute_combined_forces(
                kappa=float(slip_ratios[i]),
                alpha=float(slip_angles[i]),
                Fz=float(Fz[i]),
                steer_angle=float(steer_angles[i]),
                mu_surface=float(surface_mu[i])
            )

            # Lateral cornering force emerges naturally from friction ellipse
            fy_b = forces.Fy_body * k_blend_lat

            # Longitudinal force
            if self.state.brake > 0.0:
                # Braking strictly opposes velocity direction (-copysign(F_b, vx))
                brake_dir = -math.copysign(1.0, vx_cg) if abs(vx_cg) > 0.01 else 0.0
                fx_b = brake_dir * f_brake_wheels[i]
            elif self.state.throttle > 0.01 and self.state.gear != 0 and i in (2, 3):
                # Drive force limited by Pacejka tyre grip and wheel torque demand
                f_drive_max_per_wheel = 0.5 * f_traction_max_rear
                f_drive_act = drive_dir * min(abs(0.5 * f_drive_wheel), f_drive_max_per_wheel)
                # In sliding regime, tyre sliding friction curve applies
                s_mag = abs(rear_slip_ratio)
                sliding_taper = 1.0 - 0.05 * float(np.clip((s_mag - 0.10) / 0.10, 0.0, 1.0))
                fx_b = f_drive_act * sliding_taper
            else:
                fx_b = forces.Fx_body if abs(vx_cg) > 0.1 else 0.0

            Fx_body_total += fx_b
            Fy_body_total += fy_b

            rx = self.wheel_pos[i, 0]
            ry = self.wheel_pos[i, 1]
            Mz_total += (rx * fy_b - ry * fx_b)

        # 10. Non-Inertial Body Equations of Motion
        drag_force_x = -math.copysign(f_drag_total, vx_cg) if abs(vx_cg) > 0.05 else 0.0
        rolling_resistance_force = -math.copysign(0.015 * np.sum(Fz), vx_cg) if abs(vx_cg) > 0.05 else 0.0

        Fx_net = Fx_body_total + drag_force_x + rolling_resistance_force
        Fy_net = Fy_body_total

        # Equivalent apparent mass with drivetrain rotational inertia
        ratio_eff = self.powertrain.gearbox.get_overall_ratio() * 1.52
        m_rot = (4.0 * self.cfg.Iw + 0.015 * (ratio_eff ** 2)) / (self.cfg.Rw ** 2) if (self.state.throttle > 0.01 and self.state.gear != 0) else 0.0
        m_eff = self.cfg.mass + m_rot

        ax_body = Fx_net / m_eff
        ay_body = Fy_net / self.cfg.mass
        # Natural yaw damping (combined pneumatic trail self-aligning torque)
        C_YAW_DAMP = 950.0  # Nm·s/rad — aggregate 4-tyre self-aligning torque
        Mz_total -= C_YAW_DAMP * self.state.yaw_rate
        yaw_accel = Mz_total / self.cfg.Iz

        self.state.ax = ax_body
        self.state.ay = ay_body
        self.state.yaw_accel = yaw_accel

        dvx = (ax_body + vy_cg * r) * dt
        dvy = (ay_body - vx_cg * r) * dt
        dyaw_rate = yaw_accel * dt

        vx_prev = self.state.vx
        vx_next = vx_prev + dvx

        # Zero-crossing braking latch & standstill stabilization
        if self.state.brake > 0.01 and self.state.throttle < 0.05 and (vx_prev * vx_next <= 0.0):
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.yaw_rate = 0.0
            self.state.ax = 0.0
            self.state.ay = 0.0
        elif abs(vx_next) < 0.05 and self.state.throttle < 0.01:
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.yaw_rate = 0.0
            self.state.ax = 0.0
            self.state.ay = 0.0
        else:
            self.state.vx = vx_next
            self.state.vy += dvy
            self.state.yaw_rate += dyaw_rate

        # 11. World Coordinates
        cos_yaw = math.cos(self.state.yaw)
        sin_yaw = math.sin(self.state.yaw)
        self.state.x += (self.state.vx * cos_yaw - self.state.vy * sin_yaw) * dt
        self.state.y += (self.state.vx * sin_yaw + self.state.vy * cos_yaw) * dt
        self.state.yaw = (self.state.yaw + self.state.yaw_rate * dt + math.pi) % (2.0 * math.pi) - math.pi

        # 12. Kerb vibration amplitude
        if has_surface_mu:
            on_kerb_count = int(np.sum(surface_mu < 0.95))
            self.kerb_vibration_amplitude = 0.65 * (on_kerb_count / 4.0) if on_kerb_count > 0 else 0.0

        return self.get_telemetry()

    def get_telemetry(self) -> VehicleTelemetry:
        """Assembles standard telemetry dataclass for HUD and external queries."""
        speed_ms = float(math.sqrt(self.state.vx ** 2 + self.state.vy ** 2))
        speed_kmh = float(speed_ms * 3.6)
        alpha_f = float(0.5 * (abs(self.last_slip_angles[0]) + abs(self.last_slip_angles[1])))
        alpha_r = float(0.5 * (abs(self.last_slip_angles[2]) + abs(self.last_slip_angles[3])))

        return VehicleTelemetry(
            speed_kmh=speed_kmh,
            speed_ms=speed_ms,
            rpm=float(self.state.rpm),
            gear=int(self.state.gear),
            throttle=float(self.state.throttle),
            brake=float(self.state.brake),
            steering_angle=float(self.state.steering_angle),
            drs_available=bool(self.state.drs_available),
            drs_active=bool(self.state.drs_active),
            brake_bias=float(self.state.brake_bias),
            engine_running=bool(self.powertrain.is_running),
            slip_angle_front=alpha_f,
            slip_angle_rear=alpha_r,
            lateral_g=float(self.state.ay / 9.80665),
            longitudinal_g=float(self.state.ax / 9.80665),
            kerb_vibration=float(self.kerb_vibration_amplitude)
        )
