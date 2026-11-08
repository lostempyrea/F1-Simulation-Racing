"""
f1sim/physics/powertrain.py
Complete 1.6L V6 Turbo Hybrid Power Unit, 8-speed seamless sequential gearbox,
4-stage ignition state machine, and anti-stall clutch protection.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from f1sim.core.constants import (
    IgnitionState, TransmissionMode,
    RPM_IDLE, RPM_REDLINE, RPM_REV_LIMITER_CUT, RPM_PEAK_POWER,
    RPM_AUTO_UPSHIFT, RPM_AUTO_DOWNSHIFT, RPM_KICKDOWN_THRESHOLD,
    RPM_STALL_THRESHOLD, RPM_CRANKING_TARGET, RPM_FIRE_FLARE,
    PEAK_POWER_TOTAL_KW, PEAK_POWER_ICE_KW, PEAK_POWER_MGUK_KW,
    PEAK_TORQUE_ICE_NM, PEAK_TORQUE_MGUK_NM, MAX_COMBINED_TORQUE_NM,
    ENGINE_BRAKE_BASE_NM, ENGINE_BRAKE_HIGH_RPM_NM,
    GEAR_RATIOS, FINAL_DRIVE_RATIO, TRANSMISSION_EFFICIENCY,
    SEAMLESS_SHIFT_DURATION, THROTTLE_BLIP_DURATION, THROTTLE_BLIP_AMOUNT,
    DOWNSHIFT_OVERREV_LIMIT_RPM, TYRE_RADIUS
)


@dataclass
class EngineOutput:
    torque_net: float = 0.0         # Net output torque delivered to flywheel (Nm)
    torque_ice: float = 0.0         # Internal combustion torque (Nm)
    torque_mguk: float = 0.0        # MGU-K electric motor torque (Nm)
    power_kw: float = 0.0           # Current mechanical power output (kW)
    fuel_flow_kgh: float = 0.0      # Instantaneous fuel flow (kg/h)


@dataclass
class TransmissionOutput:
    wheel_drive_torque: float = 0.0 # Drive torque delivered to rear axle contact patch (Nm)
    engine_rpm: float = RPM_IDLE    # Engine crankshaft speed (RPM)
    gear: int = 0                   # Current gear (-1=R, 0=N, 1..8)
    is_shifting: bool = False       # True during 40ms seamless shift
    shift_progress: float = 0.0     # 0.0 to 1.0 shift progress
    is_blipping: bool = False       # True during downshift auto-blip
    clutch_engagement: float = 1.0  # 0.0 (disengaged) to 1.0 (fully locked)
    anti_stall_active: bool = False # True if anti-stall holding clutch open


class EngineModel:
    """1.6L 90° V6 Turbo Hybrid ICE + MGU-K engine model."""
    def __init__(self) -> None:
        self.rpm: float = 0.0
        self.ignition_state: IgnitionState = IgnitionState.OFF
        self.crank_timer: float = 0.0
        self.flare_timer: float = 0.0
        self.anti_stall_active: bool = False

    def set_ignition(self, on: bool) -> None:
        """Toggles ignition power switch."""
        if on and self.ignition_state == IgnitionState.OFF:
            self.ignition_state = IgnitionState.IGNITION_ON
            self.rpm = 0.0
        elif not on:
            self.kill_engine()

    def press_starter(self, dt: float = 0.0) -> None:
        """Called when starter button is held down."""
        if self.ignition_state == IgnitionState.IGNITION_ON:
            self.ignition_state = IgnitionState.CRANKING
            self.crank_timer = 0.0

    def start_cranking(self) -> None:
        """Initiates engine cranking."""
        self.press_starter(0.0)

    def start_engine_direct(self) -> None:
        """Starts engine directly into RUNNING state at idle RPM."""
        self.ignition_state = IgnitionState.RUNNING
        self.rpm = RPM_IDLE
        self.flare_timer = 0.0
        self.crank_timer = 0.80

    def release_starter(self) -> None:
        """Called when starter button is released before full crank."""
        if self.ignition_state == IgnitionState.CRANKING and self.crank_timer < 0.80:
            self.ignition_state = IgnitionState.IGNITION_ON
            self.crank_timer = 0.0
            self.rpm = 0.0

    def kill_engine(self) -> None:
        self.ignition_state = IgnitionState.OFF
        self.rpm = 0.0
        self.crank_timer = 0.0
        self.flare_timer = 0.0
        self.anti_stall_active = False

    def compute_ice_torque(self, rpm: float, throttle: float) -> float:
        """Calculates ICE torque based on RPM, fuel flow regulations, and throttle."""
        if self.ignition_state != IgnitionState.RUNNING or rpm < 500.0:
            return 0.0
        if rpm > RPM_REV_LIMITER_CUT:
            return 0.0

        omega = max(1.0, rpm * (2.0 * math.pi / 60.0))
        t_mguk_max = min(PEAK_TORQUE_MGUK_NM, (PEAK_POWER_MGUK_KW * 1000.0) / omega)

        # 3 Regimes
        if rpm <= 10500.0:
            t_ice = 420.0 + 80.0 * (1.0 - ((10500.0 - rpm) / 6500.0) ** 2)
        elif rpm <= RPM_AUTO_UPSHIFT:
            # Power saturation plateau ~ 746 kW total
            t_total = (PEAK_POWER_TOTAL_KW * 1000.0) / omega
            t_ice = t_total - t_mguk_max
        else:
            # High-RPM taper above 13,200 RPM
            t_total = ((PEAK_POWER_TOTAL_KW * 1000.0) / omega) * (1.0 - 0.15 * ((rpm - RPM_AUTO_UPSHIFT) / 1800.0))
            t_ice = t_total - t_mguk_max

        t_ice = max(180.0, min(PEAK_TORQUE_ICE_NM, t_ice))
        if throttle > 0.0:
            return throttle * t_ice
        else:
            # Zero throttle engine braking
            return -(ENGINE_BRAKE_BASE_NM + ENGINE_BRAKE_HIGH_RPM_NM * (rpm / RPM_REDLINE))

    def compute_mguk_torque(self, rpm: float, throttle: float) -> float:
        """Calculates instantaneous electric MGU-K motor boost torque."""
        if self.ignition_state != IgnitionState.RUNNING or throttle <= 0.0:
            return 0.0
        omega = max(1.0, rpm * (2.0 * math.pi / 60.0))
        t_mguk = min(PEAK_TORQUE_MGUK_NM, (PEAK_POWER_MGUK_KW * 1000.0) / omega)
        return throttle * t_mguk

    def update(self, dt: float, throttle: float, target_driveline_rpm: float | None = None) -> EngineOutput:
        """Advances engine state machine, calculates net torque and fuel flow."""
        # 1. State machine progression
        if self.ignition_state == IgnitionState.OFF:
            self.rpm = 0.0
            return EngineOutput()

        elif self.ignition_state == IgnitionState.IGNITION_ON:
            self.rpm = 0.0
            return EngineOutput()

        elif self.ignition_state == IgnitionState.CRANKING:
            self.crank_timer += dt
            self.rpm = min(RPM_CRANKING_TARGET, RPM_CRANKING_TARGET * (self.crank_timer / 0.80))
            if self.crank_timer >= 0.80:
                self.ignition_state = IgnitionState.RUNNING
                self.flare_timer = 0.60
                self.rpm = RPM_FIRE_FLARE
            return EngineOutput(torque_net=0.0, power_kw=0.0)

        elif self.ignition_state == IgnitionState.RUNNING:
            if self.flare_timer > 0.0:
                self.flare_timer -= dt
                prog = max(0.0, self.flare_timer / 0.60)
                self.rpm = RPM_IDLE + (RPM_FIRE_FLARE - RPM_IDLE) * prog
            elif target_driveline_rpm is not None and not self.anti_stall_active:
                self.rpm = max(RPM_IDLE, target_driveline_rpm)
            else:
                self.rpm = max(RPM_IDLE, self.rpm)

            # Rev limiter
            if self.rpm > RPM_REV_LIMITER_CUT:
                self.rpm = RPM_REDLINE

            t_ice = self.compute_ice_torque(self.rpm, throttle)
            t_mguk = self.compute_mguk_torque(self.rpm, throttle)
            t_net = t_ice + t_mguk
            omega = self.rpm * (2.0 * math.pi / 60.0)
            power_kw = (t_net * omega) / 1000.0 if t_net > 0 else 0.0
            fuel_flow = min(100.0, 100.0 * (self.rpm / 10500.0) * throttle) if self.rpm > 10500.0 else (self.rpm / 10500.0) * 100.0 * throttle

            return EngineOutput(
                torque_net=t_net,
                torque_ice=t_ice,
                torque_mguk=t_mguk,
                power_kw=power_kw,
                fuel_flow_kgh=fuel_flow
            )
        return EngineOutput()


class Gearbox:
    """8-speed seamless sequential gearbox with Reverse and auto/manual control."""
    def __init__(self, mode: TransmissionMode = TransmissionMode.MANUAL) -> None:
        self.gear: int = 0
        self.mode: TransmissionMode = mode
        self.shift_timer: float = 0.0
        self.blip_timer: float = 0.0
        self.kickdown_cooldown: float = 0.0
        self.clutch_engagement: float = 1.0

    def get_ratio(self, gear: int | None = None) -> float:
        g = self.gear if gear is None else gear
        return GEAR_RATIOS.get(g, 0.0)

    def get_overall_ratio(self, gear: int | None = None) -> float:
        return self.get_ratio(gear) * FINAL_DRIVE_RATIO

    def calculate_driveline_rpm(self, wheel_speed_ms: float, gear: int | None = None) -> float:
        overall = self.get_overall_ratio(gear) * 1.52
        if abs(overall) < 1e-4:
            return 0.0
        wheel_rpm = (abs(wheel_speed_ms) / (2.0 * math.pi * TYRE_RADIUS)) * 60.0
        return wheel_rpm * abs(overall)

    def calculate_engine_rpm(self, wheel_speed_ms: float, gear: int | None = None) -> float:
        driveline_rpm = self.calculate_driveline_rpm(wheel_speed_ms, gear)
        return max(RPM_IDLE, min(RPM_REDLINE, driveline_rpm))

    def request_upshift(self) -> bool:
        if self.shift_timer > 0.0:
            return False
        if self.gear < 8:
            self.gear += 1
            self.shift_timer = SEAMLESS_SHIFT_DURATION
            return True
        return False

    def request_downshift(self, wheel_speed_ms: float) -> bool:
        if self.shift_timer > 0.0:
            return False
        target_g = self.gear - 1
        if target_g < -1:
            return False
        # Reverse downshift lockout: reject shift from Neutral to Reverse when forward speed > 1 km/h (0.28 m/s)
        if target_g == -1 and wheel_speed_ms > 0.28:
            return False
        # Overrev protection check
        if target_g > 0:
            dest_rpm = self.calculate_engine_rpm(wheel_speed_ms, target_g)
            if dest_rpm > DOWNSHIFT_OVERREV_LIMIT_RPM:
                return False  # Overrev rejected!

        self.gear = target_g
        self.shift_timer = SEAMLESS_SHIFT_DURATION
        if target_g > 0:
            self.blip_timer = THROTTLE_BLIP_DURATION
        return True

    def set_gear_direct(self, gear: int) -> None:
        self.gear = max(-1, min(8, gear))
        self.shift_timer = 0.0
        self.blip_timer = 0.0

    def update(self, dt: float, engine_rpm: float, throttle: float, speed_kmh: float) -> None:
        if self.shift_timer > 0.0:
            self.shift_timer = max(0.0, self.shift_timer - dt)
        if self.blip_timer > 0.0:
            self.blip_timer = max(0.0, self.blip_timer - dt)
        if self.kickdown_cooldown > 0.0:
            self.kickdown_cooldown = max(0.0, self.kickdown_cooldown - dt)

        # Automatic transmission control
        if self.mode == TransmissionMode.AUTOMATIC and self.shift_timer <= 0.0:
            wheel_speed_ms = speed_kmh / 3.6
            # Upshift
            if engine_rpm >= RPM_AUTO_UPSHIFT and throttle > 0.50 and 1 <= self.gear < 8:
                self.request_upshift()
            # Kickdown
            elif throttle >= 0.95 and engine_rpm <= RPM_KICKDOWN_THRESHOLD and self.gear >= 2 and self.kickdown_cooldown <= 0.0:
                if self.request_downshift(wheel_speed_ms):
                    self.kickdown_cooldown = 1.0
            # Downshift (only when not in heavy acceleration)
            elif throttle < 0.50 and engine_rpm <= RPM_AUTO_DOWNSHIFT and self.gear > 1 and speed_kmh > 15.0:
                self.request_downshift(wheel_speed_ms)


class Powertrain:
    """Integrated engine, gearbox, and driveline dynamics subsystem."""
    def __init__(self, mode: TransmissionMode = TransmissionMode.MANUAL) -> None:
        self.engine: EngineModel = EngineModel()
        self.gearbox: Gearbox = Gearbox(mode=mode)
        self.anti_stall_active: bool = False

    @property
    def rpm(self) -> float:
        return self.engine.rpm

    @property
    def gear(self) -> int:
        return self.gearbox.gear

    @property
    def is_running(self) -> bool:
        return self.engine.ignition_state == IgnitionState.RUNNING

    def set_running(self) -> None:
        """Sets powertrain directly to RUNNING state for immediate simulation / testing."""
        self.engine.ignition_state = IgnitionState.RUNNING
        self.engine.rpm = RPM_IDLE
        self.engine.crank_timer = 0.80
        self.engine.flare_timer = 0.0
        self.anti_stall_active = False
        self.gearbox.clutch_engagement = 1.0

    def step(
        self,
        throttle: float,
        brake: float,
        wheel_speed_ms: float,
        dt: float,
        shift_cmd: int = 0
    ) -> TransmissionOutput:
        """
        Executes one physics integration step.
        shift_cmd: +1 for upshift, -1 for downshift, 0 for none.
        """
        # 1. Manual shift commands
        if self.gearbox.mode == TransmissionMode.MANUAL:
            if shift_cmd > 0:
                self.gearbox.request_upshift()
            elif shift_cmd < 0:
                self.gearbox.request_downshift(wheel_speed_ms)

        # 2. Driveline RPM
        driveline_rpm = self.gearbox.calculate_driveline_rpm(wheel_speed_ms)
        target_rpm = max(RPM_IDLE, min(RPM_REDLINE, driveline_rpm))

        # 3. Anti-Stall Check
        if self.gearbox.gear != 0 and self.engine.ignition_state == IgnitionState.RUNNING:
            if throttle > 0.15 or self.gearbox.gear == 0:
                self.anti_stall_active = False
                self.gearbox.clutch_engagement = min(1.0, self.gearbox.clutch_engagement + dt / 0.20)
            elif driveline_rpm < RPM_STALL_THRESHOLD:
                self.anti_stall_active = True
                self.gearbox.clutch_engagement = 0.0
            else:
                self.anti_stall_active = False
                self.gearbox.clutch_engagement = min(1.0, self.gearbox.clutch_engagement + dt / 0.20)
        else:
            self.anti_stall_active = False
            self.gearbox.clutch_engagement = 1.0 if self.gearbox.gear != 0 else 0.0

        self.engine.anti_stall_active = self.anti_stall_active

        # 4. Auto-blip throttle injection
        effective_throttle = throttle
        if self.gearbox.blip_timer > 0.0:
            effective_throttle = max(throttle, THROTTLE_BLIP_AMOUNT)

        # 5. Engine step
        eng_out = self.engine.update(dt, effective_throttle, target_rpm if not self.anti_stall_active else None)

        # 6. Automatic transmission step
        self.gearbox.update(dt, self.engine.rpm, effective_throttle, abs(wheel_speed_ms) * 3.6)

        # 7. Shifting torque handover modulation
        shift_factor = 1.0
        if self.gearbox.shift_timer > 0.0:
            # During 40ms seamless shift, torque is 60%
            shift_factor = 0.60

        overall_ratio = self.gearbox.get_overall_ratio() * 1.52
        wheel_torque = eng_out.torque_net * overall_ratio * TRANSMISSION_EFFICIENCY * shift_factor * self.gearbox.clutch_engagement

        return TransmissionOutput(
            wheel_drive_torque=wheel_torque,
            engine_rpm=self.engine.rpm,
            gear=self.gearbox.gear,
            is_shifting=self.gearbox.shift_timer > 0.0,
            shift_progress=1.0 - (self.gearbox.shift_timer / SEAMLESS_SHIFT_DURATION) if self.gearbox.shift_timer > 0 else 1.0,
            is_blipping=self.gearbox.blip_timer > 0.0,
            clutch_engagement=self.gearbox.clutch_engagement,
            anti_stall_active=self.anti_stall_active
        )

    def get_drive_torque(self, throttle: float, gear: int, speed_ms: float) -> float:
        """Convenience method for calculating total rear axle drive torque."""
        if not self.is_running or gear == 0:
            return 0.0
        self.gearbox.gear = gear
        target_rpm = self.gearbox.calculate_engine_rpm(speed_ms, gear)
        eng_out = self.engine.update(SIM_DT, throttle, target_rpm)
        overall = self.gearbox.get_overall_ratio(gear)
        return eng_out.torque_net * overall * TRANSMISSION_EFFICIENCY
