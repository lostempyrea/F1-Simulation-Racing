"""
f1sim/physics/aero.py
Formula 1 Aerodynamic Downforce, Drag, and DRS Mechanics Simulation.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
import numpy as np


class DRSState(Enum):
    """DRS Cockpit & Flap Operating States."""
    DISABLED = auto()   # Not in DRS zone, or speed below minimum activation threshold
    AVAILABLE = auto()  # Inside DRS zone, within activation criteria (green dash indicator)
    ACTIVE = auto()     # Wing flap actuator deployed (85mm slot open, drag reduced)


class AeroSetupPreset(Enum):
    """Aerodynamic trim setups for specific circuit characteristics."""
    MONZA = auto()      # Low drag / low downforce (high top speed)
    MEDIUM = auto()     # Standard balanced setup (default)
    MONACO = auto()     # High downforce / high drag (maximum cornering grip)


@dataclass(frozen=True)
class AeroForces:
    """Calculated aerodynamic forces and balance at a specific timestep."""
    downforce_total: float       # Total vertical downforce (N, positive downward)
    downforce_front: float       # Front axle downforce (N)
    downforce_rear: float        # Rear axle downforce (N)
    drag_total: float            # Total aerodynamic resistance (N, opposes motion)
    aero_balance_front: float    # Fraction of downforce acting on front axle (0.0 .. 1.0)
    flap_position: float         # DRS flap opening status (0.0 = closed, 1.0 = fully open)


@dataclass
class AeroConfig:
    """Physical aerodynamic configuration constants."""
    air_density: float = 1.225              # rho (kg/m^3) at sea level ISA
    frontal_area: float = 1.60              # Reference area A (m^2)
    c_lift_nominal: float = 2.90            # Nominal lift coefficient C_L
    c_drag_nominal: float = 0.95            # Nominal drag coefficient C_D
    front_balance: float = 0.44             # Nominal front downforce distribution (44% front)
    drs_drag_reduction: float = 0.28        # 28% drag reduction when DRS is active
    drs_rear_downforce_reduction: float = 0.32  # 32% rear downforce loss when DRS is active
    drs_actuator_time: float = 0.15         # Transition duration (seconds) to open/close flap
    drs_min_speed_kmh: float = 100.0        # Minimum speed required to deploy DRS
    drs_brake_threshold: float = 0.05       # Brake input threshold that triggers instant deactivation
    drs_lateral_g_threshold: float = 0.40   # Lateral G threshold that triggers auto-closure


class Aerodynamics:
    """
    Simulates speed-squared aerodynamic downforce, drag, and DRS flap mechanics.
    """
    def __init__(self, config: AeroConfig | None = None) -> None:
        self.config: AeroConfig = config if config is not None else AeroConfig()

        # Current active aerodynamic coefficients
        self.c_lift: float = self.config.c_lift_nominal
        self.c_drag: float = self.config.c_drag_nominal
        self.front_balance: float = self.config.front_balance

        # DRS dynamic state
        self.drs_state: DRSState = DRSState.DISABLED
        self.flap_position: float = 0.0     # 0.0 (closed) to 1.0 (open)
        self.drs_permitted: bool = False    # Controlled by track zone & regulations

        # Environmental / Slipstream scaling
        self.slipstream_drag_scalar: float = 1.0  # < 1.0 when trailing in tow
        self.dirty_air_lift_scalar: float = 1.0   # < 1.0 when cornering in wake

    def apply_preset(self, preset: AeroSetupPreset) -> None:
        """Configures aerodynamic coefficients for specific circuit types."""
        if preset == AeroSetupPreset.MONZA:
            self.c_lift = 1.80
            self.c_drag = 0.75
            self.front_balance = 0.43
        elif preset == AeroSetupPreset.MONACO:
            self.c_lift = 3.60
            self.c_drag = 1.15
            self.front_balance = 0.45
        elif preset == AeroSetupPreset.MEDIUM:
            self.c_lift = self.config.c_lift_nominal
            self.c_drag = self.config.c_drag_nominal
            self.front_balance = self.config.front_balance

    def set_slipstream_factors(self, drag_scalar: float = 1.0, lift_scalar: float = 1.0) -> None:
        """Applies external wake effects from trailing another car."""
        self.slipstream_drag_scalar = float(np.clip(drag_scalar, 0.65, 1.0))
        self.dirty_air_lift_scalar = float(np.clip(lift_scalar, 0.80, 1.0))

    def request_drs_toggle(self) -> bool:
        """
        Called when driver presses cockpit DRS toggle button.
        Returns True if DRS successfully activated or closed, False if rejected.
        """
        if self.drs_state == DRSState.AVAILABLE:
            self.drs_state = DRSState.ACTIVE
            return True
        elif self.drs_state == DRSState.ACTIVE:
            self.drs_state = DRSState.AVAILABLE if self.drs_permitted else DRSState.DISABLED
            return True
        return False

    def force_close_drs(self) -> None:
        """Immediately closes DRS wing flap due to safety override."""
        if self.drs_state == DRSState.ACTIVE:
            self.drs_state = DRSState.AVAILABLE if self.drs_permitted else DRSState.DISABLED

    def compute(self, speed_ms: float, drs_active: bool = False) -> tuple[float, float, float, float]:
        """
        Convenience computation returning (f_down_total, f_drag_total, f_down_front, f_down_rear).
        """
        speed_pos = max(0.0, speed_ms)
        q = 0.5 * self.config.air_density * (speed_pos ** 2)

        cd = self.c_drag * self.slipstream_drag_scalar * (1.0 - self.config.drs_drag_reduction if drs_active else 1.0)
        f_drag = q * cd * self.config.frontal_area

        f_down_base = q * self.c_lift * self.config.frontal_area
        f_down_front = f_down_base * self.front_balance
        rear_factor = (1.0 - self.config.drs_rear_downforce_reduction) if drs_active else 1.0
        f_down_rear = f_down_base * (1.0 - self.front_balance) * rear_factor
        f_down_total = f_down_front + f_down_rear
        return f_down_total, f_drag, f_down_front, f_down_rear

    def update(
        self,
        speed_ms: float,
        dt: float,
        in_drs_zone: bool,
        brake_input: float,
        lateral_g: float,
        trailing_gap_sec: float | None = None
    ) -> AeroForces:
        """
        Advances the aerodynamic simulation by timestep dt.
        """
        speed_kmh = speed_ms * 3.6
        speed_positive = max(0.0, speed_ms)

        # 1. Update DRS eligibility & state machine
        eligible_zone = in_drs_zone and (speed_kmh >= self.config.drs_min_speed_kmh)
        eligible_gap = (trailing_gap_sec is None) or (trailing_gap_sec <= 1.0)
        self.drs_permitted = eligible_zone and eligible_gap

        # Check safety auto-closure conditions
        safety_trip = (brake_input > self.config.drs_brake_threshold) or (abs(lateral_g) > self.config.drs_lateral_g_threshold)

        if not eligible_zone or safety_trip:
            if self.drs_state == DRSState.ACTIVE:
                self.drs_state = DRSState.AVAILABLE if self.drs_permitted and not safety_trip else DRSState.DISABLED
            elif not eligible_zone:
                self.drs_state = DRSState.DISABLED
        elif self.drs_permitted and self.drs_state == DRSState.DISABLED:
            self.drs_state = DRSState.AVAILABLE

        # 2. Slew flap actuator position towards target
        target_flap = 1.0 if self.drs_state == DRSState.ACTIVE else 0.0
        if self.flap_position < target_flap:
            self.flap_position = min(target_flap, self.flap_position + dt / self.config.drs_actuator_time)
        elif self.flap_position > target_flap:
            self.flap_position = max(target_flap, self.flap_position - dt / self.config.drs_actuator_time)

        # 3. Dynamic coefficients
        effective_cd = self.c_drag * self.slipstream_drag_scalar * (1.0 - self.config.drs_drag_reduction * self.flap_position)
        effective_cl = self.c_lift * self.dirty_air_lift_scalar

        # Dynamic pressure: q = 0.5 * rho * v^2
        q = 0.5 * self.config.air_density * (speed_positive ** 2)

        # Drag and Downforce
        f_drag = q * effective_cd * self.config.frontal_area
        f_down_base = q * effective_cl * self.config.frontal_area

        f_down_front = f_down_base * self.front_balance
        rear_loss_factor = 1.0 - (self.config.drs_rear_downforce_reduction * self.flap_position)
        f_down_rear = f_down_base * (1.0 - self.front_balance) * rear_loss_factor
        f_down_total = f_down_front + f_down_rear

        aero_balance_front = (f_down_front / f_down_total) if f_down_total > 1e-3 else self.front_balance

        return AeroForces(
            downforce_total=f_down_total,
            downforce_front=f_down_front,
            downforce_rear=f_down_rear,
            drag_total=f_drag,
            aero_balance_front=aero_balance_front,
            flap_position=self.flap_position
        )
