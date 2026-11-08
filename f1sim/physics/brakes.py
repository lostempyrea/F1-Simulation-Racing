"""
f1sim/physics/brakes.py
Formula 1 Carbon-Carbon Disc Thermal Model and Cockpit Brake Bias Distribution.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass
class BrakeConfig:
    """Physical parameters of the F1 carbon-carbon braking system."""
    default_bias: float = 0.58             # 58.0% front default
    min_bias: float = 0.52                 # 52.0% minimum front bias
    max_bias: float = 0.64                 # 64.0% maximum front bias
    bias_step: float = 0.005               # 0.5% per cockpit toggle step

    max_system_torque: float = 13000.0     # Total maximum braking torque across 4 wheels (Nm)
    disc_mass_kg: float = 1.80             # Mass of single carbon brake disc (kg)
    disc_specific_heat: float = 1400.0     # Specific heat capacity c_p (J/(kg*K))
    cooling_area_m2: float = 0.18          # Effective disc cooling surface area (m^2)
    ambient_temp_c: float = 25.0           # Ambient air temperature (deg C)

    # Cooling coefficients
    h_convection_base: float = 25.0        # Static convective cooling coefficient W/(m^2*K)
    k_convection_speed: float = 12.5       # Duct forced cooling gain per m/s
    radiation_emissivity: float = 0.85     # Carbon composite emissivity epsilon


@dataclass(frozen=True)
class BrakeTorqueOutput:
    """Instantaneous braking torques applied to the 4 wheel spindles (Nm)."""
    torque_fl: float
    torque_fr: float
    torque_rl: float
    torque_rr: float
    total_torque: float


class BrakeDisc:
    """Simulates thermal behavior and pad friction for a single carbon-carbon disc."""

    def __init__(self, config: BrakeConfig, initial_temp_c: float = 400.0) -> None:
        self.config: BrakeConfig = config
        self.temperature_c: float = initial_temp_c
        self.thermal_mass: float = config.disc_mass_kg * config.disc_specific_heat  # 2520 J/K
        self.sigma: float = 5.670374e-8    # Stefan-Boltzmann constant

    @property
    def pad_friction_coefficient(self) -> float:
        """
        Evaluates the non-linear carbon-carbon friction coefficient mu_pad(T).
        Cold glaze < 200C, optimal plateau 350C-950C (mu=0.52), fade > 1000C.
        """
        t = self.temperature_c
        if t < 100.0:
            return 0.25
        elif t < 350.0:
            return 0.25 + (0.52 - 0.25) * ((t - 100.0) / 250.0)
        elif t <= 950.0:
            return 0.52
        elif t <= 1200.0:
            return 0.52 - (0.52 - 0.32) * ((t - 950.0) / 250.0)
        else:
            return 0.30

    def step_thermal(self, brake_torque: float, wheel_speed_rads: float, vehicle_speed_ms: float, dt: float) -> float:
        """
        Computes heat generation, convective duct cooling, and radiation over dt.
        Returns the updated disc temperature in deg C.
        """
        # 1. Mechanical work heat generation (Watts)
        p_heat = abs(brake_torque * wheel_speed_rads)

        # 2. Forced convective cooling through brake duct (Watts)
        temp_diff = self.temperature_c - self.config.ambient_temp_c
        h_eff = self.config.h_convection_base + self.config.k_convection_speed * max(0.0, vehicle_speed_ms)
        p_conv = h_eff * self.config.cooling_area_m2 * temp_diff

        # 3. Radiative cooling (Stefan-Boltzmann, Watts)
        t_kelvin = self.temperature_c + 273.15
        t_amb_kelvin = self.config.ambient_temp_c + 273.15
        p_rad = self.config.radiation_emissivity * self.sigma * self.config.cooling_area_m2 * (
            (t_kelvin ** 4) - (t_amb_kelvin ** 4)
        )

        # 4. Integrate net thermal flux
        p_net = p_heat - p_conv - p_rad
        delta_temp = (p_net / self.thermal_mass) * dt
        self.temperature_c = max(self.config.ambient_temp_c, self.temperature_c + delta_temp)
        return self.temperature_c


class BrakeSystem:
    """
    Manages cockpit brake bias distribution, hydraulic torque generation,
    and 4-wheel carbon-carbon disc thermal states.
    """

    def __init__(self, config: BrakeConfig | None = None) -> None:
        self.config: BrakeConfig = config if config is not None else BrakeConfig()
        self.brake_bias: float = self.config.default_bias  # e.g. 0.58

        # 4 independent disc thermal units [FL, FR, RL, RR]
        self.discs: list[BrakeDisc] = [
            BrakeDisc(self.config, initial_temp_c=400.0),
            BrakeDisc(self.config, initial_temp_c=400.0),
            BrakeDisc(self.config, initial_temp_c=400.0),
            BrakeDisc(self.config, initial_temp_c=400.0),
        ]

    def adjust_bias(self, direction: int) -> float:
        """
        Adjusts brake bias up (+1) towards front, or down (-1) towards rear.
        Returns the new brake bias clamped to [min_bias, max_bias].
        """
        step = self.config.bias_step if direction > 0 else -self.config.bias_step
        self.brake_bias = float(np.clip(self.brake_bias + step, self.config.min_bias, self.config.max_bias))
        return self.brake_bias

    def set_bias(self, bias: float) -> float:
        """Directly sets brake bias ratio."""
        self.brake_bias = float(np.clip(bias, self.config.min_bias, self.config.max_bias))
        return self.brake_bias

    def get_bias(self) -> float:
        """Returns the current front brake bias ratio (e.g. 0.58)."""
        return self.brake_bias

    def get_temperatures(self) -> np.ndarray:
        """Returns array of disc temperatures [FL, FR, RL, RR] in deg C."""
        return np.array([d.temperature_c for d in self.discs], dtype=np.float64)

    @property
    def disc_temperatures(self) -> np.ndarray:
        return self.get_temperatures()

    def reset_temperatures(self, temp_c: float = 400.0) -> None:
        """Resets all disc temperatures to target value."""
        for d in self.discs:
            d.temperature_c = temp_c

    def get_wheel_torques(self, pedal_input: float, brake_bias: float | None = None) -> np.ndarray:
        """Convenience method returning 4-element array of torques for vehicle simulation."""
        bias = self.brake_bias if brake_bias is None else brake_bias
        pedal = float(np.clip(pedal_input, 0.0, 1.0))
        total_demand = pedal * self.config.max_system_torque
        front_demand = total_demand * bias
        rear_demand = total_demand * (1.0 - bias)
        nominal_torques = np.array([
            0.5 * front_demand,
            0.5 * front_demand,
            0.5 * rear_demand,
            0.5 * rear_demand,
        ], dtype=np.float64)
        applied = np.zeros(4, dtype=np.float64)
        for i in range(4):
            mu = self.discs[i].pad_friction_coefficient
            applied[i] = nominal_torques[i] * (mu / 0.52)
        return applied

    def update(
        self,
        pedal_input: float,
        wheel_speeds_rads: np.ndarray,
        vehicle_speed_ms: float,
        dt: float
    ) -> BrakeTorqueOutput:
        """
        Updates braking system, computes torques per wheel, and steps thermal models.
        """
        pedal = float(np.clip(pedal_input, 0.0, 1.0))
        total_demand = pedal * self.config.max_system_torque

        front_demand = total_demand * self.brake_bias
        rear_demand = total_demand * (1.0 - self.brake_bias)

        nominal_torques = np.array([
            0.5 * front_demand,  # FL
            0.5 * front_demand,  # FR
            0.5 * rear_demand,   # RL
            0.5 * rear_demand,   # RR
        ], dtype=np.float64)

        applied_torques = np.zeros(4, dtype=np.float64)
        for i in range(4):
            mu = self.discs[i].pad_friction_coefficient
            applied_torques[i] = nominal_torques[i] * (mu / 0.52)

            self.discs[i].step_thermal(
                brake_torque=applied_torques[i],
                wheel_speed_rads=float(wheel_speeds_rads[i]),
                vehicle_speed_ms=vehicle_speed_ms,
                dt=dt
            )

        return BrakeTorqueOutput(
            torque_fl=float(applied_torques[0]),
            torque_fr=float(applied_torques[1]),
            torque_rl=float(applied_torques[2]),
            torque_rr=float(applied_torques[3]),
            total_torque=float(np.sum(applied_torques))
        )
