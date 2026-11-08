"""
f1sim/input/filters.py
======================
Signal processing filters for Formula 1 cockpit steering, pedal inputs,
analog gamepad sticks/triggers, speed-sensitive lock attenuation, and
slew-rate limiting.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np


def apply_deadzone(value: float, deadzone: float = 0.08, rescale: bool = False) -> float:
    """
    Applies a radial/axial deadzone to an analog input in [-1.0, 1.0].
    Inputs with |value| < deadzone clamp strictly to 0.0.
    If rescale is True, values outside the deadzone are mapped smoothly to [0.0, 1.0].
    """
    deadzone = max(0.0, min(0.95, float(deadzone)))
    val = max(-1.0, min(1.0, float(value)))
    if abs(val) < deadzone:
        return 0.0
    if rescale and deadzone > 0.0:
        norm = (abs(val) - deadzone) / (1.0 - deadzone)
        return math.copysign(min(1.0, max(0.0, norm)), val)
    return val


def apply_cubic_gamma(value: float, gamma: float = 0.45) -> float:
    """
    Applies a blended cubic sensitivity curve:
        f(x) = sign(x) * (gamma * |x|^3 + (1 - gamma) * |x|)
    Preserves fine steering precision near center while maintaining full authority at lock.
    """
    gamma = max(0.0, min(1.0, float(gamma)))
    x = max(-1.0, min(1.0, float(value)))
    ax = abs(x)
    curved = gamma * (ax ** 3) + (1.0 - gamma) * ax
    return math.copysign(curved, x) if ax > 0.0 else 0.0


def normalize_trigger(raw_value: float, max_raw: float = 255.0, deadzone: float = 0.0) -> float:
    """
    Normalizes an 8-bit (0..255), 16-bit (0..32767), or float trigger input
    strictly into the [0.0, 1.0] interval.
    """
    if max_raw <= 0.0:
        return 0.0
    norm = float(raw_value) / float(max_raw)
    norm = max(0.0, min(1.0, norm))
    if deadzone > 0.0:
        if norm < deadzone:
            return 0.0
        norm = (norm - deadzone) / max(1e-6, 1.0 - deadzone)
    return max(0.0, min(1.0, norm))


def compute_speed_attenuation(
    speed_ms: float,
    max_speed_ref: float = 100.0,
    min_factor: float = 0.12,
    slope: float = 0.85,
) -> float:
    """
    Computes speed-sensitive steering attenuation factor in [min_factor, 1.0].
    Prevents unrealistic high-speed snap-spin from full steering lock at 300+ km/h.
    """
    v = abs(float(speed_ms))
    ref = max(1.0, float(max_speed_ref))
    factor = 1.0 - (v / ref) * float(slope)
    return max(float(min_factor), min(1.0, factor))


def blend_inputs(keyboard_val: float, gamepad_val: float) -> float:
    """
    Seamlessly blends or hot-swaps between keyboard and gamepad inputs
    by selecting the input source with greater active deflection magnitude.
    """
    kb = float(keyboard_val)
    gp = float(gamepad_val)
    return gp if abs(gp) > abs(kb) else kb


@dataclass
class SteeringFilterConfig:
    """Configuration parameters for F1 cockpit steering kinematics and filtering."""
    max_wheel_lock_rad: float = math.pi       # +/- 180 deg (+/- pi rad) visual wheel lock
    max_road_steer_rad: float = 0.900         # Max road wheel steer angle (rad)
    slew_rate_turn_in: float = 3.5            # Turn-in slew rate limit (rad/s)
    slew_rate_return: float = 6.0             # Centering return slew rate limit (rad/s)
    deadzone: float = 0.08                    # Analog stick deadzone
    cubic_gamma: float = 0.45                 # Cubic steering curve blend factor
    min_speed_attenuation: float = 0.12       # Minimum steering authority at top speed
    speed_attenuation_slope: float = 0.85     # Speed attenuation slope per 100 m/s


class SteeringFilter:
    """
    Stateful F1 steering filter managing deadzone, non-linear gamma curve,
    speed-sensitive lock attenuation, and slew-rate limiting.
    """

    def __init__(self, config: SteeringFilterConfig | None = None) -> None:
        self.cfg: SteeringFilterConfig = config if config is not None else SteeringFilterConfig()
        self.wheel_angle_rad: float = 0.0     # In [-pi, +pi]
        self.road_steer_rad: float = 0.0      # In [-max_road_steer_rad, +max_road_steer_rad]
        self.normalized_steer: float = 0.0    # In [-1.0, +1.0]

    def reset(self) -> None:
        """Resets steering angles to center (0.0)."""
        self.wheel_angle_rad = 0.0
        self.road_steer_rad = 0.0
        self.normalized_steer = 0.0

    def step_slew_limit(
        self,
        current_angle: float,
        target_angle: float,
        dt: float,
        slew_rate: float | None = None,
    ) -> float:
        """
        Applies rate-limited angle change over time step dt.
        """
        if dt <= 0.0:
            return float(current_angle)
        delta = float(target_angle) - float(current_angle)
        if abs(delta) < 1e-9:
            return float(target_angle)

        if slew_rate is not None:
            rate = float(slew_rate)
        else:
            # Faster centering return when moving back toward 0 or reversing sign
            is_returning = (
                abs(target_angle) < abs(current_angle)
                or (target_angle * current_angle < 0.0)
            )
            rate = self.cfg.slew_rate_return if is_returning else self.cfg.slew_rate_turn_in

        max_step = max(0.0, rate * float(dt))
        step_change = min(abs(delta), max_step)
        return float(current_angle) + math.copysign(step_change, delta)

    def update(
        self,
        raw_input: float,
        speed_ms: float = 0.0,
        dt: float = 1.0 / 120.0,
        is_analog: bool = False,
        sensitivity: float = 1.0,
        invert: bool = False,
    ) -> tuple[float, float]:
        """
        Updates the steering filter state for one frame/step.

        Parameters
        ----------
        raw_input : float
            Normalized steering demand in [-1.0, +1.0] (positive = left or right per convention).
        speed_ms : float
            Current forward speed in m/s for speed-sensitive attenuation.
        dt : float
            Time step in seconds.
        is_analog : bool
            If True, applies analog deadzone and cubic gamma curve.
        sensitivity : float
            User-configurable steering sensitivity multiplier (0.2 .. 2.5).
        invert : bool
            If True, flips steering sign.

        Returns
        -------
        tuple[float, float]
            (road_steer_rad, wheel_angle_rad)
        """
        x = max(-1.0, min(1.0, float(raw_input)))
        if invert:
            x = -x

        if is_analog:
            x = apply_deadzone(x, self.cfg.deadzone, rescale=True)
            x = apply_cubic_gamma(x, self.cfg.cubic_gamma)

        sens = max(0.2, min(2.5, float(sensitivity)))
        x = max(-1.0, min(1.0, x * sens))

        # Slew-limit the normalized steering position
        target_norm = x
        self.normalized_steer = self.step_slew_limit(
            self.normalized_steer,
            target_norm,
            dt,
            slew_rate=self.cfg.slew_rate_turn_in if abs(target_norm) >= abs(self.normalized_steer) else self.cfg.slew_rate_return,
        )
        self.normalized_steer = max(-1.0, min(1.0, self.normalized_steer))

        # Visual steering wheel rotation in [-pi, +pi]
        self.wheel_angle_rad = float(
            np.clip(
                self.normalized_steer * self.cfg.max_wheel_lock_rad,
                -self.cfg.max_wheel_lock_rad,
                +self.cfg.max_wheel_lock_rad,
            )
        )

        # Speed-attenuated road wheel steer angle
        attenuation = compute_speed_attenuation(
            speed_ms,
            max_speed_ref=100.0,
            min_factor=self.cfg.min_speed_attenuation,
            slope=self.cfg.speed_attenuation_slope,
        )
        self.road_steer_rad = float(
            np.clip(
                self.normalized_steer * self.cfg.max_road_steer_rad * attenuation,
                -self.cfg.max_road_steer_rad,
                +self.cfg.max_road_steer_rad,
            )
        )
        return (self.road_steer_rad, self.wheel_angle_rad)
