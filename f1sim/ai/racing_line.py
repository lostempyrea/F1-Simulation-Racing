"""
f1sim/ai/racing_line.py
=======================
Curvature-based apex speed profiling, backward braking integration pass,
and pure-pursuit lookahead calculation for F1 AI path following.
"""
from __future__ import annotations
import math
from typing import Any, Optional
import numpy as np

from f1sim.ai.driver_profiles import (
    AIDifficulty,
    DifficultyCalibration,
    get_difficulty_calibration,
)
from f1sim.core.constants import GRAVITY


def compute_lookahead_distance(speed_ms: float) -> float:
    """
    Computes the pure-pursuit lookahead distance in meters:
        L_d = clamp(8.0 + 0.38 * v, 8.0, 45.0)
    """
    v = max(0.0, float(speed_ms))
    return max(8.0, min(45.0, 8.0 + 0.38 * v))


def compute_apex_speed(
    curvature: float,
    mu: float = 1.75,
    gravity: float = GRAVITY,
    grip_utilization: float = 1.0,
    max_speed_ms: float = 98.0,
) -> float:
    """
    Computes maximum steady-state cornering speed (m/s) from curvature kappa (1/m):
        v_apex = sqrt((mu * grip_utilization * g) / |kappa|)
    """
    k = abs(float(curvature))
    if k < 1e-5:
        return float(max_speed_ms)
    eff_mu = max(0.1, float(mu) * float(grip_utilization))
    v = math.sqrt((eff_mu * float(gravity)) / k)
    return min(float(max_speed_ms), v)


def compute_braking_entry_speed(v_apex: float, a_brake: float, ds: float) -> float:
    """
    Backward kinematic integration step:
        v_entry = sqrt(v_apex^2 + 2 * a_brake * ds)
    """
    va = max(0.0, float(v_apex))
    ab = max(0.0, float(a_brake))
    dist = max(0.0, float(ds))
    return math.sqrt(va * va + 2.0 * ab * dist)


class RacingLineProfile:
    """
    Precomputes an arc-length speed and lateral offset profile around a track
    using curvature apex limits and backward braking integration.
    """

    def __init__(
        self,
        track: Any,
        difficulty: str | AIDifficulty = AIDifficulty.HARD,
        step_m: float = 5.0,
    ) -> None:
        self.track = track
        self.calibration: DifficultyCalibration = get_difficulty_calibration(difficulty)
        self.length: float = float(getattr(track, "length", 5793.0))
        self.step_m: float = max(1.0, float(step_m))
        self.num_samples: int = max(32, int(round(self.length / self.step_m)))
        self.s_grid: np.ndarray = np.linspace(0.0, self.length, self.num_samples, endpoint=False)
        self.target_speeds_ms: np.ndarray = np.full(self.num_samples, 95.0, dtype=np.float64)
        self.lateral_offsets_m: np.ndarray = np.zeros(self.num_samples, dtype=np.float64)
        self._build_profile()

    def _build_profile(self) -> None:
        n = self.num_samples
        ds = self.length / float(n)
        cal = self.calibration
        top_v = 96.0 * cal.grip_utilization

        # 1. Base apex speed calculated from spline curvature kappa(s)
        v_raw = np.zeros(n, dtype=np.float64)
        for idx, s_val in enumerate(self.s_grid):
            kappa = 0.0
            if hasattr(self.track, "get_curvature"):
                kappa = float(self.track.get_curvature(s_val))
            v_raw[idx] = compute_apex_speed(
                curvature=kappa,
                mu=1.75,
                grip_utilization=cal.grip_utilization,
                max_speed_ms=top_v,
            )

        # 2. Fuse official corner catalog apex speeds & early braking shifts
        corners = getattr(self.track, "corners", [])
        for corner in corners:
            # Defensive attribute lookup: support both distance_s and legacy s_apex
            s_apex = float(getattr(corner, "distance_s", getattr(corner, "s_apex", 0.0))) % self.length
            v_corner_ms = (float(getattr(corner, "apex_speed_kmh", 120.0)) / 3.6) * cal.grip_utilization

            # Shift effective braking target upstream by early_brake_offset_m
            s_brake_target = (s_apex - cal.early_brake_offset_m) % self.length
            for idx, s_val in enumerate(self.s_grid):
                diff_apex = abs((s_val - s_apex + 0.5 * self.length) % self.length - 0.5 * self.length)
                if diff_apex <= 25.0:
                    v_raw[idx] = min(v_raw[idx], v_corner_ms)

                if cal.early_brake_offset_m > 1.0:
                    diff_brake = abs((s_val - s_brake_target + 0.5 * self.length) % self.length - 0.5 * self.length)
                    if diff_brake <= max(10.0, 0.5 * cal.early_brake_offset_m):
                        v_raw[idx] = min(v_raw[idx], v_corner_ms * 1.05)

        # 3. Multi-pass backward braking integration around closed loop
        a_brake = cal.max_decel_ms2
        for _ in range(3):
            for i in range(n - 1, -1, -1):
                nxt = (i + 1) % n
                v_allowed = compute_braking_entry_speed(float(v_raw[nxt]), a_brake, ds)
                if v_allowed < v_raw[i]:
                    v_raw[i] = v_allowed

        # 4. Multi-pass forward acceleration smoothing pass
        a_accel = 13.5 * cal.throttle_exit_cap
        for _ in range(3):
            for i in range(n):
                prv = (i - 1) % n
                v_reach = math.sqrt(float(v_raw[prv]) ** 2 + 2.0 * a_accel * ds)
                if v_reach < v_raw[i]:
                    v_raw[i] = v_reach

        self.target_speeds_ms = v_raw

    def get_target_speed_ms(self, s: float, pace_scalar: float = 1.0) -> float:
        """Interpolates the target racing-line speed (m/s) at track distance s."""
        s_wrapped = float(s) % self.length
        idx_f = (s_wrapped / self.length) * self.num_samples
        i0 = int(idx_f) % self.num_samples
        i1 = (i0 + 1) % self.num_samples
        frac = idx_f - int(idx_f)
        v = (1.0 - frac) * float(self.target_speeds_ms[i0]) + frac * float(self.target_speeds_ms[i1])
        return v / max(0.5, float(pace_scalar))
