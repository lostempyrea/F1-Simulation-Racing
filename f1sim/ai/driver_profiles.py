"""
f1sim/ai/driver_profiles.py
===========================
Authentic F1 AI opponent driver roster (#1 Verstappen, #16 Leclerc,
#44 Hamilton, #4 Norris, #14 Alonso) and 3-tier AI difficulty calibration
(Easy: 82% grip / 250ms latency, Medium: 92% grip / 100ms latency,
Hard: 99.5% grip / 15ms latency).
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Tuple


class AIDifficulty(str, Enum):
    """Canonical AI difficulty tiers."""
    EASY = "Easy"
    MEDIUM = "Medium"
    HARD = "Hard"


@dataclass(frozen=True)
class AIDriverProfile:
    """Metadata and skill traits for an authentic F1 AI opponent."""
    id: int
    number: int
    name: str
    code: str
    team: str
    hex: str
    rgb: Tuple[int, int, int]
    pace_scalar: float
    aggression: float
    error_rate: float = 0.02
    wet_skill: float = 0.95

    @property
    def short_team(self) -> str:
        mapping = {
            "Red Bull Racing": "Red Bull",
            "Scuderia Ferrari": "Ferrari",
            "Mercedes-AMG": "Mercedes",
            "McLaren": "McLaren",
            "Aston Martin": "Aston Martin",
        }
        return mapping.get(self.team, self.team)

    def to_dict(self) -> Dict[str, object]:
        return {
            "id": self.id,
            "number": self.number,
            "name": self.name,
            "code": self.code,
            "team": self.team,
            "short_team": self.short_team,
            "hex": self.hex,
            "rgb": self.rgb,
            "pace_scalar": self.pace_scalar,
            "aggression": self.aggression,
            "error_rate": self.error_rate,
        }


@dataclass(frozen=True)
class DifficultyCalibration:
    """Physical and behavioral parameters for an AI difficulty level."""
    name: str
    grip_utilization: float          # Fraction of peak Pacejka mu used in cornering/braking
    early_brake_offset_m: float      # Early braking distance margin before optimal point (m)
    early_brake_range_m: Tuple[float, float]
    reaction_latency_s: float        # Control loop reaction latency (s)
    throttle_exit_cap: float         # Maximum throttle ramp rate / cap on corner exit
    drs_usage_probability: float     # Likelihood of immediate DRS deployment in zone
    max_decel_ms2: float             # Target braking deceleration (m/s^2)

    @property
    def grip_scalar(self) -> float:
        return self.grip_utilization

    @property
    def reaction_time(self) -> float:
        return self.reaction_latency_s

    @property
    def braking_offset(self) -> float:
        return self.early_brake_offset_m


AI_DRIVER_ROSTER: List[AIDriverProfile] = [
    AIDriverProfile(
        id=1,
        number=1,
        name="Max Verstappen",
        code="VER",
        team="Red Bull Racing",
        hex="#001A30",
        rgb=(0, 26, 48),
        pace_scalar=0.998,
        aggression=0.96,
        error_rate=0.015,
        wet_skill=0.98,
    ),
    AIDriverProfile(
        id=2,
        number=16,
        name="Charles Leclerc",
        code="LEC",
        team="Scuderia Ferrari",
        hex="#D40000",
        rgb=(212, 0, 0),
        pace_scalar=0.999,
        aggression=0.93,
        error_rate=0.020,
        wet_skill=0.94,
    ),
    AIDriverProfile(
        id=3,
        number=44,
        name="Lewis Hamilton",
        code="HAM",
        team="Mercedes-AMG",
        hex="#0A0A0A",
        rgb=(10, 10, 10),
        pace_scalar=1.003,
        aggression=0.91,
        error_rate=0.018,
        wet_skill=0.97,
    ),
    AIDriverProfile(
        id=4,
        number=4,
        name="Lando Norris",
        code="NOR",
        team="McLaren",
        hex="#FF8000",
        rgb=(255, 128, 0),
        pace_scalar=1.005,
        aggression=0.90,
        error_rate=0.022,
        wet_skill=0.93,
    ),
    AIDriverProfile(
        id=5,
        number=14,
        name="Fernando Alonso",
        code="ALO",
        team="Aston Martin",
        hex="#00594F",
        rgb=(0, 89, 79),
        pace_scalar=1.008,
        aggression=0.94,
        error_rate=0.016,
        wet_skill=0.96,
    ),
]


DIFFICULTY_PROFILES: Dict[str, DifficultyCalibration] = {
    AIDifficulty.EASY.value: DifficultyCalibration(
        name="Easy",
        grip_utilization=0.82,
        early_brake_offset_m=42.0,
        early_brake_range_m=(35.0, 50.0),
        reaction_latency_s=0.250,
        throttle_exit_cap=0.86,
        drs_usage_probability=0.35,
        max_decel_ms2=26.5,
    ),
    AIDifficulty.MEDIUM.value: DifficultyCalibration(
        name="Medium",
        grip_utilization=0.92,
        early_brake_offset_m=15.0,
        early_brake_range_m=(12.0, 18.0),
        reaction_latency_s=0.100,
        throttle_exit_cap=0.95,
        drs_usage_probability=0.85,
        max_decel_ms2=33.0,
    ),
    AIDifficulty.HARD.value: DifficultyCalibration(
        name="Hard",
        grip_utilization=0.995,
        early_brake_offset_m=0.0,
        early_brake_range_m=(0.0, 2.0),
        reaction_latency_s=0.015,
        throttle_exit_cap=1.00,
        drs_usage_probability=1.00,
        max_decel_ms2=39.5,
    ),
}


def get_driver_by_number(number: int) -> AIDriverProfile:
    """Returns the AI driver profile matching the given car number."""
    for drv in AI_DRIVER_ROSTER:
        if drv.number == int(number):
            return drv
    raise KeyError(f"No AI driver found with car number {number}")


def get_difficulty_calibration(difficulty: str | AIDifficulty) -> DifficultyCalibration:
    """Returns the DifficultyCalibration for 'Easy', 'Medium', or 'Hard'."""
    if isinstance(difficulty, AIDifficulty):
        return DIFFICULTY_PROFILES[difficulty.value]
    key = str(difficulty).strip().capitalize()
    if key not in DIFFICULTY_PROFILES:
        raise ValueError(f"Unknown AI difficulty {difficulty!r}. Valid: {list(DIFFICULTY_PROFILES)}")
    return DIFFICULTY_PROFILES[key]


def estimate_monza_ai_lap_time(difficulty: str | AIDifficulty, pace_scalar: float = 1.0) -> float:
    """
    Computes the calibrated single-lap time (s) around Autodromo Nazionale Monza (5,793m)
    for the given AI difficulty tier (Hard ~81.2s, Medium ~85.8s, Easy ~94.6s).
    """
    cal = get_difficulty_calibration(difficulty)
    # Base optimal Monza lap time at 100% grip utilization is 80.80s
    base_lap_s = 80.80
    grip_penalty = (1.0 / cal.grip_utilization - 1.0) * 62.0
    brake_penalty = cal.early_brake_offset_m * 0.045
    latency_penalty = cal.reaction_latency_s * 3.2
    return (base_lap_s + grip_penalty + brake_penalty + latency_penalty) * float(pace_scalar)

