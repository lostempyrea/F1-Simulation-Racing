"""
f1sim/ai/__init__.py
====================
Public exports for F1 AI driver profiles, difficulty calibration, racing line
curvature profiling, pure-pursuit controller, and OBB/TTC collision avoidance.
"""
from f1sim.ai.collision import (
    OBB2D,
    OVERTAKE_LATERAL_OFFSET_M,
    TTC_AVOIDANCE_THRESHOLD_S,
    VEHICLE_OBB_LENGTH_M,
    VEHICLE_OBB_WIDTH_M,
    check_obb_intersection,
    compute_overtake_and_avoidance,
    compute_slipstream_drag_multiplier,
    compute_ttc,
)
from f1sim.ai.controller import (
    AICarControlOutput,
    AIDriverController,
    estimate_monza_ai_lap_time,
)
from f1sim.ai.driver_profiles import (
    AI_DRIVER_ROSTER,
    AIDifficulty,
    AIDriverProfile,
    DIFFICULTY_PROFILES,
    DifficultyCalibration,
    get_difficulty_calibration,
    get_driver_by_number,
)
from f1sim.ai.racing_line import (
    RacingLineProfile,
    compute_apex_speed,
    compute_braking_entry_speed,
    compute_lookahead_distance,
)

__all__ = [
    "AICarControlOutput",
    "AIDifficulty",
    "AIDriverController",
    "AIDriverProfile",
    "AI_DRIVER_ROSTER",
    "DIFFICULTY_PROFILES",
    "DifficultyCalibration",
    "OBB2D",
    "OVERTAKE_LATERAL_OFFSET_M",
    "RacingLineProfile",
    "TTC_AVOIDANCE_THRESHOLD_S",
    "VEHICLE_OBB_LENGTH_M",
    "VEHICLE_OBB_WIDTH_M",
    "check_obb_intersection",
    "compute_apex_speed",
    "compute_braking_entry_speed",
    "compute_lookahead_distance",
    "compute_overtake_and_avoidance",
    "compute_slipstream_drag_multiplier",
    "compute_ttc",
    "estimate_monza_ai_lap_time",
    "get_difficulty_calibration",
    "get_driver_by_number",
]
