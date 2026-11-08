"""
f1sim/ui/__init__.py
====================
Public exports for F1 Motorsport UI menus (including Graphics Settings and
Keybinding Remapping), Broadcast TV HUD, Post-Race Podium Results, and Theme Tokens.
"""
from f1sim.ui.hud import (
    BroadcastHUD,
    MinimapDot,
    TimingTowerRow,
    format_lap_timer,
    format_speed_hud,
    get_timing_tower_positions,
)
from f1sim.ui.menu import (
    DEFAULT_CONTROLS_GUIDE,
    MENU_STATES,
    MotorsportMenuManager,
    RaceSessionConfig,
    SELECTABLE_DIFFICULTIES,
    SELECTABLE_TRACKS,
    SELECTABLE_TRANSMISSIONS,
)
from f1sim.ui.results import (
    ClassifiedDriverResult,
    RaceResultsBuilder,
    VALID_FINISH_STATUSES,
    compute_telemetry_summary,
    find_fastest_lap_driver,
    format_classification_gap,
)
from f1sim.ui.theme import (
    COLOR_CARBON_BG,
    COLOR_DRS_CYAN,
    COLOR_FASTEST_PURPLE,
    COLOR_FIA_RED,
    COLOR_PANEL_SURFACE,
    COLOR_SECTOR_YELLOW,
    COLOR_TELEMETRY_GREEN,
    COLOR_TEXT_MUTED,
    COLOR_TEXT_PRIMARY,
    TEAM_HEX_COLORS,
    hex_to_rgb,
)

__all__ = [
    "BroadcastHUD",
    "COLOR_CARBON_BG",
    "COLOR_DRS_CYAN",
    "COLOR_FASTEST_PURPLE",
    "COLOR_FIA_RED",
    "COLOR_PANEL_SURFACE",
    "COLOR_SECTOR_YELLOW",
    "COLOR_TELEMETRY_GREEN",
    "COLOR_TEXT_MUTED",
    "COLOR_TEXT_PRIMARY",
    "ClassifiedDriverResult",
    "DEFAULT_CONTROLS_GUIDE",
    "MENU_STATES",
    "MinimapDot",
    "MotorsportMenuManager",
    "RaceResultsBuilder",
    "RaceSessionConfig",
    "SELECTABLE_DIFFICULTIES",
    "SELECTABLE_TRACKS",
    "SELECTABLE_TRANSMISSIONS",
    "TEAM_HEX_COLORS",
    "TimingTowerRow",
    "VALID_FINISH_STATUSES",
    "compute_telemetry_summary",
    "find_fastest_lap_driver",
    "format_classification_gap",
    "format_lap_timer",
    "format_speed_hud",
    "get_timing_tower_positions",
    "hex_to_rgb",
]
