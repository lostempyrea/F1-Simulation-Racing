"""
f1sim/ui/theme.py
=================
F1 Broadcast Television & Carbon-Fiber Cockpit UI Design Tokens.
"""
from __future__ import annotations
from typing import Dict, Tuple


COLOR_CARBON_BG: str = "#0B0D12"
COLOR_PANEL_SURFACE: str = "#141821"
COLOR_FIA_RED: str = "#E10600"
COLOR_TELEMETRY_GREEN: str = "#00E676"
COLOR_SECTOR_YELLOW: str = "#FFB300"
COLOR_FASTEST_PURPLE: str = "#AA00FF"
COLOR_DRS_CYAN: str = "#00B0FF"
COLOR_TEXT_PRIMARY: str = "#F5F7FA"
COLOR_TEXT_MUTED: str = "#9AA5B4"

TEAM_HEX_COLORS: Dict[str, str] = {
    "Custom": "#E10600",
    "Red Bull": "#001A30",
    "Ferrari": "#D40000",
    "Mercedes": "#0A0A0A",
    "McLaren": "#FF8000",
    "Aston Martin": "#00594F",
}


def hex_to_rgb(hex_color: str) -> Tuple[int, int, int]:
    """Converts a '#RRGGBB' hex string to an (R, G, B) integer tuple."""
    h = str(hex_color).strip().lstrip("#")
    if len(h) != 6:
        return (225, 6, 0)
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))
