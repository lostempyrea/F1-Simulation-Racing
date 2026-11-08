"""
f1sim/ui/hud.py
===============
F1 Broadcast Television HUD Overlay:
1. Speedometer ("285 KM/H") & RPM graphical telemetry bar
2. Live 6-Car Timing Tower (P1 through P6, intervals, driver codes, team colors)
3. Lap Timer ("1:21.234") & Sector Delta badge
4. 2D Circuit Mini-Map Radar with player and AI car position dots
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


def format_speed_hud(speed_kmh: float) -> str:
    """Formats speed in rounded integer km/h (e.g. 284.7 -> '285 KM/H')."""
    return f"{int(round(float(speed_kmh)))} KM/H"


def format_lap_timer(lap_time_sec: float) -> str:
    """Formats a lap time in seconds as 'M:SS.mmm' (e.g. 81.234 -> '1:21.234')."""
    t = max(0.0, float(lap_time_sec))
    minutes = int(t // 60)
    seconds = t % 60.0
    return f"{minutes}:{seconds:06.3f}"


def get_timing_tower_positions(num_drivers: int = 6) -> List[str]:
    """Returns canonical timing tower position labels ['P1', ..., 'P6']."""
    return [f"P{i}" for i in range(1, int(num_drivers) + 1)]


@dataclass
class TimingTowerRow:
    """Single row in the left-side F1 TV broadcast timing tower."""
    position: int
    position_label: str
    driver_code: str
    driver_name: str
    team: str
    color_hex: str
    gap_str: str
    distance_m: float = 0.0
    is_player: bool = False


@dataclass
class MinimapDot:
    """2D normalized or world position dot for a car on the circuit mini-map."""
    id: int
    code: str
    color: str
    x: float
    y: float
    is_player: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "code": self.code,
            "color": self.color,
            "x": self.x,
            "y": self.y,
            "is_player": self.is_player,
        }


class BroadcastHUD:
    """
    Computes and formats F1 Broadcast TV HUD elements for real-time display.
    """

    def __init__(self, num_cars: int = 6) -> None:
        self.num_cars: int = max(1, int(num_cars))
        self.positions: List[str] = get_timing_tower_positions(self.num_cars)
        self.speed_display: str = "0 KM/H"
        self.rpm_value: float = 4000.0
        self.rpm_fraction: float = 0.0
        self.gear_display: str = "N"
        self.lap_timer_display: str = "0:00.000"
        self.current_lap: int = 1
        self.total_laps: int = 3
        self.timing_tower: List[TimingTowerRow] = []
        self.minimap_dots: List[Dict[str, Any]] = []

    def update(
        self,
        speed_kmh: float,
        rpm: float,
        gear_str: str,
        current_lap_time_sec: float,
        current_lap: int = 1,
        total_laps: int = 3,
        car_standings: Optional[List[Dict[str, Any]]] = None,
        minimap_cars: Optional[List[Dict[str, Any]]] = None,
    ) -> None:
        """Updates all HUD elements for the current frame."""
        self.speed_display = format_speed_hud(speed_kmh)
        self.rpm_value = float(max(0.0, rpm))
        self.rpm_fraction = max(0.0, min(1.0, (self.rpm_value - 4000.0) / 11000.0))
        self.gear_display = str(gear_str)
        self.lap_timer_display = format_lap_timer(current_lap_time_sec)
        self.current_lap = int(current_lap)
        self.total_laps = int(total_laps)

        if car_standings is not None:
            self.timing_tower = self.build_timing_tower(car_standings)
        if minimap_cars is not None:
            self.minimap_dots = self.build_minimap_dots(minimap_cars)

    def build_timing_tower(self, car_entries: List[Dict[str, Any]]) -> List[TimingTowerRow]:
        """
        Sorts active cars by total distance covered (descending) and produces
        P1-P6 `TimingTowerRow` records with live interval gaps.
        """
        sorted_cars = sorted(
            car_entries,
            key=lambda c: float(c.get("distance_m", 0.0)),
            reverse=True,
        )
        rows: List[TimingTowerRow] = []
        leader_dist = float(sorted_cars[0].get("distance_m", 0.0)) if sorted_cars else 0.0

        for idx, c in enumerate(sorted_cars[: self.num_cars]):
            pos = idx + 1
            dist = float(c.get("distance_m", 0.0))
            v_ms = max(25.0, float(c.get("speed_ms", 60.0)))
            if pos == 1:
                gap_str = "LEADER"
            else:
                gap_sec = max(0.0, (leader_dist - dist) / v_ms)
                gap_str = f"+{gap_sec:.3f}s"

            rows.append(
                TimingTowerRow(
                    position=pos,
                    position_label=f"P{pos}",
                    driver_code=str(c.get("code", f"D{pos}")),
                    driver_name=str(c.get("name", f"Driver {pos}")),
                    team=str(c.get("team", "F1")),
                    color_hex=str(c.get("hex", "#E10600")),
                    gap_str=gap_str,
                    distance_m=dist,
                    is_player=bool(c.get("is_player", False)),
                )
            )
        return rows

    @staticmethod
    def build_minimap_dots(car_positions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Builds structured mini-map radar dot entries for the player and AI cars.
        """
        dots: List[Dict[str, Any]] = []
        for idx, c in enumerate(car_positions):
            dots.append({
                "id": int(c.get("id", idx)),
                "code": str(c.get("code", f"C{idx}")),
                "color": str(c.get("color", "PLAYER" if idx == 0 else "AI_RED_BULL")),
                "x": float(c.get("x", 0.0)),
                "y": float(c.get("y", 0.0)),
                "is_player": bool(c.get("is_player", idx == 0)),
            })
        return dots
