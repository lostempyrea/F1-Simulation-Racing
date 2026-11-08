"""
f1sim/ui/results.py
===================
Post-Race Podium Classification, Winner Gap Formatting ('WINNER', '+X.XXXs'),
Fastest Lap Award Detection, and Race Telemetry Summary Statistics.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set


VALID_FINISH_STATUSES: Set[str] = {"FINISHED", "DNF"}


def format_classification_gap(position: int, driver_time_sec: float, winner_time_sec: float, status: str = "FINISHED") -> str:
    """
    Formats race classification gap:
    - P1 -> 'WINNER'
    - DNF -> 'DNF'
    - P2+ -> '+X.XXXs'
    """
    st = str(status).strip().upper()
    if st == "DNF":
        return "DNF"
    if int(position) == 1:
        return "WINNER"
    delta = max(0.0, float(driver_time_sec) - float(winner_time_sec))
    return f"+{delta:.3f}s"


def find_fastest_lap_driver(best_lap_times: Dict[str, float]) -> Optional[str]:
    """Returns the driver code/name with the minimum single-lap time."""
    if not best_lap_times:
        return None
    valid = {k: float(v) for k, v in best_lap_times.items() if float(v) > 0.0}
    if not valid:
        return None
    return min(valid, key=valid.get)


def compute_telemetry_summary(
    speed_samples_kmh: List[float],
    total_distance_m: float = 0.0,
    total_time_sec: float = 0.0,
) -> Dict[str, float]:
    """Computes post-race telemetry summary statistics (top speed & average speed in km/h)."""
    if speed_samples_kmh:
        top_speed = float(max(speed_samples_kmh))
        avg_speed = float(sum(speed_samples_kmh) / len(speed_samples_kmh))
    else:
        top_speed = 0.0
        avg_speed = 0.0

    if total_distance_m > 0.0 and total_time_sec > 0.0:
        avg_from_dist = (float(total_distance_m) / float(total_time_sec)) * 3.6
        if avg_speed <= 0.0:
            avg_speed = avg_from_dist
        top_speed = max(top_speed, avg_speed * 1.15)

    return {
        "top_speed_kmh": float(top_speed),
        "avg_speed_kmh": float(avg_speed),
    }


@dataclass
class ClassifiedDriverResult:
    """Single classified row in the post-race podium & results screen."""
    pos: int
    name: str
    code: str
    team: str
    time: float
    gap: str
    best_lap: float
    status: str = "FINISHED"
    fastest_lap_award: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "pos": self.pos,
            "name": self.name,
            "code": self.code,
            "team": self.team,
            "time": self.time,
            "gap": self.gap,
            "best_lap": self.best_lap,
            "status": self.status,
            "fastest_lap_award": self.fastest_lap_award,
        }


class RaceResultsBuilder:
    """
    Constructs sorted post-race P1-P6 classifications, gaps, fastest lap
    awards, and telemetry summaries.
    """

    def __init__(self) -> None:
        self.valid_statuses: Set[str] = set(VALID_FINISH_STATUSES)

    def classify(self, entries: List[Dict[str, Any]]) -> List[ClassifiedDriverResult]:
        """
        Sorts race entries in ascending order of total race time (with FINISHED
        drivers ahead of DNF drivers) and assigns P1-P6 positions, gaps, and
        the fastest lap award.
        """
        if not entries:
            return []

        def sort_key(item: Dict[str, Any]):
            st = str(item.get("status", "FINISHED")).upper()
            is_dnf = 1 if st == "DNF" else 0
            return (is_dnf, float(item.get("time", float("inf"))))

        sorted_entries = sorted(entries, key=sort_key)
        winner_time = float(sorted_entries[0].get("time", 0.0))

        lap_map: Dict[str, float] = {}
        for idx, item in enumerate(sorted_entries):
            code = str(item.get("code", item.get("name", f"D{idx + 1}")))
            bl = float(item.get("best_lap", item.get("time", 0.0)))
            if bl > 0.0:
                lap_map[code] = bl

        fastest_code = find_fastest_lap_driver(lap_map)

        results: List[ClassifiedDriverResult] = []
        for idx, item in enumerate(sorted_entries):
            pos = idx + 1
            t_val = float(item.get("time", 0.0))
            st = str(item.get("status", "FINISHED")).upper()
            if st not in self.valid_statuses:
                st = "FINISHED"
            code = str(item.get("code", item.get("name", f"D{pos}")))
            name = str(item.get("name", code))
            team = str(item.get("team", "F1"))
            best_lap = float(item.get("best_lap", t_val))
            gap_str = format_classification_gap(pos, t_val, winner_time, status=st)

            results.append(
                ClassifiedDriverResult(
                    pos=pos,
                    name=name,
                    code=code,
                    team=team,
                    time=t_val,
                    gap=gap_str,
                    best_lap=best_lap,
                    status=st,
                    fastest_lap_award=(code == fastest_code),
                )
            )
        return results
