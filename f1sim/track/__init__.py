"""
f1sim/track
===========
Official Formula 1 Track Geometry, Mathematical Splines, Procedural Meshes,
and Circuit Metadata for Desktop Native Simulation.

Modules:
- track_spline: C2-continuous periodic cubic spline, Bishop frame, curvature, projection
- monza: Autodromo Nazionale Monza (5,793m, 11 corners, 2 DRS zones, 80 km/h pit lane)
- monaco: Circuit de Monaco (3,337m, 19 corners, 42m elevation change, 60 km/h pit lane)
- track_builder: 3D ribbon mesh, beveled kerbs, surface friction zones, 5-light gantry
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Tuple, List, Optional, Sequence
import numpy as np


class SurfaceType(IntEnum):
    """Surface classification matching ModernGL shader inputs."""
    TARMAC = 0
    KERB_RED = 1
    KERB_WHITE = 2
    RUNOFF_ASPHALT = 3
    GRAVEL_TRAP = 4
    GRASS_VERGE = 5
    START_GRID_LINE = 6
    TRACK_EDGE_LINE = 7
    BARRIER_ARMCO = 8


@dataclass(frozen=True)
class CornerInfo:
    """Official F1 corner metadata."""
    number: int
    name: str
    distance_s: float
    radius: float
    apex_speed_kmh: float
    direction: str  # 'L' or 'R'
    gear: int
    elevation_z: float = 0.0
    has_kerb_inside: bool = True
    has_kerb_outside: bool = True


@dataclass(frozen=True)
class DRSZone:
    """FIA DRS detection, activation and deactivation zone."""
    id: int
    name: str
    detection_s: float
    activation_s: float
    deactivation_s: float

    @property
    def length(self) -> float:
        return self.deactivation_s - self.activation_s


@dataclass(frozen=True)
class DistanceMarker:
    """Braking distance indicator board approaching corner."""
    corner_number: int
    corner_name: str
    distance_to_corner: float  # e.g. 150.0, 100.0, 50.0 meters
    s_position: float
    side: str = 'L'
    position_xyz: Optional[np.ndarray] = None


@dataclass
class PitLane:
    """Pit lane geometry, speed limit and stall locations."""
    entry_s: float
    exit_s: float
    speed_limit_kmh: float
    speed_limit_entry_s: float
    speed_limit_exit_s: float
    num_stalls: int = 20
    stall_spacing: float = 12.0
    stall_positions: List[np.ndarray] = field(default_factory=list)
    waypoints: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))


@dataclass(frozen=True)
class TrackMetadata:
    """Circuit metadata and FIA homologation statistics."""
    name: str
    country: str
    length: float
    num_corners: int
    num_drs_zones: int
    pit_speed_limit_kmh: float
    elevation_change: float
    record_lap_time_sec: float
    record_driver: str
    record_year: int


@dataclass(frozen=True)
class SurfaceProperties:
    """Physical interaction properties returned by track surface queries."""
    surface_type: SurfaceType
    friction_mu: float
    rolling_resistance_mult: float
    is_kerb: bool
    vibration_amplitude: float
    vibration_frequency_hz: float
    elevation_step: float


@dataclass
class KerbZone:
    """Specification of a kerb zone alongside the track."""
    start_s: float
    end_s: float
    side: str = 'both'  # 'left', 'right', or 'both'
    width: float = 1.20
    bevel_height: float = 0.035
    stripe_length: float = 1.00
    tooth_depth: float = 0.012

    def is_in_range(self, s: float, track_length: float = 6000.0) -> bool:
        if self.start_s <= self.end_s:
            return self.start_s <= s <= self.end_s
        return s >= self.start_s or s <= self.end_s

    def lateral_bounds(self, w_half: float, side: str) -> Tuple[float, float]:
        if side == 'left':
            return -(w_half + self.width), -w_half
        return w_half, w_half + self.width


@dataclass
class RunoffZone:
    """Specification of an asphalt runoff or gravel trap zone."""
    start_s: float
    end_s: float
    side: str = 'both'
    surface_type: SurfaceType = SurfaceType.RUNOFF_ASPHALT
    width: float = 8.0

    def is_in_range(self, s: float) -> bool:
        if self.start_s <= self.end_s:
            return self.start_s <= s <= self.end_s
        return s >= self.start_s or s <= self.end_s


@dataclass
class GridBox:
    """Starting grid box position."""
    position_index: int
    s: float
    d: float
    world_pos: np.ndarray
    yaw: float
    width: float = 1.80
    length: float = 4.20


@dataclass
class DistanceMarkerBoard:
    """3D Distance marker board approaching corner."""
    distance_m: float
    s: float
    side: str
    world_pos: np.ndarray
    normal_dir: np.ndarray


@dataclass
class MeshBufferData:
    """Interleaved vertex buffer data formatted for ModernGL 3.3 Core Profile."""
    vertices: np.ndarray
    indices: np.ndarray
    vertex_format: str = "3f 3f 2f 1f 3f"
    attribute_names: List[str] = field(default_factory=lambda: [
        "in_position", "in_normal", "in_uv", "in_surface_type", "in_tangent"
    ])

    @property
    def total_vertices(self) -> int:
        return self.vertices.shape[0]

    @property
    def total_triangles(self) -> int:
        return self.indices.shape[0] // 3

    def to_bytes(self) -> Tuple[bytes, bytes]:
        return self.vertices.astype(np.float32).tobytes(), self.indices.astype(np.uint32).tobytes()


class Track:
    """
    Abstract / base contract for official Formula 1 circuits.
    Fulfills PROJECT.md track interface contract.
    """
    length: float
    nominal_width: float

    def get_spline_point(self, s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (position_xyz, tangent_vec, normal_vec) at track distance s."""
        raise NotImplementedError

    def get_track_width(self, s: float) -> float:
        """Returns tarmac width at track distance s."""
        raise NotImplementedError

    def get_curvature(self, s: float) -> float:
        """Returns instantaneous track curvature kappa(s)."""
        raise NotImplementedError

    def is_in_drs_zone(self, s: float) -> bool:
        """Returns True if vehicle at distance s is inside an active DRS zone."""
        raise NotImplementedError

    def is_on_kerb(self, position: np.ndarray) -> bool:
        """Returns True if world position lies on a kerb surface."""
        raise NotImplementedError

    def get_surface_mu(self, s: float, d: float) -> float:
        """Returns baseline physical friction coefficient at (s, d)."""
        raise NotImplementedError


# Lazy imports for clean API access
def __getattr__(name: str):
    if name == "TrackSpline":
        from f1sim.track.track_spline import TrackSpline
        return TrackSpline
    elif name == "MonzaTrack":
        from f1sim.track.monza import MonzaTrack
        return MonzaTrack
    elif name == "MonacoTrack":
        from f1sim.track.monaco import MonacoTrack
        return MonacoTrack
    elif name == "TrackBuilder":
        from f1sim.track.track_builder import TrackBuilder
        return TrackBuilder
    elif name == "StartingGantry":
        from f1sim.track.track_builder import StartingGantry
        return StartingGantry
    raise AttributeError(f"module 'f1sim.track' has no attribute '{name}'")


__all__ = [
    "SurfaceType",
    "CornerInfo",
    "DRSZone",
    "DistanceMarker",
    "PitLane",
    "TrackMetadata",
    "SurfaceProperties",
    "KerbZone",
    "RunoffZone",
    "GridBox",
    "DistanceMarkerBoard",
    "MeshBufferData",
    "Track",
    "TrackSpline",
    "MonzaTrack",
    "MonacoTrack",
    "TrackBuilder",
    "StartingGantry",
]
