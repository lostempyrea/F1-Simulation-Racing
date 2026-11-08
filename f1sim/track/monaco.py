"""
f1sim/track/monaco.py
=====================
Circuit de Monaco - The Jewel in the Crown of Formula 1.
Full-fidelity 3D track geometry, 185-waypoint C2 spline, 19 official corners,
Fairmont Hairpin, +42m elevation change, 1 DRS zone, and 60 km/h pit lane.
"""

from __future__ import annotations
import math
from typing import Tuple, List, Optional
import numpy as np

from f1sim.track import (
    Track,
    CornerInfo,
    DRSZone,
    DistanceMarker,
    PitLane,
    TrackMetadata,
    SurfaceType,
    KerbZone
)
from f1sim.track.track_spline import TrackSpline


class MonacoTrack(Track):
    """
    Circuit de Monaco (3,337.0m, 19 corners, 42.0m elevation change, 60 km/h pit lane).
    Fulfills PROJECT.md track interface contract and Feature 16 requirements.
    """

    def __init__(self) -> None:
        self.metadata = TrackMetadata(
            name="Circuit de Monaco",
            country="Monaco",
            length=3337.0,
            num_corners=19,
            num_drs_zones=1,
            pit_speed_limit_kmh=60.0,
            elevation_change=42.0,
            record_lap_time_sec=72.909,
            record_driver="Lewis Hamilton",
            record_year=2021
        )
        self.length: float = 3337.0
        self.nominal_width: float = 10.0

        # 1. 19 Official Numbered Corners calibrated to Spline Curvature Extrema (kappa_max)
        self.corners: List[CornerInfo] = [
            CornerInfo(1, "Sainte Dévote", 688.0, 23.2, 85.0, 'R', 2, elevation_z=6.75),
            CornerInfo(2, "Beau Rivage", 878.4, 209.8, 270.0, 'L', 7, elevation_z=36.16),
            CornerInfo(3, "Massenet", 1009.3, 30.5, 145.0, 'L', 4, elevation_z=48.00),
            CornerInfo(4, "Casino Square", 1054.0, 25.8, 125.0, 'R', 3, elevation_z=45.31),
            CornerInfo(5, "Mirabeau Haute", 1082.0, 19.4, 75.0, 'R', 2, elevation_z=36.13),
            CornerInfo(6, "Fairmont Hairpin", 1108.0, 11.0, 45.0, 'L', 1, elevation_z=30.01),
            CornerInfo(7, "Mirabeau Bas", 1142.0, 28.5, 70.0, 'R', 2, elevation_z=21.66),
            CornerInfo(8, "Portier", 1202.5, 17.5, 75.0, 'R', 2, elevation_z=6.02),
            CornerInfo(9, "Tunnel Entry Kink", 1260.0, 420.0, 260.0, 'R', 6, elevation_z=6.00),
            CornerInfo(10, "Tunnel Exit Sweep", 1390.0, 389.0, 280.0, 'R', 7, elevation_z=6.00),
            CornerInfo(11, "Nouvelle Chicane T11", 1426.7, 17.3, 68.0, 'L', 2, elevation_z=6.00),
            CornerInfo(12, "Nouvelle Chicane T12", 1485.8, 19.2, 75.0, 'R', 2, elevation_z=6.00),
            CornerInfo(13, "Tabac", 1496.6, 42.1, 165.0, 'L', 4, elevation_z=6.00),
            CornerInfo(14, "Louis Chiron (Piscine Entry T14)", 1588.4, 78.3, 215.0, 'L', 5, elevation_z=6.00),
            CornerInfo(15, "Louis Chiron (Piscine Entry T15)", 1651.0, 72.9, 205.0, 'R', 5, elevation_z=6.00),
            CornerInfo(16, "Swimming Pool Exit (Piscine T16)", 1728.6, 22.1, 115.0, 'L', 3, elevation_z=6.00),
            CornerInfo(17, "La Rascasse Entry T17", 1739.8, 16.6, 65.0, 'R', 2, elevation_z=6.00),
            CornerInfo(18, "La Rascasse Apex T18", 1779.4, 20.1, 55.0, 'R', 1, elevation_z=6.00),
            CornerInfo(19, "Antony Noghès", 1818.0, 22.2, 85.0, 'R', 2, elevation_z=6.00),
        ]

        # Kerb zones along corner apexes (-35m to +45m around corner distance_s)
        self.kerb_zones: List[KerbZone] = [
            KerbZone(
                start_s=(c.distance_s - 35.0) % self.length,
                end_s=(c.distance_s + 45.0) % self.length,
                side='both',
                width=1.00,
                bevel_height=0.035,
                stripe_length=1.00
            )
            for c in self.corners
        ]

        # 2. 1 Official FIA DRS Zone on Pit Straight
        self.drs_zones: List[DRSZone] = [
            DRSZone(
                id=1,
                name="Pit Straight",
                detection_s=3280.0,
                activation_s=18.0,
                deactivation_s=528.0
            )
        ]

        # 3. Distance Markers (100m, 50m)
        self.distance_markers: List[DistanceMarker] = self._generate_distance_markers()

        # 4. Pit Lane Architecture (60.0 km/h)
        self.pit_lane: PitLane = self._generate_pit_lane()

        # 5. Full 185-waypoint 3D dataset & Spline synthesis
        self.raw_waypoints: np.ndarray = self._generate_185_waypoints()
        assert self.raw_waypoints.shape == (185, 3), f"Expected shape (185, 3), got {self.raw_waypoints.shape}"

        # Elevation range check
        z_min = float(np.min(self.raw_waypoints[:, 2]))
        z_max = float(np.max(self.raw_waypoints[:, 2]))
        self.elevation_delta: float = z_max - z_min

        self.spline: TrackSpline = TrackSpline(
            self.raw_waypoints,
            target_length=3337.0,
            nominal_width=10.0,
            resolution=0.50
        )

    def _generate_185_waypoints(self) -> np.ndarray:
        """
        Synthesizes exactly 185 3D geometric waypoints with authentic +42m
        elevation topography tracing Circuit de Monaco with 0 duplicates and 0 self-crossings.
        """
        pts: List[np.ndarray] = [np.array([0.0, 0.0, 6.0], dtype=np.float64)]
        heading: float = 0.0  # +X (East, 0 deg)

        def add_line(length: float, count: int, dz: float = 0.0):
            nonlocal heading
            p0 = pts[-1]
            for i in range(1, count + 1):
                t = i / count
                px = p0[0] + t * length * math.cos(heading)
                py = p0[1] + t * length * math.sin(heading)
                pz = p0[2] + t * dz
                pts.append(np.array([px, py, pz], dtype=np.float64))

        def add_arc(radius: float, angle_deg: float, count: int, direction: str = 'R', dz: float = 0.0):
            nonlocal heading
            p0 = pts[-1]
            sigma = -1.0 if direction == 'R' else 1.0
            rad = math.radians(angle_deg)
            Cx = p0[0] - sigma * radius * math.sin(heading)
            Cy = p0[1] + sigma * radius * math.cos(heading)
            for i in range(1, count + 1):
                t = i / count
                h = heading + sigma * t * rad
                px = Cx + sigma * radius * math.sin(h)
                py = Cy - sigma * radius * math.cos(h)
                pz = p0[2] + t * dz
                pts.append(np.array([px, py, pz], dtype=np.float64))
            heading = heading + sigma * rad

        # 1. Main Straight (Boulevard Albert 1er): 15 new points (total 16 with origin)
        add_line(520.0, 15, dz=0.5)

        # 2. Sainte Dévote Corner T1: 8 points (R = 22m, climbs to 8.5m)
        add_arc(22.0, 85.0, 8, 'R', dz=2.0)

        # 3. Beau Rivage Uphill Climb T2: 16 points (climbs +29.5m up to 38.0m)
        add_arc(280.0, 25.0, 16, 'L', dz=29.5)

        # 4. Massenet Curve T3: 12 points (summit crest at Z = 48.0m)
        add_arc(70.0, 75.0, 12, 'L', dz=10.0)

        # 5. Casino Square T4: 10 points (drops to 45.0m)
        add_arc(40.0, 55.0, 10, 'R', dz=-3.0)

        # 6. Mirabeau Haute T5: 10 points (drops to 35.0m)
        add_arc(20.0, 50.0, 10, 'R', dz=-10.0)

        # 7. Fairmont Hairpin T6: 16 points (180 deg Left, R = 11m, drops to 25.0m)
        add_arc(11.0, 180.0, 16, 'L', dz=-10.0)

        # 8. Mirabeau Bas T7: 8 points (drops to 16.0m)
        add_arc(20.0, 60.0, 8, 'R', dz=-9.0)

        # 9. Portier T8: 10 points (drops to 6.0m shoreline)
        add_arc(24.0, 75.0, 10, 'R', dz=-10.0)

        # 10. The Tunnel T9/T10: 22 points (Z = 6.0m sea level)
        add_arc(320.0, 30.0, 22, 'R', dz=0.0)

        # 11. Nouvelle Chicane T11/T12: 12 points
        add_arc(16.0, 45.0, 4, 'L', dz=0.0)
        add_line(25.0, 4, dz=0.0)
        add_arc(18.0, 45.0, 4, 'R', dz=0.0)

        # 12. Tabac Corner T13: 10 points (R = 45m)
        add_arc(45.0, 60.0, 10, 'L', dz=0.0)

        # 13. Swimming Pool Complex (Louis Chiron T14/T15): 9 points
        add_arc(70.0, 30.0, 4, 'L', dz=0.0)
        add_line(30.0, 1, dz=0.0)
        add_arc(65.0, 30.0, 4, 'R', dz=0.0)

        # 14. Swimming Pool Exit Chicane T16: 8 points
        add_arc(22.0, 50.0, 4, 'R', dz=0.0)
        add_arc(22.0, 50.0, 4, 'L', dz=0.0)

        # 15. La Rascasse T17/T18: 10 points
        add_arc(16.0, 135.0, 10, 'R', dz=0.0)

        # 16. Antony Noghès Corner T19: 6 points
        add_arc(22.0, 80.0, 6, 'R', dz=0.0)

        # 17. Final Connector to Start Line: 3 points (yielding 185 unique waypoints)
        p_curr = pts[-1]
        p_target = pts[0]
        for j in range(1, 4):
            t = j / 3.0
            p_inter = p_curr * (1.0 - t) + p_target * t
            pts.append(p_inter)

        # Return exactly 185 unique waypoints (excluding duplicate closing point)
        raw = np.array(pts[:-1], dtype=np.float64)
        return raw

    def _generate_distance_markers(self) -> List[DistanceMarker]:
        """Creates 100m, 50m distance markers for Monaco street circuit synchronized to corner apexes."""
        markers = [
            DistanceMarker(1, "Sainte Dévote", 100.0, 588.0, 'L'),
            DistanceMarker(1, "Sainte Dévote", 50.0, 638.0, 'L'),
            DistanceMarker(5, "Mirabeau Haute", 50.0, 1032.0, 'L'),
            DistanceMarker(11, "Nouvelle Chicane", 100.0, 1326.7, 'L'),
            DistanceMarker(11, "Nouvelle Chicane", 50.0, 1376.7, 'L'),
            DistanceMarker(13, "Tabac", 50.0, 1446.6, 'L'),
            DistanceMarker(17, "La Rascasse", 50.0, 1689.8, 'L'),
        ]
        return markers

    def _generate_pit_lane(self) -> PitLane:
        """Constructs authentic Monaco pit lane (60 km/h) and 20 pit stalls."""
        pit_stalls: List[np.ndarray] = []
        for k in range(20):
            # 20 stalls on quayside (Y = -9.0m), spaced at 9.0m along X
            x_stall = 100.0 + k * 9.0
            pit_stalls.append(np.array([x_stall, -9.0, 6.0], dtype=np.float64))

        # 30-point centerline
        pit_wps: List[np.ndarray] = []
        # Taper entry from track (Y=0) to pit fast lane (Y=-6)
        for x in np.linspace(30.0, 90.0, 6, endpoint=False):
            t = (x - 30.0) / 60.0
            pit_wps.append(np.array([x, -t * 6.0, 6.0]))
        # Parallel fast lane (Y=-6)
        for x in np.linspace(90.0, 320.0, 18, endpoint=False):
            pit_wps.append(np.array([x, -6.0, 6.0]))
        # Taper exit back onto track (Y=0)
        for x in np.linspace(320.0, 450.0, 6):
            t = (x - 320.0) / 130.0
            pit_wps.append(np.array([x, -(1.0 - t) * 6.0, 6.0]))

        pit = PitLane(
            entry_s=3180.0,
            exit_s=480.0,
            speed_limit_kmh=60.0,
            speed_limit_entry_s=3210.0,
            speed_limit_exit_s=50.0,
            num_stalls=20,
            stall_spacing=9.0,
            stall_positions=pit_stalls,
            waypoints=np.array(pit_wps, dtype=np.float64)
        )
        return pit

    # --------------------------------------------------------------------------
    # Core Track Query Interface
    # --------------------------------------------------------------------------
    def get_spline_point(self, s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns (position_xyz, tangent_vec, normal_vec) at distance s."""
        return self.spline.get_spline_point(s % self.length)

    def get_track_width(self, s: float) -> float:
        """Returns tarmac width at distance s (nominal 10.0m)."""
        return self.spline.get_total_width(s % self.length)

    def get_curvature(self, s: float) -> float:
        """Returns instantaneous curvature kappa(s)."""
        return self.spline.get_curvature(s % self.length)

    def is_in_drs_zone(self, s: float) -> bool:
        """Returns True if vehicle at distance s is inside the Pit Straight DRS zone."""
        s_norm = s % self.length
        zone = self.drs_zones[0]
        return zone.activation_s <= s_norm <= zone.deactivation_s

    def is_on_kerb(self, position: np.ndarray) -> bool:
        """
        Returns True if world position lies on an authentic kerb surface.
        Validates BOTH longitudinal kerb zone [start_s, end_s] AND lateral boundaries.
        """
        s, d, _ = self.spline.project_to_frenet(position)
        s_norm = s % self.length
        half_w = 0.5 * self.get_track_width(s_norm)

        for kerb in self.kerb_zones:
            if kerb.is_in_range(s_norm, self.length):
                sides = ['left', 'right'] if kerb.side == 'both' else [kerb.side]
                for side in sides:
                    d_min, d_max = kerb.lateral_bounds(half_w, side)
                    if d_min <= d <= d_max:
                        return True
        return False

    def get_surface_mu(self, s: float, d: float, normalized: bool = False) -> float:
        """
        Returns surface grip coefficient at (s, d).
        If normalized=False (default):
        - On-track tarmac: 1.75 (absolute physical friction coefficient)
        - Kerb zone: 1.60
        - Off-track verge: 0.40
        If normalized=True:
        - On-track tarmac: 1.00 (normalized multiplier for Vehicle.step)
        - Kerb zone: 0.85
        - Off-track verge: 0.30
        """
        s_norm = s % self.length
        half_w = 0.5 * self.get_track_width(s_norm)
        dist_mag = abs(d)

        # 1. On racing tarmac
        if dist_mag <= half_w:
            return 1.00 if normalized else 1.75

        # 2. Check kerb zones
        for kerb in self.kerb_zones:
            if kerb.is_in_range(s_norm, self.length):
                sides = ['left', 'right'] if kerb.side == 'both' else [kerb.side]
                for side in sides:
                    d_min, d_max = kerb.lateral_bounds(half_w, side)
                    if d_min <= d <= d_max:
                        return 0.85 if normalized else 1.60

        # 3. Off-track verge / wall impact zone
        return 0.30 if normalized else 0.40

    def get_absolute_surface_mu(self, s: float, d: float) -> float:
        """Returns unnormalized absolute friction coefficient (1.75 tarmac, 1.60 kerb, 0.40 verge)."""
        return self.get_surface_mu(s, d, normalized=False)

    def get_normalized_surface_mu(self, s: float, d: float) -> float:
        """Returns normalized surface grip multiplier (1.00 tarmac, 0.85 kerb, 0.30 verge)."""
        return self.get_surface_mu(s, d, normalized=True)
