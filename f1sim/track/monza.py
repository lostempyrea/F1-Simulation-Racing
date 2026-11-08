"""
f1sim/track/monza.py
====================
Autodromo Nazionale Monza - The Temple of Speed.
Full-fidelity track geometry, 157-waypoint C2 spline, 11 official corners,
2 DRS zones, 150m/100m/50m distance markers, and 80 km/h pit lane.
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


class MonzaTrack(Track):
    """
    Autodromo Nazionale Monza (5,793.0m, 11 corners, 2 DRS zones, 80 km/h pit lane).
    Fulfills PROJECT.md track interface contract and Feature 15 requirements.
    """

    def __init__(self) -> None:
        self.metadata = TrackMetadata(
            name="Autodromo Nazionale Monza",
            country="Italy",
            length=5793.0,
            num_corners=11,
            num_drs_zones=2,
            pit_speed_limit_kmh=80.0,
            elevation_change=8.5,
            record_lap_time_sec=81.046,
            record_driver="Rubens Barrichello",
            record_year=2004
        )
        self.length: float = 5793.0
        self.nominal_width: float = 11.0

        # 1. 11 Official Numbered Corners calibrated to Spline Curvature Extrema (kappa_max)
        self.corners: List[CornerInfo] = [
            CornerInfo(1, "Variante del Rettifilo T1", 1090.3, 19.9, 75.0, 'R', 1, elevation_z=0.13),
            CornerInfo(2, "Variante del Rettifilo T2", 1131.9, 22.7, 80.0, 'L', 2, elevation_z=0.28),
            CornerInfo(3, "Curva Grande (Biassono)", 1245.5, 320.0, 295.0, 'R', 7, elevation_z=0.65),
            CornerInfo(4, "Variante della Roggia T4", 1914.9, 19.2, 110.0, 'L', 3, elevation_z=1.33),
            CornerInfo(5, "Variante della Roggia T5", 1953.3, 21.6, 120.0, 'R', 3, elevation_z=1.48),
            CornerInfo(6, "Curva di Lesmo 1", 2061.1, 57.9, 195.0, 'R', 5, elevation_z=4.22),
            CornerInfo(7, "Curva di Lesmo 2", 2277.3, 42.6, 165.0, 'R', 4, elevation_z=7.03),
            CornerInfo(8, "Variante Ascari T8", 3273.3, 36.1, 165.0, 'L', 4, elevation_z=2.03),
            CornerInfo(9, "Variante Ascari T9", 3333.3, 52.0, 195.0, 'R', 5, elevation_z=1.40),
            CornerInfo(10, "Variante Ascari T10", 3444.1, 68.5, 235.0, 'L', 6, elevation_z=0.63),
            CornerInfo(11, "Curva Parabolica (Alboreto)", 5086.1, 215.9, 185.0, 'R', 5, elevation_z=-0.02),
        ]

        # Kerb zones along corner apexes plus full coverage for long sweepers (Curva Grande & Parabolica)
        self.kerb_zones: List[KerbZone] = [
            KerbZone(
                start_s=(c.distance_s - 40.0) % self.length,
                end_s=(c.distance_s + 50.0) % self.length,
                side='both',
                width=1.20,
                bevel_height=0.035,
                stripe_length=1.00
            )
            for c in self.corners
        ]
        # Continuous kerbs across long high-speed sweepers and Rettifilo exit (covers s=1180.0m)
        self.kerb_zones.append(
            KerbZone(start_s=1170.0, end_s=1325.0, side='both', width=1.20, bevel_height=0.035, stripe_length=1.00)
        )
        self.kerb_zones.append(
            KerbZone(start_s=4380.0, end_s=5110.0, side='both', width=1.20, bevel_height=0.035, stripe_length=1.00)
        )

        # 2. 2 Official FIA DRS Zones
        self.drs_zones: List[DRSZone] = [
            DRSZone(
                id=1,
                name="Main Straight",
                detection_s=5680.0,
                activation_s=115.0,
                deactivation_s=980.0
            ),
            DRSZone(
                id=2,
                name="Serraglio Straight",
                detection_s=2040.0,
                activation_s=2360.0,
                deactivation_s=3950.0
            )
        ]

        # 3. Distance Markers (150m, 100m, 50m)
        self.distance_markers: List[DistanceMarker] = self._generate_distance_markers()

        # 4. Pit Lane Architecture (80.0 km/h)
        self.pit_lane: PitLane = self._generate_pit_lane()

        # 5. Full 157-waypoint dataset & Spline synthesis
        self.raw_waypoints: np.ndarray = self._generate_157_waypoints()
        assert self.raw_waypoints.shape == (157, 3), f"Expected shape (157, 3), got {self.raw_waypoints.shape}"

        self.spline: TrackSpline = TrackSpline(
            self.raw_waypoints,
            target_length=5793.0,
            nominal_width=11.0,
            resolution=0.50
        )

    def _generate_157_waypoints(self) -> np.ndarray:
        """
        Synthesizes exactly 157 3D geometric waypoints tracing the authentic
        layout of Autodromo Nazionale Monza with 0 duplicates and 0 self-crossings.
        """
        pts: List[np.ndarray] = [np.array([0.0, 0.0, 0.0], dtype=np.float64)]
        heading: float = math.pi / 2.0  # +Y (North, 90 deg)

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

        # 1. Main Straight: 22 points (s: 0 -> 1130)
        add_line(1130.0, 22, dz=0.1)

        # 2. Variante del Rettifilo Chicane T1/T2: 10 points
        add_arc(25.0, 55.0, 3, 'R', dz=0.1)   # T1 Right
        add_line(25.0, 2, dz=0.05)
        add_arc(28.0, 15.0, 3, 'L', dz=0.1)   # T2 Left
        add_line(30.0, 2, dz=0.05)

        # 3. Curva Grande (Biassono T3): 16 points (R = 320m sweeper)
        add_arc(320.0, 30.0, 16, 'R', dz=0.5)

        # 4. Roggia Approach Straight: 12 points
        add_line(610.0, 12, dz=0.4)

        # 5. Variante della Roggia Chicane T4/T5: 10 points
        add_arc(24.0, 35.0, 3, 'L', dz=0.1)   # T4 Left
        add_line(25.0, 2, dz=0.05)
        add_arc(26.0, 35.0, 3, 'R', dz=0.1)   # T5 Right
        add_line(25.0, 2, dz=0.05)

        # 6. Curva di Lesmo 1 (T6): 8 points
        add_arc(72.0, 70.0, 8, 'R', dz=3.0)

        # 7. Lesmo Chute: 6 points
        add_line(160.0, 6, dz=1.2)

        # 8. Curva di Lesmo 2 (T7): 8 points
        add_arc(55.0, 65.0, 8, 'R', dz=1.4)

        # 9. Serraglio Straight: 18 points (Downhill South-South-West)
        add_line(1020.0, 18, dz=-5.0)

        # 10. Variante Ascari Complex T8/T9/T10: 15 points
        add_arc(45.0, 45.0, 3, 'L', dz=-0.5)  # T8 Left
        add_line(25.0, 2, dz=-0.2)
        add_arc(62.0, 65.0, 5, 'R', dz=-0.5)  # T9 Right
        add_line(15.0, 2, dz=-0.1)
        add_arc(85.0, 45.0, 3, 'L', dz=-0.4)  # T10 Left

        # 11. Back Straight (Rialto): 14 points (South, heading -90)
        add_line(900.0, 14, dz=-0.8)

        # 12. Curva Parabolica (Alboreto T11): 14 points (180 deg Right)
        r_parab = pts[-1][0] / 2.0
        add_arc(r_parab, 180.0, 14, 'R', dz=0.3)

        # 13. Finish Connector back to (0, 0, 0): 4 points
        l_conn = -pts[-1][1]
        add_line(l_conn, 4, dz=-pts[-1][2])

        # Return exactly 157 unique waypoints (excluding duplicate closing point)
        raw = np.array(pts[:-1], dtype=np.float64)
        return raw

    def _generate_distance_markers(self) -> List[DistanceMarker]:
        """Creates 150m, 100m, 50m braking distance indicator markers synchronized to corner apexes."""
        markers = [
            # Variante del Rettifilo (T1 apex: 1090.3m)
            DistanceMarker(1, "Variante del Rettifilo T1", 150.0, 940.3, 'L'),
            DistanceMarker(1, "Variante del Rettifilo T1", 100.0, 990.3, 'L'),
            DistanceMarker(1, "Variante del Rettifilo T1", 50.0, 1040.3, 'L'),

            # Variante della Roggia (T4 apex: 1914.9m)
            DistanceMarker(4, "Variante della Roggia T4", 150.0, 1764.9, 'L'),
            DistanceMarker(4, "Variante della Roggia T4", 100.0, 1814.9, 'L'),
            DistanceMarker(4, "Variante della Roggia T4", 50.0, 1864.9, 'L'),

            # Variante Ascari (T8 apex: 3273.3m)
            DistanceMarker(8, "Variante Ascari T8", 150.0, 3123.3, 'L'),
            DistanceMarker(8, "Variante Ascari T8", 100.0, 3173.3, 'L'),
            DistanceMarker(8, "Variante Ascari T8", 50.0, 3223.3, 'L'),
        ]
        return markers

    def _generate_pit_lane(self) -> PitLane:
        """Constructs authentic Monza pit lane centerline and 20 team stalls."""
        pit_stalls: List[np.ndarray] = []
        for k in range(20):
            # 20 stalls on garage side (X = +16.0m), spaced at 12.0m along Y
            y_stall = -60.0 + k * 12.0
            pit_stalls.append(np.array([16.0, y_stall, 0.0], dtype=np.float64))

        # 30-point centerline from diverger to merger
        pit_wps: List[np.ndarray] = []
        # Taper entry from track (X=0) to pit fast lane (X=+12)
        for y in np.linspace(-200.0, -100.0, 6, endpoint=False):
            t = (y + 200.0) / 100.0
            pit_wps.append(np.array([t * 12.0, y, 0.0]))
        # Parallel pit fast lane (X=+12)
        for y in np.linspace(-100.0, 250.0, 18, endpoint=False):
            pit_wps.append(np.array([12.0, y, 0.0]))
        # Taper exit from pit lane back onto track (X=0)
        for y in np.linspace(250.0, 400.0, 6):
            t = (y - 250.0) / 150.0
            pit_wps.append(np.array([(1.0 - t) * 12.0, y, 0.0]))

        pit = PitLane(
            entry_s=5550.0,
            exit_s=400.0,
            speed_limit_kmh=80.0,
            speed_limit_entry_s=5620.0,
            speed_limit_exit_s=200.0,
            num_stalls=20,
            stall_spacing=12.0,
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
        """Returns tarmac width at distance s (nominal 11.0m, 12m on straights)."""
        return self.spline.get_total_width(s % self.length)

    def get_curvature(self, s: float) -> float:
        """Returns instantaneous curvature kappa(s)."""
        return self.spline.get_curvature(s % self.length)

    def is_in_drs_zone(self, s: float) -> bool:
        """Returns True if vehicle at distance s is inside an active DRS zone."""
        s_norm = s % self.length
        for zone in self.drs_zones:
            if zone.activation_s <= zone.deactivation_s:
                if zone.activation_s <= s_norm <= zone.deactivation_s:
                    return True
            else:
                if s_norm >= zone.activation_s or s_norm <= zone.deactivation_s:
                    return True
        return False

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

        # 3. Off-track grass verge
        return 0.30 if normalized else 0.40

    def get_absolute_surface_mu(self, s: float, d: float) -> float:
        """Returns unnormalized absolute friction coefficient (1.75 tarmac, 1.60 kerb, 0.40 verge)."""
        return self.get_surface_mu(s, d, normalized=False)

    def get_normalized_surface_mu(self, s: float, d: float) -> float:
        """Returns normalized surface grip multiplier (1.00 tarmac, 0.85 kerb, 0.30 verge)."""
        return self.get_surface_mu(s, d, normalized=True)
