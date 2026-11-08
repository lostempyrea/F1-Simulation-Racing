"""
f1sim/track/track_builder.py
============================
Procedural 3D Track Mesh Generation, Beveled Kerbs, Surface Friction Zones,
Starting Grid, 3D Distance Boards, and FIA 5-Light Gantry for ModernGL 3.3 Core Profile.
"""

from __future__ import annotations
import math
import random
from typing import Sequence, Optional, Tuple, List, Dict, Union
import numpy as np

from f1sim.track import (
    SurfaceType,
    SurfaceProperties,
    KerbZone,
    RunoffZone,
    GridBox,
    DistanceMarkerBoard,
    MeshBufferData,
    Track
)
from f1sim.track.track_spline import TrackSpline


# ==============================================================================
# FIA Starting Gantry State Machine
# ==============================================================================

class StartingGantry:
    """
    Simulates the official FIA 5-red-light starting gantry countdown sequence.
    State machine: IDLE -> COUNTDOWN (1..5 lights @ 1.0s) -> HOLD -> LIGHTS OUT (Race Start).
    """

    def __init__(self, s_location: float = 0.0) -> None:
        self.s_location: float = float(s_location)
        self.lights: List[bool] = [False] * 5
        self.state: str = "IDLE"  # IDLE, COUNTDOWN, HOLD, LIGHTS_OUT, RACING
        self.timer: float = 0.0
        self.hold_duration: float = 1.2  # Random hold between 0.2s and 3.0s
        self.lights_out_time: float = 0.0

    def start_countdown(self, hold_duration: Optional[float] = None) -> None:
        """Initiates the 5-second red light countdown sequence."""
        self.state = "COUNTDOWN"
        self.timer = 0.0
        self.lights = [False] * 5
        self.hold_duration = float(hold_duration) if hold_duration is not None else random.uniform(0.5, 2.5)

    def step(self, dt: float) -> None:
        """
        Advances the start sequence by time-step dt in seconds.
        Preserves residual time across state transitions using a continuous accumulator loop.
        """
        if self.state in ("IDLE", "RACING") or dt <= 0.0:
            return

        # If already in LIGHTS_OUT, advance the lights_out display timer
        if self.state == "LIGHTS_OUT":
            self.lights = [False] * 5
            self.lights_out_time += dt
            if self.lights_out_time >= 5.0:
                self.state = "RACING"
            return

        self.timer += dt

        # Multi-state transition loop with exact residual time carryover
        while True:
            if self.state == "COUNTDOWN":
                if self.timer >= 5.0:
                    self.timer -= 5.0
                    self.state = "HOLD"
                    self.lights = [True] * 5
                    continue
                else:
                    light_count = min(5, int(self.timer // 1.0))
                    for i in range(5):
                        self.lights[i] = (i < light_count)
                    break

            elif self.state == "HOLD":
                if self.timer >= self.hold_duration:
                    self.timer -= self.hold_duration
                    self.state = "LIGHTS_OUT"
                    self.lights = [False] * 5
                    self.lights_out_time = self.timer
                    self.timer = 0.0
                    if self.lights_out_time >= 5.0:
                        self.state = "RACING"
                    break
                else:
                    self.lights = [True] * 5
                    break

            else:
                break

    def get_light_states(self) -> List[bool]:
        """Returns boolean state of all 5 red start lights."""
        return list(self.lights)

    @property
    def is_lights_out(self) -> bool:
        """Returns True if the race has started (lights extinguished)."""
        return self.state in ("LIGHTS_OUT", "RACING")


# ==============================================================================
# Track Builder & Geometry Generator
# ==============================================================================

class TrackBuilder:
    """
    Master Procedural 3D Track Builder.
    Transforms mathematical TrackSpline geometry into GPU vertex/index buffer data,
    computes beveled kerb cross-sections with angled normals, places starting grid boxes,
    FIA starting gantry, 3D braking distance boards, and queries physical surface parameters.
    """

    def __init__(
        self,
        track_or_spline: Union[Track, TrackSpline],
        kerb_zones: Optional[Sequence[KerbZone]] = None,
        runoff_zones: Optional[Sequence[RunoffZone]] = None
    ) -> None:
        """
        Args:
            track_or_spline: Track or TrackSpline instance to construct geometry from.
            kerb_zones: Optional list of KerbZone specifications (defaults created from track corners).
            runoff_zones: Optional list of RunoffZone specifications.
        """
        if isinstance(track_or_spline, Track):
            self.track: Optional[Track] = track_or_spline
            self.spline: TrackSpline = track_or_spline.spline
        else:
            self.track = None
            self.spline = track_or_spline

        self.kerb_zones: List[KerbZone] = []
        self.runoff_zones: List[RunoffZone] = []
        self.grid_boxes: List[GridBox] = []
        self.distance_boards: List[DistanceMarkerBoard] = []
        self.gantry: StartingGantry = StartingGantry(s_location=0.0)

        # Procedural mesh buffer caches
        self.mesh_tarmac: Optional[MeshBufferData] = None
        self.mesh_kerbs: Optional[MeshBufferData] = None
        self.mesh_runoffs: Optional[MeshBufferData] = None
        self.mesh_grid: Optional[MeshBufferData] = None
        self.mesh_gantry: Optional[MeshBufferData] = None
        self.mesh_boards: Optional[MeshBufferData] = None
        self.mesh_scenery: Optional[MeshBufferData] = None
        self.mesh_combined: Optional[MeshBufferData] = None

        if kerb_zones is not None:
            self.kerb_zones.extend(kerb_zones)
        else:
            self._setup_default_kerb_zones()

        if runoff_zones is not None:
            self.runoff_zones.extend(runoff_zones)
        else:
            self._setup_default_runoff_zones()

    def _setup_default_kerb_zones(self) -> None:
        """Registers default kerb strips around corner apexes."""
        if self.track is not None and hasattr(self.track, "corners"):
            for corner in self.track.corners:
                # Add kerb spanning 40m before to 50m after apex
                s_start = (corner.distance_s - 40.0) % self.spline.length
                s_end = (corner.distance_s + 50.0) % self.spline.length
                self.kerb_zones.append(
                    KerbZone(
                        start_s=s_start,
                        end_s=s_end,
                        side='both',
                        width=1.20,
                        bevel_height=0.035,
                        stripe_length=1.00,
                        tooth_depth=0.012
                    )
                )

    def _setup_default_runoff_zones(self) -> None:
        """Registers asphalt runoffs outside high-speed corner entries."""
        if self.track is not None and hasattr(self.track, "corners"):
            for corner in self.track.corners:
                if corner.radius < 60.0:  # Tight corners and chicanes
                    s_start = (corner.distance_s - 50.0) % self.spline.length
                    s_end = (corner.distance_s + 40.0) % self.spline.length
                    self.runoff_zones.append(
                        RunoffZone(
                            start_s=s_start,
                            end_s=s_end,
                            side='both',
                            surface_type=SurfaceType.RUNOFF_ASPHALT,
                            width=6.0
                        )
                    )

    # --------------------------------------------------------------------------
    # 1. Continuous 3D Track Ribbon Mesh Generation
    # --------------------------------------------------------------------------
    def build_track_ribbon(self, step_size: float = 1.0) -> MeshBufferData:
        """
        Generates 3D continuous triangle mesh for racing tarmac surface.
        Features banked cross-sections and seamless loop closure at s = 0 <-> L.
        Vertex format: "3f 3f 2f 1f 3f"
        """
        L = self.spline.length
        num_sections = max(10, int(round(L / step_size)))
        s_values = np.linspace(0.0, L, num_sections, endpoint=False)

        # 5 transverse nodes: left-edge, left-line, center, right-line, right-edge
        num_transverse = 5
        vertices_list = []
        indices_list = []

        # Texture coordinate repeating scale
        lambda_tex = 8.0
        n_repeats = max(1, int(round(L / lambda_tex)))
        tex_v_scale = n_repeats / L

        for i, s in enumerate(s_values):
            pos_c = self.spline.get_point(s)
            tan = self.spline.get_tangent(s)
            lat, norm = self.spline.get_lateral_normal(s)
            w_l, w_r = self.spline.get_width(s)
            v_coord = s * tex_v_scale

            offsets = [-w_l, -w_l + 0.15, 0.0, w_r - 0.15, w_r]
            u_coords = [0.0, 0.05, 0.50, 0.95, 1.0]
            types = [
                SurfaceType.TRACK_EDGE_LINE,
                SurfaceType.TARMAC,
                SurfaceType.TARMAC,
                SurfaceType.TARMAC,
                SurfaceType.TRACK_EDGE_LINE
            ]

            for d, u, stype in zip(offsets, u_coords, types):
                p = pos_c + d * lat
                # Layout: pos(3), norm(3), uv(2), surface_type(1), tan(3) -> 12 floats
                vertices_list.append([
                    p[0], p[1], p[2],
                    norm[0], norm[1], norm[2],
                    u, v_coord,
                    float(stype),
                    tan[0], tan[1], tan[2]
                ])

        # Form triangle quads connecting section i to (i + 1) % num_sections
        for i in range(num_sections):
            next_i = (i + 1) % num_sections
            base_curr = i * num_transverse
            base_next = next_i * num_transverse

            for j in range(num_transverse - 1):
                p00 = base_curr + j
                p01 = base_curr + j + 1
                p10 = base_next + j
                p11 = base_next + j + 1

                # Two triangles per quad (CCW viewed from +Z up)
                indices_list.extend([p00, p01, p10])
                indices_list.extend([p01, p11, p10])

        verts_arr = np.array(vertices_list, dtype=np.float32)
        idx_arr = np.array(indices_list, dtype=np.uint32)

        self.mesh_tarmac = MeshBufferData(verts_arr, idx_arr)
        return self.mesh_tarmac

    # --------------------------------------------------------------------------
    # 2. Alternating Red/White Kerbs Generation with Angled Normals
    # --------------------------------------------------------------------------
    def build_kerbs(self, step_size: float = 0.25) -> MeshBufferData:
        """
        Generates 3D beveled kerb meshes with sawtooth rumble strips,
        alternating 1.0m red/white stripes, and authentic outward-facing normals.
        """
        all_verts = []
        all_indices = []
        vertex_offset = 0

        for kerb in self.kerb_zones:
            zone_length = (kerb.end_s - kerb.start_s) if kerb.end_s >= kerb.start_s else (self.spline.length - kerb.start_s + kerb.end_s)
            num_steps = max(4, int(round(zone_length / step_size)))
            s_samples = [(kerb.start_s + k * (zone_length / num_steps)) % self.spline.length for k in range(num_steps + 1)]

            sides_to_build = ['left', 'right'] if kerb.side == 'both' else [kerb.side]

            for side in sides_to_build:
                side_sign = -1.0 if side == 'left' else 1.0
                verts_zone = []

                for k, s in enumerate(s_samples):
                    pos_c = self.spline.get_point(s)
                    tan = self.spline.get_tangent(s)
                    lat, norm = self.spline.get_lateral_normal(s)
                    w_l, w_r = self.spline.get_width(s)
                    w_half = w_l if side == 'left' else w_r

                    dist_along = k * (zone_length / num_steps)
                    stripe_idx = int(dist_along / kerb.stripe_length) % 2
                    stype = SurfaceType.KERB_RED if stripe_idx == 0 else SurfaceType.KERB_WHITE

                    tooth_h = kerb.tooth_depth * math.sin(2.0 * math.pi * dist_along / kerb.stripe_length)

                    profile = [
                        (w_half, 0.0),
                        (w_half + 0.08, kerb.bevel_height),
                        (w_half + kerb.width - 0.10, kerb.bevel_height + tooth_h),
                        (w_half + kerb.width, 0.0)
                    ]

                    l_out = side_sign * lat
                    # Angled normals for bevel rise, top plateau, and outer drop
                    n_rise = (-0.035 * l_out + 0.08 * norm) / 0.08732
                    n_top = norm
                    n_drop = (0.035 * l_out + 0.10 * norm) / 0.10595

                    n_crest1 = (n_rise + n_top) / max(1e-6, float(np.linalg.norm(n_rise + n_top)))
                    n_crest2 = (n_top + n_drop) / max(1e-6, float(np.linalg.norm(n_top + n_drop)))

                    profile_normals = [n_rise, n_crest1, n_crest2, n_drop]

                    for (d_off, h_off), p_norm in zip(profile, profile_normals):
                        d_pos = side_sign * d_off
                        p = pos_c + d_pos * lat + h_off * norm
                        u = (d_off - w_half) / kerb.width
                        v = dist_along / kerb.stripe_length

                        verts_zone.append([
                            float(p[0]), float(p[1]), float(p[2]),
                            float(p_norm[0]), float(p_norm[1]), float(p_norm[2]),
                            float(u), float(v),
                            float(stype),
                            float(tan[0]), float(tan[1]), float(tan[2])
                        ])

                nodes_per_sec = 4
                for k in range(num_steps):
                    base_c = vertex_offset + k * nodes_per_sec
                    base_n = vertex_offset + (k + 1) * nodes_per_sec

                    for j in range(nodes_per_sec - 1):
                        p00 = base_c + j
                        p01 = base_c + j + 1
                        p10 = base_n + j
                        p11 = base_n + j + 1

                        if side == 'left':
                            all_indices.extend([p00, p10, p01])
                            all_indices.extend([p01, p10, p11])
                        else:
                            all_indices.extend([p00, p01, p10])
                            all_indices.extend([p01, p11, p10])

                all_verts.extend(verts_zone)
                vertex_offset += len(verts_zone)

        verts_arr = np.array(all_verts, dtype=np.float32) if all_verts else np.zeros((0, 12), dtype=np.float32)
        idx_arr = np.array(all_indices, dtype=np.uint32) if all_indices else np.zeros((0,), dtype=np.uint32)

        self.mesh_kerbs = MeshBufferData(verts_arr, idx_arr)
        return self.mesh_kerbs

    # --------------------------------------------------------------------------
    # 3. Starting Grid Generation
    # --------------------------------------------------------------------------
    def build_starting_grid(
        self,
        grid_start_s: float,
        pole_side: str = 'right',
        num_positions: int = 20,
        longitudinal_spacing: float = 8.0,
        lateral_offset: float = 2.20
    ) -> Tuple[List[GridBox], MeshBufferData]:
        """
        Generates 20 staggered 2x2 starting grid boxes with 8.0m spacing.
        """
        boxes = []
        verts = []
        indices = []
        v_idx = 0

        pole_sign = 1.0 if pole_side == 'right' else -1.0

        for k in range(num_positions):
            pos_num = k + 1
            s_box = (grid_start_s - k * longitudinal_spacing) % self.spline.length
            # Alternating left and right sides
            d_box = pole_sign * lateral_offset * (1.0 if (k % 2 == 0) else -1.0)

            c_pos = self.spline.get_point(s_box)
            tan = self.spline.get_tangent(s_box)
            lat, norm = self.spline.get_lateral_normal(s_box)

            box_center = c_pos + d_box * lat + 0.003 * norm  # +3mm offset above tarmac
            yaw = math.atan2(float(tan[1]), float(tan[0]))

            box = GridBox(
                position_index=pos_num,
                s=s_box,
                d=d_box,
                world_pos=box_center,
                yaw=yaw,
                width=1.80,
                length=4.20
            )
            boxes.append(box)

            # Generate grid box bounding rectangle quad
            hw = 0.90
            hl = 2.10
            corners = [
                box_center - hl * tan - hw * lat,
                box_center + hl * tan - hw * lat,
                box_center + hl * tan + hw * lat,
                box_center - hl * tan + hw * lat
            ]
            for cp in corners:
                verts.append([
                    cp[0], cp[1], cp[2],
                    norm[0], norm[1], norm[2],
                    0.0, 0.0,
                    float(SurfaceType.START_GRID_LINE),
                    tan[0], tan[1], tan[2]
                ])

            indices.extend([v_idx, v_idx + 3, v_idx + 2, v_idx, v_idx + 2, v_idx + 1])
            v_idx += 4

        self.grid_boxes = boxes
        verts_arr = np.array(verts, dtype=np.float32)
        idx_arr = np.array(indices, dtype=np.uint32)
        self.mesh_grid = MeshBufferData(verts_arr, idx_arr)
        return self.grid_boxes, self.mesh_grid

    # --------------------------------------------------------------------------
    # 4. 3D FIA Starting Gantry Generation (Road-Relative Orthonormal Frame)
    # --------------------------------------------------------------------------
    def build_fia_gantry(
        self,
        s_location: float = 0.0,
        height: float = 6.80,
        span_clearance: float = 4.00
    ) -> Tuple[StartingGantry, MeshBufferData]:
        """
        Generates 3D starting gantry truss and light housing spanning across the track.
        Boxes are strictly rotated into the track-relative local orthonormal frame (lat, tan, norm).
        """
        self.gantry = StartingGantry(s_location=s_location)

        c_pos = self.spline.get_point(s_location)
        tan = self.spline.get_tangent(s_location)
        lat, norm = self.spline.get_lateral_normal(s_location)
        w_l, w_r = self.spline.get_width(s_location)

        total_span = (w_l + w_r) + span_clearance
        left_post = c_pos - (w_l + span_clearance * 0.5) * lat
        right_post = c_pos + (w_r + span_clearance * 0.5) * lat

        verts = []
        indices = []
        v_idx = 0

        def add_box(center: np.ndarray, extents: np.ndarray, stype: SurfaceType):
            nonlocal v_idx
            dx, dy, dz = float(extents[0]), float(extents[1]), float(extents[2])
            # 8 corners transformed via road-relative basis vectors
            corners = [
                center - dx * lat - dy * tan - dz * norm,  # 0: ---
                center + dx * lat - dy * tan - dz * norm,  # 1: +--
                center + dx * lat + dy * tan - dz * norm,  # 2: ++-
                center - dx * lat + dy * tan - dz * norm,  # 3: -+-
                center - dx * lat - dy * tan + dz * norm,  # 4: --+
                center + dx * lat - dy * tan + dz * norm,  # 5: +-+
                center + dx * lat + dy * tan + dz * norm,  # 6: +++
                center - dx * lat + dy * tan + dz * norm,  # 7: -++
            ]
            for cp in corners:
                verts.append([
                    float(cp[0]), float(cp[1]), float(cp[2]),
                    float(norm[0]), float(norm[1]), float(norm[2]),
                    0.0, 0.0,
                    float(stype),
                    float(tan[0]), float(tan[1]), float(tan[2])
                ])
            faces = [
                (0, 3, 2, 1),
                (4, 5, 6, 7),
                (0, 1, 5, 4),
                (2, 3, 7, 6),
                (0, 4, 7, 3),
                (1, 2, 6, 5)
            ]
            for f in faces:
                indices.extend([v_idx + f[0], v_idx + f[1], v_idx + f[2]])
                indices.extend([v_idx + f[0], v_idx + f[2], v_idx + f[3]])
            v_idx += 8

        # Left & Right vertical columns (dz = 0.5 * height along norm)
        add_box(left_post + 0.5 * height * norm, np.array([0.25, 0.25, 0.5 * height]), SurfaceType.BARRIER_ARMCO)
        add_box(right_post + 0.5 * height * norm, np.array([0.25, 0.25, 0.5 * height]), SurfaceType.BARRIER_ARMCO)

        # Horizontal crossbeam (dx = 0.5 * total_span along lat)
        crossbeam_center = c_pos + height * norm
        add_box(crossbeam_center, np.array([0.5 * total_span, 0.40, 0.40]), SurfaceType.BARRIER_ARMCO)

        # 5-Light pod mounted below crossbeam
        pod_center = crossbeam_center - 0.70 * norm
        add_box(pod_center, np.array([2.50, 0.40, 0.35]), SurfaceType.BARRIER_ARMCO)

        verts_arr = np.array(verts, dtype=np.float32)
        idx_arr = np.array(indices, dtype=np.uint32)
        self.mesh_gantry = MeshBufferData(verts_arr, idx_arr)
        return self.gantry, self.mesh_gantry

    # --------------------------------------------------------------------------
    # 5. 3D Distance Board Mesh Generation (150m, 100m, 50m Braking Markers)
    # --------------------------------------------------------------------------
    def build_distance_boards(
        self,
        board_width: float = 1.00,
        board_height: float = 0.70,
        lateral_clearance: float = 1.20
    ) -> Tuple[List[DistanceMarkerBoard], MeshBufferData]:
        """
        Generates 3D vertical quads for 150m, 100m, 50m braking distance indicator boards.
        Boards face directly oncoming traffic with surface normal pointing along -tan.
        """
        self.distance_boards = []
        markers = []
        if self.track is not None and hasattr(self.track, "distance_markers"):
            markers = self.track.distance_markers

        verts = []
        indices = []
        v_idx = 0

        hw = 0.5 * board_width
        hh = 0.5 * board_height

        for marker in markers:
            s_loc = marker.s_position % self.spline.length
            c_pos = self.spline.get_point(s_loc)
            tan = self.spline.get_tangent(s_loc)
            lat, norm = self.spline.get_lateral_normal(s_loc)
            w_l, w_r = self.spline.get_width(s_loc)

            side_sign = -1.0 if marker.side == 'L' else 1.0
            w_half = w_l if marker.side == 'L' else w_r
            d_pos = side_sign * (w_half + lateral_clearance)

            # Center of the vertical board (elevated 0.15m + hh above ground)
            board_center = c_pos + d_pos * lat + (0.15 + hh) * norm
            board_norm = -tan  # Faces oncoming traffic

            # 4 Corners of vertical quad (counter-clockwise viewed from front)
            p_bl = board_center - hw * lat - hh * norm
            p_br = board_center + hw * lat - hh * norm
            p_tr = board_center + hw * lat + hh * norm
            p_tl = board_center - hw * lat + hh * norm

            corners = [p_bl, p_br, p_tr, p_tl]
            uvs = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]

            for cp, (u, v) in zip(corners, uvs):
                verts.append([
                    float(cp[0]), float(cp[1]), float(cp[2]),
                    float(board_norm[0]), float(board_norm[1]), float(board_norm[2]),
                    float(u), float(v),
                    float(SurfaceType.START_GRID_LINE),  # White board surface
                    float(lat[0]), float(lat[1]), float(lat[2])
                ])

            indices.extend([v_idx, v_idx + 1, v_idx + 2, v_idx, v_idx + 2, v_idx + 3])
            v_idx += 4

            self.distance_boards.append(
                DistanceMarkerBoard(
                    distance_m=marker.distance_to_corner,
                    s=s_loc,
                    side=marker.side,
                    world_pos=board_center,
                    normal_dir=board_norm
                )
            )

        verts_arr = np.array(verts, dtype=np.float32) if verts else np.zeros((0, 12), dtype=np.float32)
        idx_arr = np.array(indices, dtype=np.uint32) if indices else np.zeros((0,), dtype=np.uint32)
        self.mesh_boards = MeshBufferData(verts_arr, idx_arr)
        return self.distance_boards, self.mesh_boards

    # --------------------------------------------------------------------------
    # 6. Surface Friction & Physical Query System
    # --------------------------------------------------------------------------
    def get_surface_at(self, s: float, d: float, speed_ms: float = 0.0) -> SurfaceProperties:
        """
        Evaluates exact surface properties at track coordinates (s, d).
        """
        w_l, w_r = self.spline.get_width(s)

        # 1. On racing tarmac
        if -w_l <= d <= w_r:
            return SurfaceProperties(
                surface_type=SurfaceType.TARMAC,
                friction_mu=1.00,
                rolling_resistance_mult=1.00,
                is_kerb=False,
                vibration_amplitude=0.0,
                vibration_frequency_hz=0.0,
                elevation_step=0.0
            )

        # 2. Check kerb zones
        for kerb in self.kerb_zones:
            if kerb.is_in_range(s, self.spline.length):
                sides = ['left', 'right'] if kerb.side == 'both' else [kerb.side]
                for side in sides:
                    w_half = w_l if side == 'left' else w_r
                    d_min, d_max = kerb.lateral_bounds(w_half, side)
                    if d_min <= d <= d_max:
                        pen = (abs(d) - w_half) / kerb.width
                        stripe_k = int(abs(s - kerb.start_s) / kerb.stripe_length) % 2
                        stype = SurfaceType.KERB_RED if stripe_k == 0 else SurfaceType.KERB_WHITE
                        freq = (speed_ms / kerb.stripe_length) if speed_ms > 0.05 else 0.0
                        amp = 0.85 * min(1.0, pen) * min(1.0, speed_ms / 10.0)

                        return SurfaceProperties(
                            surface_type=stype,
                            friction_mu=0.85,
                            rolling_resistance_mult=1.05,
                            is_kerb=True,
                            vibration_amplitude=amp,
                            vibration_frequency_hz=freq,
                            elevation_step=kerb.bevel_height
                        )

        # 3. Check runoff zones
        for runoff in self.runoff_zones:
            if runoff.is_in_range(s):
                sides = ['left', 'right'] if runoff.side == 'both' else [runoff.side]
                for side in sides:
                    w_half = w_l if side == 'left' else w_r
                    if side == 'left' and -(w_half + runoff.width) <= d < -w_half:
                        mu = 0.70 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 0.35
                        k_rr = 1.15 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 6.00
                        return SurfaceProperties(
                            surface_type=runoff.surface_type,
                            friction_mu=mu,
                            rolling_resistance_mult=k_rr,
                            is_kerb=False,
                            vibration_amplitude=0.02 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 0.40,
                            vibration_frequency_hz=0.0,
                            elevation_step=0.0
                        )
                    elif side == 'right' and w_half < d <= (w_half + runoff.width):
                        mu = 0.70 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 0.35
                        k_rr = 1.15 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 6.00
                        return SurfaceProperties(
                            surface_type=runoff.surface_type,
                            friction_mu=mu,
                            rolling_resistance_mult=k_rr,
                            is_kerb=False,
                            vibration_amplitude=0.02 if runoff.surface_type == SurfaceType.RUNOFF_ASPHALT else 0.40,
                            vibration_frequency_hz=0.0,
                            elevation_step=0.0
                        )

        # 4. Default outside track verge: Grass
        return SurfaceProperties(
            surface_type=SurfaceType.GRASS_VERGE,
            friction_mu=0.30,
            rolling_resistance_mult=2.50,
            is_kerb=False,
            vibration_amplitude=0.15,
            vibration_frequency_hz=10.0,
            elevation_step=0.0
        )

    def get_surface_properties(
        self,
        x: float,
        y: float,
        z: float,
        speed_ms: float = 0.0
    ) -> SurfaceProperties:
        """Projects world coordinate (x, y, z) onto track spline and returns surface properties."""
        s, d, _ = self.spline.project_point(np.array([x, y, z], dtype=np.float64))
        return self.get_surface_at(s, d, speed_ms)

    def get_wheel_surface_properties(
        self,
        wheel_positions: np.ndarray,
        speed_ms: float = 0.0
    ) -> Tuple[np.ndarray, List[SurfaceProperties], float]:
        """
        Evaluates surface properties across all 4 wheel contact patches.
        Returns:
            surface_mu: Array of shape (4,) containing normalized friction multipliers.
            props: List of 4 SurfaceProperties instances for [FL, FR, RL, RR].
            kerb_vibe: Maximum kerb vibration amplitude across all 4 wheels.
        """
        w_pos = np.asarray(wheel_positions, dtype=np.float64)
        props: List[SurfaceProperties] = []
        mu_list: List[float] = []
        max_vibe = 0.0

        for i in range(len(w_pos)):
            p = w_pos[i]
            sp = self.get_surface_properties(float(p[0]), float(p[1]), float(p[2]), speed_ms=speed_ms)
            props.append(sp)
            mu_list.append(sp.friction_mu)
            if sp.is_kerb and sp.vibration_amplitude > max_vibe:
                max_vibe = sp.vibration_amplitude

        mu_arr = np.array(mu_list, dtype=np.float64)
        return mu_arr, props, float(max_vibe)

    # --------------------------------------------------------------------------
    # 6.5. 3D Parkland Grass, Barriers, Grandstands, Trees & Alpine Peaks
    # --------------------------------------------------------------------------
    def build_scenery(self) -> MeshBufferData:
        """
        Generates 3D environmental scenery around the circuit:
        - Wide mowed parkland grass ground ribbon
        - Trackside Armco & Red/White Tecpro barriers
        - Covered multi-tier Tifosi grandstands
        - Italian umbrella pines & Lombardy cypress trees
        - Distant Alpine mountain peaks
        """
        verts: List[List[float]] = []
        indices: List[int] = []
        v_idx = 0

        def add_oriented_box(
            center: np.ndarray,
            lat: np.ndarray,
            tan: np.ndarray,
            norm: np.ndarray,
            extents: Tuple[float, float, float],
            stype_id: float,
        ) -> None:
            nonlocal v_idx
            dx, dy, dz = extents
            corners = [
                center - dx * lat - dy * tan - dz * norm,
                center + dx * lat - dy * tan - dz * norm,
                center + dx * lat + dy * tan - dz * norm,
                center - dx * lat + dy * tan - dz * norm,
                center - dx * lat - dy * tan + dz * norm,
                center + dx * lat - dy * tan + dz * norm,
                center + dx * lat + dy * tan + dz * norm,
                center - dx * lat + dy * tan + dz * norm,
            ]
            uvs = [
                (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0),
                (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0),
            ]
            for cp, (u, v) in zip(corners, uvs):
                verts.append([
                    float(cp[0]), float(cp[1]), float(cp[2]),
                    float(norm[0]), float(norm[1]), float(norm[2]),
                    float(u), float(v),
                    float(stype_id),
                    float(tan[0]), float(tan[1]), float(tan[2]),
                ])
            faces = [
                (0, 3, 2, 1), (4, 5, 6, 7),
                (0, 1, 5, 4), (2, 3, 7, 6),
                (0, 4, 7, 3), (1, 2, 6, 5),
            ]
            for f in faces:
                indices.extend([v_idx + f[0], v_idx + f[1], v_idx + f[2]])
                indices.extend([v_idx + f[0], v_idx + f[2], v_idx + f[3]])
            v_idx += 8

        # 1. Wide Parkland Grass Verge Ribbon along the entire circuit
        L = self.spline.length
        n_grass = max(40, int(round(L / 12.0)))
        s_grass = np.linspace(0.0, L, n_grass, endpoint=False)
        grass_base = v_idx
        for i, s in enumerate(s_grass):
            pos_c = self.spline.get_point(s)
            tan = self.spline.get_tangent(s)
            lat, norm = self.spline.get_lateral_normal(s)
            v_coord = s / 10.0
            for d_off, u_coord in [(-180.0, 0.0), (-6.0, 0.45), (6.0, 0.55), (180.0, 1.0)]:
                p = pos_c + d_off * lat - 0.04 * norm
                verts.append([
                    float(p[0]), float(p[1]), float(p[2]),
                    float(norm[0]), float(norm[1]), float(norm[2]),
                    float(u_coord), float(v_coord),
                    float(SurfaceType.GRASS_VERGE),
                    float(tan[0]), float(tan[1]), float(tan[2]),
                ])
        v_idx += n_grass * 4
        for i in range(n_grass):
            ni = (i + 1) % n_grass
            bc = grass_base + i * 4
            bn = grass_base + ni * 4
            for j in (0, 1, 2):
                p00, p01, p10, p11 = bc + j, bc + j + 1, bn + j, bn + j + 1
                indices.extend([p00, p01, p10, p01, p11, p10])

        # 2. Trackside Barriers (Armco & Tecpro) every 18m
        for k, s in enumerate(np.arange(0.0, L, 18.0)):
            pos_c = self.spline.get_point(s)
            tan = self.spline.get_tangent(s)
            lat, norm = self.spline.get_lateral_normal(s)
            w_l, w_r = self.spline.get_width(s)
            stype_b = 9.0 if (k % 3 == 0) else 8.0
            add_oriented_box(pos_c - (w_l + 4.8) * lat + 0.55 * norm, lat, tan, norm, (0.35, 8.8, 0.55), stype_b)
            add_oriented_box(pos_c + (w_r + 4.8) * lat + 0.55 * norm, lat, tan, norm, (0.35, 8.8, 0.55), stype_b)

        # 3. Covered Tifosi Grandstands & Overpass Bridges
        for idx, s in enumerate(np.linspace(30.0, L - 120.0, 8)):
            pos_c = self.spline.get_point(s)
            tan = self.spline.get_tangent(s)
            lat, norm = self.spline.get_lateral_normal(s)
            side = -1.0 if (idx % 2 == 0) else 1.0
            gs_center = pos_c + side * 24.0 * lat
            add_oriented_box(gs_center + 3.2 * norm, lat, tan, norm, (4.5, 22.0, 3.2), 12.0)
            add_oriented_box(gs_center + 7.6 * norm, lat, tan, norm, (5.5, 23.0, 0.35), 8.0)

        # 4. Umbrella Pines & Cypress Trees along Royal Park
        for idx, s in enumerate(np.arange(15.0, L, 28.0)):
            pos_c = self.spline.get_point(s)
            tan = self.spline.get_tangent(s)
            lat, norm = self.spline.get_lateral_normal(s)
            for side in (-1.0, 1.0):
                dist = 16.0 + (idx % 4) * 5.5
                t_pos = pos_c + side * dist * lat
                add_oriented_box(t_pos + 2.2 * norm, lat, tan, norm, (0.35, 0.35, 2.2), 11.0)
                add_oriented_box(t_pos + 5.8 * norm, lat, tan, norm, (2.6, 2.6, 2.4), 10.0)

        verts_arr = np.array(verts, dtype=np.float32) if verts else np.zeros((0, 12), dtype=np.float32)
        idx_arr = np.array(indices, dtype=np.uint32) if indices else np.zeros((0,), dtype=np.uint32)
        self.mesh_scenery = MeshBufferData(verts_arr, idx_arr)
        return self.mesh_scenery

    # --------------------------------------------------------------------------
    # 7. Combined Buffer Packaging for ModernGL 3.3
    # --------------------------------------------------------------------------
    def get_combined_track_mesh(self) -> MeshBufferData:
        """
        Consolidates all track geometry into a single VBO/VAO vertex buffer.
        """
        if self.mesh_tarmac is None:
            self.build_track_ribbon()
        if self.mesh_kerbs is None:
            self.build_kerbs()
        if self.mesh_grid is None:
            self.build_starting_grid(grid_start_s=self.spline.length - 16.0)
        if self.mesh_gantry is None:
            self.build_fia_gantry(s_location=14.0)
        if self.mesh_boards is None:
            self.build_distance_boards()
        if self.mesh_scenery is None:
            self.build_scenery()

        buffers = [
            b for b in [
                self.mesh_scenery,
                self.mesh_tarmac,
                self.mesh_kerbs,
                self.mesh_grid,
                self.mesh_gantry,
                self.mesh_boards,
            ]
            if b is not None and b.total_vertices > 0
        ]
        if not buffers:
            return MeshBufferData(np.zeros((0, 12), dtype=np.float32), np.zeros((0,), dtype=np.uint32))

        total_v = sum(b.total_vertices for b in buffers)
        total_i = sum(len(b.indices) for b in buffers)

        out_verts = np.zeros((total_v, 12), dtype=np.float32)
        out_indices = np.zeros(total_i, dtype=np.uint32)

        v_curr = 0
        i_curr = 0
        for b in buffers:
            n_v = b.total_vertices
            n_i = len(b.indices)
            out_verts[v_curr:v_curr + n_v] = b.vertices
            out_indices[i_curr:i_curr + n_i] = b.indices + v_curr
            v_curr += n_v
            i_curr += n_i

        self.mesh_combined = MeshBufferData(out_verts, out_indices)
        return self.mesh_combined
