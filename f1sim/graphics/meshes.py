"""
f1sim/graphics/meshes.py
========================
Procedural 3D mesh generator for the Formula 1 first-person cockpit:
1. Carbon-fiber monocoque tub & nosecone
2. Titanium Halo safety structure (central pillar at X = 0.0 m, width = 0.05 m <= 0.08 m)
3. Dual aerodynamic rear-view wing mirrors (symmetric at X = -0.48 m and X = +0.48 m)
4. Rotating rectangular F1 butterfly steering wheel (+/- 180 deg = [-pi, +pi] rad)
5. Embedded PCU-8D LCD screen quad & mirror reflective glass quads

All meshes use an interleaved 8-float vertex format:
    [x, y, z, nx, ny, nz, u, v]  (32 bytes/vertex, float32)
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Dict, List, Tuple
import numpy as np

from f1sim.core.math3d import Mat3, Mat4, Vec3


@dataclass(frozen=True)
class CockpitGeometrySpec:
    """Canonical dimensions and positions of F1 cockpit components in meters."""
    driver_eye_x: float = 0.0
    driver_eye_y: float = 0.68
    driver_eye_z: float = -0.25

    halo_center_x: float = 0.0
    halo_strut_width: float = 0.05      # 50 mm titanium central strut (<= 0.08 m)
    halo_strut_z: float = -0.92
    halo_hoop_height_y: float = 0.84

    left_mirror_x: float = -0.48
    right_mirror_x: float = 0.48
    mirror_y: float = 0.64
    mirror_z: float = -0.68
    mirror_width: float = 0.16
    mirror_height: float = 0.07

    wheel_hub_x: float = 0.0
    wheel_hub_y: float = 0.55
    wheel_hub_z: float = -0.58
    wheel_width: float = 0.28           # 280 mm F1 butterfly wheel width
    wheel_height: float = 0.16          # 160 mm F1 butterfly wheel height
    wheel_min_angle_rad: float = -math.pi
    wheel_max_angle_rad: float = math.pi
    column_rake_deg: float = -16.0


@dataclass
class MeshData:
    """Interleaved float32 vertex array [x, y, z, nx, ny, nz, u, v] and uint32 triangle indices."""
    name: str
    vertices: np.ndarray  # shape (N, 8), dtype=np.float32
    indices: np.ndarray   # shape (M,), dtype=np.uint32

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def triangle_count(self) -> int:
        return int(self.indices.shape[0] // 3)

    def to_bytes(self) -> Tuple[bytes, bytes]:
        return (
            np.ascontiguousarray(self.vertices, dtype=np.float32).tobytes(),
            np.ascontiguousarray(self.indices, dtype=np.uint32).tobytes(),
        )


def compute_wheel_rotation_matrix_3x3(angle_rad: float) -> np.ndarray:
    """
    Computes the 3x3 orthogonal rotation matrix around the steering column axis.
    Clamps angle_rad to [-pi, +pi] and strictly preserves Euclidean vector length.
    """
    theta = float(np.clip(angle_rad, -math.pi, math.pi))
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array(
        [
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def compute_wheel_model_matrix_4x4(
    angle_rad: float,
    spec: CockpitGeometrySpec | None = None,
) -> Mat4:
    """
    Computes the 4x4 model matrix placing the steering wheel at the cockpit
    hub position, applying steering column rake, and rotating by `angle_rad` in [-pi, +pi].
    """
    sp = spec if spec is not None else CockpitGeometrySpec()
    theta = float(np.clip(angle_rad, sp.wheel_min_angle_rad, sp.wheel_max_angle_rad))
    rake_rad = math.radians(sp.column_rake_deg)

    rot_z = Mat3.rotation_z(theta)
    rot_rake = Mat3.rotation_x(rake_rad)
    combined_rot = rot_rake @ rot_z
    hub_pos = Vec3(sp.wheel_hub_x, sp.wheel_hub_y, sp.wheel_hub_z)
    return Mat4.from_mat3_and_pos(combined_rot, hub_pos)


def _append_box(
    verts: List[List[float]],
    inds: List[int],
    center: Tuple[float, float, float],
    size: Tuple[float, float, float],
    uv_scale: Tuple[float, float] = (1.0, 1.0),
) -> None:
    """Appends an axis-aligned 3D box with outward normals and UVs."""
    cx, cy, cz = center
    hx, hy, hz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
    u_max, v_max = uv_scale

    faces = [
        # Front (+Z towards camera)
        ((0.0, 0.0, 1.0), [
            (cx - hx, cy - hy, cz + hz, 0.0, 0.0),
            (cx + hx, cy - hy, cz + hz, u_max, 0.0),
            (cx + hx, cy + hy, cz + hz, u_max, v_max),
            (cx - hx, cy + hy, cz + hz, 0.0, v_max),
        ]),
        # Back (-Z forward)
        ((0.0, 0.0, -1.0), [
            (cx + hx, cy - hy, cz - hz, 0.0, 0.0),
            (cx - hx, cy - hy, cz - hz, u_max, 0.0),
            (cx - hx, cy + hy, cz - hz, u_max, v_max),
            (cx + hx, cy + hy, cz - hz, 0.0, v_max),
        ]),
        # Left (-X)
        ((-1.0, 0.0, 0.0), [
            (cx - hx, cy - hy, cz - hz, 0.0, 0.0),
            (cx - hx, cy - hy, cz + hz, u_max, 0.0),
            (cx - hx, cy + hy, cz + hz, u_max, v_max),
            (cx - hx, cy + hy, cz - hz, 0.0, v_max),
        ]),
        # Right (+X)
        ((1.0, 0.0, 0.0), [
            (cx + hx, cy - hy, cz + hz, 0.0, 0.0),
            (cx + hx, cy - hy, cz - hz, u_max, 0.0),
            (cx + hx, cy + hy, cz - hz, u_max, v_max),
            (cx + hx, cy + hy, cz + hz, 0.0, v_max),
        ]),
        # Top (+Y)
        ((0.0, 1.0, 0.0), [
            (cx - hx, cy + hy, cz + hz, 0.0, 0.0),
            (cx + hx, cy + hy, cz + hz, u_max, 0.0),
            (cx + hx, cy + hy, cz - hz, u_max, v_max),
            (cx - hx, cy + hy, cz - hz, 0.0, v_max),
        ]),
        # Bottom (-Y)
        ((0.0, -1.0, 0.0), [
            (cx - hx, cy - hy, cz - hz, 0.0, 0.0),
            (cx + hx, cy - hy, cz - hz, u_max, 0.0),
            (cx + hx, cy - hy, cz + hz, u_max, v_max),
            (cx - hx, cy - hy, cz + hz, 0.0, v_max),
        ]),
    ]

    for (nx, ny, nz), corners in faces:
        base_idx = len(verts)
        for px, py, pz, u, v in corners:
            verts.append([px, py, pz, nx, ny, nz, u, v])
        inds.extend([base_idx, base_idx + 1, base_idx + 2, base_idx, base_idx + 2, base_idx + 3])


def _append_quad(
    verts: List[List[float]],
    inds: List[int],
    p0: Tuple[float, float, float],
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
    p3: Tuple[float, float, float],
    normal: Tuple[float, float, float] = (0.0, 0.0, 1.0),
    flip_u: bool = False,
) -> None:
    """Appends a single textured quad [0, 1] x [0, 1]."""
    base_idx = len(verts)
    nx, ny, nz = normal
    u0, u1 = (1.0, 0.0) if flip_u else (0.0, 1.0)
    verts.append([p0[0], p0[1], p0[2], nx, ny, nz, u0, 1.0])
    verts.append([p1[0], p1[1], p1[2], nx, ny, nz, u1, 1.0])
    verts.append([p2[0], p2[1], p2[2], nx, ny, nz, u1, 0.0])
    verts.append([p3[0], p3[1], p3[2], nx, ny, nz, u0, 0.0])
    inds.extend([base_idx, base_idx + 1, base_idx + 2, base_idx, base_idx + 2, base_idx + 3])


class CockpitMeshBuilder:
    """
    Constructs all 3D meshes required for the F1 first-person cockpit view.
    """

    def __init__(self, spec: CockpitGeometrySpec | None = None) -> None:
        self.spec: CockpitGeometrySpec = spec if spec is not None else CockpitGeometrySpec()

    def build_monocoque_mesh(self) -> MeshData:
        """Builds the carbon-fiber monocoque tub, cockpit coaming, and nosecone."""
        verts: List[List[float]] = []
        inds: List[int] = []

        # Left and right cockpit side coaming walls
        _append_box(verts, inds, center=(-0.34, 0.48, -0.75), size=(0.10, 0.24, 1.30))
        _append_box(verts, inds, center=(+0.34, 0.48, -0.75), size=(0.10, 0.24, 1.30))
        # Cockpit dashboard cowl & wind deflector
        _append_box(verts, inds, center=(0.0, 0.56, -0.82), size=(0.62, 0.08, 0.40))
        # Tapered front nosecone extending forward (-Z)
        _append_box(verts, inds, center=(0.0, 0.44, -1.65), size=(0.44, 0.18, 1.30))
        _append_box(verts, inds, center=(0.0, 0.36, -2.65), size=(0.28, 0.14, 1.10))
        # Front 18-inch wheel arches / tyre silhouettes
        _append_box(verts, inds, center=(-0.82, 0.36, -1.75), size=(0.36, 0.72, 0.72))
        _append_box(verts, inds, center=(+0.82, 0.36, -1.75), size=(0.36, 0.72, 0.72))
        # Upper suspension wishbone arms
        _append_box(verts, inds, center=(-0.50, 0.46, -1.70), size=(0.56, 0.03, 0.06))
        _append_box(verts, inds, center=(+0.50, 0.46, -1.70), size=(0.56, 0.03, 0.06))

        return MeshData(
            name="monocoque",
            vertices=np.asarray(verts, dtype=np.float32),
            indices=np.asarray(inds, dtype=np.uint32),
        )

    def build_halo_mesh(self) -> MeshData:
        """
        Builds the titanium FIA Halo safety structure with central pillar at X = 0.0 m
        and width = 0.05 m (<= 0.08 m) plus top curved hoop.
        """
        verts: List[List[float]] = []
        inds: List[int] = []
        sp = self.spec

        # Central vertical/raked titanium pillar at X = 0.0 m, width = 0.05 m
        _append_box(
            verts,
            inds,
            center=(sp.halo_center_x, 0.70, sp.halo_strut_z),
            size=(sp.halo_strut_width, 0.28, 0.06),
        )
        # V-transition gusset at top of central pillar
        _append_box(
            verts,
            inds,
            center=(sp.halo_center_x, sp.halo_hoop_height_y, sp.halo_strut_z + 0.05),
            size=(0.16, 0.05, 0.12),
        )
        # Left and right sweeping titanium hoop tubes
        _append_box(verts, inds, center=(-0.22, sp.halo_hoop_height_y, -0.58), size=(0.055, 0.055, 0.72))
        _append_box(verts, inds, center=(+0.22, sp.halo_hoop_height_y, -0.58), size=(0.055, 0.055, 0.72))
        # Forward hoop crossbar segments
        _append_box(verts, inds, center=(-0.11, sp.halo_hoop_height_y, -0.88), size=(0.22, 0.05, 0.06))
        _append_box(verts, inds, center=(+0.11, sp.halo_hoop_height_y, -0.88), size=(0.22, 0.05, 0.06))

        return MeshData(
            name="halo",
            vertices=np.asarray(verts, dtype=np.float32),
            indices=np.asarray(inds, dtype=np.uint32),
        )

    def build_mirrors_mesh(self) -> Dict[str, MeshData]:
        """
        Builds the symmetric left (-0.48 m) and right (+0.48 m) wing mirror
        housings and reflective glass quads.
        """
        sp = self.spec
        h_verts: List[List[float]] = []
        h_inds: List[int] = []

        # Left mirror housing & stalk
        _append_box(
            h_verts,
            h_inds,
            center=(sp.left_mirror_x, sp.mirror_y, sp.mirror_z),
            size=(sp.mirror_width + 0.02, sp.mirror_height + 0.02, 0.05),
        )
        _append_box(
            h_verts,
            h_inds,
            center=(sp.left_mirror_x + 0.09, sp.mirror_y - 0.05, sp.mirror_z - 0.01),
            size=(0.14, 0.025, 0.03),
        )

        # Right mirror housing & stalk
        _append_box(
            h_verts,
            h_inds,
            center=(sp.right_mirror_x, sp.mirror_y, sp.mirror_z),
            size=(sp.mirror_width + 0.02, sp.mirror_height + 0.02, 0.05),
        )
        _append_box(
            h_verts,
            h_inds,
            center=(sp.right_mirror_x - 0.09, sp.mirror_y - 0.05, sp.mirror_z - 0.01),
            size=(0.14, 0.025, 0.03),
        )

        # Reflective glass quads facing +Z (toward driver eye)
        hw = sp.mirror_width * 0.5
        hh = sp.mirror_height * 0.5
        gz = sp.mirror_z + 0.027

        l_verts: List[List[float]] = []
        l_inds: List[int] = []
        _append_quad(
            l_verts,
            l_inds,
            (sp.left_mirror_x - hw, sp.mirror_y - hh, gz),
            (sp.left_mirror_x + hw, sp.mirror_y - hh, gz),
            (sp.left_mirror_x + hw, sp.mirror_y + hh, gz),
            (sp.left_mirror_x - hw, sp.mirror_y + hh, gz),
            flip_u=True,
        )

        r_verts: List[List[float]] = []
        r_inds: List[int] = []
        _append_quad(
            r_verts,
            r_inds,
            (sp.right_mirror_x - hw, sp.mirror_y - hh, gz),
            (sp.right_mirror_x + hw, sp.mirror_y - hh, gz),
            (sp.right_mirror_x + hw, sp.mirror_y + hh, gz),
            (sp.right_mirror_x - hw, sp.mirror_y + hh, gz),
            flip_u=True,
        )

        return {
            "mirror_housings": MeshData(
                name="mirror_housings",
                vertices=np.asarray(h_verts, dtype=np.float32),
                indices=np.asarray(h_inds, dtype=np.uint32),
            ),
            "left_mirror_glass": MeshData(
                name="left_mirror_glass",
                vertices=np.asarray(l_verts, dtype=np.float32),
                indices=np.asarray(l_inds, dtype=np.uint32),
            ),
            "right_mirror_glass": MeshData(
                name="right_mirror_glass",
                vertices=np.asarray(r_verts, dtype=np.float32),
                indices=np.asarray(r_inds, dtype=np.uint32),
            ),
        }

    def build_steering_wheel_meshes(self) -> Dict[str, MeshData]:
        """
        Builds the local-space F1 butterfly steering wheel body mesh and
        the embedded PCU-8D LCD screen quad centered around (0, 0, 0).
        """
        sp = self.spec
        hw = sp.wheel_width * 0.5   # 0.14 m
        hh = sp.wheel_height * 0.5  # 0.08 m

        w_verts: List[List[float]] = []
        w_inds: List[int] = []

        # Central carbon faceplate housing
        _append_box(w_verts, w_inds, center=(0.0, 0.0, 0.0), size=(0.18, 0.13, 0.032))
        # Top LED brow bezel
        _append_box(w_verts, w_inds, center=(0.0, hh - 0.012, 0.004), size=(0.22, 0.024, 0.036))
        # Left & Right sculpted alcantara butterfly grips
        _append_box(w_verts, w_inds, center=(-hw + 0.022, -0.005, 0.006), size=(0.044, 0.155, 0.038))
        _append_box(w_verts, w_inds, center=(+hw - 0.022, -0.005, 0.006), size=(0.044, 0.155, 0.038))
        # Bottom twin carbon spokes (open bottom butterfly cutout)
        _append_box(w_verts, w_inds, center=(-0.065, -hh + 0.015, 0.0), size=(0.07, 0.026, 0.030))
        _append_box(w_verts, w_inds, center=(+0.065, -hh + 0.015, 0.0), size=(0.07, 0.026, 0.030))
        # Rear carbon shift paddles (left downshift, right upshift)
        _append_box(w_verts, w_inds, center=(-0.105, 0.015, -0.028), size=(0.045, 0.075, 0.008))
        _append_box(w_verts, w_inds, center=(+0.105, 0.015, -0.028), size=(0.045, 0.075, 0.008))

        # Embedded PCU-8D LCD quad on the front face (+Z = +0.0175 m)
        lcd_hw = 0.072  # 144 mm wide telemetry display
        lcd_hh = 0.045  # 90 mm tall telemetry display
        lcd_z = 0.0175
        lcd_verts: List[List[float]] = []
        lcd_inds: List[int] = []
        _append_quad(
            lcd_verts,
            lcd_inds,
            (-lcd_hw, -lcd_hh + 0.01, lcd_z),
            (+lcd_hw, -lcd_hh + 0.01, lcd_z),
            (+lcd_hw, +lcd_hh + 0.01, lcd_z),
            (-lcd_hw, +lcd_hh + 0.01, lcd_z),
            flip_u=False,
        )

        return {
            "wheel_body": MeshData(
                name="wheel_body",
                vertices=np.asarray(w_verts, dtype=np.float32),
                indices=np.asarray(w_inds, dtype=np.uint32),
            ),
            "wheel_lcd": MeshData(
                name="wheel_lcd",
                vertices=np.asarray(lcd_verts, dtype=np.float32),
                indices=np.asarray(lcd_inds, dtype=np.uint32),
            ),
        }

    def build_ai_car_mesh(self) -> MeshData:
        """
        Builds a 3D opponent Formula 1 car mesh in track-local coordinates
        (+X = right/lat, +Y = forward/tan, +Z = up/norm).
        """
        verts: List[List[float]] = []
        inds: List[int] = []
        # Main chassis / sidepods / engine cover
        _append_box(verts, inds, center=(0.0, 0.0, 0.36), size=(1.35, 3.80, 0.42))
        _append_box(verts, inds, center=(0.0, -0.35, 0.68), size=(0.52, 1.85, 0.34))
        # Front nosecone & front wing
        _append_box(verts, inds, center=(0.0, 2.25, 0.24), size=(0.36, 1.10, 0.22))
        _append_box(verts, inds, center=(0.0, 2.65, 0.14), size=(1.95, 0.38, 0.10))
        # Rear wing & endplates
        _append_box(verts, inds, center=(0.0, -2.15, 0.82), size=(1.05, 0.34, 0.38))
        # 4 18-inch F1 wheels (FL, FR, RL, RR)
        _append_box(verts, inds, center=(-0.88, +1.55, 0.36), size=(0.38, 0.72, 0.72))
        _append_box(verts, inds, center=(+0.88, +1.55, 0.36), size=(0.38, 0.72, 0.72))
        _append_box(verts, inds, center=(-0.88, -1.55, 0.36), size=(0.42, 0.72, 0.72))
        _append_box(verts, inds, center=(+0.88, -1.55, 0.36), size=(0.42, 0.72, 0.72))
        return MeshData(
            name="ai_car",
            vertices=np.asarray(verts, dtype=np.float32),
            indices=np.asarray(inds, dtype=np.uint32),
        )

    def build_all(self) -> Dict[str, MeshData]:
        """Builds and returns a dictionary of all cockpit and steering wheel meshes."""
        meshes: Dict[str, MeshData] = {
            "monocoque": self.build_monocoque_mesh(),
            "halo": self.build_halo_mesh(),
        }
        meshes.update(self.build_mirrors_mesh())
        meshes.update(self.build_steering_wheel_meshes())
        return meshes
