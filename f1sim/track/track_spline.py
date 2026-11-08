"""
f1sim/track/track_spline.py
===========================
High-Performance C2-Continuous Closed Periodic Cubic Spline Geometry
with Orthonormal Bishop Road Frames, Gauss-Legendre Arc-Length Inversion,
and Fast Spatial Projections in Pure NumPy. Zero dependency on SciPy.
"""

from __future__ import annotations
import math
from typing import Tuple, Optional, Callable, Sequence
import numpy as np


class PeriodicCubicSpline3D:
    """
    Pure NumPy implementation of C2-continuous closed periodic cubic spline in R^3.
    Interpolates N survey waypoints with strictly continuous first and second
    derivatives across the entire circuit and through the s = 0 <-> L finish-line seam.
    """

    def __init__(self, waypoints: np.ndarray) -> None:
        """
        Args:
            waypoints: Array of shape (N, 3) representing closed circuit path.
        """
        pts = np.asarray(waypoints, dtype=np.float64).copy()
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"Waypoints must have shape (N, 3), got {pts.shape}")

        # 1. Filter out consecutive duplicate waypoints where ||P_{i+1} - P_i|| < 1e-4
        if len(pts) > 1:
            diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            mask = np.ones(len(pts), dtype=bool)
            mask[1:] = diffs >= 1e-4
            pts = pts[mask]

        # 2. Remove duplicate closing point(s) if user provided P_{last} == P_0
        while len(pts) > 1 and np.linalg.norm(pts[-1] - pts[0]) < 1e-4:
            pts = pts[:-1]

        if pts.shape[0] < 4:
            raise ValueError(f"At least 4 unique waypoints required for closed spline, got {pts.shape[0]}")

        self.N: int = pts.shape[0]
        self.P: np.ndarray = pts  # Shape (N, 3)

        # 3. Chordal knot intervals h_i = ||P_{i+1} - P_i|| (strictly positive)
        P_next = np.roll(pts, -1, axis=0)
        diffs = P_next - pts
        self.h: np.ndarray = np.linalg.norm(diffs, axis=1)
        self.h = np.maximum(self.h, 1e-4)  # Defensive safeguard

        # Cumulative knot parameter u_i
        self.u_knots: np.ndarray = np.zeros(self.N + 1, dtype=np.float64)
        self.u_knots[1:] = np.cumsum(self.h)
        self.U: float = float(self.u_knots[-1])

        # Solve cyclic tridiagonal system for second derivatives M_i = C''(u_i)
        # mu_i * M_{i-1} + 2 * M_i + lambda_i * M_{i+1} = D_i
        h_prev = np.roll(self.h, 1)
        denom = h_prev + self.h
        mu = h_prev / denom
        lam = self.h / denom

        P_prev = np.roll(pts, 1, axis=0)
        v_curr = (P_next - pts) / self.h[:, np.newaxis]
        v_prev = (pts - P_prev) / h_prev[:, np.newaxis]
        D = (6.0 / denom[:, np.newaxis]) * (v_curr - v_prev)

        # Construct cyclic tridiagonal matrix A of size (N, N)
        A = np.zeros((self.N, self.N), dtype=np.float64)
        for i in range(self.N):
            A[i, (i - 1) % self.N] = mu[i]
            A[i, i] = 2.0
            A[i, (i + 1) % self.N] = lam[i]

        # Solve A * M = D
        self.M: np.ndarray = np.linalg.solve(A, D)  # Shape (N, 3)

        # Precompute segment polynomial basis vectors for ultra-fast C(u) and C'(u) evaluation
        h_col = self.h[:, np.newaxis]
        M_next = np.roll(self.M, -1, axis=0)
        P_next = np.roll(self.P, -1, axis=0)

        self._A_pos: np.ndarray = self.M / (6.0 * h_col)
        self._B_pos: np.ndarray = M_next / (6.0 * h_col)
        self._C_pos: np.ndarray = (self.P - (h_col ** 2 / 6.0) * self.M) / h_col
        self._D_pos: np.ndarray = (P_next - (h_col ** 2 / 6.0) * M_next) / h_col

        self._A_vel: np.ndarray = - self.M / (2.0 * h_col)
        self._B_vel: np.ndarray = M_next / (2.0 * h_col)
        self._C_vel: np.ndarray = (P_next - self.P) / h_col - (h_col / 6.0) * (M_next - self.M)

    def _find_segment(self, u: float) -> Tuple[int, float, float, float]:
        """Returns segment index i and normalized relative parameter offsets."""
        u_norm = u % self.U
        idx = int(np.searchsorted(self.u_knots, u_norm, side='right') - 1)
        idx = min(max(0, idx), self.N - 1)

        u_i = self.u_knots[idx]
        u_next = self.u_knots[idx + 1]
        h_i = self.h[idx]

        du0 = u_norm - u_i
        du1 = u_next - u_norm
        return idx, du0, du1, h_i

    def evaluate_pos(self, u: float) -> np.ndarray:
        """Returns 3D position C(u) = (x, y, z)."""
        i, du0, du1, _ = self._find_segment(u)
        pos = (
            (du1 ** 3) * self._A_pos[i]
            + (du0 ** 3) * self._B_pos[i]
            + du1 * self._C_pos[i]
            + du0 * self._D_pos[i]
        )
        return pos

    def evaluate_vel(self, u: float) -> np.ndarray:
        """Returns first derivative C'(u) = dC/du."""
        i, du0, du1, _ = self._find_segment(u)
        vel = (
            (du1 ** 2) * self._A_vel[i]
            + (du0 ** 2) * self._B_vel[i]
            + self._C_vel[i]
        )
        return vel

    def evaluate_acc(self, u: float) -> np.ndarray:
        """Returns second derivative C''(u) = d^2C/du^2."""
        i, du0, du1, h = self._find_segment(u)
        next_i = (i + 1) % self.N

        M_i = self.M[i]
        M_next = self.M[next_i]
        acc = (du1 / h) * M_i + (du0 / h) * M_next
        return acc

    def evaluate(self, u: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Returns position C(u), velocity C'(u), and acceleration C''(u)."""
        i, du0, du1, h = self._find_segment(u)
        next_i = (i + 1) % self.N

        pos = (
            (du1 ** 3) * self._A_pos[i]
            + (du0 ** 3) * self._B_pos[i]
            + du1 * self._C_pos[i]
            + du0 * self._D_pos[i]
        )
        vel = (
            (du1 ** 2) * self._A_vel[i]
            + (du0 ** 2) * self._B_vel[i]
            + self._C_vel[i]
        )
        acc = (du1 / h) * self.M[i] + (du0 / h) * self.M[next_i]
        return pos, vel, acc

    def evaluate_pos_vel(self, u: float) -> Tuple[np.ndarray, np.ndarray]:
        """High-speed dual evaluation returning position C(u) and velocity C'(u)."""
        i, du0, du1, _ = self._find_segment(u)
        pos = (
            (du1 ** 3) * self._A_pos[i]
            + (du0 ** 3) * self._B_pos[i]
            + du1 * self._C_pos[i]
            + du0 * self._D_pos[i]
        )
        vel = (
            (du1 ** 2) * self._A_vel[i]
            + (du0 ** 2) * self._B_vel[i]
            + self._C_vel[i]
        )
        return pos, vel

    def get_curvature(self, u: float) -> float:
        """Returns exact analytical curvature kappa(u) = ||C' x C''|| / ||C'||^3."""
        _, vel, acc = self.evaluate(u)
        v_norm = float(np.linalg.norm(vel))
        if v_norm < 1e-9:
            return 0.0
        cross = np.cross(vel, acc)
        return float(np.linalg.norm(cross)) / (v_norm ** 3)


class TrackSpline:
    """
    Master Track Centerline Spline Geometry Engine.
    Provides true arc-length parameterization s in [0, L), orthonormal Bishop
    road frames, banking angle, variable track width, and high-speed spatial projection.
    """

    # Gauss-Legendre 5-point quadrature weights and nodes (roots of P_5(x) = 0)
    _GL_NODES, _GL_WEIGHTS = np.polynomial.legendre.leggauss(5)

    def __init__(
        self,
        waypoints: np.ndarray,
        target_length: Optional[float] = None,
        nominal_width: float = 11.0,
        width_profile: Optional[Callable[[float], Tuple[float, float]]] = None,
        banking_profile: Optional[Callable[[float], float]] = None,
        resolution: float = 0.25
    ) -> None:
        """
        Constructs and calibrates a continuous 3D TrackSpline.

        Args:
            waypoints: Array of shape (N, 3) representing circuit path.
            target_length: Official FIA length to calibrate to (e.g. 5793.0m Monza, 3337.0m Monaco).
            nominal_width: Default tarmac width in meters (default 11.0m).
            width_profile: Optional function s -> (w_left, w_right).
            banking_profile: Optional function s -> banking_angle_radians.
            resolution: Discretization step size for fast lookup table (meters).
        """
        pts = np.asarray(waypoints, dtype=np.float64).copy()
        # Pre-filter waypoints defensively
        if len(pts) > 1:
            diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            mask = np.ones(len(pts), dtype=bool)
            mask[1:] = diffs >= 1e-4
            pts = pts[mask]
        while len(pts) > 1 and np.linalg.norm(pts[-1] - pts[0]) < 1e-4:
            pts = pts[:-1]

        self.nominal_width: float = float(nominal_width)
        self.width_profile: Optional[Callable[[float], Tuple[float, float]]] = width_profile
        self.banking_profile: Optional[Callable[[float], float]] = banking_profile

        # Iterative calibration to match exact target length if requested
        if target_length is not None:
            target_L = float(target_length)
            for _ in range(6):
                spline_temp = PeriodicCubicSpline3D(pts)
                raw_L = self._compute_total_arc_length(spline_temp)
                if abs(raw_L - target_L) < 1e-6:
                    break
                scale = target_L / raw_L
                pts[:, 0] *= scale
                pts[:, 1] *= scale
                if pts.shape[1] == 3 and np.ptp(pts[:, 2]) < 1.0:
                    pts[:, 2] *= scale

        self.spline: PeriodicCubicSpline3D = PeriodicCubicSpline3D(pts)
        self.length: float = float(target_length) if target_length is not None else self._compute_total_arc_length(self.spline)

        # Build dense arc-length parameterization tables
        self._build_arc_length_tables(resolution=resolution)

    def _compute_total_arc_length(self, spline: PeriodicCubicSpline3D) -> float:
        """Computes total circuit length via sub-stepped 5-point Gauss-Legendre quadrature."""
        total_s = 0.0
        for i in range(spline.N):
            u_i = spline.u_knots[i]
            h_i = spline.h[i]
            n_sub = max(1, int(math.ceil(h_i / 1.0)))
            du = h_i / n_sub
            half_du = 0.5 * du
            for j in range(n_sub):
                u_mid = u_i + (j + 0.5) * du
                quad_sum = 0.0
                for k in range(5):
                    u_eval = u_mid + half_du * self._GL_NODES[k]
                    vel = spline.evaluate_vel(u_eval)
                    speed = float(np.linalg.norm(vel))
                    quad_sum += self._GL_WEIGHTS[k] * speed
                total_s += half_du * quad_sum
        return float(total_s)

    def _build_arc_length_tables(self, resolution: float = 0.25) -> None:
        """Builds sub-segment Cubic Hermite tables mapping arc length s <-> chord parameter u."""
        spline = self.spline
        N = spline.N
        target_L = self.length

        # 1. High-precision sub-segment discretization (sub_step <= 0.25m)
        sub_step = 0.25
        sub_u = []
        sub_s = [0.0]
        for i in range(N):
            u_i = spline.u_knots[i]
            h_i = spline.h[i]
            n_sub = max(4, int(np.ceil(h_i / sub_step)))
            du = h_i / n_sub
            for k in range(n_sub):
                uk = u_i + k * du
                sub_u.append(uk)
                um = uk + 0.5 * du
                v_evals = [np.linalg.norm(spline.evaluate_vel(um + 0.5 * du * x)) for x in self._GL_NODES]
                ds = 0.5 * du * np.sum(self._GL_WEIGHTS * np.array(v_evals))
                sub_s.append(sub_s[-1] + ds)
        sub_u.append(spline.U)
        self.sub_u: np.ndarray = np.array(sub_u, dtype=np.float64)
        raw_total_s = float(sub_s[-1])
        scale_factor = target_L / raw_total_s
        self.sub_s: np.ndarray = np.array(sub_s, dtype=np.float64) * scale_factor

        # 2. Precompute derivatives for Cubic Hermite: m_k = du/ds = (1 / ||C'(u_k)||) / scale_factor
        self.sub_m: np.ndarray = np.zeros_like(self.sub_u)
        for k in range(len(self.sub_u)):
            vel = spline.evaluate_vel(self.sub_u[k])
            v_norm = float(np.linalg.norm(vel))
            self.sub_m[k] = (1.0 / max(1e-9, v_norm)) / scale_factor

        # 3. Dense uniform grid of s samples for fast window spatial projection
        num_samples = max(100, int(round(self.length / resolution)))
        self.table_s: np.ndarray = np.linspace(0.0, self.length, num_samples, endpoint=False)
        self.table_ds: float = float(self.table_s[1] - self.table_s[0])

        self.table_pos: np.ndarray = np.zeros((num_samples, 3), dtype=np.float64)
        self.table_tangent: np.ndarray = np.zeros((num_samples, 3), dtype=np.float64)
        self.table_normal: np.ndarray = np.zeros((num_samples, 3), dtype=np.float64)
        self.table_binormal: np.ndarray = np.zeros((num_samples, 3), dtype=np.float64)
        self.table_curvature: np.ndarray = np.zeros(num_samples, dtype=np.float64)
        self.table_u: np.ndarray = np.zeros(num_samples, dtype=np.float64)
        self.table_cdp: np.ndarray = np.zeros((num_samples, 3), dtype=np.float64)

        for k in range(num_samples):
            s_k = self.table_s[k]
            u_k = self.s_to_u(s_k)
            self.table_u[k] = u_k
            pos, vel, acc = spline.evaluate(u_k)
            v_sq = float(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2])
            v_norm = math.sqrt(max(1e-9, v_sq))
            tan = vel / v_norm

            # Bishop frame: Up vector (0, 0, 1)
            up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            if abs(float(tan[0]*up[0] + tan[1]*up[1] + tan[2]*up[2])) > 0.99:
                up = np.array([0.0, 1.0, 0.0], dtype=np.float64)

            # Normal pointing right across track: T x Up
            norm_base = np.cross(tan, up)
            norm_base /= max(1e-9, float(np.linalg.norm(norm_base)))

            # Surface normal pointing upwards (+Z): B = N x T
            binorm_base = np.array([
                norm_base[1] * tan[2] - norm_base[2] * tan[1],
                norm_base[2] * tan[0] - norm_base[0] * tan[2],
                norm_base[0] * tan[1] - norm_base[1] * tan[0]
            ], dtype=np.float64)
            binorm_base /= max(1e-9, float(np.linalg.norm(binorm_base)))

            # Banking
            bank = self.get_banking(s_k)
            if abs(bank) > 1e-5:
                cos_b, sin_b = math.cos(bank), math.sin(bank)
                norm = cos_b * norm_base + sin_b * binorm_base
                binorm = -sin_b * norm_base + cos_b * binorm_base
            else:
                norm = norm_base
                binorm = binorm_base

            # Arc-length second derivative C''(s) for O(1) projection refinement
            vel_dot_acc = float(vel[0]*acc[0] + vel[1]*acc[1] + vel[2]*acc[2])
            c_double_prime = (acc - (vel_dot_acc / max(1e-9, v_sq)) * vel) / max(1e-9, v_sq)

            curv = float(np.linalg.norm(np.cross(vel, acc))) / max(1e-9, v_sq * v_norm)

            self.table_pos[k] = pos
            self.table_tangent[k] = tan
            self.table_normal[k] = norm
            self.table_binormal[k] = binorm
            self.table_curvature[k] = curv
            self.table_cdp[k] = c_double_prime

        # Precomputed contiguous coarse positions for ultra-fast 2-stage cold-path projection (stride 16)
        self.table_coarse_pos: np.ndarray = self.table_pos[::16].copy()

    def s_to_u(self, s: float) -> float:
        """Converts arc length distance s into spline parameter u via Cubic Hermite inversion."""
        s_norm = s % self.length
        idx = int(np.searchsorted(self.sub_s, s_norm, side='right') - 1)
        idx = min(max(0, idx), len(self.sub_s) - 2)

        s0 = self.sub_s[idx]
        s1 = self.sub_s[idx + 1]
        h = s1 - s0
        t = (s_norm - s0) / h

        u0 = self.sub_u[idx]
        u1 = self.sub_u[idx + 1]
        m0 = self.sub_m[idx]
        m1 = self.sub_m[idx + 1]

        h00 = (1.0 + 2.0 * t) * ((1.0 - t) ** 2)
        h10 = t * ((1.0 - t) ** 2)
        h01 = (t ** 2) * (3.0 - 2.0 * t)
        h11 = (t ** 2) * (t - 1.0)

        u_interp = h00 * u0 + h10 * h * m0 + h01 * u1 + h11 * h * m1
        return float(u_interp % self.spline.U)

    def get_point(self, s: float) -> np.ndarray:
        """Returns exact 3D centerline coordinates (x, y, z) on C2 continuous spline at distance s."""
        u = self.s_to_u(s)
        return self.spline.evaluate_pos(u)

    def get_tangent(self, s: float) -> np.ndarray:
        """Returns normalized forward unit tangent vector T(s)."""
        u = self.s_to_u(s)
        vel = self.spline.evaluate_vel(u)
        v_norm = float(np.linalg.norm(vel))
        return vel / max(1e-9, v_norm)

    def get_normal(self, s: float) -> np.ndarray:
        """Returns normalized horizontal lateral right unit vector N(s)."""
        s_norm = s % self.length
        idx = int(s_norm / self.table_ds)
        idx = min(max(0, idx), len(self.table_s) - 1)
        next_idx = (idx + 1) % len(self.table_s)

        frac = (s_norm - self.table_s[idx]) / self.table_ds
        n0 = self.table_normal[idx]
        n1 = self.table_normal[next_idx]
        n = n0 + frac * (n1 - n0)
        norm = float(np.linalg.norm(n))
        return n / max(1e-9, norm)

    def get_binormal(self, s: float) -> np.ndarray:
        """Returns normalized vertical surface normal unit vector B(s)."""
        s_norm = s % self.length
        idx = int(s_norm / self.table_ds)
        idx = min(max(0, idx), len(self.table_s) - 1)
        next_idx = (idx + 1) % len(self.table_s)

        frac = (s_norm - self.table_s[idx]) / self.table_ds
        b0 = self.table_binormal[idx]
        b1 = self.table_binormal[next_idx]
        b = b0 + frac * (b1 - b0)
        norm = float(np.linalg.norm(b))
        return b / max(1e-9, norm)

    def get_lateral_normal(self, s: float) -> Tuple[np.ndarray, np.ndarray]:
        """Returns tuple (lateral_normal_N, surface_normal_B) with banking applied (B = N x T pointing +Z)."""
        tan = self.get_tangent(s)
        norm = self.get_normal(s)
        # Gram-Schmidt orthonormality: N = N - (N . T) * T
        norm = norm - float(norm[0]*tan[0] + norm[1]*tan[1] + norm[2]*tan[2]) * tan
        norm_len = math.sqrt(max(1e-9, float(norm[0]*norm[0] + norm[1]*norm[1] + norm[2]*norm[2])))
        norm /= norm_len
        # B = N x T (pointing upwards +Z)
        binorm = np.array([
            norm[1] * tan[2] - norm[2] * tan[1],
            norm[2] * tan[0] - norm[0] * tan[2],
            norm[0] * tan[1] - norm[1] * tan[0]
        ], dtype=np.float64)
        binorm_len = math.sqrt(max(1e-9, float(binorm[0]*binorm[0] + binorm[1]*binorm[1] + binorm[2]*binorm[2])))
        binorm /= binorm_len
        return norm, binorm

    def get_spline_point(self, s: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Fulfills PROJECT.md Track interface contract:
        Returns (position_xyz, tangent_vec, normal_vec) at distance s.
        Both tangent and normal are guaranteed unit length and strictly orthogonal.
        """
        tan = self.get_tangent(s)
        norm = self.get_normal(s)
        # Gram-Schmidt orthogonalization for guaranteed dot(tan, norm) == 0.0
        norm = norm - float(np.dot(norm, tan)) * tan
        norm /= max(1e-9, float(np.linalg.norm(norm)))
        pos = self.get_point(s)
        return pos, tan, norm

    def get_curvature(self, s: float) -> float:
        """Returns instantaneous curvature kappa(s) = 1/R in 1/meters."""
        u = self.s_to_u(s)
        return self.spline.get_curvature(u)

    def get_heading(self, s: float) -> float:
        """Returns planar heading angle psi(s) in radians in [-pi, pi]."""
        tan = self.get_tangent(s)
        return math.atan2(float(tan[1]), float(tan[0]))

    def get_banking(self, s: float) -> float:
        """Returns superelevation banking angle theta(s) in radians."""
        if self.banking_profile is not None:
            return float(self.banking_profile(s % self.length))
        return 0.0

    def get_width(self, s: float) -> Tuple[float, float]:
        """Returns half-widths (w_left, w_right) in meters at distance s."""
        if self.width_profile is not None:
            wl, wr = self.width_profile(s % self.length)
            return float(wl), float(wr)
        half_w = 0.5 * self.nominal_width
        return half_w, half_w

    def get_total_width(self, s: float) -> float:
        """Returns total tarmac width at distance s in meters."""
        wl, wr = self.get_width(s)
        return wl + wr

    def project_point(
        self,
        position: np.ndarray,
        s_hint: Optional[float] = None
    ) -> Tuple[float, float, float]:
        """
        High-speed spatial projection from world (x, y, z) to track coordinates (s, d, h).
        Optimized with O(1) table precomputations and inline scalar 3D cross products.

        Returns:
            s: Arc-length distance along centerline [0, L)
            d: Lateral signed displacement from centerline (+ right, - left) in meters
            h: Height above track surface in meters (+ above, - below)
        """
        pos = np.asarray(position, dtype=np.float64)
        if pos.shape[0] < 3:
            px, py, pz = float(pos[0]), float(pos[1]), 0.0
        else:
            px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])

        num_tbl = len(self.table_s)
        # Fast contiguous slice search around s_hint or full table search
        if s_hint is not None:
            hint_norm = s_hint % self.length
            k_hint = int(hint_norm / self.table_ds)
            w = 25  # +/- 6.25m window at 0.25m resolution (+/- 12.5m at 0.50m)
            if w <= k_hint < num_tbl - w:
                diffs = self.table_pos[k_hint - w : k_hint + w + 1] - [px, py, pz]
                best_k = k_hint - w + int(np.argmin(diffs[:, 0]**2 + diffs[:, 1]**2 + diffs[:, 2]**2))
            else:
                k_indices = [(k_hint + offset) % num_tbl for offset in range(-w, w + 1)]
                diffs = self.table_pos[k_indices] - [px, py, pz]
                best_k = k_indices[int(np.argmin(diffs[:, 0]**2 + diffs[:, 1]**2 + diffs[:, 2]**2))]
        else:
            # 2-stage coarse-to-fine search: stride 16 coarse argmin followed by +/- 32 fine window
            stride = 16
            coarse_pos = getattr(self, 'table_coarse_pos', None)
            if coarse_pos is None:
                coarse_pos = self.table_pos[::stride]
            diffs_c = coarse_pos - [px, py, pz]
            c_argmin = int(np.argmin(diffs_c[:, 0]**2 + diffs_c[:, 1]**2 + diffs_c[:, 2]**2))
            best_coarse = c_argmin * stride
            w_fine = 32
            if w_fine <= best_coarse < num_tbl - w_fine:
                diffs_f = self.table_pos[best_coarse - w_fine : best_coarse + w_fine + 1] - [px, py, pz]
                best_k = best_coarse - w_fine + int(np.argmin(diffs_f[:, 0]**2 + diffs_f[:, 1]**2 + diffs_f[:, 2]**2))
            else:
                k_indices = [(best_coarse + offset) % num_tbl for offset in range(-w_fine, w_fine + 1)]
                diffs_f = self.table_pos[k_indices] - [px, py, pz]
                best_k = k_indices[int(np.argmin(diffs_f[:, 0]**2 + diffs_f[:, 1]**2 + diffs_f[:, 2]**2))]

        s_est = float(self.table_s[best_k])

        # O(1) table precomputations: eliminates redundant evaluate() and s_to_u() calls
        c_pos = self.table_pos[best_k]
        tan = self.table_tangent[best_k]
        cdp = self.table_cdp[best_k]

        diff0 = c_pos[0] - px
        diff1 = c_pos[1] - py
        diff2 = c_pos[2] - pz

        f_prime = diff0 * tan[0] + diff1 * tan[1] + diff2 * tan[2]
        diff_dot_cdp = diff0 * cdp[0] + diff1 * cdp[1] + diff2 * cdp[2]
        denom = max(0.1, 1.0 + diff_dot_cdp)

        ds = - f_prime / denom
        s_refined = (s_est + ds) % self.length

        # Strict boundary clamping: guaranteed 0.0 <= s < L (DEAD_ENDS #21 compliance)
        if s_refined >= self.length or s_refined < 0.0:
            s_refined = 0.0

        # Single high-speed evaluation at s_refined for sub-millimeter precision
        u_ref = self.s_to_u(s_refined)
        if hasattr(self.spline, 'evaluate_pos_vel'):
            final_pos, vel_ref = self.spline.evaluate_pos_vel(u_ref)
        else:
            final_pos, vel_ref, _ = self.spline.evaluate(u_ref)

        v_ref_sq = float(vel_ref[0]*vel_ref[0] + vel_ref[1]*vel_ref[1] + vel_ref[2]*vel_ref[2])
        v_ref_norm = math.sqrt(max(1e-9, v_ref_sq))
        t0 = vel_ref[0] / v_ref_norm
        t1 = vel_ref[1] / v_ref_norm
        t2 = vel_ref[2] / v_ref_norm

        # Gram-Schmidt lateral normal (N)
        norm = self.get_normal(s_refined)
        norm_dot_t = norm[0]*t0 + norm[1]*t1 + norm[2]*t2
        n0 = norm[0] - norm_dot_t * t0
        n1 = norm[1] - norm_dot_t * t1
        n2 = norm[2] - norm_dot_t * t2
        n_len = math.sqrt(max(1e-9, n0*n0 + n1*n1 + n2*n2))
        n0 /= n_len
        n1 /= n_len
        n2 /= n_len

        # Scalar cross product: B = N x T (pointing +Z, eliminating np.cross overhead)
        b0 = n1 * t2 - n2 * t1
        b1 = n2 * t0 - n0 * t2
        b2 = n0 * t1 - n1 * t0

        # Scalar displacement d (lateral) and height h (vertical)
        dx = px - final_pos[0]
        dy = py - final_pos[1]
        dz = pz - final_pos[2]

        d = float(dx * n0 + dy * n1 + dz * n2)
        h = float(dx * b0 + dy * b1 + dz * b2)
        return s_refined, d, h

    def project_to_frenet(
        self,
        position: np.ndarray,
        s_hint: Optional[float] = None
    ) -> Tuple[float, float, float]:
        """Alias for project_point."""
        return self.project_point(position, s_hint)
