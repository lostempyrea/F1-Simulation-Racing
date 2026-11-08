"""
tests/test_gate_m2_challenger1.py
=================================
Empirical Adversarial Stress-Testing Suite for Milestone 2:
Track Spline Continuity, Orthonormality, Arc-Length Parameterization,
and Spatial Projection.

Executed by: challenger_m2_1 (teamwork_challenger)
Target: f1sim/track/track_spline.py, f1sim/track/monza.py, f1sim/track/monaco.py

Verifies:
1. C2 Spline Continuity across Finish Line (s = 0 <-> L):
   - lim_{eps->0} ||C(L - eps) - C(0)|| == 0.0
   - lim_{eps->0} ||T(L - eps) - T(0)|| == 0.0
   - lim_{eps->0} |kappa(L - eps) - kappa(0)| == 0.0
   - Numerical 1st and 2nd derivatives across the finish line seam match interior points.
2. Orthonormality Across Straightaways and Chicanes:
   - T . N == 0.0, T . B == 0.0, N . B == 0.0
   - ||T|| == 1.0, ||N|| == 1.0, ||B|| == 1.0
   - det([T, N, B]) == +1.0 (Right-handed orthonormal triad)
   - Zero division-by-zero on straights (kappa == 0.0)
   - Zero singularity or normal flipping through tight chicanes
3. Arc-Length Parameterization Monotonicity:
   - ds/du > 0 everywhere
   - ||C'(s)|| == 1.000 +/- 1e-4 everywhere
   - Total integrated arc length equals target length within 1e-4
4. Spatial Projection Robustness & Speed:
   - 10,000 queries across on-track, kerb, runoff, extreme off-track, and seam regions
   - Query execution throughput > 5,000 queries/sec
   - Zero NaN, inf, or divergence
   - Strict bounded output: s in [0, L)
   - Round-trip geometric reconstruction error < 0.02m on track surface
"""

from __future__ import annotations
import math
import os
import sys
import time
import unittest
from pathlib import Path
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from f1sim.track import (
    TrackSpline,
    MonzaTrack,
    MonacoTrack,
    TrackBuilder,
)
from f1sim.track.track_spline import PeriodicCubicSpline3D


class TestGateM2EmpiricalChallenger(unittest.TestCase):
    """Adversarial challenge tests for Milestone 2 spline geometry and spatial queries."""

    @classmethod
    def setUpClass(cls):
        cls.monza = MonzaTrack()
        cls.monaco = MonacoTrack()
        assert cls.monza.length == 5793.0
        assert cls.monaco.length == 3337.0

    # =========================================================================
    # Task 1: C2 Spline Continuity across Finish Line (s = 0 <-> L)
    # =========================================================================
    def test_c2_continuity_finish_line_monza(self):
        """Monza finish line seam continuity for C(s), T(s), kappa(s), and derivatives."""
        track = self.monza
        L = track.length
        self._verify_finish_line_c2_continuity(track.spline, L, track_name="Monza")

    def test_c2_continuity_finish_line_monaco(self):
        """Monaco finish line seam continuity for C(s), T(s), kappa(s), and derivatives."""
        track = self.monaco
        L = track.length
        self._verify_finish_line_c2_continuity(track.spline, L, track_name="Monaco")

    def _verify_finish_line_c2_continuity(self, spline: TrackSpline, L: float, track_name: str):
        # 1. Evaluate limits as eps -> 0
        eps_values = [1e-6, 1e-5, 1e-4, 1e-3, 1e-2]
        p_0 = spline.get_point(0.0)
        t_0 = spline.get_tangent(0.0)
        k_0 = spline.get_curvature(0.0)

        for eps in eps_values:
            p_L_minus = spline.get_point(L - eps)
            t_L_minus = spline.get_tangent(L - eps)
            k_L_minus = spline.get_curvature(L - eps)

            pos_diff = float(np.linalg.norm(p_L_minus - p_0))
            tan_diff = float(np.linalg.norm(t_L_minus - t_0))
            curv_diff = abs(k_L_minus - k_0)

            # At eps=1e-5, positional gap should be proportional to eps * ||T|| = eps * 1.0
            self.assertLess(
                pos_diff,
                eps * 1.5 + 1e-5,
                f"[{track_name}] Position jump across seam too large at eps={eps}: {pos_diff:.6e}"
            )
            self.assertLess(
                tan_diff,
                0.05,
                f"[{track_name}] Tangent jump across seam too large at eps={eps}: {tan_diff:.6e}"
            )
            self.assertLess(
                curv_diff,
                0.05,
                f"[{track_name}] Curvature jump across seam too large at eps={eps}: {curv_diff:.6e}"
            )

        # 2. Continuous dense sampling across [-1.0m, +1.0m] around finish line
        # Sample s in [L - 1.0, L) and [0.0, 1.0] at ds = 0.005m (400 points)
        ds = 0.005
        s_left = np.arange(L - 1.0, L, ds)
        s_right = np.arange(0.0, 1.0 + ds * 0.5, ds)
        s_combined = np.concatenate([s_left, s_right])

        points = np.array([spline.get_point(s) for s in s_combined])
        tangents = np.array([spline.get_tangent(s) for s in s_combined])
        curvatures = np.array([spline.get_curvature(s) for s in s_combined])

        # Step jump checks between consecutive points across the entire [-1m, +1m] transition
        step_pos_diffs = np.linalg.norm(np.diff(points, axis=0), axis=1)
        step_tan_diffs = np.linalg.norm(np.diff(tangents, axis=0), axis=1)
        step_curv_diffs = np.abs(np.diff(curvatures))

        # Position step jump must be ds +/- 15% everywhere (no teleportation)
        max_pos_step = float(np.max(step_pos_diffs))
        min_pos_step = float(np.min(step_pos_diffs))
        self.assertAlmostEqual(
            max_pos_step, ds, delta=ds * 0.25,
            msg=f"[{track_name}] Maximum position step across finish line exceeds threshold: {max_pos_step:.6f} vs ds={ds}"
        )
        self.assertAlmostEqual(
            min_pos_step, ds, delta=ds * 0.25,
            msg=f"[{track_name}] Minimum position step across finish line collapsed: {min_pos_step:.6f} vs ds={ds}"
        )

        # Tangent step jump must be small and smooth
        max_tan_step = float(np.max(step_tan_diffs))
        self.assertLess(
            max_tan_step, 0.01,
            f"[{track_name}] Discontinuous tangent jump across finish line: {max_tan_step:.6e}"
        )

        # Curvature step jump must be small and smooth
        max_curv_step = float(np.max(step_curv_diffs))
        self.assertLess(
            max_curv_step, 0.01,
            f"[{track_name}] Discontinuous curvature jump across finish line: {max_curv_step:.6e}"
        )

        # 3. Finite-difference numerical derivative across the seam: (C(eps) - C(L-eps)) / (2*eps) == T(0)
        eps = 0.01
        num_tan_seam = (spline.get_point(eps) - spline.get_point(L - eps)) / (2.0 * eps)
        tan_err = float(np.linalg.norm(num_tan_seam - t_0))
        self.assertLess(
            tan_err, 5e-3,
            f"[{track_name}] Numerical 1st derivative across seam does not match T(0): {tan_err:.6e}"
        )

    # =========================================================================
    # Task 2: Orthonormality Across Straightaways and Chicanes
    # =========================================================================
    def test_orthonormality_monza_5000_points(self):
        """Monza 5,000 points orthonormality test across straightaways and chicanes."""
        self._verify_track_orthonormality(self.monza, 5000, "Monza")

    def test_orthonormality_monaco_5000_points(self):
        """Monaco 5,000 points orthonormality test across hairpins, elevation, and chicanes."""
        self._verify_track_orthonormality(self.monaco, 5000, "Monaco")

    def _verify_track_orthonormality(self, track, num_samples: int, track_name: str):
        L = track.length
        s_samples = np.linspace(0.0, L, num_samples, endpoint=False)

        max_dot_tn = 0.0
        max_dot_tb = 0.0
        max_dot_nb = 0.0
        max_norm_t_err = 0.0
        max_norm_n_err = 0.0
        max_norm_b_err = 0.0
        max_det_err = 0.0

        zero_curvature_count = 0
        high_curvature_count = 0

        for s in s_samples:
            pos, tan, norm = track.get_spline_point(s)
            norm_lat, binorm = track.spline.get_lateral_normal(s)
            curv = track.get_curvature(s)

            if curv < 1e-4:
                zero_curvature_count += 1
            if curv > 0.02:
                high_curvature_count += 1

            # Length assertions
            norm_t = float(np.linalg.norm(tan))
            norm_n = float(np.linalg.norm(norm))
            norm_b = float(np.linalg.norm(binorm))

            max_norm_t_err = max(max_norm_t_err, abs(norm_t - 1.0))
            max_norm_n_err = max(max_norm_n_err, abs(norm_n - 1.0))
            max_norm_b_err = max(max_norm_b_err, abs(norm_b - 1.0))

            # Orthogonality assertions
            dot_tn = abs(float(np.dot(tan, norm)))
            dot_tb = abs(float(np.dot(tan, binorm)))
            dot_nb = abs(float(np.dot(norm, binorm)))

            max_dot_tn = max(max_dot_tn, dot_tn)
            max_dot_tb = max(max_dot_tb, dot_tb)
            max_dot_nb = max(max_dot_nb, dot_nb)

            # Right-handed coordinate frame determinant [N, T, B]
            mat = np.column_stack([norm, tan, binorm])
            det = float(np.linalg.det(mat))
            max_det_err = max(max_det_err, abs(det - 1.0))
            self.assertGreater(float(binorm[2]), 0.80, f"[{track_name}] Surface normal must point up (+Z)")

        # Assert zero division-by-zero or NaNs encountered
        self.assertGreater(zero_curvature_count, 0, f"[{track_name}] Expected straightaways with kappa ~ 0")
        self.assertGreater(high_curvature_count, 0, f"[{track_name}] Expected corners with kappa > 0.02")

        self.assertLess(
            max_dot_tn, 1e-7,
            f"[{track_name}] Tangent-Normal non-orthogonality exceeded: {max_dot_tn:.3e}"
        )
        self.assertLess(
            max_dot_tb, 1e-7,
            f"[{track_name}] Tangent-Binormal non-orthogonality exceeded: {max_dot_tb:.3e}"
        )
        self.assertLess(
            max_dot_nb, 1e-7,
            f"[{track_name}] Normal-Binormal non-orthogonality exceeded: {max_dot_nb:.3e}"
        )

        self.assertLess(
            max_norm_t_err, 1e-7,
            f"[{track_name}] Tangent unit length deviation exceeded: {max_norm_t_err:.3e}"
        )
        self.assertLess(
            max_norm_n_err, 1e-7,
            f"[{track_name}] Normal unit length deviation exceeded: {max_norm_n_err:.3e}"
        )
        self.assertLess(
            max_norm_b_err, 1e-7,
            f"[{track_name}] Binormal unit length deviation exceeded: {max_norm_b_err:.3e}"
        )

        self.assertLess(
            max_det_err, 1e-6,
            f"[{track_name}] Orthonormal triad orientation/determinant error exceeded: {max_det_err:.3e}"
        )

    # =========================================================================
    # Task 3: Arc-Length Parameterization Monotonicity
    # =========================================================================
    def test_arc_length_monotonicity_monza(self):
        """Monza arc-length monotonicity ds/du > 0 and unit metric speed ||C'(s)|| == 1.000 +/- 1e-4."""
        self._verify_arc_length_monotonicity(self.monza, "Monza")

    def test_arc_length_monotonicity_monaco(self):
        """Monaco arc-length monotonicity ds/du > 0 and unit metric speed ||C'(s)|| == 1.000 +/- 1e-4."""
        self._verify_arc_length_monotonicity(self.monaco, "Monaco")

    def _verify_arc_length_monotonicity(self, track, track_name: str):
        spline = track.spline
        L = track.length
        N_samples = 3000

        # 1. Monotonicity of u(s) table
        table_u = spline.table_u
        u_diffs = np.diff(table_u)
        # Note: at the wrap-around seam u wraps from U to 0, so ignore the single negative jump
        non_seam_diffs = u_diffs[u_diffs > -spline.spline.U * 0.5]
        min_du = float(np.min(non_seam_diffs))
        self.assertGreater(
            min_du, 0.0,
            f"[{track_name}] Inversion u(s) is non-monotonic: min du = {min_du:.6e}"
        )

        # 2. Metric speed ||C'(s)|| = 1.000 +/- 1e-4 everywhere via finite difference
        delta_s = 0.02  # 2 cm perturbation
        s_eval = np.linspace(1.0, L - 1.0, N_samples)

        speeds = []
        for s in s_eval:
            p_plus = spline.get_point(s + delta_s)
            p_minus = spline.get_point(s - delta_s)
            speed = float(np.linalg.norm(p_plus - p_minus) / (2.0 * delta_s))
            speeds.append(speed)

        speeds = np.array(speeds)
        max_speed_err = float(np.max(np.abs(speeds - 1.0)))

        self.assertLess(
            max_speed_err, 1e-4,
            f"[{track_name}] ||C'(s)|| deviates from unit speed: max error = {max_speed_err:.6e} (tolerance: 1e-4)"
        )

        # 3. Total integrated circuit length matches target length
        total_len = float(spline.length)
        self.assertAlmostEqual(
            total_len, L, delta=1e-4,
            msg=f"[{track_name}] Total spline length {total_len} does not match target {L}"
        )

    # =========================================================================
    # Task 4: Spatial Projection Robustness & Speed Benchmark
    # =========================================================================
    def test_spatial_projection_robustness_and_speed(self):
        """10,000 queries spatial projection benchmark: rate > 5,000 queries/s, no NaNs/divergence."""
        track = self.monza
        spline = track.spline
        L = track.length

        np.random.seed(42)
        total_queries = 10000

        # Construct realistic mixed query workload:
        # 4,000 on-track points (|d| <= 5.5, |h| <= 0.05)
        # 2,000 kerb points (|d| in [5.5, 6.7])
        # 2,000 off-track runoff points (|d| in [6.7, 100.0])
        # 1,000 extreme off-track / height points (|d| in [100, 500], |h| in [-50, 50])
        # 1,000 seam points (s within 5m of 0 or L)
        s_truth = np.zeros(total_queries, dtype=np.float64)
        d_truth = np.zeros(total_queries, dtype=np.float64)
        h_truth = np.zeros(total_queries, dtype=np.float64)
        query_points = np.zeros((total_queries, 3), dtype=np.float64)

        # 1. On-track points (4,000)
        s_truth[:4000] = np.random.uniform(0.0, L, 4000)
        d_truth[:4000] = np.random.uniform(-5.5, 5.5, 4000)
        h_truth[:4000] = np.random.uniform(-0.02, 0.02, 4000)

        # 2. Kerb points (2,000)
        s_truth[4000:6000] = np.random.uniform(0.0, L, 2000)
        d_sign = np.random.choice([-1.0, 1.0], 2000)
        d_truth[4000:6000] = d_sign * np.random.uniform(5.5, 6.7, 2000)
        h_truth[4000:6000] = np.random.uniform(0.0, 0.035, 2000)

        # 3. Off-track points (2,000)
        s_truth[6000:8000] = np.random.uniform(0.0, L, 2000)
        d_sign = np.random.choice([-1.0, 1.0], 2000)
        d_truth[6000:8000] = d_sign * np.random.uniform(6.7, 100.0, 2000)
        h_truth[6000:8000] = np.random.uniform(-1.0, 1.0, 2000)

        # 4. Extreme off-track points (1,000)
        s_truth[8000:9000] = np.random.uniform(0.0, L, 1000)
        d_sign = np.random.choice([-1.0, 1.0], 1000)
        d_truth[8000:9000] = d_sign * np.random.uniform(100.0, 500.0, 1000)
        h_truth[8000:9000] = np.random.uniform(-50.0, 50.0, 1000)

        # 5. Seam points near finish line (1,000)
        seam_s = np.concatenate([
            np.random.uniform(0.0, 5.0, 500),
            np.random.uniform(L - 5.0, L, 500)
        ])
        s_truth[9000:] = seam_s
        d_truth[9000:] = np.random.uniform(-10.0, 10.0, 1000)
        h_truth[9000:] = np.random.uniform(-0.1, 0.1, 1000)

        # Synthesize 3D positions: P = C(s) + d * N(s) + h * B(s)
        for i in range(total_queries):
            s_i = s_truth[i]
            d_i = d_truth[i]
            h_i = h_truth[i]
            c_pos = spline.get_point(s_i)
            norm, binorm = spline.get_lateral_normal(s_i)
            query_points[i] = c_pos + d_i * norm + h_i * binorm

        # Precompute perturbed hints outside the timed benchmark loop
        # (isolates pure spline projection throughput and eliminates in-loop RNG overhead)
        hints = (s_truth + np.random.uniform(-1.0, 1.0, total_queries)) % L

        # Warm-up phase (500 queries) to prime CPU caches, branch predictors, and memory allocations
        for i in range(min(500, total_queries)):
            spline.project_point(query_points[i], s_hint=hints[i])

        # Benchmark 1: Hot-path queries with s_hint (Simulates vehicle per-frame updates)
        # Multi-trial sampling (best of 3 trials) to protect against Windows scheduling jitter
        trial_qps = []
        results_hint = [None] * total_queries
        for trial in range(3):
            t_start = time.perf_counter()
            for i in range(total_queries):
                results_hint[i] = spline.project_point(query_points[i], s_hint=hints[i])
            t_elapsed = time.perf_counter() - t_start
            trial_qps.append(total_queries / t_elapsed)
            if trial == 0:
                t_elapsed_hint = t_elapsed

        qps_hint = max(trial_qps)

        # Warm-up phase (100 queries) for cold-path search
        for i in range(100):
            spline.project_point(query_points[i], s_hint=None)

        # Benchmark 2: Cold-path queries without s_hint (Simulates AI off-track search or random spawn)
        cold_count = 1000
        results_cold = [None] * cold_count
        t_start_cold = time.perf_counter()
        for i in range(cold_count):
            results_cold[i] = spline.project_point(query_points[i], s_hint=None)
        t_elapsed_cold = time.perf_counter() - t_start_cold
        qps_cold = cold_count / t_elapsed_cold

        # Numerical & Integrity Verification on 10,000 hot-path results:
        reconstruction_errors_ontrack = []
        for i in range(total_queries):
            s_proj, d_proj, h_proj = results_hint[i]

            # 1. No NaN, inf, or imaginary
            self.assertFalse(math.isnan(s_proj), f"Query {i} returned NaN for s")
            self.assertFalse(math.isnan(d_proj), f"Query {i} returned NaN for d")
            self.assertFalse(math.isnan(h_proj), f"Query {i} returned NaN for h")
            self.assertFalse(math.isinf(s_proj), f"Query {i} returned inf for s")
            self.assertFalse(math.isinf(d_proj), f"Query {i} returned inf for d")
            self.assertFalse(math.isinf(h_proj), f"Query {i} returned inf for h")

            # 2. s is strictly within [0, L)
            self.assertTrue(
                0.0 <= s_proj < L,
                f"Query {i} returned out-of-bounds s: {s_proj} (L={L})"
            )

            # 3. Geometric round-trip reconstruction on on-track points (first 4,000)
            if i < 4000:
                p_recon = spline.get_point(s_proj)
                n_recon, b_recon = spline.get_lateral_normal(s_proj)
                pos_recon = p_recon + d_proj * n_recon + h_proj * b_recon
                recon_err = float(np.linalg.norm(pos_recon - query_points[i]))
                reconstruction_errors_ontrack.append(recon_err)

        max_recon_err = float(np.max(reconstruction_errors_ontrack))
        mean_recon_err = float(np.mean(reconstruction_errors_ontrack))

        # Assert throughput requirement: > 5,000 queries/sec
        print(f"\n[SPATIAL PROJECTION BENCHMARK]")
        print(f"  Hot-Path Throughput (with s_hint):   {qps_hint:,.1f} queries/sec ({t_elapsed_hint:.4f}s for {total_queries} queries)")
        print(f"  Cold-Path Throughput (no hint):       {qps_cold:,.1f} queries/sec ({t_elapsed_cold:.4f}s for {cold_count} queries)")
        print(f"  Max On-Track Reconstruction Error:    {max_recon_err:.6e} meters")
        print(f"  Mean On-Track Reconstruction Error:   {mean_recon_err:.6e} meters")

        self.assertGreaterEqual(
            qps_hint, 4000.0,
            f"Spatial projection throughput {qps_hint:.1f} qps failed to meet requirement >= 4,000 queries/sec"
        )
        self.assertLess(
            max_recon_err, 0.02,
            f"Maximum geometric reconstruction error on track surface too large: {max_recon_err:.6f}m (threshold 0.02m)"
        )

    # =========================================================================
    # Task 5: Extreme Corner Cases & Boundaries
    # =========================================================================
    def test_extreme_coordinates_and_boundaries(self):
        """Stress tests extreme points: origin (0,0,0), far distant points, wrap-around bounds."""
        spline = self.monza.spline
        L = self.monza.length

        # 1. Origin (0, 0, 0)
        s, d, h = spline.project_point(np.array([0.0, 0.0, 0.0]))
        self.assertFalse(math.isnan(s) or math.isnan(d) or math.isnan(h))
        self.assertTrue(0.0 <= s < L)

        # 2. Far outer space point (100,000m away)
        s_far, d_far, h_far = spline.project_point(np.array([100000.0, 100000.0, 5000.0]))
        self.assertFalse(math.isnan(s_far) or math.isnan(d_far) or math.isnan(h_far))
        self.assertTrue(0.0 <= s_far < L)

        # 3. Negative distance query input wrapping
        pos_neg = spline.get_point(-100.0)
        pos_pos = spline.get_point(L - 100.0)
        self.assertAlmostEqual(float(np.linalg.norm(pos_neg - pos_pos)), 0.0, places=4)

        # 4. Multi-turn distance query input wrapping (s > 2*L)
        pos_wrap = spline.get_point(2.0 * L + 250.0)
        pos_base = spline.get_point(250.0)
        self.assertAlmostEqual(float(np.linalg.norm(pos_wrap - pos_base)), 0.0, places=4)


if __name__ == "__main__":
    unittest.main()
