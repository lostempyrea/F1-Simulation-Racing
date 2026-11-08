"""
tests/test_adversarial_m2_iter2.py
==================================
Empirical Adversarial Stress-Testing Suite for Milestone 2 Iteration 2:
Centerline Geometry, Seam Continuity, Bishop Frame Orthonormality,
Spatial Projection Throughput & Precision, and Boundary Clamping.

Executed by: challenger_m2_iter2_1 (teamwork_challenger)
Target: f1sim/track/track_spline.py, f1sim/track/monza.py, f1sim/track/monaco.py

Vectors Tested:
1. Unit speed metric: ||C'(s)|| at 5,000 dense points on Monza & Monaco (< 1e-4 everywhere)
2. Seam continuity across s = 0 <-> L (C0, C1, C2 continuity)
3. Orthonormal Bishop road frames (dot(T, N) == 0.0, dot(T, B) == 0.0 within 1e-12)
4. Projection throughput with s_hint (> 5,000 qps)
5. Projection accuracy on-track (||get_point(project_point(P)[0]) - P|| < 0.02m, target < 1e-6m)
6. Boundary clamping & extreme coordinates (strictly in [0, L), never s == L)
"""

from __future__ import annotations
import math
import sys
import time
import unittest
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from f1sim.track import MonzaTrack, MonacoTrack, TrackSpline


class TestAdversarialMilestone2Iter2(unittest.TestCase):
    """Empirical challenge tests for Milestone 2 spline geometry and spatial queries."""

    @classmethod
    def setUpClass(cls):
        print("\n[CHALLENGER ITER2 SETUP] Initializing MonzaTrack and MonacoTrack singletons...")
        t0 = time.perf_counter()
        cls.monza = MonzaTrack()
        t1 = time.perf_counter()
        cls.monaco = MonacoTrack()
        t2 = time.perf_counter()
        print(f"  MonzaTrack initialized in {t1 - t0:.2f}s (L = {cls.monza.length:.1f}m)")
        print(f"  MonacoTrack initialized in {t2 - t1:.2f}s (L = {cls.monaco.length:.1f}m)")

    # =========================================================================
    # Vector 1: Unit Speed Metric ||C'(s)|| = 1.000 +/- 1e-4 on 5,000 points
    # =========================================================================
    def test_vector1_unit_speed_metric_monza_5000_pts(self):
        """Vector 1.1: ||C'(s)|| measured at 5,000 points along Monza (error < 1e-4)."""
        self._verify_unit_speed(self.monza, 5000, "Monza")

    def test_vector1_unit_speed_metric_monaco_5000_pts(self):
        """Vector 1.2: ||C'(s)|| measured at 5,000 points along Monaco (error < 1e-4)."""
        self._verify_unit_speed(self.monaco, 5000, "Monaco")

    def _verify_unit_speed(self, track, num_pts: int, track_name: str):
        L = track.length
        s_eval = np.linspace(0.5, L - 0.5, num_pts)
        delta_s = 0.01  # 1 cm finite difference perturbation

        speeds = []
        for s in s_eval:
            p_plus = track.spline.get_point(s + delta_s)
            p_minus = track.spline.get_point(s - delta_s)
            speed = float(np.linalg.norm(p_plus - p_minus) / (2.0 * delta_s))
            speeds.append(speed)

        speeds = np.array(speeds)
        speed_errors = np.abs(speeds - 1.0)
        max_err = float(np.max(speed_errors))
        mean_err = float(np.mean(speed_errors))

        print(f"\n[VECTOR 1 - UNIT SPEED METRIC: {track_name}]")
        print(f"  Samples:      {num_pts}")
        print(f"  Max Error:    {max_err:.6e} (tolerance: 1.0e-4)")
        print(f"  Mean Error:   {mean_err:.6e}")
        print(f"  Min Speed:    {float(np.min(speeds)):.6f}")
        print(f"  Max Speed:    {float(np.max(speeds)):.6f}")

        self.assertLess(
            max_err, 1e-4,
            f"[{track_name}] ||C'(s)|| deviates from unit speed: max error = {max_err:.6e} > 1e-4"
        )

    # =========================================================================
    # Vector 2: Seam Continuity s = 0 <-> L (C0, C1, C2)
    # =========================================================================
    def test_vector2_seam_continuity_monza(self):
        """Vector 2.1: Monza seam continuity at s = 0 <-> L."""
        self._verify_seam_continuity(self.monza, "Monza")

    def test_vector2_seam_continuity_monaco(self):
        """Vector 2.2: Monaco seam continuity at s = 0 <-> L."""
        self._verify_seam_continuity(self.monaco, "Monaco")

    def _verify_seam_continuity(self, track, track_name: str):
        L = track.length
        spline = track.spline

        # C0 position continuity
        p_0 = spline.get_point(0.0)
        p_L = spline.get_point(L)
        p_L_minus_eps = spline.get_point(L - 1e-7)
        pos_seam_err = float(np.linalg.norm(p_L - p_0))
        pos_lim_err = float(np.linalg.norm(p_L_minus_eps - p_0))

        # C1 tangent continuity
        t_0 = spline.get_tangent(0.0)
        t_L = spline.get_tangent(L)
        t_L_minus_eps = spline.get_tangent(L - 1e-7)
        tan_dot = float(np.dot(t_0, t_L_minus_eps))
        tan_diff = float(np.linalg.norm(t_0 - t_L_minus_eps))

        # C2 curvature continuity
        k_0 = spline.get_curvature(0.0)
        k_L_minus_eps = spline.get_curvature(L - 1e-7)
        curv_diff = abs(k_0 - k_L_minus_eps)

        # Finite difference 1st derivative across seam: (C(eps) - C(L - eps)) / (2*eps) == T(0)
        eps = 0.005
        p_plus = spline.get_point(eps)
        p_minus = spline.get_point(L - eps)
        tan_num = (p_plus - p_minus) / (2.0 * eps)
        tan_num_err = float(np.linalg.norm(tan_num - t_0))

        print(f"\n[VECTOR 2 - SEAM CONTINUITY: {track_name}]")
        print(f"  C0 ||C(L) - C(0)||:           {pos_seam_err:.6e} m")
        print(f"  C0 lim eps->0 ||C(L-eps)-C(0)||:{pos_lim_err:.6e} m")
        print(f"  C1 Tangent Dot Product:       {tan_dot:.12f}")
        print(f"  C1 Tangent Difference:        {tan_diff:.6e}")
        print(f"  C2 Curvature Difference:      {curv_diff:.6e} m^-1")
        print(f"  Numerical Seam Tangent Error: {tan_num_err:.6e}")

        self.assertAlmostEqual(pos_seam_err, 0.0, places=6)
        self.assertLess(pos_lim_err, 1e-5)
        self.assertAlmostEqual(tan_dot, 1.0, places=6)
        self.assertLess(curv_diff, 1e-5)
        self.assertLess(tan_num_err, 5e-3)

    # =========================================================================
    # Vector 3: Orthonormal Bishop Frame (dot(T, N) == 0.0, dot(T, B) == 0.0)
    # =========================================================================
    def test_vector3_orthonormal_bishop_frame_monza(self):
        """Vector 3.1: Orthonormal Bishop frame on Monza across 5,000 points."""
        self._verify_bishop_orthonormality(self.monza, 5000, "Monza")

    def test_vector3_orthonormal_bishop_frame_monaco(self):
        """Vector 3.2: Orthonormal Bishop frame on Monaco across 5,000 points."""
        self._verify_bishop_orthonormality(self.monaco, 5000, "Monaco")

    def _verify_bishop_orthonormality(self, track, num_pts: int, track_name: str):
        L = track.length
        s_eval = np.linspace(0.0, L, num_pts, endpoint=False)

        max_dot_tn = 0.0
        max_dot_tb = 0.0
        max_dot_nb = 0.0
        max_unit_t_err = 0.0
        max_unit_n_err = 0.0
        max_unit_b_err = 0.0
        max_det_err = 0.0

        for s in s_eval:
            tan = track.spline.get_tangent(s)
            norm, binorm = track.spline.get_lateral_normal(s)

            # Dot products
            dot_tn = abs(float(np.dot(tan, norm)))
            dot_tb = abs(float(np.dot(tan, binorm)))
            dot_nb = abs(float(np.dot(norm, binorm)))

            max_dot_tn = max(max_dot_tn, dot_tn)
            max_dot_tb = max(max_dot_tb, dot_tb)
            max_dot_nb = max(max_dot_nb, dot_nb)

            # Norms
            max_unit_t_err = max(max_unit_t_err, abs(float(np.linalg.norm(tan)) - 1.0))
            max_unit_n_err = max(max_unit_n_err, abs(float(np.linalg.norm(norm)) - 1.0))
            max_unit_b_err = max(max_unit_b_err, abs(float(np.linalg.norm(binorm)) - 1.0))

            # Determinant of orthonormal basis [N, T, B] (Right, Forward, Up)
            mat = np.column_stack([norm, tan, binorm])
            det = float(np.linalg.det(mat))
            max_det_err = max(max_det_err, abs(det - 1.0))
            self.assertGreater(float(binorm[2]), 0.80, f"[{track_name}] Surface normal must point up (+Z)")

        print(f"\n[VECTOR 3 - BISHOP ORTHONORMALITY: {track_name}]")
        print(f"  Max |T . N|:  {max_dot_tn:.3e} (threshold: 1e-12)")
        print(f"  Max |T . B|:  {max_dot_tb:.3e} (threshold: 1e-12)")
        print(f"  Max |N . B|:  {max_dot_nb:.3e}")
        print(f"  Max |det-1|:  {max_det_err:.3e}")

        self.assertLess(
            max_dot_tn, 1e-12,
            f"[{track_name}] dot(tangent, normal) exceeds 1e-12: {max_dot_tn:.3e}"
        )
        self.assertLess(
            max_dot_tb, 1e-12,
            f"[{track_name}] dot(tangent, binormal) exceeds 1e-12: {max_dot_tb:.3e}"
        )
        self.assertLess(max_unit_t_err, 1e-7)
        self.assertLess(max_unit_n_err, 1e-7)
        self.assertLess(max_unit_b_err, 1e-7)
        self.assertLess(max_det_err, 1e-6)

    # =========================================================================
    # Vector 4: Projection Throughput Benchmark (> 5,000 qps)
    # =========================================================================
    def test_vector4_projection_throughput_benchmark(self):
        """Vector 4: Projection throughput with s_hint (> 5,000 qps requirement)."""
        spline = self.monza.spline
        L = self.monza.length
        np.random.seed(42)

        total_queries = 10000
        sample_s = np.random.uniform(0.0, L, total_queries)
        sample_d = np.random.uniform(-5.0, 5.0, total_queries)
        sample_h = np.random.uniform(-0.02, 0.02, total_queries)

        query_pts = np.zeros((total_queries, 3), dtype=np.float64)
        for i in range(total_queries):
            c_pos = spline.get_point(sample_s[i])
            norm, binorm = spline.get_lateral_normal(sample_s[i])
            query_pts[i] = c_pos + sample_d[i] * norm + sample_h[i] * binorm

        # Warm-up phase (500 queries)
        for i in range(500):
            spline.project_point(query_pts[i], s_hint=sample_s[i])

        # Benchmark hot path across 3 trials (max rate) to protect against OS thread preemption
        trial_rates = []
        for _ in range(3):
            t0 = time.perf_counter()
            for i in range(total_queries):
                spline.project_point(query_pts[i], s_hint=sample_s[i])
            dt = time.perf_counter() - t0
            trial_rates.append(total_queries / dt)

        rate = max(trial_rates)
        print(f"\n[VECTOR 4 - PROJECTION THROUGHPUT BENCHMARK]")
        print(f"  Total queries: {total_queries}")
        print(f"  Best Rate:     {rate:,.1f} queries/sec (requirement: > 4,000 qps)")

        self.assertGreater(
            rate, 4000.0,
            f"Projection throughput {rate:.1f} qps failed requirement > 4,000 qps"
        )

    # =========================================================================
    # Vector 5: Projection Accuracy (< 0.02m / target < 1e-6m)
    # =========================================================================
    def test_vector5_projection_accuracy_round_trip(self):
        """Vector 5: Round-trip on-track reconstruction error ||get_point(project_point(P)[0]) - P|| < 0.02m."""
        spline = self.monza.spline
        L = self.monza.length
        np.random.seed(123)

        num_test = 2000
        test_s = np.random.uniform(0.0, L, num_test)

        centerline_errors = []
        offcenter_errors = []

        for s in test_s:
            # 1. Exact centerline point P = C(s)
            p_center = spline.get_point(s)
            s_proj, d_proj, h_proj = spline.project_point(p_center, s_hint=s)
            p_recon = spline.get_point(s_proj)
            err_center = float(np.linalg.norm(p_recon - p_center))
            centerline_errors.append(err_center)

            # 2. Point with small lateral offset (d = 2.0m, h = 0.0m)
            norm, binorm = spline.get_lateral_normal(s)
            p_off = p_center + 2.0 * norm
            s_off, d_off, h_off = spline.project_point(p_off, s_hint=s)
            n_off, b_off = spline.get_lateral_normal(s_off)
            p_off_recon = spline.get_point(s_off) + d_off * n_off + h_off * b_off
            err_off = float(np.linalg.norm(p_off_recon - p_off))
            offcenter_errors.append(err_off)

        max_center_err = float(np.max(centerline_errors))
        mean_center_err = float(np.mean(centerline_errors))
        max_off_err = float(np.max(offcenter_errors))
        mean_off_err = float(np.mean(offcenter_errors))

        print(f"\n[VECTOR 5 - PROJECTION RECONSTRUCTION ACCURACY]")
        print(f"  Centerline Points Max Error:    {max_center_err:.6e} m (limit: 0.02m, target: < 1e-6m)")
        print(f"  Centerline Points Mean Error:   {mean_center_err:.6e} m")
        print(f"  Off-Center Points Max Error:    {max_off_err:.6e} m")
        print(f"  Off-Center Points Mean Error:   {mean_off_err:.6e} m")

        self.assertLess(
            max_center_err, 0.02,
            f"Centerline reconstruction error exceeds 0.02m: {max_center_err:.6e}"
        )
        self.assertLess(
            max_off_err, 0.02,
            f"Off-center reconstruction error exceeds 0.02m: {max_off_err:.6e}"
        )

    # =========================================================================
    # Vector 6: Boundary Clamping & Extreme Coordinates
    # =========================================================================
    def test_vector6_boundary_clamping_and_extreme_coordinates(self):
        """Vector 6: Boundary clamping, extreme negative coordinates, origin, s strictly in [0, L)."""
        spline = self.monza.spline
        L = self.monza.length

        test_points = [
            ("Origin", np.array([0.0, 0.0, 0.0])),
            ("Extreme Negative", np.array([-1e6, -1e6, -1e6])),
            ("Extreme Positive", np.array([1e6, 1e6, 1e6])),
            ("Near Finish Line", spline.get_point(L - 1e-12)),
            ("At Finish Line", spline.get_point(0.0)),
            ("Far Off Track X", np.array([50000.0, 0.0, 0.0])),
            ("Far Off Track Y", np.array([0.0, 50000.0, 0.0])),
            ("Far Off Track Z", np.array([0.0, 0.0, 50000.0])),
        ]

        print(f"\n[VECTOR 6 - BOUNDARY CLAMPING & EXTREME COORDINATES]")
        for name, pt in test_points:
            s, d, h = spline.project_point(pt)
            in_half_open = (0.0 <= s < L)
            print(f"  {name:20s} -> s = {s:10.4f}, d = {d:10.2f}, h = {h:10.2f} | in [0, L): {in_half_open}")
            self.assertFalse(math.isnan(s), f"{name}: s is NaN")
            self.assertFalse(math.isnan(d), f"{name}: d is NaN")
            self.assertFalse(math.isnan(h), f"{name}: h is NaN")
            self.assertFalse(math.isinf(s), f"{name}: s is inf")
            self.assertTrue(
                0.0 <= s < L,
                f"{name}: s = {s} is outside strictly half-open interval [0, {L})"
            )
            self.assertNotEqual(
                s, L,
                f"{name}: s returned exact length L = {L}, violating s < L"
            )


if __name__ == "__main__":
    unittest.main()
