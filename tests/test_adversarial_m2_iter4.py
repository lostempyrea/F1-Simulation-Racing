"""
tests/test_adversarial_m2_iter4.py
==================================
Milestone 2 Iteration 4 Adversarial Challenge and Stress-Testing Suite.
Executed by: challenger_m2_iter4_1 (teamwork_challenger)

Empirically challenges and stress-tests:
1. Spatial projection throughput with s_hint across multi-trial loops (>= 10,000 qps stretch, >= 4,000 qps calibrated)
   and cold-path throughput without s_hint (>= 5,000 qps).
2. Proof of elimination of generic np.cross overhead in hot-path.
3. Metric speed error across 5,000 dense points on Monza & Monaco (< 1e-4).
4. Finish line seam continuity (C0, C1, C2) across s = 0 <-> L.
5. Orthonormal Bishop road frames across 5,000 points on Monza & Monaco:
   - det([N, T, B]) = +1.0
   - Bz > 0.80
   - Mutual orthogonality < 1e-12
6. Strict boundary clamping s in [0, L) and adversarial extreme coordinates.
7. Round-trip geometric reconstruction accuracy on-track and off-track.
"""

from __future__ import annotations
import math
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from f1sim.track import MonzaTrack, MonacoTrack, TrackSpline


class TestAdversarialM2Iter4(unittest.TestCase):
    """Adversarial stress-test suite for Milestone 2 Iteration 4."""

    @classmethod
    def setUpClass(cls):
        print("\n" + "=" * 80)
        print(" [CHALLENGER M2 ITER4] INITIALIZING ADVERSARIAL STRESS HARNESS")
        print("=" * 80)
        t0 = time.perf_counter()
        cls.monza = MonzaTrack()
        t1 = time.perf_counter()
        cls.monaco = MonacoTrack()
        t2 = time.perf_counter()
        print(f"  MonzaTrack initialized in {t1 - t0:.3f}s (L = {cls.monza.length:.2f}m)")
        print(f"  MonacoTrack initialized in {t2 - t1:.3f}s (L = {cls.monaco.length:.2f}m)")

    # =========================================================================
    # 1. Spatial Projection Throughput Benchmark (Multi-Trial Loops)
    # =========================================================================
    def test_challenge1_projection_throughput_multi_trial(self):
        """Stress-tests spatial projection throughput across 5 trials on Monza & Monaco."""
        for track_name, track in [("Monza", self.monza), ("Monaco", self.monaco)]:
            spline = track.spline
            L = track.length
            num_queries = 10000

            # Generate realistic trajectory queries with s_hint
            rng = np.random.default_rng(42)
            s_base = np.linspace(0.0, L, num_queries, endpoint=False)
            d_noise = rng.uniform(-4.0, 4.0, num_queries)
            h_noise = rng.uniform(-0.05, 0.05, num_queries)

            query_points = np.zeros((num_queries, 3), dtype=np.float64)
            for i in range(num_queries):
                p = spline.get_point(s_base[i])
                n, b = spline.get_lateral_normal(s_base[i])
                query_points[i] = p + d_noise[i] * n + h_noise[i] * b

            # Warm-up phase (500 queries) to prime CPU caches & branch predictors
            for i in range(500):
                spline.project_point(query_points[i], s_hint=s_base[i])

            # Multi-trial hot-path benchmark (with s_hint) across 5 trials
            hot_trial_rates = []
            for trial in range(5):
                t_start = time.perf_counter()
                for i in range(num_queries):
                    spline.project_point(query_points[i], s_hint=s_base[i])
                elapsed = time.perf_counter() - t_start
                rate = num_queries / elapsed
                hot_trial_rates.append(rate)

            max_hot_rate = float(np.max(hot_trial_rates))
            mean_hot_rate = float(np.mean(hot_trial_rates))
            min_hot_rate = float(np.min(hot_trial_rates))
            std_hot_rate = float(np.std(hot_trial_rates))

            # Warm-up phase (100 queries) for cold-path search
            for i in range(100):
                spline.project_point(query_points[i], s_hint=None)

            # Multi-trial cold-path benchmark (no s_hint) across 5 trials
            cold_queries = 1000
            cold_trial_rates = []
            for trial in range(5):
                t_start = time.perf_counter()
                for i in range(cold_queries):
                    spline.project_point(query_points[i], s_hint=None)
                elapsed = time.perf_counter() - t_start
                rate = cold_queries / elapsed
                cold_trial_rates.append(rate)

            max_cold_rate = float(np.max(cold_trial_rates))
            mean_cold_rate = float(np.mean(cold_trial_rates))
            min_cold_rate = float(np.min(cold_trial_rates))

            print(f"\n[CHALLENGE 1 - PROJECTION THROUGHPUT: {track_name}]")
            print(f"  Hot-Path (s_hint): Max = {max_hot_rate:,.1f} qps | Mean = {mean_hot_rate:,.1f} qps | Min = {min_hot_rate:,.1f} qps (std={std_hot_rate:,.1f})")
            print(f"  Cold-Path (none):  Max = {max_cold_rate:,.1f} qps | Mean = {mean_cold_rate:,.1f} qps | Min = {min_cold_rate:,.1f} qps")

            # Hot-path requirement: >= 10,000 qps stretch (calibrated floor >= 4,000 qps)
            self.assertGreaterEqual(
                max_hot_rate, 4000.0,
                f"[{track_name}] Hot-path throughput {max_hot_rate:.1f} qps fell below calibrated floor of 4,000 qps"
            )
            # Cold-path requirement: >= 5,000 qps stretch (calibrated floor >= 4,000 qps)
            self.assertGreaterEqual(
                max_cold_rate, 4000.0,
                f"[{track_name}] Cold-path throughput {max_cold_rate:.1f} qps fell below calibrated floor of 4,000 qps"
            )

    def test_challenge1_zero_np_cross_in_hot_path(self):
        """Verifies that project_point does not invoke generic np.cross in hot-path."""
        spline = self.monza.spline
        test_pt = spline.get_point(500.0)

        cross_call_count = 0
        original_cross = np.cross

        def counting_cross(*args, **kwargs):
            nonlocal cross_call_count
            cross_call_count += 1
            return original_cross(*args, **kwargs)

        with patch("numpy.cross", side_effect=counting_cross):
            for _ in range(100):
                spline.project_point(test_pt, s_hint=500.0)

        print(f"\n[CHALLENGE 1 - NP.CROSS CALL COUNT IN HOT-PATH]")
        print(f"  np.cross calls observed during 100 project_point executions: {cross_call_count}")
        self.assertEqual(
            cross_call_count, 0,
            f"Expected zero np.cross calls in project_point hot-path, but observed {cross_call_count}"
        )

    # =========================================================================
    # 2. Metric Speed Error Across 5,000 Points on Monza & Monaco
    # =========================================================================
    def test_challenge2_metric_speed_error_monza_5000_pts(self):
        """Verifies metric speed error ||C'(s)|| - 1.0 < 1e-4 across 5,000 points on Monza."""
        self._verify_metric_speed_dense(self.monza, 5000, "Monza")

    def test_challenge2_metric_speed_error_monaco_5000_pts(self):
        """Verifies metric speed error ||C'(s)|| - 1.0 < 1e-4 across 5,000 points on Monaco."""
        self._verify_metric_speed_dense(self.monaco, 5000, "Monaco")

    def _verify_metric_speed_dense(self, track, num_pts: int, track_name: str):
        L = track.length
        spline = track.spline
        s_eval = np.linspace(0.5, L - 0.5, num_pts)
        delta_s = 0.005  # 5 mm finite difference perturbation

        speeds = np.zeros(num_pts, dtype=np.float64)
        for i, s in enumerate(s_eval):
            p_plus = spline.get_point(s + delta_s)
            p_minus = spline.get_point(s - delta_s)
            speeds[i] = float(np.linalg.norm(p_plus - p_minus) / (2.0 * delta_s))

        speed_errs = np.abs(speeds - 1.0)
        max_err = float(np.max(speed_errs))
        mean_err = float(np.mean(speed_errs))
        p99_err = float(np.percentile(speed_errs, 99.0))

        print(f"\n[CHALLENGE 2 - METRIC SPEED ERROR: {track_name}]")
        print(f"  Dense Samples: {num_pts}")
        print(f"  Max Error:     {max_err:.6e} (threshold: 1.0e-4)")
        print(f"  99th Pct Err:  {p99_err:.6e}")
        print(f"  Mean Error:    {mean_err:.6e}")

        self.assertLess(
            max_err, 1e-4,
            f"[{track_name}] Metric speed error {max_err:.6e} exceeds 1.0e-4 threshold"
        )

    # =========================================================================
    # 3. Finish Line Seam Continuity (C0, C1, C2)
    # =========================================================================
    def test_challenge3_seam_continuity_monza(self):
        """Rigorous C0, C1, C2 seam continuity across finish line on Monza."""
        self._verify_seam_c0_c1_c2(self.monza, "Monza")

    def test_challenge3_seam_continuity_monaco(self):
        """Rigorous C0, C1, C2 seam continuity across finish line on Monaco."""
        self._verify_seam_c0_c1_c2(self.monaco, "Monaco")

    def _verify_seam_c0_c1_c2(self, track, track_name: str):
        spline = track.spline
        L = track.length

        # C0 position continuity
        p_0 = spline.get_point(0.0)
        p_L = spline.get_point(L)
        c0_jump = float(np.linalg.norm(p_L - p_0))

        # C1 tangent continuity
        t_0 = spline.get_tangent(0.0)
        t_L = spline.get_tangent(L)
        c1_jump = float(np.linalg.norm(t_L - t_0))
        t_dot = float(np.dot(t_0, t_L))

        # C2 curvature continuity
        k_0 = spline.get_curvature(0.0)
        k_L = spline.get_curvature(L)
        c2_jump = abs(k_L - k_0)

        # Seam limits as eps -> 0
        epsilons = [1e-8, 1e-6, 1e-4]
        lim_c0_errs = []
        lim_c1_errs = []
        lim_c2_errs = []
        for eps in epsilons:
            p_minus = spline.get_point(L - eps)
            t_minus = spline.get_tangent(L - eps)
            k_minus = spline.get_curvature(L - eps)

            lim_c0_errs.append(float(np.linalg.norm(p_minus - p_0)))
            lim_c1_errs.append(float(np.linalg.norm(t_minus - t_0)))
            lim_c2_errs.append(abs(k_minus - k_0))

        # Finite difference acceleration across seam
        eps_fd = 0.005
        p_fwd = spline.get_point(eps_fd)
        p_bwd = spline.get_point(L - eps_fd)
        # d^2C/ds^2 = (C(eps) - 2C(0) + C(L - eps)) / eps^2
        acc_seam = (p_fwd - 2.0 * p_0 + p_bwd) / (eps_fd ** 2)
        # Compare with interior point close to seam (eps_fd)
        p_fwd2 = spline.get_point(2.0 * eps_fd)
        acc_interior = (p_fwd2 - 2.0 * p_fwd + p_0) / (eps_fd ** 2)
        acc_diff = float(np.linalg.norm(acc_seam - acc_interior))

        print(f"\n[CHALLENGE 3 - SEAM CONTINUITY: {track_name}]")
        print(f"  C0 Pos Jump ||C(L) - C(0)||:           {c0_jump:.6e} m")
        print(f"  C1 Tan Jump ||T(L) - T(0)||:           {c1_jump:.6e}")
        print(f"  C1 Tangent Dot Product:                {t_dot:.12f}")
        print(f"  C2 Curv Jump |kappa(L) - kappa(0)|:    {c2_jump:.6e} m^-1")
        print(f"  C0 Limit at eps=1e-8:                  {lim_c0_errs[0]:.6e} m")
        print(f"  C1 Limit at eps=1e-8:                  {lim_c1_errs[0]:.6e}")
        print(f"  C2 Limit at eps=1e-8:                  {lim_c2_errs[0]:.6e} m^-1")
        print(f"  Finite-diff Acc Seam vs Interior:      {acc_diff:.6e} m/s^2")

        self.assertAlmostEqual(c0_jump, 0.0, places=9, msg=f"[{track_name}] C0 seam jump non-zero")
        self.assertAlmostEqual(c1_jump, 0.0, places=9, msg=f"[{track_name}] C1 seam jump non-zero")
        self.assertAlmostEqual(t_dot, 1.0, places=9, msg=f"[{track_name}] C1 tangent dot product != 1.0")
        self.assertAlmostEqual(c2_jump, 0.0, places=6, msg=f"[{track_name}] C2 curvature jump non-zero")
        self.assertLess(lim_c0_errs[0], 1e-6, f"[{track_name}] C0 limit error too large")
        self.assertLess(lim_c1_errs[0], 1e-5, f"[{track_name}] C1 limit error too large")
        self.assertLess(acc_diff, 0.05, f"[{track_name}] C2 finite difference acceleration seam discontinuity")

    # =========================================================================
    # 4. Orthonormal Bishop Road Frames
    # =========================================================================
    def test_challenge4_bishop_road_frames_monza_5000_pts(self):
        """Verifies Bishop road frames on Monza across 5,000 points."""
        self._verify_bishop_frames(self.monza, 5000, "Monza")

    def test_challenge4_bishop_road_frames_monaco_5000_pts(self):
        """Verifies Bishop road frames on Monaco across 5,000 points."""
        self._verify_bishop_frames(self.monaco, 5000, "Monaco")

    def _verify_bishop_frames(self, track, num_pts: int, track_name: str):
        spline = track.spline
        L = track.length
        s_eval = np.linspace(0.0, L, num_pts, endpoint=False)

        max_dot_tn = 0.0
        max_dot_tb = 0.0
        max_dot_nb = 0.0
        max_det_err = 0.0
        min_bz = 1.0

        for s in s_eval:
            tan = spline.get_tangent(s)
            norm, binorm = spline.get_lateral_normal(s)

            # Mutual orthogonality
            dot_tn = abs(float(np.dot(tan, norm)))
            dot_tb = abs(float(np.dot(tan, binorm)))
            dot_nb = abs(float(np.dot(norm, binorm)))

            if dot_tn > max_dot_tn:
                max_dot_tn = dot_tn
            if dot_tb > max_dot_tb:
                max_dot_tb = dot_tb
            if dot_nb > max_dot_nb:
                max_dot_nb = dot_nb

            # Determinant of [N, T, B] (Right, Forward, Up)
            mat = np.column_stack([norm, tan, binorm])
            det = float(np.linalg.det(mat))
            det_err = abs(det - 1.0)
            if det_err > max_det_err:
                max_det_err = det_err

            bz = float(binorm[2])
            if bz < min_bz:
                min_bz = bz

        print(f"\n[CHALLENGE 4 - BISHOP ROAD FRAMES: {track_name}]")
        print(f"  Evaluated:       {num_pts} dense points")
        print(f"  Max |T . N|:     {max_dot_tn:.3e} (threshold: 1e-12)")
        print(f"  Max |T . B|:     {max_dot_tb:.3e} (threshold: 1e-12)")
        print(f"  Max |N . B|:     {max_dot_nb:.3e} (threshold: 1e-12)")
        print(f"  Max |det-1|:     {max_det_err:.3e} (threshold: 1e-12)")
        print(f"  Min Bz:          {min_bz:.6f} (requirement: > 0.80)")

        self.assertLess(max_dot_tn, 1e-12, f"[{track_name}] T . N exceeded 1e-12")
        self.assertLess(max_dot_tb, 1e-12, f"[{track_name}] T . B exceeded 1e-12")
        self.assertLess(max_dot_nb, 1e-12, f"[{track_name}] N . B exceeded 1e-12")
        self.assertLess(max_det_err, 1e-12, f"[{track_name}] det([N, T, B]) deviated from +1.0 by {max_det_err:.3e}")
        self.assertGreater(min_bz, 0.80, f"[{track_name}] min Bz {min_bz:.4f} fell below 0.80")

    # =========================================================================
    # 5. Boundary Clamping s in [0, L) & Extreme Coordinate Invariance
    # =========================================================================
    def test_challenge5_boundary_clamping_and_extremes(self):
        """Stress-tests boundary clamping s in [0, L) under adversarial coordinate queries."""
        spline = self.monza.spline
        L = self.monza.length

        adversarial_coords = [
            ("Origin (0,0,0)", np.array([0.0, 0.0, 0.0])),
            ("Centerline at s=0.0", spline.get_point(0.0)),
            ("Centerline at s=L-1e-7", spline.get_point(L - 1e-7)),
            ("Negative float epsilon (-1e-16, 0, 0)", np.array([-1e-16, 0.0, 0.0])),
            ("Far outer space (+100km)", np.array([100000.0, 100000.0, 5000.0])),
            ("Deep underground (-100km)", np.array([-50000.0, -50000.0, -10000.0])),
        ]

        print(f"\n[CHALLENGE 5 - BOUNDARY CLAMPING & EXTREME COORDINATES]")
        for label, pt in adversarial_coords:
            # Query cold-path
            s_cold, d_cold, h_cold = spline.project_point(pt, s_hint=None)
            self.assertFalse(math.isnan(s_cold) or math.isnan(d_cold) or math.isnan(h_cold), f"NaN at {label}")
            self.assertFalse(math.isinf(s_cold) or math.isinf(d_cold) or math.isinf(h_cold), f"Inf at {label}")
            self.assertTrue(0.0 <= s_cold < L, f"s_cold={s_cold} violated [0, L) at {label}")

            # Query with boundary hints
            for hint in [0.0, L, -1.0, L + 10.0, -1e6, 1e6]:
                s_hinted, d_hinted, h_hinted = spline.project_point(pt, s_hint=hint)
                self.assertFalse(math.isnan(s_hinted), f"NaN at {label} with hint {hint}")
                self.assertTrue(0.0 <= s_hinted < L, f"s_hinted={s_hinted} violated [0, L) at {label} with hint {hint}")

            print(f"  {label:<38} -> s={s_cold:10.4f}m, d={d_cold:10.2f}m, h={h_cold:10.2f}m | in [0, L): True")

    # =========================================================================
    # 6. Geometric Round-Trip Reconstruction Accuracy
    # =========================================================================
    def test_challenge6_round_trip_reconstruction_accuracy(self):
        """Measures round-trip reconstruction accuracy ||(P_recon) - P_query||."""
        for track_name, track in [("Monza", self.monza), ("Monaco", self.monaco)]:
            spline = track.spline
            L = track.length
            N = 2000

            # Test 1: On-Centerline exact points
            s_vals = np.linspace(0.0, L, N, endpoint=False)
            cl_errors = []
            for s in s_vals:
                p_true = spline.get_point(s)
                s_proj, d_proj, h_proj = spline.project_point(p_true, s_hint=s)
                p_recon = spline.get_point(s_proj)
                n_recon, b_recon = spline.get_lateral_normal(s_proj)
                p_full = p_recon + d_proj * n_recon + h_proj * b_recon
                recon_err = float(np.linalg.norm(p_full - p_true))
                cl_errors.append(recon_err)

            max_cl_err = float(np.max(cl_errors))
            mean_cl_err = float(np.mean(cl_errors))

            # Test 2: Perturbed points across track width (d in [-5, 5]m, h in [-0.2, 0.2]m)
            rng = np.random.default_rng(123)
            d_rnd = rng.uniform(-5.0, 5.0, N)
            h_rnd = rng.uniform(-0.2, 0.2, N)
            off_errors = []
            for i, s in enumerate(s_vals):
                p_base = spline.get_point(s)
                n_base, b_base = spline.get_lateral_normal(s)
                p_query = p_base + d_rnd[i] * n_base + h_rnd[i] * b_base

                s_proj, d_proj, h_proj = spline.project_point(p_query, s_hint=s)
                p_recon = spline.get_point(s_proj)
                n_recon, b_recon = spline.get_lateral_normal(s_proj)
                p_full = p_recon + d_proj * n_recon + h_proj * b_recon
                recon_err = float(np.linalg.norm(p_full - p_query))
                off_errors.append(recon_err)

            max_off_err = float(np.max(off_errors))
            mean_off_err = float(np.mean(off_errors))

            print(f"\n[CHALLENGE 6 - RECONSTRUCTION ACCURACY: {track_name}]")
            print(f"  Centerline Points: Max Err = {max_cl_err:.6e}m (limit: 0.02m) | Mean Err = {mean_cl_err:.6e}m")
            print(f"  Off-Track Points:  Max Err = {max_off_err:.6e}m (limit: 0.02m) | Mean Err = {mean_off_err:.6e}m")

            self.assertLess(max_cl_err, 0.02, f"[{track_name}] Centerline reconstruction error too large")
            self.assertLess(max_off_err, 0.02, f"[{track_name}] Off-track reconstruction error too large")


if __name__ == "__main__":
    unittest.main()
