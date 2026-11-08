"""
tests/run_all_tests.py
======================
Master Test Runner for F1 Desktop Native Simulation Game.

Executes all verification suites sequentially:
- Headless Physics & AI Benchmark (14 automated benchmarks)
- Milestone 1 Core & Dynamics Suite (30 physics unit tests)
- Milestone 1 Adversarial Suite (16 adversarial stress tests)
- Milestone 1 Empirical Challenger (10 empirical challenger tests)
- Milestone 2 Track Geometry Suite (24 track geometry & 3D tests)
- Milestone 2 Adversarial Suite (10 adversarial stress & precision tests)
- Milestone 2 Empirical Stress & Dynamics (15 empirical stress tests)
- Milestone 3 Cockpit, Input & Settings (9 tests)
- Milestone 3 Adversarial Suite (6 tests)
- Milestone 4 AI Racing System (40 tests)
- Milestone 4 Adversarial Suite (18 tests)
- Tier 1: Feature Coverage Suite (26 Features, >= 130 assertions)
- Tier 2: Boundary Value Analysis & Corner Cases (22 tests)
- Tier 3: Pairwise Combinations & Cross-Feature Interactions (8 tests)
- Tier 4: Real-World Race Workload Scenarios (5 scenarios)

Returns exit code 0 on complete pass, non-zero on any active failure.
"""

import sys
import os
import time
import argparse
import unittest
from typing import Dict, Any, List, Tuple


def create_parser() -> argparse.ArgumentParser:
    """Creates the CLI argument parser for master test runner."""
    parser = argparse.ArgumentParser(description="F1 Simulation Master E2E Test Runner")
    parser.add_argument("--tier", choices=["1", "2", "3", "4", "all"], default="all",
                        help="Select test tier to execute (default: all)")
    parser.add_argument("--benchmark", action="store_true",
                        help="Run only the headless physics & AI benchmark")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Enable detailed per-test verbose output")
    return parser


def parse_args(args: list[str] | None = None) -> argparse.Namespace:
    """Parses command-line arguments."""
    return create_parser().parse_args(args)


def run_benchmark_suite(verbose: bool = True) -> Tuple[bool, int, int, int, int, float]:
    """Execute the headless physics and AI benchmark suite."""
    from tests.benchmark_physics_ai import TestBenchmarkPhysicsAI
    t0 = time.perf_counter()
    suite = unittest.TestLoader().loadTestsFromTestCase(TestBenchmarkPhysicsAI)
    runner = unittest.TextTestRunner(verbosity=2 if verbose else 1)
    result = runner.run(suite)
    duration = time.perf_counter() - t0

    passed = result.wasSuccessful()
    tests_run = result.testsRun
    failures = len(result.failures) + len(result.errors)
    skipped = len(result.skipped)
    passed_count = tests_run - failures - skipped
    return passed, tests_run, passed_count, skipped, failures, duration


def run_tier_suite(test_module_name: str, verbose: bool = True) -> Tuple[bool, int, int, int, int, float]:
    """Execute a unittest module by name and report statistics."""
    t0 = time.perf_counter()
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromName(test_module_name)
    runner = unittest.TextTestRunner(verbosity=2 if verbose else 1)
    result = runner.run(suite)
    duration = time.perf_counter() - t0

    passed = result.wasSuccessful()
    tests_run = result.testsRun
    failures = len(result.failures) + len(result.errors)
    skipped = len(result.skipped)
    passed_count = tests_run - failures - skipped
    return passed, tests_run, passed_count, skipped, failures, duration


def main():
    args = parse_args()

    # Ensure repository root is on sys.path
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

    print("=" * 90)
    print(" FORMULA 1 DESKTOP NATIVE SIMULATION - MASTER E2E TEST RUNNER")
    print("=" * 90)
    print(f" Environment: Python {sys.version.split()[0]} 64-bit | Platform: {sys.platform}")
    print(f" Root Path:   {repo_root}")
    print("-" * 90)

    suites_to_run = []
    if args.benchmark:
        suites_to_run.append(("Headless Benchmark (14 Benchmarks)", "tests.benchmark_physics_ai", "benchmark"))
    else:
        # Default all or selected tiers
        suites_to_run.append(("Headless Benchmark (14 Benchmarks)", "tests.benchmark_physics_ai", "benchmark"))
        suites_to_run.append(("Milestone 1 Core & Dynamics", "tests.test_milestone1_physics", "tier"))
        suites_to_run.append(("Milestone 1 Adversarial Suite", "tests.test_adversarial_m1", "tier"))
        suites_to_run.append(("Milestone 1 Empirical Challenger", "tests.test_empirical_challenger", "tier"))
        suites_to_run.append(("Milestone 2 Track Geometry", "tests.test_milestone2_tracks", "tier"))
        suites_to_run.append(("Milestone 2 Adversarial Suite", "tests.test_adversarial_m2_iter4", "tier"))
        suites_to_run.append(("Milestone 2 Empirical Stress", "tests.challenge_m2_empirical_stress", "tier"))
        suites_to_run.append(("Milestone 3 Cockpit, Input & Settings", "tests.test_milestone3_cockpit_input", "tier"))
        suites_to_run.append(("Milestone 3 Adversarial Suite", "tests.test_adversarial_m3", "tier"))
        suites_to_run.append(("Milestone 4 AI Racing System", "tests.test_milestone4_ai", "tier"))
        suites_to_run.append(("Milestone 4 Adversarial Suite", "tests.test_adversarial_m4", "tier"))
        suites_to_run.append(("Milestone 5 UI, HUD, Audio & App", "tests.test_milestone5_ui_audio_app", "tier"))
        suites_to_run.append(("Milestone 5 Adversarial Suite", "tests.test_adversarial_m5", "tier"))
        if args.tier in ["1", "all"]:
            suites_to_run.append(("Tier 1: Feature Coverage (26 Features)", "tests.test_tier1_features", "tier"))
        if args.tier in ["2", "all"]:
            suites_to_run.append(("Tier 2: Boundary Value Analysis", "tests.test_tier2_boundaries", "tier"))
        if args.tier in ["3", "all"]:
            suites_to_run.append(("Tier 3: Pairwise Combinations", "tests.test_tier3_pairwise", "tier"))
        if args.tier in ["4", "all"]:
            suites_to_run.append(("Tier 4: Real-World Scenarios", "tests.test_tier4_scenarios", "tier"))

    overall_passed = True
    total_tests = 0
    total_passed_count = 0
    total_skipped_count = 0
    total_failures = 0
    total_duration = 0.0
    summary_records = []

    for label, mod_name, suite_type in suites_to_run:
        print(f"\n>>> Executing {label} [{mod_name}] ...")
        if suite_type == "benchmark":
            passed, n_run, n_pass, n_skip, n_fail, dur = run_benchmark_suite(verbose=args.verbose)
        else:
            passed, n_run, n_pass, n_skip, n_fail, dur = run_tier_suite(mod_name, verbose=args.verbose)

        total_tests += n_run
        total_passed_count += n_pass
        total_skipped_count += n_skip
        total_failures += n_fail
        total_duration += dur
        if not passed:
            overall_passed = False

        summary_records.append({
            "label": label,
            "status": "PASS" if passed else "FAIL",
            "tests": n_run,
            "passed": n_pass,
            "skipped": n_skip,
            "failed": n_fail,
            "duration": dur
        })

    # Print Final Summary Table
    print("\n" + "=" * 90)
    print(f" {'TEST SUITE SUMMARY':<40} | {'STATUS':<6} | {'TOTAL':<5} | {'PASS':<5} | {'SKIP':<5} | {'FAIL':<5} | {'TIME (s)':<8}")
    print("-" * 90)
    for rec in summary_records:
        print(f" {rec['label']:<40} | {rec['status']:<6} | {rec['tests']:<5} | {rec['passed']:<5} | {rec['skipped']:<5} | {rec['failed']:<5} | {rec['duration']:<8.3f}")
    print("=" * 90)
    print(f" Total Tests Run:       {total_tests}")
    print(f" Total Tests Passed:    {total_passed_count}")
    print(f" Total Tests Skipped:   {total_skipped_count} (Planned for M3-M5)")
    print(f" Total Failures/Errors: {total_failures}")
    print(f" Total Elapsed Time:    {total_duration:.3f} seconds")
    print(f" Final Test Verdict:    {'OVERALL PASS [SUCCESS]' if overall_passed else 'OVERALL FAIL [DEFECTS DETECTED]'}")
    print("=" * 90)

    sys.exit(0 if overall_passed else 1)


if __name__ == "__main__":
    main()
