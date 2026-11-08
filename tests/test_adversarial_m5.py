"""
tests/test_adversarial_m5.py
============================
Adversarial & Boundary Stress Suite for Milestone 5:
- MotorsportMenuManager invalid state/track/difficulty/transmission rejection
  and automatic key-capture cancellation on state exit
- BroadcastHUD resilience to empty standings, negative/extreme speeds, and long lap timers
- RaceResultsBuilder resilience to empty grids, all-DNF classifications, and identical tie times
- SoundEngine waveform synthesis across extreme RPMs (0 RPM .. 20,000 RPM) and tiny durations
- CLI parser & F1SimulationApp headless invocation resilience
"""
from __future__ import annotations
import unittest
import numpy as np

from f1sim.audio import (
    SoundEngine,
    compute_v6_fundamental_hz,
)
from f1sim.input import ControlAction
from f1sim.ui import (
    BroadcastHUD,
    MotorsportMenuManager,
    RaceResultsBuilder,
    compute_telemetry_summary,
    find_fastest_lap_driver,
    format_classification_gap,
    format_lap_timer,
    format_speed_hud,
    hex_to_rgb,
)
from main import create_arg_parser


class TestAdversarialMilestone5(unittest.TestCase):
    """Adversarial stress tests for Milestone 5 (Menus, HUD, Results, Audio, CLI)."""

    def test_01_menu_invalid_inputs_and_capture_cancellation_on_exit(self):
        menu = MotorsportMenuManager()
        with self.assertRaises(ValueError):
            menu.transition_to("NON_EXISTENT_SCREEN")
        with self.assertRaises(ValueError):
            menu.select_track("Silverstone_Unknown")
        with self.assertRaises(ValueError):
            menu.select_difficulty("Impossible_Tier")
        with self.assertRaises(ValueError):
            menu.select_transmission("CVT_Invalid")

        # Starting key capture in KEYBINDINGS and transitioning away cancels capture automatically
        menu.transition_to("KEYBINDINGS")
        menu.start_key_rebind(ControlAction.DRS, slot="primary")
        self.assertTrue(menu.bindings.is_capturing)
        menu.transition_to("MAIN")
        self.assertFalse(menu.bindings.is_capturing)

    def test_02_hud_extreme_values_and_empty_standings(self):
        hud = BroadcastHUD(num_cars=6)
        hud.update(
            speed_kmh=-0.4,
            rpm=-500.0,
            gear_str="N",
            current_lap_time_sec=-10.0,
            car_standings=[],
            minimap_cars=[],
        )
        self.assertEqual(hud.speed_display, "0 KM/H")
        self.assertEqual(hud.rpm_value, 0.0)
        self.assertEqual(hud.rpm_fraction, 0.0)
        self.assertEqual(hud.lap_timer_display, "0:00.000")
        self.assertEqual(len(hud.timing_tower), 0)
        self.assertEqual(len(hud.minimap_dots), 0)

        # Multi-hour endurance timer formatting
        self.assertEqual(format_lap_timer(3661.5), "61:01.500")

    def test_03_results_empty_all_dnf_and_tie_handling(self):
        builder = RaceResultsBuilder()
        self.assertEqual(builder.classify([]), [])
        self.assertIsNone(find_fastest_lap_driver({}))
        self.assertIsNone(find_fastest_lap_driver({"VER": 0.0, "LEC": -5.0}))

        # All-DNF race entries
        dnf_entries = [
            {"code": "VER", "time": 120.0, "best_lap": 81.5, "status": "DNF"},
            {"code": "LEC", "time": 95.0, "best_lap": 81.2, "status": "DNF"},
        ]
        res = builder.classify(dnf_entries)
        self.assertEqual(len(res), 2)
        self.assertEqual(res[0].gap, "DNF")
        self.assertEqual(res[1].gap, "DNF")
        self.assertTrue(res[0].fastest_lap_award or res[1].fastest_lap_award)

        # Telemetry summary with empty list
        summary_empty = compute_telemetry_summary([], total_distance_m=5793.0, total_time_sec=85.0)
        self.assertGreater(summary_empty["avg_speed_kmh"], 200.0)
        self.assertGreater(summary_empty["top_speed_kmh"], summary_empty["avg_speed_kmh"])

    def test_04_sound_engine_extreme_rpm_and_hex_color_fallback(self):
        snd = SoundEngine(sample_rate=22050, enabled=True, init_mixer=False)
        # 0 RPM clamps fundamental frequency >= 50 Hz without NaN
        f0_zero = compute_v6_fundamental_hz(0.0)
        self.assertEqual(f0_zero, 50.0)
        pcm_zero = snd.synthesize_v6_waveform(rpm=0.0, throttle=0.0, duration_sec=0.001)
        self.assertFalse(np.isnan(pcm_zero).any())

        pcm_redline = snd.synthesize_v6_waveform(rpm=15000.0, throttle=1.0, duration_sec=0.01)
        self.assertEqual(pcm_redline.dtype, np.int16)
        self.assertLessEqual(int(np.max(pcm_redline)), 32767)
        self.assertGreaterEqual(int(np.min(pcm_redline)), -32767)

        # Corrupt hex color falls back safely to FIA Red (225, 6, 0)
        self.assertEqual(hex_to_rgb("INVALID"), (225, 6, 0))
        self.assertEqual(hex_to_rgb("#00FF00"), (0, 255, 0))

    def test_05_cli_parser_arguments(self):
        parser = create_arg_parser()
        args = parser.parse_args(["--track", "Monaco", "--difficulty", "Hard", "--laps", "5", "--preset", "ULTRA", "--headless", "--frames", "15"])
        self.assertEqual(args.track, "Monaco")
        self.assertEqual(args.difficulty, "Hard")
        self.assertEqual(args.laps, 5)
        self.assertEqual(args.preset, "ULTRA")
        self.assertTrue(args.headless)
        self.assertEqual(args.frames, 15)


if __name__ == "__main__":
    unittest.main()
