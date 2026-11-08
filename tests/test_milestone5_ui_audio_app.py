"""
tests/test_milestone5_ui_audio_app.py
=====================================
Comprehensive Milestone 5 & 6 Verification Suite covering:
1. Motorsport UI & Main Menu State Machine (MAIN, TRACK_SELECT, DIFFICULTY_SELECT,
   GRAPHICS_SETTINGS, KEYBINDINGS, CONTROLS_HELP, RACE, PAUSE, RESULTS)
2. Interactive Graphics Settings UI & Keybinding Remapping UI workflows
3. Broadcast Television HUD (Speedometer, RPM bar, P1-P6 Timing Tower, Lap Timer, Mini-Map)
4. Post-Race Podium & Telemetry Summary Classification (Winner gap, +X.XXXs, Fastest Lap, DNF)
5. Procedural 1.6L V6 Turbo-Hybrid Audio Synthesizer & Sound Cues
6. End-to-End F1SimulationApp (`main.py`) Headless Execution on Monza & Monaco
"""
from __future__ import annotations
import tempfile
from types import SimpleNamespace
import unittest
from pathlib import Path
import numpy as np

from f1sim.audio import (
    SUPPORTED_SOUND_EVENTS,
    SoundEngine,
    compute_v6_fundamental_hz,
    compute_v6_harmonics_hz,
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
from main import F1SimulationApp, create_arg_parser


class TestMilestone5MenuAndSettingsUI(unittest.TestCase):
    """Tests MotorsportMenuManager navigation, Graphics Settings UI, and Keybindings UI."""

    def test_menu_navigation_and_session_selection(self):
        menu = MotorsportMenuManager()
        self.assertEqual(menu.current_state, "MAIN")
        for st in ("TRACK_SELECT", "DIFFICULTY_SELECT", "GRAPHICS_SETTINGS", "KEYBINDINGS", "RACE", "PAUSE", "RESULTS"):
            self.assertEqual(menu.transition_to(st), st)

        self.assertEqual(menu.select_track("monaco"), "Monaco")
        self.assertEqual(menu.select_difficulty("hard"), "Hard")
        self.assertEqual(menu.select_transmission("manual"), "Manual")
        self.assertEqual(menu.session.track_name, "Monaco")
        self.assertEqual(menu.session.difficulty, "Hard")
        self.assertEqual(menu.session.transmission, "Manual")

    def test_graphics_settings_ui_controls_and_keybinding_ui(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            menu = MotorsportMenuManager(config_dir=Path(tmpdir))
            menu.transition_to("GRAPHICS_SETTINGS")
            p1 = menu.cycle_graphics_preset(forward=True)
            self.assertEqual(p1, "ULTRA")
            res = menu.cycle_resolution(forward=True)
            self.assertIsInstance(res, tuple)
            msaa = menu.cycle_msaa()
            self.assertIn(msaa, (0, 2, 4, 8))
            fov = menu.adjust_fov(+2.5)
            self.assertAlmostEqual(fov, 70.5)
            menu.toggle_rear_mirrors()
            menu.toggle_motion_blur()
            menu.cycle_halo_opacity()

            g_rows = menu.get_graphics_menu_rows()
            self.assertEqual(len(g_rows), 12)

            # Keybindings UI screen
            menu.transition_to("KEYBINDINGS")
            k_rows = menu.get_keybinding_menu_rows()
            self.assertEqual(len(k_rows), len(ControlAction))

            swapped = menu.rebind_action_key(ControlAction.THROTTLE, "S")
            self.assertEqual(swapped, ControlAction.BRAKE.value)

            import pygame
            menu.start_key_rebind(ControlAction.DRS, slot="primary")
            self.assertTrue(menu.bindings.is_capturing)
            ev = SimpleNamespace(type=pygame.KEYDOWN, key=pygame.K_l)
            menu.handle_key_capture_event(ev)
            self.assertEqual(menu.bindings.get_key(ControlAction.DRS), "L")

            g_path, b_path = menu.save_all_settings()
            self.assertTrue(g_path.exists())
            self.assertTrue(b_path.exists())


class TestMilestone5HUDResultsAndAudio(unittest.TestCase):
    """Tests BroadcastHUD, RaceResultsBuilder, and SoundEngine."""

    def test_broadcast_hud_timing_tower_and_formatting(self):
        hud = BroadcastHUD(num_cars=6)
        standings = [
            {"code": "VER", "name": "Max Verstappen", "team": "Red Bull", "hex": "#001A30", "distance_m": 1500.0, "speed_ms": 80.0},
            {"code": "PLY", "name": "Player", "team": "Custom", "hex": "#E10600", "distance_m": 1460.0, "speed_ms": 80.0, "is_player": True},
            {"code": "LEC", "name": "Charles Leclerc", "team": "Ferrari", "hex": "#D40000", "distance_m": 1420.0, "speed_ms": 80.0},
        ]
        minimap = [
            {"id": 0, "code": "PLY", "color": "PLAYER", "x": 100.0, "y": 50.0, "is_player": True},
            {"id": 1, "code": "VER", "color": "AI_RED_BULL", "x": 120.0, "y": 55.0, "is_player": False},
        ]
        hud.update(
            speed_kmh=312.4,
            rpm=13100.0,
            gear_str="8",
            current_lap_time_sec=81.234,
            current_lap=2,
            total_laps=3,
            car_standings=standings,
            minimap_cars=minimap,
        )
        self.assertEqual(hud.speed_display, "312 KM/H")
        self.assertEqual(hud.lap_timer_display, "1:21.234")
        self.assertEqual(len(hud.timing_tower), 3)
        self.assertEqual(hud.timing_tower[0].position_label, "P1")
        self.assertEqual(hud.timing_tower[0].gap_str, "LEADER")
        self.assertEqual(hud.timing_tower[1].position_label, "P2")
        self.assertEqual(hud.timing_tower[1].gap_str, "+0.500s")
        self.assertEqual(len(hud.minimap_dots), 2)

    def test_podium_results_classification_and_dnf_handling(self):
        builder = RaceResultsBuilder()
        entries = [
            {"code": "NOR", "name": "Lando Norris", "team": "McLaren", "time": 120.0, "best_lap": 83.0, "status": "DNF"},
            {"code": "LEC", "name": "Charles Leclerc", "team": "Ferrari", "time": 248.5, "best_lap": 81.1, "status": "FINISHED"},
            {"code": "VER", "name": "Max Verstappen", "team": "Red Bull", "time": 245.2, "best_lap": 81.4, "status": "FINISHED"},
        ]
        classified = builder.classify(entries)
        self.assertEqual(classified[0].code, "VER")
        self.assertEqual(classified[0].gap, "WINNER")
        self.assertEqual(classified[1].code, "LEC")
        self.assertEqual(classified[1].gap, "+3.300s")
        self.assertTrue(classified[1].fastest_lap_award)
        self.assertEqual(classified[2].code, "NOR")
        self.assertEqual(classified[2].gap, "DNF")

    def test_procedural_v6_sound_engine(self):
        snd = SoundEngine(sample_rate=44100, enabled=True, init_mixer=False)
        pcm = snd.synthesize_v6_waveform(rpm=12000.0, throttle=1.0, duration_sec=0.05)
        self.assertEqual(pcm.dtype, np.int16)
        self.assertEqual(pcm.shape[0], 2205)
        self.assertGreater(int(np.max(np.abs(pcm))), 5000)

        for ev in SUPPORTED_SOUND_EVENTS:
            self.assertTrue(snd.trigger_event(ev))
        self.assertFalse(snd.trigger_event("UNKNOWN_EVENT"))


class TestMilestone5EndToEndApp(unittest.TestCase):
    """Tests F1SimulationApp end-to-end headless execution on Monza and Monaco."""

    def test_app_headless_race_monza_and_monaco(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            app_monza = F1SimulationApp(
                track_name="Monza",
                difficulty="Medium",
                total_laps=3,
                preset="HIGH",
                headless=True,
                config_dir=Path(tmpdir),
            )
            rc1 = app_monza.run_headless(num_frames=30)
            self.assertEqual(rc1, 0)
            self.assertEqual(len(app_monza.ai_controllers), 5)
            self.assertEqual(len(app_monza.hud.timing_tower), 6)

            app_monaco = F1SimulationApp(
                track_name="Monaco",
                difficulty="Hard",
                total_laps=3,
                preset="LOW",
                headless=True,
                config_dir=Path(tmpdir),
            )
            rc2 = app_monaco.run_headless(num_frames=20)
            self.assertEqual(rc2, 0)
            self.assertEqual(app_monaco.track.length, 3337.0)


if __name__ == "__main__":
    unittest.main()
