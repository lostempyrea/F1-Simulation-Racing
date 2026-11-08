"""
main.py
=======
Primary Desktop Native Entry Point & Game State Machine for the Formula 1
First-Person Cockpit Simulation Game.

Supports:
- Windowed OpenGL 3.3 Core Profile interactive mode (Menu -> Track/Difficulty
  Select -> Graphics Settings / Keybindings -> 5-Red-Light Grid Start ->
  Race -> Podium Results)
- Headless simulation mode (`--headless --frames N`) for automated CI/E2E testing
- Automated Physics & AI Benchmark mode (`--benchmark`)
"""
from __future__ import annotations
import argparse
import math
from pathlib import Path
import sys
from typing import Any, Dict, List, Optional
import numpy as np

from f1sim.ai import (
    AI_DRIVER_ROSTER,
    AIDifficulty,
    AIDriverController,
    RacingLineProfile,
)
from f1sim.audio import SoundEngine
from f1sim.core.math3d import Mat3, Mat4, Vec3
from f1sim.core.time_step import AccumulatorClock
from f1sim.graphics import (
    CockpitRenderer,
    GraphicsSettings,
)
from f1sim.input import (
    InputManager,
    KeyBindingConfig,
)
from f1sim.physics.vehicle import Vehicle
from f1sim.track.monaco import MonacoTrack
from f1sim.track.monza import MonzaTrack
from f1sim.track.track_builder import StartingGantry, TrackBuilder
from f1sim.ui import (
    BroadcastHUD,
    MotorsportMenuManager,
    RaceResultsBuilder,
)


def create_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Formula 1 First-Person Cockpit Simulation")
    parser.add_argument("--track", choices=["Monza", "Monaco"], default="Monza", help="Select F1 circuit")
    parser.add_argument("--difficulty", choices=["Easy", "Medium", "Hard"], default="Medium", help="Select AI difficulty")
    parser.add_argument("--laps", type=int, default=3, help="Number of race laps (default: 3)")
    parser.add_argument("--preset", choices=["LOW", "MEDIUM", "HIGH", "ULTRA"], default="HIGH", help="Graphics preset")
    parser.add_argument("--headless", action="store_true", help="Run headlessly without opening a desktop window")
    parser.add_argument("--frames", type=int, default=0, help="Number of simulation frames to run headlessly before exiting (0 = unlimited)")
    parser.add_argument("--window-frames", type=int, default=0, help="Number of desktop window frames to run before exiting (0 = unlimited)")
    parser.add_argument("--benchmark", action="store_true", help="Run the headless physics & AI benchmark suite and exit")
    return parser


class F1SimulationApp:
    """
    Top-level Formula 1 Simulation Application orchestrating physics, track,
    AI opponents, first-person 3D cockpit renderer, dual-input manager,
    graphics settings, keybinding remapping, broadcast HUD, and audio engine.
    """

    def __init__(
        self,
        track_name: str = "Monza",
        difficulty: str = "Medium",
        total_laps: int = 3,
        preset: str = "HIGH",
        headless: bool = False,
        config_dir: Optional[Path] = None,
    ) -> None:
        self.headless: bool = bool(headless)
        self.config_dir: Path = Path(config_dir) if config_dir is not None else (Path(__file__).resolve().parent / "config")

        # Load or initialize GraphicsSettings and KeyBindingConfig
        self.graphics_settings: GraphicsSettings = GraphicsSettings.load(self.config_dir / "graphics_settings.json")
        if preset:
            self.graphics_settings.apply_preset(preset)
        self.keybindings: KeyBindingConfig = KeyBindingConfig.load(self.config_dir / "keybindings.json")

        self.menu: MotorsportMenuManager = MotorsportMenuManager(
            graphics_settings=self.graphics_settings,
            keybindings=self.keybindings,
            config_dir=self.config_dir,
        )
        self.menu.select_track(track_name)
        self.menu.select_difficulty(difficulty)
        self.menu.session.total_laps = max(1, int(total_laps))

        self.input_manager: InputManager = InputManager(bindings=self.keybindings)
        self.sound_engine: SoundEngine = SoundEngine(enabled=True, init_mixer=not self.headless)
        self.hud: BroadcastHUD = BroadcastHUD(num_cars=6)
        self.results_builder: RaceResultsBuilder = RaceResultsBuilder()
        self.clock: AccumulatorClock = AccumulatorClock()

        # Session objects initialized by `setup_race_session()`
        self.track: Any = None
        self.track_builder: Optional[TrackBuilder] = None
        self.gantry: StartingGantry = StartingGantry()
        self.player_car: Vehicle = Vehicle()
        self.player_s: float = 0.0
        self.player_d: float = 0.0
        self.player_lap: int = 1
        self.player_lap_start_t: float = 0.0
        self.player_lap_times: List[float] = []
        self.ai_controllers: List[AIDriverController] = []
        self.renderer: Optional[CockpitRenderer] = None
        self.sim_time: float = 0.0

        self.setup_race_session()

    def setup_race_session(self) -> None:
        """Initializes the selected F1 circuit, player vehicle, and 5 AI opponents on the starting grid."""
        if self.menu.session.track_name.lower() == "monaco":
            self.track = MonacoTrack()
        else:
            self.track = MonzaTrack()

        self.track_builder = TrackBuilder(self.track)
        self.gantry = StartingGantry()
        self.gantry.start_countdown(hold_duration=0.8)

        self.player_car = Vehicle()
        self.player_s = 0.0
        self.player_d = 0.0
        self.player_lap = 1
        self.player_lap_start_t = 0.0
        self.player_lap_times = []
        self.sim_time = 0.0

        shared_line = RacingLineProfile(self.track, difficulty=self.menu.session.difficulty)
        self.ai_controllers = []
        for idx, drv in enumerate(AI_DRIVER_ROSTER):
            ai_ctrl = AIDriverController(
                driver=drv,
                track=self.track,
                difficulty=self.menu.session.difficulty,
                racing_line=shared_line,
                initial_s=(idx + 1) * 9.0,  # Staggered FIA starting grid ahead of player
                initial_d=(-2.0 if (idx % 2 == 0) else 2.0),
            )
            self.ai_controllers.append(ai_ctrl)

    def get_ai_render_list(self) -> List[ tuple[Mat4, tuple[float, float, float]] ]:
        """Builds 3D world model matrices and RGB livery colors for all 5 AI cars."""
        if self.track is None or not hasattr(self.track, "spline"):
            return []
        from f1sim.ui import hex_to_rgb
        track_len = float(self.track.length)
        out: List[tuple[Mat4, tuple[float, float, float]]] = []
        for ai in self.ai_controllers:
            s_ai = max(0.0, float(ai.total_distance_m)) % track_len
            d_ai = float(getattr(ai, "d_m", getattr(ai, "lateral_offset_m", 0.0)))
            pos_c = self.track.spline.get_point(s_ai)
            tan = self.track.spline.get_tangent(s_ai)
            lat, norm = self.track.spline.get_lateral_normal(s_ai)
            car_pos = pos_c + d_ai * lat
            basis = Mat3([
                [float(lat[0]), float(tan[0]), float(norm[0])],
                [float(lat[1]), float(tan[1]), float(norm[1])],
                [float(lat[2]), float(tan[2]), float(norm[2])],
            ])
            model_mat = Mat4.from_mat3_and_pos(basis, Vec3(float(car_pos[0]), float(car_pos[1]), float(car_pos[2])))
            r, g, b = hex_to_rgb(ai.driver.hex)
            out.append((model_mat, (r / 255.0, g / 255.0, b / 255.0)))
        return out

    def step_simulation(self, dt: float = 1.0 / 120.0) -> None:
        """Advances one simulation frame (gantry lights, player input & physics, 5 AI cars, HUD, audio)."""
        self.sim_time += dt
        self.gantry.step(dt)

        track_len = float(self.track.length)
        s_mod = max(0.0, self.player_s) % track_len
        in_drs = bool(self.track.is_in_drs_zone(s_mod))

        # 1. Update driver input & step player car with 120 Hz substeps for crisp acceleration
        inp_state = self.input_manager.update(speed_ms=self.player_car.vx, dt=dt)
        if inp_state.halo_toggle_pressed:
            self.graphics_settings.cycle_halo_opacity()

        if inp_state.throttle > 0.05 and self.player_car.gear == 0:
            self.player_car.shift_up()

        num_sub = max(1, min(6, int(round(dt / (1.0 / 120.0)))))
        sub_dt = dt / num_sub
        self.input_manager.apply_to_vehicle(self.player_car, dt=sub_dt, in_drs_zone=in_drs)
        for _ in range(num_sub - 1):
            self.player_car.step(
                throttle=inp_state.throttle,
                brake=inp_state.brake,
                steer=inp_state.road_steer_rad,
                dt=sub_dt,
            )

        self.player_s += self.player_car.vx * dt
        self.player_d = float(np.clip(
            self.player_d - self.player_car.vx * math.sin(self.player_car.steering_angle * 0.45) * dt,
            -6.5,
            6.5,
        ))
        s_mod = max(0.0, self.player_s) % track_len

        calc_lap = int(max(0.0, self.player_s) // track_len) + 1
        if calc_lap > self.player_lap:
            self.player_lap_times.append(self.sim_time - self.player_lap_start_t)
            self.player_lap_start_t = self.sim_time
            self.player_lap = min(self.menu.session.total_laps, calc_lap)

        # 2. Step all 5 AI opponents
        for ai in self.ai_controllers:
            ai.step(
                dt=dt,
                sim_time=self.sim_time,
                lead_gap_m=max(0.0, self.player_s - ai.total_distance_m),
                lead_speed_ms=self.player_car.vx,
                lead_d_m=self.player_d,
                total_laps=self.menu.session.total_laps,
            )

        # 3. Update Broadcast TV HUD & Cockpit Telemetry LCD
        standings = [
            {
                "code": "PLY",
                "name": "Player",
                "team": "Custom",
                "hex": "#E10600",
                "distance_m": self.player_s,
                "speed_ms": self.player_car.vx,
                "is_player": True,
            }
        ]
        for ai in self.ai_controllers:
            standings.append({
                "code": ai.driver.code,
                "name": ai.driver.name,
                "team": ai.driver.team,
                "hex": ai.driver.hex,
                "distance_m": ai.total_distance_m,
                "speed_ms": ai.vehicle.vx,
                "is_player": False,
            })

        self.hud.update(
            speed_kmh=self.player_car.speed_kmh,
            rpm=self.player_car.rpm,
            gear_str=str(self.player_car.gear if self.player_car.gear > 0 else ("R" if self.player_car.gear < 0 else "N")),
            current_lap_time_sec=self.sim_time - self.player_lap_start_t,
            current_lap=self.player_lap,
            total_laps=self.menu.session.total_laps,
            car_standings=standings,
        )

        self.sound_engine.update(
            rpm=self.player_car.rpm,
            throttle=self.player_car.throttle,
            speed_kmh=self.player_car.speed_kmh,
            on_kerb=(self.player_car.kerb_vibration > 0.01),
            dt=dt,
        )

        if self.renderer is not None:
            self.renderer.lcd.update_from_vehicle(self.player_car, dt=dt)
            self.renderer.camera.update(
                dt=dt,
                speed_kmh=self.player_car.speed_kmh,
                longitudinal_g=self.player_car.longitudinal_g,
                lateral_g=self.player_car.lateral_g,
                rpm=self.player_car.rpm,
                on_kerb=(self.player_car.kerb_vibration > 0.01),
                steer_norm=inp_state.raw_steer,
            )
            if self.track is not None and hasattr(self.track, "spline"):
                pos_c = self.track.spline.get_point(s_mod)
                tan = self.track.spline.get_tangent(s_mod)
                lat, norm = self.track.spline.get_lateral_normal(s_mod)
                cam = self.renderer.camera
                car_pos = pos_c + self.player_d * lat
                eye_arr = (
                    car_pos
                    + (cam.cfg.eye_y + 0.95 + cam.neck_offset.y) * norm
                    + (cam.cfg.eye_z + cam.neck_offset.z) * tan
                    + cam.neck_offset.x * lat
                )
                steer_yaw = self.player_car.steering_angle * 0.35 + cam.head_yaw_rad
                road_pitch = cam.head_pitch_rad - 0.135
                fwd_arr = (
                    tan * math.cos(steer_yaw) * math.cos(road_pitch)
                    + lat * math.sin(steer_yaw) * math.cos(road_pitch)
                    + norm * math.sin(road_pitch)
                )
                up_arr = norm * math.cos(cam.head_roll_rad) + lat * math.sin(cam.head_roll_rad)
                cam.world_eye_pos = Vec3(float(eye_arr[0]), float(eye_arr[1]), float(eye_arr[2]))
                cam.world_forward = Vec3(float(fwd_arr[0]), float(fwd_arr[1]), float(fwd_arr[2])).normalized()
                cam.world_up = Vec3(float(up_arr[0]), float(up_arr[1]), float(up_arr[2])).normalized()

    def run_headless(self, num_frames: int = 120) -> int:
        """Executes `num_frames` of simulation headlessly with standalone OpenGL rendering."""
        self.renderer = CockpitRenderer(settings=self.graphics_settings, standalone=True, init_gl=True)
        try:
            if self.track_builder is not None:
                self.renderer.upload_track_mesh(self.track_builder.get_combined_track_mesh())
            self.menu.transition_to("RACE")
            steps = max(1, int(num_frames))
            for _ in range(steps):
                self.step_simulation(dt=1.0 / 120.0)
            self.renderer.render_frame(
                wheel_angle_rad=self.input_manager.state.wheel_angle_rad,
                ai_cars=self.get_ai_render_list(),
            )
        finally:
            self.renderer.release()
            self.renderer = None
        return 0


def main(argv: Optional[List[str]] = None) -> int:
    args = create_arg_parser().parse_args(argv)
    if args.benchmark:
        from tests.run_all_tests import run_benchmark_suite
        passed, *_ = run_benchmark_suite(verbose=True)
        return 0 if passed else 1

    if args.headless or args.frames > 0:
        app = F1SimulationApp(
            track_name=args.track,
            difficulty=args.difficulty,
            total_laps=args.laps,
            preset=args.preset,
            headless=True,
        )
        return app.run_headless(num_frames=args.frames if args.frames > 0 else 60)

    # Interactive desktop window loop: open Pygame window IMMEDIATELY so Windows never marks it unresponsive
    app: Optional[F1SimulationApp] = None
    try:
        import pygame
        pygame.init()
        cfg_dir = Path(__file__).resolve().parent / "config"
        gs_init = GraphicsSettings.load(cfg_dir / "graphics_settings.json")
        if args.preset:
            gs_init.apply_preset(args.preset)
        w, h = gs_init.resolution
        flags = pygame.OPENGL | pygame.DOUBLEBUF
        if gs_init.fullscreen:
            flags |= pygame.FULLSCREEN
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_PROFILE_MASK, pygame.GL_CONTEXT_PROFILE_CORE)
        pygame.display.set_mode((w, h), flags)
        pygame.display.set_caption(f"F1 Cockpit Simulation - Dang tai duong dua {args.track}...")
        pygame.event.pump()

        app = F1SimulationApp(
            track_name=args.track,
            difficulty=args.difficulty,
            total_laps=args.laps,
            preset=args.preset,
            headless=False,
        )
        pygame.event.pump()

        app.renderer = CockpitRenderer(settings=app.graphics_settings, standalone=False, init_gl=True)
        pygame.event.pump()
        if app.track_builder is not None:
            app.track_builder.build_track_ribbon(step_size=2.0)
            app.track_builder.build_kerbs(step_size=0.5)
            app.renderer.upload_track_mesh(app.track_builder.get_combined_track_mesh())
        pygame.event.pump()

        app.menu.transition_to("RACE")
        clock = pygame.time.Clock()
        running = True
        win_frames = 0
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif app.keybindings.is_capturing:
                    app.menu.handle_key_capture_event(event)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        app.setup_race_session()
                    elif event.key == pygame.K_m:
                        next_trk = "Monaco" if app.menu.session.track_name.lower() == "monza" else "Monza"
                        pygame.display.set_caption(f"F1 Cockpit Simulation - Dang chuyen sang {next_trk}...")
                        pygame.event.pump()
                        app.menu.select_track(next_trk)
                        app.setup_race_session()
                        if app.track_builder is not None and app.renderer is not None:
                            app.track_builder.build_track_ribbon(step_size=2.0)
                            app.track_builder.build_kerbs(step_size=0.5)
                            app.renderer.upload_track_mesh(app.track_builder.get_combined_track_mesh())

            dt = max(0.001, min(0.05, clock.tick(app.graphics_settings.fps_limit or 120) / 1000.0))
            app.step_simulation(dt=dt)
            app.renderer.render_frame(
                wheel_angle_rad=app.input_manager.state.wheel_angle_rad,
                ai_cars=app.get_ai_render_list(),
            )
            pygame.display.flip()
            win_frames += 1
            if win_frames % 4 == 0:
                pos_str = "P1"
                for row in app.hud.timing_tower:
                    if row.driver_code == "PLY":
                        pos_str = row.position_label
                        break
                drs_txt = "ACTIVE" if app.player_car.drs_active else ("AVAIL" if app.player_car.drs_available else "OFF")
                pygame.display.set_caption(
                    f"F1 Sim [{app.menu.session.track_name}] | {int(round(app.player_car.speed_kmh)):3d} KM/H | "
                    f"Gear {app.renderer.lcd.state.gear_str} | {int(round(app.player_car.rpm)):5d} RPM | "
                    f"{pos_str}/6 | Lap {app.player_lap}/{app.menu.session.total_laps} | DRS:{drs_txt} | "
                    f"[W/Up] Ga  [S/Down] Phanh  [A/D] Lai  [F] DRS  [H] Halo  [M] Doi Map  [R] Reset"
                )
            if args.window_frames > 0 and win_frames >= args.window_frames:
                running = False
    finally:
        if app is not None and app.renderer is not None:
            app.renderer.release()
        try:
            import pygame
            pygame.quit()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
