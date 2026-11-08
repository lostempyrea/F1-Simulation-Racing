"""
f1sim/core/time_step.py
Fixed-frequency physics integration accumulator clock (120 Hz)
with render alpha interpolation and headless testing sub-stepping harness.
"""
import math
from dataclasses import dataclass
from typing import Callable
from f1sim.core.constants import SIM_HZ, SIM_DT, MAX_ACCUMULATOR_SEC, BENCHMARK_HZ, BENCHMARK_DT


@dataclass
class TimeStepState:
    tick_count: int = 0
    sim_time_sec: float = 0.0
    wall_time_sec: float = 0.0
    dt: float = SIM_DT
    alpha: float = 0.0
    fps_measured: float = 120.0
    physics_rate_measured: float = 120.0


class AccumulatorClock:
    """
    Fixed-frequency accumulator managing decoupling between variable
    rendering frame rates (60/144/240 Hz) and fixed 120 Hz physics integration.
    """
    def __init__(self, hz: int = SIM_HZ, max_accumulator_sec: float = MAX_ACCUMULATOR_SEC) -> None:
        self.hz: int = hz
        self.dt: float = 1.0 / hz
        self.max_accumulator: float = max_accumulator_sec
        self.accumulator: float = 0.0
        self.sim_time: float = 0.0
        self.wall_time: float = 0.0
        self.tick_count: int = 0
        self.alpha: float = 0.0
        self._frame_count: int = 0
        self._fps_timer: float = 0.0
        self.current_fps: float = float(hz)

    def reset(self) -> None:
        self.accumulator = 0.0
        self.sim_time = 0.0
        self.wall_time = 0.0
        self.tick_count = 0
        self.alpha = 0.0
        self._frame_count = 0
        self._fps_timer = 0.0

    def tick(self, frame_dt_wall: float) -> int:
        """
        Consumes wall-clock delta time, advances accumulator, and returns the
        exact number of fixed physics steps (0, 1, 2, ...) that must be executed.
        """
        clamped_dt = min(frame_dt_wall, self.max_accumulator)
        self.accumulator += clamped_dt
        self.wall_time += frame_dt_wall

        # FPS metering
        self._frame_count += 1
        self._fps_timer += frame_dt_wall
        if self._fps_timer >= 1.0:
            self.current_fps = float(self._frame_count) / self._fps_timer
            self._frame_count = 0
            self._fps_timer = 0.0

        ticks = int(self.accumulator // self.dt)
        self.accumulator -= ticks * self.dt
        self.sim_time += ticks * self.dt
        self.tick_count += ticks

        # Render interpolation factor alpha in [0.0, 1.0)
        self.alpha = self.accumulator / self.dt
        return ticks

    def get_state(self) -> TimeStepState:
        return TimeStepState(
            tick_count=self.tick_count,
            sim_time_sec=self.sim_time,
            wall_time_sec=self.wall_time,
            dt=self.dt,
            alpha=self.alpha,
            fps_measured=self.current_fps,
            physics_rate_measured=float(self.hz)
        )


class SubstepRunner:
    """Harness for running headless physics benchmarks at 1000 Hz."""
    @staticmethod
    def run_substeps(total_time_sec: float, substep_dt: float, step_callback: Callable[[float], None]) -> int:
        total_steps = int(math.ceil(total_time_sec / substep_dt))
        for _ in range(total_steps):
            step_callback(substep_dt)
        return total_steps
