"""
f1sim/audio/sound_engine.py
===========================
Procedural 1.6L V6 Turbo-Hybrid Audio Synthesizer & Event Sound Engine.
Synthesizes real-time multi-harmonic V6 engine firing waveforms (f0 = RPM / 20)
and cockpit sound events (SHIFT_POP, TYRE_SCREECH, DRS_BEEP, KERB_RUMBLE)
with full headless safety when no hardware audio output device is present.
"""
from __future__ import annotations
import math
from typing import Dict, List, Optional
import numpy as np


SUPPORTED_SOUND_EVENTS: List[str] = [
    "SHIFT_POP",
    "TYRE_SCREECH",
    "DRS_BEEP",
    "KERB_RUMBLE",
]


def compute_v6_fundamental_hz(rpm: float) -> float:
    """
    Computes the primary 4-stroke 6-cylinder firing frequency (Hz):
        f0 = (RPM / 60) * (6 / 2) = RPM / 20.0
    """
    return max(50.0, float(rpm) / 20.0)


def compute_v6_harmonics_hz(rpm: float) -> tuple[float, float, float]:
    """Returns (f0, 2*f0, 3*f0) harmonic frequencies in Hz."""
    f0 = compute_v6_fundamental_hz(rpm)
    return (f0, 2.0 * f0, 3.0 * f0)


class SoundEngine:
    """
    Procedural V6 Turbo-Hybrid sound synthesizer and event player.
    """

    def __init__(self, sample_rate: int = 44100, enabled: bool = True, init_mixer: bool = False) -> None:
        self.sample_rate: int = max(8000, int(sample_rate))
        self.enabled: bool = bool(enabled)
        self.supported_events: List[str] = list(SUPPORTED_SOUND_EVENTS)
        self.mixer_initialized: bool = False
        self.triggered_events: List[str] = []
        self._phase: float = 0.0

        if init_mixer and self.enabled:
            self.try_init_mixer()

    def try_init_mixer(self) -> bool:
        """Attempts to initialize pygame.mixer safely (no-op on headless CI/servers)."""
        try:
            import pygame
            if not pygame.mixer.get_init():
                pygame.mixer.init(frequency=self.sample_rate, size=-16, channels=1, buffer=512)
            self.mixer_initialized = bool(pygame.mixer.get_init())
        except Exception:
            self.mixer_initialized = False
        return self.mixer_initialized

    def synthesize_v6_waveform(
        self,
        rpm: float,
        throttle: float = 1.0,
        duration_sec: float = 0.02,
    ) -> np.ndarray:
        """
        Synthesizes a 16-bit PCM mono waveform (`np.int16`) for the 1.6L V6
        Turbo-Hybrid engine at the given `rpm` and `throttle`.
        """
        num_samples = max(16, int(round(self.sample_rate * float(duration_sec))))
        t = (np.arange(num_samples, dtype=np.float64) / self.sample_rate) + self._phase
        self._phase = float(t[-1]) % 10.0

        f0, f1, f2 = compute_v6_harmonics_hz(rpm)
        thr = max(0.0, min(1.0, float(throttle)))

        # Multi-harmonic V6 exhaust + MGU-H turbo whine
        wave = (
            0.52 * np.sin(2.0 * math.pi * f0 * t)
            + 0.28 * np.sin(2.0 * math.pi * f1 * t + 0.3)
            + 0.14 * np.sin(2.0 * math.pi * f2 * t + 0.7)
            + 0.06 * thr * np.sin(2.0 * math.pi * (f0 * 5.5) * t)
        )
        gain = 0.35 + 0.65 * thr
        pcm = np.clip(wave * gain * 28000.0, -32767.0, 32767.0).astype(np.int16)
        return pcm

    def trigger_event(self, event_name: str) -> bool:
        """
        Triggers a discrete cockpit audio cue (`SHIFT_POP`, `TYRE_SCREECH`,
        `DRS_BEEP`, or `KERB_RUMBLE`).
        """
        ev = str(event_name).strip().upper()
        if ev not in self.supported_events:
            return False
        self.triggered_events.append(ev)
        if len(self.triggered_events) > 64:
            self.triggered_events = self.triggered_events[-64:]
        return True

    def update(
        self,
        rpm: float = 4000.0,
        throttle: float = 0.0,
        speed_kmh: float = 0.0,
        on_kerb: bool = False,
        dt: float = 1.0 / 120.0,
    ) -> None:
        """Updates continuous V6 engine state and kerb rumble cue."""
        if not self.enabled:
            return
        if on_kerb and speed_kmh > 10.0:
            self.trigger_event("KERB_RUMBLE")
