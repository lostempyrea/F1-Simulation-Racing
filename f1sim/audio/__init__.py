"""
f1sim/audio/__init__.py
=======================
Public exports for procedural V6 engine audio synthesis and event cues.
"""
from f1sim.audio.sound_engine import (
    SUPPORTED_SOUND_EVENTS,
    SoundEngine,
    compute_v6_fundamental_hz,
    compute_v6_harmonics_hz,
)

__all__ = [
    "SUPPORTED_SOUND_EVENTS",
    "SoundEngine",
    "compute_v6_fundamental_hz",
    "compute_v6_harmonics_hz",
]
