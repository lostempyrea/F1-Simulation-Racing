"""
f1sim/graphics/settings.py
==========================
Graphics configuration, quality presets (Low, Medium, High, Ultra, Custom),
cockpit visual options (FOV, Halo pillar opacity, rear-view mirror FBOs,
camera G-shake, radial speed blur), and JSON persistence.
"""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
from typing import Any, Dict, Tuple


class GraphicsPreset(str, Enum):
    """Standard graphics quality tiers."""
    LOW = "LOW"
    MEDIUM = "MEDIUM"
    HIGH = "HIGH"
    ULTRA = "ULTRA"
    CUSTOM = "CUSTOM"


SUPPORTED_RESOLUTIONS: Tuple[Tuple[int, int], ...] = (
    (1280, 720),
    (1600, 900),
    (1920, 1080),
    (2560, 1440),
    (3840, 2160),
)

VALID_MSAA_SAMPLES: Tuple[int, ...] = (0, 2, 4, 8)


@dataclass
class GraphicsSettings:
    """
    Complete graphics & cockpit visual configuration for F1 Simulation.
    """
    preset: str = GraphicsPreset.HIGH.value
    resolution: Tuple[int, int] = (1280, 720)
    fullscreen: bool = False
    vsync: bool = True
    fps_limit: int = 120
    msaa_samples: int = 4
    base_fov_deg: float = 68.0
    dynamic_fov_enabled: bool = True
    max_fov_delta_deg: float = 10.0
    rear_mirrors_enabled: bool = True
    mirror_resolution: Tuple[int, int] = (384, 160)
    motion_blur_enabled: bool = True
    motion_blur_samples: int = 6
    camera_shake_intensity: float = 1.0
    head_g_tilt_intensity: float = 1.0
    halo_opacity: float = 1.0
    draw_distance_m: float = 1200.0

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Clamps and validates all graphics settings within safe hardware bounds."""
        preset_upper = str(self.preset).strip().upper()
        valid_presets = {p.value for p in GraphicsPreset}
        self.preset = preset_upper if preset_upper in valid_presets else GraphicsPreset.CUSTOM.value

        if isinstance(self.resolution, (list, tuple)) and len(self.resolution) == 2:
            w = max(640, min(7680, int(self.resolution[0])))
            h = max(360, min(4320, int(self.resolution[1])))
            self.resolution = (w, h)
        else:
            self.resolution = (1280, 720)

        self.fullscreen = bool(self.fullscreen)
        self.vsync = bool(self.vsync)
        self.fps_limit = max(0, min(500, int(self.fps_limit)))

        msaa = int(self.msaa_samples)
        if msaa not in VALID_MSAA_SAMPLES:
            msaa = min(VALID_MSAA_SAMPLES, key=lambda x: abs(x - msaa))
        self.msaa_samples = msaa

        self.base_fov_deg = max(60.0, min(75.0, float(self.base_fov_deg)))
        self.dynamic_fov_enabled = bool(self.dynamic_fov_enabled)
        self.max_fov_delta_deg = max(0.0, min(15.0, float(self.max_fov_delta_deg)))

        self.rear_mirrors_enabled = bool(self.rear_mirrors_enabled)
        if isinstance(self.mirror_resolution, (list, tuple)) and len(self.mirror_resolution) == 2:
            mw = max(128, min(1024, int(self.mirror_resolution[0])))
            mh = max(64, min(512, int(self.mirror_resolution[1])))
            self.mirror_resolution = (mw, mh)
        else:
            self.mirror_resolution = (384, 160)

        self.motion_blur_enabled = bool(self.motion_blur_enabled)
        self.motion_blur_samples = max(0, min(12, int(self.motion_blur_samples)))
        self.camera_shake_intensity = max(0.0, min(2.0, float(self.camera_shake_intensity)))
        self.head_g_tilt_intensity = max(0.0, min(2.0, float(self.head_g_tilt_intensity)))
        self.halo_opacity = max(0.15, min(1.0, float(self.halo_opacity)))
        self.draw_distance_m = max(300.0, min(2500.0, float(self.draw_distance_m)))

    def apply_preset(self, preset: str | GraphicsPreset) -> None:
        """Applies a named quality preset (LOW, MEDIUM, HIGH, ULTRA)."""
        p = preset.value if isinstance(preset, GraphicsPreset) else str(preset).strip().upper()
        if p == GraphicsPreset.LOW.value:
            self.preset = GraphicsPreset.LOW.value
            self.msaa_samples = 0
            self.rear_mirrors_enabled = False
            self.mirror_resolution = (192, 80)
            self.motion_blur_enabled = False
            self.motion_blur_samples = 0
            self.draw_distance_m = 600.0
        elif p == GraphicsPreset.MEDIUM.value:
            self.preset = GraphicsPreset.MEDIUM.value
            self.msaa_samples = 2
            self.rear_mirrors_enabled = True
            self.mirror_resolution = (256, 112)
            self.motion_blur_enabled = False
            self.motion_blur_samples = 0
            self.draw_distance_m = 900.0
        elif p == GraphicsPreset.HIGH.value:
            self.preset = GraphicsPreset.HIGH.value
            self.msaa_samples = 4
            self.rear_mirrors_enabled = True
            self.mirror_resolution = (384, 160)
            self.motion_blur_enabled = True
            self.motion_blur_samples = 6
            self.draw_distance_m = 1200.0
        elif p == GraphicsPreset.ULTRA.value:
            self.preset = GraphicsPreset.ULTRA.value
            self.msaa_samples = 8
            self.rear_mirrors_enabled = True
            self.mirror_resolution = (512, 224)
            self.motion_blur_enabled = True
            self.motion_blur_samples = 10
            self.draw_distance_m = 2000.0
        else:
            self.preset = GraphicsPreset.CUSTOM.value
        self.validate()

    def cycle_halo_opacity(self) -> float:
        """Cycles Halo center strut opacity between 100%, 35% (translucent), and 15%."""
        if self.halo_opacity > 0.65:
            self.halo_opacity = 0.35
        elif self.halo_opacity > 0.20:
            self.halo_opacity = 0.15
        else:
            self.halo_opacity = 1.0
        return self.halo_opacity

    def to_dict(self) -> Dict[str, Any]:
        """Serializes graphics settings to a JSON-compatible dictionary."""
        return {
            "preset": self.preset,
            "resolution": [int(self.resolution[0]), int(self.resolution[1])],
            "fullscreen": bool(self.fullscreen),
            "vsync": bool(self.vsync),
            "fps_limit": int(self.fps_limit),
            "msaa_samples": int(self.msaa_samples),
            "base_fov_deg": float(self.base_fov_deg),
            "dynamic_fov_enabled": bool(self.dynamic_fov_enabled),
            "max_fov_delta_deg": float(self.max_fov_delta_deg),
            "rear_mirrors_enabled": bool(self.rear_mirrors_enabled),
            "mirror_resolution": [int(self.mirror_resolution[0]), int(self.mirror_resolution[1])],
            "motion_blur_enabled": bool(self.motion_blur_enabled),
            "motion_blur_samples": int(self.motion_blur_samples),
            "camera_shake_intensity": float(self.camera_shake_intensity),
            "head_g_tilt_intensity": float(self.head_g_tilt_intensity),
            "halo_opacity": float(self.halo_opacity),
            "draw_distance_m": float(self.draw_distance_m),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> GraphicsSettings:
        """Constructs GraphicsSettings from a dictionary, applying safe defaults."""
        res_raw = data.get("resolution", [1280, 720])
        res = (int(res_raw[0]), int(res_raw[1])) if isinstance(res_raw, (list, tuple)) and len(res_raw) == 2 else (1280, 720)
        mres_raw = data.get("mirror_resolution", [384, 160])
        mres = (int(mres_raw[0]), int(mres_raw[1])) if isinstance(mres_raw, (list, tuple)) and len(mres_raw) == 2 else (384, 160)

        return cls(
            preset=str(data.get("preset", GraphicsPreset.HIGH.value)),
            resolution=res,
            fullscreen=bool(data.get("fullscreen", False)),
            vsync=bool(data.get("vsync", True)),
            fps_limit=int(data.get("fps_limit", 120)),
            msaa_samples=int(data.get("msaa_samples", 4)),
            base_fov_deg=float(data.get("base_fov_deg", 68.0)),
            dynamic_fov_enabled=bool(data.get("dynamic_fov_enabled", True)),
            max_fov_delta_deg=float(data.get("max_fov_delta_deg", 10.0)),
            rear_mirrors_enabled=bool(data.get("rear_mirrors_enabled", True)),
            mirror_resolution=mres,
            motion_blur_enabled=bool(data.get("motion_blur_enabled", True)),
            motion_blur_samples=int(data.get("motion_blur_samples", 6)),
            camera_shake_intensity=float(data.get("camera_shake_intensity", 1.0)),
            head_g_tilt_intensity=float(data.get("head_g_tilt_intensity", 1.0)),
            halo_opacity=float(data.get("halo_opacity", 1.0)),
            draw_distance_m=float(data.get("draw_distance_m", 1200.0)),
        )

    def save(self, path: str | Path) -> Path:
        """Saves graphics settings to a JSON file."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return target

    @classmethod
    def load(cls, path: str | Path) -> GraphicsSettings:
        """Loads graphics settings from a JSON file, or returns defaults if missing."""
        target = Path(path)
        if not target.exists():
            return cls()
        raw = json.loads(target.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            return cls()
        return cls.from_dict(raw)
