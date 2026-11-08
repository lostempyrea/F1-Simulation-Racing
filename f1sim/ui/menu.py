"""
f1sim/ui/menu.py
================
Motorsport UI & Main Menu State Machine covering:
- MAIN, TRACK_SELECT, DIFFICULTY_SELECT, GRAPHICS_SETTINGS, KEYBINDINGS,
  CONTROLS_HELP, RACE, PAUSE, and RESULTS states
- Track selection (Autodromo Nazionale Monza, Circuit de Monaco)
- AI Difficulty selection (Easy, Medium, Hard)
- Transmission mode selection (Manual, Auto)
- Interactive Graphics Settings UI screen (Presets, Resolution, Fullscreen,
  VSync, MSAA, Dynamic FOV, Rear Mirrors, Motion Blur, Halo Opacity)
- Interactive Keybinding & Controller Remapping UI screen (Rebind primary/secondary
  keys & gamepad buttons, live key capture, conflict auto-swap, reset to defaults)
"""
from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from f1sim.graphics.settings import (
    GraphicsPreset,
    GraphicsSettings,
    SUPPORTED_RESOLUTIONS,
    VALID_MSAA_SAMPLES,
)
from f1sim.input.bindings import (
    ACTION_DISPLAY_NAMES,
    ControlAction,
    KeyBindingConfig,
)


MENU_STATES: List[str] = [
    "MAIN",
    "TRACK_SELECT",
    "DIFFICULTY_SELECT",
    "GRAPHICS_SETTINGS",
    "KEYBINDINGS",
    "CONTROLS_HELP",
    "RACE",
    "PAUSE",
    "RESULTS",
]

SELECTABLE_TRACKS: List[str] = ["Monza", "Monaco"]
SELECTABLE_DIFFICULTIES: List[str] = ["Easy", "Medium", "Hard"]
SELECTABLE_TRANSMISSIONS: List[str] = ["Manual", "Auto"]

DEFAULT_CONTROLS_GUIDE: Dict[str, str] = {
    "Keyboard": "W/S (Throttle/Brake), A/D (Steer), E/Q (Shift Up/Down), F (DRS), I (Ignition), [/] (Brake Bias), T (Auto/Manual), H (Halo Opacity)",
    "Gamepad": "RT/LT Triggers (Throttle/Brake), Left Thumbstick (Steer), RB/LB (Shift), Y (DRS), A (Ignition), D-Pad Up/Down (Brake Bias)",
}


@dataclass
class RaceSessionConfig:
    """Selected parameters for launching a Grand Prix session."""
    track_name: str = "Monza"
    difficulty: str = "Medium"
    transmission: str = "Auto"
    total_laps: int = 3


class MotorsportMenuManager:
    """
    Manages UI screen navigation, race configuration, Graphics Settings UI,
    and Keybinding Remapping UI.
    """

    def __init__(
        self,
        graphics_settings: Optional[GraphicsSettings] = None,
        keybindings: Optional[KeyBindingConfig] = None,
        config_dir: Optional[Path] = None,
    ) -> None:
        self.states: List[str] = list(MENU_STATES)
        self.tracks: List[str] = list(SELECTABLE_TRACKS)
        self.difficulties: List[str] = list(SELECTABLE_DIFFICULTIES)
        self.transmissions: List[str] = list(SELECTABLE_TRANSMISSIONS)
        self.controls_guide: Dict[str, str] = dict(DEFAULT_CONTROLS_GUIDE)

        self.current_state: str = "MAIN"
        self.session: RaceSessionConfig = RaceSessionConfig()
        self.graphics: GraphicsSettings = (
            graphics_settings if graphics_settings is not None else GraphicsSettings()
        )
        self.bindings: KeyBindingConfig = (
            keybindings if keybindings is not None else KeyBindingConfig()
        )
        self.config_dir: Path = (
            Path(config_dir) if config_dir is not None else Path.cwd() / "config"
        )
        self.last_status_message: str = ""

    def transition_to(self, target_state: str) -> str:
        """Transitions the menu state machine to `target_state`."""
        target = str(target_state).strip().upper()
        if target not in self.states:
            raise ValueError(f"Invalid menu state {target_state!r}. Valid states: {self.states}")
        # Cancel any pending key capture when leaving KEYBINDINGS
        if self.current_state == "KEYBINDINGS" and target != "KEYBINDINGS":
            self.bindings.cancel_capture()
        self.current_state = target
        return self.current_state

    def select_track(self, track_name: str) -> str:
        """Selects 'Monza' or 'Monaco'."""
        for t in self.tracks:
            if t.lower() == str(track_name).strip().lower():
                self.session.track_name = t
                return t
        raise ValueError(f"Unsupported track {track_name!r}. Valid: {self.tracks}")

    def select_difficulty(self, difficulty: str) -> str:
        """Selects 'Easy', 'Medium', or 'Hard'."""
        for d in self.difficulties:
            if d.lower() == str(difficulty).strip().lower():
                self.session.difficulty = d
                return d
        raise ValueError(f"Unsupported difficulty {difficulty!r}. Valid: {self.difficulties}")

    def select_transmission(self, transmission: str) -> str:
        """Selects 'Manual' or 'Auto'."""
        for tr in self.transmissions:
            if tr.lower() == str(transmission).strip().lower():
                self.session.transmission = tr
                return tr
        raise ValueError(f"Unsupported transmission {transmission!r}. Valid: {self.transmissions}")

    # ------------------------------------------------------------------
    # Graphics Settings UI Actions
    # ------------------------------------------------------------------
    def cycle_graphics_preset(self, forward: bool = True) -> str:
        """Cycles quality preset across LOW -> MEDIUM -> HIGH -> ULTRA."""
        order = [
            GraphicsPreset.LOW.value,
            GraphicsPreset.MEDIUM.value,
            GraphicsPreset.HIGH.value,
            GraphicsPreset.ULTRA.value,
        ]
        curr = self.graphics.preset if self.graphics.preset in order else GraphicsPreset.HIGH.value
        idx = order.index(curr)
        next_idx = (idx + (1 if forward else -1)) % len(order)
        self.graphics.apply_preset(order[next_idx])
        self.last_status_message = f"Graphics preset set to {self.graphics.preset}"
        return self.graphics.preset

    def cycle_resolution(self, forward: bool = True) -> Tuple[int, int]:
        """Cycles display resolution across standard 16:9 desktop resolutions."""
        res_list = list(SUPPORTED_RESOLUTIONS)
        curr = self.graphics.resolution
        idx = res_list.index(curr) if curr in res_list else 0
        next_idx = (idx + (1 if forward else -1)) % len(res_list)
        self.graphics.resolution = res_list[next_idx]
        self.graphics.preset = GraphicsPreset.CUSTOM.value
        self.graphics.validate()
        return self.graphics.resolution

    def cycle_msaa(self) -> int:
        """Cycles MSAA anti-aliasing samples (0 -> 2 -> 4 -> 8 -> 0)."""
        msaa_list = list(VALID_MSAA_SAMPLES)
        idx = msaa_list.index(self.graphics.msaa_samples) if self.graphics.msaa_samples in msaa_list else 0
        self.graphics.msaa_samples = msaa_list[(idx + 1) % len(msaa_list)]
        self.graphics.preset = GraphicsPreset.CUSTOM.value
        return self.graphics.msaa_samples

    def adjust_fov(self, delta_deg: float) -> float:
        """Adjusts baseline vertical cockpit FOV within [60.0, 75.0] deg."""
        self.graphics.base_fov_deg += float(delta_deg)
        self.graphics.validate()
        return self.graphics.base_fov_deg

    def toggle_rear_mirrors(self) -> bool:
        """Toggles real-time dual rear-view wing mirror FBO rendering."""
        self.graphics.rear_mirrors_enabled = not self.graphics.rear_mirrors_enabled
        self.graphics.preset = GraphicsPreset.CUSTOM.value
        return self.graphics.rear_mirrors_enabled

    def toggle_motion_blur(self) -> bool:
        """Toggles high-speed radial motion blur."""
        self.graphics.motion_blur_enabled = not self.graphics.motion_blur_enabled
        self.graphics.preset = GraphicsPreset.CUSTOM.value
        return self.graphics.motion_blur_enabled

    def cycle_halo_opacity(self) -> float:
        """Cycles Halo central strut opacity (100% -> 35% -> 15% -> 100%)."""
        return self.graphics.cycle_halo_opacity()

    def get_graphics_menu_rows(self) -> List[Dict[str, str]]:
        """Returns structured rows for rendering the Graphics Settings screen."""
        g = self.graphics
        return [
            {"label": "Quality Preset", "value": g.preset},
            {"label": "Resolution", "value": f"{g.resolution[0]}x{g.resolution[1]}"},
            {"label": "Fullscreen", "value": "ON" if g.fullscreen else "OFF"},
            {"label": "VSync", "value": "ON" if g.vsync else "OFF"},
            {"label": "FPS Limit", "value": f"{g.fps_limit} FPS" if g.fps_limit > 0 else "UNCAPPED"},
            {"label": "Anti-Aliasing (MSAA)", "value": f"{g.msaa_samples}x" if g.msaa_samples > 0 else "OFF"},
            {"label": "Cockpit Base FOV", "value": f"{g.base_fov_deg:.1f} deg"},
            {"label": "Dynamic Speed FOV", "value": "ON" if g.dynamic_fov_enabled else "OFF"},
            {"label": "Rear-View Mirrors (FBO)", "value": "ON" if g.rear_mirrors_enabled else "OFF"},
            {"label": "Radial Motion Blur", "value": "ON" if g.motion_blur_enabled else "OFF"},
            {"label": "Camera Shake Intensity", "value": f"{int(round(g.camera_shake_intensity * 100))}%"},
            {"label": "Halo Pillar Opacity", "value": f"{int(round(g.halo_opacity * 100))}%"},
        ]

    # ------------------------------------------------------------------
    # Keybinding & Controller Remapping UI Actions
    # ------------------------------------------------------------------
    def start_key_rebind(self, action: str | ControlAction, slot: str = "primary") -> None:
        """Starts interactive key/button capture for the selected cockpit action."""
        self.bindings.start_capture(action, slot=slot)
        self.last_status_message = f"Press any key/button for {action} ({slot})..."

    def handle_key_capture_event(self, event: Any) -> Optional[Tuple[str, str, Optional[str]]]:
        """Processes a pygame event during interactive keybinding capture."""
        res = self.bindings.process_capture_event(event)
        if res is not None:
            act, new_key, swapped = res
            if swapped:
                self.last_status_message = f"Bound {act} -> {new_key} (swapped with {swapped})"
            else:
                self.last_status_message = f"Bound {act} -> {new_key}"
        return res

    def rebind_action_key(
        self,
        action: str | ControlAction,
        new_key: str,
        secondary: bool = False,
    ) -> Optional[str]:
        """Directly rebinds `action` to `new_key` with automatic conflict swap."""
        swapped = self.bindings.rebind_key(action, new_key, secondary=secondary, resolve_conflict=True)
        if swapped:
            self.last_status_message = f"Bound {action} -> {new_key} (swapped with {swapped})"
        else:
            self.last_status_message = f"Bound {action} -> {new_key}"
        return swapped

    def reset_keybindings(self) -> None:
        """Resets all keybindings and analog sensitivity settings to factory defaults."""
        self.bindings.reset_to_defaults()
        self.last_status_message = "Keybindings restored to factory defaults."

    def get_keybinding_menu_rows(self) -> List[Dict[str, str]]:
        """Returns structured rows for rendering the Keybinding Remapping screen."""
        rows: List[Dict[str, str]] = []
        for act in ControlAction:
            key = act.value
            p_key = self.bindings.primary_keys.get(key, "-")
            s_key = self.bindings.secondary_keys.get(key, "-")
            gp_btn = self.bindings.gamepad_buttons.get(key)
            gp_str = f"BTN_{gp_btn}" if gp_btn is not None else "AXIS"
            rows.append({
                "action": key,
                "label": ACTION_DISPLAY_NAMES.get(key, key),
                "primary": p_key,
                "secondary": s_key,
                "gamepad": gp_str,
            })
        return rows

    def save_all_settings(self) -> Tuple[Path, Path]:
        """Persists both GraphicsSettings and KeyBindingConfig to JSON files."""
        self.config_dir.mkdir(parents=True, exist_ok=True)
        g_path = self.graphics.save(self.config_dir / "graphics_settings.json")
        b_path = self.bindings.save(self.config_dir / "keybindings.json")
        return (g_path, b_path)
