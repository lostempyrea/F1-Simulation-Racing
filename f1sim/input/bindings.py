"""
f1sim/input/bindings.py
=======================
Configurable keyboard and gamepad input bindings with interactive key/button
capture, automatic conflict detection & swap resolution, analog sensitivity
settings, and JSON persistence.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class ControlAction(str, Enum):
    """Canonical cockpit & simulation control actions."""
    THROTTLE = "throttle"
    BRAKE = "brake"
    STEER_LEFT = "steer_left"
    STEER_RIGHT = "steer_right"
    GEAR_UP = "gear_up"
    GEAR_DOWN = "gear_down"
    DRS = "drs"
    IGNITION = "ignition"
    BRAKE_BIAS_INC = "brake_bias_inc"
    BRAKE_BIAS_DEC = "brake_bias_dec"
    TRANSMISSION_TOGGLE = "transmission_toggle"
    HALO_TOGGLE = "halo_toggle"
    PAUSE = "pause"


DEFAULT_PRIMARY_KEYS: Dict[str, str] = {
    ControlAction.THROTTLE.value: "W",
    ControlAction.BRAKE.value: "S",
    ControlAction.STEER_LEFT.value: "A",
    ControlAction.STEER_RIGHT.value: "D",
    ControlAction.GEAR_UP.value: "E",
    ControlAction.GEAR_DOWN.value: "Q",
    ControlAction.DRS.value: "F",
    ControlAction.IGNITION.value: "I",
    ControlAction.BRAKE_BIAS_INC.value: "]",
    ControlAction.BRAKE_BIAS_DEC.value: "[",
    ControlAction.TRANSMISSION_TOGGLE.value: "T",
    ControlAction.HALO_TOGGLE.value: "H",
    ControlAction.PAUSE.value: "ESCAPE",
}

DEFAULT_SECONDARY_KEYS: Dict[str, str] = {
    ControlAction.THROTTLE.value: "UP",
    ControlAction.BRAKE.value: "DOWN",
    ControlAction.STEER_LEFT.value: "LEFT",
    ControlAction.STEER_RIGHT.value: "RIGHT",
    ControlAction.GEAR_UP.value: "PAGEUP",
    ControlAction.GEAR_DOWN.value: "PAGEDOWN",
    ControlAction.DRS.value: "SPACE",
    ControlAction.IGNITION.value: "RETURN",
    ControlAction.BRAKE_BIAS_INC.value: "EQUALS",
    ControlAction.BRAKE_BIAS_DEC.value: "MINUS",
    ControlAction.TRANSMISSION_TOGGLE.value: "TAB",
    ControlAction.HALO_TOGGLE.value: "V",
    ControlAction.PAUSE.value: "P",
}

# Standard SDL2 / XInput button indices:
# 0: A (South), 1: B (East), 2: X (West), 3: Y (North),
# 4: LB (L1), 5: RB (R1), 6: Back/Select, 7: Start,
# 8: LStick, 9: RStick, 11: DPad-Up, 12: DPad-Down
DEFAULT_GAMEPAD_BUTTONS: Dict[str, int] = {
    ControlAction.GEAR_UP.value: 5,             # RB (Right Shoulder)
    ControlAction.GEAR_DOWN.value: 4,           # LB (Left Shoulder)
    ControlAction.DRS.value: 3,                 # Y / Triangle
    ControlAction.IGNITION.value: 0,            # A / Cross
    ControlAction.BRAKE_BIAS_INC.value: 11,     # D-Pad Up
    ControlAction.BRAKE_BIAS_DEC.value: 12,     # D-Pad Down
    ControlAction.TRANSMISSION_TOGGLE.value: 6, # Back / Select
    ControlAction.HALO_TOGGLE.value: 2,         # X / Square
    ControlAction.PAUSE.value: 7,               # Start / Options
}

DEFAULT_GAMEPAD_AXES: Dict[str, int] = {
    "steer_axis": 0,     # Left Stick X (-1.0 left .. +1.0 right)
    "brake_axis": 4,     # Left Trigger (LT / L2)
    "throttle_axis": 5,  # Right Trigger (RT / R2)
}

ACTION_DISPLAY_NAMES: Dict[str, str] = {
    ControlAction.THROTTLE.value: "Throttle (Accelerate)",
    ControlAction.BRAKE.value: "Brake / Reverse",
    ControlAction.STEER_LEFT.value: "Steer Left",
    ControlAction.STEER_RIGHT.value: "Steer Right",
    ControlAction.GEAR_UP.value: "Upshift (+Gear)",
    ControlAction.GEAR_DOWN.value: "Downshift (-Gear)",
    ControlAction.DRS.value: "DRS Flap Toggle",
    ControlAction.IGNITION.value: "Engine Ignition / Start",
    ControlAction.BRAKE_BIAS_INC.value: "Brake Bias Forward (+%)",
    ControlAction.BRAKE_BIAS_DEC.value: "Brake Bias Rearward (-%)",
    ControlAction.TRANSMISSION_TOGGLE.value: "Transmission Mode (Auto/Manual)",
    ControlAction.HALO_TOGGLE.value: "Halo Center Pillar Opacity",
    ControlAction.PAUSE.value: "Pause / Menu",
}


def normalize_key_name(key_name: str) -> str:
    """Normalizes a human-readable key string to canonical uppercase format."""
    if not isinstance(key_name, str):
        raise ValueError(f"Key name must be a string, got {type(key_name)!r}")
    cleaned = key_name.strip().upper()
    if not cleaned:
        raise ValueError("Key name cannot be empty")
    aliases = {
        "ESC": "ESCAPE",
        "ENTER": "RETURN",
        "PGUP": "PAGEUP",
        "PGDN": "PAGEDOWN",
        "LEFTBRACKET": "[",
        "RIGHTBRACKET": "]",
        "PLUS": "EQUALS",
    }
    return aliases.get(cleaned, cleaned)


def get_pygame_keycode(key_name: str) -> int:
    """
    Maps a canonical key name (e.g. 'W', '[', 'ESCAPE', 'PAGEUP') to its
    pygame key constant integer without requiring an initialized display.
    """
    import pygame
    norm = normalize_key_name(key_name)
    special_map = {
        "[": pygame.K_LEFTBRACKET,
        "]": pygame.K_RIGHTBRACKET,
        "ESCAPE": pygame.K_ESCAPE,
        "SPACE": pygame.K_SPACE,
        "RETURN": pygame.K_RETURN,
        "TAB": pygame.K_TAB,
        "UP": pygame.K_UP,
        "DOWN": pygame.K_DOWN,
        "LEFT": pygame.K_LEFT,
        "RIGHT": pygame.K_RIGHT,
        "PAGEUP": pygame.K_PAGEUP,
        "PAGEDOWN": pygame.K_PAGEDOWN,
        "EQUALS": pygame.K_EQUALS,
        "MINUS": pygame.K_MINUS,
        "LSHIFT": pygame.K_LSHIFT,
        "RSHIFT": pygame.K_RSHIFT,
        "LCTRL": pygame.K_LCTRL,
        "RCTRL": pygame.K_RCTRL,
        "LALT": pygame.K_LALT,
        "RALT": pygame.K_RALT,
        "BACKSPACE": pygame.K_BACKSPACE,
    }
    if norm in special_map:
        return int(special_map[norm])
    if len(norm) == 1 and norm.isalpha():
        attr = f"K_{norm.lower()}"
        if hasattr(pygame, attr):
            return int(getattr(pygame, attr))
    if len(norm) == 1 and norm.isdigit():
        attr = f"K_{norm}"
        if hasattr(pygame, attr):
            return int(getattr(pygame, attr))
    attr_upper = f"K_{norm}"
    if hasattr(pygame, attr_upper):
        return int(getattr(pygame, attr_upper))
    attr_lower = f"K_{norm.lower()}"
    if hasattr(pygame, attr_lower):
        return int(getattr(pygame, attr_lower))
    raise ValueError(f"Unrecognized key name: {key_name!r}")


def keycode_to_canonical_name(keycode: int) -> str:
    """Converts a pygame keycode integer into a canonical key name."""
    import pygame
    reverse_special = {
        pygame.K_LEFTBRACKET: "[",
        pygame.K_RIGHTBRACKET: "]",
        pygame.K_ESCAPE: "ESCAPE",
        pygame.K_SPACE: "SPACE",
        pygame.K_RETURN: "RETURN",
        pygame.K_TAB: "TAB",
        pygame.K_UP: "UP",
        pygame.K_DOWN: "DOWN",
        pygame.K_LEFT: "LEFT",
        pygame.K_RIGHT: "RIGHT",
        pygame.K_PAGEUP: "PAGEUP",
        pygame.K_PAGEDOWN: "PAGEDOWN",
        pygame.K_EQUALS: "EQUALS",
        pygame.K_MINUS: "MINUS",
        pygame.K_LSHIFT: "LSHIFT",
        pygame.K_RSHIFT: "RSHIFT",
        pygame.K_LCTRL: "LCTRL",
        pygame.K_RCTRL: "RCTRL",
        pygame.K_LALT: "LALT",
        pygame.K_RALT: "RALT",
        pygame.K_BACKSPACE: "BACKSPACE",
    }
    if keycode in reverse_special:
        return reverse_special[keycode]
    raw_name = pygame.key.name(int(keycode))
    return normalize_key_name(raw_name)


@dataclass
class KeyBindingConfig:
    """
    Complete user-customizable keybinding and controller configuration.
    Supports primary keys, secondary keys, gamepad buttons/axes, analog
    steering sensitivity, deadzone, cubic gamma, conflict resolution, and
    JSON save/load persistence.
    """
    primary_keys: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_PRIMARY_KEYS))
    secondary_keys: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_SECONDARY_KEYS))
    gamepad_buttons: Dict[str, int] = field(default_factory=lambda: dict(DEFAULT_GAMEPAD_BUTTONS))
    gamepad_axes: Dict[str, int] = field(default_factory=lambda: dict(DEFAULT_GAMEPAD_AXES))
    steering_sensitivity: float = 1.0
    steering_deadzone: float = 0.08
    steering_cubic_gamma: float = 0.45
    trigger_deadzone: float = 0.02
    invert_steering: bool = False

    # Interactive capture state (transient)
    _capture_target_action: Optional[str] = field(default=None, init=False, repr=False)
    _capture_slot: str = field(default="primary", init=False, repr=False)

    def __post_init__(self) -> None:
        self.validate()

    @staticmethod
    def _resolve_action_key(action: str | ControlAction) -> str:
        if isinstance(action, ControlAction):
            return action.value
        act_str = str(action).strip().lower()
        valid = {a.value for a in ControlAction}
        if act_str not in valid:
            raise ValueError(f"Unknown ControlAction {action!r}. Valid actions: {sorted(valid)}")
        return act_str

    def validate(self) -> None:
        """Validates and normalizes all keybindings and analog parameters."""
        for act in ControlAction:
            if act.value not in self.primary_keys:
                self.primary_keys[act.value] = DEFAULT_PRIMARY_KEYS[act.value]
            else:
                self.primary_keys[act.value] = normalize_key_name(self.primary_keys[act.value])

            if act.value not in self.secondary_keys:
                self.secondary_keys[act.value] = DEFAULT_SECONDARY_KEYS[act.value]
            else:
                self.secondary_keys[act.value] = normalize_key_name(self.secondary_keys[act.value])

        for act_key, default_btn in DEFAULT_GAMEPAD_BUTTONS.items():
            if act_key not in self.gamepad_buttons:
                self.gamepad_buttons[act_key] = default_btn
            else:
                self.gamepad_buttons[act_key] = int(self.gamepad_buttons[act_key])

        for axis_key, default_axis in DEFAULT_GAMEPAD_AXES.items():
            if axis_key not in self.gamepad_axes:
                self.gamepad_axes[axis_key] = default_axis
            else:
                self.gamepad_axes[axis_key] = int(self.gamepad_axes[axis_key])

        self.steering_sensitivity = max(0.2, min(2.5, float(self.steering_sensitivity)))
        self.steering_deadzone = max(0.0, min(0.40, float(self.steering_deadzone)))
        self.steering_cubic_gamma = max(0.0, min(1.0, float(self.steering_cubic_gamma)))
        self.trigger_deadzone = max(0.0, min(0.30, float(self.trigger_deadzone)))
        self.invert_steering = bool(self.invert_steering)

    def get_key(self, action: str | ControlAction, secondary: bool = False) -> str:
        """Returns the bound key name for the given action."""
        act_key = self._resolve_action_key(action)
        table = self.secondary_keys if secondary else self.primary_keys
        return table[act_key]

    def get_gamepad_button(self, action: str | ControlAction) -> Optional[int]:
        """Returns the bound gamepad button index for the given action, if any."""
        act_key = self._resolve_action_key(action)
        return self.gamepad_buttons.get(act_key)

    def rebind_key(
        self,
        action: str | ControlAction,
        new_key: str,
        secondary: bool = False,
        resolve_conflict: bool = True,
    ) -> Optional[str]:
        """
        Binds `action` to `new_key`.
        If another action already uses `new_key` in the same slot (primary/secondary)
        and `resolve_conflict` is True, automatically swaps the previous key of `action`
        into the conflicting action and returns the conflicting action's name.
        """
        act_key = self._resolve_action_key(action)
        norm_new = normalize_key_name(new_key)
        # Validate that pygame recognizes the key
        _ = get_pygame_keycode(norm_new)

        table = self.secondary_keys if secondary else self.primary_keys
        old_key = table[act_key]
        conflicting_action: Optional[str] = None

        for other_act, bound_key in table.items():
            if other_act != act_key and bound_key == norm_new:
                conflicting_action = other_act
                break

        if conflicting_action is not None and resolve_conflict:
            table[conflicting_action] = old_key

        table[act_key] = norm_new
        return conflicting_action

    def rebind_gamepad_button(
        self,
        action: str | ControlAction,
        button_index: int,
        resolve_conflict: bool = True,
    ) -> Optional[str]:
        """
        Binds `action` to `button_index` on the gamepad.
        Swaps with any conflicting action if `resolve_conflict` is True.
        """
        act_key = self._resolve_action_key(action)
        btn = int(button_index)
        if btn < 0 or btn > 31:
            raise ValueError(f"Gamepad button index out of range [0, 31]: {btn}")

        old_btn = self.gamepad_buttons.get(act_key)
        conflicting_action: Optional[str] = None
        for other_act, bound_btn in self.gamepad_buttons.items():
            if other_act != act_key and bound_btn == btn:
                conflicting_action = other_act
                break

        if conflicting_action is not None and resolve_conflict and old_btn is not None:
            self.gamepad_buttons[conflicting_action] = old_btn

        self.gamepad_buttons[act_key] = btn
        return conflicting_action

    def detect_conflicts(self) -> List[Tuple[str, str, str]]:
        """
        Detects any duplicate key or gamepad button bindings.
        Returns a list of tuples: (slot_type, key_or_button, action_pair_str).
        """
        conflicts: List[Tuple[str, str, str]] = []
        seen_primary: Dict[str, str] = {}
        for act, key in self.primary_keys.items():
            if key in seen_primary:
                conflicts.append(("primary_key", key, f"{seen_primary[key]} <-> {act}"))
            else:
                seen_primary[key] = act

        seen_secondary: Dict[str, str] = {}
        for act, key in self.secondary_keys.items():
            if key in seen_secondary:
                conflicts.append(("secondary_key", key, f"{seen_secondary[key]} <-> {act}"))
            else:
                seen_secondary[key] = act

        seen_buttons: Dict[int, str] = {}
        for act, btn in self.gamepad_buttons.items():
            if btn in seen_buttons:
                conflicts.append(("gamepad_button", str(btn), f"{seen_buttons[btn]} <-> {act}"))
            else:
                seen_buttons[btn] = act

        return conflicts

    def start_capture(self, action: str | ControlAction, slot: str = "primary") -> None:
        """Enters interactive key/button capture mode for a specific control action."""
        act_key = self._resolve_action_key(action)
        if slot not in ("primary", "secondary", "gamepad"):
            raise ValueError(f"Invalid capture slot {slot!r}")
        self._capture_target_action = act_key
        self._capture_slot = slot

    def cancel_capture(self) -> None:
        """Cancels an active interactive key/button capture."""
        self._capture_target_action = None

    @property
    def is_capturing(self) -> bool:
        """Returns True if waiting for the user to press a key or button to bind."""
        return self._capture_target_action is not None

    @property
    def capture_target(self) -> Optional[Tuple[str, str]]:
        """Returns (action, slot) currently being captured, or None."""
        if self._capture_target_action is None:
            return None
        return (self._capture_target_action, self._capture_slot)

    def process_capture_event(self, event: Any) -> Optional[Tuple[str, str, Optional[str]]]:
        """
        Processes a pygame event (KEYDOWN or JOYBUTTONDOWN) while in capture mode.
        Returns (action, new_binding_str, swapped_conflict_action) when captured, or None.
        """
        if self._capture_target_action is None:
            return None
        import pygame
        act = self._capture_target_action
        slot = self._capture_slot

        if slot in ("primary", "secondary") and getattr(event, "type", None) == pygame.KEYDOWN:
            key_name = keycode_to_canonical_name(int(event.key))
            conflict = self.rebind_key(
                act,
                key_name,
                secondary=(slot == "secondary"),
                resolve_conflict=True,
            )
            self._capture_target_action = None
            return (act, key_name, conflict)

        if slot == "gamepad" and getattr(event, "type", None) == pygame.JOYBUTTONDOWN:
            btn_idx = int(event.button)
            conflict = self.rebind_gamepad_button(act, btn_idx, resolve_conflict=True)
            self._capture_target_action = None
            return (act, f"BTN_{btn_idx}", conflict)

        return None

    def reset_to_defaults(self) -> None:
        """Restores all keyboard, gamepad, and sensitivity settings to factory defaults."""
        self.primary_keys = dict(DEFAULT_PRIMARY_KEYS)
        self.secondary_keys = dict(DEFAULT_SECONDARY_KEYS)
        self.gamepad_buttons = dict(DEFAULT_GAMEPAD_BUTTONS)
        self.gamepad_axes = dict(DEFAULT_GAMEPAD_AXES)
        self.steering_sensitivity = 1.0
        self.steering_deadzone = 0.08
        self.steering_cubic_gamma = 0.45
        self.trigger_deadzone = 0.02
        self.invert_steering = False
        self._capture_target_action = None

    def to_dict(self) -> Dict[str, Any]:
        """Serializes the binding configuration to a JSON-compatible dictionary."""
        return {
            "primary_keys": dict(self.primary_keys),
            "secondary_keys": dict(self.secondary_keys),
            "gamepad_buttons": dict(self.gamepad_buttons),
            "gamepad_axes": dict(self.gamepad_axes),
            "steering_sensitivity": float(self.steering_sensitivity),
            "steering_deadzone": float(self.steering_deadzone),
            "steering_cubic_gamma": float(self.steering_cubic_gamma),
            "trigger_deadzone": float(self.trigger_deadzone),
            "invert_steering": bool(self.invert_steering),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> KeyBindingConfig:
        """Deserializes a KeyBindingConfig from a dictionary, filling missing keys safely."""
        p_keys = dict(DEFAULT_PRIMARY_KEYS)
        if isinstance(data.get("primary_keys"), dict):
            p_keys.update({str(k): str(v) for k, v in data["primary_keys"].items()})

        s_keys = dict(DEFAULT_SECONDARY_KEYS)
        if isinstance(data.get("secondary_keys"), dict):
            s_keys.update({str(k): str(v) for k, v in data["secondary_keys"].items()})

        gp_btns = dict(DEFAULT_GAMEPAD_BUTTONS)
        if isinstance(data.get("gamepad_buttons"), dict):
            gp_btns.update({str(k): int(v) for k, v in data["gamepad_buttons"].items()})

        gp_axes = dict(DEFAULT_GAMEPAD_AXES)
        if isinstance(data.get("gamepad_axes"), dict):
            gp_axes.update({str(k): int(v) for k, v in data["gamepad_axes"].items()})

        return cls(
            primary_keys=p_keys,
            secondary_keys=s_keys,
            gamepad_buttons=gp_btns,
            gamepad_axes=gp_axes,
            steering_sensitivity=float(data.get("steering_sensitivity", 1.0)),
            steering_deadzone=float(data.get("steering_deadzone", 0.08)),
            steering_cubic_gamma=float(data.get("steering_cubic_gamma", 0.45)),
            trigger_deadzone=float(data.get("trigger_deadzone", 0.02)),
            invert_steering=bool(data.get("invert_steering", False)),
        )

    def save(self, path: str | Path) -> Path:
        """Saves the keybinding configuration to a JSON file."""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return target

    @classmethod
    def load(cls, path: str | Path) -> KeyBindingConfig:
        """Loads a KeyBindingConfig from a JSON file, or returns defaults if missing."""
        target = Path(path)
        if not target.exists():
            return cls()
        raw = json.loads(target.read_text(encoding="utf-8"))
        if not isinstance(raw, dict):
            return cls()
        return cls.from_dict(raw)
