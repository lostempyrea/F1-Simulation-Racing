"""
f1sim/input/__init__.py
=======================
Public exports for F1 cockpit dual-input handling, signal filters, and
customizable keyboard/gamepad keybindings.
"""
from f1sim.input.bindings import (
    ACTION_DISPLAY_NAMES,
    ControlAction,
    DEFAULT_GAMEPAD_AXES,
    DEFAULT_GAMEPAD_BUTTONS,
    DEFAULT_PRIMARY_KEYS,
    DEFAULT_SECONDARY_KEYS,
    KeyBindingConfig,
    get_pygame_keycode,
    keycode_to_canonical_name,
    normalize_key_name,
)
from f1sim.input.filters import (
    SteeringFilter,
    SteeringFilterConfig,
    apply_cubic_gamma,
    apply_deadzone,
    blend_inputs,
    compute_speed_attenuation,
    normalize_trigger,
)
from f1sim.input.input_manager import (
    DriverInputState,
    InputManager,
)

__all__ = [
    "ACTION_DISPLAY_NAMES",
    "ControlAction",
    "DEFAULT_GAMEPAD_AXES",
    "DEFAULT_GAMEPAD_BUTTONS",
    "DEFAULT_PRIMARY_KEYS",
    "DEFAULT_SECONDARY_KEYS",
    "DriverInputState",
    "InputManager",
    "KeyBindingConfig",
    "SteeringFilter",
    "SteeringFilterConfig",
    "apply_cubic_gamma",
    "apply_deadzone",
    "blend_inputs",
    "compute_speed_attenuation",
    "get_pygame_keycode",
    "keycode_to_canonical_name",
    "normalize_key_name",
    "normalize_trigger",
]
