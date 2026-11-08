"""
f1sim/input/input_manager.py
============================
Unified dual-input subsystem combining Keyboard/Mouse (Pygame-ce) and
Gamepad/Controller (Windows XInput 1.4 + SDL2 Joystick) with seamless
hot-swapping, customizable keybindings, steering filtering, and direct
F1 Vehicle cockpit actuation.
"""
from __future__ import annotations
import ctypes
from dataclasses import dataclass
import math
from typing import Any, Dict, Optional, Sequence, Set

from f1sim.input.bindings import (
    ControlAction,
    KeyBindingConfig,
    get_pygame_keycode,
)
from f1sim.input.filters import (
    SteeringFilter,
    SteeringFilterConfig,
    apply_cubic_gamma,
    apply_deadzone,
    blend_inputs,
    normalize_trigger,
)


class _XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("wButtons", ctypes.c_ushort),
        ("bLeftTrigger", ctypes.c_ubyte),
        ("bRightTrigger", ctypes.c_ubyte),
        ("sThumbLX", ctypes.c_short),
        ("sThumbLY", ctypes.c_short),
        ("sThumbRX", ctypes.c_short),
        ("sThumbRY", ctypes.c_short),
    ]


class _XInputState(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", ctypes.c_ulong),
        ("Gamepad", _XInputGamepad),
    ]


@dataclass
class DriverInputState:
    """Snapshot of processed driver cockpit inputs for a single simulation frame."""
    throttle: float = 0.0
    brake: float = 0.0
    raw_steer: float = 0.0
    road_steer_rad: float = 0.0
    wheel_angle_rad: float = 0.0
    gear_up_pressed: bool = False
    gear_down_pressed: bool = False
    drs_pressed: bool = False
    ignition_pressed: bool = False
    brake_bias_inc_pressed: bool = False
    brake_bias_dec_pressed: bool = False
    transmission_toggle_pressed: bool = False
    halo_toggle_pressed: bool = False
    pause_pressed: bool = False
    active_device: str = "KEYBOARD"  # "KEYBOARD" or "GAMEPAD"


class InputManager:
    """
    Polls and processes Keyboard and Gamepad (XInput / SDL2) inputs using
    `KeyBindingConfig` and `SteeringFilter`.
    """

    def __init__(
        self,
        bindings: Optional[KeyBindingConfig] = None,
        steering_config: Optional[SteeringFilterConfig] = None,
    ) -> None:
        self.bindings: KeyBindingConfig = bindings if bindings is not None else KeyBindingConfig()
        if steering_config is None:
            steering_config = SteeringFilterConfig(
                deadzone=self.bindings.steering_deadzone,
                cubic_gamma=self.bindings.steering_cubic_gamma,
            )
        self.steering_filter: SteeringFilter = SteeringFilter(steering_config)
        self.state: DriverInputState = DriverInputState()

        # Edge-detection state for discrete cockpit actions
        self._prev_actions_held: Set[str] = set()
        self._xinput_dll: Optional[Any] = self._load_xinput_dll()

    @staticmethod
    def _load_xinput_dll() -> Optional[Any]:
        """Attempts to load Windows native XInput DLL for low-latency controller polling."""
        for dll_name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
            try:
                return getattr(ctypes.windll, dll_name)
            except Exception:
                continue
        return None

    def sync_filter_with_bindings(self) -> None:
        """Synchronizes SteeringFilter parameters with current KeyBindingConfig."""
        self.steering_filter.cfg.deadzone = self.bindings.steering_deadzone
        self.steering_filter.cfg.cubic_gamma = self.bindings.steering_cubic_gamma

    def reset(self) -> None:
        """Resets steering filter and discrete button edge states."""
        self.steering_filter.reset()
        self.state = DriverInputState()
        self._prev_actions_held.clear()

    def _is_key_pressed(self, pressed_seq: Any, key_name: str) -> bool:
        try:
            code = get_pygame_keycode(key_name)
            if isinstance(pressed_seq, dict):
                return bool(pressed_seq.get(code, False) or pressed_seq.get(key_name, False))
            if isinstance(pressed_seq, set):
                return (code in pressed_seq) or (key_name.upper() in pressed_seq)
            return bool(pressed_seq[code])
        except Exception:
            return False

    def _is_action_key_held(self, pressed_seq: Any, action: ControlAction) -> bool:
        p_key = self.bindings.get_key(action, secondary=False)
        s_key = self.bindings.get_key(action, secondary=True)
        return self._is_key_pressed(pressed_seq, p_key) or self._is_key_pressed(pressed_seq, s_key)

    def _poll_xinput(self, controller_index: int = 0) -> Optional[Dict[str, Any]]:
        """Polls native Windows XInput controller 0 if connected."""
        if self._xinput_dll is None:
            return None
        state = _XInputState()
        res = self._xinput_dll.XInputGetState(int(controller_index), ctypes.byref(state))
        if res != 0:  # ERROR_SUCCESS == 0
            return None
        gp = state.Gamepad
        # Normalize left thumbstick X [-32768..32767] -> [-1.0..1.0]
        lx = float(gp.sThumbLX) / (32767.0 if gp.sThumbLX >= 0 else 32768.0)
        lx = max(-1.0, min(1.0, lx))
        lt = normalize_trigger(float(gp.bLeftTrigger), 255.0, self.bindings.trigger_deadzone)
        rt = normalize_trigger(float(gp.bRightTrigger), 255.0, self.bindings.trigger_deadzone)

        # Map XInput bitmask to canonical SDL/XInput button indices
        w = int(gp.wButtons)
        buttons: Dict[int, bool] = {
            0: bool(w & 0x1000),   # A
            1: bool(w & 0x2000),   # B
            2: bool(w & 0x4000),   # X
            3: bool(w & 0x8000),   # Y
            4: bool(w & 0x0100),   # LB
            5: bool(w & 0x0200),   # RB
            6: bool(w & 0x0020),   # Back
            7: bool(w & 0x0010),   # Start
            8: bool(w & 0x0040),   # LThumb
            9: bool(w & 0x0080),   # RThumb
            11: bool(w & 0x0001),  # DPad Up
            12: bool(w & 0x0002),  # DPad Down
            13: bool(w & 0x0004),  # DPad Left
            14: bool(w & 0x0008),  # DPad Right
        }
        return {
            "steer": lx,
            "brake": lt,
            "throttle": rt,
            "buttons": buttons,
        }

    def update(
        self,
        speed_ms: float = 0.0,
        dt: float = 1.0 / 120.0,
        pressed_keys: Optional[Any] = None,
        gamepad_state: Optional[Dict[str, Any]] = None,
    ) -> DriverInputState:
        """
        Updates the driver input state from either provided virtual inputs
        (for deterministic testing) or live Pygame + XInput hardware polling.
        """
        self.sync_filter_with_bindings()

        if pressed_keys is None:
            try:
                import pygame
                if pygame.get_init() and pygame.display.get_init():
                    pressed_keys = pygame.key.get_pressed()
                else:
                    pressed_keys = set()
            except Exception:
                pressed_keys = set()

        if gamepad_state is None:
            gamepad_state = self._poll_xinput(0)

        # 1. Keyboard continuous channels
        kb_throttle = 1.0 if self._is_action_key_held(pressed_keys, ControlAction.THROTTLE) else 0.0
        kb_brake = 1.0 if self._is_action_key_held(pressed_keys, ControlAction.BRAKE) else 0.0
        kb_left = 1.0 if self._is_action_key_held(pressed_keys, ControlAction.STEER_LEFT) else 0.0
        kb_right = 1.0 if self._is_action_key_held(pressed_keys, ControlAction.STEER_RIGHT) else 0.0
        kb_steer = kb_left - kb_right  # Positive = Left, Negative = Right

        # 2. Gamepad continuous channels
        gp_throttle = 0.0
        gp_brake = 0.0
        gp_steer = 0.0
        gp_buttons: Dict[int, bool] = {}

        if gamepad_state is not None:
            raw_gp_steer = float(gamepad_state.get("steer", 0.0))
            # Convention: Left Stick left (-1.0) -> +1.0 left steer, or direct if caller passes signed steer
            gp_steer = apply_deadzone(raw_gp_steer, self.bindings.steering_deadzone, rescale=True)
            gp_steer = apply_cubic_gamma(gp_steer, self.bindings.steering_cubic_gamma)

            raw_gp_thr = float(gamepad_state.get("throttle", 0.0))
            raw_gp_brk = float(gamepad_state.get("brake", 0.0))
            max_trig = float(gamepad_state.get("trigger_max", 1.0))
            gp_throttle = normalize_trigger(raw_gp_thr, max_trig, self.bindings.trigger_deadzone)
            gp_brake = normalize_trigger(raw_gp_brk, max_trig, self.bindings.trigger_deadzone)
            gp_buttons = dict(gamepad_state.get("buttons", {}))

        # 3. Seamless hot-swapping & blending between Keyboard and Gamepad
        throttle = blend_inputs(kb_throttle, gp_throttle)
        brake = blend_inputs(kb_brake, gp_brake)
        using_gp_steer = abs(gp_steer) > abs(kb_steer)
        raw_steer = blend_inputs(kb_steer, gp_steer)

        gp_any_active = (
            using_gp_steer
            or gp_throttle > 0.01
            or gp_brake > 0.01
            or any(gp_buttons.values())
        )
        kb_any_active = (
            abs(kb_steer) > 0.01
            or kb_throttle > 0.01
            or kb_brake > 0.01
        )
        if gp_any_active and not kb_any_active:
            active_device = "GAMEPAD"
        elif kb_any_active and not gp_any_active:
            active_device = "KEYBOARD"
        else:
            active_device = "GAMEPAD" if using_gp_steer else self.state.active_device

        # 4. Run SteeringFilter (Note: gp_steer already had deadzone+gamma applied above,
        # so pass is_analog=False to avoid double-applying deadzone/gamma)
        road_steer_rad, wheel_angle_rad = self.steering_filter.update(
            raw_input=raw_steer,
            speed_ms=speed_ms,
            dt=dt,
            is_analog=False,
            sensitivity=self.bindings.steering_sensitivity,
            invert=self.bindings.invert_steering,
        )

        # 5. Edge-triggered discrete actions
        discrete_actions = (
            ControlAction.GEAR_UP,
            ControlAction.GEAR_DOWN,
            ControlAction.DRS,
            ControlAction.IGNITION,
            ControlAction.BRAKE_BIAS_INC,
            ControlAction.BRAKE_BIAS_DEC,
            ControlAction.TRANSMISSION_TOGGLE,
            ControlAction.HALO_TOGGLE,
            ControlAction.PAUSE,
        )
        currently_held: Set[str] = set()
        just_pressed: Dict[str, bool] = {}

        for act in discrete_actions:
            held_kb = self._is_action_key_held(pressed_keys, act)
            btn_idx = self.bindings.get_gamepad_button(act)
            held_gp = bool(gp_buttons.get(btn_idx, False)) if btn_idx is not None else False
            is_held = held_kb or held_gp
            if is_held:
                currently_held.add(act.value)
            just_pressed[act.value] = is_held and (act.value not in self._prev_actions_held)

        self._prev_actions_held = currently_held

        self.state = DriverInputState(
            throttle=float(max(0.0, min(1.0, throttle))),
            brake=float(max(0.0, min(1.0, brake))),
            raw_steer=float(raw_steer),
            road_steer_rad=float(road_steer_rad),
            wheel_angle_rad=float(wheel_angle_rad),
            gear_up_pressed=just_pressed[ControlAction.GEAR_UP.value],
            gear_down_pressed=just_pressed[ControlAction.GEAR_DOWN.value],
            drs_pressed=just_pressed[ControlAction.DRS.value],
            ignition_pressed=just_pressed[ControlAction.IGNITION.value],
            brake_bias_inc_pressed=just_pressed[ControlAction.BRAKE_BIAS_INC.value],
            brake_bias_dec_pressed=just_pressed[ControlAction.BRAKE_BIAS_DEC.value],
            transmission_toggle_pressed=just_pressed[ControlAction.TRANSMISSION_TOGGLE.value],
            halo_toggle_pressed=just_pressed[ControlAction.HALO_TOGGLE.value],
            pause_pressed=just_pressed[ControlAction.PAUSE.value],
            active_device=active_device,
        )
        return self.state

    def apply_to_vehicle(self, vehicle: Any, dt: float, in_drs_zone: bool = False) -> DriverInputState:
        """
        Applies the current `DriverInputState` to a `Vehicle` instance (shifts,
        DRS toggle, brake bias adjustments, transmission mode, and physics step).
        """
        st = self.state
        if st.transmission_toggle_pressed and hasattr(vehicle, "auto_shift"):
            vehicle.auto_shift = not bool(vehicle.auto_shift)

        if st.gear_up_pressed and hasattr(vehicle, "shift_up"):
            vehicle.shift_up()
        if st.gear_down_pressed and hasattr(vehicle, "shift_down"):
            vehicle.shift_down()

        if st.brake_bias_inc_pressed and hasattr(vehicle, "adjust_brake_bias"):
            vehicle.adjust_brake_bias(+0.005)
        elif st.brake_bias_inc_pressed and hasattr(vehicle, "brake_bias"):
            vehicle.brake_bias = float(vehicle.brake_bias) + 0.005

        if st.brake_bias_dec_pressed and hasattr(vehicle, "adjust_brake_bias"):
            vehicle.adjust_brake_bias(-0.005)
        elif st.brake_bias_dec_pressed and hasattr(vehicle, "brake_bias"):
            vehicle.brake_bias = float(vehicle.brake_bias) - 0.005

        if in_drs_zone and hasattr(vehicle, "drs_available"):
            vehicle.drs_available = True

        if st.drs_pressed:
            if hasattr(vehicle, "toggle_drs"):
                vehicle.toggle_drs()
            elif hasattr(vehicle, "drs_active"):
                if getattr(vehicle, "drs_available", False) or in_drs_zone:
                    vehicle.drs_active = not bool(vehicle.drs_active)

        if st.ignition_pressed and hasattr(vehicle, "powertrain"):
            pt = vehicle.powertrain
            if hasattr(pt, "advance_ignition"):
                pt.advance_ignition()

        if hasattr(vehicle, "step"):
            vehicle.step(
                throttle=st.throttle,
                brake=st.brake,
                steer=st.road_steer_rad,
                dt=dt,
            )
        return st
