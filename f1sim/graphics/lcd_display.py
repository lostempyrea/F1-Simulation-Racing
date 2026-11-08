"""
f1sim/graphics/lcd_display.py
=============================
McLaren PCU-8D Steering Wheel Telemetry LCD & 15-LED Shift Light Array.
Computes 15-LED RPM rev-light colors (Green / Amber / Red / 16 Hz Blue Strobe),
formats Gear ('R', 'N', '1'-'8'), Speed (km/h), DRS ('OFF', 'AVAILABLE', 'ACTIVE'),
Brake Bias ('56.5% F'), and Lap Delta, and renders a 512x320 RGBA texture buffer
for 3D steering wheel shader mapping.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Tuple
import numpy as np


RPM_THRESHOLDS: List[Tuple[int, int, str]] = [
    (10500, 11800, "GREEN"),
    (11800, 12800, "AMBER"),
    (12800, 13500, "RED"),
    (13500, 15000, "BLUE_STROBE"),
]

VALID_DRS_STATES: set[str] = {"OFF", "AVAILABLE", "ACTIVE"}

LED_COLOR_OFF: Tuple[int, int, int] = (24, 28, 34)
LED_COLOR_GREEN: Tuple[int, int, int] = (0, 235, 80)
LED_COLOR_AMBER: Tuple[int, int, int] = (255, 195, 0)
LED_COLOR_RED: Tuple[int, int, int] = (255, 35, 35)
LED_COLOR_BLUE: Tuple[int, int, int] = (45, 165, 255)

_FONT_5X7: Dict[str, Tuple[int, ...]] = {
    "0": (0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110),
    "1": (0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110),
    "2": (0b01110, 0b10001, 0b00001, 0b00110, 0b01000, 0b10000, 0b11111),
    "3": (0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110),
    "4": (0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010),
    "5": (0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110),
    "6": (0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110),
    "7": (0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000),
    "8": (0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110),
    "9": (0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100),
    "N": (0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001),
    "R": (0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001),
    "D": (0b11100, 0b10010, 0b10001, 0b10001, 0b10001, 0b10010, 0b11100),
    "S": (0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110),
    "F": (0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000),
    "K": (0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001),
    "M": (0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001),
    "H": (0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001),
    ".": (0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100),
    "%": (0b11001, 0b11010, 0b00010, 0b00100, 0b01000, 0b01011, 0b10011),
    "+": (0b00000, 0b00100, 0b00100, 0b11111, 0b00100, 0b00100, 0b00000),
    "-": (0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000),
    " ": (0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000),
}


def _draw_bitmap_text(
    buf: np.ndarray,
    text: str,
    x: int,
    y: int,
    scale: int,
    color: Tuple[int, int, int],
) -> None:
    """Renders crisp 5x7 pixel-font characters directly into an (H, W, 4) uint8 buffer."""
    h, w, _ = buf.shape
    cursor_x = int(x)
    for ch in text.upper():
        rows = _FONT_5X7.get(ch, _FONT_5X7[" "])
        for r_idx, row_bits in enumerate(rows):
            py0 = int(y + r_idx * scale)
            py1 = min(h, py0 + scale)
            if py0 >= h or py1 <= 0:
                continue
            py0 = max(0, py0)
            for c_idx in range(5):
                if (row_bits >> (4 - c_idx)) & 1:
                    px0 = int(cursor_x + c_idx * scale)
                    px1 = min(w, px0 + scale)
                    if px0 < w and px1 > 0:
                        buf[py0:py1, max(0, px0):px1, 0] = color[0]
                        buf[py0:py1, max(0, px0):px1, 1] = color[1]
                        buf[py0:py1, max(0, px0):px1, 2] = color[2]
        cursor_x += 6 * scale


def format_gear(gear: int) -> str:
    """Formats gear integer (-1..8) into PCU-8D character ('R', 'N', '1'..'8')."""
    g = int(gear)
    if g == -1:
        return "R"
    if g == 0:
        return "N"
    if 1 <= g <= 8:
        return str(g)
    return "N"


def compute_speed_kmh(speed_ms: float) -> float:
    """Converts vehicle longitudinal speed from m/s to km/h (v_ms * 3.6)."""
    return float(speed_ms) * 3.6


def resolve_drs_status(drs_active: bool = False, drs_available: bool = False) -> str:
    """Resolves canonical DRS status string in {'OFF', 'AVAILABLE', 'ACTIVE'}."""
    if bool(drs_active):
        return "ACTIVE"
    if bool(drs_available):
        return "AVAILABLE"
    return "OFF"


def format_brake_bias(brake_bias: float) -> str:
    """Formats front brake bias ratio (e.g. 0.565) as '56.5% F'."""
    bb = float(brake_bias)
    if bb <= 1.0:
        bb *= 100.0
    return f"{bb:.1f}% F"


def format_lap_delta(delta_sec: float) -> str:
    """Formats live lap time delta in seconds (e.g. -0.24 -> '-0.24s', +0.15 -> '+0.15s')."""
    d = float(delta_sec)
    sign = "+" if d >= 0.0 else "-"
    return f"{sign}{abs(d):.2f}s"


@dataclass
class PCU8DTelemetryState:
    """Structured snapshot of all telemetry displayed on the PCU-8D steering wheel screen."""
    rpm: float = 4000.0
    gear: int = 0
    gear_str: str = "N"
    speed_kmh: float = 0.0
    drs_status: str = "OFF"
    brake_bias_str: str = "56.5% F"
    lap_delta_str: str = "+0.00s"
    ers_soc_pct: float = 100.0
    auto_shift: bool = True
    led_colors: List[Tuple[int, int, int]] = field(default_factory=lambda: [LED_COLOR_OFF] * 15)
    active_led_count: int = 0
    shift_strobe_active: bool = False


class TelemetryLCDDisplay:
    """
    McLaren PCU-8D Steering Wheel LCD & 15-LED Shift Light Bar controller and renderer.
    """

    NUM_LEDS: int = 15

    def __init__(self, width: int = 512, height: int = 320) -> None:
        self.width: int = max(128, int(width))
        self.height: int = max(80, int(height))
        self.rpm_thresholds: List[Tuple[int, int, str]] = list(RPM_THRESHOLDS)
        self.state: PCU8DTelemetryState = PCU8DTelemetryState()
        self._rgba_buffer: np.ndarray = np.zeros((self.height, self.width, 4), dtype=np.uint8)
        self._elapsed_time: float = 0.0
        self._render_to_buffer()

    def compute_rev_lights(
        self,
        rpm: float,
        time_sec: Optional[float] = None,
    ) -> Tuple[List[Tuple[int, int, int]], int, bool]:
        """
        Computes the 15-LED RGB states for a given engine RPM:
        - LEDs 1..5  (idx 0..4):  GREEN  (10,500 - 11,800 RPM)
        - LEDs 6..10 (idx 5..9):  AMBER  (11,800 - 12,800 RPM)
        - LEDs 11..14 (idx 10..13): RED  (12,800 - 13,500 RPM)
        - LED 15 (idx 14) + Strobe: BLUE_STROBE (13,500 - 15,000 RPM at 16 Hz)
        """
        r = float(rpm)
        t = self._elapsed_time if time_sec is None else float(time_sec)

        if r < 10500.0:
            return ([LED_COLOR_OFF] * self.NUM_LEDS, 0, False)

        # Above 13,500 RPM: upshift blue strobe mode (16 Hz square wave)
        if r >= 13500.0:
            strobe_on = (int(t * 32.0) % 2) == 0
            if strobe_on:
                return ([LED_COLOR_BLUE] * self.NUM_LEDS, self.NUM_LEDS, True)
            # Even on the alternate half-cycle, keep LED 15 lit blue so shift cue is unmistakable
            leds_alt = [LED_COLOR_OFF] * 14 + [LED_COLOR_BLUE]
            return (leds_alt, 15, True)

        colors: List[Tuple[int, int, int]] = []
        active_count = 0

        # LEDs 0..4: Green (10500 .. 11800)
        for i in range(5):
            thresh = 10500.0 + (i / 5.0) * (11800.0 - 10500.0)
            if r >= thresh:
                colors.append(LED_COLOR_GREEN)
                active_count += 1
            else:
                colors.append(LED_COLOR_OFF)

        # LEDs 5..9: Amber (11800 .. 12800)
        for i in range(5):
            thresh = 11800.0 + (i / 5.0) * (12800.0 - 11800.0)
            if r >= thresh:
                colors.append(LED_COLOR_AMBER)
                active_count += 1
            else:
                colors.append(LED_COLOR_OFF)

        # LEDs 10..13: Red (12800 .. 13500)
        for i in range(4):
            thresh = 12800.0 + (i / 4.0) * (13500.0 - 12800.0)
            if r >= thresh:
                colors.append(LED_COLOR_RED)
                active_count += 1
            else:
                colors.append(LED_COLOR_OFF)

        # LED 14: Blue (13500+)
        colors.append(LED_COLOR_OFF)
        return (colors, active_count, False)

    def update(
        self,
        rpm: float = 4000.0,
        gear: int = 0,
        speed_ms: float = 0.0,
        drs_active: bool = False,
        drs_available: bool = False,
        brake_bias: float = 0.565,
        lap_delta_sec: float = 0.0,
        ers_soc_pct: float = 100.0,
        auto_shift: bool = True,
        dt: float = 1.0 / 60.0,
    ) -> PCU8DTelemetryState:
        """
        Updates the PCU-8D state and regenerates the 512x320 RGBA pixel buffer.
        """
        self._elapsed_time += max(0.0, float(dt))
        led_colors, active_count, strobe = self.compute_rev_lights(rpm, self._elapsed_time)

        self.state = PCU8DTelemetryState(
            rpm=float(rpm),
            gear=int(gear),
            gear_str=format_gear(gear),
            speed_kmh=compute_speed_kmh(speed_ms),
            drs_status=resolve_drs_status(drs_active, drs_available),
            brake_bias_str=format_brake_bias(brake_bias),
            lap_delta_str=format_lap_delta(lap_delta_sec),
            ers_soc_pct=float(max(0.0, min(100.0, ers_soc_pct))),
            auto_shift=bool(auto_shift),
            led_colors=led_colors,
            active_led_count=active_count,
            shift_strobe_active=strobe,
        )
        self._render_to_buffer()
        return self.state

    def update_from_vehicle(
        self,
        vehicle: Any,
        dt: float = 1.0 / 60.0,
        lap_delta_sec: float = 0.0,
    ) -> PCU8DTelemetryState:
        """Convenience updater extracting live telemetry directly from a `Vehicle` instance."""
        rpm = float(getattr(vehicle, "rpm", 4000.0))
        gear = int(getattr(vehicle, "gear", 0))
        vx = float(getattr(vehicle, "vx", 0.0))
        drs_active = bool(getattr(vehicle, "drs_active", False))
        drs_avail = bool(getattr(vehicle, "drs_available", False))
        bb = float(getattr(vehicle, "brake_bias", 0.565))
        auto_shift = bool(getattr(vehicle, "auto_shift", True))
        return self.update(
            rpm=rpm,
            gear=gear,
            speed_ms=vx,
            drs_active=drs_active,
            drs_available=drs_avail,
            brake_bias=bb,
            lap_delta_sec=lap_delta_sec,
            auto_shift=auto_shift,
            dt=dt,
        )

    def _render_to_buffer(self) -> None:
        """
        Renders the PCU-8D layout into `self._rgba_buffer` (height, width, 4) uint8.
        Works deterministically in headless mode using vectorized NumPy rasterization.
        """
        buf = self._rgba_buffer
        h, w, _ = buf.shape

        # 1. Carbon-black OLED background (12, 15, 20, 255)
        buf[:, :, 0] = 12
        buf[:, :, 1] = 15
        buf[:, :, 2] = 20
        buf[:, :, 3] = 255

        # Outer bezel border
        buf[0:3, :, :3] = (45, 55, 68)
        buf[h - 3:h, :, :3] = (45, 55, 68)
        buf[:, 0:3, :3] = (45, 55, 68)
        buf[:, w - 3:w, :3] = (45, 55, 68)

        # 2. Top 15-LED Shift Light Bar
        led_y0 = int(h * 0.04)
        led_y1 = int(h * 0.14)
        margin_x = int(w * 0.05)
        usable_w = w - 2 * margin_x
        slot_w = usable_w / self.NUM_LEDS
        for i, rgb in enumerate(self.state.led_colors):
            x0 = int(margin_x + i * slot_w + 2)
            x1 = int(margin_x + (i + 1) * slot_w - 2)
            if x1 > x0:
                buf[led_y0:led_y1, x0:x1, 0] = rgb[0]
                buf[led_y0:led_y1, x0:x1, 1] = rgb[1]
                buf[led_y0:led_y1, x0:x1, 2] = rgb[2]

        # 3. RPM Continuous Progress Bar below LEDs
        bar_y0 = int(h * 0.17)
        bar_y1 = int(h * 0.22)
        rpm_frac = max(0.0, min(1.0, (self.state.rpm - 4000.0) / 11000.0))
        fill_x1 = int(margin_x + usable_w * rpm_frac)
        buf[bar_y0:bar_y1, margin_x: margin_x + usable_w, :3] = (28, 34, 42)
        if fill_x1 > margin_x:
            bar_color = (
                LED_COLOR_BLUE if self.state.rpm >= 13500.0
                else (LED_COLOR_RED if self.state.rpm >= 12800.0 else (0, 210, 120))
            )
            buf[bar_y0:bar_y1, margin_x:fill_x1, :3] = bar_color

        # 4. Central Gear Box panel
        gear_y0, gear_y1 = int(h * 0.28), int(h * 0.78)
        gear_x0, gear_x1 = int(w * 0.36), int(w * 0.64)
        buf[gear_y0:gear_y1, gear_x0:gear_x1, :3] = (22, 27, 36)

        # Highlight center gear indicator color by state
        gear_accent = (255, 220, 60) if not self.state.shift_strobe_active else (60, 180, 255)
        buf[gear_y0:gear_y0 + 3, gear_x0:gear_x1, :3] = gear_accent
        buf[gear_y1 - 3:gear_y1, gear_x0:gear_x1, :3] = gear_accent

        # 5. DRS Status Indicator Box (Top-Left Panel)
        drs_y0, drs_y1 = int(h * 0.30), int(h * 0.48)
        drs_x0, drs_x1 = margin_x, int(w * 0.32)
        if self.state.drs_status == "ACTIVE":
            drs_color = (0, 230, 90)
        elif self.state.drs_status == "AVAILABLE":
            drs_color = (255, 190, 0)
        else:
            drs_color = (42, 48, 58)
        buf[drs_y0:drs_y1, drs_x0:drs_x1, :3] = drs_color

        # 6. Brake Bias Box (Bottom-Left Panel)
        bb_y0, bb_y1 = int(h * 0.54), int(h * 0.76)
        buf[bb_y0:bb_y1, drs_x0:drs_x1, :3] = (28, 36, 48)

        # 7. Speed & Lap Delta Box (Right Panel)
        spd_x0, spd_x1 = int(w * 0.68), w - margin_x
        buf[drs_y0:drs_y1, spd_x0:spd_x1, :3] = (26, 34, 46)
        delta_bg = (18, 65, 36) if self.state.lap_delta_str.startswith("-") else (68, 28, 28)
        buf[bb_y0:bb_y1, spd_x0:spd_x1, :3] = delta_bg

        # 8. Crisp 5x7 Bitmap Digits for Gear, Speed (KM/H), DRS, Brake Bias & RPM
        _draw_bitmap_text(buf, self.state.gear_str[:1], gear_x0 + 36, gear_y0 + 26, 15, gear_accent)
        spd_str = f"{int(round(self.state.speed_kmh)):3d}"
        _draw_bitmap_text(buf, spd_str, spd_x0 + 18, drs_y0 + 12, 5, (240, 248, 255))
        _draw_bitmap_text(buf, "DRS", drs_x0 + 24, drs_y0 + 14, 4, (255, 255, 255))
        _draw_bitmap_text(buf, self.state.brake_bias_str[:5], drs_x0 + 10, bb_y0 + 18, 4, (130, 210, 255))
        _draw_bitmap_text(buf, f"{int(round(self.state.rpm)):5d}", int(w * 0.36), int(h * 0.82), 4, (180, 220, 250))

    def get_rgba_bytes(self) -> bytes:
        """Returns the raw 512x320x4 RGBA byte buffer for OpenGL texture upload."""
        return self._rgba_buffer.tobytes()

    def get_rgba_array(self) -> np.ndarray:
        """Returns a view of the (H, W, 4) uint8 RGBA pixel array."""
        return self._rgba_buffer
