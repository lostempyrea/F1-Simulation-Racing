"""
f1sim/graphics/__init__.py
==========================
Public exports for F1 first-person 3D cockpit camera, procedural cockpit and
steering wheel meshes, PCU-8D telemetry LCD, OpenGL 3.3 shaders, renderer,
and user graphics settings.
"""
from f1sim.graphics.camera import (
    CockpitCamera,
    CockpitCameraConfig,
)
from f1sim.graphics.lcd_display import (
    LED_COLOR_AMBER,
    LED_COLOR_BLUE,
    LED_COLOR_GREEN,
    LED_COLOR_OFF,
    LED_COLOR_RED,
    PCU8DTelemetryState,
    RPM_THRESHOLDS,
    TelemetryLCDDisplay,
    VALID_DRS_STATES,
    compute_speed_kmh,
    format_brake_bias,
    format_gear,
    format_lap_delta,
    resolve_drs_status,
)
from f1sim.graphics.meshes import (
    CockpitGeometrySpec,
    CockpitMeshBuilder,
    MeshData,
    compute_wheel_model_matrix_4x4,
    compute_wheel_rotation_matrix_3x3,
)
from f1sim.graphics.renderer import (
    CockpitRenderer,
    mat4_to_gl_bytes,
)
from f1sim.graphics.settings import (
    GraphicsPreset,
    GraphicsSettings,
    SUPPORTED_RESOLUTIONS,
    VALID_MSAA_SAMPLES,
)
from f1sim.graphics.shaders import (
    COCKPIT_FRAGMENT_SHADER,
    COCKPIT_VERTEX_SHADER,
    TRACK_FRAGMENT_SHADER,
    TRACK_VERTEX_SHADER,
)

__all__ = [
    "COCKPIT_FRAGMENT_SHADER",
    "COCKPIT_VERTEX_SHADER",
    "CockpitCamera",
    "CockpitCameraConfig",
    "CockpitGeometrySpec",
    "CockpitMeshBuilder",
    "CockpitRenderer",
    "GraphicsPreset",
    "GraphicsSettings",
    "LED_COLOR_AMBER",
    "LED_COLOR_BLUE",
    "LED_COLOR_GREEN",
    "LED_COLOR_OFF",
    "LED_COLOR_RED",
    "MeshData",
    "PCU8DTelemetryState",
    "RPM_THRESHOLDS",
    "SUPPORTED_RESOLUTIONS",
    "TRACK_FRAGMENT_SHADER",
    "TRACK_VERTEX_SHADER",
    "TelemetryLCDDisplay",
    "VALID_DRS_STATES",
    "VALID_MSAA_SAMPLES",
    "compute_speed_kmh",
    "compute_wheel_model_matrix_4x4",
    "compute_wheel_rotation_matrix_3x3",
    "format_brake_bias",
    "format_gear",
    "format_lap_delta",
    "mat4_to_gl_bytes",
    "resolve_drs_status",
]
