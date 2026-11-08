"""
f1sim/graphics/camera.py
========================
First-person Formula 1 cockpit camera at driver eye height (Y = 0.68 m,
Z = -0.25 m) with 68.0-degree baseline vertical FOV, speed-based dynamic
FOV expansion, damped spring-mass neck G-force displacement, braking pitch
dip, lateral cornering head-tilt roll, 25 Hz kerb rumble shudder, and
rear-view wing mirror frustum generation.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Optional, Tuple
import numpy as np

from f1sim.core.math3d import Mat3, Mat4, Vec3
from f1sim.graphics.settings import GraphicsSettings


@dataclass
class CockpitCameraConfig:
    """Physical and optical parameters for the F1 driver eye camera."""
    eye_x: float = 0.0
    eye_y: float = 0.68
    eye_z: float = -0.25
    nominal_fov_deg: float = 68.0
    min_fov_deg: float = 60.0
    max_fov_deg: float = 75.0
    dynamic_fov_max_delta_deg: float = 10.0
    near_plane: float = 0.08
    far_plane: float = 1200.0

    # Neck spring-damper parameters (omega_n = 22 rad/s, zeta = 0.72)
    neck_omega_n: float = 22.0
    neck_damping_ratio: float = 0.72
    longitudinal_g_compliance_m: float = 0.009   # 9 mm/G fore-aft excursion
    lateral_g_compliance_m: float = 0.008        # 8 mm/G lateral excursion
    max_neck_offset_m: float = 0.045             # +/- 45 mm headrest/HANS stop

    # Angular head pitch & cornering roll
    pitch_per_long_g_deg: float = 1.15           # Forward head nod under 5G braking
    roll_per_lat_g_deg: float = 2.60             # Apex lean into corner (deg/G)
    max_head_pitch_deg: float = 6.5
    max_head_roll_deg: float = 9.0

    # Kerb 25 Hz rumble & engine RPM micro-vibration
    kerb_rumble_freq_hz: float = 25.0
    kerb_rumble_amp_m: float = 0.006             # 6 mm vertical shudder on 35mm kerbs
    engine_vibe_amp_m: float = 0.0008            # 0.8 mm high-RPM harmonic buzz


class CockpitCamera:
    """
    First-person cockpit camera with realistic F1 neck kinematics and optical frustums.
    """

    def __init__(
        self,
        config: Optional[CockpitCameraConfig] = None,
        settings: Optional[GraphicsSettings] = None,
    ) -> None:
        self.cfg: CockpitCameraConfig = config if config is not None else CockpitCameraConfig()
        if settings is not None:
            self.apply_graphics_settings(settings)

        # Local driver eye offset relative to vehicle CG/cockpit datum
        self.base_eye_local: Vec3 = Vec3(self.cfg.eye_x, self.cfg.eye_y, self.cfg.eye_z)
        self.neck_offset: Vec3 = Vec3(0.0, 0.0, 0.0)
        self.neck_velocity: Vec3 = Vec3(0.0, 0.0, 0.0)

        self.head_pitch_rad: float = 0.0
        self.head_yaw_rad: float = 0.0
        self.head_roll_rad: float = 0.0
        self.current_fov_deg: float = self.cfg.nominal_fov_deg

        # World-space camera pose
        self.world_eye_pos: Vec3 = Vec3(self.cfg.eye_x, self.cfg.eye_y, self.cfg.eye_z)
        self.world_forward: Vec3 = Vec3(0.0, 0.0, 1.0)
        self.world_up: Vec3 = Vec3(0.0, 1.0, 0.0)

        # Intensity multipliers from GraphicsSettings
        self.shake_intensity: float = 1.0
        self.head_tilt_intensity: float = 1.0
        self.dynamic_fov_enabled: bool = True
        self._phase_time: float = 0.0

    @property
    def eye_y(self) -> float:
        """Driver nominal eye height above ground/reference plane (0.68 m)."""
        return self.cfg.eye_y

    @property
    def eye_z(self) -> float:
        """Driver nominal longitudinal eye offset (-0.25 m)."""
        return self.cfg.eye_z

    @property
    def nominal_fov(self) -> float:
        """Baseline vertical Field of View in degrees (68.0 deg)."""
        return self.cfg.nominal_fov_deg

    def apply_graphics_settings(self, settings: GraphicsSettings) -> None:
        """Synchronizes optical and motion parameters with `GraphicsSettings`."""
        self.cfg.nominal_fov_deg = max(
            self.cfg.min_fov_deg,
            min(self.cfg.max_fov_deg, float(settings.base_fov_deg)),
        )
        self.cfg.dynamic_fov_max_delta_deg = float(settings.max_fov_delta_deg)
        self.cfg.far_plane = float(settings.draw_distance_m)
        self.dynamic_fov_enabled = bool(settings.dynamic_fov_enabled)
        self.shake_intensity = float(settings.camera_shake_intensity)
        self.head_tilt_intensity = float(settings.head_g_tilt_intensity)
        self.current_fov_deg = self.cfg.nominal_fov_deg

    def reset(self) -> None:
        """Resets dynamic neck spring offsets, angles, and FOV to rest state."""
        self.neck_offset = Vec3(0.0, 0.0, 0.0)
        self.neck_velocity = Vec3(0.0, 0.0, 0.0)
        self.head_pitch_rad = 0.0
        self.head_yaw_rad = 0.0
        self.head_roll_rad = 0.0
        self.current_fov_deg = self.cfg.nominal_fov_deg
        self._phase_time = 0.0

    def update(
        self,
        dt: float,
        speed_kmh: float = 0.0,
        longitudinal_g: float = 0.0,
        lateral_g: float = 0.0,
        rpm: float = 4000.0,
        on_kerb: bool = False,
        steer_norm: float = 0.0,
        car_world_pos: Optional[Vec3] = None,
        car_yaw_rad: float = 0.0,
        car_pitch_rad: float = 0.0,
        car_roll_rad: float = 0.0,
    ) -> None:
        """
        Steps the second-order damped spring-mass neck model, kerb vibration,
        and speed-dependent dynamic FOV expansion.
        """
        dt_clamped = max(0.0, min(0.05, float(dt)))
        self._phase_time += dt_clamped

        # 1. Dynamic FOV expansion with speed: 68 deg at rest -> up to +10 deg at 350 km/h
        v_ratio = max(0.0, min(1.0, float(speed_kmh) / 350.0))
        if self.dynamic_fov_enabled:
            fov_delta = self.cfg.dynamic_fov_max_delta_deg * (v_ratio ** 1.35)
        else:
            fov_delta = 0.0
        target_fov = self.cfg.nominal_fov_deg + fov_delta
        if dt_clamped > 0.0:
            alpha_fov = min(1.0, dt_clamped * 10.0)
            self.current_fov_deg += (target_fov - self.current_fov_deg) * alpha_fov
        else:
            self.current_fov_deg = target_fov

        # 2. Second-order damped spring-mass neck displacement
        # Under braking (negative longitudinal_g), head pitches/moves forward (+Z in forward-Z frame)
        wn = self.cfg.neck_omega_n
        zeta = self.cfg.neck_damping_ratio
        target_x = float(np.clip(
            -lateral_g * self.cfg.lateral_g_compliance_m * self.head_tilt_intensity,
            -self.cfg.max_neck_offset_m,
            self.cfg.max_neck_offset_m,
        ))
        target_z = float(np.clip(
            -longitudinal_g * self.cfg.longitudinal_g_compliance_m * self.head_tilt_intensity,
            -self.cfg.max_neck_offset_m,
            self.cfg.max_neck_offset_m,
        ))

        # Kerb 25 Hz vertical excitation + high-RPM harmonic buzz
        rpm_ratio = max(0.0, min(1.0, (float(rpm) - 4000.0) / 11000.0))
        vibe_y = (
            self.cfg.engine_vibe_amp_m
            * rpm_ratio
            * math.sin(2.0 * math.pi * 85.0 * self._phase_time)
            * self.shake_intensity
        )
        if on_kerb:
            vibe_y += (
                self.cfg.kerb_rumble_amp_m
                * math.sin(2.0 * math.pi * self.cfg.kerb_rumble_freq_hz * self._phase_time)
                * self.shake_intensity
            )
        target_y = float(np.clip(vibe_y, -self.cfg.max_neck_offset_m, self.cfg.max_neck_offset_m))

        if dt_clamped > 0.0:
            # Semi-implicit Euler on neck spring-damper
            target_vec = Vec3(target_x, target_y, target_z)
            disp_err = self.neck_offset - target_vec
            accel = disp_err * (-(wn * wn)) - self.neck_velocity * (2.0 * zeta * wn)
            self.neck_velocity = self.neck_velocity + accel * dt_clamped
            self.neck_offset = self.neck_offset + self.neck_velocity * dt_clamped
            self.neck_offset = Vec3(
                float(np.clip(self.neck_offset.x, -self.cfg.max_neck_offset_m, self.cfg.max_neck_offset_m)),
                float(np.clip(self.neck_offset.y, -self.cfg.max_neck_offset_m, self.cfg.max_neck_offset_m)),
                float(np.clip(self.neck_offset.z, -self.cfg.max_neck_offset_m, self.cfg.max_neck_offset_m)),
            )

        # 3. Angular head pitch, apex roll, and subtle look-to-apex yaw
        pitch_deg = float(np.clip(
            -longitudinal_g * self.cfg.pitch_per_long_g_deg * self.head_tilt_intensity,
            -self.cfg.max_head_pitch_deg,
            self.cfg.max_head_pitch_deg,
        ))
        roll_deg = float(np.clip(
            lateral_g * self.cfg.roll_per_lat_g_deg * self.head_tilt_intensity,
            -self.cfg.max_head_roll_deg,
            self.cfg.max_head_roll_deg,
        ))
        yaw_deg = float(np.clip(steer_norm * 3.5 * self.head_tilt_intensity, -5.0, 5.0))

        self.head_pitch_rad = math.radians(pitch_deg)
        self.head_roll_rad = math.radians(roll_deg)
        self.head_yaw_rad = math.radians(yaw_deg)

        # 4. Compute world-space eye position and basis vectors
        local_eye = self.base_eye_local + self.neck_offset
        car_rot = Mat3.from_euler(car_pitch_rad, car_yaw_rad, car_roll_rad)
        base_pos = car_world_pos if car_world_pos is not None else Vec3(0.0, 0.0, 0.0)
        self.world_eye_pos = base_pos + car_rot.transform(local_eye)

        total_rot = car_rot @ Mat3.from_euler(self.head_pitch_rad, self.head_yaw_rad, self.head_roll_rad)
        # Vehicle local forward is +Z (or +X depending on caller convention; default cockpit mesh uses -Z/+Z).
        # In cockpit local coordinates, camera looks along -Z (OpenGL convention) or +Z; let's support both cleanly:
        # In our cockpit mesh & renderer, cockpit forward is -Z (standard right-handed OpenGL: +X right, +Y up, -Z forward).
        self.world_forward = total_rot.transform(Vec3(0.0, 0.0, -1.0)).normalized()
        self.world_up = total_rot.transform(Vec3(0.0, 1.0, 0.0)).normalized()

    def get_local_eye_position(self) -> Vec3:
        """Returns the current local eye position (base + dynamic neck displacement)."""
        return self.base_eye_local + self.neck_offset

    def get_view_matrix(self, cockpit_local: bool = False) -> Mat4:
        """
        Returns the 4x4 right-handed OpenGL View Matrix.
        If `cockpit_local` is True, returns the view matrix in vehicle cockpit space
        (used for rendering the monocoque, Halo, mirrors, and steering wheel).
        """
        if cockpit_local:
            eye = self.get_local_eye_position()
            head_rot = Mat3.from_euler(self.head_pitch_rad, self.head_yaw_rad, self.head_roll_rad)
            fwd = head_rot.transform(Vec3(0.0, 0.0, -1.0)).normalized()
            up = head_rot.transform(Vec3(0.0, 1.0, 0.0)).normalized()
            return Mat4.look_at(eye, eye + fwd, up)

        target = self.world_eye_pos + self.world_forward
        return Mat4.look_at(self.world_eye_pos, target, self.world_up)

    def get_projection_matrix(self, aspect_ratio: float = 16.0 / 9.0) -> Mat4:
        """Returns the 4x4 perspective projection matrix for the current FOV."""
        safe_aspect = max(0.25, float(aspect_ratio))
        return Mat4.perspective(
            fov_y_deg=self.current_fov_deg,
            aspect=safe_aspect,
            near=self.cfg.near_plane,
            far=self.cfg.far_plane,
        )

    def get_mirror_matrices(
        self,
        side: str = "left",
        aspect_ratio: float = 2.4,
        car_world_pos: Optional[Vec3] = None,
        car_yaw_rad: float = 0.0,
    ) -> Tuple[Mat4, Mat4]:
        """
        Computes the (view_matrix, projection_matrix) for the left (-0.48 m) or
        right (+0.48 m) rear-view wing mirror looking backward.
        """
        mirror_x = -0.48 if str(side).lower().startswith("l") else 0.48
        if car_world_pos is not None:
            local_mirror_pos = Vec3(mirror_x, 0.62, -0.58)
            car_rot = Mat3.from_euler(0.0, car_yaw_rad, 0.0)
            world_mirror_pos = car_world_pos + car_rot.transform(local_mirror_pos)
            splay_x = -0.08 if mirror_x < 0.0 else 0.08
            backward_dir = car_rot.transform(Vec3(splay_x, -0.02, 1.0)).normalized()
            up_dir = car_rot.transform(Vec3(0.0, 1.0, 0.0)).normalized()
        else:
            right_dir = self.world_forward.cross(self.world_up).normalized()
            world_mirror_pos = (
                self.world_eye_pos
                + right_dir * mirror_x
                + self.world_up * (-0.06)
                + self.world_forward * 0.33
            )
            splay_x = -0.08 if mirror_x < 0.0 else 0.08
            backward_dir = (self.world_forward * (-1.0) + right_dir * splay_x + self.world_up * (-0.02)).normalized()
            up_dir = self.world_up

        view_mat = Mat4.look_at(world_mirror_pos, world_mirror_pos + backward_dir, up_dir)
        proj_mat = Mat4.perspective(fov_y_deg=38.0, aspect=max(0.5, float(aspect_ratio)), near=0.2, far=400.0)
        return (view_mat, proj_mat)
