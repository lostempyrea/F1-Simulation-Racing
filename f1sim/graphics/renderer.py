"""
f1sim/graphics/renderer.py
==========================
ModernGL 3.3 Core Profile 3D Renderer for the F1 Cockpit, Rotating Steering
Wheel, PCU-8D Telemetry LCD Texture, Rear-View Wing Mirror FBOs, and Track
Ribbon Geometry. Supports both windowed display contexts and standalone
offscreen OpenGL contexts for automated testing.
"""
from __future__ import annotations
from typing import Any, Dict, Optional, Tuple
import numpy as np

from f1sim.core.math3d import Mat4, Vec3
from f1sim.graphics.camera import CockpitCamera
from f1sim.graphics.lcd_display import TelemetryLCDDisplay
from f1sim.graphics.meshes import (
    CockpitGeometrySpec,
    CockpitMeshBuilder,
    MeshData,
    compute_wheel_model_matrix_4x4,
)
from f1sim.graphics.settings import GraphicsSettings
from f1sim.graphics.shaders import (
    COCKPIT_FRAGMENT_SHADER,
    COCKPIT_VERTEX_SHADER,
    TRACK_FRAGMENT_SHADER,
    TRACK_VERTEX_SHADER,
)


def mat4_to_gl_bytes(mat: Mat4) -> bytes:
    """Converts a row-major Mat4 into column-major float32 bytes for ModernGL uniforms."""
    arr_col = np.ascontiguousarray(mat.to_numpy(dtype=np.float32).T)
    return arr_col.tobytes()


class CockpitRenderer:
    """
    Manages OpenGL 3.3 shader programs, VAOs, PCU-8D LCD texture streaming,
    rear-view mirror FBOs, and cockpit + track draw calls.
    """

    def __init__(
        self,
        settings: Optional[GraphicsSettings] = None,
        ctx: Optional[Any] = None,
        standalone: bool = True,
        init_gl: bool = True,
    ) -> None:
        self.settings: GraphicsSettings = settings if settings is not None else GraphicsSettings()
        self.spec: CockpitGeometrySpec = CockpitGeometrySpec()
        self.camera: CockpitCamera = CockpitCamera(settings=self.settings)
        self.mesh_builder: CockpitMeshBuilder = CockpitMeshBuilder(self.spec)
        self.lcd: TelemetryLCDDisplay = TelemetryLCDDisplay(width=512, height=320)
        self.meshes: Dict[str, MeshData] = self.mesh_builder.build_all()

        self.ctx: Optional[Any] = ctx
        self.cockpit_prog: Optional[Any] = None
        self.track_prog: Optional[Any] = None
        self.vaos: Dict[str, Any] = {}
        self.lcd_texture: Optional[Any] = None
        self.offscreen_fbo: Optional[Any] = None
        self.left_mirror_fbo: Optional[Any] = None
        self.right_mirror_fbo: Optional[Any] = None
        self.left_mirror_tex: Optional[Any] = None
        self.right_mirror_tex: Optional[Any] = None
        self.frame_count: int = 0

        if init_gl:
            self.initialize_gl(ctx=ctx, standalone=standalone)

    def initialize_gl(self, ctx: Optional[Any] = None, standalone: bool = True) -> bool:
        """Initializes ModernGL 3.3 Core context, shaders, VAOs, LCD texture, and mirror FBOs."""
        import moderngl

        if ctx is not None:
            self.ctx = ctx
        elif self.ctx is None:
            if standalone:
                self.ctx = moderngl.create_standalone_context(require=330)
            else:
                self.ctx = moderngl.create_context(require=330)

        gl = self.ctx
        self.cockpit_prog = gl.program(
            vertex_shader=COCKPIT_VERTEX_SHADER,
            fragment_shader=COCKPIT_FRAGMENT_SHADER,
        )
        self.track_prog = gl.program(
            vertex_shader=TRACK_VERTEX_SHADER,
            fragment_shader=TRACK_FRAGMENT_SHADER,
        )

        # Upload cockpit & wheel VAOs
        self.vaos.clear()
        all_meshes = dict(self.meshes)
        all_meshes["ai_car"] = self.mesh_builder.build_ai_car_mesh()
        for name, mesh in all_meshes.items():
            v_bytes, i_bytes = mesh.to_bytes()
            vbo = gl.buffer(v_bytes)
            ibo = gl.buffer(i_bytes)
            vao = gl.vertex_array(
                self.cockpit_prog,
                [(vbo, "3f 3f 2f", "in_position", "in_normal", "in_uv")],
                index_buffer=ibo,
            )
            self.vaos[name] = vao

        # Create PCU-8D LCD texture (512x320 RGBA)
        self.lcd_texture = gl.texture(
            (self.lcd.width, self.lcd.height),
            4,
            self.lcd.get_rgba_bytes(),
        )
        self.lcd_texture.filter = (moderngl.LINEAR, moderngl.LINEAR)

        # Create Rear-View Mirror FBOs
        mw, mh = self.settings.mirror_resolution
        self.left_mirror_tex = gl.texture((mw, mh), 3)
        self.right_mirror_tex = gl.texture((mw, mh), 3)
        self.left_mirror_fbo = gl.framebuffer(
            color_attachments=[self.left_mirror_tex],
            depth_attachment=gl.depth_renderbuffer((mw, mh)),
        )
        self.right_mirror_fbo = gl.framebuffer(
            color_attachments=[self.right_mirror_tex],
            depth_attachment=gl.depth_renderbuffer((mw, mh)),
        )

        # Standalone offscreen framebuffer when running headless
        if standalone:
            w, h = self.settings.resolution
            color_tex = gl.texture((w, h), 4)
            depth_rb = gl.depth_renderbuffer((w, h))
            self.offscreen_fbo = gl.framebuffer(
                color_attachments=[color_tex],
                depth_attachment=depth_rb,
            )
        return True

    def apply_settings(self, settings: GraphicsSettings) -> None:
        """Applies updated `GraphicsSettings` at runtime."""
        self.settings = settings
        self.camera.apply_graphics_settings(settings)

    def upload_lcd_texture(self) -> None:
        """Streams the latest PCU-8D RGBA buffer to the GPU texture."""
        if self.lcd_texture is not None:
            self.lcd_texture.write(self.lcd.get_rgba_bytes())

    def upload_track_mesh(self, track_mesh: Any) -> bool:
        """Uploads a 12-float combined `MeshBufferData` from `TrackBuilder` into a ModernGL VAO."""
        if self.ctx is None or self.track_prog is None or track_mesh is None:
            return False
        verts = getattr(track_mesh, "vertices", None)
        inds = getattr(track_mesh, "indices", None)
        if verts is None or inds is None or len(verts) == 0 or len(inds) == 0:
            return False
        vbo = self.ctx.buffer(np.ascontiguousarray(verts, dtype=np.float32).tobytes())
        ibo = self.ctx.buffer(np.ascontiguousarray(inds, dtype=np.uint32).tobytes())
        self.vaos["track"] = self.ctx.vertex_array(
            self.track_prog,
            [(vbo, "3f 3f 2f 1f 3f", "in_position", "in_normal", "in_uv", "in_surface_id", "in_tangent")],
            index_buffer=ibo,
        )
        return True

    def render_frame(
        self,
        wheel_angle_rad: float = 0.0,
        car_world_pos: Optional[Vec3] = None,
        car_yaw_rad: float = 0.0,
        ai_cars: Optional[ list[Tuple[Mat4, Tuple[float, float, float]]] ] = None,
    ) -> None:
        """
        Executes a complete 3D rendering pass:
        1. Left & Right rear-view mirror FBO passes (if enabled)
        2. Main viewport clear & world track + 3D AI opponent cars pass
        3. Cockpit interior pass (Monocoque, Mirrors, Rotating Steering Wheel + LCD, Halo)
        """
        if self.ctx is None or self.cockpit_prog is None:
            return
        import moderngl

        gl = self.ctx
        w, h = self.settings.resolution
        aspect = float(w) / max(1.0, float(h))
        has_track = "track" in self.vaos and self.track_prog is not None
        prog = self.cockpit_prog

        # 1. Rear-view mirror FBO passes
        if self.settings.rear_mirrors_enabled and self.left_mirror_fbo is not None and self.right_mirror_fbo is not None:
            mw, mh = self.settings.mirror_resolution
            m_aspect = float(mw) / max(1.0, float(mh))
            for side, fbo in (("left", self.left_mirror_fbo), ("right", self.right_mirror_fbo)):
                fbo.use()
                gl.viewport = (0, 0, mw, mh)
                gl.clear(0.18, 0.24, 0.34, 1.0, depth=1.0)
                if has_track:
                    m_view, m_proj = self.camera.get_mirror_matrices(side, m_aspect, car_world_pos, car_yaw_rad)
                    self.track_prog["u_view"].write(mat4_to_gl_bytes(m_view))
                    self.track_prog["u_proj"].write(mat4_to_gl_bytes(m_proj))
                    self.track_prog["u_light_dir"].value = (0.35, 0.85, 0.40)
                    self.track_prog["u_camera_pos"].value = self.camera.world_eye_pos.to_tuple()
                    self.track_prog["u_fog_far"].value = float(self.settings.draw_distance_m)
                    self.vaos["track"].render()
                    if ai_cars and "ai_car" in self.vaos:
                        prog["u_view"].write(mat4_to_gl_bytes(m_view))
                        prog["u_proj"].write(mat4_to_gl_bytes(m_proj))
                        prog["u_light_dir"].value = (0.35, 0.85, 0.40)
                        prog["u_opacity"].value = 1.0
                        prog["u_use_texture"].value = 0
                        prog["u_emissive"].value = 0
                        for ai_mat, ai_rgb in ai_cars:
                            prog["u_model"].write(mat4_to_gl_bytes(ai_mat))
                            prog["u_base_color"].value = ai_rgb
                            self.vaos["ai_car"].render()

        # 2. Bind main framebuffer
        target_fbo = self.offscreen_fbo if self.offscreen_fbo is not None else gl.screen
        target_fbo.use()
        gl.viewport = (0, 0, w, h)
        gl.enable(moderngl.DEPTH_TEST | moderngl.BLEND)
        gl.blend_func = (moderngl.SRC_ALPHA, moderngl.ONE_MINUS_SRC_ALPHA)
        gl.clear(0.42, 0.58, 0.78, 1.0, depth=1.0)

        # 2b. Render 3D world track and 3D AI opponent cars if uploaded
        if has_track:
            world_view = self.camera.get_view_matrix(cockpit_local=False)
            world_proj = self.camera.get_projection_matrix(aspect_ratio=aspect)
            self.track_prog["u_view"].write(mat4_to_gl_bytes(world_view))
            self.track_prog["u_proj"].write(mat4_to_gl_bytes(world_proj))
            self.track_prog["u_light_dir"].value = (0.35, 0.85, 0.40)
            self.track_prog["u_camera_pos"].value = self.camera.world_eye_pos.to_tuple()
            self.track_prog["u_fog_far"].value = float(self.settings.draw_distance_m)
            self.vaos["track"].render()

            if ai_cars and "ai_car" in self.vaos:
                prog["u_view"].write(mat4_to_gl_bytes(world_view))
                prog["u_proj"].write(mat4_to_gl_bytes(world_proj))
                prog["u_light_dir"].value = (0.35, 0.85, 0.40)
                prog["u_opacity"].value = 1.0
                prog["u_use_texture"].value = 0
                prog["u_emissive"].value = 0
                for ai_mat, ai_rgb in ai_cars:
                    prog["u_model"].write(mat4_to_gl_bytes(ai_mat))
                    prog["u_base_color"].value = ai_rgb
                    self.vaos["ai_car"].render()

            # Clear only depth buffer so cockpit interior never clips with world track
            target_fbo.color_mask = (False, False, False, False)
            gl.clear(depth=1.0)
            target_fbo.color_mask = (True, True, True, True)
            target_fbo.use()
            gl.viewport = (0, 0, w, h)

        # 3. Upload latest LCD telemetry pixels
        self.upload_lcd_texture()

        # 4. Cockpit local view & projection matrices
        view_cockpit = self.camera.get_view_matrix(cockpit_local=True)
        proj_mat = self.camera.get_projection_matrix(aspect_ratio=aspect)
        identity_model = Mat4.identity()
        wheel_model = compute_wheel_model_matrix_4x4(wheel_angle_rad, self.spec)

        prog["u_view"].write(mat4_to_gl_bytes(view_cockpit))
        prog["u_proj"].write(mat4_to_gl_bytes(proj_mat))
        prog["u_light_dir"].value = (0.35, 0.85, 0.40)

        # 4a. Monocoque tub & front suspension
        prog["u_model"].write(mat4_to_gl_bytes(identity_model))
        prog["u_base_color"].value = (0.14, 0.16, 0.19)
        prog["u_opacity"].value = 1.0
        prog["u_use_texture"].value = 0
        prog["u_emissive"].value = 0
        self.vaos["monocoque"].render()

        # 4b. Mirror housings & reflective glass
        prog["u_base_color"].value = (0.18, 0.20, 0.24)
        self.vaos["mirror_housings"].render()

        if self.settings.rear_mirrors_enabled and self.left_mirror_tex is not None and self.right_mirror_tex is not None:
            prog["u_use_texture"].value = 1
            prog["u_emissive"].value = 1
            self.left_mirror_tex.use(location=0)
            prog["u_tex"].value = 0
            self.vaos["left_mirror_glass"].render()
            self.right_mirror_tex.use(location=0)
            self.vaos["right_mirror_glass"].render()

        # 4c. Rotating F1 Steering Wheel & PCU-8D Telemetry LCD
        prog["u_model"].write(mat4_to_gl_bytes(wheel_model))
        prog["u_use_texture"].value = 0
        prog["u_emissive"].value = 0
        prog["u_base_color"].value = (0.11, 0.12, 0.15)
        self.vaos["wheel_body"].render()

        if self.lcd_texture is not None:
            self.lcd_texture.use(location=0)
            prog["u_tex"].value = 0
            prog["u_use_texture"].value = 1
            prog["u_emissive"].value = 1
            self.vaos["wheel_lcd"].render()

        # 4d. Titanium Halo Pillar & Hoop (supports adjustable opacity)
        prog["u_model"].write(mat4_to_gl_bytes(identity_model))
        prog["u_use_texture"].value = 0
        prog["u_emissive"].value = 0
        prog["u_base_color"].value = (0.10, 0.11, 0.13)
        prog["u_opacity"].value = float(self.settings.halo_opacity)
        self.vaos["halo"].render()

        self.frame_count += 1

    def read_offscreen_pixels(self) -> Optional[np.ndarray]:
        """Reads back the rendered offscreen FBO as an (H, W, 4) uint8 array (for testing)."""
        if self.offscreen_fbo is None:
            return None
        w, h = self.settings.resolution
        raw = self.offscreen_fbo.read(components=4)
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))
        return np.flipud(arr).copy()

    def release(self) -> None:
        """Releases OpenGL resources cleanly."""
        for vao in self.vaos.values():
            try:
                vao.release()
            except Exception:
                pass
        self.vaos.clear()
        if self.ctx is not None:
            try:
                self.ctx.release()
            except Exception:
                pass
            self.ctx = None
