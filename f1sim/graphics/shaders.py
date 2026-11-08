"""
f1sim/graphics/shaders.py
=========================
OpenGL 3.3 Core Profile (#version 330 core) GLSL shader source definitions
for the F1 cockpit, steering wheel, PCU-8D LCD texture, rear-view mirrors,
track ribbons/kerbs, and high-speed radial motion blur post-processing.
"""
from __future__ import annotations


COCKPIT_VERTEX_SHADER: str = """#version 330 core
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;

uniform mat4 u_model;
uniform mat4 u_view;
uniform mat4 u_proj;

out vec3 v_world_pos;
out vec3 v_normal;
out vec2 v_uv;

void main() {
    vec4 world_pos = u_model * vec4(in_position, 1.0);
    v_world_pos = world_pos.xyz;
    v_normal = normalize(mat3(u_model) * in_normal);
    v_uv = in_uv;
    gl_Position = u_proj * u_view * world_pos;
}
"""


COCKPIT_FRAGMENT_SHADER: str = """#version 330 core
in vec3 v_world_pos;
in vec3 v_normal;
in vec2 v_uv;

uniform vec3 u_base_color;
uniform float u_opacity;
uniform int u_use_texture;
uniform int u_emissive;
uniform sampler2D u_tex;
uniform vec3 u_light_dir;

out vec4 frag_color;

void main() {
    vec3 n = normalize(v_normal);
    vec3 l = normalize(u_light_dir);
    float ndotl = max(dot(n, l), 0.0);
    float ambient = 0.32;
    float spec = pow(max(dot(n, normalize(l + vec3(0.0, 0.3, 1.0))), 0.0), 28.0) * 0.22;

    if (u_use_texture == 1) {
        vec4 tex_col = texture(u_tex, v_uv);
        if (u_emissive == 1) {
            frag_color = vec4(tex_col.rgb, tex_col.a * u_opacity);
        } else {
            vec3 lit = tex_col.rgb * (ambient + 0.68 * ndotl) + vec3(spec * 0.4);
            frag_color = vec4(lit, tex_col.a * u_opacity);
        }
    } else {
        // Subtle carbon weave / metallic modulation from UV
        float weave = 0.96 + 0.04 * sin(v_uv.x * 80.0) * cos(v_uv.y * 80.0);
        vec3 lit = u_base_color * weave * (ambient + 0.68 * ndotl) + vec3(spec);
        frag_color = vec4(lit, u_opacity);
    }
}
"""


TRACK_VERTEX_SHADER: str = """#version 330 core
layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec2 in_uv;
layout(location = 3) in float in_surface_id;
layout(location = 4) in vec3 in_tangent;

uniform mat4 u_view;
uniform mat4 u_proj;

out vec3 v_world_pos;
out vec3 v_normal;
out vec2 v_uv;
out float v_surface_id;
out vec3 v_tangent;

void main() {
    v_world_pos = in_position;
    v_normal = normalize(in_normal);
    v_uv = in_uv;
    v_surface_id = in_surface_id;
    v_tangent = in_tangent;
    gl_Position = u_proj * u_view * vec4(in_position, 1.0);
}
"""


TRACK_FRAGMENT_SHADER: str = """#version 330 core
in vec3 v_world_pos;
in vec3 v_normal;
in vec2 v_uv;
in float v_surface_id;
in vec3 v_tangent;

uniform vec3 u_light_dir;
uniform vec3 u_camera_pos;
uniform float u_fog_far;

out vec4 frag_color;

void main() {
    vec3 n = normalize(v_normal);
    vec3 l = normalize(u_light_dir);
    float ndotl = max(dot(n, l), 0.0);
    float grain = 0.94 + 0.06 * fract(sin(dot(v_uv * 24.0 + v_tangent.xy * 0.01, vec2(12.9898, 78.233))) * 43758.5453);

    // Derive surface color from SurfaceType ID
    vec3 surf_col = vec3(0.20, 0.22, 0.25);
    if (v_surface_id < 0.5) {
        // Transverse asphalt paving bands every 2m so forward speed is visually unmistakable
        float band = 0.92 + 0.08 * step(0.5, fract(v_uv.y * 2.0));
        surf_col = vec3(0.18, 0.20, 0.23) * band;
        // Darker rubbered racing groove near center
        float groove = 1.0 - 0.14 * exp(-pow((v_uv.x - 0.5) * 4.5, 2.0));
        surf_col *= groove;
        // White track edge lines
        if (v_uv.x < 0.035 || v_uv.x > 0.965) {
            surf_col = vec3(0.92, 0.94, 0.96);
        }
    } else if (v_surface_id > 0.5 && v_surface_id < 1.5) {
        surf_col = vec3(0.86, 0.08, 0.06);
    } else if (v_surface_id >= 1.5 && v_surface_id < 2.5) {
        surf_col = vec3(0.94, 0.94, 0.96);
    } else if (v_surface_id >= 2.5 && v_surface_id < 3.5) {
        surf_col = vec3(0.25, 0.27, 0.30);
    } else if (v_surface_id >= 3.5 && v_surface_id < 4.5) {
        surf_col = vec3(0.76, 0.68, 0.52);
    } else if (v_surface_id >= 4.5 && v_surface_id < 5.5) {
        float lawn = (fract(v_uv.y * 0.25) < 0.5) ? 1.0 : 0.86;
        surf_col = vec3(0.22, 0.44, 0.18) * lawn;
    } else if (v_surface_id >= 5.5 && v_surface_id < 8.5) {
        surf_col = vec3(0.84, 0.87, 0.90);
    } else if (v_surface_id >= 8.5 && v_surface_id < 9.5) {
        surf_col = (fract(v_uv.x * 2.0) < 0.5) ? vec3(0.88, 0.10, 0.08) : vec3(0.95, 0.95, 0.97);
    } else if (v_surface_id >= 9.5 && v_surface_id < 10.5) {
        surf_col = vec3(0.15, 0.36, 0.16);
    } else if (v_surface_id >= 10.5 && v_surface_id < 11.5) {
        surf_col = vec3(0.38, 0.25, 0.15);
    } else if (v_surface_id >= 11.5 && v_surface_id < 12.5) {
        float crowd = fract(sin(dot(floor(v_uv * vec2(40.0, 12.0)), vec2(12.9898, 78.233))) * 43758.5453);
        surf_col = (crowd < 0.55) ? vec3(0.86, 0.08, 0.08) : ((crowd < 0.80) ? vec3(0.95, 0.85, 0.15) : vec3(0.92, 0.94, 0.96));
    } else if (v_surface_id >= 12.5) {
        surf_col = mix(vec3(0.34, 0.40, 0.52), vec3(0.88, 0.90, 0.95), clamp((v_world_pos.z - 60.0) / 140.0, 0.0, 1.0));
    }

    vec3 base = surf_col * grain * (0.42 + 0.58 * ndotl);

    float dist = length(v_world_pos - u_camera_pos);
    float fog_factor = clamp(dist / max(200.0, u_fog_far), 0.0, 1.0);
    vec3 sky_horizon = vec3(0.55, 0.70, 0.88);
    vec3 final_rgb = mix(base, sky_horizon, fog_factor * fog_factor * 0.65);
    frag_color = vec4(final_rgb, 1.0);
}
"""
