"""
f1sim/core/math3d.py
High-performance 3D vector, matrix, quaternion, and spline mathematics
optimized for vehicle dynamics simulation and ModernGL rendering.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Tuple, Sequence, Union
import numpy as np


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: Union[Vec3, float]) -> Vec3:
        if isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        return Vec3(self.x + other, self.y + other, self.z + other)

    def __radd__(self, other: Union[Vec3, float]) -> Vec3:
        return self.__add__(other)

    def __sub__(self, other: Union[Vec3, float]) -> Vec3:
        if isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        return Vec3(self.x - other, self.y - other, self.z - other)

    def __rsub__(self, other: Union[Vec3, float]) -> Vec3:
        if isinstance(other, Vec3):
            return Vec3(other.x - self.x, other.y - self.y, other.z - self.z)
        return Vec3(other - self.x, other - self.y, other - self.z)

    def __mul__(self, scalar: float) -> Vec3:
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> Vec3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vec3:
        inv = 1.0 / scalar
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def __neg__(self) -> Vec3:
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, other: Vec3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vec3) -> Vec3:
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x
        )

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

    def length_squared(self) -> float:
        return self.x * self.x + self.y * self.y + self.z * self.z

    def normalized(self) -> Vec3:
        l = self.length()
        if l < 1e-12:
            return Vec3(0.0, 0.0, 0.0)
        inv = 1.0 / l
        return Vec3(self.x * inv, self.y * inv, self.z * inv)

    def distance_to(self, other: Vec3) -> float:
        return (self - other).length()

    def lerp(self, other: Vec3, t: float) -> Vec3:
        return self + (other - self) * t

    def to_numpy(self, dtype=np.float64) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=dtype)

    @classmethod
    def from_numpy(cls, arr: Union[Sequence[float], np.ndarray]) -> Vec3:
        return cls(float(arr[0]), float(arr[1]), float(arr[2]))

    def to_tuple(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)


class Mat3:
    """3x3 orientation and rotation matrix."""
    def __init__(self, data: np.ndarray | None = None) -> None:
        if data is not None:
            self.data = np.asarray(data, dtype=np.float64).reshape((3, 3))
        else:
            self.data = np.eye(3, dtype=np.float64)

    @classmethod
    def identity(cls) -> Mat3:
        return cls(np.eye(3, dtype=np.float64))

    @classmethod
    def from_euler(cls, pitch: float, yaw: float, roll: float) -> Mat3:
        """Euler angles: pitch (Y), yaw (Z), roll (X). Canonical aerospace ZYX composition."""
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cr, sr = math.cos(roll), math.sin(roll)

        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        return cls(rz @ ry @ rx)

    @classmethod
    def rotation_x(cls, rad: float) -> Mat3:
        c, s = math.cos(rad), math.sin(rad)
        return cls(np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64))

    @classmethod
    def rotation_y(cls, rad: float) -> Mat3:
        c, s = math.cos(rad), math.sin(rad)
        return cls(np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64))

    @classmethod
    def rotation_z(cls, rad: float) -> Mat3:
        c, s = math.cos(rad), math.sin(rad)
        return cls(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64))

    def to_euler(self) -> Tuple[float, float, float]:
        """Extracts pitch (Y), yaw (Z), roll (X) from rotation matrix."""
        sy = math.sqrt(self.data[0, 0] ** 2 + self.data[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            pitch = math.atan2(-self.data[2, 0], sy)
            yaw = math.atan2(self.data[1, 0], self.data[0, 0])
            roll = math.atan2(self.data[2, 1], self.data[2, 2])
        else:
            pitch = math.atan2(-self.data[2, 0], sy)
            yaw = math.atan2(-self.data[0, 1], self.data[1, 1])
            roll = 0.0
        return (pitch, yaw, roll)

    def transform(self, v: Vec3) -> Vec3:
        arr = self.data @ v.to_numpy()
        return Vec3.from_numpy(arr)

    def transpose(self) -> Mat3:
        return Mat3(self.data.T)

    def inverse(self) -> Mat3:
        return Mat3(np.linalg.inv(self.data))

    def __matmul__(self, other: Mat3) -> Mat3:
        return Mat3(self.data @ other.data)

    def to_numpy(self) -> np.ndarray:
        return self.data.copy()


class Mat4:
    """4x4 transformation matrix for 3D world, view, and projection transforms."""
    def __init__(self, data: np.ndarray | None = None) -> None:
        if data is not None:
            self.data = np.asarray(data, dtype=np.float64).reshape((4, 4))
        else:
            self.data = np.eye(4, dtype=np.float64)

    @classmethod
    def identity(cls) -> Mat4:
        return cls(np.eye(4, dtype=np.float64))

    @classmethod
    def translation(cls, x: float, y: float, z: float) -> Mat4:
        m = np.eye(4, dtype=np.float64)
        m[0, 3] = x
        m[1, 3] = y
        m[2, 3] = z
        return cls(m)

    @classmethod
    def from_mat3_and_pos(cls, rot: Mat3, pos: Vec3) -> Mat4:
        m = np.eye(4, dtype=np.float64)
        m[:3, :3] = rot.to_numpy()
        m[0, 3] = pos.x
        m[1, 3] = pos.y
        m[2, 3] = pos.z
        return cls(m)

    @classmethod
    def perspective(cls, fov_y_deg: float, aspect: float, near: float, far: float) -> Mat4:
        """Right-handed OpenGL projection matrix."""
        f = 1.0 / math.tan(math.radians(fov_y_deg) / 2.0)
        m = np.zeros((4, 4), dtype=np.float64)
        m[0, 0] = f / aspect
        m[1, 1] = f
        m[2, 2] = (far + near) / (near - far)
        m[2, 3] = (2.0 * far * near) / (near - far)
        m[3, 2] = -1.0
        return cls(m)

    @classmethod
    def look_at(cls, eye: Vec3, target: Vec3, up: Vec3) -> Mat4:
        """Right-handed OpenGL Look-At matrix."""
        forward = (target - eye).normalized()
        side = forward.cross(up).normalized()
        up_vec = side.cross(forward)

        m = np.eye(4, dtype=np.float64)
        m[0, :3] = side.to_numpy()
        m[1, :3] = up_vec.to_numpy()
        m[2, :3] = (-forward).to_numpy()

        m[0, 3] = -side.dot(eye)
        m[1, 3] = -up_vec.dot(eye)
        m[2, 3] = forward.dot(eye)
        return cls(m)

    def transform_point(self, p: Vec3) -> Vec3:
        v = np.array([p.x, p.y, p.z, 1.0], dtype=np.float64)
        res = self.data @ v
        w = res[3] if abs(res[3]) > 1e-12 else 1.0
        return Vec3(res[0] / w, res[1] / w, res[2] / w)

    def transform_vector(self, v: Vec3) -> Vec3:
        vec = np.array([v.x, v.y, v.z, 0.0], dtype=np.float64)
        res = self.data @ vec
        return Vec3(res[0], res[1], res[2])

    def __matmul__(self, other: Mat4) -> Mat4:
        return Mat4(self.data @ other.data)

    def to_numpy(self, dtype=np.float32) -> np.ndarray:
        return np.asarray(self.data, dtype=dtype)


@dataclass
class Quaternion:
    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def identity(cls) -> Quaternion:
        return cls(1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_axis_angle(cls, axis: Vec3, angle_rad: float) -> Quaternion:
        ax = axis.normalized()
        half = angle_rad * 0.5
        s = math.sin(half)
        return cls(math.cos(half), ax.x * s, ax.y * s, ax.z * s)

    @classmethod
    def from_euler(cls, pitch: float, yaw: float, roll: float) -> Quaternion:
        # pitch (Y), yaw (Z), roll (X) -> q = qz * qy * qx
        qz = cls.from_axis_angle(Vec3(0, 0, 1), yaw)
        qy = cls.from_axis_angle(Vec3(0, 1, 0), pitch)
        qx = cls.from_axis_angle(Vec3(1, 0, 0), roll)
        return (qz * qy * qx).normalized()

    def to_mat3(self) -> Mat3:
        w, x, y, z = self.w, self.x, self.y, self.z
        m = np.array([
            [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
            [2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
            [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)]
        ], dtype=np.float64)
        return Mat3(m)

    def to_euler(self) -> Tuple[float, float, float]:
        return self.to_mat3().to_euler()

    def normalized(self) -> Quaternion:
        n = math.sqrt(self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        if n < 1e-12:
            return Quaternion.identity()
        inv = 1.0 / n
        return Quaternion(self.w * inv, self.x * inv, self.y * inv, self.z * inv)

    def slerp(self, other: Quaternion, t: float) -> Quaternion:
        dot = self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z
        q2 = other
        if dot < 0.0:
            dot = -dot
            q2 = Quaternion(-other.w, -other.x, -other.y, -other.z)

        if dot > 0.9995:
            # Linear interpolation for tiny angles
            return Quaternion(
                self.w + t * (q2.w - self.w),
                self.x + t * (q2.x - self.x),
                self.y + t * (q2.y - self.y),
                self.z + t * (q2.z - self.z)
            ).normalized()

        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        w1 = math.sin((1.0 - t) * theta) / sin_theta
        w2 = math.sin(t * theta) / sin_theta

        return Quaternion(
            self.w * w1 + q2.w * w2,
            self.x * w1 + q2.x * w2,
            self.y * w1 + q2.y * w2,
            self.z * w1 + q2.z * w2
        ).normalized()

    def __mul__(self, other: Quaternion) -> Quaternion:
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ).normalized()


# --- Spline & Mathematical Helpers ---

def catmull_rom_spline(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    """Uniform Catmull-Rom spline interpolation at parameter t in [0.0, 1.0]."""
    t2 = t * t
    t3 = t2 * t
    return 0.5 * (
        (2.0 * p1) +
        (-p0 + p2) * t +
        (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
        (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
    )

def catmull_rom_tangent(p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, t: float) -> np.ndarray:
    """Computes first derivative (velocity / tangent vector) of Catmull-Rom spline at t."""
    t2 = t * t
    return 0.5 * (
        (-p0 + p2) +
        2.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t +
        3.0 * (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t2
    )

def calculate_curvature(p_prev: np.ndarray, p_curr: np.ndarray, p_next: np.ndarray) -> float:
    """Calculates Menger curvature kappa = 1 / R from three sequential 2D/3D points."""
    a = float(np.linalg.norm(p_curr - p_prev))
    b = float(np.linalg.norm(p_next - p_curr))
    c = float(np.linalg.norm(p_next - p_prev))
    if a < 1e-6 or b < 1e-6 or c < 1e-6:
        return 0.0
    # Semi-perimeter
    s = 0.5 * (a + b + c)
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0.0:
        return 0.0
    area = math.sqrt(area_sq)
    # Curvature kappa = 4 * Area / (a * b * c)
    return float(4.0 * area / (a * b * c))

def frenet_frame(tangent: Vec3, up_hint: Vec3 = Vec3(0, 0, 1)) -> Tuple[Vec3, Vec3, Vec3]:
    """Computes orthonormal Frenet-Serret basis (Tangent, Normal, Binormal)."""
    t = tangent.normalized()
    # Check if tangent is collinear with up_hint
    if abs(t.dot(up_hint)) > 0.999:
        alt_up = Vec3(1, 0, 0) if abs(t.x) < 0.9 else Vec3(0, 1, 0)
        n = t.cross(alt_up).normalized()
    else:
        n = t.cross(up_hint).normalized()
    b = t.cross(n).normalized()
    return (t, n, b)

def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, val))

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def smoothstep(edge0: float, edge1: float, x: float) -> float:
    t = clamp((x - edge0) / max(1e-9, edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)

def wrap_angle_pi(rad: float) -> float:
    """Wraps angle into [-pi, +pi] interval."""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi
