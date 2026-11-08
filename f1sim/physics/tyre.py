"""
f1sim/physics/tyre.py
Non-linear Pacejka '96 Magic Formula Tyre Model with Load Sensitivity
and Combined Slip Friction Ellipse.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class PacejkaParams:
    """Coefficients for Pacejka '96 Magic Formula."""
    # Longitudinal coefficients (x)
    B_x: float = 11.5
    C_x: float = 1.65
    E_x: float = -0.50
    mu_x_0: float = 1.80
    k_load_x: float = 0.075

    # Lateral coefficients (y)
    B_y: float = 13.0
    C_y: float = 1.35
    E_y: float = -0.80
    mu_y_0: float = 1.75
    k_load_y: float = 0.080

    # Reference normal load (N)
    Fz0: float = 4000.0


@dataclass
class TyreForces:
    """Output contact patch forces."""
    Fx: float = 0.0      # Longitudinal force in wheel frame (N)
    Fy: float = 0.0      # Lateral cornering force in wheel frame (N)
    Fx_body: float = 0.0 # Transformed to vehicle body X (N)
    Fy_body: float = 0.0 # Transformed to vehicle body Y (N)
    mu_eff: float = 0.0  # Effective load-sensitive friction coefficient


class TyreModel:
    """
    Evaluates pure and combined slip tyre forces using Pacejka '96 Magic Formula.
    """
    def __init__(self, params: PacejkaParams | None = None) -> None:
        self.params: PacejkaParams = params if params is not None else PacejkaParams()

    def compute_mu(self, Fz: float, is_lateral: bool = False) -> float:
        """Computes normal-load-sensitive friction coefficient mu(Fz)."""
        Fz_clamped = max(100.0, Fz)
        mu0 = self.params.mu_y_0 if is_lateral else self.params.mu_x_0
        k_load = self.params.k_load_y if is_lateral else self.params.k_load_x
        load_factor = (Fz_clamped - self.params.Fz0) / self.params.Fz0
        mu = mu0 * (1.0 - k_load * load_factor)
        return float(np.clip(mu, 1.10, 2.05))

    def compute_pure_lateral(self, alpha: float, Fz: float, mu_surface: float = 1.0) -> float:
        """
        Pure lateral cornering force Fy0(alpha, Fz).
        Returns force in Newtons. Positive alpha -> Positive Fy.
        """
        Fz_clamped = max(100.0, Fz)
        mu_eff = self.compute_mu(Fz_clamped, is_lateral=True) * mu_surface
        D = mu_eff * Fz_clamped
        B = self.params.B_y
        C = self.params.C_y
        E = self.params.E_y

        Bx = B * alpha
        inner = Bx - E * (Bx - math.atan(Bx))
        Fy0 = D * math.sin(C * math.atan(inner))
        return float(Fy0)

    def compute_pure_longitudinal(self, kappa: float, Fz: float, mu_surface: float = 1.0) -> float:
        """
        Pure longitudinal force Fx0(kappa, Fz).
        Returns force in Newtons. Positive kappa -> Positive Fx.
        """
        Fz_clamped = max(100.0, Fz)
        mu_eff = self.compute_mu(Fz_clamped, is_lateral=False) * mu_surface
        D = mu_eff * Fz_clamped
        B = self.params.B_x
        C = self.params.C_x
        E = self.params.E_x

        Bx = B * kappa
        inner = Bx - E * (Bx - math.atan(Bx))
        Fx0 = D * math.sin(C * math.atan(inner))
        return float(Fx0)

    def compute_combined_forces(
        self,
        kappa: float,
        alpha: float,
        Fz: float,
        steer_angle: float = 0.0,
        mu_surface: float = 1.0
    ) -> TyreForces:
        """
        Computes combined slip tyre forces using friction ellipse coupling.
        Rotates forces to vehicle body frame using steer_angle.
        """
        Fz_clamped = max(100.0, Fz)
        mu_x = self.compute_mu(Fz_clamped, is_lateral=False) * mu_surface
        mu_y = self.compute_mu(Fz_clamped, is_lateral=True) * mu_surface
        Dx = max(1.0, mu_x * Fz_clamped)
        Dy = max(1.0, mu_y * Fz_clamped)

        # 1. Pure forces
        Fx0 = self.compute_pure_longitudinal(kappa, Fz_clamped, mu_surface)
        Fy0 = self.compute_pure_lateral(alpha, Fz_clamped, mu_surface)

        # 2. Coupled vector interaction
        C_alpha = self.params.B_y * self.params.C_y * Dy
        C_kappa = self.params.B_x * self.params.C_x * Dx
        ratio_alpha = C_alpha / max(1.0, C_kappa)
        ratio_kappa = C_kappa / max(1.0, C_alpha)

        denom_x = math.sqrt(kappa * kappa + (ratio_alpha * alpha) ** 2 + 1e-6)
        denom_y = math.sqrt((ratio_kappa * kappa) ** 2 + alpha * alpha + 1e-6)

        Fx_c = Fx0 * (abs(kappa) / denom_x)
        Fy_c = Fy0 * (abs(alpha) / denom_y)

        # 3. Strict Friction Ellipse Boundary Clamp
        ellipse_ratio = math.sqrt((Fx_c / Dx) ** 2 + (Fy_c / Dy) ** 2)
        if ellipse_ratio > 1.0:
            Fx_c /= ellipse_ratio
            Fy_c /= ellipse_ratio

        # Preserve original signs
        Fx_w = math.copysign(Fx_c, kappa)
        Fy_w = math.copysign(Fy_c, alpha)

        # 4. Transform to vehicle body coordinate frame
        # [Fx_b] = [cos(delta)  -sin(delta)] [Fx_w]
        # [Fy_b]   [sin(delta)   cos(delta)] [Fy_w]
        cos_d = math.cos(steer_angle)
        sin_d = math.sin(steer_angle)
        Fx_body = Fx_w * cos_d - Fy_w * sin_d
        Fy_body = Fx_w * sin_d + Fy_w * cos_d

        return TyreForces(
            Fx=Fx_w,
            Fy=Fy_w,
            Fx_body=Fx_body,
            Fy_body=Fy_body,
            mu_eff=mu_y
        )
