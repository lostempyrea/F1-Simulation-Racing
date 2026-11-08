"""
f1sim.physics: Core vehicle dynamics simulation modules.
"""

from f1sim.physics.powertrain import Powertrain, EngineModel, Gearbox
from f1sim.physics.aero import Aerodynamics, DRSState, AeroForces
from f1sim.physics.tyre import TyreModel, PacejkaParams, TyreForces
from f1sim.physics.brakes import BrakeSystem, BrakeDisc, BrakeTorqueOutput
from f1sim.physics.vehicle import Vehicle, VehicleTelemetry, VehicleConfig, VehicleState

__all__ = [
    "Powertrain",
    "EngineModel",
    "Gearbox",
    "Aerodynamics",
    "DRSState",
    "AeroForces",
    "TyreModel",
    "PacejkaParams",
    "TyreForces",
    "BrakeSystem",
    "BrakeDisc",
    "BrakeTorqueOutput",
    "Vehicle",
    "VehicleTelemetry",
    "VehicleConfig",
    "VehicleState",
]
