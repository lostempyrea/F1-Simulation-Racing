"""
f1sim/core/constants.py
Physical constants, vehicle chassis geometry, powertrain specifications,
transmission gear ratios, and simulation settings for Formula 1 simulator.
"""
from enum import IntEnum

# --- Environmental & Physical Constants ---
GRAVITY: float = 9.80665               # Standard acceleration due to gravity (m/s^2)
AIR_DENSITY: float = 1.225             # Sea-level atmospheric air density (kg/m^3)
SPEED_OF_SOUND: float = 343.0          # Speed of sound at 20°C (m/s)

# --- Simulation Timing & Clocks ---
SIM_HZ: int = 120                      # Primary fixed physics update frequency (Hz)
SIM_DT: float = 1.0 / SIM_HZ           # Primary simulation time step (~0.008333 s)
BENCHMARK_HZ: int = 1000               # Headless test suite sub-stepping frequency (Hz)
BENCHMARK_DT: float = 1.0 / BENCHMARK_HZ # Benchmark sub-step time (0.001 s)
MAX_ACCUMULATOR_SEC: float = 0.20      # Safety cap against physics accumulator spiral of death

# --- Chassis, Mass & Inertial Constants ---
VEHICLE_MASS: float = 820.0            # Total vehicle mass with driver and fuel (kg)
WHEELBASE: float = 3.600               # Wheelbase length from front to rear axle (m)
WEIGHT_BIAS_FRONT: float = 0.46        # Static front weight distribution (46%)
WEIGHT_BIAS_REAR: float = 0.54         # Static rear weight distribution (54%)
DIST_CG_FRONT: float = WHEELBASE * WEIGHT_BIAS_REAR # 1.944 m
DIST_CG_REAR: float = WHEELBASE * WEIGHT_BIAS_FRONT  # 1.656 m
TRACK_WIDTH_FRONT: float = 1.600       # Front axle track width (m)
TRACK_WIDTH_REAR: float = 1.550        # Rear axle track width (m)
CG_HEIGHT: float = 0.280               # Center of gravity height above ground plane (m)

MOMENT_INERTIA_YAW: float = 1150.0     # Iz: Rotational yaw inertia (kg*m^2)
MOMENT_INERTIA_PITCH: float = 1050.0   # Iy: Rotational pitch inertia (kg*m^2)
MOMENT_INERTIA_ROLL: float = 260.0     # Ix: Rotational roll inertia (kg*m^2)

TYRE_RADIUS: float = 0.360             # Nominal 18-inch Pirelli outer tyre radius (m)
WHEEL_MASS: float = 12.0               # Single wheel + tyre rim mass (kg)
WHEEL_INERTIA: float = 1.15            # Rotational inertia per wheel assembly (kg*m^2)
ENGINE_INERTIA: float = 0.18           # Engine rotational inertia (crankshaft + flywheel) (kg*m^2)

# --- Engine & Hybrid Power Unit Specifications ---
ENGINE_DISPLACEMENT: float = 1.6       # 1.6 Litres
ENGINE_CYLINDERS: int = 6              # 90-degree V6 Turbo
RPM_IDLE: float = 4000.0               # Regulated idle speed (RPM)
RPM_REDLINE: float = 15000.0           # Maximum operating engine speed (RPM)
RPM_REV_LIMITER_CUT: float = 15200.0   # Hard electronic spark/fuel cut limit (RPM)
RPM_PEAK_POWER: float = 12500.0        # Engine speed for peak 1000 hp power (RPM)
RPM_AUTO_UPSHIFT: float = 13200.0      # Automatic transmission upshift threshold (RPM)
RPM_AUTO_DOWNSHIFT: float = 8500.0     # Automatic transmission downshift threshold (RPM)
RPM_KICKDOWN_THRESHOLD: float = 7000.0 # Throttle kickdown RPM ceiling (RPM)
RPM_STALL_THRESHOLD: float = 2800.0    # Anti-stall clutch disengage trigger (RPM)
RPM_CRANKING_TARGET: float = 2500.0    # Starter MGU-K motor crank spin target (RPM)
RPM_FIRE_FLARE: float = 6500.0         # Acoustic start-up ignition flare RPM (RPM)

PEAK_POWER_TOTAL_KW: float = 746.0     # Total combined output: 1000 bhp (kW)
PEAK_POWER_ICE_KW: float = 626.0       # ICE combustion output: ~840 bhp (kW)
PEAK_POWER_MGUK_KW: float = 120.0      # MGU-K hybrid electrical output: ~160 bhp (kW)
PEAK_TORQUE_ICE_NM: float = 500.0      # Maximum ICE mechanical torque (Nm)
PEAK_TORQUE_MGUK_NM: float = 350.0     # Maximum MGU-K electric motor torque (Nm)
MAX_COMBINED_TORQUE_NM: float = 750.0  # Peak combined low-end torque (Nm)

ENGINE_BRAKE_BASE_NM: float = 45.0     # Base mechanical engine braking at zero throttle (Nm)
ENGINE_BRAKE_HIGH_RPM_NM: float = 25.0 # Progressive high-RPM engine braking adder (Nm)

# --- Transmission Gear Ratios ---
GEAR_RATIOS: dict[int, float] = {
    -1: -3.20,  # Reverse (R)
     0:  0.00,  # Neutral (N)
     1:  3.25,  # 1st Gear
     2:  2.45,  # 2nd Gear
     3:  1.92,  # 3rd Gear
     4:  1.56,  # 4th Gear
     5:  1.30,  # 5th Gear
     6:  1.11,  # 6th Gear
     7:  0.97,  # 7th Gear
     8:  0.86,  # 8th Gear
}
FINAL_DRIVE_RATIO: float = 3.85        # Differential final drive reduction
TRANSMISSION_EFFICIENCY: float = 0.95  # Mechanical drivetrain efficiency
SEAMLESS_SHIFT_DURATION: float = 0.040 # Seamless shift dog-handover duration (seconds)
THROTTLE_BLIP_DURATION: float = 0.060  # Downshift auto-blip duration (seconds)
THROTTLE_BLIP_AMOUNT: float = 0.55     # Auto-blip throttle boost amplitude (0.0 .. 1.0)
DOWNSHIFT_OVERREV_LIMIT_RPM: float = 14800.0 # Downshift overrev lockout RPM

# --- Aerodynamics Reference Constants ---
AERO_REF_AREA: float = 1.60            # Frontal aerodynamic reference area A (m^2)
AERO_CD_NOMINAL: float = 0.95          # Baseline drag coefficient C_D
AERO_CL_NOMINAL: float = 2.90          # Baseline downforce coefficient C_L
AERO_CDA_BASE: float = AERO_CD_NOMINAL * AERO_REF_AREA   # Baseline drag area (Cd * A) (1.52 m^2)
AERO_CLA_BASE: float = AERO_CL_NOMINAL * AERO_REF_AREA   # Baseline downforce area (Cl * A) (4.64 m^2)
AERO_FRONT_DOWNFORCE_RATIO: float = 0.44 # Front downforce balance (44%)
AERO_REAR_DOWNFORCE_RATIO: float = 0.56  # Rear downforce balance (56%)
DRS_CD_REDUCTION: float = 0.28         # Drag reduction factor when DRS open (28%)
DRS_REAR_DOWNFORCE_REDUCTION: float = 0.32 # Rear downforce reduction when DRS open (32%)
ROLLING_RESISTANCE_COEFF: float = 0.015 # Tyre rolling resistance coefficient Crr

# --- Braking Reference Constants ---
BRAKE_BIAS_DEFAULT: float = 0.58       # Default front brake bias (58%)
BRAKE_BIAS_MIN: float = 0.52           # Minimum front brake bias (52%)
BRAKE_BIAS_MAX: float = 0.64           # Maximum front brake bias (64%)
BRAKE_BIAS_STEP: float = 0.005         # Bias adjustment increment (0.5% per click)
MAX_BRAKE_TORQUE_TOTAL: float = 13000.0 # Peak combined four-wheel braking torque (Nm)

# --- State Enums ---
class IgnitionState(IntEnum):
    OFF = 0
    IGNITION_ON = 1
    CRANKING = 2
    RUNNING = 3

class TransmissionMode(IntEnum):
    MANUAL = 0
    AUTOMATIC = 1
