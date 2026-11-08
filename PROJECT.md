# Project: F1 Desktop Native Simulation Game

## Architecture
- **Platform**: Windows Desktop Native (Python 3.12 64-bit + ModernGL 5.13 + Pygame-ce 2.5.8 + NumPy 2.5.4).
- **GPU Acceleration**: OpenGL 3.3 Core Profile with GLSL 330 shaders (verified 3,890 FPS on RTX 4050).
- **Simulation Frequency**: 120 Hz fixed-step physics integration (with 1,000 Hz sub-stepping in benchmark harnesses).
- **Input Subsystem**: Dual input support via Windows native XInput 1.4 (`ctypes.windll.xinput1_4`) and SDL2 GameController, with smooth exponential slew rate filtering for keyboard.
- **Rendering Architecture**: Multi-pass ModernGL pipeline:
  1. Off-screen Mirror FBO (512x256 rear-facing camera).
  2. Steering Wheel PCU-8D Telemetry LCD texture upload (512x320 surface).
  3. Main 3D perspective pass (Track ribbon, kerbs, scenery, distance boards, AI cars, cockpit tub, halo, rotating steering wheel).
  4. Post-processing & HUD overlay pass (dynamic FOV, speed blur, TV timing tower, mini-map, race alerts).
- **Audio Architecture**: Procedural multi-harmonic V6 Turbo engine synthesizer + event audio (gear shift clack, kerb rumble, tyre screech, DRS chime) with <12ms latency via SDL_mixer.

## Code Layout
```
c:\Users\nguye\PROJECT\F1\
├── main.py                     # Primary desktop entry point & game state machine
├── requirements.txt            # Python dependencies (moderngl, pygame-ce, numpy, pyrr)
├── f1sim/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── constants.py        # Physical constants, engine specs, FIA rules
│   │   ├── math3d.py           # Vectors, matrices, quaternions, spline math
│   │   └── time_step.py        # Fixed 120 Hz accumulator clock
│   ├── physics/
│   │   ├── __init__.py
│   │   ├── powertrain.py       # 1.6L V6 Turbo Hybrid, RPM curve, 8-speed gearbox
│   │   ├── aero.py             # Downforce, drag, DRS flap mechanics
│   │   ├── tyre.py             # Pacejka '96 Magic Formula, slip angle, friction ellipse
│   │   ├── brakes.py           # Carbon disc thermal model & brake bias distribution
│   │   └── vehicle.py          # 4-wheel two-track dynamics, understeer/oversteer, state
│   ├── track/
│   │   ├── __init__.py
│   │   ├── track_spline.py     # 3D centerline, Frenet frames, banking, width
│   │   ├── track_builder.py    # Procedural 3D mesh generation (ribbon, kerbs, boards)
│   │   ├── monza.py            # Autodromo Nazionale Monza layout & metadata
│   │   └── monaco.py           # Circuit de Monaco layout & metadata
│   ├── ai/
│   │   ├── __init__.py
│   │   ├── racing_line.py      # Curvature profiling & speed lookup
│   │   ├── controller.py       # Pure-pursuit steering & backward braking pass
│   │   ├── collision.py        # OBB SAT collision detection & TTC avoidance
│   │   └── driver_profiles.py  # 5 AI drivers & Easy/Medium/Hard calibration
│   ├── graphics/
│   │   ├── __init__.py
│   │   ├── settings.py         # Graphics quality presets, FOV, MSAA, mirrors, blur, JSON persistence
│   │   ├── renderer.py         # ModernGL 3.3 context & draw call pipeline
│   │   ├── shaders.py          # GLSL vertex & fragment shaders (track, cockpit, post)
│   │   ├── meshes.py           # 3D procedural meshes (cockpit, halo, wheel, track)
│   │   ├── camera.py           # Damped mass-spring driver head physics & G-forces
│   │   └── lcd_display.py      # PCU-8D 15-LED rev bar & telemetry LCD texture
│   ├── input/
│   │   ├── __init__.py
│   │   ├── bindings.py         # Remappable keyboard & gamepad keybindings with conflict resolution & JSON save/load
│   │   ├── input_manager.py    # Unified keyboard, mouse, and XInput/gamepad handler
│   │   └── filters.py          # Exponential steering slew rate & speed attenuation
│   ├── audio/
│   │   ├── __init__.py
│   │   └── sound_engine.py     # Procedural V6 audio synthesizer & sound player
│   └── ui/
│       ├── __init__.py
│       ├── theme.py            # F1 carbon & telemetry color tokens
│       ├── menu.py             # Main menu, track select, difficulty select
│       ├── hud.py              # Broadcast TV timing tower, mini-map, lap timer
│       └── results.py          # Post-race podium & telemetry summary
└── tests/
    ├── __init__.py
    ├── benchmark_physics_ai.py # Headless automated physics & AI performance verification
    ├── test_tier1_features.py  # Tier 1: Feature verification tests
    ├── test_tier2_boundaries.py# Tier 2: Boundary & corner case tests
    ├── test_tier3_pairwise.py  # Tier 3: Cross-feature integration tests
    ├── test_tier4_scenarios.py # Tier 4: Real-world race workload scenarios
    └── run_all_tests.py        # Comprehensive test runner with pass/fail exit codes
```

## Feature Inventory
| # | Feature | Description | Milestone | Source |
|---|---------|-------------|-----------|--------|
| 1 | Desktop Native Platform | Windows 64-bit desktop runtime using Python 3.12, ModernGL 3.3, Pygame-ce, and NumPy | M1 | ORIGINAL_REQUEST §R1 |
| 2 | Cockpit First-Person View | Driver eye position (Y=0.68m), carbon tub monocoque, halo pillar with fairing, dual mirrors | M3 | ORIGINAL_REQUEST §R1 |
| 3 | Rotating Steering Wheel | Modern rectangular F1 wheel rotating smoothly matching steering angle (±180°), slew rate filter | M3 | ORIGINAL_REQUEST §R1 |
| 4 | Telemetry LCD & Rev Lights | PCU-8D display with 15-LED RPM rev bar (Green/Amber/Red/Strobe), Gear N/1-8, Speed, DRS, Brake Bias, Delta | M3 | ORIGINAL_REQUEST §R1 |
| 5 | Dual Input Support | Complete keyboard/mouse and gamepad (XInput / SDL2 GameController) support with analog steering/triggers | M3 | ORIGINAL_REQUEST §R2 |
| 6 | Manual & Automatic Transmission | Manual paddle shifting with auto-blip throttle blip vs automatic transmission with kickdown | M1 | ORIGINAL_REQUEST §R2 |
| 7 | DRS Cockpit Control | Cockpit button toggle to open DRS wing flap when in designated DRS zones | M1 | ORIGINAL_REQUEST §R2 |
| 8 | Engine Start / Ignition | Realistic 4-stage ignition and starter crank sequence, idle RPM stabilization | M1 | ORIGINAL_REQUEST §R2 |
| 9 | Brake Bias Adjustment | Real-time cockpit adjustment of front/rear braking distribution (52% to 64% front) | M1 | ORIGINAL_REQUEST §R2 |
| 10 | Powertrain & Acceleration Dynamics | 1.6L V6 Turbo Hybrid power curve (1000 hp), calibrated for 0-100 km/h in 2.53s, 0-200 km/h in 5.05s | M1 | ORIGINAL_REQUEST §R3 |
| 11 | Carbon Braking Dynamics | Deceleration 200-0 km/h in <50m with >4.5G peak deceleration | M1 | ORIGINAL_REQUEST §R3 |
| 12 | Pacejka Tyre Grip Model | Non-linear Pacejka '96 Magic Formula with load sensitivity and combined friction ellipse | M1 | ORIGINAL_REQUEST §R3 |
| 13 | Understeer & Oversteer Handling | Slip angle breakaway, understeer on excessive entry speed, oversteer snap on throttle/kerbs, counter-steering | M1 | ORIGINAL_REQUEST §R3 |
| 14 | Aerodynamics & DRS Drag Reduction | Speed-squared downforce and drag, DRS reduces drag by ~28% boosting top speed >355 km/h | M1 | ORIGINAL_REQUEST §R3 |
| 15 | Autodromo Nazionale Monza | 5.793 km layout, 11 turns, 2 DRS zones, 150m/100m/50m distance boards, pit lane | M2 | ORIGINAL_REQUEST §R4 |
| 16 | Circuit de Monaco | 3.337 km layout, 19 turns, elevation changes, Fairmont hairpin, tunnel, 1 DRS zone, boards, pit lane | M2 | ORIGINAL_REQUEST §R4 |
| 17 | 3D Track Elements & Kerbs | Continuous 3D spline ribbon, red/white alternating beveled kerbs, starting grid, 5-light FIA gantry | M2 | ORIGINAL_REQUEST §R4 |
| 18 | 5 Authentic AI Opponents | Verstappen #1 (Red Bull), Leclerc #16 (Ferrari), Hamilton #44 (Mercedes), Norris #4 (McLaren), Alonso #14 | M4 | ORIGINAL_REQUEST §R5 |
| 19 | 3 AI Difficulty Tiers | Easy (conservative apex, early brake), Medium (clean racing line), Hard (optimal late braking, aggressive) | M4 | ORIGINAL_REQUEST §R5 |
| 20 | AI Racing Behavior & Collision Avoidance | Pure-pursuit path following, curvature speed profiling, OBB SAT collision detection, slipstream tow, 3-5 laps | M4 | ORIGINAL_REQUEST §R5 |
| 21 | Motorsport UI & Main Menu | Start screen, track selection (Monza/Monaco), difficulty selection (Easy/Medium/Hard), controls guide | M5 | ORIGINAL_REQUEST §R6 |
| 22 | Broadcast Television HUD | Speedometer, RPM, P1-P20 timing tower, lap timer, sector delta, vector circuit mini-map | M5 | ORIGINAL_REQUEST §R6 |
| 23 | Post-Race Podium & Results | Leaderboard, total time, fastest lap, gap to leader, telemetry summary | M5 | ORIGINAL_REQUEST §R6 |
| 24 | Dynamic Camera & Audio Engine | Head G-force inertia neck spring, kerb shudder, dynamic FOV, procedural V6 engine sound, gear clack, tyre squeal | M5 | ORIGINAL_REQUEST §R6 |
| 25 | Automated Headless Physics Benchmark | Automated script testing 0-100, 0-200, 200-0 braking, DRS speed delta, slip angle handling, AI lap run | M6 | ORIGINAL_REQUEST §Delivery |
| 26 | Desktop Delivery & PowerShell Launch | Verified source tree with single-command launch via Windows PowerShell | M6 | ORIGINAL_REQUEST §Delivery |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Core Engine, Vehicle Physics & Powertrain | `f1sim/core/`, `f1sim/physics/` (powertrain, transmission, aero, Pacejka tyres, brakes, under/oversteer, DRS) | none | DONE |
| M2 | Official Track Geometry & Environments | `f1sim/track/` (splines, Monza, Monaco, 3D ribbons, kerbs, boards, DRS zones, pit lanes) | M1 | DONE |
| M3 | Cockpit View, Telemetry LCD, Graphics Settings & Keybindings | `f1sim/graphics/`, `f1sim/input/` (ModernGL renderer, cockpit, halo, wheel, PCU-8D LCD, camera G-forces, graphics settings, keybinding remapping) | M1, M2 | DONE |
| M4 | Multi-Tier AI Racing System | `f1sim/ai/` (5 liveries, pure-pursuit line, speed profiles, 3 difficulty tiers, collision avoidance, overtaking) | M1, M2 | DONE |
| M5 | Motorsport UI/UX, Audio & Game Loop | `f1sim/ui/`, `f1sim/audio/`, `main.py` (menus, HUD, results, procedural audio, 5-light start, race lifecycle) | M3, M4 | DONE |
| M6 | E2E Test Suite Pass (Tiers 1-4) & Adversarial Hardening (Tier 5) | `tests/`, full benchmark verification, adversarial edge-case stress testing, delivery check | M5 | DONE |

## Interface Contracts

### `f1sim.physics.vehicle.Vehicle` ↔ `f1sim.graphics.renderer` / `f1sim.ui.hud`
- **Telemetry State Dataclass (`VehicleTelemetry`)**:
  ```python
  @dataclass
  class VehicleTelemetry:
      speed_kmh: float        # Current vehicle speed in km/h
      speed_ms: float         # Speed in m/s
      rpm: float              # Engine RPM (1000 - 15000)
      gear: int               # Current gear (-1=R, 0=N, 1..8)
      throttle: float         # 0.0 .. 1.0
      brake: float            # 0.0 .. 1.0
      steering_angle: float   # Current steering wheel angle in radians (-pi .. +pi)
      drs_available: bool     # True if vehicle is inside a DRS zone
      drs_active: bool        # True if wing flap is currently open
      brake_bias: float       # Front brake bias (e.g. 0.58)
      engine_running: bool    # Ignition status
      slip_angle_front: float # Front tyre slip angle (rad)
      slip_angle_rear: float  # Rear tyre slip angle (rad)
      lateral_g: float        # Lateral acceleration in Gs
      longitudinal_g: float   # Longitudinal acceleration in Gs
      kerb_vibration: float   # 0.0 .. 1.0 amplitude of kerb rumble
  ```

### `f1sim.track.track_spline.Track` ↔ `f1sim.physics.vehicle` & `f1sim.ai.controller`
- **Track Query Interface**:
  ```python
  class Track:
      length: float
      def get_spline_point(self, s: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
          """Returns (position_xyz, tangent_vec, normal_vec) at track distance s."""
      def get_track_width(self, s: float) -> float: ...
      def get_curvature(self, s: float) -> float: ...
      def is_in_drs_zone(self, s: float) -> bool: ...
      def is_on_kerb(self, position: np.ndarray) -> bool: ...
  ```

### `f1sim.ai.driver_profiles.AIDriver` ↔ `f1sim.physics.vehicle.Vehicle`
- **AI Control Output**:
  ```python
  @dataclass
  class ControlCommand:
      steer: float    # -1.0 .. +1.0
      throttle: float # 0.0 .. 1.0
      brake: float    # 0.0 .. 1.0
      shift: int      # -1 = down, 0 = keep, +1 = up
      drs: bool       # Request DRS toggle
  ```
