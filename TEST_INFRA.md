# E2E Test Infra: F1 Desktop Native Simulation Game

## Test Philosophy
- **Requirement-Driven & Opaque-Box**: Tests verify user requirements directly from `ORIGINAL_REQUEST.md` without coupling to internal private methods.
- **Progressive Testability**: Low-level vehicle dynamics and track splines can be tested headlessly; high-level race loops and telemetry can be validated via state transitions and automated benchmarks.
- **Zero-Flicker Automation**: Automated suites run both headlessly (for fast CI validation < 5 seconds) and with off-screen/software OpenGL contexts where graphics verification is required.

## Feature Inventory Mapping
| # | Feature | Requirement Source | Tier 1 (Isolated) | Tier 2 (Boundary) | Tier 3 (Pairwise) | Tier 4 (Workload) |
|---|---------|-------------------|:-----------------:|:-----------------:|:-----------------:|:-----------------:|
| 1 | Desktop Native Platform | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ | ✓ |
| 2 | Cockpit First-Person View | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ | ✓ |
| 3 | Rotating Steering Wheel | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ | ✓ |
| 4 | Telemetry LCD & Rev Lights | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ | ✓ |
| 5 | Dual Input Handling | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ | ✓ |
| 6 | Transmission Shifting | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ | ✓ |
| 7 | DRS Cockpit Control | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ | ✓ |
| 8 | Engine Ignition & Start | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ | ✓ |
| 9 | Brake Bias Adjustment | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ | ✓ |
| 10 | Acceleration Dynamics | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ | ✓ |
| 11 | Braking Dynamics | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ | ✓ |
| 12 | Pacejka Tyre Grip | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ | ✓ |
| 13 | Understeer & Oversteer | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ | ✓ |
| 14 | Aerodynamics & DRS Drag | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ | ✓ |
| 15 | Monza Track Layout | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ | ✓ |
| 16 | Monaco Track Layout | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ | ✓ |
| 17 | Track Elements & Kerbs | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ | ✓ |
| 18 | 5 AI Opponents | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ | ✓ |
| 19 | 3 AI Difficulties | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ | ✓ |
| 20 | AI Racing Intelligence | ORIGINAL_REQUEST §R5 | 5 | 5 | ✓ | ✓ |
| 21 | Motorsport UI & Menus | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ | ✓ |
| 22 | Television Race HUD | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ | ✓ |
| 23 | Post-Race Podium | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ | ✓ |
| 24 | Dynamic Camera & Audio | ORIGINAL_REQUEST §R6 | 5 | 5 | ✓ | ✓ |
| 25 | Headless Benchmark Suite | ORIGINAL_REQUEST §Delivery | 5 | 5 | ✓ | ✓ |
| 26 | PowerShell Delivery Run | ORIGINAL_REQUEST §Delivery | 5 | 5 | ✓ | ✓ |

## Test Architecture & Directory Layout
- **Test Runner**: `tests/run_all_tests.py` (executes all tiers sequentially, prints clear summary, exits with code 0 on all pass, non-zero on failure).
- **Benchmark Suite**: `tests/benchmark_physics_ai.py` (runs high-frequency 1000 Hz physics sub-steps, benchmarks 0-100, 0-200, 200-0 braking, DRS speed delta, and AI lap progression).
- **Tier 1 (Feature Coverage)**: `tests/test_tier1_features.py` (>= 5 unit/contract assertions per feature).
- **Tier 2 (Boundary & Corner Cases)**: `tests/test_tier2_boundaries.py` (Zero throttle, max steering lock, redline RPM, reverse gear lock, out-of-bounds recovery, extreme slip angles).
- **Tier 3 (Pairwise Combinations)**: `tests/test_tier3_pairwise.py` (DRS active under hard braking, manual downshift without throttle blip into hairpin, AI overtaking player during pit lane exit, kerb strikes under full throttle).
- **Tier 4 (Real-World Scenarios)**: `tests/test_tier4_scenarios.py` (Complete race simulations: Monza 3-lap sprint, Monaco street duel, 5-car grid launch from red lights to turn 1, podium classification).

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Pass Criteria |
|---|----------|-------------------|---------------|
| 1 | Monza Full GP Sprint (3 Laps) | F1, F6, F7, F10, F11, F14, F15, F18, F19, F20, F22, F23 | Player & 5 AI finish 3 laps without collision, lap counter reaches 3/3, winner classified |
| 2 | Monaco Precision Street Duel | F2, F3, F4, F9, F12, F13, F16, F17, F20 | Hairpin negotiated at ~45 km/h, understeer/oversteer managed, no wall penetration |
| 3 | Standing Grid 5-Red-Light Launch | F8, F10, F17, F18, F20, F24 | Gantry 5 lights sequence -> lights out -> clutch launch, 0-100 in 2.4-2.8s, clean T1 entry |
| 4 | DRS Straight Overtake Maneuver | F4, F7, F10, F14, F18, F20 | Trailing car enters DRS zone within 1.0s, activates DRS, delta speed +16 km/h, performs overtake |
| 5 | Extreme Brake Bias Cornering Snap | F9, F11, F12, F13 | Shift brake bias to 45% front (55% rear) under high-speed braking -> rear breaks loose into oversteer |
