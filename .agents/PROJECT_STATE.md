# Project State: Formula 1 Simulation Racing (Monza Grand Prix)

**Date**: 2026-11-09  
**Repository**: [https://github.com/lostempyrea/F1-Simulation-Racing](https://github.com/lostempyrea/F1-Simulation-Racing)  
**Main Branch**: `main` | **Latest Release Tag**: `v1.0.0`  
**Status**: 100% Focused on Unity 6 URP. Full Monza overhaul, autonomous sensor AI, 3D driver arms, and standalone build complete.

---

## 1. Project Overview & Architecture

- **Sole Interactive Runtime: Unity 6 URP (`UnityF1/` & `Build/F1_GrandPrix.exe`)**:
  - Standalone game executable `Build/F1_GrandPrix.exe` is completely self-contained with zero runtime dependencies on Python.
  - Python (`f1sim/`, `tests/`) is retained strictly as an offline deterministic physics verification benchmark suite (47/47 passing).
- **Official Monza Circuit Overhaul (`MonzaTrackData.cs`, `SceneryBuilder.cs`)**:
  - Reconstructed 100% of Autodromo Nazionale Monza from the official schematic:
    - Canonical 5,793m clockwise loop with non-intersecting layout (minimum separation of 98m between any ribbons).
    - 11 corners: Rettifilo Chicane (T1-T2), Curva Grande (T3), Roggia Chicane (T4-T5), Curva di Lesmo 1 & 2 (T6-T7), Rettifilo del Serraglio, Variante Ascari (T8-T10), Rettifilo Parabolica, and Curva Parabolica Alboreto (T11).
    - 2 DRS zones: Main Straight (s = 150m..1120m) and Serraglio Straight (s = 2850m..4050m).
    - 0.40m extruded 3D solid road slab with double-sided rendering (`_Cull = Off`, `_ZWrite = 1.0f`).
    - 34 official grandstand complexes (numbered 1..35 per Monza map) and pit lane hospitality buildings.
    - Historic Sopraelevata High-Speed Banked Oval overpass crossing high above the Serraglio straight at 26° banking angle.
    - Expanded Monza Parkland terrain (8,000m x 8,000m) and Distant Alpine Mountain horizon ring (4,000m radius).
- **Sensor-Based Map-Agnostic Autonomous Racing AI (`F1AutonomousPilot.cs`, `F1AIDriver.cs`)**:
  - Multi-directional forward & diagonal raycast array (9 probes: -70°, -45°, -25°, -12°, 0°, +12°, +25°, +45°, +70°).
  - Dynamic road boundary sensing and centerline error calculation: $e = (d_R - d_L) / (d_R + d_L)$.
  - PD steering controller with lookahead curvature feedforward ($\kappa$).
  - Adaptive Speed Control: predicts upcoming radius $R$, computes cornering limit $v = \sqrt{\mu g R}$, executes threshold braking prior to apex, and applies exit acceleration.
  - Forward rival detection radar (SphereCast 32m) with intelligent overtaking lane shift bias.
  - Stuck & Crash Recovery State Machine: automatically enters `Reversing` when blocked (< 1.8 m/s with throttle > 0.35s for > 1.4s), shifts to reverse gear (-1), backs away, turns around, and resumes racing.
- **First-Person 3D Driver Arms & Kinematics (`CockpitModelBuilder.cs`, `F1CockpitCamera.cs`)**:
  - White fireproof Nomex suit sleeves with red trim and biometric gloves gripping the butterfly steering wheel at 9 o'clock and 3 o'clock.
  - Real-time kinematic arm articulation in `Update()` dynamically stretching and rotating forearms between shoulder anchors and rotating gloves.
  - Cockpit camera eye positioned at `(0.0f, 0.72f, -0.28f)` with `80°` FOV framing wheel, hands, LCD screen, Halo, mirrors, front wheels, and the circuit.
- **Strict Tier 0 Directive**:
  - **TUYỆT ĐỐI KHÔNG CHẠY INNO SETUP / PACKAGING INSTALLER**. Only standalone executable and test suite verification performed.

---

## 2. Key Files & Deliverables

- `Build/F1_GrandPrix.exe`: Standalone 64-bit Windows game executable (assembled with updated `Assembly-CSharp.dll`).
- `Preview_3D_Monza.png`: High-resolution full overhead aerial render showing the complete 5,793m circuit layout.
- `Preview_3D_Cockpit_Arms.png`: High-resolution first-person cockpit render showcasing 3D driver arms, butterfly wheel, LCD, and grid.
- `Preview_3D_Cockpit.png`: Cockpit perspective render with telemetry display and rival opponents.
- `Preview_3D_Showcase.png`: 3D exterior showcase render at the starting grid.
- `Preview_3D_Chase.png`: Third-person chase camera render.
- `UnityF1/Assets/Scripts/Track/MonzaTrackData.cs`: 51 canonical waypoints forming the official 5,793m Monza circuit.
- `UnityF1/Assets/Scripts/Track/SceneryBuilder.cs`: Grandstands, historic banked oval overpass, terrain, and mountain ring.
- `UnityF1/Assets/Scripts/AI/F1AutonomousPilot.cs`: Map-agnostic multi-raycast autonomous racing AI with stuck recovery.
- `UnityF1/Assets/Scripts/AI/F1AIDriver.cs`: Coordinates roster profiles, grid positions, and delegates physics driving to `F1AutonomousPilot`.
- `UnityF1/Assets/Scripts/Cockpit/CockpitModelBuilder.cs`: 3D driver arms and glove model with real-time kinematic steering tracking.
- `UnityF1/Assets/Editor/F1SceneSetup.cs`: Automated URP scene builder, multi-angle screenshot capture, and standalone player builder.

---

## 3. Verification & Test Results

- **Python Deterministic Physics Suite**: `47 passed in 12.98s` (100% pass rate).
- **Unity Batchmode Build**: Succeeded (`0 errors, 0 warnings`).
- **Preview Visual Inspection**: All 5 high-resolution PNG renders verified and confirmed visually accurate.
- **Auto-Sync Scripts**: `auto_sync.bat` and `scripts/auto_sync.ps1` completely removed as instructed.
