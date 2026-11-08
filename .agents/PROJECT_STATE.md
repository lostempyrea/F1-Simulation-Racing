# Project State: Formula 1 Simulation Racing (Monza Grand Prix)

**Date**: 2026-11-09  
**Repository**: [https://github.com/lostempyrea/F1-Simulation-Racing](https://github.com/lostempyrea/F1-Simulation-Racing)  
**Main Branch**: `main` | **Latest Release Tag**: `v1.0.0`  
**Status**: 100% Focused on Unity 6 URP. Bug fixes for steering animation smoothing, high-speed autonomous AI & recovery, and elevation slope noclip prevention complete.

---

## 1. Project Overview & Architecture

- **Sole Interactive Runtime: Unity 6 URP (`UnityF1/` & `Build/F1_GrandPrix.exe`)**:
  - Standalone game executable `Build/F1_GrandPrix.exe` is completely self-contained with zero runtime dependencies on Python.
  - Python (`f1sim/`, `tests/`) is retained strictly as an offline deterministic physics verification benchmark suite (47/47 passing).
- **Steering Wheel & Driver Arms Kinematics (Fix for Bug 1)**:
  - In `CockpitModelBuilder.cs`, implemented exponential smoothing interpolation (`Time.deltaTime * 16.0f`) on steering wheel rotation (`currentSmoothedSteerDeg`), completely eliminating jerky discrete jumps / teleporting when steering with keyboard or gamepad.
  - Applied `Vector3.Lerp` and `Quaternion.Slerp` (`Time.deltaTime * 24.0f`) to driver forearm positions, rotations, and scales for fluid, natural arm articulation tracking.
- **Aggressive High-Speed AI & Teleport Recovery (Fix for Bug 2)**:
  - In `F1AutonomousPilot.cs`:
    - Extended forward sensor raycast array to `150.0m` (from 55m) for earlier corner detection on high-speed Monza straights.
    - Raised top speed to `96.0 m/s` (~345 km/h) and cornering lateral G limit to `3.2 G` (31.4 m/s²), utilizing realistic modern F1 aerodynamic downforce.
    - Implemented full-throttle attack on straights and corner exits (`throttle = 1.0f`), with trail-braking into corner entries.
    - Reduced stuck detection threshold to `0.85s` (from 1.4s), increased reverse recovery time to `2.4s`, with aggressive turnaround steering lock (`±0.92`).
    - Added fail-safe Teleport Recovery: if an AI car is stuck 3 times consecutively or total recovery exceeds 5.0s, it instantly snaps to the nearest track centerline with 100 km/h launch momentum in 2nd gear.
- **Deep Road Slab & Vehicle Elevation Tracking (Fix for Bug 3)**:
  - In `TrackMeshGenerator.cs`:
    - Increased baseline road slab thickness from 0.40m to `1.20m`.
    - Implemented dynamic slab deepening on slopes and elevated sections (Lesmo, Ascari): underbelly vertices automatically extend down past terrain level (`Y = -0.35m`), permanently eliminating any gaps or see-through cracks beneath the track.
    - Full 3D concave `MeshCollider` covers top surface, side skirts, and underbelly.
  - In `VehicleDynamics.cs`:
    - Implemented real-time vertical downward ground probe (`Physics.Raycast(origin + Vector3.up * 8.0f, Vector3.down, 25.0f)`).
    - Smoothly tracks track elevation changes (`Mathf.MoveTowards(..., dt * 25.0f)`) and immediately snaps the chassis up if dipping below the road surface, permanently preventing the car from noclipping or falling under elevated track sections.
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
- `UnityF1/Assets/Scripts/Track/TrackMeshGenerator.cs`: 1.20m deep road slab with slope embedding.
- `UnityF1/Assets/Scripts/Track/SceneryBuilder.cs`: Grandstands, historic banked oval overpass, terrain, and mountain ring.
- `UnityF1/Assets/Scripts/Physics/VehicleDynamics.cs`: Vehicle dynamics with ground probe elevation tracking and noclip prevention.
- `UnityF1/Assets/Scripts/AI/F1AutonomousPilot.cs`: High-speed autonomous racing AI with 150m raycasts and teleport recovery.
- `UnityF1/Assets/Scripts/AI/F1AIDriver.cs`: Coordinates roster profiles, grid positions, and delegates physics driving to `F1AutonomousPilot`.
- `UnityF1/Assets/Scripts/Cockpit/CockpitModelBuilder.cs`: Smooth steering wheel rotation and fluid driver arms kinematics.
- `UnityF1/Assets/Editor/F1SceneSetup.cs`: Automated URP scene builder, multi-angle screenshot capture, and standalone player builder.

---

## 3. Verification & Test Results

- **Python Deterministic Physics Suite**: `47 passed in 10.73s` (100% pass rate).
- **Unity Batchmode Build**: Succeeded (`0 errors, 0 warnings`).
- **Preview Visual Inspection**: All 5 high-resolution PNG renders verified and confirmed visually accurate.
