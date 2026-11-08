# Project State: Formula 1 Simulation Racing (Monza Grand Prix)

**Date**: 2026-11-09  
**Repository**: [https://github.com/lostempyrea/F1-Simulation-Racing](https://github.com/lostempyrea/F1-Simulation-Racing)  
**Main Branch**: `main` | **Latest Release Tag**: `v1.0.0`  
**Status**: 100% Focused on Unity 6 URP. Python game runner retired. Unity standalone (`Build/F1_GrandPrix.exe`) completely self-contained.

---

## 1. Project Pivot & Architecture Overview
- **Sole Interactive Runtime: Unity 6 URP (`UnityF1/` & `Build/F1_GrandPrix.exe`)**:
  - Per Product Owner directive, Python interactive gameplay (`pygame`/`moderngl` desktop game loop) has been retired.
  - 100% of gameplay, 3D rendering, physics substepping (120 Hz FixedUpdate), procedural Monza circuit, audio synthesis, 5 AI competitors, and broadcast telemetry HUD execute natively in Unity.
  - The compiled standalone game `Build/F1_GrandPrix.exe` is completely self-contained and has zero dependencies on any Python interpreter or libraries.
- **Python Role (`f1sim/`, `tests/`)**:
  - Retained exclusively as an offline mathematical physics benchmark suite and deterministic CI validation harness (47/47 passing).
- **Vehicle Dynamics Model**:
  - Non-linear Pacejka '96 tyre formulation with combined slip friction ellipse and normal load sensitivity.
  - 1.6L V6 Turbocharged Hybrid (ICE + MGU-K) powertrain with 8-speed seamless gearbox, auto-blip downshift, anti-stall, and full reverse gear functionality.
  - Speed-sensitive steering limiter (`k=0.0018`) preventing high-speed snap oversteer.
  - Natural yaw damping (`Mz_damp = -950.0 * r`) with rotational collision stabilizer (`dt * 22.0` damping) preventing pinwheel spinning on barrier impacts.
  - Elevated solid barrier collision response (`Y=0.55m`, `halfExtents=0.92x0.35x2.30m`) strictly targeting authentic obstacles (`Barrier`, `Stand_Backwall`, `Overpass_Pillar`, `Guardrail`, `Pit_Garage`, `Tecpro`).
  - Instant track respawn kinematics on `[R]` key aligned with spline tangent vector and zeroed momentum.

---

## 2. Key Files & Responsibilities
- `Build/F1_GrandPrix.exe`: Primary self-contained 64-bit Windows standalone game executable (no Python runtime required).
- `Play_F1_Unity.bat` & `Play_F1_Simulation.bat`: Desktop launcher scripts directly initiating `Build/F1_GrandPrix.exe`.
- `UnityF1/Assets/Scripts/Track/MonzaTrackData.cs`: Original canonical Monza circuit waypoints preserved from commit `fde1599`, ensuring 100% stable AI car pathfinding and track curvature fidelity.
- `UnityF1/Assets/Scripts/Track/SceneryBuilder.cs`: Dense track clearance precomputation (checked every 5.0m); skips any barrier rail encroaching within 7.0m of any non-adjacent track ribbon, completely eliminating barrier intrusions.
- `UnityF1/Assets/Scripts/Track/TrackMeshGenerator.cs`: 3D solid road slab extrusion (0.40m thickness) generating top road, side skirts, and inverted underbelly quads to permanently eliminate elevated road see-through / backface culling transparency.
- `UnityF1/Assets/Scripts/Core/F1Palette.cs`: Road materials (`Mat_MonzaAsphalt`, `Mat_FiaKerb`) configured with double-sided rendering (`_Cull = Off`), `RenderQueue.Geometry`, Opaque (`_Surface = 0`), and `_ZWrite = 1.0f`.
- `UnityF1/Assets/Scripts/Cockpit/CockpitModelBuilder.cs`: Mercedes W14/W15 reference cockpit POV; butterfly carbon steering wheel at `(0, 0.42, 0.24)`, lowered dashboard cowl/deflector (`Y=0.36m`), and live PCU-8D LCD display.
- `UnityF1/Assets/Scripts/Cockpit/F1CockpitCamera.cs`: Driver eye camera positioned at `(0.0f, 0.72f, -0.28f)` with `basePitchDown = 4.5f` and `baseFov = 80.0f`, providing an authentic unobstructed view of the steering wheel, front wheels, and track ahead.
- `UnityF1/Assets/Scripts/Cockpit/F1CarModelBuilder.cs`: Immediate suppression of helmet sphere and internal monocoque tub in cockpit perspective.
- `UnityF1/Assets/Models/McLarenMP44/`: Imported 1988 McLaren MP4/4 FBX model (`138 MB`) and PBR textures (`BaseColor`, `Metalness`, `Roughness`).
- `main.py`: Retired interactive game loop; redirects desktop launches to `Build/F1_GrandPrix.exe` while maintaining `--benchmark` support for CI.

---

## 3. Completed vs Pending Tasks

### Completed:
- [x] **Project Direction Pivot**: Retired interactive Python game runner. Updated `main.py`, `Play_F1_Simulation.bat`, and `Play_F1_Unity.bat` to execute 100% native Unity 6 standalone game.
- [x] **Standalone Dependency Audit**: Verified `Build/F1_GrandPrix.exe` has zero runtime dependency on Python (runs via UnityPlayer.dll / MonoBleedingEdge).
- [x] **Monza Track Preservation & Clearance**: Reverted `MonzaTrackData.cs` to original commit `fde1599` for perfect AI vehicle handling. Implemented precomputed 7.0m non-adjacent clearance check in `SceneryBuilder.cs` to prevent barrier crossing.
- [x] **Solid 3D Road Slab**: Extruded 0.40m road slab with top surface, side skirts, and underbelly quads in `TrackMeshGenerator.cs`; double-sided rendering (`_Cull = Off`, `_ZWrite = 1`) in `F1Palette.cs`.
- [x] **Cockpit POV & Steering Wheel Redesign**: Removed visual obstruction from glare shield/deflector; positioned butterfly steering wheel at `(0, 0.42, 0.24)`; camera eye at `(0, 0.72, -0.28)` with pitch `4.5°`, FOV `80°`; hid helmet sphere in CockpitEye view.
- [x] **Python Physics Benchmark Suite**: 47/47 passing (`pytest tests/test_milestone1_physics.py tests/test_challenger_m1_iter2_handling.py tests/test_high_speed_stability.py`).
- [x] **Unity Standalone Rebuild**: `Build/F1_GrandPrix.exe` compiled cleanly with 0 errors and 0 warnings. Verification screenshots captured (`Preview_3D_Cockpit.png`, `Preview_3D_Showcase.png`, `Preview_3D_Chase.png`).
- [x] **Documentation**: Updated `README.md` and `.agents/PROJECT_STATE.md`.

### Pending / Next Actions:
- [ ] **Installer Policy (Tier 0 Directive)**: TUYỆT ĐỐI KHÔNG đóng gói installer (Inno Setup) cho đến khi người dùng yêu cầu trực tiếp.
- [ ] User playtesting of standalone game `Build/F1_GrandPrix.exe`.
