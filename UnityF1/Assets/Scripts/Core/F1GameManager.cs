using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using F1Sim.Physics;
using F1Sim.Track;
using F1Sim.Cockpit;
using F1Sim.AI;
using F1Sim.UI;
using F1Sim.Audio;

namespace F1Sim.Core
{
    public class F1GameManager : MonoBehaviour
    {
        [Header("Singletons & Managers")]
        public static F1GameManager Instance { get; private set; }

        [Header("Prefabs & Systems")]
        public VehicleDynamics playerVehicle;
        public GameObject aiVehiclePrefab;
        public TrackMeshGenerator trackMeshGen;
        public SceneryBuilder sceneryBuilder;
        public F1MenuManager menuManager;
        public CockpitModelBuilder cockpitBuilder;
        public F1CockpitCamera cockpitCamera;

        [Header("Race Configuration")]
        public int totalLaps = 3;
        public float startGridSpacing = 10.0f;

        private TrackSpline spline;
        private List<F1AIDriver> aiDrivers = new List<F1AIDriver>();
        private float simTime = 0.0f;
        private float playerLapStartTime = 0.0f;
        private int playerCurrentLap = 1;
        private float playerTotalDistance = 0.0f;
        private float lastProjectedS = 0.0f;
        private List<float> playerLapTimes = new List<float>();
        private bool isInitialized = false;
        private bool autoCaptureMode = false;
        private int captureFrameCount = 0;

        private void Awake()
        {
            if (Instance == null) Instance = this;
            else { Destroy(gameObject); return; }

            string[] cmdArgs = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < cmdArgs.Length; i++)
            {
                if (cmdArgs[i] == "--capture-previews")
                {
                    autoCaptureMode = true;
                    break;
                }
            }

            BootstrapSystems();
            ConfigureLightingAndPostProcessing();
        }

        public void InitializeForEditorPreview()
        {
            BootstrapSystems();
            ConfigureLightingAndPostProcessing();
            InitializeWorldAndGrid();
            if (cockpitBuilder != null)
            {
                cockpitBuilder.BuildProceduralCockpit();
            }
        }

        private void BootstrapSystems()
        {
            // 1. Track Mesh System
            if (trackMeshGen == null)
            {
                GameObject trackObj = new GameObject("Track_Mesh_System");
                trackObj.transform.SetParent(transform, false);
                trackMeshGen = trackObj.AddComponent<TrackMeshGenerator>();
                trackMeshGen.trackFilter = trackObj.AddComponent<MeshFilter>();
                trackMeshGen.trackRenderer = trackObj.AddComponent<MeshRenderer>();
                trackMeshGen.trackCollider = trackObj.AddComponent<MeshCollider>();
            }

            // 2. Scenery Builder System
            if (sceneryBuilder == null)
            {
                GameObject sceneryObj = new GameObject("Scenery_Builder_System");
                sceneryObj.transform.SetParent(transform, false);
                sceneryBuilder = sceneryObj.AddComponent<SceneryBuilder>();
            }

            // 3. Player F1 Vehicle
            if (playerVehicle == null)
            {
                GameObject carObj = new GameObject("Player_F1_Vehicle");
                carObj.transform.SetParent(transform, false);
                playerVehicle = carObj.AddComponent<VehicleDynamics>();

                F1CarModelBuilder carModel = carObj.AddComponent<F1CarModelBuilder>();
                carModel.vehicle = playerVehicle;
                carModel.primaryLivery = F1Palette.RacingRed;
                carModel.accentLivery = F1Palette.ModenaYellow;
                carModel.helmetColor = Color.white;
                carModel.isPlayerCar = true;
                carModel.BuildCarModel();
            }

            // 4. Cockpit & Front Chassis Model Builder
            if (cockpitBuilder == null)
            {
                GameObject cockpitObj = new GameObject("Cockpit_Model_System");
                cockpitObj.transform.SetParent(playerVehicle.transform, false);
                cockpitBuilder = cockpitObj.AddComponent<CockpitModelBuilder>();
                cockpitBuilder.vehicle = playerVehicle;
            }

            // 5. Camera & URP Post-Processing
            Camera mainCam = Camera.main;
            if (mainCam == null)
            {
                GameObject camObj = new GameObject("Main Camera");
                mainCam = camObj.AddComponent<Camera>();
                camObj.AddComponent<AudioListener>();
                camObj.tag = "MainCamera";
            }
            mainCam.nearClipPlane = 0.05f;
            mainCam.farClipPlane = 3500.0f;
            mainCam.clearFlags = CameraClearFlags.Skybox;
            mainCam.backgroundColor = F1Palette.SkyHorizon;

            cockpitCamera = mainCam.GetComponent<F1CockpitCamera>();
            if (cockpitCamera == null)
                cockpitCamera = mainCam.gameObject.AddComponent<F1CockpitCamera>();
            cockpitCamera.vehicle = playerVehicle;
            cockpitCamera.mainCamera = mainCam;
            if (playerVehicle != null)
            {
                playerVehicle.OnImpact += (intensity) => cockpitCamera?.TriggerImpact(intensity);
            }

            // 6. Broadcast Menu & HUD System
            if (menuManager == null)
            {
                GameObject menuObj = new GameObject("Menu_UI_System");
                menuObj.transform.SetParent(transform, false);
                menuManager = menuObj.AddComponent<F1MenuManager>();
            }

            // 7. Procedural V6 Turbo-Hybrid Audio Synthesizer (RequireComponent handles single AudioSource)
            F1AudioSynthesizer audioSynth = playerVehicle.GetComponent<F1AudioSynthesizer>();
            if (audioSynth == null)
            {
                audioSynth = playerVehicle.gameObject.AddComponent<F1AudioSynthesizer>();
            }
            audioSynth.vehicle = playerVehicle;
        }

        private void ConfigureLightingAndPostProcessing()
        {
            // 1. Golden-Hour Directional Key Light (per craft-game-floors scene-lighting.md)
            Light sun = FindFirstObjectByType<Light>();
            if (sun == null)
            {
                GameObject sunObj = new GameObject("Sun_Directional_Light");
                sun = sunObj.AddComponent<Light>();
            }
            sun.type = LightType.Directional;
            sun.color = F1Palette.SunGolden;
            sun.intensity = 1.35f;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.82f;
            sun.transform.rotation = Quaternion.Euler(38.0f, -35.0f, 0.0f);

            // 2. Trilinear Gradient Ambient Lighting & Horizon Fog
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = F1Palette.SkyZenith;
            RenderSettings.ambientEquatorColor = F1Palette.SkyHorizon;
            RenderSettings.ambientGroundColor = F1Palette.GroundBounce;
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogColor = F1Palette.SkyHorizon;
            RenderSettings.fogDensity = 0.00038f;

            // 3. URP Global Volume (ACES Tonemapping, HDR Bloom, Vignette, Color Adjustments)
            Camera mainCam = Camera.main;
            if (mainCam != null)
            {
                if (!mainCam.TryGetComponent<UniversalAdditionalCameraData>(out var camData))
                    camData = mainCam.gameObject.AddComponent<UniversalAdditionalCameraData>();
                camData.renderPostProcessing = true;
                camData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
                camData.antialiasingQuality = AntialiasingQuality.High;
            }

            Volume globalVol = FindFirstObjectByType<Volume>();
            if (globalVol == null)
            {
                GameObject volObj = new GameObject("Global_PostProcess_Volume");
                volObj.transform.SetParent(transform, false);
                globalVol = volObj.AddComponent<Volume>();
            }
            globalVol.isGlobal = true;
            globalVol.priority = 1.0f;

            VolumeProfile profile = ScriptableObject.CreateInstance<VolumeProfile>();
            profile.name = "F1_GoldenHour_Profile";

            var tonemap = profile.Add<Tonemapping>(true);
            tonemap.mode.overrideState = true;
            tonemap.mode.value = TonemappingMode.ACES;

            var bloom = profile.Add<Bloom>(true);
            bloom.threshold.overrideState = true;
            bloom.threshold.value = 0.92f;
            bloom.intensity.overrideState = true;
            bloom.intensity.value = 0.38f;
            bloom.scatter.overrideState = true;
            bloom.scatter.value = 0.68f;

            var vignette = profile.Add<Vignette>(true);
            vignette.intensity.overrideState = true;
            vignette.intensity.value = 0.22f;
            vignette.smoothness.overrideState = true;
            vignette.smoothness.value = 0.42f;

            var colorAdj = profile.Add<ColorAdjustments>(true);
            colorAdj.postExposure.overrideState = true;
            colorAdj.postExposure.value = 0.15f;
            colorAdj.contrast.overrideState = true;
            colorAdj.contrast.value = 10.0f;
            colorAdj.saturation.overrideState = true;
            colorAdj.saturation.value = 12.0f;

            globalVol.sharedProfile = profile;
        }

        private void Start()
        {
            InitializeWorldAndGrid();

            if (menuManager != null)
            {
                menuManager.OnTransmissionChanged += HandleTransmissionChanged;
                menuManager.OnSensitivityChanged += HandleSensitivityChanged;
                menuManager.OnCameraCycleRequested += HandleCameraCycle;
                menuManager.OnHaloCycleRequested += () => cockpitBuilder?.CycleHaloOpacity();
                menuManager.OnRestartRequested += ResetRaceSession;
            }
        }

        private void InitializeWorldAndGrid()
        {
            if (isInitialized) return;
            isInitialized = true;

            List<Vector3> waypoints = MonzaTrackData.GetMonzaWaypoints();
            spline = new TrackSpline(waypoints);

            if (trackMeshGen != null)
            {
                trackMeshGen.GenerateTrack(spline, 2.0f);
                trackMeshGen.GenerateKerbs(spline, 1.0f);
                trackMeshGen.SpawnFiaGantry(spline, 18.0f);
            }

            if (sceneryBuilder != null)
            {
                sceneryBuilder.BuildScenery(spline);
            }

            SetupGridPositions();
        }

        private void SetupGridPositions()
        {
            if (spline == null) return;

            // Clear old AI cars if restarting
            for (int i = 0; i < aiDrivers.Count; i++)
            {
                if (aiDrivers[i] != null) Destroy(aiDrivers[i].gameObject);
            }
            aiDrivers.Clear();

            // Player starts at Pole / Grid Slot (s = 0.0m)
            Vector3 startPos = spline.GetPoint(0.0f);
            Vector3 startTan = spline.GetTangent(0.0f);
            spline.GetFrenetFrame(0.0f, out _, out Vector3 lat, out _);

            float poleYawRad = Mathf.Atan2(startTan.z, startTan.x);
            if (playerVehicle != null)
            {
                playerVehicle.state = new VehicleState();
                playerVehicle.SetWorldPosition(startPos - lat * 2.2f, poleYawRad);
                playerTotalDistance = 0.0f;
                lastProjectedS = 0.0f;
            }

            // Spawn 5 3D AI F1 Cars on staggered FIA Starting Grid ahead & alongside player
            // so the player immediately sees rival F1 cars in front at race start!
            List<AIDriverProfile> roster = AIDriverProfile.GetOfficialRoster();
            Color[] accentColors = new Color[]
            {
                new Color(1.0f, 0.80f, 0.0f), // VER Red Bull Yellow/Red
                Color.white,                  // LEC Ferrari White
                new Color(0.0f, 0.90f, 0.82f),// HAM Mercedes Petronas Teal
                new Color(0.12f, 0.14f, 0.18f),// NOR McLaren Anthracite
                new Color(0.75f, 0.95f, 0.0f) // ALO Aston Martin Lime
            };
            Color[] helmetColors = new Color[]
            {
                new Color(1.0f, 0.45f, 0.0f),
                new Color(0.90f, 0.08f, 0.08f),
                new Color(0.55f, 0.15f, 0.85f),
                new Color(0.85f, 1.00f, 0.00f),
                new Color(0.15f, 0.65f, 1.00f)
            };

            float[] gridSOffsets = new float[] { 32.0f, 24.0f, 18.0f, 12.0f, 6.0f };

            for (int i = 0; i < roster.Count; i++)
            {
                float rawS = gridSOffsets[i];
                float gridS = Mathf.Repeat(rawS, spline.Length);
                Vector3 aiPos = spline.GetPoint(gridS);
                Vector3 aiTan = spline.GetTangent(gridS);
                spline.GetFrenetFrame(gridS, out _, out Vector3 aiLat, out _);

                float sideOffset = (i % 2 == 0) ? 2.2f : -2.2f;
                float aiYawRad = Mathf.Atan2(aiTan.z, aiTan.x);

                GameObject aiObj = (aiVehiclePrefab != null) ? Instantiate(aiVehiclePrefab) : new GameObject($"AI_Car_{roster[i].code}");
                aiObj.transform.SetParent(transform, false);

                VehicleDynamics aiPhys = aiObj.GetComponent<VehicleDynamics>();
                if (aiPhys == null) aiPhys = aiObj.AddComponent<VehicleDynamics>();

                F1AIDriver aiDriver = aiObj.GetComponent<F1AIDriver>();
                if (aiDriver == null) aiDriver = aiObj.AddComponent<F1AIDriver>();

                aiDriver.profile = roster[i];
                aiDriver.trackSpline = spline;
                aiDriver.targetLateralD = sideOffset * 0.65f;
                aiPhys.SetWorldPosition(aiPos + aiLat * sideOffset, aiYawRad);

                // Build Full 3D F1 Car Mesh for AI Opponent
                F1CarModelBuilder carVisual = aiObj.AddComponent<F1CarModelBuilder>();
                carVisual.vehicle = aiPhys;
                carVisual.primaryLivery = roster[i].teamColor;
                carVisual.accentLivery = accentColors[i % accentColors.Length];
                carVisual.helmetColor = helmetColors[i % helmetColors.Length];
                carVisual.isPlayerCar = false;
                carVisual.BuildCarModel();

                aiDrivers.Add(aiDriver);
            }
        }

        private void ResetRaceSession()
        {
            simTime = 0.0f;
            playerLapStartTime = 0.0f;
            playerCurrentLap = 1;
            playerLapTimes.Clear();
            SetupGridPositions();
        }

        private void Update()
        {
            if (autoCaptureMode)
            {
                StepAutoCaptureSequence();
            }

            // Keep camera in cinematic orbit while in MainMenu or Settings
            if (cockpitCamera != null && menuManager != null)
            {
                cockpitCamera.isMenuOrbitActive =
                    (menuManager.CurrentState == GameState.MainMenu || menuManager.CurrentState == GameState.Settings);
            }

            if (Input.GetKeyDown(KeyCode.C))
            {
                HandleCameraCycle();
            }
            if (Input.GetKeyDown(KeyCode.H))
            {
                cockpitBuilder?.CycleHaloOpacity();
            }

            if (menuManager != null && menuManager.CurrentState != GameState.Racing)
            {
                if (Input.GetKeyDown(KeyCode.Escape))
                    menuManager.TogglePause();
                return;
            }

            if (Input.GetKeyDown(KeyCode.Escape))
            {
                menuManager?.TogglePause();
                return;
            }

            float dt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.05f);
            simTime += dt;

            // 1. Process Player Input (No Auto-Cornering!)
            float throttle = (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow) || autoCaptureMode) ? 1.0f : 0.0f;
            float brake = (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) ? 1.0f : 0.0f;
            float rawSteer = 0.0f;
            if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) rawSteer += 1.0f;
            if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) rawSteer -= 1.0f;

            float steer = rawSteer * (menuManager != null ? menuManager.SteeringSensitivity : 1.0f);

            // Manual gear shifts & Transmission Mode Toggle
            if (Input.GetKeyDown(KeyCode.E)) playerVehicle?.powertrain?.ShiftUp();
            if (Input.GetKeyDown(KeyCode.Q)) playerVehicle?.powertrain?.ShiftDown();
            if (Input.GetKeyDown(KeyCode.R)) RespawnPlayerOnTrack();
            if (Input.GetKeyDown(KeyCode.T))
            {
                playerVehicle?.powertrain?.ToggleTransmissionMode();
                if (menuManager != null && playerVehicle != null && playerVehicle.powertrain != null)
                {
                    menuManager.CurrentTransmission = playerVehicle.powertrain.transmissionMode;
                }
            }

            // DRS availability & activation
            if (playerVehicle != null && spline != null)
            {
                spline.ProjectPoint(playerVehicle.state.worldPosition, out float sPlayer, out float dPlayer);
                bool canDrs = MonzaTrackData.IsInDrsZone(sPlayer);
                playerVehicle.state.drsAvailable = canDrs;

                if (Input.GetKeyDown(KeyCode.F) && canDrs)
                    playerVehicle.state.drsActive = !playerVehicle.state.drsActive;
                // Auto-close DRS on brake or any steering input (>2 degrees road wheel)
                if (!canDrs || brake > 0.05f || Mathf.Abs(steer) > 0.035f)
                    playerVehicle.state.drsActive = false;

                // Kerb rumble feedback when riding kerbs
                float halfW = MonzaTrackData.TrackWidth * 0.5f;
                playerVehicle.KerbVibration = (Mathf.Abs(dPlayer) >= halfW - 0.4f && Mathf.Abs(dPlayer) <= halfW + 1.0f) ? 1.0f : 0.0f;
            }

            // 2. Step Player Vehicle Physics (120 Hz Substeps)
            if (playerVehicle != null)
            {
                int subSteps = 2;
                float subDt = dt / subSteps;
                for (int s = 0; s < subSteps; s++)
                {
                    playerVehicle.StepPhysics(throttle, brake, steer, subDt);
                }

                if (spline != null)
                {
                    spline.ProjectPoint(playerVehicle.state.worldPosition, out float sProg, out _);
                    float ds = sProg - lastProjectedS;
                    if (ds < -spline.Length * 0.5f) ds += spline.Length;
                    else if (ds > spline.Length * 0.5f) ds -= spline.Length;
                    playerTotalDistance += ds;
                    lastProjectedS = sProg;

                    if (sProg < 50.0f && (simTime - playerLapStartTime) > 25.0f && playerTotalDistance > spline.Length * 0.8f)
                    {
                        playerLapTimes.Add(simTime - playerLapStartTime);
                        playerLapStartTime = simTime;
                        playerCurrentLap++;

                        if (playerCurrentLap > totalLaps)
                        {
                            menuManager?.ShowResults("PLAYER", "PLY", 81.046f,
                                "1. PLY  WINNER\n2. VER  +0.420s\n3. LEC  +1.850s\n4. NOR  +2.310s\n5. HAM  +3.120s\n6. ALO  +4.250s");
                        }
                    }
                }
            }

            // 3. Step 5 AI Opponent Drivers
            for (int i = 0; i < aiDrivers.Count; i++)
            {
                aiDrivers[i].StepAI(dt);
            }

            // 4. Update Live Broadcast Standings & Telemetry HUD
            if (menuManager != null && playerVehicle != null)
            {
                List<DriverStandingEntry> standings = BuildLiveStandings();
                string camLabel = cockpitCamera != null ? cockpitCamera.cameraMode.ToString().ToUpperInvariant() : "COCKPIT";

                List<Vector3> aiPos = new List<Vector3>(aiDrivers.Count);
                List<Color> aiCol = new List<Color>(aiDrivers.Count);
                for (int i = 0; i < aiDrivers.Count; i++)
                {
                    if (aiDrivers[i] != null)
                    {
                        aiPos.Add(aiDrivers[i].transform.position);
                        aiCol.Add(aiDrivers[i].profile != null ? aiDrivers[i].profile.teamColor : Color.yellow);
                    }
                }

                menuManager.UpdateTelemetryHUD(
                    playerVehicle,
                    playerCurrentLap,
                    totalLaps,
                    simTime - playerLapStartTime,
                    camLabel,
                    standings,
                    aiPos,
                    aiCol);
            }
        }

        private List<DriverStandingEntry> BuildLiveStandings()
        {
            List<DriverStandingEntry> list = new List<DriverStandingEntry>(6);
            list.Add(new DriverStandingEntry
            {
                code = "PLY",
                name = "Player",
                teamColor = F1Palette.RacingRed,
                distanceM = playerTotalDistance,
                isPlayer = true
            });

            for (int i = 0; i < aiDrivers.Count; i++)
            {
                F1AIDriver ai = aiDrivers[i];
                if (ai == null || ai.profile == null) continue;
                list.Add(new DriverStandingEntry
                {
                    code = ai.profile.code,
                    name = ai.profile.name,
                    teamColor = ai.profile.teamColor,
                    distanceM = ai.currentS,
                    isPlayer = false
                });
            }

            list.Sort((a, b) => b.distanceM.CompareTo(a.distanceM));
            float leaderDist = list.Count > 0 ? list[0].distanceM : 0.0f;
            for (int i = 0; i < list.Count; i++)
            {
                float distBehind = Mathf.Max(0.0f, leaderDist - list[i].distanceM);
                list[i].gapSec = distBehind / 68.0f; // approx time gap at Monza average speed
            }
            return list;
        }

        private void HandleTransmissionChanged(TransmissionMode mode)
        {
            if (playerVehicle != null && playerVehicle.powertrain != null)
            {
                playerVehicle.powertrain.transmissionMode = mode;
            }
        }

        private void HandleSensitivityChanged(float sens)
        {
            // Applied in Update
        }

        private void HandleCameraCycle()
        {
            if (cockpitCamera != null)
            {
                cockpitCamera.CycleCameraMode();
                if (menuManager != null)
                {
                    menuManager.UpdateTelemetryHUD(
                        playerVehicle,
                        playerCurrentLap,
                        totalLaps,
                        simTime - playerLapStartTime,
                        cockpitCamera.cameraMode.ToString().ToUpperInvariant());
                }
            }
        }

        public void RespawnPlayerOnTrack()
        {
            if (playerVehicle == null || spline == null) return;

            spline.ProjectPoint(playerVehicle.state.worldPosition, out float sPlayer, out _);
            Vector3 centerPos = spline.GetPoint(sPlayer) + Vector3.up * 0.15f;
            Vector3 tan = spline.GetTangent(sPlayer);
            float yawRad = Mathf.Atan2(tan.z, tan.x);

            playerVehicle.SetWorldPosition(centerPos, yawRad);
            playerVehicle.state.vx = 0.0f;
            playerVehicle.state.vy = 0.0f;
            playerVehicle.state.yawRate = 0.0f;
            playerVehicle.state.ax = 0.0f;
            playerVehicle.state.ay = 0.0f;
            playerVehicle.state.throttle = 0.0f;
            playerVehicle.state.brake = 0.0f;
            playerVehicle.state.gear = 1;
            playerVehicle.state.drsActive = false;
            if (playerVehicle.powertrain != null)
            {
                playerVehicle.powertrain.currentGear = 1;
                playerVehicle.powertrain.currentRpm = playerVehicle.powertrain.idleRpm;
            }

            menuManager?.ShowRespawnBanner("ĐÃ ĐẶT LẠI XE TRÊN ĐƯỜNG ĐUA (RESPAWN)");
        }

        private void StepAutoCaptureSequence()
        {
            captureFrameCount++;
            string outDir = @"c:\Users\nguye\PROJECT\F1";
            if (captureFrameCount == 20)
            {
                ScreenCapture.CaptureScreenshot(System.IO.Path.Combine(outDir, "Preview_Runtime_MainMenu.png"));
            }
            else if (captureFrameCount == 25)
            {
                if (menuManager != null) menuManager.SetState(GameState.Racing);
                if (cockpitCamera != null) cockpitCamera.cameraMode = F1CameraMode.CockpitEye;
            }
            else if (captureFrameCount == 60)
            {
                ScreenCapture.CaptureScreenshot(System.IO.Path.Combine(outDir, "Preview_Runtime_Cockpit.png"));
            }
            else if (captureFrameCount == 65)
            {
                if (cockpitCamera != null) cockpitCamera.cameraMode = F1CameraMode.ChaseView;
            }
            else if (captureFrameCount == 85)
            {
                ScreenCapture.CaptureScreenshot(System.IO.Path.Combine(outDir, "Preview_Runtime_Chase.png"));
            }
            else if (captureFrameCount >= 95)
            {
                Application.Quit();
            }
        }
    }
}
