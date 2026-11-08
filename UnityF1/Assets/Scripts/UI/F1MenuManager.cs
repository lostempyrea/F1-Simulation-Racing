using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using F1Sim.Core;
using F1Sim.Physics;
using F1Sim.Cockpit;
using F1Sim.Track;
using F1Sim.Audio;

namespace F1Sim.UI
{
    public enum GameState
    {
        MainMenu,
        Settings,
        Racing,
        Paused,
        Results
    }

    public enum MinimapMode
    {
        NorthUp,
        HeadingUp
    }

    [Serializable]
    public class DriverStandingEntry
    {
        public string code;
        public string name;
        public Color teamColor;
        public float distanceM;
        public float gapSec;
        public bool isPlayer;
    }

    public class F1MenuManager : MonoBehaviour
    {
        [Header("Menu Canvas Panels (Optional)")]
        public GameObject mainMenuPanel;
        public GameObject settingsPanel;
        public GameObject inRaceHudPanel;
        public GameObject pausePanel;
        public GameObject resultsPanel;

        [Header("Runtime State")]
        public GameState CurrentState { get; private set; } = GameState.MainMenu;
        public TransmissionMode CurrentTransmission { get; set; } = TransmissionMode.Automatic;
        public MinimapMode CurrentMinimapMode { get; set; } = MinimapMode.HeadingUp;
        public float SteeringSensitivity { get; set; } = 1.0f;
        public string GraphicsPreset { get; set; } = "ULTRA";

        public event Action<TransmissionMode> OnTransmissionChanged;
        public event Action<float> OnSensitivityChanged;
        public event Action OnCameraCycleRequested;
        public event Action OnHaloCycleRequested;
        public event Action OnRestartRequested;

        // Live Telemetry Cache for Broadcast HUD
        private float hudSpeedKmh = 0.0f;
        private int hudGear = 1;
        private float hudRpm = 4000.0f;
        private float hudThrottle = 0.0f;
        private float hudBrake = 0.0f;
        private bool hudDrsActive = false;
        private bool hudDrsAvailable = false;
        private int hudCurrentLap = 1;
        private int hudTotalLaps = 3;
        private float hudLapTimeSec = 0.0f;
        private float hudLongG = 0.0f;
        private float hudLatG = 0.0f;
        private string hudCameraLabel = "COCKPIT";
        private List<DriverStandingEntry> hudStandings = new List<DriverStandingEntry>();

        // 2D Circuit Radar Minimap
        private Vector3 hudPlayerPos;
        private float hudPlayerYaw;
        private List<Vector3> hudAiPositions = new List<Vector3>();
        private List<Color> hudAiColors = new List<Color>();
        private Vector2[] minimapTrackPoints = null;
        private bool[] minimapDrsZones = null;
        private const float MapMinX = -210.0f;
        private const float MapMaxX = 310.0f;
        private const float MapMinZ = -120.0f;
        private const float MapMaxZ = 3580.0f;

        private string resultsWinner = "PLAYER";
        private string resultsFastestLap = "1:21.046 (PLY)";
        private string resultsClassification = "1. PLY  WINNER\n2. VER  +0.420s\n3. LEC  +1.850s\n4. NOR  +2.310s\n5. HAM  +3.120s\n6. ALO  +4.250s";

        // Audio Settings Cache
        private F1AudioSynthesizer cachedAudioSynth;
        private float audioMasterVolume = 0.80f;
        private float audioEngineVolume = 0.85f;
        private float audioSfxVolume = 0.85f;
        private bool audioIsMuted = false;

        // Respawn Banner Notification
        private float respawnBannerTimer = 0.0f;
        private string respawnBannerText = "";

        public void ShowRespawnBanner(string message, float duration = 2.5f)
        {
            respawnBannerText = message;
            respawnBannerTimer = duration;
        }

        private F1AudioSynthesizer GetAudioSynthesizer()
        {
            if (cachedAudioSynth == null)
            {
                cachedAudioSynth = UnityEngine.Object.FindFirstObjectByType<F1AudioSynthesizer>();
            }
            return cachedAudioSynth;
        }

        // Procedural UI Textures & Styles
        private Texture2D texObsidianPanel;
        private Texture2D texCardBg;
        private Texture2D texCrimsonAccent;
        private Texture2D texGoldAccent;
        private Texture2D texEmeraldPill;
        private Texture2D texBtnNormal;
        private Texture2D texBtnHover;
        private Texture2D texBtnPrimary;
        private Texture2D texWhite;

        private GUIStyle styleBrandTitle;
        private GUIStyle styleSubHeader;
        private GUIStyle styleSectionLabel;
        private GUIStyle styleBodyText;
        private GUIStyle styleMutedText;
        private GUIStyle stylePrimaryBtn;
        private GUIStyle styleSecondaryBtn;
        private GUIStyle styleActivePillBtn;
        private GUIStyle styleHudSpeed;
        private GUIStyle styleHudGear;
        private GUIStyle styleHudUnit;
        private GUIStyle styleTowerRow;
        private GUIStyle styleBadge;
        private bool stylesReady = false;

        private void Start()
        {
            InitializeDefaultStandings();
            SetState(GameState.MainMenu);
        }

        private void InitializeDefaultStandings()
        {
            if (hudStandings.Count > 0) return;
            hudStandings.Add(new DriverStandingEntry { code = "PLY", name = "Player", teamColor = F1Palette.RacingRed, gapSec = 0.0f, isPlayer = true });
            hudStandings.Add(new DriverStandingEntry { code = "VER", name = "Verstappen", teamColor = new Color(0.12f, 0.25f, 0.95f), gapSec = 0.42f, isPlayer = false });
            hudStandings.Add(new DriverStandingEntry { code = "LEC", name = "Leclerc", teamColor = new Color(0.88f, 0.05f, 0.05f), gapSec = 1.18f, isPlayer = false });
            hudStandings.Add(new DriverStandingEntry { code = "NOR", name = "Norris", teamColor = new Color(1.00f, 0.53f, 0.00f), gapSec = 1.94f, isPlayer = false });
            hudStandings.Add(new DriverStandingEntry { code = "HAM", name = "Hamilton", teamColor = new Color(0.00f, 0.82f, 0.75f), gapSec = 2.65f, isPlayer = false });
            hudStandings.Add(new DriverStandingEntry { code = "ALO", name = "Alonso", teamColor = new Color(0.00f, 0.48f, 0.38f), gapSec = 3.40f, isPlayer = false });
        }

        public void SetState(GameState newState)
        {
            CurrentState = newState;

            if (mainMenuPanel != null) mainMenuPanel.SetActive(newState == GameState.MainMenu);
            if (settingsPanel != null) settingsPanel.SetActive(newState == GameState.Settings);
            if (inRaceHudPanel != null) inRaceHudPanel.SetActive(newState == GameState.Racing);
            if (pausePanel != null) pausePanel.SetActive(newState == GameState.Paused);
            if (resultsPanel != null) resultsPanel.SetActive(newState == GameState.Results);

            Time.timeScale = (newState == GameState.Paused) ? 0.0f : 1.0f;
        }

        public void OnClickStartRace() => SetState(GameState.Racing);
        public void OnClickOpenSettings() => SetState(GameState.Settings);
        public void OnClickBackToMain() => SetState(GameState.MainMenu);
        public void OnClickResume() => SetState(GameState.Racing);

        public void OnClickRestart()
        {
            OnRestartRequested?.Invoke();
            SetState(GameState.Racing);
        }

        public void TogglePause()
        {
            if (CurrentState == GameState.Racing) SetState(GameState.Paused);
            else if (CurrentState == GameState.Paused) SetState(GameState.Racing);
        }

        public void UpdateTelemetryHUD(
            VehicleDynamics player,
            int currentLap,
            int totalLaps,
            float lapTimeSec,
            string cameraModeName,
            List<DriverStandingEntry> liveStandings = null,
            List<Vector3> aiPositions = null,
            List<Color> aiColors = null)
        {
            if (player != null)
            {
                hudSpeedKmh = player.SpeedKmh;
                hudGear = player.state.gear;
                hudRpm = player.state.rpm;
                hudThrottle = player.state.throttle;
                hudBrake = player.state.brake;
                hudDrsActive = player.state.drsActive;
                hudDrsAvailable = player.state.drsAvailable;
                hudLongG = player.LongitudinalG;
                hudLatG = player.LateralG;
                hudPlayerPos = player.state.worldPosition;
                hudPlayerYaw = player.state.yawAngle;
            }
            hudCurrentLap = currentLap;
            hudTotalLaps = totalLaps;
            hudLapTimeSec = lapTimeSec;
            if (!string.IsNullOrEmpty(cameraModeName)) hudCameraLabel = cameraModeName;
            if (liveStandings != null && liveStandings.Count > 0) hudStandings = liveStandings;
            if (aiPositions != null) hudAiPositions = aiPositions;
            if (aiColors != null) hudAiColors = aiColors;
        }

        public void UpdateHUD(float speedKmh, int gear, float lapTimeSec, string timingTowerStr)
        {
            hudSpeedKmh = speedKmh;
            hudGear = gear;
            hudLapTimeSec = lapTimeSec;
        }

        public void ShowResults(string winnerName, string fastestLapDriver, float bestTime, string classification)
        {
            resultsWinner = winnerName;
            int min = Mathf.FloorToInt(bestTime / 60.0f);
            float sec = bestTime % 60.0f;
            resultsFastestLap = $"{min}:{sec:00.000} ({fastestLapDriver})";
            resultsClassification = classification;
            SetState(GameState.Results);
        }

        private Texture2D MakeSolidTex(Color col)
        {
            Texture2D t = new Texture2D(2, 2, TextureFormat.RGBA32, false) { wrapMode = TextureWrapMode.Clamp };
            t.SetPixels(new Color[] { col, col, col, col });
            t.Apply();
            return t;
        }

        private void EnsureStyles()
        {
            if (stylesReady && texWhite != null) return;

            texWhite         = MakeSolidTex(Color.white);
            texObsidianPanel = MakeSolidTex(new Color(0.045f, 0.055f, 0.080f, 0.90f));
            texCardBg        = MakeSolidTex(new Color(0.085f, 0.105f, 0.145f, 0.92f));
            texCrimsonAccent = MakeSolidTex(new Color(0.880f, 0.024f, 0.000f, 1.00f));
            texGoldAccent    = MakeSolidTex(new Color(1.000f, 0.780f, 0.000f, 1.00f));
            texEmeraldPill   = MakeSolidTex(new Color(0.000f, 0.780f, 0.320f, 0.95f));
            texBtnNormal     = MakeSolidTex(new Color(0.110f, 0.135f, 0.185f, 0.95f));
            texBtnHover      = MakeSolidTex(new Color(0.200f, 0.240f, 0.320f, 0.98f));
            texBtnPrimary    = MakeSolidTex(new Color(0.840f, 0.030f, 0.020f, 0.98f));

            styleBrandTitle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 26,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft
            };
            styleBrandTitle.normal.textColor = Color.white;

            styleSubHeader = new GUIStyle(GUI.skin.label)
            {
                fontSize = 13,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft,
                wordWrap = false
            };
            styleSubHeader.normal.textColor = new Color(1.0f, 0.78f, 0.0f);

            styleSectionLabel = new GUIStyle(GUI.skin.label)
            {
                fontSize = 12,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft,
                wordWrap = false
            };
            styleSectionLabel.normal.textColor = new Color(0.62f, 0.70f, 0.82f);

            styleBodyText = new GUIStyle(GUI.skin.label)
            {
                fontSize = 13,
                fontStyle = FontStyle.Normal,
                alignment = TextAnchor.MiddleLeft
            };
            styleBodyText.normal.textColor = new Color(0.92f, 0.95f, 0.98f);

            styleMutedText = new GUIStyle(GUI.skin.label)
            {
                fontSize = 12,
                fontStyle = FontStyle.Normal,
                alignment = TextAnchor.MiddleLeft
            };
            styleMutedText.normal.textColor = new Color(0.58f, 0.65f, 0.75f);

            stylePrimaryBtn = new GUIStyle(GUI.skin.button)
            {
                fontSize = 15,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            stylePrimaryBtn.normal.background = texBtnPrimary;
            stylePrimaryBtn.hover.background = texCrimsonAccent;
            stylePrimaryBtn.active.background = texGoldAccent;
            stylePrimaryBtn.normal.textColor = Color.white;
            stylePrimaryBtn.hover.textColor = Color.white;
            stylePrimaryBtn.active.textColor = Color.black;

            styleSecondaryBtn = new GUIStyle(GUI.skin.button)
            {
                fontSize = 13,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            styleSecondaryBtn.normal.background = texBtnNormal;
            styleSecondaryBtn.hover.background = texBtnHover;
            styleSecondaryBtn.active.background = texCrimsonAccent;
            styleSecondaryBtn.normal.textColor = new Color(0.90f, 0.94f, 0.98f);
            styleSecondaryBtn.hover.textColor = Color.white;
            styleSecondaryBtn.active.textColor = Color.white;

            styleActivePillBtn = new GUIStyle(styleSecondaryBtn);
            styleActivePillBtn.normal.background = texEmeraldPill;
            styleActivePillBtn.hover.background = texEmeraldPill;
            styleActivePillBtn.normal.textColor = new Color(0.02f, 0.06f, 0.04f);
            styleActivePillBtn.hover.textColor = Color.black;

            styleHudSpeed = new GUIStyle(GUI.skin.label)
            {
                fontSize = 40,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleRight
            };
            styleHudSpeed.normal.textColor = Color.white;

            styleHudGear = new GUIStyle(GUI.skin.label)
            {
                fontSize = 54,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            styleHudGear.normal.textColor = new Color(1.0f, 0.82f, 0.0f);

            styleHudUnit = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleRight
            };
            styleHudUnit.normal.textColor = new Color(0.65f, 0.72f, 0.82f);

            styleTowerRow = new GUIStyle(GUI.skin.label)
            {
                fontSize = 13,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleLeft
            };
            styleTowerRow.normal.textColor = Color.white;

            styleBadge = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                fontStyle = FontStyle.Bold,
                alignment = TextAnchor.MiddleCenter
            };
            styleBadge.normal.textColor = Color.white;

            stylesReady = true;
        }

        private void DrawStyledPanel(Rect rect, bool leftRedStripe = true, bool topRedStripe = false)
        {
            GUI.DrawTexture(rect, texObsidianPanel);

            // Subtle 1px glassmorphism border (#1a202c / #10151f)
            DrawSolidRect(new Rect(rect.x, rect.y, rect.width, 1.0f), new Color(0.24f, 0.28f, 0.36f, 0.45f));
            DrawSolidRect(new Rect(rect.x, rect.yMax - 1.0f, rect.width, 1.0f), new Color(0.12f, 0.14f, 0.18f, 0.55f));
            DrawSolidRect(new Rect(rect.x, rect.y, 1.0f, rect.height), new Color(0.24f, 0.28f, 0.36f, 0.45f));
            DrawSolidRect(new Rect(rect.xMax - 1.0f, rect.y, 1.0f, rect.height), new Color(0.12f, 0.14f, 0.18f, 0.55f));

            if (leftRedStripe)
                GUI.DrawTexture(new Rect(rect.x, rect.y, 4.0f, rect.height), texCrimsonAccent);
            if (topRedStripe)
                GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, 3.0f), texCrimsonAccent);
        }

        private void DrawSolidRect(Rect rect, Color col)
        {
            Color prev = GUI.color;
            GUI.color = col;
            GUI.DrawTexture(rect, texWhite);
            GUI.color = prev;
        }

        private void OnGUI()
        {
            EnsureStyles();

            float w = Screen.width;
            float h = Screen.height;

            switch (CurrentState)
            {
                case GameState.MainMenu:
                    DrawMainMenu(w, h);
                    break;
                case GameState.Settings:
                    DrawSettings(w, h);
                    break;
                case GameState.Racing:
                    DrawRacingHUD(w, h);
                    break;
                case GameState.Paused:
                    DrawRacingHUD(w, h);
                    DrawPauseMenu(w, h);
                    break;
                case GameState.Results:
                    DrawResultsPodium(w, h);
                    break;
            }
        }

        private void DrawMainMenu(float w, float h)
        {
            // Top Broadcast Championship Header Bar
            Rect topBar = new Rect(0, 0, w, 46);
            GUI.DrawTexture(topBar, texObsidianPanel);
            GUI.DrawTexture(new Rect(0, 44, w, 2), texCrimsonAccent);
            DrawSolidRect(new Rect(28, 10, 56, 26), F1Palette.RacingRed);
            GUI.Label(new Rect(28, 10, 56, 26), "F1 26", styleBadge);
            GUI.Label(new Rect(96, 10, 720, 26), "FIA FORMULA ONE WORLD CHAMPIONSHIP  |  AUTODROMO NAZIONALE MONZA", styleSubHeader);

            // Left Broadcast Dock Panel (leaves right 65% of screen open for live 3D Orbiting F1 Car!)
            float panelW = 450.0f;
            float panelH = Mathf.Min(575.0f, h - 100.0f);
            Rect dockRect = new Rect(36.0f, 68.0f, panelW, panelH);
            DrawStyledPanel(dockRect, leftRedStripe: true, topRedStripe: false);

            GUILayout.BeginArea(new Rect(dockRect.x + 24, dockRect.y + 20, dockRect.width - 44, dockRect.height - 36));

            GUILayout.Label("GRAN PREMIO D'ITALIA", styleBrandTitle);
            GUILayout.Label("MONZA CIRCUIT  •  5.793 KM  •  11 CORNERS  •  2 DRS ZONES", styleSubHeader);
            GUILayout.Space(16);

            // Quick Transmission Selector right on Main Menu
            GUILayout.Label("CHẾ ĐỘ HỘP SỐ (TRANSMISSION MODE)", styleSectionLabel);
            GUILayout.Space(4);
            GUILayout.BeginHorizontal();
            bool isAuto = (CurrentTransmission == TransmissionMode.Automatic);
            if (GUILayout.Button(isAuto ? "● TỰ ĐỘNG (AUTO)" : "○ TỰ ĐỘNG (AUTO)", isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(36)))
            {
                CurrentTransmission = TransmissionMode.Automatic;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            if (GUILayout.Button(!isAuto ? "● SỐ SÀN (MANUAL E/Q)" : "○ SỐ SÀN (MANUAL E/Q)", !isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(36)))
            {
                CurrentTransmission = TransmissionMode.Manual;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(12);

            // Quick Camera Selector
            GUILayout.Label($"GÓC NHÌN CAMERA (CURRENT: {hudCameraLabel})", styleSectionLabel);
            GUILayout.Space(4);
            if (GUILayout.Button("🎥  ĐỔI GÓC NHÌN (COCKPIT / T-CAM / CHASE) [PHÍM C]", styleSecondaryBtn, GUILayout.Height(34)))
            {
                OnCameraCycleRequested?.Invoke();
            }

            GUILayout.Space(18);

            // Primary Start Race CTA
            if (GUILayout.Button("▶   BẮT ĐẦU CHẶNG ĐUA (START GRAND PRIX)", stylePrimaryBtn, GUILayout.Height(52)))
            {
                OnClickStartRace();
            }
            GUILayout.Space(10);

            if (GUILayout.Button("⚙   CÀI ĐẶT CHI TIẾT (SETTINGS & SENSITIVITY)", styleSecondaryBtn, GUILayout.Height(40)))
            {
                OnClickOpenSettings();
            }
            GUILayout.Space(8);

            if (GUILayout.Button("✕   THOÁT TRÒ CHƠI (QUIT TO DESKTOP)", styleSecondaryBtn, GUILayout.Height(36)))
            {
                Application.Quit();
            }

            GUILayout.Space(16);
            GUILayout.Label("BẢNG ĐIỀU KHIỂN NHANH (RACE CONTROLS):", styleSectionLabel);
            GUILayout.Label("• [W / S] : Đạp Ga (1000 HP Hybrid) / Phanh Carbon (4.5G)", styleBodyText);
            GUILayout.Label("• [A / D] : Đánh lái tự do (Đã tắt hoàn toàn tự cua góc)", styleBodyText);
            GUILayout.Label("• [E / Q] : Tăng số / Giảm số (Khi chọn Số Sàn - Manual)", styleBodyText);
            GUILayout.Label("• [T] Đổi Auto/Manual   • [F] Mở DRS   • [R] Đặt lại xe khi kẹt   • [C] Camera   • [H] Halo", styleMutedText);

            GUILayout.EndArea();

            // Bottom Right Live Car Showcase Badge
            Rect showcaseRect = new Rect(w - 390, h - 78, 354, 56);
            DrawStyledPanel(showcaseRect, leftRedStripe: true);
            GUI.Label(new Rect(showcaseRect.x + 16, showcaseRect.y + 8, 330, 22), "SCUDERIA F1-26 HYBRID  •  POLE POSITION", styleSubHeader);
            GUI.Label(new Rect(showcaseRect.x + 16, showcaseRect.y + 30, 330, 20), "1.6L V6 Turbo + MGU-K/MGU-H  |  798 kg", styleMutedText);
        }

        private void DrawSettings(float w, float h)
        {
            float panelW = 560.0f;
            float panelH = 710.0f;
            Rect modal = new Rect((w - panelW) * 0.5f, (h - panelH) * 0.5f, panelW, panelH);
            DrawStyledPanel(modal, leftRedStripe: false, topRedStripe: true);

            GUILayout.BeginArea(new Rect(modal.x + 28, modal.y + 18, modal.width - 56, modal.height - 32));
            GUILayout.Label("CÀI ĐẶT XE & ĐỒ HỌA (VEHICLE & GRAPHICS SETTINGS)", styleBrandTitle);
            GUILayout.Label("TÙY CHỈNH HỘP SỐ, ĐỘ NHẠY LÁI, ĐỒ HỌA VÀ ÂM THANH", styleSubHeader);
            GUILayout.Space(12);

            // 1. Transmission Mode
            GUILayout.Label("1. CHẾ ĐỘ HỘP SỐ (TRANSMISSION):", styleSectionLabel);
            GUILayout.BeginHorizontal();
            bool isAuto = (CurrentTransmission == TransmissionMode.Automatic);
            if (GUILayout.Button(isAuto ? "● AUTOMATIC (TỰ ĐỘNG)" : "○ AUTOMATIC (TỰ ĐỘNG)", isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(34)))
            {
                CurrentTransmission = TransmissionMode.Automatic;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            if (GUILayout.Button(!isAuto ? "● MANUAL (SỐ SÀN - E/Q)" : "○ MANUAL (SỐ SÀN - E/Q)", !isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(34)))
            {
                CurrentTransmission = TransmissionMode.Manual;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            // 2. Steering Sensitivity
            GUILayout.Label($"2. ĐỘ NHẠY VÔ LĂNG (STEERING SENSITIVITY): {SteeringSensitivity:F2}x", styleSectionLabel);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("- 0.1", styleSecondaryBtn, GUILayout.Width(64), GUILayout.Height(28)))
            {
                SteeringSensitivity = Mathf.Clamp(SteeringSensitivity - 0.1f, 0.4f, 2.0f);
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            float sliderVal = GUILayout.HorizontalSlider(SteeringSensitivity, 0.4f, 2.0f, GUILayout.Height(28));
            if (Mathf.Abs(sliderVal - SteeringSensitivity) > 0.005f)
            {
                SteeringSensitivity = sliderVal;
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            if (GUILayout.Button("+ 0.1", styleSecondaryBtn, GUILayout.Width(64), GUILayout.Height(28)))
            {
                SteeringSensitivity = Mathf.Clamp(SteeringSensitivity + 0.1f, 0.4f, 2.0f);
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            // 3. Halo Opacity & Camera
            GUILayout.Label("3. TÙY CHỈNH BUỒNG LÁI & GÓC NHÌN:", styleSectionLabel);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("ĐỔI ĐỘ TRONG SUỐT HALO [H]", styleSecondaryBtn, GUILayout.Height(30)))
            {
                OnHaloCycleRequested?.Invoke();
            }
            if (GUILayout.Button($"GÓC NHÌN: {hudCameraLabel} [C]", styleSecondaryBtn, GUILayout.Height(30)))
            {
                OnCameraCycleRequested?.Invoke();
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            // 4. Graphics Quality Preset
            GUILayout.Label($"4. CHẤT LƯỢNG ĐỒ HỌA URP (CURRENT: {GraphicsPreset}):", styleSectionLabel);
            GUILayout.BeginHorizontal();
            string[] presets = new string[] { "LOW", "MEDIUM", "HIGH", "ULTRA" };
            int[] qLevels = new int[] { 0, 2, 4, 5 };
            for (int i = 0; i < presets.Length; i++)
            {
                bool active = (GraphicsPreset == presets[i]);
                if (GUILayout.Button(presets[i], active ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(28)))
                {
                    GraphicsPreset = presets[i];
                    QualitySettings.SetQualityLevel(qLevels[i]);
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            // 5. Audio Settings
            GUILayout.Label("5. CÀI ĐẶT ÂM THANH (AUDIO SETTINGS):", styleSectionLabel);
            F1AudioSynthesizer synth = GetAudioSynthesizer();
            if (synth != null)
            {
                audioMasterVolume = synth.masterVolume;
                audioEngineVolume = synth.engineVolume;
                audioSfxVolume = synth.sfxVolume;
                audioIsMuted = synth.isMuted;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Âm lượng tổng (Master): {Mathf.RoundToInt(audioMasterVolume * 100.0f)}%", styleBodyText, GUILayout.Width(220));
            float newMaster = GUILayout.HorizontalSlider(audioMasterVolume, 0.0f, 1.0f, GUILayout.Height(20));
            if (Mathf.Abs(newMaster - audioMasterVolume) > 0.01f)
            {
                audioMasterVolume = newMaster;
                if (synth != null) synth.masterVolume = audioMasterVolume;
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Động cơ V6 Turbo: {Mathf.RoundToInt(audioEngineVolume * 100.0f)}%", styleBodyText, GUILayout.Width(220));
            float newEng = GUILayout.HorizontalSlider(audioEngineVolume, 0.0f, 1.0f, GUILayout.Height(20));
            if (Mathf.Abs(newEng - audioEngineVolume) > 0.01f)
            {
                audioEngineVolume = newEng;
                if (synth != null)
                {
                    synth.engineVolume = audioEngineVolume;
                    synth.turboVolume = audioEngineVolume * 0.9f;
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            GUILayout.Label($"Hiệu ứng SFX: {Mathf.RoundToInt(audioSfxVolume * 100.0f)}%", styleBodyText, GUILayout.Width(220));
            float newSfx = GUILayout.HorizontalSlider(audioSfxVolume, 0.0f, 1.0f, GUILayout.Height(20));
            if (Mathf.Abs(newSfx - audioSfxVolume) > 0.01f)
            {
                audioSfxVolume = newSfx;
                if (synth != null) synth.sfxVolume = audioSfxVolume;
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(2);
            string muteLabel = audioIsMuted ? "🔇 ÂM THANH: ĐÃ TẮT (CLICK ĐỂ BẬT LẠI)" : "🔊 ÂM THANH: ĐANG BẬT (CLICK ĐỂ TẮT TIẾNG)";
            if (GUILayout.Button(muteLabel, audioIsMuted ? styleSecondaryBtn : styleActivePillBtn, GUILayout.Height(28)))
            {
                audioIsMuted = !audioIsMuted;
                if (synth != null) synth.isMuted = audioIsMuted;
            }

            GUILayout.Space(10);

            // 6. Minimap Radar Orientation Setting
            GUILayout.Label("6. HƯỚNG BẢN ĐỒ RADAR (MINIMAP ORIENTATION):", styleSectionLabel);
            GUILayout.BeginHorizontal();
            bool isHeadingUp = (CurrentMinimapMode == MinimapMode.HeadingUp);
            if (GUILayout.Button(isHeadingUp ? "● XOAY THEO XE (HEADING-UP)" : "○ XOAY THEO XE (HEADING-UP)", isHeadingUp ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(30)))
            {
                CurrentMinimapMode = MinimapMode.HeadingUp;
            }
            if (GUILayout.Button(!isHeadingUp ? "● CỐ ĐỊNH BẮC (NORTH-UP)" : "○ CỐ ĐỊNH BẮC (NORTH-UP)", !isHeadingUp ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(30)))
            {
                CurrentMinimapMode = MinimapMode.NorthUp;
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(14);

            if (GUILayout.Button("◀   LƯU & QUAY LẠI (BACK TO MAIN MENU)", stylePrimaryBtn, GUILayout.Height(38)))
            {
                OnClickBackToMain();
            }
            GUILayout.EndArea();
        }

        private void DrawRacingHUD(float w, float h)
        {
            // 1. TOP-LEFT OFFICIAL F1 TV TIMING TOWER
            float towerW = 224.0f;
            float rowH = 28.0f;
            float towerH = 44.0f + hudStandings.Count * rowH + 10.0f;
            Rect towerRect = new Rect(20.0f, 20.0f, towerW, towerH);
            DrawStyledPanel(towerRect, leftRedStripe: false, topRedStripe: true);

            // F1 Header inside Timing Tower
            DrawSolidRect(new Rect(towerRect.x + 10, towerRect.y + 10, 38, 22), F1Palette.RacingRed);
            GUI.Label(new Rect(towerRect.x + 10, towerRect.y + 10, 38, 22), "F1", styleBadge);
            GUI.Label(new Rect(towerRect.x + 56, towerRect.y + 10, 160, 22), $"LAP {hudCurrentLap} / {hudTotalLaps}", styleSubHeader);

            for (int i = 0; i < hudStandings.Count; i++)
            {
                DriverStandingEntry entry = hudStandings[i];
                float ry = towerRect.y + 40.0f + i * rowH;

                if (entry.isPlayer)
                    DrawSolidRect(new Rect(towerRect.x + 4, ry, towerW - 8, rowH - 2), new Color(0.88f, 0.04f, 0.02f, 0.28f));

                // Position Box
                Color posBoxCol = (i == 0) ? F1Palette.RacingRed : new Color(0.16f, 0.19f, 0.25f, 1.0f);
                DrawSolidRect(new Rect(towerRect.x + 10, ry + 3, 24, 20), posBoxCol);
                GUI.Label(new Rect(towerRect.x + 10, ry + 3, 24, 20), (i + 1).ToString(), styleBadge);

                // Team Color Vertical Bar
                DrawSolidRect(new Rect(towerRect.x + 40, ry + 3, 4, 20), entry.teamColor);

                // Driver 3-letter Code
                GUI.Label(new Rect(towerRect.x + 52, ry + 2, 60, 22), entry.code, styleTowerRow);

                // Interval / Leader Text
                string gapTxt = (i == 0) ? "LEADER" : $"+{entry.gapSec:F2}s";
                GUI.Label(new Rect(towerRect.x + 122, ry + 2, 92, 22), gapTxt, styleMutedText);
            }

            // 2. TOP-CENTER LAP TIMER & CIRCUIT BANNER
            int min = Mathf.FloorToInt(hudLapTimeSec / 60.0f);
            float sec = hudLapTimeSec % 60.0f;
            Rect timerRect = new Rect((w - 310.0f) * 0.5f, 20.0f, 310.0f, 50.0f);
            DrawStyledPanel(timerRect, leftRedStripe: true);
            GUI.Label(new Rect(timerRect.x + 16, timerRect.y + 5, 285, 20), "MONZA GRAND PRIX  •  SESSION TIME", styleSectionLabel);
            GUI.Label(new Rect(timerRect.x + 16, timerRect.y + 24, 285, 22), $"{min:D2}:{sec:00.000}", styleSubHeader);

            // Respawn Notification Banner
            if (respawnBannerTimer > 0.0f)
            {
                respawnBannerTimer -= Time.deltaTime;
                float bw = 460.0f;
                float bh = 32.0f;
                Rect bannerRect = new Rect((w - bw) * 0.5f, 76.0f, bw, bh);
                DrawSolidRect(bannerRect, new Color(0.88f, 0.04f, 0.02f, 0.92f));
                DrawSolidRect(new Rect(bannerRect.x, bannerRect.y, bannerRect.width, 1.0f), F1Palette.ModenaYellow);
                DrawSolidRect(new Rect(bannerRect.x, bannerRect.yMax - 1.0f, bannerRect.width, 1.0f), F1Palette.ModenaYellow);
                GUI.Label(bannerRect, respawnBannerText, styleBadge);
            }

            // 3. TOP-RIGHT TELEMETRY G-FORCE & SHORTCUTS PILL
            Rect topRightRect = new Rect(w - 285.0f, 20.0f, 265.0f, 64.0f);
            DrawStyledPanel(topRightRect, leftRedStripe: true);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 6, 245, 20), $"G-FORCE: LON {hudLongG:+0.0;-0.0;0.0}G  |  LAT {hudLatG:+0.0;-0.0;0.0}G", styleSubHeader);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 26, 245, 18), $"CAM: {hudCameraLabel} [C]   |   HALO [H]", styleBodyText);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 43, 245, 18), "[T] Auto/Man  [F] DRS  [R] Respawn", styleMutedText);

            // 4. BOTTOM-RIGHT F1 BROADCAST TELEMETRY & SPEEDOMETER CLUSTER
            float clusterW = 300.0f;
            float clusterH = 152.0f;
            Rect clusterRect = new Rect(w - clusterW - 24.0f, h - clusterH - 24.0f, clusterW, clusterH);
            DrawStyledPanel(clusterRect, leftRedStripe: false, topRedStripe: false);

            // 15-LED Shift Light Bar across Top of HUD Cluster (Green -> Yellow -> Red -> Redline Flash)
            float ledStartX = clusterRect.x + 14.0f;
            float ledY = clusterRect.y + 10.0f;
            float ledW = (clusterW - 28.0f) / 15.0f - 3.0f;
            bool isRedline = hudRpm >= 12800.0f;
            Color flashCol = (Mathf.Sin(Time.time * 40.0f) > 0) ? Color.magenta : Color.cyan;

            for (int i = 0; i < 15; i++)
            {
                float thresh = 9500.0f + i * 235.0f;
                Color ledCol = new Color(0.12f, 0.14f, 0.18f, 1.0f);
                if (hudRpm >= thresh)
                {
                    if (isRedline && i >= 10)
                    {
                        ledCol = flashCol;
                    }
                    else if (i < 5)
                    {
                        ledCol = F1Palette.DrsEmerald;
                    }
                    else if (i < 10)
                    {
                        ledCol = F1Palette.ModenaYellow;
                    }
                    else if (i < 14)
                    {
                        ledCol = F1Palette.RacingRed;
                    }
                    else
                    {
                        ledCol = isRedline ? flashCol : F1Palette.RacingRed;
                    }
                }
                DrawSolidRect(new Rect(ledStartX + i * (ledW + 3.0f), ledY, ledW, 8.0f), ledCol);
            }

            // Throttle & Brake Vertical Telemetry Bars (Left side of cluster)
            float barBaseY = clusterRect.y + 30.0f;
            float barMaxH = 82.0f;
            // Brake Bar (Red)
            DrawSolidRect(new Rect(clusterRect.x + 16, barBaseY, 12, barMaxH), new Color(0.15f, 0.16f, 0.20f));
            float brkH = Mathf.Clamp01(hudBrake) * barMaxH;
            if (brkH > 0.5f)
                DrawSolidRect(new Rect(clusterRect.x + 16, barBaseY + (barMaxH - brkH), 12, brkH), F1Palette.RacingRed);
            // Throttle Bar (Green)
            DrawSolidRect(new Rect(clusterRect.x + 34, barBaseY, 12, barMaxH), new Color(0.15f, 0.16f, 0.20f));
            float thrH = Mathf.Clamp01(hudThrottle) * barMaxH;
            if (thrH > 0.5f)
                DrawSolidRect(new Rect(clusterRect.x + 34, barBaseY + (barMaxH - thrH), 12, thrH), F1Palette.DrsEmerald);

            // Giant Gear Box (Center of cluster)
            Rect gearBox = new Rect(clusterRect.x + 58, clusterRect.y + 28, 72, 84);
            GUI.DrawTexture(gearBox, texCardBg);
            string gearStr = (hudGear == 0) ? "N" : (hudGear < 0 ? "R" : hudGear.ToString());
            GUI.Label(gearBox, gearStr, styleHudGear);

            // Speed & RPM Readout (Right side of cluster)
            GUI.Label(new Rect(clusterRect.x + 136, clusterRect.y + 26, 146, 48), $"{Mathf.RoundToInt(hudSpeedKmh)}", styleHudSpeed);
            GUI.Label(new Rect(clusterRect.x + 136, clusterRect.y + 72, 146, 18), "KM / H", styleHudUnit);
            GUI.Label(new Rect(clusterRect.x + 136, clusterRect.y + 92, 146, 18), $"{Mathf.RoundToInt(hudRpm)} RPM", styleHudUnit);

            // Bottom Status Pills: DRS & Transmission Mode
            Rect drsPill = new Rect(clusterRect.x + 16, clusterRect.y + 118, 114, 24);
            Color drsCol = hudDrsActive ? F1Palette.DrsEmerald : (hudDrsAvailable ? F1Palette.ModenaYellow : new Color(0.18f, 0.21f, 0.27f));
            DrawSolidRect(drsPill, drsCol);
            string drsLabel = hudDrsActive ? "DRS ACTIVE" : (hudDrsAvailable ? "DRS READY [F]" : "DRS LOCKED");
            GUI.Label(drsPill, drsLabel, styleBadge);

            Rect transPill = new Rect(clusterRect.x + 138, clusterRect.y + 118, 146, 24);
            bool isAutoMode = (CurrentTransmission == TransmissionMode.Automatic);
            DrawSolidRect(transPill, isAutoMode ? new Color(0.16f, 0.28f, 0.45f) : new Color(0.55f, 0.22f, 0.05f));
            GUI.Label(transPill, isAutoMode ? "GEAR: AUTO [T]" : "GEAR: MANUAL [E/Q]", styleBadge);

            // 5. BOTTOM-LEFT 2D CIRCUIT RADAR MINIMAP
            DrawMinimap(w, h);
        }

        private void EnsureMinimapPoints()
        {
            if (minimapTrackPoints != null) return;
            List<Vector3> wps = MonzaTrackData.GetMonzaWaypoints();
            TrackSpline sp = new TrackSpline(wps);
            int samples = 240;
            minimapTrackPoints = new Vector2[samples];
            minimapDrsZones = new bool[samples];
            for (int i = 0; i < samples; i++)
            {
                float s = (i / (float)samples) * sp.Length;
                Vector3 pt = sp.GetPoint(s);
                minimapTrackPoints[i] = new Vector2(pt.x, pt.z);
                minimapDrsZones[i] = MonzaTrackData.IsInDrsZone(s);
            }
        }

        private void DrawMinimap(float w, float h)
        {
            EnsureMinimapPoints();
            float mapW = 190.0f;
            float mapH = 210.0f;
            Rect mapRect = new Rect(24.0f, h - mapH - 24.0f, mapW, mapH);
            DrawStyledPanel(mapRect, leftRedStripe: true, topRedStripe: false);

            string title = (CurrentMinimapMode == MinimapMode.HeadingUp) ? "MONZA • HEADING-UP" : "MONZA • NORTH-UP";
            GUI.Label(new Rect(mapRect.x + 10, mapRect.y + 4, mapW - 20, 18), title, styleSectionLabel);

            Rect inner = new Rect(mapRect.x + 10.0f, mapRect.y + 24.0f, mapW - 20.0f, mapH - 32.0f);

            if (CurrentMinimapMode == MinimapMode.HeadingUp)
            {
                Vector2 center = new Vector2(inner.x + inner.width * 0.5f, inner.y + inner.height * 0.55f);
                float cosY = Mathf.Cos(hudPlayerYaw);
                float sinY = Mathf.Sin(hudPlayerYaw);
                float radarScale = 0.050f;

                // 1. Draw Rotated Circuit Outline
                for (int i = 0; i < minimapTrackPoints.Length; i++)
                {
                    Vector2 pt = minimapTrackPoints[i];
                    float relX = pt.x - hudPlayerPos.x;
                    float relZ = pt.y - hudPlayerPos.z;

                    float localX = relX * sinY - relZ * cosY;
                    float localFwd = relX * cosY + relZ * sinY;

                    float px = center.x + localX * radarScale;
                    float py = center.y - localFwd * radarScale;

                    if (!inner.Contains(new Vector2(px, py))) continue;

                    Color dotCol = minimapDrsZones[i] ? F1Palette.DrsEmerald : new Color(0.42f, 0.48f, 0.58f, 0.85f);
                    DrawSolidRect(new Rect(px - 1.0f, py - 1.0f, 2.5f, 2.5f), dotCol);
                }

                // 2. Start/Finish Line Indicator
                float relSfx = 0.0f - hudPlayerPos.x;
                float relSfz = 0.0f - hudPlayerPos.z;
                float sfLocalX = relSfx * sinY - relSfz * cosY;
                float sfLocalFwd = relSfx * cosY + relSfz * sinY;
                float sfpx = center.x + sfLocalX * radarScale;
                float sfpy = center.y - sfLocalFwd * radarScale;
                if (inner.Contains(new Vector2(sfpx, sfpy)))
                {
                    DrawSolidRect(new Rect(sfpx - 4, sfpy - 1, 8, 2), Color.white);
                }

                // 3. Draw AI Cars
                for (int i = 0; i < hudAiPositions.Count; i++)
                {
                    Vector3 aiPos = hudAiPositions[i];
                    float relX = aiPos.x - hudPlayerPos.x;
                    float relZ = aiPos.z - hudPlayerPos.z;

                    float localX = relX * sinY - relZ * cosY;
                    float localFwd = relX * cosY + relZ * sinY;

                    float apx = center.x + localX * radarScale;
                    float apy = center.y - localFwd * radarScale;

                    if (!inner.Contains(new Vector2(apx, apy))) continue;

                    Color aiCol = (i < hudAiColors.Count) ? hudAiColors[i] : F1Palette.ModenaYellow;
                    DrawSolidRect(new Rect(apx - 2.5f, apy - 2.5f, 5.0f, 5.0f), aiCol);
                }

                // 4. Draw Player Car Fixed at Center pointing straight UP
                DrawSolidRect(new Rect(center.x - 4.5f, center.y - 4.5f, 9.0f, 9.0f), Color.white);
                DrawSolidRect(new Rect(center.x - 3.0f, center.y - 3.0f, 6.0f, 6.0f), F1Palette.RacingRed);
                // Fixed Forward Heading Indicator Arrow (pointing straight up)
                DrawSolidRect(new Rect(center.x - 1.0f, center.y - 12.0f, 2.5f, 7.0f), Color.yellow);
                DrawSolidRect(new Rect(center.x - 2.5f, center.y - 10.0f, 5.5f, 2.0f), Color.yellow);
            }
            else
            {
                // North-Up Mode (Fixed circuit orientation)
                // Draw Circuit Outline
                for (int i = 0; i < minimapTrackPoints.Length; i++)
                {
                    Vector2 pt = minimapTrackPoints[i];
                    float nx = Mathf.Clamp01((pt.x - MapMinX) / (MapMaxX - MapMinX));
                    float nz = Mathf.Clamp01((pt.y - MapMinZ) / (MapMaxZ - MapMinZ));
                    float px = inner.x + nx * inner.width;
                    float py = inner.y + (1.0f - nz) * inner.height;

                    Color dotCol = minimapDrsZones[i] ? F1Palette.DrsEmerald : new Color(0.42f, 0.48f, 0.58f, 0.85f);
                    DrawSolidRect(new Rect(px - 1.0f, py - 1.0f, 2.5f, 2.5f), dotCol);
                }

                // Start/Finish Line Indicator
                float sfnx = Mathf.Clamp01((0.0f - MapMinX) / (MapMaxX - MapMinX));
                float sfnz = Mathf.Clamp01((0.0f - MapMinZ) / (MapMaxZ - MapMinZ));
                float sfpx = inner.x + sfnx * inner.width;
                float sfpy = inner.y + (1.0f - sfnz) * inner.height;
                DrawSolidRect(new Rect(sfpx - 4, sfpy - 1, 8, 2), Color.white);

                // Draw AI Cars
                for (int i = 0; i < hudAiPositions.Count; i++)
                {
                    Vector3 aiPos = hudAiPositions[i];
                    float anx = Mathf.Clamp01((aiPos.x - MapMinX) / (MapMaxX - MapMinX));
                    float anz = Mathf.Clamp01((aiPos.z - MapMinZ) / (MapMaxZ - MapMinZ));
                    float apx = inner.x + anx * inner.width;
                    float apy = inner.y + (1.0f - anz) * inner.height;
                    Color aiCol = (i < hudAiColors.Count) ? hudAiColors[i] : F1Palette.ModenaYellow;
                    DrawSolidRect(new Rect(apx - 2.5f, apy - 2.5f, 5.0f, 5.0f), aiCol);
                }

                // Draw Player Car (Bright Outline + Red Core + Heading Tick)
                float pnx = Mathf.Clamp01((hudPlayerPos.x - MapMinX) / (MapMaxX - MapMinX));
                float pnz = Mathf.Clamp01((hudPlayerPos.z - MapMinZ) / (MapMaxZ - MapMinZ));
                float ppx = inner.x + pnx * inner.width;
                float ppy = inner.y + (1.0f - pnz) * inner.height;

                DrawSolidRect(new Rect(ppx - 4.5f, ppy - 4.5f, 9.0f, 9.0f), Color.white);
                DrawSolidRect(new Rect(ppx - 3.0f, ppy - 3.0f, 6.0f, 6.0f), F1Palette.RacingRed);

                // Heading Vector Tick
                float headingAngle = hudPlayerYaw;
                float tickLen = 8.0f;
                float tx = ppx + Mathf.Cos(headingAngle) * tickLen;
                float ty = ppy - Mathf.Sin(headingAngle) * tickLen;
                DrawSolidRect(new Rect(tx - 1.0f, ty - 1.0f, 2.5f, 2.5f), Color.yellow);
            }
        }

        private void DrawPauseMenu(float w, float h)
        {
            DrawSolidRect(new Rect(0, 0, w, h), new Color(0.02f, 0.03f, 0.05f, 0.65f));

            float panelW = 420.0f;
            float panelH = 360.0f;
            Rect modal = new Rect((w - panelW) * 0.5f, (h - panelH) * 0.5f, panelW, panelH);
            DrawStyledPanel(modal, leftRedStripe: false, topRedStripe: true);

            GUILayout.BeginArea(new Rect(modal.x + 28, modal.y + 24, modal.width - 56, modal.height - 44));
            GUILayout.Label("TẠM DỪNG CHẶNG ĐUA (PAUSED)", styleBrandTitle);
            GUILayout.Label("AUTODROMO NAZIONALE MONZA", styleSubHeader);
            GUILayout.Space(22);

            if (GUILayout.Button("▶   TIẾP TỤC ĐUA (RESUME RACE)", stylePrimaryBtn, GUILayout.Height(46)))
                OnClickResume();
            GUILayout.Space(10);

            if (GUILayout.Button("↻   ĐUA LẠI TỪ ĐẦU (RESTART SESSION)", styleSecondaryBtn, GUILayout.Height(40)))
                OnClickRestart();
            GUILayout.Space(10);

            if (GUILayout.Button("⚙   CÀI ĐẶT HỘP SỐ & ĐỘ NHẠY (SETTINGS)", styleSecondaryBtn, GUILayout.Height(40)))
                OnClickOpenSettings();
            GUILayout.Space(10);

            if (GUILayout.Button("◀   VỀ MÀN HÌNH CHÍNH (MAIN MENU)", styleSecondaryBtn, GUILayout.Height(40)))
                OnClickBackToMain();

            GUILayout.EndArea();
        }

        private void DrawResultsPodium(float w, float h)
        {
            DrawSolidRect(new Rect(0, 0, w, h), new Color(0.02f, 0.03f, 0.05f, 0.78f));

            float panelW = 520.0f;
            float panelH = 460.0f;
            Rect modal = new Rect((w - panelW) * 0.5f, (h - panelH) * 0.5f, panelW, panelH);
            DrawStyledPanel(modal, leftRedStripe: false, topRedStripe: true);

            GUILayout.BeginArea(new Rect(modal.x + 28, modal.y + 24, modal.width - 56, modal.height - 44));
            GUILayout.Label("KẾT QUẢ CHẶNG ĐUA (RACE CLASSIFICATION)", styleBrandTitle);
            GUILayout.Label($"🏆 NGƯỜI CHIẾN THẮNG: {resultsWinner}   |   ⚡ FASTEST LAP: {resultsFastestLap}", styleSubHeader);
            GUILayout.Space(18);

            GUILayout.Label("BẢNG XẾP HẠNG CHÍNH THỨC:", styleSectionLabel);
            GUILayout.Label(resultsClassification, styleBodyText);
            GUILayout.Space(24);

            if (GUILayout.Button("↻   ĐUA CHẶNG MỚI (RACE AGAIN)", stylePrimaryBtn, GUILayout.Height(46)))
                OnClickRestart();
            GUILayout.Space(10);

            if (GUILayout.Button("◀   VỀ MENU CHÍNH (MAIN MENU)", styleSecondaryBtn, GUILayout.Height(40)))
                OnClickBackToMain();

            GUILayout.EndArea();
        }
    }
}
