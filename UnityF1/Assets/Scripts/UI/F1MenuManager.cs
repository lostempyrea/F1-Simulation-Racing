using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using F1Sim.Core;
using F1Sim.Physics;
using F1Sim.Cockpit;

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

        private string resultsWinner = "PLAYER";
        private string resultsFastestLap = "1:21.046 (PLY)";
        private string resultsClassification = "1. PLY  WINNER\n2. VER  +0.420s\n3. LEC  +1.850s\n4. NOR  +2.310s\n5. HAM  +3.120s\n6. ALO  +4.250s";

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
            List<DriverStandingEntry> liveStandings = null)
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
            }
            hudCurrentLap = currentLap;
            hudTotalLaps = totalLaps;
            hudLapTimeSec = lapTimeSec;
            if (!string.IsNullOrEmpty(cameraModeName)) hudCameraLabel = cameraModeName;
            if (liveStandings != null && liveStandings.Count > 0) hudStandings = liveStandings;
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
            if (leftRedStripe)
                GUI.DrawTexture(new Rect(rect.x, rect.y, 5.0f, rect.height), texCrimsonAccent);
            if (topRedStripe)
                GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, 4.0f), texCrimsonAccent);
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
            GUILayout.Label("• [T] Đổi Auto/Manual   • [F] Mở DRS   • [C] Camera   • [H] Halo", styleMutedText);

            GUILayout.EndArea();

            // Bottom Right Live Car Showcase Badge
            Rect showcaseRect = new Rect(w - 390, h - 78, 354, 56);
            DrawStyledPanel(showcaseRect, leftRedStripe: true);
            GUI.Label(new Rect(showcaseRect.x + 16, showcaseRect.y + 8, 330, 22), "SCUDERIA F1-26 HYBRID  •  POLE POSITION", styleSubHeader);
            GUI.Label(new Rect(showcaseRect.x + 16, showcaseRect.y + 30, 330, 20), "1.6L V6 Turbo + MGU-K/MGU-H  |  798 kg", styleMutedText);
        }

        private void DrawSettings(float w, float h)
        {
            float panelW = 540.0f;
            float panelH = 490.0f;
            Rect modal = new Rect((w - panelW) * 0.5f, (h - panelH) * 0.5f, panelW, panelH);
            DrawStyledPanel(modal, leftRedStripe: false, topRedStripe: true);

            GUILayout.BeginArea(new Rect(modal.x + 28, modal.y + 24, modal.width - 56, modal.height - 44));
            GUILayout.Label("CÀI ĐẶT XE & ĐỒ HỌA (VEHICLE & GRAPHICS SETTINGS)", styleBrandTitle);
            GUILayout.Label("TÙY CHỈNH HỘP SỐ, ĐỘ NHẠY LÁI VÀ HIỆU NĂNG URP", styleSubHeader);
            GUILayout.Space(18);

            // 1. Transmission Mode
            GUILayout.Label("1. CHẾ ĐỘ HỘP SỐ (TRANSMISSION):", styleSectionLabel);
            GUILayout.BeginHorizontal();
            bool isAuto = (CurrentTransmission == TransmissionMode.Automatic);
            if (GUILayout.Button(isAuto ? "● AUTOMATIC (TỰ ĐỘNG)" : "○ AUTOMATIC (TỰ ĐỘNG)", isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(40)))
            {
                CurrentTransmission = TransmissionMode.Automatic;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            if (GUILayout.Button(!isAuto ? "● MANUAL (SỐ SÀN - E/Q)" : "○ MANUAL (SỐ SÀN - E/Q)", !isAuto ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(40)))
            {
                CurrentTransmission = TransmissionMode.Manual;
                OnTransmissionChanged?.Invoke(CurrentTransmission);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(16);

            // 2. Steering Sensitivity
            GUILayout.Label($"2. ĐỘ NHẠY VÔ LĂNG (STEERING SENSITIVITY): {SteeringSensitivity:F2}x", styleSectionLabel);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("- 0.1", styleSecondaryBtn, GUILayout.Width(64), GUILayout.Height(32)))
            {
                SteeringSensitivity = Mathf.Clamp(SteeringSensitivity - 0.1f, 0.4f, 2.0f);
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            float sliderVal = GUILayout.HorizontalSlider(SteeringSensitivity, 0.4f, 2.0f, GUILayout.Height(32));
            if (Mathf.Abs(sliderVal - SteeringSensitivity) > 0.005f)
            {
                SteeringSensitivity = sliderVal;
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            if (GUILayout.Button("+ 0.1", styleSecondaryBtn, GUILayout.Width(64), GUILayout.Height(32)))
            {
                SteeringSensitivity = Mathf.Clamp(SteeringSensitivity + 0.1f, 0.4f, 2.0f);
                OnSensitivityChanged?.Invoke(SteeringSensitivity);
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(16);

            // 3. Halo Opacity & Camera
            GUILayout.Label("3. TÙY CHỈNH BUỒNG LÁI & GÓC NHÌN:", styleSectionLabel);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("ĐỔI ĐỘ TRONG SUỐT HALO [H]", styleSecondaryBtn, GUILayout.Height(36)))
            {
                OnHaloCycleRequested?.Invoke();
            }
            if (GUILayout.Button($"GÓC NHÌN: {hudCameraLabel} [C]", styleSecondaryBtn, GUILayout.Height(36)))
            {
                OnCameraCycleRequested?.Invoke();
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(16);

            // 4. Graphics Quality Preset
            GUILayout.Label($"4. CHẤT LƯỢNG ĐỒ HỌA URP (CURRENT: {GraphicsPreset}):", styleSectionLabel);
            GUILayout.BeginHorizontal();
            string[] presets = new string[] { "LOW", "MEDIUM", "HIGH", "ULTRA" };
            int[] qLevels = new int[] { 0, 2, 4, 5 };
            for (int i = 0; i < presets.Length; i++)
            {
                bool active = (GraphicsPreset == presets[i]);
                if (GUILayout.Button(presets[i], active ? styleActivePillBtn : styleSecondaryBtn, GUILayout.Height(34)))
                {
                    GraphicsPreset = presets[i];
                    QualitySettings.SetQualityLevel(qLevels[i]);
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(26);

            if (GUILayout.Button("◀   LƯU & QUAY LẠI (BACK TO MAIN MENU)", stylePrimaryBtn, GUILayout.Height(46)))
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

            // 3. TOP-RIGHT TELEMETRY G-FORCE & SHORTCUTS PILL
            Rect topRightRect = new Rect(w - 285.0f, 20.0f, 265.0f, 64.0f);
            DrawStyledPanel(topRightRect, leftRedStripe: true);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 6, 245, 20), $"G-FORCE: LON {hudLongG:+0.0;-0.0;0.0}G  |  LAT {hudLatG:+0.0;-0.0;0.0}G", styleSubHeader);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 26, 245, 18), $"CAM: {hudCameraLabel} [C]   |   HALO [H]", styleBodyText);
            GUI.Label(new Rect(topRightRect.x + 14, topRightRect.y + 43, 245, 18), "[T] Auto/Manual   [F] DRS   [ESC] Pause", styleMutedText);

            // 4. BOTTOM-RIGHT F1 BROADCAST TELEMETRY & SPEEDOMETER CLUSTER
            float clusterW = 300.0f;
            float clusterH = 152.0f;
            Rect clusterRect = new Rect(w - clusterW - 24.0f, h - clusterH - 24.0f, clusterW, clusterH);
            DrawStyledPanel(clusterRect, leftRedStripe: false, topRedStripe: false);

            // 15-LED Shift Light Bar across Top of HUD Cluster
            float ledStartX = clusterRect.x + 14.0f;
            float ledY = clusterRect.y + 10.0f;
            float ledW = (clusterW - 28.0f) / 15.0f - 3.0f;
            for (int i = 0; i < 15; i++)
            {
                float thresh = 9800.0f + i * 240.0f;
                Color ledCol = new Color(0.14f, 0.16f, 0.20f, 1.0f);
                if (hudRpm >= thresh)
                {
                    if (i < 5) ledCol = F1Palette.DrsEmerald;
                    else if (i < 10) ledCol = F1Palette.RacingRed;
                    else ledCol = F1Palette.TelemetryCyan;
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
