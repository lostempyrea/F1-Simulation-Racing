using UnityEngine;
using F1Sim.Core;
using F1Sim.Physics;

namespace F1Sim.Cockpit
{
    /// <summary>
    /// Renders a live McLaren PCU-8D Telemetry Display onto a procedural Texture2D
    /// on the F1 steering wheel, plus updates the 15 3D emissive shift-light LEDs
    /// with authentic F1 sequence: 4 Green -> 4 Yellow -> 4 Red -> 3 Purple/Blue,
    /// followed by full 15-LED strobe at rev limiter.
    /// </summary>
    public class TelemetryLCD : MonoBehaviour
    {
        public VehicleDynamics vehicle;
        public Renderer lcdQuadRenderer;
        public Renderer[] shiftLedRenderers; // 15 3D LED capsules on the steering wheel brow

        private Texture2D lcdTex;
        private Color32[] pixelBuffer;
        private const int TexW = 160;
        private const int TexH = 96;
        private Material lcdMat;
        private Material[] ledMats;
        private float refreshTimer = 0.0f;
        private float phaseTime = 0.0f;

        private static readonly Color LedOff    = new Color(0.06f, 0.07f, 0.09f);
        private static readonly Color LedGreen  = new Color(0.00f, 1.00f, 0.35f);
        private static readonly Color LedYellow = new Color(1.00f, 0.82f, 0.00f);
        private static readonly Color LedRed    = new Color(1.00f, 0.08f, 0.08f);
        private static readonly Color LedPurple = new Color(0.80f, 0.15f, 1.00f);

        // Compact 5x7 bitmap font for 0-9, A-Z, %, ., -, :
        private static readonly string[] GlyphMap = new string[]
        {
            "0:01110,10001,10011,10101,11001,10001,01110",
            "1:00100,01100,00100,00100,00100,00100,01110",
            "2:01110,10001,00001,00110,01000,10000,11111",
            "3:11110,00001,00001,01110,00001,00001,11110",
            "4:00010,00110,01010,10010,11111,00010,00010",
            "5:11111,10000,11110,00001,00001,10001,01110",
            "6:00110,01000,10000,11110,10001,10001,01110",
            "7:11111,00001,00010,00100,01000,01000,01000",
            "8:01110,10001,10001,01110,10001,10001,01110",
            "9:01110,10001,10001,01111,00001,00010,01100",
            "N:10001,11001,10101,10011,10001,10001,10001",
            "R:11110,10001,10001,11110,10100,10010,10001",
            "D:11110,10001,10001,10001,10001,10001,11110",
            "S:01111,10000,10000,01110,00001,00001,11110",
            "K:10001,10010,10100,11000,10100,10010,10001",
            "M:10001,11011,10101,10101,10001,10001,10001",
            "H:10001,10001,10001,11111,10001,10001,10001",
            "B:11110,10001,10001,11110,10001,10001,11110",
            "P:11110,10001,10001,11110,10000,10000,10000",
            "A:01110,10001,10001,11111,10001,10001,10001",
            "U:10001,10001,10001,10001,10001,10001,01110",
            "T:11111,00100,00100,00100,00100,00100,00100",
            "O:01110,10001,10001,10001,10001,10001,01110",
            ".:00000,00000,00000,00000,00000,01100,01100",
            "%:11001,11010,00100,01000,10110,00110,00000"
        };

        private void Start()
        {
            InitializeLcdDisplay();
        }

        public void InitializeLcdDisplay()
        {
            if (lcdTex == null)
            {
                lcdTex = new Texture2D(TexW, TexH, TextureFormat.RGBA32, false)
                {
                    name = "Tex_PCU8D_LCD",
                    filterMode = FilterMode.Point,
                    wrapMode = TextureWrapMode.Clamp
                };
                pixelBuffer = new Color32[TexW * TexH];
            }

            if (lcdQuadRenderer == null) lcdQuadRenderer = GetComponent<Renderer>();
            if (lcdQuadRenderer != null)
            {
                Material tmpl = Resources.Load<Material>("F1_TexturedLit");
                lcdMat = (tmpl != null)
                    ? new Material(tmpl) { name = "Mat_PCU8D_Live" }
                    : new Material(F1Palette.GetLitShader()) { name = "Mat_PCU8D_Live" };
                lcdMat.mainTexture = lcdTex;
                if (lcdMat.HasProperty("_BaseMap")) lcdMat.SetTexture("_BaseMap", lcdTex);
                if (lcdMat.HasProperty("_BaseColor")) lcdMat.SetColor("_BaseColor", Color.white);
                lcdMat.EnableKeyword("_EMISSION");
                if (lcdMat.HasProperty("_EmissionMap")) lcdMat.SetTexture("_EmissionMap", lcdTex);
                if (lcdMat.HasProperty("_EmissionColor")) lcdMat.SetColor("_EmissionColor", Color.white * 1.5f);
                lcdQuadRenderer.sharedMaterial = lcdMat;
            }

            if (shiftLedRenderers != null && shiftLedRenderers.Length > 0)
            {
                Material tmpl = Resources.Load<Material>("F1_TexturedLit");
                ledMats = new Material[shiftLedRenderers.Length];
                for (int i = 0; i < shiftLedRenderers.Length; i++)
                {
                    if (shiftLedRenderers[i] != null)
                    {
                        Material m = (tmpl != null)
                            ? new Material(tmpl) { name = $"Mat_ShiftLED_{i}" }
                            : new Material(F1Palette.GetLitShader()) { name = $"Mat_ShiftLED_{i}" };
                        m.mainTexture = null;
                        if (m.HasProperty("_BaseMap")) m.SetTexture("_BaseMap", null);
                        if (m.HasProperty("_EmissionMap")) m.SetTexture("_EmissionMap", null);
                        m.EnableKeyword("_EMISSION");
                        shiftLedRenderers[i].sharedMaterial = m;
                        ledMats[i] = m;
                    }
                }
            }

            UpdateShiftLeds(11800.0f);
            RedrawLcdTexture(1, 0, 11800, false, true, 57.5f);
        }

        private void Update()
        {
            if (vehicle == null) return;

            phaseTime += Time.deltaTime;
            refreshTimer += Time.deltaTime;

            UpdateShiftLeds(vehicle.state.rpm);

            if (refreshTimer >= 0.04f) // 25 Hz crisp LCD update
            {
                refreshTimer = 0.0f;
                RedrawLcdTexture(
                    vehicle.state.gear,
                    Mathf.RoundToInt(vehicle.SpeedKmh),
                    Mathf.RoundToInt(vehicle.state.rpm),
                    vehicle.state.drsActive,
                    vehicle.state.drsAvailable,
                    vehicle.state.brakeBias * 100.0f
                );
            }
        }

        private void UpdateShiftLeds(float rpm)
        {
            if (ledMats == null) return;

            bool strobe = (rpm >= 13400.0f) && ((Mathf.FloorToInt(phaseTime * 24.0f) % 2) == 0);

            for (int i = 0; i < ledMats.Length; i++)
            {
                Material m = ledMats[i];
                if (m == null) continue;

                Color targetCol = LedOff;
                float emissionMul = 0.0f;

                if (rpm >= 13400.0f)
                {
                    // Rev Limiter: all 15 LEDs flash simultaneously in purple
                    targetCol = strobe ? LedPurple : LedOff;
                    emissionMul = strobe ? 4.5f : 0.0f;
                }
                else
                {
                    float thresh = 9600.0f + i * 250.0f;
                    if (rpm >= thresh)
                    {
                        // 15 LEDs in 4 distinct color bands: Green (0..3), Yellow (4..7), Red (8..11), Purple (12..14)
                        if (i < 4) targetCol = LedGreen;
                        else if (i < 8) targetCol = LedYellow;
                        else if (i < 12) targetCol = LedRed;
                        else targetCol = LedPurple;

                        emissionMul = 3.6f;
                    }
                }

                m.color = targetCol;
                if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", targetCol);
                if (m.HasProperty("_EmissionColor")) m.SetColor("_EmissionColor", targetCol * emissionMul);
            }
        }

        private void RedrawLcdTexture(int gear, int speedKmh, int rpm, bool drsAct, bool drsAvail, float bbPct)
        {
            Color32 bg = new Color32(10, 14, 22, 255);
            Color32 border = new Color32(42, 54, 72, 255);
            Color32 accentGold = new Color32(255, 208, 0, 255);
            Color32 white = new Color32(240, 245, 255, 255);
            Color32 cyan = new Color32(0, 225, 255, 255);
            Color32 green = new Color32(0, 235, 95, 255);
            Color32 red = new Color32(235, 35, 35, 255);
            Color32 panelDark = new Color32(18, 24, 36, 255);

            for (int i = 0; i < pixelBuffer.Length; i++)
                pixelBuffer[i] = bg;

            // Outer frame & telemetry grid lines
            DrawRect(2, 2, TexW - 4, TexH - 4, border, false);
            DrawRect(58, 12, 44, 62, panelDark, true);
            DrawRect(58, 12, 44, 62, accentGold, false);

            // Top RPM Progress Bar (y = 78..88)
            DrawRect(8, 78, TexW - 16, 10, panelDark, true);
            float rpmRatio = Mathf.Clamp01((rpm - 4000.0f) / 11000.0f);
            int fillW = Mathf.RoundToInt((TexW - 18) * rpmRatio);
            Color32 barCol = rpm > 12500 ? red : (rpm > 10500 ? accentGold : green);
            if (fillW > 0) DrawRect(9, 79, fillW, 8, barCol, true);

            // Center Giant Gear Numeral
            string gearChar = (gear == 0) ? "N" : (gear < 0 ? "R" : gear.ToString());
            DrawString(gearChar, 71, 24, 4, accentGold);

            // Left Column: DRS Status & Brake Bias
            Color32 drsBoxCol = drsAct ? green : (drsAvail ? accentGold : panelDark);
            DrawRect(8, 50, 44, 20, drsBoxCol, true);
            DrawString("DRS", 15, 55, 1, drsAct || drsAvail ? bg : white);

            string bbStr = $"{bbPct:F0}%";
            DrawString(bbStr, 12, 24, 2, cyan);

            // Right Column: Speed KM/H & ERS Battery Bar
            string spdStr = speedKmh.ToString().PadLeft(3, ' ');
            DrawString(spdStr.Trim(), 110, 44, 2, white);
            DrawString("KMH", 114, 28, 1, accentGold);
            DrawRect(108, 12, 44, 8, panelDark, true);
            DrawRect(108, 12, 38, 8, green, true); // ERS battery charge bar
            DrawString("ERS", 116, 13, 1, bg);

            // Bottom Center: RPM numerical
            DrawString(rpm.ToString(), 62, 3, 1, white);

            lcdTex.SetPixels32(pixelBuffer);
            lcdTex.Apply(false);
        }

        private void DrawRect(int rx, int ry, int rw, int rh, Color32 col, bool fill)
        {
            for (int y = ry; y < ry + rh; y++)
            {
                if (y < 0 || y >= TexH) continue;
                for (int x = rx; x < rx + rw; x++)
                {
                    if (x < 0 || x >= TexW) continue;
                    if (fill || x == rx || x == rx + rw - 1 || y == ry || y == ry + rh - 1)
                        pixelBuffer[y * TexW + x] = col;
                }
            }
        }

        private void DrawString(string text, int startX, int startY, int scale, Color32 col)
        {
            int curX = startX;
            foreach (char ch in text.ToUpperInvariant())
            {
                DrawChar(ch, curX, startY, scale, col);
                curX += (5 * scale) + scale;
            }
        }

        private void DrawChar(char c, int px, int py, int scale, Color32 col)
        {
            string pattern = null;
            string prefix = c + ":";
            for (int i = 0; i < GlyphMap.Length; i++)
            {
                if (GlyphMap[i].StartsWith(prefix))
                {
                    pattern = GlyphMap[i].Substring(2);
                    break;
                }
            }
            if (pattern == null) return;

            string[] rows = pattern.Split(',');
            for (int r = 0; r < rows.Length; r++)
            {
                int drawY = py + (rows.Length - 1 - r) * scale;
                string rowBits = rows[r];
                for (int bit = 0; bit < rowBits.Length; bit++)
                {
                    if (rowBits[bit] == '1')
                    {
                        int drawX = px + bit * scale;
                        DrawRect(drawX, drawY, scale, scale, col, true);
                    }
                }
            }
        }
    }
}
