using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace F1Sim.Core
{
    /// <summary>
    /// Art Bible & Procedural PBR Material/Texture Factory for Formula 1 Grand Prix Simulation.
    /// Style: "Golden-Hour Italian Grand Prix Realism: sleek satin carbon-fiber aerodynamics,
    /// high-contrast FIA livery accents, sun-drenched Monza parkland, warm late-afternoon light,
    /// and crisp F1 Broadcast telemetry UI."
    /// </summary>
    public static class F1Palette
    {
        // --- 1. Art Bible Palette ---
        public static readonly Color AsphaltBase      = new Color(0.14f, 0.15f, 0.17f, 1.0f); // #24262B
        public static readonly Color AsphaltGroove    = new Color(0.09f, 0.10f, 0.12f, 1.0f); // #17191E
        public static readonly Color TrackLineWhite   = new Color(0.92f, 0.93f, 0.95f, 1.0f); // #EBEFF2
        public static readonly Color KerbRed          = new Color(0.86f, 0.06f, 0.06f, 1.0f); // #DC0F0F
        public static readonly Color KerbWhite        = new Color(0.94f, 0.95f, 0.96f, 1.0f); // #F0F2F5
        public static readonly Color ParkGrassDark    = new Color(0.16f, 0.30f, 0.17f, 1.0f); // #294D2B
        public static readonly Color ParkGrassLight   = new Color(0.20f, 0.36f, 0.21f, 1.0f); // #335C36
        public static readonly Color GravelSand       = new Color(0.76f, 0.67f, 0.53f, 1.0f); // #C2AB87
        public static readonly Color GravelShadow     = new Color(0.64f, 0.55f, 0.42f, 1.0f); // #A38C6B

        // Car & Cockpit Materials
        public static readonly Color CarbonSatin      = new Color(0.09f, 0.10f, 0.12f, 1.0f); // #17191E
        public static readonly Color CarbonDark       = new Color(0.05f, 0.06f, 0.07f, 1.0f); // #0D0F12
        public static readonly Color TitaniumHalo     = new Color(0.15f, 0.16f, 0.19f, 1.0f); // #262930
        public static readonly Color AlcantaraGrip    = new Color(0.07f, 0.07f, 0.08f, 1.0f); // #121214
        public static readonly Color TireRubber       = new Color(0.08f, 0.08f, 0.09f, 1.0f); // #141417
        public static readonly Color WheelRimGunmetal = new Color(0.22f, 0.24f, 0.27f, 1.0f); // #383D45

        // Hero & Livery Colors
        public static readonly Color RacingRed        = new Color(0.88f, 0.02f, 0.00f, 1.0f); // #E10600
        public static readonly Color ModenaYellow     = new Color(1.00f, 0.78f, 0.00f, 1.0f); // #FFC700
        public static readonly Color TelemetryCyan    = new Color(0.00f, 0.90f, 1.00f, 1.0f); // #00E5FF
        public static readonly Color DrsEmerald       = new Color(0.00f, 0.92f, 0.38f, 1.0f); // #00EB61
        public static readonly Color ShiftBlue        = new Color(0.15f, 0.62f, 1.00f, 1.0f); // #269EFF

        // Trackside Props & Trees
        public static readonly Color ArmcoSteel       = new Color(0.72f, 0.76f, 0.80f, 1.0f); // #B8C2CC
        public static readonly Color TecproBlue       = new Color(0.05f, 0.35f, 0.72f, 1.0f); // #0D59B8
        public static readonly Color PineCanopyDark   = new Color(0.11f, 0.24f, 0.13f, 1.0f); // #1C3D21
        public static readonly Color PineCanopyLight  = new Color(0.17f, 0.33f, 0.18f, 1.0f); // #2B542E
        public static readonly Color TreeBark         = new Color(0.32f, 0.22f, 0.15f, 1.0f); // #523826

        // Lighting & Atmosphere
        public static readonly Color SunGolden        = new Color(1.00f, 0.94f, 0.83f, 1.0f); // #FFF0D4
        public static readonly Color SkyZenith        = new Color(0.22f, 0.42f, 0.68f, 1.0f); // #386BAD
        public static readonly Color SkyHorizon       = new Color(0.68f, 0.78f, 0.88f, 1.0f); // #ADC7E0
        public static readonly Color GroundBounce     = new Color(0.20f, 0.18f, 0.15f, 1.0f); // #332E26

        private static readonly Dictionary<string, Material> materialCache = new Dictionary<string, Material>();

        public static Shader GetLitShader()
        {
            Material resMat = Resources.Load<Material>("F1_BaseLit");
            if (resMat != null && resMat.shader != null)
                return resMat.shader;

            Shader s = Shader.Find("Universal Render Pipeline/Lit");
            if (s == null) s = Shader.Find("Standard");
            if (s == null) s = Shader.Find("Diffuse");
            return s;
        }

        public static Shader GetUnlitShader()
        {
            Shader s = Shader.Find("Universal Render Pipeline/Unlit");
            if (s == null) s = Shader.Find("Unlit/Color");
            if (s == null) s = GetLitShader();
            return s;
        }

        public static Material GetOrCreateMaterial(
            string key,
            Color baseColor,
            float metallic = 0.2f,
            float smoothness = 0.5f,
            Texture2D mainTex = null,
            Color? emissionColor = null,
            bool transparent = false)
        {
            if (materialCache.TryGetValue(key, out Material cached) && cached != null)
                return cached;

            Material templateMat = null;
            if (transparent)
                templateMat = Resources.Load<Material>("F1_TransparentLit");
            else if (mainTex != null || emissionColor.HasValue)
                templateMat = Resources.Load<Material>("F1_TexturedLit");
            else
                templateMat = Resources.Load<Material>("F1_BaseLit");

            Material mat = (templateMat != null)
                ? new Material(templateMat) { name = key }
                : new Material(GetLitShader()) { name = key };

            mat.color = baseColor;
            if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", baseColor);
            if (mat.HasProperty("_Color")) mat.SetColor("_Color", baseColor);
            if (mat.HasProperty("_Metallic")) mat.SetFloat("_Metallic", metallic);
            if (mat.HasProperty("_Smoothness")) mat.SetFloat("_Smoothness", smoothness);
            if (mat.HasProperty("_Glossiness")) mat.SetFloat("_Glossiness", smoothness);

            if (mainTex != null)
            {
                mat.mainTexture = mainTex;
                if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", mainTex);
                if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", mainTex);
            }
            else
            {
                mat.mainTexture = null;
                if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", null);
                if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", null);
            }

            if (emissionColor.HasValue)
            {
                mat.EnableKeyword("_EMISSION");
                if (mat.HasProperty("_EmissionColor"))
                    mat.SetColor("_EmissionColor", emissionColor.Value);
            }
            else
            {
                mat.DisableKeyword("_EMISSION");
                if (mat.HasProperty("_EmissionColor"))
                    mat.SetColor("_EmissionColor", Color.black);
            }

            if (transparent)
            {
                if (mat.HasProperty("_Surface")) mat.SetFloat("_Surface", 1.0f);
                if (mat.HasProperty("_Blend")) mat.SetFloat("_Blend", 0.0f);
                if (mat.HasProperty("_SrcBlend")) mat.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
                if (mat.HasProperty("_DstBlend")) mat.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
                if (mat.HasProperty("_ZWrite")) mat.SetInt("_ZWrite", 0);
                mat.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                mat.renderQueue = (int)RenderQueue.Transparent;
            }

            materialCache[key] = mat;
            return mat;
        }

        public static Material CreateAsphaltMaterial()
        {
            if (materialCache.TryGetValue("Mat_MonzaAsphalt", out Material cached) && cached != null)
                return cached;

            int w = 512, h = 512;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_MonzaAsphalt",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear,
                anisoLevel = 8
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                float v = y / (float)h;
                for (int x = 0; x < w; x++)
                {
                    float u = x / (float)w;

                    // Subtle Perlin aggregate grain
                    float n1 = Mathf.PerlinNoise(u * 64.0f, v * 64.0f);
                    float n2 = Mathf.PerlinNoise(u * 220.0f + 13.7f, v * 220.0f + 7.3f);
                    float grain = (n1 * 0.6f + n2 * 0.4f - 0.5f) * 0.045f;

                    // Darker rubbered-in racing line near center
                    float centerDist = Mathf.Abs(u - 0.5f) * 2.0f;
                    float grooveFactor = Mathf.Exp(-centerDist * centerDist * 4.5f);
                    Color c = Color.Lerp(AsphaltBase, AsphaltGroove, grooveFactor * 0.65f);
                    c.r = Mathf.Clamp01(c.r + grain);
                    c.g = Mathf.Clamp01(c.g + grain);
                    c.b = Mathf.Clamp01(c.b + grain);

                    // Crisp white track limit sidelines
                    if ((u >= 0.025f && u <= 0.048f) || (u >= 0.952f && u <= 0.975f))
                    {
                        c = Color.Lerp(c, TrackLineWhite, 0.92f);
                    }

                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_MonzaAsphalt", Color.white, 0.08f, 0.32f, tex);
            if (mat.HasProperty("_Cull")) mat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Geometry;
            if (mat.HasProperty("_Surface")) mat.SetFloat("_Surface", 0.0f);
            if (mat.HasProperty("_ZWrite")) mat.SetFloat("_ZWrite", 1.0f);
            return mat;
        }

        public static Material CreateKerbMaterial()
        {
            if (materialCache.TryGetValue("Mat_FiaKerb", out Material cached) && cached != null)
                return cached;

            int w = 128, h = 256;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_FiaKerb",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear,
                anisoLevel = 8
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                float v = y / (float)h;
                bool isRedBlock = (v < 0.5f);
                float serrationShadow = Mathf.Lerp(0.82f, 1.0f, (v * 2.0f) % 1.0f);

                for (int x = 0; x < w; x++)
                {
                    float u = x / (float)w;
                    Color baseCol = isRedBlock ? KerbRed : KerbWhite;
                    // Tire skid mark darkening on inner edge
                    float skid = Mathf.SmoothStep(0.85f, 1.0f, 1.0f - Mathf.Abs(u - 0.35f));
                    Color c = baseCol * serrationShadow * Mathf.Lerp(1.0f, 0.88f, skid * 0.4f);
                    c.a = 1.0f;
                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material kMat = GetOrCreateMaterial("Mat_FiaKerb", Color.white, 0.05f, 0.38f, tex);
            if (kMat.HasProperty("_Cull")) kMat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            kMat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Geometry;
            if (kMat.HasProperty("_Surface")) kMat.SetFloat("_Surface", 0.0f);
            if (kMat.HasProperty("_ZWrite")) kMat.SetFloat("_ZWrite", 1.0f);
            return kMat;
        }

        public static Material CreateGrassMaterial()
        {
            if (materialCache.TryGetValue("Mat_ParkGrass", out Material cached) && cached != null)
                return cached;

            int w = 256, h = 256;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_ParkGrass",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear,
                anisoLevel = 4
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                float v = y / (float)h;
                for (int x = 0; x < w; x++)
                {
                    float u = x / (float)w;
                    // Mowed stadium stripes + organic patchiness
                    float stripe = Mathf.Sin(u * Mathf.PI * 8.0f) * 0.5f + 0.5f;
                    float noise = Mathf.PerlinNoise(u * 24.0f, v * 24.0f);
                    Color c = Color.Lerp(ParkGrassDark, ParkGrassLight, stripe * 0.55f + noise * 0.45f);
                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_ParkGrass", Color.white, 0.0f, 0.15f, tex);
            mat.mainTextureScale = new Vector2(80.0f, 80.0f);
            if (mat.HasProperty("_BaseMap")) mat.SetTextureScale("_BaseMap", new Vector2(80.0f, 80.0f));
            return mat;
        }

        public static Material CreateGravelMaterial()
        {
            if (materialCache.TryGetValue("Mat_GravelRunoff", out Material cached) && cached != null)
                return cached;

            int w = 128, h = 128;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_GravelRunoff",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    float n = Mathf.PerlinNoise(x * 0.35f, y * 0.35f);
                    float r = Mathf.Sin(x * 0.8f) * 0.04f; // raked gravel grooves
                    pixels[y * w + x] = Color.Lerp(GravelShadow, GravelSand, Mathf.Clamp01(n + r));
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_GravelRunoff", Color.white, 0.0f, 0.12f, tex);
            mat.mainTextureScale = new Vector2(8.0f, 16.0f);
            if (mat.HasProperty("_BaseMap")) mat.SetTextureScale("_BaseMap", new Vector2(8.0f, 16.0f));
            return mat;
        }

        public static Material CreateCarbonMaterial()
        {
            if (materialCache.TryGetValue("Mat_CarbonWeave", out Material cached) && cached != null)
                return cached;

            int w = 64, h = 64;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_CarbonWeave",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    int pattern = ((x / 2) + (y / 2)) % 2;
                    pixels[y * w + x] = (pattern == 0) ? CarbonSatin : CarbonDark;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_CarbonWeave", Color.white, 0.65f, 0.72f, tex);
            mat.mainTextureScale = new Vector2(12.0f, 12.0f);
            if (mat.HasProperty("_BaseMap")) mat.SetTextureScale("_BaseMap", new Vector2(12.0f, 12.0f));
            return mat;
        }

        public static Material CreateGrandstandCrowdMaterial()
        {
            if (materialCache.TryGetValue("Mat_GrandstandCrowd", out Material cached) && cached != null)
                return cached;

            int w = 128, h = 128;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, true)
            {
                name = "Tex_GrandstandCrowd",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Point
            };

            Color[] crowdPalette = new Color[]
            {
                RacingRed, RacingRed, ModenaYellow, Color.white,
                new Color(0.12f, 0.25f, 0.55f), new Color(1.0f, 0.50f, 0.0f), new Color(0.18f, 0.20f, 0.24f)
            };

            System.Random rng = new System.Random(1922);
            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                bool isAisleRow = (y % 8 == 0);
                for (int x = 0; x < w; x++)
                {
                    if (isAisleRow || (x % 16 == 0))
                    {
                        pixels[y * w + x] = new Color(0.22f, 0.24f, 0.28f, 1.0f);
                    }
                    else
                    {
                        Color fan = crowdPalette[rng.Next(crowdPalette.Length)];
                        pixels[y * w + x] = fan;
                    }
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            return GetOrCreateMaterial("Mat_GrandstandCrowd", Color.white, 0.1f, 0.25f, tex);
        }

        public static Material CreateCheckeredMaterial()
        {
            if (materialCache.TryGetValue("Mat_Checkered", out Material cached) && cached != null)
                return cached;

            int w = 128, h = 32;
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, false)
            {
                name = "Tex_Checkered",
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Point
            };

            Color[] pixels = new Color[w * h];
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    bool white = (((x / 8) + (y / 8)) % 2) == 0;
                    pixels[y * w + x] = white ? TrackLineWhite : AsphaltGroove;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(false);

            return GetOrCreateMaterial("Mat_Checkered", Color.white, 0.1f, 0.4f, tex);
        }
    }
}
