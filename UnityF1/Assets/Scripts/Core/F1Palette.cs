using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace F1Sim.Core
{
    /// <summary>
    /// Art Bible & Procedural PBR Material/Texture Factory for Formula 1 Grand Prix Simulation.
    /// Style: "Golden-Hour Italian Grand Prix Realism: sleek satin carbon-fiber aerodynamics,
    /// high-contrast FIA livery accents, sun-drenched Monza parkland, warm late-afternoon light,
    /// procedural micro-roughness normal maps, and crisp F1 Broadcast telemetry UI."
    /// </summary>
    public static class F1Palette
    {
        // =========================================================================
        // 1. Art Bible Palette - Colors
        // =========================================================================
        // Track Surfaces
        public static readonly Color AsphaltBase      = new Color(0.13f, 0.14f, 0.16f, 1.0f); // #212429
        public static readonly Color AsphaltGroove    = new Color(0.07f, 0.08f, 0.10f, 1.0f); // #12141A
        public static readonly Color TrackLineWhite   = new Color(0.94f, 0.95f, 0.97f, 1.0f); // #F0F2F7
        public static readonly Color KerbRed          = new Color(0.88f, 0.05f, 0.05f, 1.0f); // #E00D0D
        public static readonly Color KerbWhite        = new Color(0.96f, 0.97f, 0.98f, 1.0f); // #F5F7FA
        public static readonly Color ParkGrassDark    = new Color(0.15f, 0.28f, 0.16f, 1.0f); // #264729
        public static readonly Color ParkGrassLight   = new Color(0.22f, 0.38f, 0.22f, 1.0f); // #386138
        public static readonly Color GravelSand       = new Color(0.78f, 0.68f, 0.52f, 1.0f); // #C7AE85
        public static readonly Color GravelShadow     = new Color(0.62f, 0.52f, 0.38f, 1.0f); // #9E8561

        // Car & Cockpit Materials
        public static readonly Color CarbonSatin      = new Color(0.09f, 0.10f, 0.12f, 1.0f); // #17191F
        public static readonly Color CarbonDark       = new Color(0.04f, 0.05f, 0.06f, 1.0f); // #0A0D0F
        public static readonly Color TitaniumHalo     = new Color(0.18f, 0.19f, 0.22f, 1.0f); // #2E3038
        public static readonly Color AlcantaraGrip    = new Color(0.07f, 0.07f, 0.08f, 1.0f); // #121214
        public static readonly Color TireRubber       = new Color(0.08f, 0.08f, 0.09f, 1.0f); // #141417
        public static readonly Color WheelRimGunmetal = new Color(0.24f, 0.26f, 0.29f, 1.0f); // #3D424A

        // Hero & Livery Colors
        public static readonly Color RacingRed        = new Color(0.90f, 0.02f, 0.00f, 1.0f); // #E60500 - Scuderia Corsa
        public static readonly Color ModenaYellow     = new Color(1.00f, 0.80f, 0.00f, 1.0f); // #FFCC00 - Giallo Modena
        public static readonly Color TelemetryCyan    = new Color(0.00f, 0.92f, 1.00f, 1.0f); // #00EBFF
        public static readonly Color DrsEmerald       = new Color(0.00f, 0.94f, 0.40f, 1.0f); // #00F066
        public static readonly Color ShiftBlue        = new Color(0.15f, 0.65f, 1.00f, 1.0f); // #26A6FF
        public static readonly Color PirelliSoftRed   = new Color(0.92f, 0.06f, 0.08f, 1.0f); // P-Zero Soft
        public static readonly Color PirelliMedYellow = new Color(1.00f, 0.82f, 0.00f, 1.0f); // P-Zero Medium

        // Trackside Props & Scenery
        public static readonly Color ArmcoSteel       = new Color(0.74f, 0.78f, 0.82f, 1.0f); // #BDC7D1
        public static readonly Color TecproBlue       = new Color(0.05f, 0.36f, 0.75f, 1.0f); // #0D5CBF
        public static readonly Color PineCanopyDark   = new Color(0.10f, 0.23f, 0.12f, 1.0f); // #1A3B1F
        public static readonly Color PineCanopyLight  = new Color(0.18f, 0.35f, 0.19f, 1.0f); // #2E5930
        public static readonly Color TreeBark         = new Color(0.30f, 0.20f, 0.14f, 1.0f); // #4D3324

        // Golden Hour Atmospheric Lighting (Warm radiant late-afternoon Italian Grand Prix)
        public static readonly Color SunGolden        = new Color(1.00f, 0.89f, 0.70f, 1.0f); // #FFE3B3 - warm, radiant Italian sunset
        public static readonly Color SkyZenith        = new Color(0.20f, 0.38f, 0.64f, 1.0f); // #3361A3 - deep dusk blue
        public static readonly Color SkyHorizon       = new Color(0.85f, 0.60f, 0.42f, 1.0f); // #D9996B - golden peach sunset haze
        public static readonly Color GroundBounce     = new Color(0.24f, 0.18f, 0.12f, 1.0f); // #3D2E1F - warm amber ground reflection

        private static readonly Dictionary<string, Material> materialCache = new Dictionary<string, Material>();
        private static readonly Dictionary<string, Texture2D> textureCache = new Dictionary<string, Texture2D>();

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

        // =========================================================================
        // 2. Procedural Normal Map Generators (Tangent Space with DXT5nm & RGB pack)
        // =========================================================================
        public static Texture2D CreateNormalMapFromHeight(
            int width,
            int height,
            System.Func<float, float, float> heightFunc,
            float strength = 2.0f,
            string name = "Tex_Normal")
        {
            if (textureCache.TryGetValue(name, out Texture2D cached) && cached != null)
                return cached;

            Texture2D normTex = new Texture2D(width, height, TextureFormat.RGBA32, true)
            {
                name = name,
                wrapMode = TextureWrapMode.Repeat,
                filterMode = FilterMode.Trilinear,
                anisoLevel = 8
            };

            Color[] pixels = new Color[width * height];
            float invW = 1.0f / width;
            float invH = 1.0f / height;

            for (int y = 0; y < height; y++)
            {
                int yPrev = (y - 1 + height) % height;
                int yNext = (y + 1) % height;
                float v = y * invH;

                for (int x = 0; x < width; x++)
                {
                    int xPrev = (x - 1 + width) % width;
                    int xNext = (x + 1) % width;
                    float u = x * invW;

                    float hL = heightFunc(xPrev * invW, v);
                    float hR = heightFunc(xNext * invW, v);
                    float hD = heightFunc(u, yPrev * invH);
                    float hU = heightFunc(u, yNext * invH);

                    float dX = (hR - hL) * strength;
                    float dY = (hU - hD) * strength;

                    Vector3 n = new Vector3(-dX, -dY, 1.0f).normalized;
                    // Pack into RGBA with both R and A carrying nx for DXT5nm & RGB unpack compatibility in URP Lit
                    float r = Mathf.Clamp01(n.x * 0.5f + 0.5f);
                    float g = Mathf.Clamp01(n.y * 0.5f + 0.5f);
                    float b = Mathf.Clamp01(n.z * 0.5f + 0.5f);
                    pixels[y * width + x] = new Color(r, g, b, r);
                }
            }
            normTex.SetPixels(pixels);
            normTex.Apply(true);
            textureCache[name] = normTex;
            return normTex;
        }

        /// <summary>
        /// Monza Asphalt Normal Map: aggregate micro-roughness + longitudinal rubbered grooves
        /// </summary>
        public static Texture2D CreateAsphaltNormalMap()
        {
            return CreateNormalMapFromHeight(512, 512, (u, v) =>
            {
                // Tight, high-frequency aggregate stone micro-grit (fine road gravel)
                float n1 = Mathf.PerlinNoise(u * 512.0f, v * 512.0f);
                float n2 = Mathf.PerlinNoise(u * 1024.0f + 13.7f, v * 1024.0f + 7.3f);
                float grit = (n1 * 0.60f + n2 * 0.40f);

                // Subtle directional rubber groove micro-lines
                float centerDist = Mathf.Abs(u - 0.5f) * 2.0f;
                float groove = Mathf.Sin(u * 64.0f * Mathf.PI) * 0.02f * Mathf.Exp(-centerDist * centerDist * 4.0f);

                return grit * 0.16f + groove;
            }, 1.2f, "Tex_MonzaAsphalt_Normal");
        }

        /// <summary>
        /// 2x2 Twill Carbon Fiber Weave Normal Map: authentic diagonal weave bands with cylindrical yarn curvature
        /// </summary>
        public static Texture2D CreateCarbonNormalMap()
        {
            return CreateNormalMapFromHeight(128, 128, (u, v) =>
            {
                float freq = 24.0f;
                float tx = (u * freq) % 4.0f;
                float ty = (v * freq) % 4.0f;

                // 2x2 twill alternating diagonal bands
                bool isHorizontal = ((tx + ty) % 4.0f < 2.0f);
                float threadProfile;
                if (isHorizontal)
                {
                    float localY = (ty % 1.0f);
                    threadProfile = Mathf.Sin(localY * Mathf.PI);
                }
                else
                {
                    float localX = (tx % 1.0f);
                    threadProfile = Mathf.Sin(localX * Mathf.PI);
                }
                return threadProfile * 0.5f;
            }, 2.8f, "Tex_CarbonWeave_Normal");
        }

        /// <summary>
        /// Pirelli Tire Normal Map: slick rubber grain, circumferential tread lines & sidewall bead ridge
        /// </summary>
        public static Texture2D CreateTireNormalMap()
        {
            return CreateNormalMapFromHeight(256, 256, (u, v) =>
            {
                // u = tread width across tire, v = circumference
                float beadL = Mathf.SmoothStep(0.06f, 0.12f, u);
                float beadR = Mathf.SmoothStep(0.94f, 0.88f, u);
                float shoulder = beadL * beadR;

                // Micro-grain rubber roughness
                float rubberNoise = Mathf.PerlinNoise(u * 120.0f, v * 120.0f) * 0.25f;

                // Circumferential grain lines
                float grain = Mathf.Sin(u * 32.0f * Mathf.PI) * 0.04f * shoulder;

                return shoulder * 0.6f + rubberNoise + grain;
            }, 2.4f, "Tex_PirelliTire_Normal");
        }

        /// <summary>
        /// 3D FIA Kerb Normal Map: sharp saw-tooth rumble serration + concrete porosity
        /// </summary>
        public static Texture2D CreateKerbNormalMap()
        {
            return CreateNormalMapFromHeight(128, 256, (u, v) =>
            {
                // v = direction along track, u = lateral width
                float tooth = (v * 16.0f) % 1.0f;
                float serration = Mathf.Pow(tooth, 0.65f); // saw-tooth rumble bevel
                float concreteGrain = Mathf.PerlinNoise(u * 64.0f, v * 64.0f) * 0.18f;
                float bevelEdge = Mathf.SmoothStep(0.0f, 0.15f, u) * Mathf.SmoothStep(1.0f, 0.85f, u);

                return (serration * 0.85f + concreteGrain) * bevelEdge;
            }, 4.2f, "Tex_Kerb_Normal");
        }

        /// <summary>
        /// Galvanized Corrugated Armco Barrier Normal Map: horizontal wavy corrugation rails
        /// </summary>
        public static Texture2D CreateArmcoNormalMap()
        {
            return CreateNormalMapFromHeight(64, 128, (u, v) =>
            {
                // v = vertical height of barrier rail (corrugation bumps)
                float corrugation = Mathf.Sin(v * Mathf.PI * 6.0f);
                float metalScratch = Mathf.PerlinNoise(u * 128.0f, v * 16.0f) * 0.08f;
                return corrugation * 0.5f + metalScratch;
            }, 3.0f, "Tex_ArmcoCorrugated_Normal");
        }

        // =========================================================================
        // 3. PBR Material Factory with Normal Maps, Smoothness, Metallic & Clearcoat
        // =========================================================================
        public static Material GetOrCreateMaterial(
            string key,
            Color baseColor,
            float metallic = 0.2f,
            float smoothness = 0.5f,
            Texture2D mainTex = null,
            Color? emissionColor = null,
            bool transparent = false,
            Texture2D bumpMap = null,
            float bumpScale = 1.0f,
            float clearcoat = 0.0f)
        {
            if (materialCache.TryGetValue(key, out Material cached) && cached != null)
                return cached;

            Material templateMat = null;
            if (transparent)
                templateMat = Resources.Load<Material>("F1_TransparentLit");
            else if (mainTex != null || emissionColor.HasValue || bumpMap != null)
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

            // Normal Map Setup
            if (bumpMap != null)
            {
                mat.EnableKeyword("_NORMALMAP");
                if (mat.HasProperty("_BumpMap")) mat.SetTexture("_BumpMap", bumpMap);
                if (mat.HasProperty("_BumpScale")) mat.SetFloat("_BumpScale", bumpScale);
            }
            else
            {
                mat.DisableKeyword("_NORMALMAP");
                if (mat.HasProperty("_BumpMap")) mat.SetTexture("_BumpMap", null);
            }

            // Clearcoat (Automotive finish)
            if (clearcoat > 0.0f)
            {
                mat.EnableKeyword("_CLEARCOAT");
                if (mat.HasProperty("_ClearCoatMask")) mat.SetFloat("_ClearCoatMask", clearcoat);
                if (mat.HasProperty("_ClearCoatSmoothness")) mat.SetFloat("_ClearCoatSmoothness", 0.92f);
            }

            // Emissive Setup
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

            // Transparent Setup
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

        // =========================================================================
        // 4. Track & Environment PBR Materials
        // =========================================================================
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

                    // Perlin aggregate gravel grain
                    float n1 = Mathf.PerlinNoise(u * 64.0f, v * 64.0f);
                    float n2 = Mathf.PerlinNoise(u * 220.0f + 13.7f, v * 220.0f + 7.3f);
                    float grain = (n1 * 0.6f + n2 * 0.4f - 0.5f) * 0.05f;

                    // Darker rubbered-in racing groove near center
                    float centerDist = Mathf.Abs(u - 0.5f) * 2.0f;
                    float grooveFactor = Mathf.Exp(-centerDist * centerDist * 4.5f);
                    Color c = Color.Lerp(AsphaltBase, AsphaltGroove, grooveFactor * 0.72f);
                    c.r = Mathf.Clamp01(c.r + grain);
                    c.g = Mathf.Clamp01(c.g + grain);
                    c.b = Mathf.Clamp01(c.b + grain);

                    // Crisp white track limit sidelines
                    if ((u >= 0.025f && u <= 0.048f) || (u >= 0.952f && u <= 0.975f))
                    {
                        c = Color.Lerp(c, TrackLineWhite, 0.94f);
                    }

                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Texture2D normalMap = CreateAsphaltNormalMap();
            Material mat = GetOrCreateMaterial("Mat_MonzaAsphalt", Color.white, 0.08f, 0.42f, tex, null, false, normalMap, 0.45f);
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
                float serrationShadow = Mathf.Lerp(0.80f, 1.0f, (v * 2.0f) % 1.0f);

                for (int x = 0; x < w; x++)
                {
                    float u = x / (float)w;
                    Color baseCol = isRedBlock ? KerbRed : KerbWhite;
                    // Tire skid mark darkening on inner edge where F1 cars clip the apex
                    float skid = Mathf.SmoothStep(0.85f, 1.0f, 1.0f - Mathf.Abs(u - 0.35f));
                    Color c = baseCol * serrationShadow * Mathf.Lerp(1.0f, 0.85f, skid * 0.45f);
                    c.a = 1.0f;
                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Texture2D normalMap = CreateKerbNormalMap();
            Material kMat = GetOrCreateMaterial("Mat_FiaKerb", Color.white, 0.05f, 0.45f, tex, null, false, normalMap, 1.35f);
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
                    // Mowed parkland stripes + organic patchiness
                    float stripe = Mathf.Sin(u * Mathf.PI * 8.0f) * 0.5f + 0.5f;
                    float noise = Mathf.PerlinNoise(u * 24.0f, v * 24.0f);
                    Color c = Color.Lerp(ParkGrassDark, ParkGrassLight, stripe * 0.55f + noise * 0.45f);
                    pixels[y * w + x] = c;
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_ParkGrass", Color.white, 0.0f, 0.18f, tex);
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
                    float r = Mathf.Sin(x * 0.8f) * 0.05f; // raked gravel grooves
                    pixels[y * w + x] = Color.Lerp(GravelShadow, GravelSand, Mathf.Clamp01(n + r));
                }
            }
            tex.SetPixels(pixels);
            tex.Apply(true);

            Material mat = GetOrCreateMaterial("Mat_GravelRunoff", Color.white, 0.0f, 0.15f, tex);
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

            Texture2D normalMap = CreateCarbonNormalMap();
            Material mat = GetOrCreateMaterial("Mat_CarbonWeave", Color.white, 0.45f, 0.85f, tex, null, false, normalMap, 1.2f, clearcoat: 1.0f);
            mat.mainTextureScale = new Vector2(12.0f, 12.0f);
            if (mat.HasProperty("_BaseMap")) mat.SetTextureScale("_BaseMap", new Vector2(12.0f, 12.0f));
            if (mat.HasProperty("_BumpMap")) mat.SetTextureScale("_BumpMap", new Vector2(12.0f, 12.0f));
            return mat;
        }

        public static Material CreatePirelliTireMaterial(bool isSoft = true)
        {
            string key = isSoft ? "Mat_PirelliSoft" : "Mat_PirelliMedium";
            if (materialCache.TryGetValue(key, out Material cached) && cached != null)
                return cached;

            Texture2D normalMap = CreateTireNormalMap();
            Material mat = GetOrCreateMaterial(key, TireRubber, 0.08f, 0.32f, null, null, false, normalMap, 1.25f);
            return mat;
        }

        public static Material CreateArmcoMaterial()
        {
            if (materialCache.TryGetValue("Mat_ArmcoCorrugatedSteel", out Material cached) && cached != null)
                return cached;

            Texture2D normalMap = CreateArmcoNormalMap();
            Material mat = GetOrCreateMaterial("Mat_ArmcoCorrugatedSteel", ArmcoSteel, 0.85f, 0.65f, null, null, false, normalMap, 1.5f);
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
