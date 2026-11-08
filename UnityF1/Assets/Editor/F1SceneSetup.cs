using System.IO;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using F1Sim.Core;
using F1Sim.Cockpit;

namespace F1Sim.Editor
{
    [InitializeOnLoad]
    public static class F1SceneSetup
    {
        private const string ScenePath = "Assets/Scenes/MainCircuit.unity";

        static F1SceneSetup()
        {
            EditorApplication.delayCall += EnsureSceneExists;
        }

        [MenuItem("F1 Simulation/Setup & Build Main Scene")]
        public static void SetupScene()
        {
            Debug.Log("[F1 Sim] Configuring URP & Setting up Monza Circuit Main Scene...");

            EnsureUrpPipelineConfigured();

            if (!Directory.Exists("Assets/Scenes"))
            {
                Directory.CreateDirectory("Assets/Scenes");
            }

            Scene newScene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            // 1. Configure Trilight Ambient & Fog in Scene RenderSettings
            RenderSettings.ambientMode = AmbientMode.Trilight;
            RenderSettings.ambientSkyColor = F1Palette.SkyZenith;
            RenderSettings.ambientEquatorColor = F1Palette.SkyHorizon;
            RenderSettings.ambientGroundColor = F1Palette.GroundBounce;
            RenderSettings.fog = true;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogColor = F1Palette.SkyHorizon;
            RenderSettings.fogDensity = 0.00038f;

            // 2. Create Main Camera with URP Post-Processing enabled
            GameObject camObj = new GameObject("Main Camera");
            Camera cam = camObj.AddComponent<Camera>();
            camObj.AddComponent<AudioListener>();
            camObj.tag = "MainCamera";
            cam.clearFlags = CameraClearFlags.Skybox;
            cam.backgroundColor = F1Palette.SkyHorizon;
            cam.fieldOfView = 68.0f;
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 3500.0f;

            UniversalAdditionalCameraData camData = camObj.AddComponent<UniversalAdditionalCameraData>();
            camData.renderPostProcessing = true;
            camData.antialiasing = AntialiasingMode.SubpixelMorphologicalAntiAliasing;
            camData.antialiasingQuality = AntialiasingQuality.High;

            // 3. Create Golden-Hour Directional Key Light
            GameObject lightObj = new GameObject("Sun Directional Light");
            Light sun = lightObj.AddComponent<Light>();
            sun.type = LightType.Directional;
            sun.color = F1Palette.SunGolden;
            sun.intensity = 1.35f;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 0.82f;
            lightObj.transform.rotation = Quaternion.Euler(38.0f, -35.0f, 0.0f);

            // 4. Create F1 Game Manager Root
            GameObject managerObj = new GameObject("F1_Game_Manager");
            managerObj.AddComponent<F1GameManager>();

            // 5. Create URP Resource Materials & In-Scene Variant Anchors so BuildPlayer includes all URP Lit shader variants
            EnsureResourceMaterialsAndAnchors(managerObj.transform);

            // Save Scene
            EditorSceneManager.SaveScene(newScene, ScenePath);
            Debug.Log($"[F1 Sim] Scene saved successfully to {ScenePath}");

            // Register in Build Settings
            EditorBuildSettingsScene[] buildScenes = new EditorBuildSettingsScene[]
            {
                new EditorBuildSettingsScene(ScenePath, true)
            };
            EditorBuildSettings.scenes = buildScenes;
            AssetDatabase.SaveAssets();
        }

        private static void EnsureResourceMaterialsAndAnchors(Transform parent)
        {
            if (!Directory.Exists("Assets/Resources"))
            {
                Directory.CreateDirectory("Assets/Resources");
            }

            Shader litShader = Shader.Find("Universal Render Pipeline/Lit");
            if (litShader == null) litShader = Shader.Find("Standard");

            Texture2D refTex = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Resources/F1_RefTex.asset");
            if (refTex == null)
            {
                refTex = new Texture2D(4, 4, TextureFormat.RGBA32, false) { name = "F1_RefTex" };
                Color[] cols = new Color[16];
                for (int i = 0; i < 16; i++) cols[i] = Color.white;
                refTex.SetPixels(cols);
                refTex.Apply();
                AssetDatabase.CreateAsset(refTex, "Assets/Resources/F1_RefTex.asset");
            }

            Material baseLit = AssetDatabase.LoadAssetAtPath<Material>("Assets/Resources/F1_BaseLit.mat");
            if (baseLit == null)
            {
                baseLit = new Material(litShader) { name = "F1_BaseLit" };
                AssetDatabase.CreateAsset(baseLit, "Assets/Resources/F1_BaseLit.mat");
            }

            Material texLit = AssetDatabase.LoadAssetAtPath<Material>("Assets/Resources/F1_TexturedLit.mat");
            if (texLit == null)
            {
                texLit = new Material(litShader) { name = "F1_TexturedLit" };
                texLit.mainTexture = refTex;
                if (texLit.HasProperty("_BaseMap")) texLit.SetTexture("_BaseMap", refTex);
                texLit.EnableKeyword("_EMISSION");
                if (texLit.HasProperty("_EmissionMap")) texLit.SetTexture("_EmissionMap", refTex);
                if (texLit.HasProperty("_EmissionColor")) texLit.SetColor("_EmissionColor", Color.white);
                AssetDatabase.CreateAsset(texLit, "Assets/Resources/F1_TexturedLit.mat");
            }

            Material transLit = AssetDatabase.LoadAssetAtPath<Material>("Assets/Resources/F1_TransparentLit.mat");
            if (transLit == null)
            {
                transLit = new Material(litShader) { name = "F1_TransparentLit" };
                if (transLit.HasProperty("_Surface")) transLit.SetFloat("_Surface", 1.0f);
                if (transLit.HasProperty("_SrcBlend")) transLit.SetInt("_SrcBlend", (int)BlendMode.SrcAlpha);
                if (transLit.HasProperty("_DstBlend")) transLit.SetInt("_DstBlend", (int)BlendMode.OneMinusSrcAlpha);
                if (transLit.HasProperty("_ZWrite")) transLit.SetInt("_ZWrite", 0);
                transLit.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
                transLit.renderQueue = (int)RenderQueue.Transparent;
                AssetDatabase.CreateAsset(transLit, "Assets/Resources/F1_TransparentLit.mat");
            }

            AssetDatabase.SaveAssets();

            // Place tiny out-of-sight anchor meshes inside the scene so Unity's URP Shader Preprocessor
            // compiles all directional light, shadow, fog, texture, emission & transparency variants into the Player build
            GameObject anchorRoot = new GameObject("URP_Shader_Variant_Anchors");
            anchorRoot.transform.SetParent(parent, false);
            anchorRoot.transform.position = new Vector3(0.0f, -200.0f, 0.0f);

            Material[] mats = new Material[] { baseLit, texLit, transLit };
            for (int i = 0; i < mats.Length; i++)
            {
                GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
                quad.name = $"ShaderAnchor_{i}";
                quad.transform.SetParent(anchorRoot.transform, false);
                quad.transform.localScale = new Vector3(0.01f, 0.01f, 0.01f);
                quad.GetComponent<Renderer>().sharedMaterial = mats[i];
            }
        }

        private static void EnsureUrpPipelineConfigured()
        {
            if (GraphicsSettings.defaultRenderPipeline != null)
            {
                return;
            }

            string[] guids = AssetDatabase.FindAssets("t:UniversalRenderPipelineAsset");
            if (guids != null && guids.Length > 0)
            {
                string path = AssetDatabase.GUIDToAssetPath(guids[0]);
                UniversalRenderPipelineAsset existingAsset = AssetDatabase.LoadAssetAtPath<UniversalRenderPipelineAsset>(path);
                if (existingAsset != null)
                {
                    existingAsset.supportsHDR = true;
                    existingAsset.msaaSampleCount = 4;
                    existingAsset.shadowDistance = 220.0f;
                    GraphicsSettings.defaultRenderPipeline = existingAsset;
                    QualitySettings.renderPipeline = existingAsset;
                    EditorUtility.SetDirty(existingAsset);
                    return;
                }
            }

            if (!Directory.Exists("Assets/Settings"))
            {
                Directory.CreateDirectory("Assets/Settings");
            }

            UniversalRendererData rendererData = ScriptableObject.CreateInstance<UniversalRendererData>();
            AssetDatabase.CreateAsset(rendererData, "Assets/Settings/F1_URP_Renderer.asset");

            UniversalRenderPipelineAsset urpAsset = UniversalRenderPipelineAsset.Create(rendererData);
            urpAsset.supportsHDR = true;
            urpAsset.msaaSampleCount = 4;
            urpAsset.shadowDistance = 220.0f;
            AssetDatabase.CreateAsset(urpAsset, "Assets/Settings/F1_URP_Asset.asset");

            GraphicsSettings.defaultRenderPipeline = urpAsset;
            QualitySettings.renderPipeline = urpAsset;
            AssetDatabase.SaveAssets();
            Debug.Log("[F1 Sim] Created and assigned F1_URP_Asset with HDR, 4x MSAA, and 220m shadow distance.");
        }

        [MenuItem("F1 Simulation/Verify, Capture Previews & Build Executable")]
        public static void VerifyCaptureAndBuild()
        {
            SetupScene();

            // 1. Instantiate full 3D world in scene for high-res RenderTexture verification
            F1GameManager gm = Object.FindFirstObjectByType<F1GameManager>();
            if (gm != null)
            {
                gm.InitializeForEditorPreview();
                Camera cam = Camera.main;
                if (cam != null && gm.playerVehicle != null)
                {
                    Transform carTf = gm.playerVehicle.transform;
                    string rootOut = @"c:\Users\nguye\PROJECT\F1";

                    // Shot A: 3D Showcase Orbit View (Car + Grid + Gantry + Grandstands + Alps)
                    Vector3 focus = carTf.position + Vector3.up * 0.65f + carTf.forward * 1.2f;
                    cam.transform.position = focus - carTf.forward * 5.2f + carTf.right * 3.4f + Vector3.up * 1.65f;
                    cam.transform.LookAt(focus + carTf.forward * 4.0f, Vector3.up);
                    cam.fieldOfView = 50.0f;
                    CaptureCameraToPng(cam, Path.Combine(rootOut, "Preview_3D_Showcase.png"), 1920, 1080);

                    // Shot B: First-Person Cockpit Driver Eye View (Steering Wheel + LCD + Halo + Mirrors + Rivals)
                    Transform helmetTr = carTf.Find("F1_Chassis_Visual/Driver_Helmet_Assembly");
                    if (helmetTr != null) helmetTr.gameObject.SetActive(false);
                    cam.transform.position = carTf.TransformPoint(new Vector3(0.0f, 0.72f, -0.28f));
                    cam.transform.rotation = carTf.rotation * Quaternion.Euler(4.5f, 0.0f, 0.0f);
                    cam.fieldOfView = 80.0f;
                    CaptureCameraToPng(cam, Path.Combine(rootOut, "Preview_3D_Cockpit.png"), 1920, 1080);

                    // Shot B2: Driver Arms & Animated Steering Wheel Cockpit View
                    cam.transform.position = carTf.TransformPoint(new Vector3(0.0f, 0.68f, -0.24f));
                    cam.transform.rotation = carTf.rotation * Quaternion.Euler(8.0f, 0.0f, 0.0f);
                    cam.fieldOfView = 76.0f;
                    CaptureCameraToPng(cam, Path.Combine(rootOut, "Preview_3D_Cockpit_Arms.png"), 1920, 1080);

                    // Shot C: External Chase / Broadcast Grid View
                    cam.transform.position = carTf.TransformPoint(new Vector3(0.0f, 2.05f, -5.4f));
                    cam.transform.LookAt(carTf.position + carTf.forward * 16.0f + Vector3.up * 0.7f, Vector3.up);
                    cam.fieldOfView = 60.0f;
                    CaptureCameraToPng(cam, Path.Combine(rootOut, "Preview_3D_Chase.png"), 1920, 1080);

                    // Shot D: Grand Monza Circuit Aerial Overview (5,793m Autodromo Nazionale Layout + Royal Parkland + Alps)
                    cam.transform.position = new Vector3(250.0f, 1550.0f, 950.0f);
                    cam.transform.LookAt(new Vector3(250.0f, 0.0f, 1100.0f), Vector3.forward);
                    cam.fieldOfView = 70.0f;
                    CaptureCameraToPng(cam, Path.Combine(rootOut, "Preview_3D_Monza.png"), 1920, 1080);
                }
            }

            // 2. Re-save clean runtime scene before building standalone player
            SetupScene();

            // 3. Build Standalone Windows 64-bit Executable
            string buildDir = @"c:\Users\nguye\PROJECT\F1\Build";
            if (!Directory.Exists(buildDir))
            {
                Directory.CreateDirectory(buildDir);
            }
            string exePath = Path.Combine(buildDir, "F1_GrandPrix.exe");

            BuildPlayerOptions buildOptions = new BuildPlayerOptions
            {
                scenes = new[] { ScenePath },
                locationPathName = exePath,
                target = BuildTarget.StandaloneWindows64,
                options = BuildOptions.None
            };

            Debug.Log($"[F1 Sim] Building Standalone Windows64 player to {exePath}...");
            BuildReport report = BuildPipeline.BuildPlayer(buildOptions);
            Debug.Log($"[F1 Sim] Build result: {report.summary.result} ({report.summary.totalErrors} errors, {report.summary.totalWarnings} warnings)");
        }

        private static void CaptureCameraToPng(Camera cam, string filePath, int width, int height)
        {
            RenderTexture rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            rt.antiAliasing = 4;
            RenderTexture prevActive = RenderTexture.active;
            RenderTexture prevTarget = cam.targetTexture;

            cam.targetTexture = rt;
            cam.Render();

            RenderTexture.active = rt;
            Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();

            cam.targetTexture = prevTarget;
            RenderTexture.active = prevActive;
            Object.DestroyImmediate(rt);

            byte[] pngBytes = tex.EncodeToPNG();
            Object.DestroyImmediate(tex);
            File.WriteAllBytes(filePath, pngBytes);
            Debug.Log($"[F1 Sim] Captured 3D verification screenshot: {filePath}");
        }

        private static void EnsureSceneExists()
        {
            if (!File.Exists(ScenePath))
            {
                SetupScene();
            }
        }
    }
}
