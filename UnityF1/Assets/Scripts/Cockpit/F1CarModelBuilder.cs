using UnityEngine;
using F1Sim.Core;
using F1Sim.Physics;

namespace F1Sim.Cockpit
{
    /// <summary>
    /// Builds a detailed, multi-part 3D Formula 1 car model with authentic silhouette,
    /// double-wishbone carbon suspension, rotating 18-inch Pirelli tires, active DRS flap,
    /// driver helmet, Halo titanium hoop, and glowing FIA rear rain light.
    /// </summary>
    public class F1CarModelBuilder : MonoBehaviour
    {
        public VehicleDynamics vehicle;
        public Color primaryLivery = new Color(0.88f, 0.02f, 0.00f);
        public Color accentLivery = new Color(1.00f, 0.78f, 0.00f);
        public Color helmetColor = Color.white;
        public string driverNumber = "1";
        public bool isPlayerCar = false;

        private Transform[] wheelSpinRoots = new Transform[4];
        private Transform[] frontSteerPivots = new Transform[2];
        private Transform drsFlapPivot;
        private Renderer rearLightRenderer;
        private Material rearLightMat;
        private float wheelSpinAngle = 0.0f;

        public void BuildCarModel()
        {
            Transform existing = transform.Find("F1_Chassis_Visual");
            if (existing != null) Destroy(existing.gameObject);

            GameObject root = new GameObject("F1_Chassis_Visual");
            root.transform.SetParent(transform, false);

            Material carbonMat = F1Palette.CreateCarbonMaterial();
            Material tireMat = F1Palette.GetOrCreateMaterial("Mat_PirelliTire", F1Palette.TireRubber, 0.05f, 0.22f);
            Material rimMat = F1Palette.GetOrCreateMaterial("Mat_WheelRim", F1Palette.WheelRimGunmetal, 0.75f, 0.65f);
            Material haloMat = F1Palette.GetOrCreateMaterial("Mat_HaloTitanium", F1Palette.TitaniumHalo, 0.55f, 0.60f);
            Material visorMat = F1Palette.GetOrCreateMaterial("Mat_HelmetVisor", new Color(0.05f, 0.08f, 0.15f), 0.95f, 0.95f);

            string liveryKey = $"Mat_Livery_{primaryLivery.r:F2}_{primaryLivery.g:F2}_{primaryLivery.b:F2}";
            Material bodyMat = F1Palette.GetOrCreateMaterial(liveryKey, primaryLivery, 0.35f, 0.82f);

            string accentKey = $"Mat_Accent_{accentLivery.r:F2}_{accentLivery.g:F2}_{accentLivery.b:F2}";
            Material accentMat = F1Palette.GetOrCreateMaterial(accentKey, accentLivery, 0.25f, 0.75f);

            string helmetKey = $"Mat_Helmet_{helmetColor.r:F2}_{helmetColor.g:F2}_{helmetColor.b:F2}";
            Material helmetMat = F1Palette.GetOrCreateMaterial(helmetKey, helmetColor, 0.30f, 0.88f);

            // 1. Carbon Underfloor & Venturi Tunnels (Ground Effect Floor)
            CreatePart(PrimitiveType.Cube, "Venturi_Underfloor", root.transform,
                new Vector3(0.0f, 0.07f, -0.10f), Vector3.zero, new Vector3(1.88f, 0.045f, 4.10f), carbonMat);

            // Rear Diffuser Strakes
            CreatePart(PrimitiveType.Cube, "Rear_Diffuser", root.transform,
                new Vector3(0.0f, 0.14f, -2.18f), new Vector3(-12.0f, 0.0f, 0.0f), new Vector3(1.15f, 0.16f, 0.45f), carbonMat);

            // 2. Central Monocoque Tub & Tapered Nosecone
            if (!isPlayerCar)
            {
                CreatePart(PrimitiveType.Cube, "Monocoque_Center_Tub", root.transform,
                    new Vector3(0.0f, 0.34f, 0.05f), Vector3.zero, new Vector3(0.68f, 0.46f, 1.85f), bodyMat);
            }

            // Upper Nose Section (sloped down at 8 deg)
            CreatePart(PrimitiveType.Cube, "Nosecone_Upper", root.transform,
                new Vector3(0.0f, 0.28f, 1.38f), new Vector3(7.5f, 0.0f, 0.0f), new Vector3(0.46f, 0.28f, 1.25f), bodyMat);

            // Nose Tip (sloped down at 11 deg)
            CreatePart(PrimitiveType.Cube, "Nosecone_Tip", root.transform,
                new Vector3(0.0f, 0.18f, 2.22f), new Vector3(10.5f, 0.0f, 0.0f), new Vector3(0.30f, 0.16f, 0.85f), bodyMat);

            // Championship Racing Stripe along Nose
            CreatePart(PrimitiveType.Cube, "Nose_Racing_Stripe", root.transform,
                new Vector3(0.0f, 0.355f, 1.55f), new Vector3(8.5f, 0.0f, 0.0f), new Vector3(0.12f, 0.02f, 1.45f), accentMat);

            // 3. Multi-Element Front Wing
            GameObject fwRoot = new GameObject("Front_Wing_Assembly");
            fwRoot.transform.SetParent(root.transform, false);
            fwRoot.transform.localPosition = new Vector3(0.0f, 0.10f, 2.48f);

            CreatePart(PrimitiveType.Cube, "FW_Mainplane", fwRoot.transform,
                new Vector3(0.0f, 0.0f, 0.0f), Vector3.zero, new Vector3(1.94f, 0.035f, 0.34f), carbonMat);
            CreatePart(PrimitiveType.Cube, "FW_UpperFlap_L", fwRoot.transform,
                new Vector3(-0.48f, 0.045f, -0.06f), new Vector3(-14.0f, 6.0f, 0.0f), new Vector3(0.76f, 0.025f, 0.22f), bodyMat);
            CreatePart(PrimitiveType.Cube, "FW_UpperFlap_R", fwRoot.transform,
                new Vector3(0.48f, 0.045f, -0.06f), new Vector3(-14.0f, -6.0f, 0.0f), new Vector3(0.76f, 0.025f, 0.22f), bodyMat);
            CreatePart(PrimitiveType.Cube, "FW_Endplate_L", fwRoot.transform,
                new Vector3(-0.97f, 0.12f, -0.02f), new Vector3(0.0f, 0.0f, -6.0f), new Vector3(0.035f, 0.26f, 0.44f), bodyMat);
            CreatePart(PrimitiveType.Cube, "FW_Endplate_R", fwRoot.transform,
                new Vector3(0.97f, 0.12f, -0.02f), new Vector3(0.0f, 0.0f, 6.0f), new Vector3(0.035f, 0.26f, 0.44f), bodyMat);

            // 4. Sculpted Sidepods & Engine Cover (for AI cars and exterior)
            for (int side = -1; side <= 1; side += 2)
            {
                string sLabel = side < 0 ? "L" : "R";
                // Sidepod main body
                CreatePart(PrimitiveType.Cube, $"Sidepod_Body_{sLabel}", root.transform,
                    new Vector3(side * 0.56f, 0.31f, -0.35f), new Vector3(0.0f, side * -4.0f, 0.0f), new Vector3(0.48f, 0.38f, 1.75f), bodyMat);
                // Sidepod radiator inlet mouth
                CreatePart(PrimitiveType.Cube, $"Sidepod_Inlet_{sLabel}", root.transform,
                    new Vector3(side * 0.58f, 0.36f, 0.53f), Vector3.zero, new Vector3(0.38f, 0.22f, 0.06f), carbonMat);
                // Sidepod accent stripe
                CreatePart(PrimitiveType.Cube, $"Sidepod_Accent_{sLabel}", root.transform,
                    new Vector3(side * 0.805f, 0.32f, -0.25f), new Vector3(0.0f, side * -4.0f, 0.0f), new Vector3(0.02f, 0.09f, 1.40f), accentMat);
            }

            if (!isPlayerCar)
            {
                // Airbox Roll Hoop & Engine Cover Slope
                CreatePart(PrimitiveType.Cube, "Engine_Cover_Spine", root.transform,
                    new Vector3(0.0f, 0.62f, -0.82f), new Vector3(-14.0f, 0.0f, 0.0f), new Vector3(0.42f, 0.42f, 1.65f), bodyMat);
                CreatePart(PrimitiveType.Cylinder, "Airbox_RollHoop", root.transform,
                    new Vector3(0.0f, 0.82f, -0.22f), new Vector3(85.0f, 0.0f, 0.0f), new Vector3(0.28f, 0.22f, 0.32f), bodyMat);

                // Dorsal Shark Fin
                CreatePart(PrimitiveType.Cube, "Aero_Shark_Fin", root.transform,
                    new Vector3(0.0f, 0.78f, -1.18f), Vector3.zero, new Vector3(0.03f, 0.34f, 1.15f), carbonMat);
                CreatePart(PrimitiveType.Cube, "Shark_Fin_Stripe", root.transform,
                    new Vector3(0.0f, 0.92f, -1.18f), Vector3.zero, new Vector3(0.036f, 0.06f, 1.12f), accentMat);

                // Driver Helmet & Visor inside Cockpit
                CreatePart(PrimitiveType.Sphere, "Driver_Helmet", root.transform,
                    new Vector3(0.0f, 0.64f, -0.05f), Vector3.zero, new Vector3(0.27f, 0.28f, 0.30f), helmetMat);
                CreatePart(PrimitiveType.Sphere, "Driver_Visor", root.transform,
                    new Vector3(0.0f, 0.65f, 0.04f), Vector3.zero, new Vector3(0.24f, 0.14f, 0.22f), visorMat);

                // Exterior Halo Hoop
                CreatePart(PrimitiveType.Cylinder, "AI_Halo_Pillar", root.transform,
                    new Vector3(0.0f, 0.60f, 0.38f), new Vector3(22.0f, 0.0f, 0.0f), new Vector3(0.035f, 0.16f, 0.035f), haloMat);
                CreatePart(PrimitiveType.Cube, "AI_Halo_Arch_L", root.transform,
                    new Vector3(-0.20f, 0.71f, 0.10f), new Vector3(4.0f, -16.0f, 0.0f), new Vector3(0.045f, 0.045f, 0.62f), haloMat);
                CreatePart(PrimitiveType.Cube, "AI_Halo_Arch_R", root.transform,
                    new Vector3(0.20f, 0.71f, 0.10f), new Vector3(4.0f, 16.0f, 0.0f), new Vector3(0.045f, 0.045f, 0.62f), haloMat);
            }

            // 5. Rear Wing Assembly & Active DRS Flap
            GameObject rwRoot = new GameObject("Rear_Wing_Assembly");
            rwRoot.transform.SetParent(root.transform, false);
            rwRoot.transform.localPosition = new Vector3(0.0f, 0.68f, -2.05f);

            // Swan-neck central pillars
            CreatePart(PrimitiveType.Cube, "RW_Pillar_L", rwRoot.transform,
                new Vector3(-0.08f, -0.14f, 0.08f), new Vector3(12.0f, 0.0f, 0.0f), new Vector3(0.03f, 0.42f, 0.18f), carbonMat);
            CreatePart(PrimitiveType.Cube, "RW_Pillar_R", rwRoot.transform,
                new Vector3(0.08f, -0.14f, 0.08f), new Vector3(12.0f, 0.0f, 0.0f), new Vector3(0.03f, 0.42f, 0.18f), carbonMat);

            // Mainplane
            CreatePart(PrimitiveType.Cube, "RW_Mainplane", rwRoot.transform,
                new Vector3(0.0f, 0.06f, 0.0f), new Vector3(8.0f, 0.0f, 0.0f), new Vector3(1.04f, 0.035f, 0.28f), carbonMat);

            // Active DRS Upper Flap
            GameObject drsPivot = new GameObject("RW_DRS_Flap_Pivot");
            drsPivot.transform.SetParent(rwRoot.transform, false);
            drsPivot.transform.localPosition = new Vector3(0.0f, 0.15f, -0.06f);
            drsFlapPivot = drsPivot.transform;

            CreatePart(PrimitiveType.Cube, "RW_DRS_Flap", drsPivot.transform,
                new Vector3(0.0f, 0.0f, 0.05f), new Vector3(24.0f, 0.0f, 0.0f), new Vector3(1.02f, 0.025f, 0.18f), accentMat);

            // Rear Wing Endplates
            CreatePart(PrimitiveType.Cube, "RW_Endplate_L", rwRoot.transform,
                new Vector3(-0.53f, -0.02f, 0.0f), Vector3.zero, new Vector3(0.035f, 0.46f, 0.42f), bodyMat);
            CreatePart(PrimitiveType.Cube, "RW_Endplate_R", rwRoot.transform,
                new Vector3(0.53f, -0.02f, 0.0f), Vector3.zero, new Vector3(0.035f, 0.46f, 0.42f), bodyMat);

            // FIA Rear Rain / ERS Harvesting LED Tail Light
            rearLightMat = new Material(F1Palette.GetLitShader()) { name = "Mat_RearRainLight" };
            Color tailRed = new Color(1.0f, 0.08f, 0.05f);
            rearLightMat.color = tailRed;
            if (rearLightMat.HasProperty("_BaseColor")) rearLightMat.SetColor("_BaseColor", tailRed);
            rearLightMat.EnableKeyword("_EMISSION");
            if (rearLightMat.HasProperty("_EmissionColor")) rearLightMat.SetColor("_EmissionColor", tailRed * 3.5f);

            GameObject rainLight = CreatePart(PrimitiveType.Cube, "FIA_Rear_Rain_Light", root.transform,
                new Vector3(0.0f, 0.26f, -2.28f), Vector3.zero, new Vector3(0.06f, 0.12f, 0.04f), rearLightMat);
            rearLightRenderer = rainLight.GetComponent<Renderer>();

            // 6. Four 18-Inch Pirelli Wheels & Double-Wishbone Carbon Suspension
            Vector3[] wheelPositions = new Vector3[4]
            {
                new Vector3(-0.84f, 0.36f,  1.62f), // FL
                new Vector3( 0.84f, 0.36f,  1.62f), // FR
                new Vector3(-0.82f, 0.36f, -1.78f), // RL
                new Vector3( 0.82f, 0.36f, -1.78f)  // RR
            };

            for (int i = 0; i < 4; i++)
            {
                bool isFront = (i < 2);
                float side = (i % 2 == 0) ? -1.0f : 1.0f;
                Vector3 wPos = wheelPositions[i];

                // Double-Wishbone Suspension Struts (Upper & Lower Wishbones + Pushrod)
                float chassisAnchorX = side * 0.28f;
                float wheelHubX = wPos.x - side * 0.14f;
                float armSpan = Mathf.Abs(wheelHubX - chassisAnchorX);
                float midX = (chassisAnchorX + wheelHubX) * 0.5f;

                // Upper Wishbone Forward & Rearward Arms
                CreatePart(PrimitiveType.Cube, $"Susp_Upper_Fwd_{i}", root.transform,
                    new Vector3(midX, 0.44f, wPos.z + 0.08f), new Vector3(0.0f, side * -14.0f, 0.0f),
                    new Vector3(armSpan, 0.024f, 0.045f), carbonMat);
                CreatePart(PrimitiveType.Cube, $"Susp_Upper_Rear_{i}", root.transform,
                    new Vector3(midX, 0.44f, wPos.z - 0.08f), new Vector3(0.0f, side * 14.0f, 0.0f),
                    new Vector3(armSpan, 0.024f, 0.045f), carbonMat);

                // Lower Wishbone
                CreatePart(PrimitiveType.Cube, $"Susp_Lower_{i}", root.transform,
                    new Vector3(midX, 0.24f, wPos.z), Vector3.zero,
                    new Vector3(armSpan, 0.026f, 0.06f), carbonMat);

                // Diagonal Pushrod
                CreatePart(PrimitiveType.Cube, $"Susp_Pushrod_{i}", root.transform,
                    new Vector3(midX, 0.38f, wPos.z + 0.03f), new Vector3(0.0f, 0.0f, side * -26.0f),
                    new Vector3(armSpan * 1.08f, 0.020f, 0.028f), carbonMat);

                // Wheel Steer Pivot
                GameObject steerPivot = new GameObject($"Wheel_Pivot_{i}");
                steerPivot.transform.SetParent(root.transform, false);
                steerPivot.transform.localPosition = wPos;
                if (isFront) frontSteerPivots[i] = steerPivot.transform;

                // Over-Wheel Aerodynamic Wake Deflector (FIA 2022+ Regulation Winglet)
                if (isFront)
                {
                    CreatePart(PrimitiveType.Cube, $"Aero_Wheel_Fender_{i}", steerPivot.transform,
                        new Vector3(-side * 0.08f, 0.39f, -0.05f), new Vector3(12.0f, 0.0f, side * -15.0f),
                        new Vector3(0.24f, 0.025f, 0.32f), carbonMat);
                }

                // Spinning Tire & Rim Hub
                GameObject spinHub = new GameObject($"Wheel_Spin_{i}");
                spinHub.transform.SetParent(steerPivot.transform, false);
                wheelSpinRoots[i] = spinHub.transform;

                float halfTireWidth = isFront ? 0.165f : 0.205f;
                // Tire Cylinder (rotated 90 deg on Z so cylinder axis is X)
                CreatePart(PrimitiveType.Cylinder, $"Pirelli_Slick_{i}", spinHub.transform,
                    Vector3.zero, new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.72f, halfTireWidth, 0.72f), tireMat);

                // Aero Wheel Cover Disc
                CreatePart(PrimitiveType.Cylinder, $"Aero_Rim_Cover_{i}", spinHub.transform,
                    Vector3.zero, new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.46f, halfTireWidth + 0.006f, 0.46f), rimMat);

                // Pirelli Soft Red Sidewall Stripe + Rotational Spoke Marker
                CreatePart(PrimitiveType.Cylinder, $"Pirelli_Stripe_{i}", spinHub.transform,
                    Vector3.zero, new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.56f, halfTireWidth + 0.004f, 0.56f), accentMat);

                CreatePart(PrimitiveType.Cube, $"Wheel_Spoke_Marker_{i}", spinHub.transform,
                    new Vector3(side * (halfTireWidth + 0.008f), 0.14f, 0.0f), Vector3.zero,
                    new Vector3(0.01f, 0.16f, 0.04f), accentMat);
            }
        }

        private GameObject CreatePart(
            PrimitiveType type,
            string name,
            Transform parent,
            Vector3 localPos,
            Vector3 localEuler,
            Vector3 localScale,
            Material mat)
        {
            GameObject go = GameObject.CreatePrimitive(type);
            go.name = name;
            go.transform.SetParent(parent, false);
            go.transform.localPosition = localPos;
            go.transform.localRotation = Quaternion.Euler(localEuler);
            go.transform.localScale = localScale;

            Collider col = go.GetComponent<Collider>();
            if (col != null) Destroy(col);

            Renderer rend = go.GetComponent<Renderer>();
            if (rend != null && mat != null)
            {
                rend.sharedMaterial = mat;
                rend.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
                rend.receiveShadows = true;
            }
            return go;
        }

        private void Update()
        {
            if (vehicle == null) return;

            float dt = Time.deltaTime;
            float speedMs = vehicle.SpeedMs;
            wheelSpinAngle = Mathf.Repeat(wheelSpinAngle + (speedMs / 0.36f) * Mathf.Rad2Deg * dt, 360.0f);

            // 1. Steer Front Wheels
            float steerDeg = -vehicle.state.steeringAngle * Mathf.Rad2Deg * 0.55f;
            for (int i = 0; i < 2; i++)
            {
                if (frontSteerPivots[i] != null)
                    frontSteerPivots[i].localRotation = Quaternion.Euler(0.0f, steerDeg, 0.0f);
            }

            // 2. Spin All 4 Wheels
            for (int i = 0; i < 4; i++)
            {
                if (wheelSpinRoots[i] != null)
                    wheelSpinRoots[i].localRotation = Quaternion.Euler(wheelSpinAngle, 0.0f, 0.0f);
            }

            // 3. Animate DRS Rear Wing Flap
            if (drsFlapPivot != null)
            {
                float targetPitch = vehicle.state.drsActive ? -24.0f : 0.0f;
                drsFlapPivot.localRotation = Quaternion.Lerp(
                    drsFlapPivot.localRotation,
                    Quaternion.Euler(targetPitch, 0.0f, 0.0f),
                    dt * 12.0f);
            }

            // 4. Blink FIA Rear Rain / ERS Harvesting Tail Light when braking or harvesting
            if (rearLightMat != null)
            {
                bool harvesting = (vehicle.state.brake > 0.05f) || (vehicle.state.throttle < 0.25f && speedMs > 40.0f);
                float blink = harvesting ? ((Mathf.Sin(Time.time * 28.0f) > 0.0f) ? 4.5f : 0.4f) : 1.5f;
                Color baseRed = new Color(1.0f, 0.08f, 0.05f);
                if (rearLightMat.HasProperty("_EmissionColor"))
                    rearLightMat.SetColor("_EmissionColor", baseRed * blink);
            }
        }
    }
}
