using System.Collections.Generic;
using UnityEngine;
using F1Sim.Core;
using F1Sim.Physics;

namespace F1Sim.Cockpit
{
    /// <summary>
    /// Builds an authentic, high-fidelity 3D Ground Effect Formula 1 car model (2022-2026 regulations):
    /// - Sculpted aerodynamic needle nosecone with front camera pods and FIA impact crash structure
    /// - 4-tier cascade front wing flaps with curved outwash endplates and dive planes
    /// - Ground-effect Venturi floor channels with 4 forward strakes, edge winglets & steep upswept rear diffuser
    /// - Deep sidepod undercut channels, aggressive radiator cooling mouths, cooling louvers & downwash gullies
    /// - Dorsal shark fin along engine cover spine, roll-hoop airbox with broadcast T-cam
    /// - FIA titanium Halo safety hoop wrapped in carbon fiber weave, unobstructed cockpit view
    /// - 18-inch wheels with aerodynamic flat wheel covers, Pirelli P-Zero Soft/Medium colored sidewall stripes & tire normal maps
    /// - Swan-neck rear wing with animated DRS flap, dual beam wing & glowing FIA rear rain LED warning light (_EMISSION)
    /// </summary>
    public class F1CarModelBuilder : MonoBehaviour
    {
        public VehicleDynamics vehicle;
        public Color primaryLivery = new Color(0.90f, 0.02f, 0.00f);
        public Color accentLivery = new Color(1.00f, 0.80f, 0.00f);
        public Color helmetColor = Color.white;
        public string driverNumber = "1";
        public bool isPlayerCar = false;
        public bool useSoftTires = true;

        private Transform[] wheelSpinRoots = new Transform[4];
        private Transform[] frontSteerPivots = new Transform[2];
        private Transform drsFlapPivot;
        private GameObject driverHelmetGroup;
        private List<GameObject> interiorConflictingParts = new List<GameObject>();
        private Renderer rearLightRenderer;
        private Material rearLightMat;
        private float wheelSpinAngle = 0.0f;
        private F1CockpitCamera cachedCockpitCam;

        public void BuildCarModel()
        {
            Transform existing = transform.Find("F1_Chassis_Visual");
            if (existing != null) Destroy(existing.gameObject);

            GameObject root = new GameObject("F1_Chassis_Visual");
            root.transform.SetParent(transform, false);

            Material carbonMat = F1Palette.CreateCarbonMaterial();
            Material tireMat = F1Palette.CreatePirelliTireMaterial(useSoftTires);
            Material rimMat = F1Palette.GetOrCreateMaterial("Mat_WheelAeroRim", F1Palette.WheelRimGunmetal, 0.82f, 0.70f);
            Material haloMat = F1Palette.GetOrCreateMaterial("Mat_HaloTitanium", F1Palette.TitaniumHalo, 0.60f, 0.75f, null, null, false, F1Palette.CreateCarbonNormalMap(), 0.9f);
            Material visorMat = F1Palette.GetOrCreateMaterial("Mat_HelmetVisor", new Color(0.04f, 0.07f, 0.14f), 0.95f, 0.95f);
            Material chromeMat = F1Palette.GetOrCreateMaterial("Mat_ChromeExhaust", new Color(0.85f, 0.88f, 0.92f), 0.95f, 0.92f);

            string liveryKey = $"Mat_Livery_{primaryLivery.r:F2}_{primaryLivery.g:F2}_{primaryLivery.b:F2}";
            Material bodyMat = F1Palette.GetOrCreateMaterial(liveryKey, primaryLivery, 0.38f, 0.88f, null, null, false, null, 1.0f, clearcoat: 1.0f);

            string accentKey = $"Mat_Accent_{accentLivery.r:F2}_{accentLivery.g:F2}_{accentLivery.b:F2}";
            Material accentMat = F1Palette.GetOrCreateMaterial(accentKey, accentLivery, 0.30f, 0.82f, null, null, false, null, 1.0f, clearcoat: 0.85f);

            string helmetKey = $"Mat_Helmet_{helmetColor.r:F2}_{helmetColor.g:F2}_{helmetColor.b:F2}";
            Material helmetMat = F1Palette.GetOrCreateMaterial(helmetKey, helmetColor, 0.30f, 0.88f, null, null, false, null, 1.0f, clearcoat: 0.9f);

            // =========================================================================
            // 1. Carbon Ground-Effect Floor & Venturi Underfloor Tunnels
            // =========================================================================
            // Main Venturi underfloor plank (shaped tea-tray splitter to rear axle)
            CreatePart(PrimitiveType.Cube, "Venturi_Underfloor_Plank", root.transform,
                new Vector3(0.0f, 0.065f, -0.10f), Vector3.zero, new Vector3(1.94f, 0.040f, 4.25f), carbonMat);

            // Floor Edge Wings & Longitudinal Sealing Strakes (2024 ground effect floor edge regulations)
            for (int side = -1; side <= 1; side += 2)
            {
                string sLabel = side < 0 ? "L" : "R";
                // Outer floor edge scroll wing
                CreatePart(PrimitiveType.Cube, $"Floor_Edge_Wing_{sLabel}", root.transform,
                    new Vector3(side * 0.965f, 0.082f, -0.20f), new Vector3(0.0f, 0.0f, side * 1.5f),
                    new Vector3(0.035f, 0.024f, 2.30f), carbonMat);

                // Venturi Tunnel Leading Edge Strakes (4 vertical fences channeling underfloor airflow)
                for (int f = 0; f < 2; f++)
                {
                    float xPos = side * (0.42f + f * 0.46f);
                    CreatePart(PrimitiveType.Cube, $"Venturi_Fwd_Fence_{sLabel}_{f}", root.transform,
                        new Vector3(xPos, 0.095f, 0.72f), new Vector3(0.0f, side * (f == 0 ? -4.0f : -10.0f), 0.0f),
                        new Vector3(0.022f, 0.075f, 0.85f), carbonMat);
                }

                // Bargeboard Airflow Cascade Vanes
                CreatePart(PrimitiveType.Cube, $"Aero_Bargeboard_Vane_{sLabel}", root.transform,
                    new Vector3(side * 0.72f, 0.22f, 0.74f), new Vector3(0.0f, side * -14.0f, side * -4.0f),
                    new Vector3(0.022f, 0.24f, 0.58f), carbonMat);
            }

            // Front Splitter Under-Tray extending forward beneath nosecone
            CreatePart(PrimitiveType.Cube, "Front_Splitter_Tray", root.transform,
                new Vector3(0.0f, 0.062f, 2.28f), Vector3.zero, new Vector3(1.84f, 0.032f, 0.58f), carbonMat);

            // Steep Upswept Rear Diffuser (Expanded volume kicking up at 16 degrees)
            CreatePart(PrimitiveType.Cube, "Rear_Diffuser_Main", root.transform,
                new Vector3(0.0f, 0.165f, -2.18f), new Vector3(-16.0f, 0.0f, 0.0f), new Vector3(1.28f, 0.20f, 0.58f), carbonMat);

            // Diffuser Vertical Flow Splitter Strakes (5 vertical fins)
            float[] diffFins = new float[] { -0.45f, -0.22f, 0.0f, 0.22f, 0.45f };
            for (int k = 0; k < diffFins.Length; k++)
            {
                CreatePart(PrimitiveType.Cube, $"Diffuser_Strafe_{k}", root.transform,
                    new Vector3(diffFins[k], 0.165f, -2.18f), new Vector3(-16.0f, 0.0f, 0.0f), new Vector3(0.018f, 0.18f, 0.56f), carbonMat);
            }

            // =========================================================================
            // 2. Sculpted Monocoque Chassis & Needle Nosecone
            // =========================================================================
            interiorConflictingParts.Clear();

            // Central Survival Tub (FIA Homologated Survival Cell)
            GameObject centerTub = CreatePart(PrimitiveType.Cube, "Monocoque_Center_Tub", root.transform,
                new Vector3(0.0f, 0.35f, 0.05f), Vector3.zero, new Vector3(0.68f, 0.46f, 1.85f), bodyMat);
            interiorConflictingParts.Add(centerTub);

            // Upper Sloped Nosecone Section (Vanity panel sloping forward)
            CreatePart(PrimitiveType.Cube, "Nosecone_Upper_Slope", root.transform,
                new Vector3(0.0f, 0.29f, 1.38f), new Vector3(8.0f, 0.0f, 0.0f), new Vector3(0.48f, 0.27f, 1.25f), bodyMat);

            // Needle Nosecone Mid-Section tapering to a fine aerodynamic point
            CreatePart(PrimitiveType.Cube, "Nosecone_Mid_Section", root.transform,
                new Vector3(0.0f, 0.21f, 2.12f), new Vector3(11.5f, 0.0f, 0.0f), new Vector3(0.34f, 0.18f, 0.72f), bodyMat);

            // Needle Nosecone Tip (FIA Impact Crash Cone)
            CreatePart(PrimitiveType.Cube, "Needle_Nosecone_Tip", root.transform,
                new Vector3(0.0f, 0.155f, 2.58f), new Vector3(13.0f, 0.0f, 0.0f), new Vector3(0.22f, 0.13f, 0.45f), bodyMat);

            // Forward Pylon Struts anchoring Nosecone to Front Wing Mainplane
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cube, $"Nose_Wing_Pylon_{(side < 0 ? "L" : "R")}", root.transform,
                    new Vector3(side * 0.085f, 0.12f, 2.45f), new Vector3(18.0f, 0.0f, 0.0f), new Vector3(0.024f, 0.12f, 0.22f), carbonMat);
            }

            // Centerline Championship Livery Racing Stripe
            CreatePart(PrimitiveType.Cube, "Nose_Championship_Stripe", root.transform,
                new Vector3(0.0f, 0.365f, 1.55f), new Vector3(8.5f, 0.0f, 0.0f), new Vector3(0.12f, 0.018f, 1.48f), accentMat);

            // Front Nose Aerodynamic Camera Pods (FIA TV Onboard Cameras)
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cylinder, $"Nose_Camera_Pod_{(side < 0 ? "L" : "R")}", root.transform,
                    new Vector3(side * 0.21f, 0.31f, 1.58f), new Vector3(90.0f, 0.0f, 0.0f), new Vector3(0.028f, 0.065f, 0.028f), carbonMat);
            }

            // =========================================================================
            // 3. Sidepods with Deep Undercut, Inlets, Louvers & Radiator Gullies
            // =========================================================================
            for (int side = -1; side <= 1; side += 2)
            {
                string sLabel = side < 0 ? "L" : "R";

                // Upper sidepod body with downwash water-slide curvature
                CreatePart(PrimitiveType.Cube, $"Sidepod_Upper_Body_{sLabel}", root.transform,
                    new Vector3(side * 0.58f, 0.33f, -0.35f), new Vector3(-3.5f, side * -4.0f, side * 2.0f),
                    new Vector3(0.48f, 0.36f, 1.78f), bodyMat);

                // Deep Sidepod Undercut Channel (huge aerodynamic sculpted undercut feeding rear diffuser)
                CreatePart(PrimitiveType.Cube, $"Sidepod_Undercut_{sLabel}", root.transform,
                    new Vector3(side * 0.50f, 0.145f, 0.18f), new Vector3(0.0f, side * -9.0f, 0.0f),
                    new Vector3(0.28f, 0.13f, 0.78f), carbonMat);

                // Radiator Air Intake Mouth with aggressive overbite lip
                CreatePart(PrimitiveType.Cube, $"Sidepod_Inlet_Mouth_{sLabel}", root.transform,
                    new Vector3(side * 0.60f, 0.37f, 0.54f), new Vector3(-6.0f, 0.0f, 0.0f),
                    new Vector3(0.38f, 0.20f, 0.05f), carbonMat);

                // Sidepod Upper Overbite Air Scoop
                CreatePart(PrimitiveType.Cube, $"Sidepod_Inlet_Lip_{sLabel}", root.transform,
                    new Vector3(side * 0.60f, 0.44f, 0.52f), Vector3.zero,
                    new Vector3(0.39f, 0.025f, 0.08f), bodyMat);

                // Sidepod Livery Accent Pinstripe along outer shoulder
                CreatePart(PrimitiveType.Cube, $"Sidepod_Livery_Stripe_{sLabel}", root.transform,
                    new Vector3(side * 0.825f, 0.33f, -0.25f), new Vector3(-3.0f, side * -4.0f, 0.0f),
                    new Vector3(0.024f, 0.085f, 1.45f), accentMat);

                // Engine Cover Cooling Louvers / Gills on sidepod shoulder
                for (int l = 0; l < 4; l++)
                {
                    CreatePart(PrimitiveType.Cube, $"Cooling_Louver_{sLabel}_{l}", root.transform,
                        new Vector3(side * 0.44f, 0.48f, -0.20f - l * 0.18f), new Vector3(0.0f, side * 22.0f, side * -12.0f),
                        new Vector3(0.12f, 0.015f, 0.065f), carbonMat);
                }
            }

            // Engine Cover Spine sloping back to rear exhaust
            CreatePart(PrimitiveType.Cube, "Engine_Cover_Spine", root.transform,
                new Vector3(0.0f, 0.63f, -0.85f), new Vector3(-14.5f, 0.0f, 0.0f), new Vector3(0.40f, 0.42f, 1.70f), bodyMat);

            // Turbo V6 Exhaust Tailpipe & Wastegate Pipes (Chrome / Inconel)
            CreatePart(PrimitiveType.Cylinder, "Turbo_Exhaust_Tailpipe", root.transform,
                new Vector3(0.0f, 0.38f, -1.95f), new Vector3(90.0f, 0.0f, 0.0f), new Vector3(0.12f, 0.25f, 0.12f), chromeMat);

            // Airbox Roll-Hoop Engine Intake (Above driver's head)
            GameObject rollHoop = CreatePart(PrimitiveType.Cylinder, "Airbox_RollHoop", root.transform,
                new Vector3(0.0f, 0.83f, -0.22f), new Vector3(84.0f, 0.0f, 0.0f), new Vector3(0.28f, 0.24f, 0.32f), bodyMat);
            interiorConflictingParts.Add(rollHoop);

            // Airbox Triangular Intake Mouth (dark internal cavity)
            GameObject rollHoopIntake = CreatePart(PrimitiveType.Cube, "Airbox_Intake_Mouth", root.transform,
                new Vector3(0.0f, 0.84f, -0.06f), new Vector3(-12.0f, 0.0f, 0.0f), new Vector3(0.18f, 0.12f, 0.04f), carbonMat);
            interiorConflictingParts.Add(rollHoopIntake);

            // FIA Broadcast T-Cam on top of Roll-Hoop
            CreatePart(PrimitiveType.Cube, "FIA_Broadcast_TCam", root.transform,
                new Vector3(0.0f, 0.98f, -0.18f), Vector3.zero, new Vector3(0.14f, 0.045f, 0.09f), accentMat);

            // Dorsal Aerodynamic Shark Fin (Directing clean airflow to rear wing)
            CreatePart(PrimitiveType.Cube, "Dorsal_Shark_Fin", root.transform,
                new Vector3(0.0f, 0.80f, -1.18f), Vector3.zero, new Vector3(0.030f, 0.36f, 1.18f), carbonMat);
            CreatePart(PrimitiveType.Cube, "Shark_Fin_Livery_Edge", root.transform,
                new Vector3(0.0f, 0.94f, -1.18f), Vector3.zero, new Vector3(0.036f, 0.055f, 1.15f), accentMat);

            // =========================================================================
            // 4. Titanium Halo & Driver Helmet
            // =========================================================================
            // Halo Center Pillar & Forward Arches (hidden in CockpitEye so CockpitModelBuilder's aero blade Halo is used)
            GameObject extHaloPillar = CreatePart(PrimitiveType.Cylinder, "Halo_Center_Pillar", root.transform,
                new Vector3(0.0f, 0.60f, 0.38f), new Vector3(22.0f, 0.0f, 0.0f), new Vector3(0.035f, 0.16f, 0.035f), haloMat);
            GameObject extHaloArchL = CreatePart(PrimitiveType.Cube, "Halo_Arch_L", root.transform,
                new Vector3(-0.20f, 0.71f, 0.10f), new Vector3(4.0f, -16.0f, 0.0f), new Vector3(0.045f, 0.045f, 0.62f), haloMat);
            GameObject extHaloArchR = CreatePart(PrimitiveType.Cube, "Halo_Arch_R", root.transform,
                new Vector3(0.20f, 0.71f, 0.10f), new Vector3(4.0f, 16.0f, 0.0f), new Vector3(0.045f, 0.045f, 0.62f), haloMat);
            interiorConflictingParts.Add(extHaloPillar);
            interiorConflictingParts.Add(extHaloArchL);
            interiorConflictingParts.Add(extHaloArchR);

            // Driver Helmet & Visor inside Cockpit
            driverHelmetGroup = new GameObject("Driver_Helmet_Assembly");
            driverHelmetGroup.transform.SetParent(root.transform, false);

            CreatePart(PrimitiveType.Sphere, "Driver_Helmet", driverHelmetGroup.transform,
                new Vector3(0.0f, 0.64f, -0.05f), Vector3.zero, new Vector3(0.27f, 0.28f, 0.30f), helmetMat);
            CreatePart(PrimitiveType.Sphere, "Driver_Visor", driverHelmetGroup.transform,
                new Vector3(0.0f, 0.65f, 0.04f), Vector3.zero, new Vector3(0.24f, 0.14f, 0.22f), visorMat);

            // =========================================================================
            // 5. 4-Tier Cascade Front Wing with Curved Outwash Endplates
            // =========================================================================
            GameObject fwRoot = new GameObject("Front_Wing_Assembly");
            fwRoot.transform.SetParent(root.transform, false);
            fwRoot.transform.localPosition = new Vector3(0.0f, 0.10f, 2.52f);

            // Tier 1: Mainplane Aerofoil (Lower curved ground-effect wing plane)
            CreatePart(PrimitiveType.Cube, "FW_Tier1_Mainplane", fwRoot.transform,
                new Vector3(0.0f, 0.0f, 0.0f), Vector3.zero, new Vector3(1.96f, 0.032f, 0.36f), carbonMat);

            // Tier 2: Cascade Flap 1
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cube, $"FW_Tier2_Flap_{(side < 0 ? "L" : "R")}", fwRoot.transform,
                    new Vector3(side * 0.48f, 0.035f, -0.05f), new Vector3(-12.0f, side * 5.0f, 0.0f),
                    new Vector3(0.78f, 0.022f, 0.20f), bodyMat);
            }

            // Tier 3: Cascade Flap 2
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cube, $"FW_Tier3_Flap_{(side < 0 ? "L" : "R")}", fwRoot.transform,
                    new Vector3(side * 0.46f, 0.065f, -0.10f), new Vector3(-18.0f, side * 7.0f, 0.0f),
                    new Vector3(0.74f, 0.020f, 0.18f), bodyMat);
            }

            // Tier 4: Upper Gurney Flap Element
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cube, $"FW_Tier4_Flap_{(side < 0 ? "L" : "R")}", fwRoot.transform,
                    new Vector3(side * 0.44f, 0.092f, -0.14f), new Vector3(-24.0f, side * 9.0f, 0.0f),
                    new Vector3(0.70f, 0.018f, 0.14f), accentMat);
            }

            // Curved Front Wing Endplates with Outer Footplate & Outwash Dive Plane
            for (int side = -1; side <= 1; side += 2)
            {
                string sLabel = side < 0 ? "L" : "R";
                // Main Endplate
                CreatePart(PrimitiveType.Cube, $"FW_Endplate_{sLabel}", fwRoot.transform,
                    new Vector3(side * 0.98f, 0.12f, -0.02f), new Vector3(0.0f, 0.0f, side * -7.0f),
                    new Vector3(0.032f, 0.27f, 0.48f), bodyMat);

                // Outer Footplate / Tire Squirt Lip
                CreatePart(PrimitiveType.Cube, $"FW_Footplate_{sLabel}", fwRoot.transform,
                    new Vector3(side * 1.01f, 0.015f, -0.02f), Vector3.zero,
                    new Vector3(0.045f, 0.016f, 0.45f), carbonMat);

                // Curved Outwash Dive Plane flick
                CreatePart(PrimitiveType.Cube, $"FW_DivePlane_{sLabel}", fwRoot.transform,
                    new Vector3(side * 1.00f, 0.18f, 0.04f), new Vector3(-22.0f, 0.0f, side * 15.0f),
                    new Vector3(0.055f, 0.018f, 0.22f), carbonMat);
            }

            // =========================================================================
            // 6. Swan-Neck Rear Wing, Dual Beam Wing & Working DRS Flap
            // =========================================================================
            GameObject rwRoot = new GameObject("Rear_Wing_Assembly");
            rwRoot.transform.SetParent(root.transform, false);
            rwRoot.transform.localPosition = new Vector3(0.0f, 0.70f, -2.05f);

            // Twin swan-neck carbon pylons (curved over top of main aerofoil)
            CreatePart(PrimitiveType.Cube, "RW_Pylon_L", rwRoot.transform,
                new Vector3(-0.08f, -0.12f, 0.08f), new Vector3(14.0f, 0.0f, 0.0f), new Vector3(0.028f, 0.44f, 0.18f), carbonMat);
            CreatePart(PrimitiveType.Cube, "RW_Pylon_R", rwRoot.transform,
                new Vector3(0.08f, -0.12f, 0.08f), new Vector3(14.0f, 0.0f, 0.0f), new Vector3(0.028f, 0.44f, 0.18f), carbonMat);

            // Sculpted Mainplane Aerofoil (Cambered carbon wing profile)
            CreatePart(PrimitiveType.Cube, "RW_Mainplane", rwRoot.transform,
                new Vector3(0.0f, 0.065f, 0.0f), new Vector3(8.5f, 0.0f, 0.0f), new Vector3(1.08f, 0.035f, 0.30f), carbonMat);

            // Dual Lower Beam Wing Elements (2022+ regulation beam wing mounted above diffuser)
            CreatePart(PrimitiveType.Cube, "RW_BeamWing_Lower", rwRoot.transform,
                new Vector3(0.0f, -0.32f, 0.04f), new Vector3(16.0f, 0.0f, 0.0f), new Vector3(0.92f, 0.024f, 0.22f), carbonMat);
            CreatePart(PrimitiveType.Cube, "RW_BeamWing_Upper", rwRoot.transform,
                new Vector3(0.0f, -0.24f, 0.01f), new Vector3(22.0f, 0.0f, 0.0f), new Vector3(0.88f, 0.020f, 0.18f), bodyMat);

            // Active DRS Upper Flap on animated pivot
            GameObject drsPivot = new GameObject("RW_DRS_Flap_Pivot");
            drsPivot.transform.SetParent(rwRoot.transform, false);
            drsPivot.transform.localPosition = new Vector3(0.0f, 0.155f, -0.06f);
            drsFlapPivot = drsPivot.transform;

            CreatePart(PrimitiveType.Cube, "RW_DRS_Flap", drsPivot.transform,
                new Vector3(0.0f, 0.0f, 0.05f), new Vector3(25.0f, 0.0f, 0.0f), new Vector3(1.04f, 0.026f, 0.19f), accentMat);

            // DRS Center Hydraulic Actuator Pod
            CreatePart(PrimitiveType.Cube, "DRS_Actuator_Pod", rwRoot.transform,
                new Vector3(0.0f, 0.14f, 0.02f), Vector3.zero, new Vector3(0.055f, 0.065f, 0.12f), carbonMat);

            // Rear Wing Endplates with aerodynamic edge cutouts
            for (int side = -1; side <= 1; side += 2)
            {
                CreatePart(PrimitiveType.Cube, $"RW_Endplate_{(side < 0 ? "L" : "R")}", rwRoot.transform,
                    new Vector3(side * 0.54f, -0.02f, 0.0f), Vector3.zero, new Vector3(0.035f, 0.48f, 0.44f), bodyMat);
            }

            // FIA Rear Rain / ERS Harvesting LED Tail Light (15-LED cluster with intense emission)
            rearLightMat = new Material(F1Palette.GetLitShader()) { name = "Mat_RearRainLight" };
            Color tailRed = new Color(1.0f, 0.06f, 0.04f);
            rearLightMat.color = tailRed;
            if (rearLightMat.HasProperty("_BaseColor")) rearLightMat.SetColor("_BaseColor", tailRed);
            rearLightMat.EnableKeyword("_EMISSION");
            if (rearLightMat.HasProperty("_EmissionColor")) rearLightMat.SetColor("_EmissionColor", tailRed * 4.5f);

            GameObject rainLight = CreatePart(PrimitiveType.Cube, "FIA_Rear_Rain_Light", root.transform,
                new Vector3(0.0f, 0.26f, -2.29f), Vector3.zero, new Vector3(0.08f, 0.13f, 0.035f), rearLightMat);
            rearLightRenderer = rainLight.GetComponent<Renderer>();

            // =========================================================================
            // 7. Four 18-Inch Pirelli Wheels & Carbon Double-Wishbone Suspension
            // =========================================================================
            Vector3[] wheelPositions = new Vector3[4]
            {
                new Vector3(-0.84f, 0.36f,  1.62f), // FL
                new Vector3( 0.84f, 0.36f,  1.62f), // FR
                new Vector3(-0.82f, 0.36f, -1.78f), // RL
                new Vector3( 0.82f, 0.36f, -1.78f)  // RR
            };

            Color tireStripeColor = useSoftTires ? F1Palette.PirelliSoftRed : F1Palette.PirelliMedYellow;
            Material tireStripeMat = F1Palette.GetOrCreateMaterial(
                useSoftTires ? "Mat_PirelliStripeSoft" : "Mat_PirelliStripeMed",
                tireStripeColor,
                0.20f,
                0.75f,
                null,
                tireStripeColor * 0.8f);

            for (int i = 0; i < 4; i++)
            {
                bool isFront = (i < 2);
                float side = (i % 2 == 0) ? -1.0f : 1.0f;
                Vector3 wPos = wheelPositions[i];

                // Double-Wishbone Carbon Suspension Struts
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

                // Pushrod / Pullrod Strut
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
                        new Vector3(0.24f, 0.025f, 0.34f), carbonMat);
                }

                // Carbon Brake Cooling Duct Scoop
                CreatePart(PrimitiveType.Cube, $"Brake_Duct_{i}", steerPivot.transform,
                    new Vector3(-side * 0.12f, 0.0f, 0.0f), Vector3.zero,
                    new Vector3(0.06f, 0.18f, 0.16f), carbonMat);

                // Spinning Tire & Rim Hub
                GameObject spinHub = new GameObject($"Wheel_Spin_{i}");
                spinHub.transform.SetParent(steerPivot.transform, false);
                wheelSpinRoots[i] = spinHub.transform;

                float halfTireWidth = isFront ? 0.165f : 0.205f;

                // 18-inch Pirelli Slick Tire Rubber Cylinder with Normal Map
                CreatePart(PrimitiveType.Cylinder, $"Pirelli_Slick_{i}", spinHub.transform,
                    Vector3.zero, new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.72f, halfTireWidth, 0.72f), tireMat);

                // 18-inch Aerodynamic Flat Wheel Cover Disc (BBS Aero Disc)
                CreatePart(PrimitiveType.Cylinder, $"Wheel_Aero_Cover_{i}", spinHub.transform,
                    new Vector3(side * (halfTireWidth * 0.5f + 0.003f), 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.48f, 0.004f, 0.48f), rimMat);

                // Center Lock Wheel Nut (Anodized Red / Blue lock nut)
                Color nutCol = (side < 0) ? new Color(0.9f, 0.1f, 0.1f) : new Color(0.1f, 0.4f, 0.9f);
                Material nutMat = F1Palette.GetOrCreateMaterial($"Mat_CenterNut_{side}", nutCol, 0.85f, 0.85f);
                CreatePart(PrimitiveType.Cylinder, $"Wheel_Center_Nut_{i}", spinHub.transform,
                    new Vector3(side * (halfTireWidth * 0.5f + 0.008f), 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.10f, 0.006f, 0.10f), nutMat);

                // Pirelli Soft (Red) / Medium (Yellow) Sidewall Pinstripe Ring
                CreatePart(PrimitiveType.Cylinder, $"Pirelli_Sidewall_Ring_{i}", spinHub.transform,
                    new Vector3(side * (halfTireWidth * 0.5f + 0.002f), 0.0f, 0.0f), new Vector3(0.0f, 0.0f, 90.0f),
                    new Vector3(0.58f, 0.003f, 0.58f), tireStripeMat);
            }

            // For player car in cockpit perspective, hide helmet and internal monocoque obstacles immediately
            if (isPlayerCar)
            {
                if (driverHelmetGroup != null) driverHelmetGroup.SetActive(false);
                for (int p = 0; p < interiorConflictingParts.Count; p++)
                {
                    if (interiorConflictingParts[p] != null) interiorConflictingParts[p].SetActive(false);
                }
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

            // 1. Steer Front Wheels with Vehicle Dynamics Steering Angle
            float steerDeg = -vehicle.state.steeringAngle * Mathf.Rad2Deg * 0.55f;
            for (int i = 0; i < 2; i++)
            {
                if (frontSteerPivots[i] != null)
                    frontSteerPivots[i].localRotation = Quaternion.Euler(0.0f, steerDeg, 0.0f);
            }

            // 2. Spin All 4 Wheels with Forward Speed
            for (int i = 0; i < 4; i++)
            {
                if (wheelSpinRoots[i] != null)
                    wheelSpinRoots[i].localRotation = Quaternion.Euler(wheelSpinAngle, 0.0f, 0.0f);
            }

            // 3. Animate DRS Rear Wing Flap
            if (drsFlapPivot != null)
            {
                float targetPitch = vehicle.state.drsActive ? -25.0f : 0.0f;
                drsFlapPivot.localRotation = Quaternion.Lerp(
                    drsFlapPivot.localRotation,
                    Quaternion.Euler(targetPitch, 0.0f, 0.0f),
                    dt * 12.0f);
            }

            // 4. Driver Helmet & Cockpit Overlap Visibility Management: hide in CockpitEye mode
            if (isPlayerCar)
            {
                if (cachedCockpitCam == null) cachedCockpitCam = FindFirstObjectByType<F1CockpitCamera>();
                bool inCockpitEye = (cachedCockpitCam == null || cachedCockpitCam.cameraMode == F1CameraMode.CockpitEye);
                if (driverHelmetGroup != null && driverHelmetGroup.activeSelf == inCockpitEye)
                {
                    driverHelmetGroup.SetActive(!inCockpitEye);
                }
                for (int p = 0; p < interiorConflictingParts.Count; p++)
                {
                    if (interiorConflictingParts[p] != null && interiorConflictingParts[p].activeSelf == inCockpitEye)
                    {
                        interiorConflictingParts[p].SetActive(!inCockpitEye);
                    }
                }
            }

            // 5. Blink FIA Rear Rain / ERS Harvesting Tail Light when braking or harvesting
            if (rearLightMat != null)
            {
                bool harvesting = (vehicle.state.brake > 0.05f) || (vehicle.state.throttle < 0.25f && speedMs > 40.0f);
                float blink = harvesting ? ((Mathf.Sin(Time.time * 28.0f) > 0.0f) ? 5.0f : 0.4f) : 1.8f;
                Color baseRed = new Color(1.0f, 0.06f, 0.04f);
                if (rearLightMat.HasProperty("_EmissionColor"))
                    rearLightMat.SetColor("_EmissionColor", baseRed * blink);
            }
        }
    }
}
