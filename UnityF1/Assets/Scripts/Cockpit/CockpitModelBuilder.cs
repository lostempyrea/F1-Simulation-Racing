using System.Collections.Generic;
using UnityEngine;
using F1Sim.Core;
using F1Sim.Physics;

namespace F1Sim.Cockpit
{
    /// <summary>
    /// Builds a high-craft first-person F1 cockpit interior, sculpted titanium Halo,
    /// butterfly carbon steering wheel with 15 3D emissive shift LEDs, rotary dials,
    /// tactile buttons, rear-view mirrors, and visible front wheels/suspension.
    /// </summary>
    public class CockpitModelBuilder : MonoBehaviour
    {
        [Header("Vehicle Reference")]
        public VehicleDynamics vehicle;

        [Header("Cockpit Meshes")]
        public Transform steeringWheelTransform;
        public Renderer haloRenderer;
        public Camera leftMirrorCamera;
        public Camera rightMirrorCamera;

        [Header("Driver Arms & Gloves")]
        public Transform leftArmTransform;
        public Transform rightArmTransform;
        public Transform leftGloveTransform;
        public Transform rightGloveTransform;
        public Transform leftShoulderAnchor;
        public Transform rightShoulderAnchor;

        [Header("Halo Settings")]
        public float haloOpacity = 1.0f; // 1.0 -> 0.35 -> 0.15 -> 1.0

        private float columnRakeDeg = -12.0f;
        private Material haloMaterial;
        private List<Renderer> haloParts = new List<Renderer>();

        private void Start()
        {
            if (steeringWheelTransform == null)
            {
                BuildProceduralCockpit();
            }
        }

        public void BuildProceduralCockpit()
        {
            Transform old = transform.Find("Cockpit_Interior");
            if (old != null) Destroy(old.gameObject);

            // 1. Attach external front chassis, front wing, double-wishbone suspension & spinning 18" Pirelli wheels
            F1CarModelBuilder carBuilder = (vehicle != null) ? vehicle.GetComponent<F1CarModelBuilder>() : GetComponent<F1CarModelBuilder>();
            if (carBuilder == null && vehicle != null) carBuilder = vehicle.gameObject.AddComponent<F1CarModelBuilder>();
            if (carBuilder != null)
            {
                carBuilder.vehicle = vehicle;
                carBuilder.primaryLivery = F1Palette.RacingRed;
                carBuilder.accentLivery = F1Palette.ModenaYellow;
                carBuilder.isPlayerCar = true;
                carBuilder.BuildCarModel();
            }

            // 2. Root container for detailed first-person cockpit interior
            GameObject cockpitRoot = new GameObject("Cockpit_Interior");
            cockpitRoot.transform.SetParent(transform, false);

            Material carbonMat = F1Palette.CreateCarbonMaterial();
            Material alcantaraMat = F1Palette.GetOrCreateMaterial("Mat_Alcantara", F1Palette.AlcantaraGrip, 0.05f, 0.18f);
            Material redLiveryMat = F1Palette.GetOrCreateMaterial("Mat_PlayerRed", F1Palette.RacingRed, 0.38f, 0.85f);
            Material goldMat = F1Palette.GetOrCreateMaterial("Mat_PlayerGold", F1Palette.ModenaYellow, 0.45f, 0.80f);
            Material chromeMat = F1Palette.GetOrCreateMaterial("Mat_Chrome", new Color(0.82f, 0.85f, 0.88f), 0.92f, 0.90f);

            haloMaterial = F1Palette.GetOrCreateMaterial(
                "Mat_PlayerHaloTitanium",
                F1Palette.TitaniumHalo,
                0.55f,
                0.65f,
                null,
                null,
                transparent: true);

            // 3. Cockpit Surround Coaming, Side Bolsters & Glare Shield
            CreateCockpitPart(PrimitiveType.Cube, "Cockpit_Wall_Left", cockpitRoot.transform,
                new Vector3(-0.31f, 0.36f, 0.08f), new Vector3(0.0f, 3.5f, 0.0f), new Vector3(0.09f, 0.28f, 0.95f), carbonMat);
            CreateCockpitPart(PrimitiveType.Cube, "Cockpit_Wall_Right", cockpitRoot.transform,
                new Vector3(0.31f, 0.36f, 0.08f), new Vector3(0.0f, -3.5f, 0.0f), new Vector3(0.09f, 0.28f, 0.95f), carbonMat);

            // Red Livery Coaming Rim Trim
            CreateCockpitPart(PrimitiveType.Cube, "Cockpit_Rim_Left", cockpitRoot.transform,
                new Vector3(-0.33f, 0.50f, 0.22f), new Vector3(4.0f, 5.0f, 0.0f), new Vector3(0.065f, 0.045f, 0.82f), redLiveryMat);
            CreateCockpitPart(PrimitiveType.Cube, "Cockpit_Rim_Right", cockpitRoot.transform,
                new Vector3(0.33f, 0.50f, 0.22f), new Vector3(4.0f, -5.0f, 0.0f), new Vector3(0.065f, 0.045f, 0.82f), redLiveryMat);

            // Curved Dashboard Cowl & Windscreen Aero Lip (lowered so steering wheel LCD and forward sightline are 100% unobstructed)
            CreateCockpitPart(PrimitiveType.Cylinder, "Dashboard_GlareShield", cockpitRoot.transform,
                new Vector3(0.0f, 0.36f, 0.38f), new Vector3(84.0f, 0.0f, 0.0f), new Vector3(0.48f, 0.12f, 0.12f), alcantaraMat);
            CreateCockpitPart(PrimitiveType.Cube, "Aero_Windscreen_Deflector", cockpitRoot.transform,
                new Vector3(0.0f, 0.46f, 0.56f), new Vector3(-28.0f, 0.0f, 0.0f), new Vector3(0.32f, 0.02f, 0.01f), carbonMat);

            // Nose Pitot Tube & High-Speed Camera Pods
            CreateCockpitPart(PrimitiveType.Cylinder, "Aero_Pitot_Tube", cockpitRoot.transform,
                new Vector3(0.0f, 0.46f, 0.92f), new Vector3(0.0f, 0.0f, 0.0f), new Vector3(0.012f, 0.06f, 0.012f), chromeMat);

            // 4. Sculpted Curved Titanium Halo Safety Structure
            GameObject haloObj = new GameObject("Halo_Titanium_Structure");
            haloObj.transform.SetParent(cockpitRoot.transform, false);
            haloParts.Clear();

            // Slender Aerodynamic Center Pillar (thin blade-like aero profile placed above the steering wheel)
            GameObject centerPillar = CreateCockpitPart(PrimitiveType.Cube, "Halo_Center_Pillar", haloObj.transform,
                new Vector3(0.0f, 0.68f, 0.40f), new Vector3(20.0f, 0.0f, 0.0f), new Vector3(0.016f, 0.22f, 0.032f), haloMaterial);
            haloParts.Add(centerPillar.GetComponent<Renderer>());
            haloRenderer = centerPillar.GetComponent<Renderer>();

            // V-Transition Gusset at top of pillar
            for (int side = -1; side <= 1; side += 2)
            {
                GameObject vGusset = CreateCockpitPart(PrimitiveType.Cube, $"Halo_VGusset_{(side < 0 ? "L" : "R")}", haloObj.transform,
                    new Vector3(side * 0.055f, 0.80f, 0.33f), new Vector3(8.0f, side * 32.0f, side * -12.0f),
                    new Vector3(0.028f, 0.028f, 0.16f), haloMaterial);
                haloParts.Add(vGusset.GetComponent<Renderer>());
            }

            // Swept Curved Upper Halo Arch (10 smooth segments framing the top of view)
            int archSegs = 10;
            for (int s = 0; s < archSegs; s++)
            {
                float t0 = (s / (float)archSegs) * Mathf.PI - Mathf.PI * 0.5f;
                float t1 = ((s + 1) / (float)archSegs) * Mathf.PI - Mathf.PI * 0.5f;

                Vector3 p0 = new Vector3(Mathf.Sin(t0) * 0.35f, 0.835f - Mathf.Abs(t0) * 0.055f, 0.32f - (1.0f - Mathf.Cos(t0)) * 0.48f);
                Vector3 p1 = new Vector3(Mathf.Sin(t1) * 0.35f, 0.835f - Mathf.Abs(t1) * 0.055f, 0.32f - (1.0f - Mathf.Cos(t1)) * 0.48f);
                Vector3 mid = (p0 + p1) * 0.5f;
                Vector3 dir = (p1 - p0);
                float segLen = dir.magnitude;

                GameObject archSeg = CreateCockpitPart(PrimitiveType.Cube, $"Halo_Arch_Seg_{s}", haloObj.transform,
                    mid, Vector3.zero, new Vector3(0.034f, 0.028f, segLen * 1.08f), haloMaterial);
                if (dir.sqrMagnitude > 1e-6f)
                    archSeg.transform.localRotation = Quaternion.LookRotation(dir.normalized, Vector3.up);
                haloParts.Add(archSeg.GetComponent<Renderer>());
            }

            // 5. High-Craft F1 Butterfly Steering Wheel (Cleanly positioned in lower view at Y=0.42, Z=0.24)
            GameObject wheelColumn = new GameObject("SteeringWheel_Column");
            wheelColumn.transform.SetParent(cockpitRoot.transform, false);
            wheelColumn.transform.localPosition = new Vector3(0.0f, 0.42f, 0.24f);
            wheelColumn.transform.localRotation = Quaternion.Euler(columnRakeDeg, 0.0f, 0.0f);

            // Steering Column Shaft
            CreateCockpitPart(PrimitiveType.Cylinder, "Steering_Shaft", wheelColumn.transform,
                new Vector3(0.0f, 0.0f, 0.12f), new Vector3(90.0f, 0.0f, 0.0f), new Vector3(0.045f, 0.12f, 0.045f), carbonMat);

            GameObject wheelBody = new GameObject("SteeringWheel_Body");
            wheelBody.transform.SetParent(wheelColumn.transform, false);
            steeringWheelTransform = wheelBody.transform;

            // Sculpted Carbon Main Chassis Plate & Upper Shift Brow
            CreateCockpitPart(PrimitiveType.Cube, "SW_Center_Plate", wheelBody.transform,
                new Vector3(0.0f, 0.005f, 0.0f), Vector3.zero, new Vector3(0.23f, 0.125f, 0.018f), carbonMat);
            CreateCockpitPart(PrimitiveType.Cube, "SW_Upper_Shift_Brow", wheelBody.transform,
                new Vector3(0.0f, 0.074f, -0.003f), Vector3.zero, new Vector3(0.20f, 0.024f, 0.022f), carbonMat);
            CreateCockpitPart(PrimitiveType.Cube, "SW_Lower_Spoke_Bridge", wheelBody.transform,
                new Vector3(0.0f, -0.068f, 0.0f), Vector3.zero, new Vector3(0.16f, 0.022f, 0.018f), carbonMat);

            // Sculpted Butterfly Alcantara Grips (Left & Right)
            for (int side = -1; side <= 1; side += 2)
            {
                string sTag = side < 0 ? "L" : "R";
                // Main vertical grip contour
                CreateCockpitPart(PrimitiveType.Cylinder, $"SW_Grip_Main_{sTag}", wheelBody.transform,
                    new Vector3(side * 0.128f, -0.005f, -0.004f), new Vector3(0.0f, 0.0f, side * -6.0f),
                    new Vector3(0.032f, 0.072f, 0.034f), alcantaraMat);
                // Upper horn flare
                CreateCockpitPart(PrimitiveType.Cube, $"SW_Grip_UpperHorn_{sTag}", wheelBody.transform,
                    new Vector3(side * 0.112f, 0.062f, -0.004f), new Vector3(0.0f, 0.0f, side * -28.0f),
                    new Vector3(0.038f, 0.026f, 0.026f), alcantaraMat);
                // Lower horn flare
                CreateCockpitPart(PrimitiveType.Cube, $"SW_Grip_LowerHorn_{sTag}", wheelBody.transform,
                    new Vector3(side * 0.105f, -0.065f, -0.004f), new Vector3(0.0f, 0.0f, side * 32.0f),
                    new Vector3(0.045f, 0.024f, 0.026f), alcantaraMat);
                // Carbon Gear Shift Paddles behind the wheel
                CreateCockpitPart(PrimitiveType.Cube, $"SW_Shift_Paddle_{sTag}", wheelBody.transform,
                    new Vector3(side * 0.095f, 0.015f, 0.022f), new Vector3(0.0f, side * 10.0f, 0.0f),
                    new Vector3(0.042f, 0.075f, 0.006f), carbonMat);
            }

            // 15 3D Emissive Shift-Light LEDs across the Upper Brow
            Renderer[] ledRenderers = new Renderer[15];
            for (int i = 0; i < 15; i++)
            {
                float xLed = Mathf.Lerp(-0.082f, 0.082f, i / 14.0f);
                GameObject ledObj = CreateCockpitPart(PrimitiveType.Sphere, $"SW_Shift_LED_{i:D2}", wheelBody.transform,
                    new Vector3(xLed, 0.075f, -0.015f), Vector3.zero, new Vector3(0.0085f, 0.0085f, 0.006f), carbonMat);
                ledRenderers[i] = ledObj.GetComponent<Renderer>();
            }

            // 6 Color-Coded Rotary Encoder Dials + 8 Push Buttons
            Color[] dialColors = new Color[]
            {
                F1Palette.RacingRed, F1Palette.ModenaYellow, F1Palette.TelemetryCyan,
                F1Palette.DrsEmerald, new Color(1.0f, 0.45f, 0.0f), Color.white
            };
            Vector3[] dialPositions = new Vector3[]
            {
                new Vector3(-0.048f, -0.042f, -0.011f),
                new Vector3( 0.000f, -0.046f, -0.011f),
                new Vector3( 0.048f, -0.042f, -0.011f),
                new Vector3(-0.086f,  0.005f, -0.011f),
                new Vector3( 0.086f,  0.005f, -0.011f),
                new Vector3( 0.000f, -0.015f, -0.011f)
            };
            for (int d = 0; d < dialPositions.Length; d++)
            {
                Material dialMat = F1Palette.GetOrCreateMaterial($"Mat_SW_Dial_{d}", dialColors[d], 0.3f, 0.7f);
                CreateCockpitPart(PrimitiveType.Cylinder, $"SW_Rotary_Dial_{d}", wheelBody.transform,
                    dialPositions[d], new Vector3(90.0f, 0.0f, 0.0f), new Vector3(0.022f, 0.006f, 0.022f), dialMat);
            }

            // 8 Tactile Push Buttons around Left & Right Bezels
            Vector3[] btnPositions = new Vector3[]
            {
                new Vector3(-0.088f,  0.048f, -0.011f), // DRS (Green)
                new Vector3(-0.088f,  0.028f, -0.011f), // OVERTAKE (Red)
                new Vector3(-0.084f, -0.022f, -0.011f), // NEUTRAL (White)
                new Vector3(-0.080f, -0.042f, -0.011f), // PIT (Yellow)
                new Vector3( 0.088f,  0.048f, -0.011f), // RADIO (Cyan)
                new Vector3( 0.088f,  0.028f, -0.011f), // ACK (Orange)
                new Vector3( 0.084f, -0.022f, -0.011f), // +10 (Blue)
                new Vector3( 0.080f, -0.042f, -0.011f)  // -1 (Red)
            };
            Color[] btnColors = new Color[]
            {
                F1Palette.DrsEmerald, F1Palette.RacingRed, Color.white, F1Palette.ModenaYellow,
                F1Palette.TelemetryCyan, new Color(1.0f, 0.5f, 0.0f), F1Palette.ShiftBlue, F1Palette.RacingRed
            };
            for (int b = 0; b < btnPositions.Length; b++)
            {
                Material btnMat = F1Palette.GetOrCreateMaterial($"Mat_SW_Btn_{b}", btnColors[b], 0.2f, 0.8f, null, btnColors[b] * 0.6f);
                CreateCockpitPart(PrimitiveType.Cylinder, $"SW_Push_Button_{b}", wheelBody.transform,
                    btnPositions[b], new Vector3(90.0f, 0.0f, 0.0f), new Vector3(0.013f, 0.005f, 0.013f), btnMat);
            }

            // McLaren PCU-8D Live LCD Screen Quad
            GameObject lcdQuad = CreateCockpitPart(PrimitiveType.Cube, "PCU8D_LCD_Display", wheelBody.transform,
                new Vector3(0.0f, 0.024f, -0.010f), new Vector3(0.0f, 180.0f, 0.0f), new Vector3(0.118f, 0.068f, 0.004f), carbonMat);

            TelemetryLCD lcdComp = lcdQuad.AddComponent<TelemetryLCD>();
            lcdComp.vehicle = vehicle;
            lcdComp.lcdQuadRenderer = lcdQuad.GetComponent<Renderer>();
            lcdComp.shiftLedRenderers = ledRenderers;
            lcdComp.InitializeLcdDisplay();

            // 6. Aerodynamic Swan-Neck Wing Mirrors with Reflective Glass
            for (int side = -1; side <= 1; side += 2)
            {
                string sTag = side < 0 ? "L" : "R";
                // Mirror Stalk
                CreateCockpitPart(PrimitiveType.Cube, $"Mirror_Stalk_{sTag}", cockpitRoot.transform,
                    new Vector3(side * 0.42f, 0.49f, 0.34f), new Vector3(0.0f, 0.0f, side * -18.0f),
                    new Vector3(0.14f, 0.018f, 0.04f), carbonMat);
                // Mirror Aero Housing
                CreateCockpitPart(PrimitiveType.Cube, $"Mirror_Housing_{sTag}", cockpitRoot.transform,
                    new Vector3(side * 0.50f, 0.52f, 0.34f), new Vector3(0.0f, side * 12.0f, 0.0f),
                    new Vector3(0.15f, 0.068f, 0.055f), redLiveryMat);
                // Reflective Mirror Glass Face
                CreateCockpitPart(PrimitiveType.Cube, $"Mirror_Glass_{sTag}", cockpitRoot.transform,
                    new Vector3(side * 0.50f, 0.52f, 0.312f), new Vector3(0.0f, side * 12.0f, 0.0f),
                    new Vector3(0.135f, 0.054f, 0.006f), chromeMat);
            }

            // 7. 3D Driver Arms & Gloves (Fireproof Nomex Suit Sleeves & Biometric Racing Gloves)
            Material suitMat = F1Palette.GetOrCreateMaterial("Mat_DriverSuit", new Color(0.92f, 0.92f, 0.95f), 0.1f, 0.25f);
            Material suitStripeMat = F1Palette.GetOrCreateMaterial("Mat_SuitStripe", F1Palette.RacingRed, 0.2f, 0.4f);
            Material gloveMat = F1Palette.GetOrCreateMaterial("Mat_DriverGlove", new Color(0.12f, 0.12f, 0.14f), 0.15f, 0.35f);
            Material gripPadMat = F1Palette.GetOrCreateMaterial("Mat_GripPad", F1Palette.RacingRed, 0.3f, 0.6f);

            GameObject armsRoot = new GameObject("Driver_Arms_Assembly");
            armsRoot.transform.SetParent(cockpitRoot.transform, false);

            // Left & Right Shoulder Anchors inside cockpit bolsters
            GameObject lShoulder = new GameObject("Anchor_Shoulder_L");
            lShoulder.transform.SetParent(armsRoot.transform, false);
            lShoulder.transform.localPosition = new Vector3(-0.24f, 0.30f, -0.15f);
            leftShoulderAnchor = lShoulder.transform;

            GameObject rShoulder = new GameObject("Anchor_Shoulder_R");
            rShoulder.transform.SetParent(armsRoot.transform, false);
            rShoulder.transform.localPosition = new Vector3(0.24f, 0.30f, -0.15f);
            rightShoulderAnchor = rShoulder.transform;

            // Gloves gripping wheel body at 9 o'clock and 3 o'clock (child of wheelBody so they turn 1:1 with wheel)
            GameObject lGlove = new GameObject("Driver_Glove_L");
            lGlove.transform.SetParent(wheelBody.transform, false);
            lGlove.transform.localPosition = new Vector3(-0.128f, -0.005f, -0.012f);
            lGlove.transform.localRotation = Quaternion.Euler(0.0f, 0.0f, 8.0f);
            leftGloveTransform = lGlove.transform;

            GameObject rGlove = new GameObject("Driver_Glove_R");
            rGlove.transform.SetParent(wheelBody.transform, false);
            rGlove.transform.localPosition = new Vector3(0.128f, -0.005f, -0.012f);
            rGlove.transform.localRotation = Quaternion.Euler(0.0f, 0.0f, -8.0f);
            rightGloveTransform = rGlove.transform;

            // Detail glove geometry for Left & Right hands
            for (int side = -1; side <= 1; side += 2)
            {
                Transform gParent = (side < 0) ? lGlove.transform : rGlove.transform;

                // Glove back/palm
                CreateCockpitPart(PrimitiveType.Cube, "Glove_Palm", gParent,
                    Vector3.zero, Vector3.zero, new Vector3(0.046f, 0.068f, 0.038f), gloveMat);

                // High-grip silicone palm patch
                CreateCockpitPart(PrimitiveType.Cube, "Glove_Silicone_Grip", gParent,
                    new Vector3(0.0f, 0.0f, 0.018f), Vector3.zero, new Vector3(0.038f, 0.052f, 0.006f), gripPadMat);

                // Thumb resting on front face
                CreateCockpitPart(PrimitiveType.Cylinder, "Glove_Thumb", gParent,
                    new Vector3(side * -0.018f, 0.026f, -0.012f), new Vector3(15.0f, 0.0f, side * -25.0f),
                    new Vector3(0.016f, 0.024f, 0.016f), gloveMat);

                // Fingers wrapped around grip
                CreateCockpitPart(PrimitiveType.Cube, "Glove_Fingers_Wrap", gParent,
                    new Vector3(side * 0.014f, -0.005f, 0.022f), Vector3.zero,
                    new Vector3(0.024f, 0.058f, 0.022f), gloveMat);

                // Glove wrist cuff
                CreateCockpitPart(PrimitiveType.Cylinder, "Glove_Wrist_Cuff", gParent,
                    new Vector3(0.0f, -0.042f, -0.008f), new Vector3(35.0f, 0.0f, 0.0f),
                    new Vector3(0.048f, 0.018f, 0.048f), suitStripeMat);
            }

            // Driver Forearm Sleeves spanning from shoulders to gloves
            GameObject lArm = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            lArm.name = "Driver_Arm_Left";
            lArm.transform.SetParent(armsRoot.transform, false);
            Destroy(lArm.GetComponent<Collider>());
            lArm.GetComponent<Renderer>().sharedMaterial = suitMat;
            leftArmTransform = lArm.transform;

            GameObject rArm = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            rArm.name = "Driver_Arm_Right";
            rArm.transform.SetParent(armsRoot.transform, false);
            Destroy(rArm.GetComponent<Collider>());
            rArm.GetComponent<Renderer>().sharedMaterial = suitMat;
            rightArmTransform = rArm.transform;

            UpdateDriverArmsKinematics();
        }

        private GameObject CreateCockpitPart(
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

            if (steeringWheelTransform != null)
            {
                float steerRatio = vehicle.config.steerRatio;
                float roadWheelAngleRad = vehicle.state.steeringAngle;
                float wheelRotationDeg = Mathf.Clamp(roadWheelAngleRad * Mathf.Rad2Deg * steerRatio, -180.0f, 180.0f);

                steeringWheelTransform.localRotation = Quaternion.Euler(0.0f, 0.0f, -wheelRotationDeg);
            }

            UpdateDriverArmsKinematics();
        }

        private void UpdateDriverArmsKinematics()
        {
            if (leftShoulderAnchor != null && leftGloveTransform != null && leftArmTransform != null)
            {
                AlignArmToHand(leftShoulderAnchor, leftGloveTransform, leftArmTransform);
            }
            if (rightShoulderAnchor != null && rightGloveTransform != null && rightArmTransform != null)
            {
                AlignArmToHand(rightShoulderAnchor, rightGloveTransform, rightArmTransform);
            }
        }

        private void AlignArmToHand(Transform shoulder, Transform glove, Transform arm)
        {
            Vector3 p0 = shoulder.position;
            Vector3 p1 = glove.position;
            Vector3 dir = p1 - p0;
            float dist = dir.magnitude;
            if (dist < 0.01f) return;

            arm.position = (p0 + p1) * 0.5f;
            arm.rotation = Quaternion.LookRotation(dir.normalized, transform.up) * Quaternion.Euler(90.0f, 0.0f, 0.0f);
            arm.localScale = new Vector3(0.072f, dist * 0.5f, 0.072f);
        }

        public void CycleHaloOpacity()
        {
            if (haloOpacity > 0.8f)
                haloOpacity = 0.35f;
            else if (haloOpacity > 0.25f)
                haloOpacity = 0.12f;
            else
                haloOpacity = 1.0f;

            if (haloMaterial != null)
            {
                Color col = haloMaterial.color;
                col.a = haloOpacity;
                haloMaterial.color = col;
                if (haloMaterial.HasProperty("_BaseColor"))
                    haloMaterial.SetColor("_BaseColor", col);
            }
        }
    }
}
