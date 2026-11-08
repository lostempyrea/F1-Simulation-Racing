using System.Collections.Generic;
using UnityEngine;
using F1Sim.Core;

namespace F1Sim.Track
{
    public class SceneryBuilder : MonoBehaviour
    {
        [Header("Scenery Materials (Auto-generated from F1Palette if null)")]
        public Material barrierMaterial;
        public Material gravelMaterial;
        public Material grandstandMaterial;
        public Material treeLeavesMaterial;
        public Material treeTrunkMaterial;

        public void BuildScenery(TrackSpline spline)
        {
            if (spline == null) return;

            Transform old = transform.Find("Scenery_Environment");
            if (old != null) Destroy(old.gameObject);

            if (barrierMaterial == null) barrierMaterial = F1Palette.GetOrCreateMaterial("Mat_ArmcoSteel", F1Palette.ArmcoSteel, 0.70f, 0.55f);
            if (gravelMaterial == null) gravelMaterial = F1Palette.CreateGravelMaterial();
            if (grandstandMaterial == null) grandstandMaterial = F1Palette.CreateGrandstandCrowdMaterial();
            if (treeLeavesMaterial == null) treeLeavesMaterial = F1Palette.GetOrCreateMaterial("Mat_PineCanopyDark", F1Palette.PineCanopyDark, 0.0f, 0.18f);
            if (treeTrunkMaterial == null) treeTrunkMaterial = F1Palette.GetOrCreateMaterial("Mat_TreeBark", F1Palette.TreeBark, 0.0f, 0.12f);

            GameObject sceneryRoot = new GameObject("Scenery_Environment");
            sceneryRoot.transform.SetParent(transform, false);

            BuildParklandTerrain(sceneryRoot.transform);
            BuildElevatedTrackSupports(spline, sceneryRoot.transform);
            BuildArmcoAndTecproBarriers(spline, sceneryRoot.transform);
            BuildGrandstandsAndPaddock(spline, sceneryRoot.transform);
            BuildOverpassBridges(spline, sceneryRoot.transform);
            BuildGravelTrapsAndBrakeMarkers(spline, sceneryRoot.transform);
            BuildMonzaParkForest(spline, sceneryRoot.transform);
            BuildDistantAlpineMountains(sceneryRoot.transform);
        }

        private void BuildParklandTerrain(Transform parent)
        {
            GameObject ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "Monza_Parkland_Ground";
            ground.transform.SetParent(parent, false);
            ground.transform.position = new Vector3(250.0f, -0.15f, 1100.0f);
            ground.transform.localScale = new Vector3(8000.0f, 0.20f, 8000.0f);
            ground.GetComponent<Renderer>().sharedMaterial = F1Palette.CreateGrassMaterial();
        }

        private void BuildElevatedTrackSupports(TrackSpline spline, Transform parent)
        {
            GameObject supportGroup = new GameObject("Elevated_Track_Supports");
            supportGroup.transform.SetParent(parent, false);

            Material pillarMat = F1Palette.GetOrCreateMaterial("Mat_BridgePillars", new Color(0.48f, 0.52f, 0.56f), 0.15f, 0.40f);
            Material earthMat = F1Palette.GetOrCreateMaterial("Mat_EarthEmbankment", F1Palette.ParkGrassDark, 0.05f, 0.20f);

            float length = spline.Length;
            float step = 15.0f;
            int count = Mathf.RoundToInt(length / step);

            for (int i = 0; i < count; i++)
            {
                float s = i * step;
                Vector3 center = spline.GetPoint(s);

                // If track is elevated above 0.75m (e.g. Lesmo curves up to Y=5.5m)
                if (center.y > 0.75f)
                {
                    spline.GetFrenetFrame(s, out Vector3 tan, out Vector3 lat, out Vector3 norm);
                    float trackW = MonzaTrackData.TrackWidth;
                    float groundY = -0.15f;
                    float elevH = center.y - groundY;

                    // 1. Solid embankment base underneath track ribbon
                    GameObject embankment = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    embankment.name = $"Embankment_s{s:F0}";
                    embankment.transform.SetParent(supportGroup.transform, false);
                    embankment.transform.position = new Vector3(center.x, groundY + elevH * 0.5f, center.z);
                    embankment.transform.rotation = Quaternion.LookRotation(tan, Vector3.up);
                    embankment.transform.localScale = new Vector3(trackW + 4.5f, elevH, step * 1.05f);
                    embankment.GetComponent<Renderer>().sharedMaterial = earthMat;
                    Collider cEmb = embankment.GetComponent<Collider>();
                    if (cEmb != null) Destroy(cEmb);

                    // 2. Concrete abutment pillars every 30m for sections > 2.0m high
                    if (center.y > 2.0f && (i % 2 == 0))
                    {
                        for (int side = -1; side <= 1; side += 2)
                        {
                            GameObject pillar = GameObject.CreatePrimitive(PrimitiveType.Cube);
                            pillar.name = $"Concrete_Pillar_{(side < 0 ? "L" : "R")}_s{s:F0}";
                            pillar.transform.SetParent(supportGroup.transform, false);
                            Vector3 pillarPos = center + lat * (side * (trackW * 0.5f + 0.8f));
                            pillarPos.y = groundY + elevH * 0.5f;
                            pillar.transform.position = pillarPos;
                            pillar.transform.rotation = Quaternion.LookRotation(tan, Vector3.up);
                            pillar.transform.localScale = new Vector3(1.2f, elevH, 1.2f);
                            pillar.GetComponent<Renderer>().sharedMaterial = pillarMat;
                            Collider cPil = pillar.GetComponent<Collider>();
                            if (cPil != null) Destroy(cPil);
                        }
                    }
                }
            }
        }

        private void BuildArmcoAndTecproBarriers(TrackSpline spline, Transform parent)
        {
            GameObject barrierObj = new GameObject("Armco_Tecpro_Barriers");
            barrierObj.transform.SetParent(parent, false);

            Material tecproRed = F1Palette.GetOrCreateMaterial("Mat_TecproRed", F1Palette.RacingRed, 0.15f, 0.50f);
            Material tecproWhite = F1Palette.GetOrCreateMaterial("Mat_TecproWhite", F1Palette.KerbWhite, 0.15f, 0.50f);
            Material tecproBlue = F1Palette.GetOrCreateMaterial("Mat_TecproBlue", F1Palette.TecproBlue, 0.15f, 0.50f);

            float length = spline.Length;
            float step = 12.0f;
            int count = Mathf.RoundToInt(length / step);
            float halfWidth = MonzaTrackData.TrackWidth * 0.5f + 2.8f;

            // Precompute dense track samples (every 5m) for non-adjacent safety clearance checks
            int checkSamples = Mathf.RoundToInt(length / 5.0f);
            Vector3[] samplePoints = new Vector3[checkSamples];
            float[] sampleS = new float[checkSamples];
            for (int k = 0; k < checkSamples; k++)
            {
                sampleS[k] = (k / (float)checkSamples) * length;
                samplePoints[k] = spline.GetPoint(sampleS[k]);
            }

            for (int i = 0; i < count; i++)
            {
                float s = i * step;
                Vector3 center = spline.GetPoint(s);
                spline.GetFrenetFrame(s, out Vector3 tan, out Vector3 lat, out Vector3 norm);
                Quaternion rot = Quaternion.LookRotation(tan, norm);

                bool isChicaneZone = (s > 1180f && s < 1330f) || (s > 2150f && s < 2320f) || (s > 4150f && s < 4550f);
                Material mat = barrierMaterial;
                float barrierH = 0.92f;
                float barrierW = 0.22f;

                if (isChicaneZone)
                {
                    mat = (i % 3 == 0) ? tecproRed : ((i % 3 == 1) ? tecproWhite : tecproBlue);
                    barrierH = 1.05f;
                    barrierW = 0.55f;
                }

                // Left & Right continuous barrier rails
                for (int side = -1; side <= 1; side += 2)
                {
                    Vector3 railPos = center + lat * (side * halfWidth) + norm * (barrierH * 0.5f);

                    // Safety check: verify this barrier does not encroach into ANY non-adjacent track ribbon (< 7.0m)
                    bool encroaches = false;
                    float safeDistSq = 7.0f * 7.0f;
                    for (int k = 0; k < checkSamples; k++)
                    {
                        float sDiff = Mathf.Abs(sampleS[k] - s);
                        if (sDiff > length * 0.5f) sDiff = length - sDiff;
                        if (sDiff > 35.0f)
                        {
                            if ((samplePoints[k] - railPos).sqrMagnitude < safeDistSq)
                            {
                                encroaches = true;
                                break;
                            }
                        }
                    }
                    if (encroaches)
                    {
                        continue; // Encroaches on another track ribbon, skip to keep track 100% clear
                    }

                    GameObject rail = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    rail.name = $"Barrier_{i}_{(side < 0 ? "L" : "R")}";
                    rail.transform.SetParent(barrierObj.transform, false);
                    rail.transform.position = railPos;
                    rail.transform.rotation = rot;
                    rail.transform.localScale = new Vector3(barrierW, barrierH, step * 1.02f);
                    rail.GetComponent<Renderer>().sharedMaterial = mat;
                }
            }
        }

        private void BuildGrandstandsAndPaddock(TrackSpline spline, Transform parent)
        {
            GameObject standsObj = new GameObject("Covered_Grandstands");
            standsObj.transform.SetParent(parent, false);

            Material roofMat = F1Palette.GetOrCreateMaterial("Mat_StandRoof", new Color(0.92f, 0.94f, 0.96f), 0.30f, 0.65f);
            Material structureMat = F1Palette.GetOrCreateMaterial("Mat_StandSteel", new Color(0.20f, 0.22f, 0.26f), 0.60f, 0.50f);
            Material bannerRed = F1Palette.GetOrCreateMaterial("Mat_StandBannerRed", F1Palette.RacingRed, 0.2f, 0.7f);
            Material pitGlassMat = F1Palette.GetOrCreateMaterial("Mat_PitBuildingGlass", new Color(0.14f, 0.24f, 0.36f), 0.85f, 0.92f);

            // Official Monza Grandstand Complexes (numbered 1..35 per official Monza circuit map)
            // Main Straight Left: 1, 2, 3, 4, 5, 8A/B (s = 60..900)
            // Rettifilo Chicane: 8B, 9, 10 (s = 1250)
            // Roggia Chicane: 11, 12, 13 (s = 2220)
            // Lesmo 1 & 2: 14, 16, 17, 18 (s = 2480)
            // Serraglio: 21, 22 (s = 3200)
            // Variante Ascari: 23, 24, 25, 26, 27, 28 (s = 4250..4420)
            // Rettifilo Parabolica: 29, 30 (s = 4850)
            // Parabolica Alboreto: 32, 33, 34, 35 (s = 5450, 5650)
            float[] standS     = new float[] { 60.0f, 200.0f, 400.0f, 650.0f, 900.0f, 1250.0f, 2220.0f, 2480.0f, 3200.0f, 4250.0f, 4420.0f, 4850.0f, 5450.0f, 5650.0f };
            float[] standSides = new float[] { -1.0f,  -1.0f,  -1.0f,  -1.0f,  -1.0f,   -1.0f,   -1.0f,   -1.0f,    1.0f,   -1.0f,    1.0f,   -1.0f,    1.0f,    1.0f };

            for (int i = 0; i < standS.Length; i++)
            {
                float s = standS[i];
                float side = standSides[i];
                Vector3 pos = spline.GetPoint(s);
                spline.GetFrenetFrame(s, out Vector3 tan, out Vector3 lat, out Vector3 norm);
                Quaternion rot = Quaternion.LookRotation(tan, norm);

                GameObject standGroup = new GameObject($"Grandstand_Complex_s{s:F0}");
                standGroup.transform.SetParent(standsObj.transform, false);
                standGroup.transform.position = pos + lat * (side * 20.0f);
                standGroup.transform.rotation = rot;

                // Sloped spectator seating bank facing the track
                GameObject seating = GameObject.CreatePrimitive(PrimitiveType.Cube);
                seating.name = "Spectator_Seating_Tier";
                seating.transform.SetParent(standGroup.transform, false);
                seating.transform.localPosition = new Vector3(0.0f, 4.2f, 0.0f);
                seating.transform.localRotation = Quaternion.Euler(0.0f, 0.0f, side * 24.0f);
                seating.transform.localScale = new Vector3(11.0f, 2.2f, 56.0f);
                Destroy(seating.GetComponent<Collider>());
                seating.GetComponent<Renderer>().sharedMaterial = grandstandMaterial;

                // Rear concrete/steel structural wall
                GameObject backWall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                backWall.name = "Stand_Backwall";
                backWall.transform.SetParent(standGroup.transform, false);
                backWall.transform.localPosition = new Vector3(side * 5.2f, 4.8f, 0.0f);
                backWall.transform.localScale = new Vector3(1.6f, 9.6f, 56.0f);
                backWall.GetComponent<Renderer>().sharedMaterial = structureMat;

                // Cantilevered White Canopy Roof
                GameObject roof = GameObject.CreatePrimitive(PrimitiveType.Cube);
                roof.name = "Cantilever_Roof";
                roof.transform.SetParent(standGroup.transform, false);
                roof.transform.localPosition = new Vector3(-side * 0.5f, 10.2f, 0.0f);
                roof.transform.localRotation = Quaternion.Euler(0.0f, 0.0f, side * -6.0f);
                roof.transform.localScale = new Vector3(13.5f, 0.45f, 58.0f);
                Destroy(roof.GetComponent<Collider>());
                roof.GetComponent<Renderer>().sharedMaterial = roofMat;

                // Red Championship Fascia Frieze
                GameObject fascia = GameObject.CreatePrimitive(PrimitiveType.Cube);
                fascia.name = "Roof_Fascia_Banner";
                fascia.transform.SetParent(standGroup.transform, false);
                fascia.transform.localPosition = new Vector3(-side * 6.8f, 9.8f, 0.0f);
                fascia.transform.localScale = new Vector3(0.35f, 0.95f, 58.0f);
                Destroy(fascia.GetComponent<Collider>());
                fascia.GetComponent<Renderer>().sharedMaterial = bannerRed;
            }

            // Pit Lane Garages & Hospitality Building along Right Side of Main Straight (s = 40..360m)
            for (int p = 0; p < 4; p++)
            {
                float sPit = 50.0f + p * 75.0f;
                Vector3 pos = spline.GetPoint(sPit);
                spline.GetFrenetFrame(sPit, out Vector3 tan, out Vector3 lat, out Vector3 norm);

                GameObject pitBldg = GameObject.CreatePrimitive(PrimitiveType.Cube);
                pitBldg.name = $"F1_Pit_Garage_Block_{p + 1}";
                pitBldg.transform.SetParent(standsObj.transform, false);
                pitBldg.transform.position = pos + lat * 16.5f + norm * 4.5f;
                pitBldg.transform.rotation = Quaternion.LookRotation(tan, norm);
                pitBldg.transform.localScale = new Vector3(9.0f, 9.0f, 68.0f);
                Destroy(pitBldg.GetComponent<Collider>());
                pitBldg.GetComponent<Renderer>().sharedMaterial = pitGlassMat;
            }
        }

        private void BuildOverpassBridges(TrackSpline spline, Transform parent)
        {
            GameObject bridgesRoot = new GameObject("Sponsor_Overpass_Bridges");
            bridgesRoot.transform.SetParent(parent, false);

            Material yellowBridgeMat = F1Palette.GetOrCreateMaterial("Mat_BridgeYellow", F1Palette.ModenaYellow, 0.25f, 0.75f);
            Material redBridgeMat = F1Palette.GetOrCreateMaterial("Mat_BridgeRed", F1Palette.RacingRed, 0.25f, 0.75f);
            Material darkPillarMat = F1Palette.GetOrCreateMaterial("Mat_BridgePillar", F1Palette.CarbonSatin, 0.5f, 0.6f);
            Material historicConcreteMat = F1Palette.GetOrCreateMaterial("Mat_HistoricConcrete", new Color(0.62f, 0.62f, 0.60f), 0.1f, 0.35f);
            Material historicTarmacMat = F1Palette.GetOrCreateMaterial("Mat_HistoricTarmac", new Color(0.28f, 0.28f, 0.28f), 0.05f, 0.2f);

            float[] bridgeLocations = new float[] { 450.0f, 1650.0f, 3523.0f, 5100.0f };
            for (int i = 0; i < bridgeLocations.Length; i++)
            {
                float s = bridgeLocations[i];
                Vector3 center = spline.GetPoint(s);
                spline.GetFrenetFrame(s, out Vector3 tan, out Vector3 lat, out Vector3 norm);
                Quaternion rot = Quaternion.LookRotation(tan, norm);

                // Check if this is the Historic Sopraelevata High-Speed Oval Banking Overpass
                if (Mathf.Abs(s - 3523.0f) < 10.0f)
                {
                    GameObject historicBridge = new GameObject("Monza_Historic_Sopraelevata_Overpass");
                    historicBridge.transform.SetParent(bridgesRoot.transform, false);
                    historicBridge.transform.position = center;
                    historicBridge.transform.rotation = rot;

                    float clearHeight = 8.5f;
                    float bridgeSpan = MonzaTrackData.TrackWidth + 14.0f;

                    // Heavy historic reinforced concrete abutments
                    for (int side = -1; side <= 1; side += 2)
                    {
                        GameObject abutment = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        abutment.name = $"Historic_Abutment_{(side < 0 ? "L" : "R")}";
                        abutment.transform.SetParent(historicBridge.transform, false);
                        abutment.transform.localPosition = new Vector3(side * bridgeSpan * 0.5f, clearHeight * 0.45f, 0.0f);
                        abutment.transform.localScale = new Vector3(2.8f, clearHeight * 1.1f, 14.0f);
                        abutment.GetComponent<Renderer>().sharedMaterial = historicConcreteMat;
                    }

                    // Banked Oval Track deck cutting across above Serraglio at 28-degree inclination
                    GameObject bankedOval = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    bankedOval.name = "Sopraelevata_Banked_Oval_Deck";
                    bankedOval.transform.SetParent(historicBridge.transform, false);
                    bankedOval.transform.localPosition = new Vector3(0.0f, clearHeight + 1.2f, 0.0f);
                    bankedOval.transform.localRotation = Quaternion.Euler(0.0f, 90.0f, 26.0f); // 26-degree historic banking angle
                    bankedOval.transform.localScale = new Vector3(14.0f, 1.4f, bridgeSpan + 12.0f);
                    bankedOval.GetComponent<Renderer>().sharedMaterial = historicTarmacMat;
                    Destroy(bankedOval.GetComponent<Collider>());

                    // Historic concrete parapet safety wall
                    GameObject parapet = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    parapet.name = "Sopraelevata_Parapet_Wall";
                    parapet.transform.SetParent(historicBridge.transform, false);
                    parapet.transform.localPosition = new Vector3(0.0f, clearHeight + 2.4f, 0.0f);
                    parapet.transform.localRotation = Quaternion.Euler(0.0f, 90.0f, 26.0f);
                    parapet.transform.localScale = new Vector3(0.45f, 1.2f, bridgeSpan + 12.0f);
                    parapet.GetComponent<Renderer>().sharedMaterial = historicConcreteMat;
                    Destroy(parapet.GetComponent<Collider>());
                }
                else
                {
                    float span = MonzaTrackData.TrackWidth + 9.0f;
                    float h = 7.6f;

                    for (int side = -1; side <= 1; side += 2)
                    {
                        GameObject leg = GameObject.CreatePrimitive(PrimitiveType.Cube);
                        leg.name = $"Overpass_Pillar_{i}_{(side < 0 ? "L" : "R")}";
                        leg.transform.SetParent(bridgesRoot.transform, false);
                        leg.transform.position = center + lat * (side * span * 0.5f) + norm * (h * 0.5f);
                        leg.transform.rotation = rot;
                        leg.transform.localScale = new Vector3(0.9f, h, 2.2f);
                        leg.GetComponent<Renderer>().sharedMaterial = darkPillarMat;
                    }

                    GameObject deck = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    deck.name = $"Overpass_Sponsor_Span_{i}";
                    deck.transform.SetParent(bridgesRoot.transform, false);
                    deck.transform.position = center + norm * h;
                    deck.transform.rotation = rot;
                    deck.transform.localScale = new Vector3(span + 1.5f, 1.5f, 2.6f);
                    Destroy(deck.GetComponent<Collider>());
                    deck.GetComponent<Renderer>().sharedMaterial = (i % 2 == 0) ? yellowBridgeMat : redBridgeMat;
                }
            }
        }

        private void BuildGravelTrapsAndBrakeMarkers(TrackSpline spline, Transform parent)
        {
            GameObject gravelObj = new GameObject("Gravel_Traps_And_Brake_Boards");
            gravelObj.transform.SetParent(parent, false);

            Material boardMat = F1Palette.GetOrCreateMaterial("Mat_BrakeBoard", F1Palette.TrackLineWhite, 0.1f, 0.4f);

            float[] cornerEntries = new float[] { 1180.0f, 2160.0f, 2360.0f, 4180.0f, 5220.0f };
            foreach (float sCorner in cornerEntries)
            {
                Vector3 pos = spline.GetPoint(sCorner);
                spline.GetFrenetFrame(sCorner, out Vector3 tan, out Vector3 lat, out Vector3 norm);

                // Gravel Runoff Bed
                GameObject bed = GameObject.CreatePrimitive(PrimitiveType.Cube);
                bed.name = $"Gravel_Trap_s{sCorner:F0}";
                bed.transform.SetParent(gravelObj.transform, false);
                bed.transform.position = pos - lat * 12.0f + norm * 0.01f;
                bed.transform.rotation = Quaternion.LookRotation(tan, norm);
                bed.transform.localScale = new Vector3(11.0f, 0.04f, 65.0f);
                Destroy(bed.GetComponent<Collider>());
                bed.GetComponent<Renderer>().sharedMaterial = gravelMaterial;

                // 150m, 100m, 50m Braking Distance Boards on Track Left
                float[] dists = new float[] { 150.0f, 100.0f, 50.0f };
                foreach (float d in dists)
                {
                    float sBoard = Mathf.Repeat(sCorner - d, spline.Length);
                    Vector3 bPos = spline.GetPoint(sBoard);
                    spline.GetFrenetFrame(sBoard, out Vector3 bTan, out Vector3 bLat, out Vector3 bNorm);

                    GameObject board = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    board.name = $"Brake_Marker_{d:F0}m_s{sCorner:F0}";
                    board.transform.SetParent(gravelObj.transform, false);
                    board.transform.position = bPos - bLat * (MonzaTrackData.TrackWidth * 0.5f + 1.6f) + bNorm * 0.85f;
                    board.transform.rotation = Quaternion.LookRotation(bTan, bNorm);
                    board.transform.localScale = new Vector3(1.2f, 0.9f, 0.08f);
                    Destroy(board.GetComponent<Collider>());
                    board.GetComponent<Renderer>().sharedMaterial = boardMat;
                }
            }
        }

        private void BuildMonzaParkForest(TrackSpline spline, Transform parent)
        {
            GameObject treesObj = new GameObject("Royal_Monza_Park_Trees");
            treesObj.transform.SetParent(parent, false);

            Material canopyLightMat = F1Palette.GetOrCreateMaterial("Mat_PineCanopyLight", F1Palette.PineCanopyLight, 0.0f, 0.20f);

            float length = spline.Length;
            float treeSpacing = 18.0f;
            int numTrees = Mathf.RoundToInt(length / treeSpacing);
            System.Random rng = new System.Random(42);

            for (int i = 0; i < numTrees; i++)
            {
                float s = i * treeSpacing;
                Vector3 center = spline.GetPoint(s);
                spline.GetFrenetFrame(s, out _, out Vector3 lat, out Vector3 norm);

                for (int side = -1; side <= 1; side += 2)
                {
                    // Leave room for grandstands on main straight
                    float minOffset = (s < 600.0f) ? 32.0f : 15.5f;
                    float distFromCenter = minOffset + (float)rng.NextDouble() * 24.0f;
                    float scaleVar = 0.85f + (float)rng.NextDouble() * 0.55f;

                    Vector3 treePos = center + lat * (side * distFromCenter);
                    bool isUmbrellaPine = (i + (side > 0 ? 1 : 0)) % 3 != 0;

                    GameObject tree = new GameObject($"Monza_Tree_{i}_{side}");
                    tree.transform.SetParent(treesObj.transform, false);
                    tree.transform.position = treePos;
                    tree.transform.localScale = Vector3.one * scaleVar;

                    // Trunk
                    GameObject trunk = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    trunk.transform.SetParent(tree.transform, false);
                    trunk.transform.localPosition = new Vector3(0.0f, 3.2f, 0.0f);
                    trunk.transform.localScale = new Vector3(0.55f, 3.2f, 0.55f);
                    Destroy(trunk.GetComponent<Collider>());
                    trunk.GetComponent<Renderer>().sharedMaterial = treeTrunkMaterial;

                    if (isUmbrellaPine)
                    {
                        // Mediterranean Umbrella Pine: wide flat-topped layered canopy
                        GameObject lowerCrown = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        lowerCrown.transform.SetParent(tree.transform, false);
                        lowerCrown.transform.localPosition = new Vector3(0.0f, 6.4f, 0.0f);
                        lowerCrown.transform.localScale = new Vector3(6.2f, 2.4f, 6.2f);
                        Destroy(lowerCrown.GetComponent<Collider>());
                        lowerCrown.GetComponent<Renderer>().sharedMaterial = treeLeavesMaterial;

                        GameObject upperCrown = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        upperCrown.transform.SetParent(tree.transform, false);
                        upperCrown.transform.localPosition = new Vector3(0.0f, 7.6f, 0.0f);
                        upperCrown.transform.localScale = new Vector3(4.6f, 1.8f, 4.6f);
                        Destroy(upperCrown.GetComponent<Collider>());
                        upperCrown.GetComponent<Renderer>().sharedMaterial = canopyLightMat;
                    }
                    else
                    {
                        // Tall Lombardy Cypress spire
                        GameObject cypress = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                        cypress.transform.SetParent(tree.transform, false);
                        cypress.transform.localPosition = new Vector3(0.0f, 6.8f, 0.0f);
                        cypress.transform.localScale = new Vector3(2.2f, 4.8f, 2.2f);
                        Destroy(cypress.GetComponent<Collider>());
                        cypress.GetComponent<Renderer>().sharedMaterial = treeLeavesMaterial;
                    }
                }
            }
        }

        private void BuildDistantAlpineMountains(Transform parent)
        {
            GameObject alpsRoot = new GameObject("Italian_Alps_Horizon");
            alpsRoot.transform.SetParent(parent, false);

            Material mountainMat = F1Palette.GetOrCreateMaterial(
                "Mat_DistantAlps",
                new Color(0.32f, 0.42f, 0.56f),
                0.0f,
                0.10f);

            int peaks = 28;
            float radius = 4000.0f;
            Vector3 ringCenter = new Vector3(250.0f, -10.0f, 1100.0f);

            for (int i = 0; i < peaks; i++)
            {
                float angle = (i / (float)peaks) * Mathf.PI * 2.0f;
                float h = 140.0f + (i % 5) * 45.0f;
                float w = 360.0f + (i % 3) * 90.0f;

                Vector3 pos = ringCenter + new Vector3(Mathf.Cos(angle) * radius, h * 0.35f, Mathf.Sin(angle) * radius);
                GameObject peak = GameObject.CreatePrimitive(PrimitiveType.Cube);
                peak.name = $"Alpine_Peak_{i}";
                peak.transform.SetParent(alpsRoot.transform, false);
                peak.transform.position = pos;
                peak.transform.rotation = Quaternion.Euler(35.0f, angle * Mathf.Rad2Deg + 45.0f, 35.0f);
                peak.transform.localScale = new Vector3(w, h, w);
                Destroy(peak.GetComponent<Collider>());
                peak.GetComponent<Renderer>().sharedMaterial = mountainMat;
            }
        }
    }
}
