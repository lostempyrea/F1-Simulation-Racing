using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using F1Sim.Core;

namespace F1Sim.Track
{
    public class TrackMeshGenerator : MonoBehaviour
    {
        [Header("Materials")]
        public Material tarmacMaterial;
        public Material kerbMaterial;
        public Material gantryMaterial;
        public Material boardMaterial;

        public MeshFilter trackFilter;
        public MeshRenderer trackRenderer;
        public MeshCollider trackCollider;

        public void GenerateTrack(TrackSpline spline, float stepSize = 2.0f)
        {
            if (spline == null) return;

            if (tarmacMaterial == null) tarmacMaterial = F1Palette.CreateAsphaltMaterial();
            if (kerbMaterial == null) kerbMaterial = F1Palette.CreateKerbMaterial();

            float length = spline.Length;
            int numSteps = Mathf.Max(40, Mathf.RoundToInt(length / stepSize));

            List<Vector3> vertices = new List<Vector3>(numSteps * 4);
            List<Vector3> normals = new List<Vector3>(numSteps * 4);
            List<Vector2> uvs = new List<Vector2>(numSteps * 4);
            List<int> triangles = new List<int>(numSteps * 18);

            float halfWidth = MonzaTrackData.TrackWidth * 0.5f;

            for (int i = 0; i < numSteps; i++)
            {
                float s = (i / (float)numSteps) * length;
                Vector3 center = spline.GetPoint(s) + new Vector3(0.0f, 0.01f, 0.0f);
                spline.GetFrenetFrame(s, out Vector3 tan, out Vector3 lat, out Vector3 norm);

                Vector3 pL  = center - lat * halfWidth;
                Vector3 pLL = center - lat * (halfWidth - 0.35f);
                Vector3 pRR = center + lat * (halfWidth - 0.35f);
                Vector3 pR  = center + lat * halfWidth;

                float vCoord = s * 0.12f;

                vertices.Add(pL);  normals.Add(norm); uvs.Add(new Vector2(0.0f,  vCoord));
                vertices.Add(pLL); normals.Add(norm); uvs.Add(new Vector2(0.06f, vCoord));
                vertices.Add(pRR); normals.Add(norm); uvs.Add(new Vector2(0.94f, vCoord));
                vertices.Add(pR);  normals.Add(norm); uvs.Add(new Vector2(1.0f,  vCoord));
            }

            for (int i = 0; i < numSteps; i++)
            {
                int curr = i * 4;
                int next = ((i + 1) % numSteps) * 4;

                for (int j = 0; j < 3; j++)
                {
                    int p00 = curr + j;
                    int p01 = curr + j + 1;
                    int p10 = next + j;
                    int p11 = next + j + 1;

                    triangles.Add(p00); triangles.Add(p10); triangles.Add(p01);
                    triangles.Add(p01); triangles.Add(p10); triangles.Add(p11);
                }
            }

            Mesh mesh = new Mesh { name = "MonzaTrackRibbonMesh" };
            mesh.indexFormat = IndexFormat.UInt32;
            mesh.SetVertices(vertices);
            mesh.SetNormals(normals);
            mesh.SetUVs(0, uvs);
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateBounds();

            if (trackFilter == null) trackFilter = GetComponent<MeshFilter>();
            if (trackRenderer == null) trackRenderer = GetComponent<MeshRenderer>();
            if (trackCollider == null) trackCollider = GetComponent<MeshCollider>();

            if (trackFilter != null) trackFilter.sharedMesh = mesh;
            if (trackRenderer != null)
            {
                trackRenderer.sharedMaterial = tarmacMaterial;
                trackRenderer.receiveShadows = true;
            }
            if (trackCollider != null) trackCollider.sharedMesh = mesh;

            // Start / Finish Checkered Line & Grid Boxes
            SpawnStartFinishLine(spline);
        }

        private void SpawnStartFinishLine(TrackSpline spline)
        {
            Transform old = transform.Find("Start_Finish_Checkered");
            if (old != null) Destroy(old.gameObject);

            Vector3 pos = spline.GetPoint(6.0f);
            spline.GetFrenetFrame(6.0f, out Vector3 tan, out _, out Vector3 norm);

            GameObject lineObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            lineObj.name = "Start_Finish_Checkered";
            lineObj.transform.SetParent(transform, false);
            lineObj.transform.position = pos + norm * 0.018f;
            lineObj.transform.rotation = Quaternion.LookRotation(tan, norm);
            lineObj.transform.localScale = new Vector3(MonzaTrackData.TrackWidth - 0.6f, 0.005f, 1.8f);
            Destroy(lineObj.GetComponent<Collider>());
            lineObj.GetComponent<Renderer>().sharedMaterial = F1Palette.CreateCheckeredMaterial();
        }

        public void GenerateKerbs(TrackSpline spline, float stepSize = 1.0f)
        {
            if (spline == null) return;
            if (kerbMaterial == null) kerbMaterial = F1Palette.CreateKerbMaterial();

            Transform old = transform.Find("TrackKerbs");
            if (old != null) Destroy(old.gameObject);

            GameObject kerbObj = new GameObject("TrackKerbs");
            kerbObj.transform.SetParent(transform, false);
            MeshFilter mf = kerbObj.AddComponent<MeshFilter>();
            MeshRenderer mr = kerbObj.AddComponent<MeshRenderer>();
            mr.sharedMaterial = kerbMaterial;
            mr.receiveShadows = true;

            float length = spline.Length;
            int numSteps = Mathf.Max(40, Mathf.RoundToInt(length / stepSize));
            float halfWidth = MonzaTrackData.TrackWidth * 0.5f;
            float kerbWidth = 0.95f;
            float kerbHeight = 0.055f;

            List<Vector3> verts = new List<Vector3>(numSteps * 6);
            List<Vector3> norms = new List<Vector3>(numSteps * 6);
            List<Vector2> uvs = new List<Vector2>(numSteps * 6);
            List<int> tris = new List<int>(numSteps * 24);

            for (int i = 0; i < numSteps; i++)
            {
                float s = (i / (float)numSteps) * length;
                Vector3 center = spline.GetPoint(s);
                spline.GetFrenetFrame(s, out _, out Vector3 lat, out Vector3 norm);

                float v = s * 0.45f;

                // Left Kerb (3 profile vertices)
                Vector3 kL0 = center - lat * halfWidth + norm * 0.01f;
                Vector3 kL1 = center - lat * (halfWidth + kerbWidth * 0.5f) + norm * kerbHeight;
                Vector3 kL2 = center - lat * (halfWidth + kerbWidth) + norm * 0.01f;

                verts.Add(kL0); norms.Add(norm); uvs.Add(new Vector2(0.0f, v));
                verts.Add(kL1); norms.Add(norm); uvs.Add(new Vector2(0.5f, v));
                verts.Add(kL2); norms.Add(norm); uvs.Add(new Vector2(1.0f, v));

                // Right Kerb (3 profile vertices)
                Vector3 kR0 = center + lat * halfWidth + norm * 0.01f;
                Vector3 kR1 = center + lat * (halfWidth + kerbWidth * 0.5f) + norm * kerbHeight;
                Vector3 kR2 = center + lat * (halfWidth + kerbWidth) + norm * 0.01f;

                verts.Add(kR0); norms.Add(norm); uvs.Add(new Vector2(0.0f, v));
                verts.Add(kR1); norms.Add(norm); uvs.Add(new Vector2(0.5f, v));
                verts.Add(kR2); norms.Add(norm); uvs.Add(new Vector2(1.0f, v));
            }

            for (int i = 0; i < numSteps; i++)
            {
                int cL = i * 6;
                int nL = ((i + 1) % numSteps) * 6;

                // Left Kerb ribbon
                tris.Add(cL);     tris.Add(nL);     tris.Add(cL + 1);
                tris.Add(cL + 1); tris.Add(nL);     tris.Add(nL + 1);
                tris.Add(cL + 1); tris.Add(nL + 1); tris.Add(cL + 2);
                tris.Add(cL + 2); tris.Add(nL + 1); tris.Add(nL + 2);

                // Right Kerb ribbon
                int cR = cL + 3;
                int nR = nL + 3;
                tris.Add(cR);     tris.Add(cR + 1); tris.Add(nR);
                tris.Add(cR + 1); tris.Add(nR + 1); tris.Add(nR);
                tris.Add(cR + 1); tris.Add(cR + 2); tris.Add(nR + 1);
                tris.Add(cR + 2); tris.Add(nR + 2); tris.Add(nR + 1);
            }

            Mesh km = new Mesh { name = "MonzaKerbsMesh" };
            km.indexFormat = IndexFormat.UInt32;
            km.SetVertices(verts);
            km.SetNormals(norms);
            km.SetUVs(0, uvs);
            km.SetTriangles(tris, 0);
            km.RecalculateBounds();
            mf.sharedMesh = km;
        }

        public void SpawnFiaGantry(TrackSpline spline, float sLocation = 16.0f)
        {
            Transform old = transform.Find("FIA_Starting_Gantry");
            if (old != null) Destroy(old.gameObject);

            GameObject gantryObj = new GameObject("FIA_Starting_Gantry");
            gantryObj.transform.SetParent(transform, false);

            Material steelMat = F1Palette.GetOrCreateMaterial("Mat_GantrySteel", new Color(0.18f, 0.20f, 0.24f), 0.75f, 0.65f);
            Material bannerMat = F1Palette.GetOrCreateMaterial("Mat_GantryBanner", F1Palette.RacingRed, 0.25f, 0.70f);
            Material podMat = F1Palette.GetOrCreateMaterial("Mat_GantryPod", F1Palette.CarbonDark, 0.4f, 0.5f);
            Material redLightMat = F1Palette.GetOrCreateMaterial(
                "Mat_GantryRedLight",
                F1Palette.RacingRed,
                0.2f,
                0.9f,
                null,
                F1Palette.RacingRed * 3.5f);

            Vector3 center = spline.GetPoint(sLocation);
            spline.GetFrenetFrame(sLocation, out Vector3 tan, out Vector3 lat, out Vector3 norm);
            Quaternion rot = Quaternion.LookRotation(tan, norm);

            float span = MonzaTrackData.TrackWidth + 5.0f;
            float height = 6.8f;

            // Left & Right Twin Truss Support Pillars
            for (int side = -1; side <= 1; side += 2)
            {
                GameObject post = GameObject.CreatePrimitive(PrimitiveType.Cube);
                post.name = $"Gantry_Pillar_{(side < 0 ? "L" : "R")}";
                post.transform.SetParent(gantryObj.transform, false);
                post.transform.position = center + lat * (side * span * 0.5f) + norm * (height * 0.5f);
                post.transform.rotation = rot;
                post.transform.localScale = new Vector3(0.45f, height, 0.65f);
                Destroy(post.GetComponent<Collider>());
                post.GetComponent<Renderer>().sharedMaterial = steelMat;
            }

            // Overhead Crossbeam & Red Championship Banner
            GameObject beam = GameObject.CreatePrimitive(PrimitiveType.Cube);
            beam.name = "Gantry_Overhead_Truss";
            beam.transform.SetParent(gantryObj.transform, false);
            beam.transform.position = center + norm * height;
            beam.transform.rotation = rot;
            beam.transform.localScale = new Vector3(span + 0.6f, 0.65f, 0.75f);
            Destroy(beam.GetComponent<Collider>());
            beam.GetComponent<Renderer>().sharedMaterial = steelMat;

            GameObject banner = GameObject.CreatePrimitive(PrimitiveType.Cube);
            banner.name = "Gantry_Championship_Banner";
            banner.transform.SetParent(gantryObj.transform, false);
            banner.transform.position = center + norm * (height + 0.65f);
            banner.transform.rotation = rot;
            banner.transform.localScale = new Vector3(span * 0.78f, 0.75f, 0.25f);
            Destroy(banner.GetComponent<Collider>());
            banner.GetComponent<Renderer>().sharedMaterial = bannerMat;

            // 5 Individual FIA Start Light Pods & Glowing Red Lenses
            for (int i = 0; i < 5; i++)
            {
                float latOffset = (i - 2) * 0.72f;
                Vector3 podPos = center + lat * latOffset + norm * (height - 0.68f);

                GameObject podHousing = GameObject.CreatePrimitive(PrimitiveType.Cube);
                podHousing.name = $"FIA_Light_Pod_{i + 1}";
                podHousing.transform.SetParent(gantryObj.transform, false);
                podHousing.transform.position = podPos;
                podHousing.transform.rotation = rot;
                podHousing.transform.localScale = new Vector3(0.48f, 0.82f, 0.32f);
                Destroy(podHousing.GetComponent<Collider>());
                podHousing.GetComponent<Renderer>().sharedMaterial = podMat;

                // Dual Red Emissive Lenses facing the grid
                for (int r = 0; r < 2; r++)
                {
                    GameObject lens = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    lens.name = $"FIA_Red_Lens_{i + 1}_{r}";
                    lens.transform.SetParent(gantryObj.transform, false);
                    lens.transform.position = podPos - tan * 0.17f + norm * (r == 0 ? 0.16f : -0.16f);
                    lens.transform.rotation = rot * Quaternion.Euler(90.0f, 0.0f, 0.0f);
                    lens.transform.localScale = new Vector3(0.24f, 0.03f, 0.24f);
                    Destroy(lens.GetComponent<Collider>());
                    lens.GetComponent<Renderer>().sharedMaterial = redLightMat;
                }
            }
        }
    }
}
