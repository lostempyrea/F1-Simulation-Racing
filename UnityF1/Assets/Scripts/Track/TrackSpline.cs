using System;
using System.Collections.Generic;
using UnityEngine;

namespace F1Sim.Track
{
    public enum TrackSurface
    {
        Tarmac,
        KerbRed,
        KerbWhite,
        Gravel,
        Grass,
        WhiteLine
    }

    public class TrackSpline
    {
        public List<Vector3> Waypoints { get; private set; }
        public float Length { get; private set; }

        private float[] cumulativeDistances;
        private int count;

        public TrackSpline(List<Vector3> points)
        {
            if (points == null || points.Count < 3)
                throw new ArgumentException("TrackSpline requires at least 3 waypoints.");

            Waypoints = new List<Vector3>(points);
            count = Waypoints.Count;
            ComputeCumulativeDistances();
        }

        private void ComputeCumulativeDistances()
        {
            cumulativeDistances = new float[count + 1];
            cumulativeDistances[0] = 0.0f;
            float total = 0.0f;

            for (int i = 0; i < count; i++)
            {
                int next = (i + 1) % count;
                total += Vector3.Distance(Waypoints[i], Waypoints[next]);
                cumulativeDistances[i + 1] = total;
            }
            Length = total;
        }

        public Vector3 GetPoint(float s)
        {
            s = Mathf.Repeat(s, Length);
            int idx = FindSegmentIndex(s);
            int p0 = (idx - 1 + count) % count;
            int p1 = idx;
            int p2 = (idx + 1) % count;
            int p3 = (idx + 2) % count;

            float segStart = cumulativeDistances[idx];
            float segLen = Mathf.Max(0.001f, cumulativeDistances[idx + 1] - segStart);
            float t = (s - segStart) / segLen;

            return CatmullRom(Waypoints[p0], Waypoints[p1], Waypoints[p2], Waypoints[p3], t);
        }

        public Vector3 GetTangent(float s)
        {
            float ds = 0.5f;
            Vector3 pA = GetPoint(s - ds);
            Vector3 pB = GetPoint(s + ds);
            return (pB - pA).normalized;
        }

        public void GetFrenetFrame(float s, out Vector3 tangent, out Vector3 lateral, out Vector3 normal)
        {
            tangent = GetTangent(s);
            Vector3 upApprox = Vector3.up;
            lateral = Vector3.Cross(upApprox, tangent).normalized;
            normal = Vector3.Cross(tangent, lateral).normalized;
        }

        public void ProjectPoint(Vector3 worldPos, out float sOut, out float dOut)
        {
            // Coarse search along waypoints
            int bestIdx = 0;
            float bestDistSq = float.MaxValue;
            for (int i = 0; i < count; i++)
            {
                float dSq = (Waypoints[i] - worldPos).sqrMagnitude;
                if (dSq < bestDistSq)
                {
                    bestDistSq = dSq;
                    bestIdx = i;
                }
            }

            // Fine binary / golden search around best segment
            float sCenter = cumulativeDistances[bestIdx];
            float sBest = sCenter;
            float minError = float.MaxValue;

            for (int step = -20; step <= 20; step++)
            {
                float sTest = sCenter + step * 1.0f;
                Vector3 pTest = GetPoint(sTest);
                float err = (pTest - worldPos).sqrMagnitude;
                if (err < minError)
                {
                    minError = err;
                    sBest = sTest;
                }
            }

            sOut = Mathf.Repeat(sBest, Length);
            Vector3 pTrack = GetPoint(sOut);
            GetFrenetFrame(sOut, out _, out Vector3 lat, out _);
            dOut = Vector3.Dot(worldPos - pTrack, lat);
        }

        private int FindSegmentIndex(float s)
        {
            int low = 0, high = count - 1;
            while (low <= high)
            {
                int mid = (low + high) / 2;
                if (s < cumulativeDistances[mid])
                    high = mid - 1;
                else if (s >= cumulativeDistances[mid + 1])
                    low = mid + 1;
                else
                    return mid;
            }
            return count - 1;
        }

        private static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                (2.0f * p1) +
                (-p0 + p2) * t +
                (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3
            );
        }
    }
}
