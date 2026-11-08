using System.Collections.Generic;
using UnityEngine;

namespace F1Sim.Track
{
    public static class MonzaTrackData
    {
        public const float TotalLength = 5793.0f;
        public const float TrackWidth = 12.0f;

        public static List<Vector3> GetMonzaWaypoints()
        {
            // Key control points replicating the 5,793m Monza circuit topology
            List<Vector3> pts = new List<Vector3>();

            // 1. Pit Straight & Main Start/Finish (s = 0 to 1050m)
            for (float s = 0.0f; s < 1050.0f; s += 60.0f)
                pts.Add(new Vector3(0.0f, 0.0f, s));

            // 2. Variante del Rettifilo Chicane T1 & T2 (s = 1050 to 1180m)
            pts.Add(new Vector3(12.0f, 0.2f, 1090.0f));
            pts.Add(new Vector3(28.0f, 0.3f, 1120.0f));
            pts.Add(new Vector3(15.0f, 0.2f, 1150.0f));

            // 3. Curva Grande (Biassono) Sweeper to T4 (s = 1180 to 1900m)
            for (int i = 0; i <= 8; i++)
            {
                float angle = Mathf.PI * 0.5f - (i / 8.0f) * 0.45f;
                float r = 320.0f;
                float cx = -180.0f;
                float cz = 1350.0f + i * 65.0f;
                pts.Add(new Vector3(cx + Mathf.Cos(angle) * r, 0.8f, cz));
            }

            // 4. Variante della Roggia Chicane T4 & T5 (s = 1900 to 2050m)
            pts.Add(new Vector3(110.0f, 1.4f, 1920.0f));
            pts.Add(new Vector3(92.0f,  1.5f, 1960.0f));
            pts.Add(new Vector3(105.0f, 1.6f, 2010.0f));

            // 5. Curva di Lesmo 1 & Lesmo 2 (s = 2050 to 2400m)
            pts.Add(new Vector3(150.0f, 3.8f, 2100.0f));
            pts.Add(new Vector3(210.0f, 4.5f, 2180.0f)); // Lesmo 1 Apex
            pts.Add(new Vector3(260.0f, 6.2f, 2260.0f)); // Lesmo 2 Apex
            pts.Add(new Vector3(280.0f, 5.0f, 2350.0f));

            // 6. Serraglio & Curva del Serraglio Back Straight (s = 2400 to 3200m)
            for (float s = 2400.0f; s < 3200.0f; s += 80.0f)
            {
                float frac = (s - 2400.0f) / 800.0f;
                pts.Add(new Vector3(Mathf.Lerp(280.0f, 240.0f, frac), Mathf.Lerp(5.0f, 1.5f, frac), 2350.0f + frac * 850.0f));
            }

            // 7. Variante Ascari Complex T8, T9, T10 (s = 3200 to 3500m)
            pts.Add(new Vector3(210.0f, 1.8f, 3260.0f)); // T8 Entry Left
            pts.Add(new Vector3(235.0f, 1.4f, 3330.0f)); // T9 Right
            pts.Add(new Vector3(195.0f, 0.8f, 3420.0f)); // T10 Exit Left

            // 8. Straight to Parabolica (s = 3500 to 4800m)
            for (int i = 0; i < 12; i++)
            {
                float frac = i / 12.0f;
                pts.Add(new Vector3(Mathf.Lerp(190.0f, 140.0f, frac), 0.2f, 3500.0f - frac * 1700.0f));
            }

            // 9. Curva Parabolica (Alboreto) (s = 4800 to 5600m)
            for (int i = 0; i <= 10; i++)
            {
                float theta = Mathf.PI * 0.5f + (i / 10.0f) * Mathf.PI;
                float r = 210.0f;
                float cx = 70.0f;
                float cz = 80.0f;
                pts.Add(new Vector3(cx + Mathf.Cos(theta) * r, 0.0f, cz + Mathf.Sin(theta) * r * 0.85f));
            }

            // 10. Final entry connecting back to Main Straight s = 0
            pts.Add(new Vector3(30.0f, 0.0f, -80.0f));
            pts.Add(new Vector3(10.0f, 0.0f, -30.0f));

            return pts;
        }

        public static bool IsInDrsZone(float s)
        {
            s = Mathf.Repeat(s, TotalLength);
            // DRS Zone 1: Main Pit Straight (115m to 1020m)
            if (s >= 115.0f && s <= 1020.0f) return true;
            // DRS Zone 2: Serraglio to Ascari (2450m to 3180m)
            if (s >= 2450.0f && s <= 3180.0f) return true;
            return false;
        }
    }
}
