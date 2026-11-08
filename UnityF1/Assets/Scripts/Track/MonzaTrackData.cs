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
            // Official Autodromo Nazionale Monza 5,793m topology
            // Reconstructed from official circuit schematic:
            // Rettifilo Pit Straight -> Variante del Rettifilo (T1-T2) -> Curva Grande (T3) ->
            // Variante della Roggia (T4-T5) -> Curva di Lesmo 1 & 2 (T6-T7) ->
            // Rettifilo del Serraglio (Through Old Banking Overpass) -> Variante Ascari (T8-T10) ->
            // Rettifilo Parabolica -> Curva Parabolica (Alboreto T11).
            List<Vector3> pts = new List<Vector3>
            {
                // 1. Pit Straight & Main Start/Finish Line (s = 0 to 1165m)
                new Vector3(  0.0f, 0.00f,   -32.7f),
                new Vector3(  0.0f, 0.00f,     0.0f), // Start / Finish Line
                new Vector3(  0.0f, 0.00f,   130.7f),
                new Vector3(  0.0f, 0.00f,   283.1f),
                new Vector3(  0.0f, 0.00f,   457.3f),
                new Vector3(  0.0f, 0.00f,   631.5f),
                new Vector3(  0.0f, 0.00f,   805.7f),
                new Vector3(  0.0f, 0.00f,   979.9f),
                new Vector3(  0.0f, 0.00f,  1132.3f),

                // 2. Variante del Rettifilo Chicane T1 & T2 (s = 1165 to 1340m)
                new Vector3( 13.1f, 0.20f,  1208.5f), // T1 Entry Right
                new Vector3( 34.8f, 0.30f,  1252.1f), // T1 Apex Right
                new Vector3( 19.6f, 0.20f,  1301.1f), // T2 Apex Left

                // 3. Curva Grande (Curva Biassono) Sweeper T3 (s = 1340 to 2150m)
                new Vector3( 24.0f, 0.40f,  1371.8f),
                new Vector3( 49.0f, 0.60f,  1502.5f),
                new Vector3( 87.1f, 0.80f,  1644.0f),
                new Vector3(136.1f, 0.80f,  1796.5f),
                new Vector3(179.6f, 0.80f,  1948.9f),
                new Vector3(206.9f, 1.00f,  2101.3f), // Curva Grande Exit

                // 4. Variante della Roggia Chicane T4 & T5 (s = 2150 to 2350m)
                new Vector3(190.5f, 1.30f,  2172.1f), // T4 Entry Left
                new Vector3(168.8f, 1.50f,  2226.5f), // T4 Apex Left
                new Vector3(185.1f, 1.60f,  2286.4f), // T5 Apex Right

                // 5. Curva di Lesmo 1 & Lesmo 2 T6 & T7 (s = 2350 to 2750m)
                new Vector3(217.8f, 2.50f,  2362.6f),
                new Vector3(272.2f, 3.80f,  2422.5f), // Lesmo 1 Apex (90 deg Right)
                new Vector3(343.0f, 4.20f,  2433.4f),
                new Vector3(408.3f, 3.80f,  2406.2f), // Lesmo 2 Entry
                new Vector3(457.3f, 2.60f,  2357.2f), // Lesmo 2 Apex (Downhill Right)
                new Vector3(489.9f, 1.80f,  2286.4f),
                new Vector3(500.8f, 1.20f,  2199.3f), // Lesmo Exit onto Serraglio

                // 6. Rettifilo del Serraglio - Passing Under Historic Banking Overpass (s = 2750 to 4150m)
                new Vector3(495.4f, 0.80f,  2014.2f),
                new Vector3(479.1f, 0.60f,  1796.5f),
                new Vector3(457.3f, 0.40f,  1578.7f), // Historic Banked Oval Bridge Overpass
                new Vector3(430.1f, 0.30f,  1361.0f),
                new Vector3(402.8f, 0.20f,  1143.2f),
                new Vector3(386.5f, 0.20f,   979.9f),

                // 7. Variante Ascari Chicane T8, T9, T10 (s = 4150 to 4550m)
                new Vector3(359.3f, 0.20f,   909.1f), // T8 Entry Left
                new Vector3(332.1f, 0.20f,   838.3f), // T8 Apex Left
                new Vector3(364.7f, 0.20f,   767.6f), // T9 Apex Right
                new Vector3(381.1f, 0.20f,   691.4f), // T10 Apex Left
                new Vector3(353.8f, 0.20f,   609.7f), // Ascari Exit

                // 8. Rettifilo Parabolica - Back Straight (s = 4550 to 5200m)
                new Vector3(315.7f, 0.10f,   500.8f),
                new Vector3(310.3f, 0.00f,   359.3f),
                new Vector3(310.3f, 0.00f,   206.9f),
                new Vector3(310.3f, 0.00f,    54.4f),
                new Vector3(304.9f, 0.00f,   -43.6f), // Parabolica Braking Zone

                // 9. Curva Parabolica (Curva Alboreto) T11 (s = 5200 to 5750m)
                new Vector3(294.0f, 0.00f,  -125.2f), // Parabolica Entry
                new Vector3(261.3f, 0.00f,  -196.0f),
                new Vector3(206.9f, 0.00f,  -250.4f),
                new Vector3(141.5f, 0.00f,  -272.2f), // Parabolica Apex
                new Vector3( 76.2f, 0.00f,  -250.4f),
                new Vector3( 27.2f, 0.00f,  -190.5f), // Parabolica Exit onto Pit Straight
                new Vector3(  6.5f, 0.00f,  -108.9f)
            };

            return pts;
        }

        public static bool IsInDrsZone(float s)
        {
            s = Mathf.Repeat(s, TotalLength);
            // DRS Zone 1: Main Pit Straight (150m to 1120m)
            if (s >= 150.0f && s <= 1120.0f) return true;
            // DRS Zone 2: Serraglio Straight to Ascari (2850m to 4050m)
            if (s >= 2850.0f && s <= 4050.0f) return true;
            return false;
        }
    }
}
