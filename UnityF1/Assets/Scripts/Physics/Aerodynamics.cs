using System;
using UnityEngine;

namespace F1Sim.Physics
{
    public class Aerodynamics : MonoBehaviour
    {
        [Header("Aerodynamic Coefficients")]
        public float airDensity = 1.225f;          // kg/m^3 @ 15 deg C sea level
        public float frontalArea = 1.45f;           // m^2
        public float liftCoeff = 3.25f;             // Total downforce Cl
        public float dragCoeffNominal = 0.79f;      // Cd with DRS closed
        public float drsDragReduction = 0.28f;      // 28% drag reduction when open
        public float frontDownforceRatio = 0.45f;   // 45% front, 55% rear

        public void ComputeAeroForces(
            float speedMs,
            bool drsActive,
            out float downforceFront,
            out float downforceRear,
            out float dragTotal
        )
        {
            float q = 0.5f * airDensity * speedMs * speedMs * frontalArea;

            float currentCd = drsActive ? dragCoeffNominal * (1.0f - drsDragReduction) : dragCoeffNominal;
            dragTotal = q * currentCd;

            float currentCl = drsActive ? liftCoeff * 0.82f : liftCoeff;
            float totalDownforce = q * currentCl;

            downforceFront = totalDownforce * frontDownforceRatio;
            downforceRear = totalDownforce * (1.0f - frontDownforceRatio);
        }
    }
}
