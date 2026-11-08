using System;
using System.Collections.Generic;
using UnityEngine;
using F1Sim.Physics;
using F1Sim.Track;

namespace F1Sim.AI
{
    [Serializable]
    public class AIDriverProfile
    {
        public string code;
        public string name;
        public string team;
        public Color teamColor;
        public float cornerAggression;
        public float brakingConfidence;
        public float overtakingBias;

        public static List<AIDriverProfile> GetOfficialRoster()
        {
            return new List<AIDriverProfile>
            {
                new AIDriverProfile { code = "VER", name = "Max Verstappen", team = "Red Bull Racing", teamColor = new Color(0.0f, 0.1f, 0.35f), cornerAggression = 0.98f, brakingConfidence = 0.99f, overtakingBias = 0.95f },
                new AIDriverProfile { code = "LEC", name = "Charles Leclerc", team = "Scuderia Ferrari", teamColor = new Color(0.85f, 0.05f, 0.05f), cornerAggression = 0.95f, brakingConfidence = 0.96f, overtakingBias = 0.90f },
                new AIDriverProfile { code = "HAM", name = "Lewis Hamilton", team = "Mercedes-AMG F1", teamColor = new Color(0.0f, 0.85f, 0.8f), cornerAggression = 0.92f, brakingConfidence = 0.94f, overtakingBias = 0.88f },
                new AIDriverProfile { code = "NOR", name = "Lando Norris", team = "McLaren F1", teamColor = new Color(1.0f, 0.55f, 0.0f), cornerAggression = 0.93f, brakingConfidence = 0.93f, overtakingBias = 0.89f },
                new AIDriverProfile { code = "ALO", name = "Fernando Alonso", team = "Aston Martin F1", teamColor = new Color(0.0f, 0.45f, 0.35f), cornerAggression = 0.96f, brakingConfidence = 0.95f, overtakingBias = 0.94f }
            };
        }
    }

    public class F1AIDriver : MonoBehaviour
    {
        public AIDriverProfile profile;
        public VehicleDynamics vehicle;
        public TrackSpline trackSpline;

        [Header("Telemetry State")]
        public float currentS = 0.0f;
        public float lateralOffsetD = 0.0f;
        public float targetLateralD = 0.0f;

        [Header("AI Racing Tuning")]
        public float lookaheadDistance = 35.0f;
        public float maxCornerSpeed = 85.0f; // m/s

        private void Awake()
        {
            if (vehicle == null) vehicle = GetComponent<VehicleDynamics>();
        }

        public void StepAI(float dt, float leadCarGap = 999.0f)
        {
            if (vehicle == null || trackSpline == null) return;

            // 1. Measure track progress s
            trackSpline.ProjectPoint(vehicle.state.worldPosition, out currentS, out lateralOffsetD);

            // 2. Pure Pursuit Target Lookahead Point
            float lookS = Mathf.Repeat(currentS + lookaheadDistance, trackSpline.Length);
            Vector3 targetTrackPos = trackSpline.GetPoint(lookS);
            trackSpline.GetFrenetFrame(lookS, out _, out Vector3 latLook, out _);

            Vector3 targetWorldPos = targetTrackPos + latLook * targetLateralD;
            Vector3 toTarget = targetWorldPos - vehicle.state.worldPosition;

            // Transform to local car body frame
            float cosPsi = Mathf.Cos(vehicle.state.yawAngle);
            float sinPsi = Mathf.Sin(vehicle.state.yawAngle);

            float localX = toTarget.x * cosPsi + toTarget.z * sinPsi;
            float localY = -toTarget.x * sinPsi + toTarget.z * cosPsi;

            // Pure Pursuit steering angle (lateral offset localY drives steering)
            float curvature = (2.0f * localY) / Mathf.Max(1.0f, toTarget.sqrMagnitude);
            float steerDemand = Mathf.Clamp(curvature * 12.0f, -1.0f, 1.0f);

            // 3. Curvature-Based Braking Profile
            float speedMs = vehicle.SpeedMs;
            float targetSpeed = maxCornerSpeed * profile.cornerAggression;

            // Check if approaching tight corner apex
            Vector3 tanAhead = trackSpline.GetTangent(currentS + 50.0f);
            Vector3 tanNow = trackSpline.GetTangent(currentS);
            float angleDelta = Vector3.Angle(tanNow, tanAhead);

            if (angleDelta > 25.0f)
            {
                targetSpeed = Mathf.Lerp(30.0f, 55.0f, profile.brakingConfidence);
            }

            // 4. Collision Avoidance & Overtaking
            if (leadCarGap < 20.0f)
            {
                targetSpeed = Mathf.Min(targetSpeed, speedMs * 0.92f);
                targetLateralD = (targetLateralD > 0.0f) ? -2.0f : 2.0f; // Slipstream lane switch
            }

            float throttle = 0.0f;
            float brake = 0.0f;

            if (speedMs < targetSpeed)
            {
                throttle = Mathf.Clamp01((targetSpeed - speedMs) / 5.0f);
                brake = 0.0f;
            }
            else
            {
                throttle = 0.0f;
                brake = Mathf.Clamp01((speedMs - targetSpeed) / 8.0f);
            }

            // DRS activation
            bool inDrs = MonzaTrackData.IsInDrsZone(currentS);
            vehicle.state.drsActive = inDrs && (speedMs > 50.0f) && (brake < 0.05f);

            // 5. Apply to vehicle dynamics
            vehicle.StepPhysics(throttle, brake, steerDemand, dt);
        }
    }
}
