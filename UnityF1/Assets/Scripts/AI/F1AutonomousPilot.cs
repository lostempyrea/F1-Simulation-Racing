using System;
using System.Collections.Generic;
using UnityEngine;
using F1Sim.Physics;
using F1Sim.Track;

namespace F1Sim.AI
{
    public enum PilotRecoveryState
    {
        Racing,
        Reversing,
        Turnaround
    }

    /// <summary>
    /// High-performance Map-Agnostic Sensor-Based Autonomous Pilot for F1 Cars.
    /// Operates purely via forward multi-raycast sensor arrays, centerline error estimation,
    /// dynamic curvature lookahead, PD steering, threshold braking, rival radar avoidance,
    /// and automatic stuck recovery (reverse / turnaround).
    /// </summary>
    public class F1AutonomousPilot : MonoBehaviour
    {
        [Header("Vehicle & Track Components")]
        public VehicleDynamics vehicle;
        public AIDriverProfile profile;
        public TrackSpline trackSpline;

        [Header("Sensor Suite Configuration")]
        public float sensorOriginHeight = 0.28f;
        public float sensorForwardOffset = 2.15f;
        public float maxRayDistance = 150.0f;
        public LayerMask obstacleLayerMask = ~0;

        [Header("Control Gains")]
        public float steerKp = 2.2f;
        public float steerKd = 0.75f;
        public float curvatureFeedforward = 28.0f;
        public float targetSpeedStraight = 96.0f; // ~345 km/h
        public float maxCornerLateralG = 3.2f; // High-downforce F1 cornering limit (31.4 m/s2)

        [Header("State & Recovery")]
        public PilotRecoveryState recoveryState = PilotRecoveryState.Racing;
        public float recoveryTimer = 0.0f;
        public float stuckTimer = 0.0f;
        public int consecutiveStuckCount = 0;
        public float totalStuckRecoveryDuration = 0.0f;

        [Header("Telemetry (Debug Readouts)")]
        public float centerlineError = 0.0f;
        public float sensedCurvature = 0.0f;
        public float targetSpeed = 0.0f;
        public float rivalDistance = 999.0f;
        public float lateralOvertakeBias = 0.0f;

        private float previousCenterlineError = 0.0f;
        private static readonly float[] RayAngles = new float[] { -70f, -45f, -25f, -12f, 0f, 12f, 25f, 45f, 70f };
        private float[] rayDistances = new float[9];
        private Vector3[] hitPoints = new Vector3[9];

        private void Awake()
        {
            if (vehicle == null) vehicle = GetComponent<VehicleDynamics>();
        }

        private void Update()
        {
            // If running standalone without F1AIDriver coordinator
            if (GetComponent<F1AIDriver>() == null && vehicle != null)
            {
                StepPilot(Time.deltaTime);
            }
        }

        public void StepPilot(float dt, float leadCarGap = 999.0f)
        {
            if (vehicle == null) return;
            dt = Mathf.Clamp(dt, 0.001f, 0.05f);

            rivalDistance = leadCarGap;

            // 1. Stuck & Crash Detection
            UpdateStuckDetection(dt);

            // 2. Recovery State Machine Execution
            if (recoveryState != PilotRecoveryState.Racing)
            {
                ExecuteRecovery(dt);
                return;
            }

            // 3. Multi-Raycast Sensor Sweep (Scan road boundaries and barriers)
            PerformSensorSweep();

            // 4. Centerline Estimation & Curvature Lookahead
            EstimateCenterlineAndCurvature(dt);

            // 5. Rival Radar & Collision Avoidance
            ScanRivalRadar();

            // 6. Compute PD Steering Demand
            float steerDemand = ComputeSteeringDemand(dt);

            // 7. Compute Adaptive Speed, Throttle & Threshold Braking
            ComputeThrottleAndBrake(dt, out float throttle, out float brake);

            // 8. DRS Activation Check
            if (trackSpline != null)
            {
                trackSpline.ProjectPoint(vehicle.state.worldPosition, out float s, out _);
                bool inDrs = MonzaTrackData.IsInDrsZone(s);
                vehicle.state.drsActive = inDrs && (vehicle.SpeedMs > 50.0f) && (brake < 0.05f);
            }

            // 9. Step Vehicle Dynamics
            vehicle.StepPhysics(throttle, brake, steerDemand, dt);
        }

        private void PerformSensorSweep()
        {
            Transform carTr = vehicle.transform;
            Vector3 origin = carTr.position + carTr.forward * sensorForwardOffset + Vector3.up * sensorOriginHeight;

            for (int i = 0; i < RayAngles.Length; i++)
            {
                float angle = RayAngles[i];
                Quaternion rot = Quaternion.AngleAxis(angle, carTr.up);
                Vector3 rayDir = rot * carTr.forward;

                // Downward slope slightly to detect road surface drop-off / edges as well as armco
                Vector3 probeDir = (rayDir + Vector3.down * 0.04f).normalized;

                if (UnityEngine.Physics.Raycast(origin, probeDir, out RaycastHit hit, maxRayDistance, obstacleLayerMask, QueryTriggerInteraction.Ignore))
                {
                    // Check if hit is track or wall
                    rayDistances[i] = hit.distance;
                    hitPoints[i] = hit.point;
                }
                else
                {
                    rayDistances[i] = maxRayDistance;
                    hitPoints[i] = origin + rayDir * maxRayDistance;
                }
            }
        }

        private void EstimateCenterlineAndCurvature(float dt)
        {
            // Left rays: indices 0 (-70), 1 (-45), 2 (-25)
            // Right rays: indices 8 (+70), 7 (+45), 6 (+25)
            float dLeft = (rayDistances[0] * 0.35f + rayDistances[1] * 0.45f + rayDistances[2] * 0.20f);
            float dRight = (rayDistances[8] * 0.35f + rayDistances[7] * 0.45f + rayDistances[6] * 0.20f);

            // Normalized error: positive = drifting right (need steer left), negative = drifting left (need steer right)
            float denom = Mathf.Max(1.0f, dLeft + dRight);
            centerlineError = (dRight - dLeft) / denom;

            // Apply lateral overtaking bias (if overtaking rival)
            centerlineError += lateralOvertakeBias;

            // Long-range lookahead probes: indices 3 (-12°), 4 (0° center), 5 (+12°)
            float forwardDist = rayDistances[4];
            float midLeft = rayDistances[3];
            float midRight = rayDistances[5];

            // Curvature estimation from angle differential between left and right depth
            float curvatureSign = (midRight < midLeft) ? 1.0f : -1.0f; // Right corner vs Left corner
            float depthDiff = Mathf.Abs(midRight - midLeft);
            sensedCurvature = Mathf.Clamp(curvatureSign * (depthDiff / Mathf.Max(10.0f, forwardDist)), -0.25f, 0.25f);
        }

        private float ComputeSteeringDemand(float dt)
        {
            float errorRate = (centerlineError - previousCenterlineError) / dt;
            previousCenterlineError = centerlineError;

            // PD controller + dynamic curvature feedforward
            float pTerm = centerlineError * steerKp;
            float dTerm = errorRate * steerKd;
            float ffTerm = sensedCurvature * curvatureFeedforward;

            float agg = (profile != null) ? profile.cornerAggression : 1.0f;
            float steerDemand = Mathf.Clamp((pTerm + dTerm + ffTerm) * agg, -1.0f, 1.0f);

            // Smooth steering transitions
            return steerDemand;
        }

        private void ComputeThrottleAndBrake(float dt, out float throttle, out float brake)
        {
            throttle = 0.0f;
            brake = 0.0f;

            float speedMs = vehicle.SpeedMs;
            float forwardClearance = rayDistances[4];

            // 1. Calculate Target Cornering Speed from Curvature & Clearance
            float absCurv = Mathf.Abs(sensedCurvature);
            float confidence = (profile != null) ? profile.brakingConfidence : 0.98f;

            if (absCurv > 0.045f || forwardClearance < 65.0f)
            {
                // Radius of upcoming bend: R = 1 / curvature
                float estimatedRadius = 1.0f / Mathf.Max(0.012f, absCurv * 0.30f);
                float maxSafeSpeed = Mathf.Sqrt(maxCornerLateralG * 9.81f * estimatedRadius);
                targetSpeed = Mathf.Clamp(maxSafeSpeed * confidence, 35.0f, targetSpeedStraight);
            }
            else
            {
                // Clear straight ahead: maximum full attack speed
                targetSpeed = targetSpeedStraight * ((profile != null) ? profile.cornerAggression : 1.0f);
            }

            // 2. Proximity Braking for Rivals / Obstacles
            if (rivalDistance < 15.0f)
            {
                targetSpeed = Mathf.Min(targetSpeed, speedMs * 0.92f);
            }

            // 3. Threshold Braking vs Aggressive Throttle Application
            float speedDelta = speedMs - targetSpeed;
            if (speedDelta > 4.5f)
            {
                // Hard threshold braking zone
                throttle = 0.0f;
                brake = Mathf.Clamp01(speedDelta / 6.0f + 0.45f);
            }
            else if (speedDelta > 0.5f)
            {
                // Trail-braking into corner entry
                throttle = 0.0f;
                brake = Mathf.Clamp01(speedDelta / 5.0f);
            }
            else
            {
                // Decisive acceleration: full throttle on straights and corner exits
                brake = 0.0f;
                throttle = (speedDelta < -3.0f) ? 1.0f : Mathf.Clamp01((-speedDelta) / 2.5f + 0.55f);
            }
        }

        private void ScanRivalRadar()
        {
            Transform carTr = vehicle.transform;
            Vector3 scanOrigin = carTr.position + carTr.forward * 2.0f + Vector3.up * 0.45f;

            // SphereCast forward along car path
            if (UnityEngine.Physics.SphereCast(scanOrigin, 1.4f, carTr.forward, out RaycastHit hit, 32.0f, obstacleLayerMask, QueryTriggerInteraction.Ignore))
            {
                if (hit.collider != null && hit.collider.gameObject != vehicle.gameObject)
                {
                    rivalDistance = hit.distance;

                    // Evaluate passing room on Left vs Right
                    float leftRoom = rayDistances[1];
                    float rightRoom = rayDistances[7];

                    float overtBias = (profile != null) ? profile.overtakingBias : 0.9f;
                    if (leftRoom > rightRoom && leftRoom > 6.0f)
                    {
                        lateralOvertakeBias = Mathf.MoveTowards(lateralOvertakeBias, -0.45f * overtBias, Time.deltaTime * 1.5f);
                    }
                    else if (rightRoom > 6.0f)
                    {
                        lateralOvertakeBias = Mathf.MoveTowards(lateralOvertakeBias, 0.45f * overtBias, Time.deltaTime * 1.5f);
                    }
                    return;
                }
            }

            lateralOvertakeBias = Mathf.MoveTowards(lateralOvertakeBias, 0.0f, Time.deltaTime * 0.8f);
        }

        private void UpdateStuckDetection(float dt)
        {
            float speed = vehicle.SpeedMs;
            float forwardWallDist = rayDistances[4];

            // Condition: barely moving (< 2.0 m/s) with throttle or front wall obstacle (< 1.8m)
            if (speed < 2.0f && (vehicle.state.throttle > 0.25f || forwardWallDist < 2.0f))
            {
                stuckTimer += dt;
                // Quick detection threshold: 0.85s
                if (stuckTimer > 0.85f)
                {
                    recoveryState = PilotRecoveryState.Reversing;
                    recoveryTimer = 2.4f; // generous 2.4s reverse distance
                    stuckTimer = 0.0f;
                    consecutiveStuckCount++;
                    totalStuckRecoveryDuration += 2.4f;
                }
            }
            else
            {
                stuckTimer = Mathf.Max(0.0f, stuckTimer - dt * 2.0f);
            }

            // Teleport Recovery: if stuck 3 times consecutively or total recovery duration exceeds 5.0s
            if (consecutiveStuckCount >= 3 || totalStuckRecoveryDuration > 5.0f)
            {
                TriggerCenterlineTeleportRecovery();
            }
        }

        private void TriggerCenterlineTeleportRecovery()
        {
            if (trackSpline != null && vehicle != null)
            {
                trackSpline.ProjectPoint(vehicle.state.worldPosition, out float s, out _);
                Vector3 centerPt = trackSpline.GetPoint(s);
                Vector3 tan = trackSpline.GetTangent(s);
                float yawRad = Mathf.Atan2(tan.z, tan.x);

                vehicle.SetWorldPosition(centerPt + Vector3.up * 0.25f, yawRad);
                vehicle.state.vx = 28.0f; // 100 km/h launch speed
                vehicle.state.vy = 0.0f;
                vehicle.state.yawRate = 0.0f;
                if (vehicle.powertrain != null)
                {
                    vehicle.powertrain.currentGear = 2;
                }
            }

            recoveryState = PilotRecoveryState.Racing;
            stuckTimer = 0.0f;
            consecutiveStuckCount = 0;
            totalStuckRecoveryDuration = 0.0f;
        }

        private void ExecuteRecovery(float dt)
        {
            recoveryTimer -= dt;

            // Fail-safe check
            if (recoveryTimer <= -1.0f)
            {
                TriggerCenterlineTeleportRecovery();
                return;
            }

            if (recoveryState == PilotRecoveryState.Reversing)
            {
                // Force reverse gear
                if (vehicle.powertrain != null)
                {
                    vehicle.powertrain.currentGear = -1;
                }

                // Steer opposite to wall obstacle with strong lock
                float reverseSteer = (centerlineError >= 0.0f) ? -0.92f : 0.92f;
                vehicle.StepPhysics(0.75f, 0.0f, reverseSteer, dt);

                if (recoveryTimer <= 0.0f)
                {
                    recoveryState = PilotRecoveryState.Turnaround;
                    recoveryTimer = 1.0f;
                }
            }
            else if (recoveryState == PilotRecoveryState.Turnaround)
            {
                // Shift to 1st gear and turn nose back towards open track
                if (vehicle.powertrain != null)
                {
                    vehicle.powertrain.currentGear = 1;
                }

                float turnSteer = (centerlineError >= 0.0f) ? 0.90f : -0.90f;
                vehicle.StepPhysics(0.70f, 0.0f, turnSteer, dt);

                if (recoveryTimer <= 0.0f)
                {
                    recoveryState = PilotRecoveryState.Racing;
                    stuckTimer = 0.0f;
                    totalStuckRecoveryDuration = 0.0f;
                }
            }
        }
    }
}
