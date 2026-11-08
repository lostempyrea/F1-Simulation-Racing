using System;
using UnityEngine;

namespace F1Sim.Physics
{
    [Serializable]
    public class VehicleConfig
    {
        public float mass = 798.0f;               // Total mass with driver (kg)
        public float wheelbase = 3.60f;           // Wheelbase L (m)
        public float distCgFront = 1.656f;        // Lf (46% front weight bias)
        public float distCgRear = 1.944f;         // Lr (54% rear weight bias)
        public float trackWidthFront = 1.60f;     // Wf (m)
        public float trackWidthRear = 1.55f;      // Wr (m)
        public float cgHeight = 0.31f;            // Center of gravity height (m)
        public float yawInertia = 1150.0f;        // Iz (kg*m^2)
        public float wheelInertia = 1.25f;        // Iw (kg*m^2)
        public float tyreRadius = 0.360f;         // Rw (18-inch Pirelli slick)
        public float frontRollStiffness = 0.55f;  // 55% front anti-roll bar distribution
        public float maxSteerRad = 0.900f;        // ~51.5 degrees road wheel steering
        public float steerRatio = 14.0f;          // Steering wheel to road wheel ratio
    }

    [Serializable]
    public class VehicleState
    {
        // 3D World Cartesian Coordinates & Heading (Independent of track curves)
        public Vector3 worldPosition = Vector3.zero;
        public float yawAngle = 0.0f;             // Heading angle psi (radians)

        // Body Frame Velocities (m/s)
        public float vx = 0.0f;                   // Longitudinal speed
        public float vy = 0.0f;                   // Lateral slip speed
        public float yawRate = 0.0f;              // Angular velocity r (rad/s)

        // Accelerations (Body Frame)
        public float ax = 0.0f;                   // Longitudinal m/s^2
        public float ay = 0.0f;                   // Lateral m/s^2
        public float yawAccel = 0.0f;             // rad/s^2

        // Wheel Angular Speeds (FL, FR, RL, RR) (rad/s)
        public float[] wheelOmega = new float[4];

        // Driver Actuation
        public float throttle = 0.0f;
        public float brake = 0.0f;
        public float steeringAngle = 0.0f;
        public int gear = 1;
        public float rpm = 4000.0f;
        public bool drsActive = false;
        public bool drsAvailable = false;
        public float brakeBias = 0.58f;           // 58% front brake bias default
    }

    public class VehicleDynamics : MonoBehaviour
    {
        [Header("Configuration")]
        public VehicleConfig config = new VehicleConfig();
        public VehicleState state = new VehicleState();

        [Header("Subsystems")]
        public Powertrain powertrain;
        public Aerodynamics aero;

        // Wheel contact points relative to CG (FL, FR, RL, RR)
        private Vector2[] wheelOffsets;
        private float[] lastSlipAngles = new float[4];
        private float[] lastSlipRatios = new float[4];
        private float[] normalLoads = new float[4];

        public float SpeedKmh => Mathf.Sqrt(state.vx * state.vx + state.vy * state.vy) * 3.6f;
        public float SpeedMs => Mathf.Sqrt(state.vx * state.vx + state.vy * state.vy);
        public float LongitudinalG => state.ax / 9.80665f;
        public float LateralG => state.ay / 9.80665f;
        public float KerbVibration { get; set; } = 0.0f;
        public System.Action<float> OnImpact;

        private void Awake()
        {
            if (powertrain == null) powertrain = GetComponent<Powertrain>();
            if (powertrain == null) powertrain = gameObject.AddComponent<Powertrain>();
            if (aero == null) aero = GetComponent<Aerodynamics>();
            if (aero == null) aero = gameObject.AddComponent<Aerodynamics>();

            wheelOffsets = new Vector2[4]
            {
                new Vector2(+config.distCgFront, +config.trackWidthFront * 0.5f), // FL
                new Vector2(+config.distCgFront, -config.trackWidthFront * 0.5f), // FR
                new Vector2(-config.distCgRear,  +config.trackWidthRear  * 0.5f), // RL
                new Vector2(-config.distCgRear,  -config.trackWidthRear  * 0.5f), // RR
            };

            for (int i = 0; i < 4; i++)
                state.wheelOmega[i] = 0.0f;
        }

        /// <summary>
        /// Integrates true Cartesian 6-DOF planar physics using Newton-Euler equations of motion.
        /// When the driver does not steer, the vehicle strictly preserves linear momentum and
        /// moves straight ahead without artificial auto-cornering.
        /// </summary>
        public void StepPhysics(float throttleInput, float brakeInput, float steerInput, float dt, float[] surfaceMu = null)
        {
            dt = Mathf.Max(0.0001f, dt);
            if (surfaceMu == null || surfaceMu.Length < 4)
                surfaceMu = new float[] { 1.0f, 1.0f, 1.0f, 1.0f };

            state.throttle = Mathf.Clamp01(throttleInput);
            state.brake = Mathf.Clamp01(brakeInput);
            // Speed-sensitive steering limiter: prevent snap oversteer at high speed
            float speedFactor = 1.0f / (1.0f + 0.0018f * state.vx * state.vx);
            float effectiveMaxSteer = config.maxSteerRad * speedFactor;
            state.steeringAngle = Mathf.Clamp(steerInput * effectiveMaxSteer, -effectiveMaxSteer, effectiveMaxSteer);

            // 1. Aerodynamics Downforce & Drag
            float vEff = Mathf.Max(1.0f, state.vx);
            float downforceFront = 0.0f, downforceRear = 0.0f, dragTotal = 0.0f;
            if (aero != null)
            {
                aero.ComputeAeroForces(vEff, state.drsActive, out downforceFront, out downforceRear, out dragTotal);
            }

            // 2. Dynamic Weight Transfer (Static + Longitudinal + Lateral with roll stiffness)
            ComputeNormalLoads(state.ax, state.ay, downforceFront, downforceRear);

            // 3. Powertrain Drive Torques
            float engineDriveTorque = 0.0f;
            if (powertrain != null)
            {
                powertrain.Step(state.throttle, state.vx, dt, state.brake);
                state.rpm = powertrain.Rpm;
                state.gear = powertrain.CurrentGear;
                engineDriveTorque = powertrain.GetRearAxleTorque();
            }

            // 4. Contact Patch Kinematics (Slip Angles & Slip Ratios)
            float vxCg = state.vx;
            float vyCg = state.vy;
            float r = state.yawRate;

            float[] steerAngles = new float[4] { state.steeringAngle, state.steeringAngle, 0.0f, 0.0f };
            float[] vxWheels = new float[4];
            float[] vyWheels = new float[4];
            float[] slipAngles = new float[4];

            for (int i = 0; i < 4; i++)
            {
                float rx = wheelOffsets[i].x;
                float ry = wheelOffsets[i].y;
                float vxLocal = vxCg - r * ry;
                float vyLocal = vyCg + r * rx;

                float delta = steerAngles[i];
                float cosD = Mathf.Cos(delta);
                float sinD = Mathf.Sin(delta);

                vxWheels[i] = vxLocal * cosD + vyLocal * sinD;
                vyWheels[i] = -vxLocal * sinD + vyLocal * cosD;

                float vMag = Mathf.Max(0.5f, Mathf.Abs(vxWheels[i]));
                slipAngles[i] = -Mathf.Atan2(vyWheels[i], vMag);
            }

            // 5. Wheel Angular Speeds & Slip Ratios
            float[] brakeTorques = ComputeBrakeTorques(state.brake, state.brakeBias);
            float[] fDriveAct = new float[4];
            float[] fBrakeAct = new float[4];
            float[] slipRatios = new float[4];

            for (int i = 0; i < 4; i++)
            {
                float fz = normalLoads[i];
                float muPeak = ComputePacejkaMu(fz, false) * surfaceMu[i];
                float maxGrip = muPeak * fz;

                if (state.brake > 0.01f)
                {
                    float fDemand = brakeTorques[i] / config.tyreRadius;
                    fBrakeAct[i] = Mathf.Min(fDemand, maxGrip);
                    float brakeSlip = -0.09f * state.brake;
                    state.wheelOmega[i] = (vxCg * (1.0f + brakeSlip)) / config.tyreRadius;
                }
                else
                {
                    fBrakeAct[i] = 0.0f;
                    if (i >= 2 && state.throttle > 0.01f && state.gear != 0) // Rear wheel drive (Forward or Reverse)
                    {
                        float fDriveDemand = (engineDriveTorque * 0.5f) / config.tyreRadius;
                        if (state.gear < 0)
                        {
                            // Reverse drive: negative force
                            fDriveAct[i] = Mathf.Max(fDriveDemand, -maxGrip * 0.98f);
                            float rearSlipMag = Mathf.Clamp01(Mathf.Abs(fDriveDemand) / Mathf.Max(1.0f, maxGrip)) * 0.15f;
                            state.wheelOmega[i] = (Mathf.Min(-0.2f, vxCg) * (1.0f + rearSlipMag)) / config.tyreRadius;
                        }
                        else
                        {
                            fDriveAct[i] = Mathf.Min(fDriveDemand, maxGrip * 0.98f);
                            float fDemandMag = Mathf.Abs(fDriveDemand);
                            float rearSlipMag = (fDemandMag <= maxGrip)
                                ? 0.05f * (fDemandMag / Mathf.Max(1.0f, maxGrip))
                                : Mathf.Min(0.30f, 0.05f + 0.20f * ((fDemandMag - maxGrip) / Mathf.Max(1.0f, maxGrip)));
                            state.wheelOmega[i] = (Mathf.Max(0.2f, vxCg) * (1.0f + rearSlipMag)) / config.tyreRadius;
                        }
                    }
                    else
                    {
                        fDriveAct[i] = 0.0f;
                        state.wheelOmega[i] = vxCg / config.tyreRadius;
                    }
                }

                float vBase = Mathf.Max(0.5f, Mathf.Abs(vxWheels[i]));
                slipRatios[i] = (state.wheelOmega[i] * config.tyreRadius - vxWheels[i]) / vBase;
            }

            // 6. Tyre Forces via Pacejka Magic Formula & Friction Ellipse
            float fxBodyTotal = 0.0f;
            float fyBodyTotal = 0.0f;
            float mzTotal = 0.0f;
            float kBlend = Mathf.Clamp01(Mathf.Abs(vxCg) / 0.25f);

            for (int i = 0; i < 4; i++)
            {
                float fz = normalLoads[i];
                float muLat = ComputePacejkaMu(fz, true) * surfaceMu[i];
                float fyPure = muLat * fz * Mathf.Sin(1.65f * Mathf.Atan(14.5f * slipAngles[i]));

                // Friction Ellipse Coupling: prevent excessive lateral grip collapse under normal throttle
                float kappaNorm = Mathf.Clamp01(Mathf.Abs(slipRatios[i]) / 0.18f);
                float ellScale = Mathf.Sqrt(Mathf.Max(0.20f, 1.0f - kappaNorm * kappaNorm * 0.70f));
                float fyCombined = fyPure * ellScale * kBlend;

                float delta = steerAngles[i];
                float cosD = Mathf.Cos(delta);
                float sinD = Mathf.Sin(delta);

                float fxWheel = 0.0f;
                if (state.brake > 0.01f)
                {
                    float signMotion = (Mathf.Abs(vxCg) > 0.05f) ? Mathf.Sign(vxCg) : 1.0f;
                    fxWheel = -signMotion * fBrakeAct[i];
                }
                else if (i >= 2)
                {
                    fxWheel = fDriveAct[i];
                }

                float fxBody = fxWheel * cosD - fyCombined * sinD;
                float fyBody = fxWheel * sinD + fyCombined * cosD;

                fxBodyTotal += fxBody;
                fyBodyTotal += fyBody;

                float rx = wheelOffsets[i].x;
                float ry = wheelOffsets[i].y;
                mzTotal += (rx * fyBody - ry * fxBody);
            }

            // 7. Equations of Motion (Non-Inertial Body Frame)
            float signVx = (Mathf.Abs(vxCg) > 0.05f) ? Mathf.Sign(vxCg) : 0.0f;
            float dragForce = -signVx * dragTotal;
            float rollResist = -signVx * (0.015f * config.mass * 9.81f);

            float fxNet = fxBodyTotal + dragForce + rollResist;
            float fyNet = fyBodyTotal;

            state.ax = fxNet / config.mass;
            state.ay = fyNet / config.mass;
            // Natural yaw damping (combined pneumatic trail self-aligning torque)
            const float C_YAW_DAMP = 950.0f; // Nm·s/rad
            mzTotal -= C_YAW_DAMP * state.yawRate;
            state.yawAccel = mzTotal / config.yawInertia;

            float dvx = (state.ax + vyCg * r) * dt;
            float dvy = (state.ay - vxCg * r) * dt;
            float dyawRate = state.yawAccel * dt;

            float vxNext = state.vx + dvx;
            if (state.brake > 0.05f && state.throttle < 0.02f && (state.vx * vxNext <= 0.0f))
            {
                state.vx = 0.0f;
                state.vy = 0.0f;
                state.yawRate = 0.0f;
            }
            else
            {
                state.vx = Mathf.Clamp(vxNext, -18.0f, 105.0f);
                state.vy += dvy;
                state.vy *= Mathf.Clamp01(1.0f - 8.0f * dt); // Natural high-grip tyre scrub damping
                state.yawRate += dyawRate;
            }

            // 8. Integration & Solid Barrier Collision Detection
            float cosPsi = Mathf.Cos(state.yawAngle);
            float sinPsi = Mathf.Sin(state.yawAngle);

            Vector3 worldVelocity = new Vector3(
                state.vx * cosPsi - state.vy * sinPsi,
                0.0f,
                state.vx * sinPsi + state.vy * cosPsi
            );

            Vector3 moveDelta = worldVelocity * dt;
            float moveDist = moveDelta.magnitude;
            bool hitObstacle = false;
            Vector3 hitNormal = Vector3.zero;

            Vector3 halfExtents = new Vector3(0.92f, 0.35f, 2.30f);
            Quaternion carRot = Quaternion.Euler(0.0f, -state.yawAngle * Mathf.Rad2Deg + 90.0f, 0.0f);
            Vector3 testPos = state.worldPosition + new Vector3(0.0f, 0.55f, 0.0f);

            if (moveDist > 0.001f)
            {
                Vector3 moveDir = moveDelta / moveDist;
                RaycastHit[] hits = UnityEngine.Physics.BoxCastAll(testPos, halfExtents, moveDir, carRot, moveDist + 0.15f);
                for (int hIdx = 0; hIdx < hits.Length; hIdx++)
                {
                    RaycastHit h = hits[hIdx];
                    if (h.collider != null && h.collider.transform != transform && !h.collider.transform.IsChildOf(transform))
                    {
                        string cName = h.collider.name;
                        bool isTrackOrGround = cName.Contains("Track_Mesh") || cName.Contains("TrackKerb") ||
                                              cName.Contains("Start_Finish") || cName.Contains("Ground") ||
                                              cName.Contains("Gravel");
                        bool isObstacle = (cName.Contains("Barrier") || cName.Contains("Stand_Backwall") ||
                                          cName.Contains("Overpass_Pillar") || cName.Contains("Guardrail") ||
                                          cName.Contains("Pit_Garage") || cName.Contains("Rail_") ||
                                          cName.Contains("Tecpro")) && !isTrackOrGround;
                        if (isObstacle)
                        {
                            hitObstacle = true;
                            hitNormal = h.normal;
                            hitNormal.y = 0.0f;
                            if (hitNormal.sqrMagnitude > 0.01f) hitNormal.Normalize();
                            else hitNormal = -moveDir;
                            break;
                        }
                    }
                }
            }

            if (!hitObstacle)
            {
                Collider[] overlaps = UnityEngine.Physics.OverlapBox(testPos + moveDelta, halfExtents, carRot);
                for (int oIdx = 0; oIdx < overlaps.Length; oIdx++)
                {
                    Collider col = overlaps[oIdx];
                    if (col != null && col.transform != transform && !col.transform.IsChildOf(transform))
                    {
                        string cName = col.name;
                        bool isTrackOrGround = cName.Contains("Track_Mesh") || cName.Contains("TrackKerb") ||
                                              cName.Contains("Start_Finish") || cName.Contains("Ground") ||
                                              cName.Contains("Gravel");
                        bool isObstacle = (cName.Contains("Barrier") || cName.Contains("Stand_Backwall") ||
                                          cName.Contains("Overpass_Pillar") || cName.Contains("Guardrail") ||
                                          cName.Contains("Pit_Garage") || cName.Contains("Rail_") ||
                                          cName.Contains("Tecpro")) && !isTrackOrGround;
                        if (isObstacle)
                        {
                            Vector3 closest = col.ClosestPoint(testPos);
                            Vector3 pushDir = testPos - closest;
                            pushDir.y = 0.0f;
                            if (pushDir.sqrMagnitude > 0.0001f)
                            {
                                hitObstacle = true;
                                hitNormal = pushDir.normalized;
                                break;
                            }
                        }
                    }
                }
            }

            if (hitObstacle)
            {
                // Prevent penetrating through barrier: slide along contact plane
                Vector3 slideMove = moveDelta - Vector3.Dot(moveDelta, hitNormal) * hitNormal;
                state.worldPosition += slideMove + hitNormal * 0.04f;

                float speedNorm = worldVelocity.magnitude;
                float impactDot = (speedNorm > 0.1f) ? Mathf.Abs(Vector3.Dot(worldVelocity / speedNorm, hitNormal)) : 0.0f;

                // Reduce forward speed based on impact angle
                state.vx *= Mathf.Clamp01(1.0f - 0.50f * impactDot);
                state.vy = -state.vy * 0.25f; // Restitution & slide damping

                // Deflect yaw glance off the barrier with strong rotational stabilization (prevent helicopter spin)
                Vector3 localNormal = Quaternion.Inverse(carRot) * hitNormal;
                state.yawRate *= Mathf.Clamp01(1.0f - 22.0f * dt);
                float glanceTorque = Mathf.Sign(localNormal.x) * Mathf.Min(1.5f, 0.35f * (speedNorm / 20.0f) * impactDot);
                state.yawRate = Mathf.Clamp(state.yawRate * 0.3f + glanceTorque, -3.0f, 3.0f);

                OnImpact?.Invoke(Mathf.Clamp01(impactDot * (state.vx / 35.0f)));
            }
            else
            {
                state.worldPosition += moveDelta;
            }

            state.yawAngle = Mathf.Repeat(state.yawAngle + state.yawRate * dt + Mathf.PI, Mathf.PI * 2.0f) - Mathf.PI;

            // Update transform
            transform.position = state.worldPosition;
            transform.rotation = Quaternion.Euler(0.0f, -state.yawAngle * Mathf.Rad2Deg + 90.0f, 0.0f);
        }

        private void ComputeNormalLoads(float ax, float ay, float dfFront, float dfRear)
        {
            float g = 9.80665f;
            float m = config.mass;
            float L = config.wheelbase;
            float h = config.cgHeight;

            float fzFStat = m * g * (config.distCgRear / L);
            float fzRStat = m * g * (config.distCgFront / L);

            float dFzLong = m * ax * (h / L);
            float dFzLatF = m * ay * (h / config.trackWidthFront) * config.frontRollStiffness;
            float dFzLatR = m * ay * (h / config.trackWidthRear) * (1.0f - config.frontRollStiffness);

            normalLoads[0] = Mathf.Max(120.0f, 0.5f * fzFStat - 0.5f * dFzLong + 0.5f * dfFront - dFzLatF); // FL
            normalLoads[1] = Mathf.Max(120.0f, 0.5f * fzFStat - 0.5f * dFzLong + 0.5f * dfFront + dFzLatF); // FR
            normalLoads[2] = Mathf.Max(120.0f, 0.5f * fzRStat + 0.5f * dFzLong + 0.5f * dfRear  - dFzLatR); // RL
            normalLoads[3] = Mathf.Max(120.0f, 0.5f * fzRStat + 0.5f * dFzLong + 0.5f * dfRear  + dFzLatR); // RR
        }

        private float[] ComputeBrakeTorques(float brakePedal, float bias)
        {
            float maxTotalTorque = 14800.0f; // Nm
            float tFront = maxTotalTorque * bias * brakePedal * 0.5f;
            float tRear = maxTotalTorque * (1.0f - bias) * brakePedal * 0.5f;
            return new float[] { tFront, tFront, tRear, tRear };
        }

        private float ComputePacejkaMu(float fz, bool isLateral)
        {
            float mu0 = isLateral ? 2.10f : 2.15f;
            float fz0 = 4000.0f; // Reference load (N)
            float loadSensitivity = 0.08f;
            return mu0 * (1.0f - loadSensitivity * ((fz - fz0) / fz0));
        }

        public void SetWorldPosition(Vector3 pos, float yawRad)
        {
            state.worldPosition = pos;
            state.yawAngle = yawRad;
            transform.position = pos;
            transform.rotation = Quaternion.Euler(0.0f, -yawRad * Mathf.Rad2Deg + 90.0f, 0.0f);
        }
    }
}
