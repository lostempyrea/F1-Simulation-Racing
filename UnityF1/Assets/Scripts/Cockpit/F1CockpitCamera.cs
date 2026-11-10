using UnityEngine;
using F1Sim.Physics;

namespace F1Sim.Cockpit
{
    public enum F1CameraMode
    {
        CockpitEye = 0,
        BroadcastTCam = 1,
        ChaseView = 2
    }

    /// <summary>
    /// Implements authentic Formula 1 cockpit camera with:
    /// - 2nd-order biomechanical neck spring-damper reacting to longitudinal & lateral G-forces
    /// - High-speed buffeting camera vibration (>280-350 km/h Monza straight speeds)
    /// - Kerb rumble vibration & roll kick when riding curb serrations
    /// - Dynamic optical FOV expansion with speed
    /// - Unobstructed forward perspective matching contemporary F1 onboard cameras
    /// </summary>
    public class F1CockpitCamera : MonoBehaviour
    {
        [Header("Target & Camera Mode")]
        public VehicleDynamics vehicle;
        public F1CameraMode cameraMode = F1CameraMode.CockpitEye;
        public bool isMenuOrbitActive = true;

        [Header("Camera Presets (Local to Car)")]
        public Vector3 cockpitEyeOffset = new Vector3(0.0f, 0.72f, -0.28f);
        public Vector3 broadcastTCamOffset = new Vector3(0.0f, 0.98f, -0.10f);
        public Vector3 chaseCamOffset = new Vector3(0.0f, 1.55f, -4.60f);

        [Header("Optical Parameters")]
        public Camera mainCamera;
        public float baseFov = 80.0f;
        public float maxFovDelta = 12.0f;

        [Header("Neck Kinematics (2nd-Order Spring-Damper)")]
        public float omegaN = 22.0f;
        public float dampingRatio = 0.72f;
        public float longGCompliance = 0.009f;    // 9 mm / G
        public float latGCompliance = 0.008f;     // 8 mm / G
        public float maxOffsetM = 0.045f;         // 45 mm stop

        [Header("Head Pitch & Roll")]
        public float pitchPerLongGDeg = 1.15f;
        public float rollPerLatGDeg = 2.60f;

        [Header("Kerb Vibration")]
        public float kerbFreqHz = 28.0f;
        public float kerbAmpM = 0.008f;

        [Header("High-Speed Buffeting (>280 - 360 km/h)")]
        public float highSpeedThresholdKmh = 275.0f;
        public float highSpeedMaxShakeM = 0.0045f;
        public float highSpeedMaxRollDeg = 0.55f;

        private Vector3 neckOffset = Vector3.zero;
        private Vector3 neckVelocity = Vector3.zero;
        private float phaseTime = 0.0f;
        private float orbitAngleDeg = 28.0f;

        private void Start()
        {
            if (mainCamera == null) mainCamera = GetComponent<Camera>();
        }

        public void CycleCameraMode()
        {
            int next = ((int)cameraMode + 1) % 3;
            cameraMode = (F1CameraMode)next;
        }

        public void TriggerImpact(float intensity)
        {
            float mag = Mathf.Clamp01(intensity);
            neckVelocity += new Vector3(
                Random.Range(-0.45f, 0.45f) * mag,
                Random.Range(0.20f, 0.60f) * mag,
                -0.60f * mag
            );
        }

        private void LateUpdate()
        {
            if (vehicle == null) return;

            Transform carTr = vehicle.transform;

            // Cinematic Orbit Camera while in Main Menu / Settings
            if (isMenuOrbitActive)
            {
                orbitAngleDeg += Time.unscaledDeltaTime * 8.5f;
                float rad = orbitAngleDeg * Mathf.Deg2Rad;
                float radius = 5.4f;
                float height = 1.65f + Mathf.Sin(Time.unscaledTime * 0.6f) * 0.2f;

                Vector3 localOrbit = new Vector3(Mathf.Sin(rad) * radius, height, Mathf.Cos(rad) * radius);
                Vector3 worldCamPos = carTr.position + carTr.TransformDirection(localOrbit);
                Vector3 lookTarget = carTr.position + carTr.TransformDirection(new Vector3(0.0f, 0.55f, 0.35f));

                transform.position = worldCamPos;
                transform.LookAt(lookTarget, Vector3.up);
                if (mainCamera != null) mainCamera.fieldOfView = 52.0f;
                return;
            }

            float dt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.05f);
            phaseTime += dt;

            // 1. Dynamic FOV expansion with speed (sensation of blistering acceleration)
            float vRatio = Mathf.Clamp01(vehicle.SpeedKmh / 360.0f);
            float targetFov = baseFov + maxFovDelta * Mathf.Pow(vRatio, 1.4f);
            if (mainCamera != null)
            {
                mainCamera.fieldOfView = Mathf.Lerp(mainCamera.fieldOfView, targetFov, dt * 10.0f);
            }

            // 2. Neck Spring-Damper Simulation
            float ax = vehicle.LongitudinalG;
            float ay = vehicle.LateralG;

            float targetX = Mathf.Clamp(-ay * latGCompliance, -maxOffsetM, maxOffsetM);
            float targetZ = Mathf.Clamp(-ax * longGCompliance, -maxOffsetM, maxOffsetM);

            // Kerb rumble vibration
            float vibeY = 0.0f;
            float kerbRoll = 0.0f;
            if (vehicle.KerbVibration > 0.01f)
            {
                vibeY = kerbAmpM * Mathf.Sin(2.0f * Mathf.PI * kerbFreqHz * phaseTime);
                kerbRoll = Mathf.Sin(2.0f * Mathf.PI * (kerbFreqHz * 0.5f) * phaseTime) * 1.6f;
            }

            // High-speed aerodynamic buffeting shake (>280 km/h up to 360 km/h)
            float speedKmh = vehicle.SpeedKmh;
            float speedVibeFactor = Mathf.Clamp01((speedKmh - highSpeedThresholdKmh) / 75.0f);
            float speedShakeX = (Mathf.PerlinNoise(phaseTime * 32.0f, 0.0f) - 0.5f) * highSpeedMaxShakeM * speedVibeFactor;
            float speedShakeY = (Mathf.PerlinNoise(0.0f, phaseTime * 36.0f) - 0.5f) * highSpeedMaxShakeM * speedVibeFactor;
            float speedShakeRoll = (Mathf.PerlinNoise(phaseTime * 22.0f, 15.0f) - 0.5f) * highSpeedMaxRollDeg * speedVibeFactor;

            float targetY = Mathf.Clamp(vibeY + speedShakeY, -maxOffsetM, maxOffsetM);

            Vector3 targetVec = new Vector3(targetX + speedShakeX, targetY, targetZ);
            Vector3 dispErr = neckOffset - targetVec;
            Vector3 accel = -dispErr * (omegaN * omegaN) - neckVelocity * (2.0f * dampingRatio * omegaN);
            neckVelocity += accel * dt;
            neckOffset += neckVelocity * dt;

            // 3. Angular Head Pitch & Cornering Lean Roll
            float pitchDeg = Mathf.Clamp(-ax * pitchPerLongGDeg, -6.5f, 6.5f);
            float rollDeg = Mathf.Clamp(ay * rollPerLatGDeg + kerbRoll + speedShakeRoll, -9.5f, 9.5f);
            float yawDeg = Mathf.Clamp(-vehicle.state.steeringAngle * Mathf.Rad2Deg * 0.12f, -4.5f, 4.5f);

            // 4. Transform Placement by Camera Mode
            Vector3 baseOffset = cockpitEyeOffset;
            float basePitchDown = 4.5f; // tuned F1 cockpit perspective
            if (cameraMode == F1CameraMode.BroadcastTCam)
            {
                baseOffset = broadcastTCamOffset;
                basePitchDown = 6.5f;
            }
            else if (cameraMode == F1CameraMode.ChaseView)
            {
                baseOffset = chaseCamOffset;
                basePitchDown = 8.0f;
                rollDeg *= 0.35f;
            }

            Vector3 worldEye = carTr.position + carTr.TransformDirection(baseOffset + neckOffset);
            transform.position = worldEye;
            transform.rotation = carTr.rotation * Quaternion.Euler(basePitchDown + pitchDeg, yawDeg, rollDeg);
        }
    }
}
