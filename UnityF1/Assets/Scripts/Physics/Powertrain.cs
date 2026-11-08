using System;
using UnityEngine;

namespace F1Sim.Physics
{
    public enum TransmissionMode
    {
        Automatic,
        Manual
    }

    public class Powertrain : MonoBehaviour
    {
        [Header("Engine Configuration")]
        public float idleRpm = 4000.0f;
        public float redlineRpm = 15000.0f;
        public float upshiftRpm = 11800.0f;
        public float downshiftRpm = 6200.0f;
        public float maxEngineTorque = 420.0f;    // ICE Nm peak @ 10,500 RPM
        public float mgukTorque = 250.0f;          // 120 kW MGU-K instant torque
        public float finalDriveRatio = 3.85f;

        [Header("Gear Ratios (1st to 8th + R)")]
        public float[] gearRatios = new float[9]
        {
            -3.20f, // R (index 0)
            3.40f,  // 1st (index 1)
            2.65f,  // 2nd (index 2)
            2.15f,  // 3rd (index 3)
            1.78f,  // 4th (index 4)
            1.52f,  // 5th (index 5)
            1.32f,  // 6th (index 6)
            1.16f,  // 7th (index 7)
            1.04f   // 8th (index 8)
        };

        [Header("Transmission Settings")]
        public TransmissionMode transmissionMode = TransmissionMode.Automatic;
        public int currentGear = 1;
        public float currentRpm = 4000.0f;
        public bool isRunning = true;

        public event Action OnShiftUp;
        public event Action OnShiftDown;

        public float Rpm => currentRpm;
        public int CurrentGear => currentGear;

        public void Step(float throttle, float vx, float dt)
        {
            if (!isRunning)
            {
                currentRpm = Mathf.MoveTowards(currentRpm, 0.0f, 2000.0f * dt);
                return;
            }

            // Target RPM derived from wheel speed and current gear ratio
            float ratio = (currentGear >= 1 && currentGear <= 8) ? gearRatios[currentGear] * finalDriveRatio : 0.0f;
            float wheelRadius = 0.360f;
            float targetRpm = (vx / wheelRadius) * ratio * (60.0f / (2.0f * Mathf.PI));

            if (currentGear == 0) // Neutral
            {
                float idleTarget = idleRpm + throttle * (redlineRpm - idleRpm);
                currentRpm = Mathf.MoveTowards(currentRpm, idleTarget, 15000.0f * dt);
            }
            else
            {
                currentRpm = Mathf.Clamp(targetRpm, idleRpm, redlineRpm);
            }

            // Automatic gearshift logic
            if (transmissionMode == TransmissionMode.Automatic)
            {
                if (currentRpm >= upshiftRpm && currentGear < 8)
                {
                    ShiftUp();
                }
                else if (currentRpm <= downshiftRpm && currentGear > 1 && vx > 2.0f)
                {
                    ShiftDown();
                }
                else if (currentGear == 0 && throttle > 0.05f)
                {
                    currentGear = 1;
                }
            }
        }

        public float GetRearAxleTorque()
        {
            if (!isRunning || currentGear <= 0 || currentGear > 8)
                return 0.0f;

            // Power curve: peak torque between 9,000 and 12,000 RPM + instant MGU-K boost
            float rpmFrac = Mathf.Clamp01((currentRpm - idleRpm) / (redlineRpm - idleRpm));
            float iceTorque = maxEngineTorque * (0.65f + 0.35f * Mathf.Sin(rpmFrac * Mathf.PI));
            float totalTorque = (iceTorque + mgukTorque) * gearRatios[currentGear] * finalDriveRatio;
            return totalTorque;
        }

        public bool ShiftUp()
        {
            if (currentGear < 8)
            {
                currentGear++;
                OnShiftUp?.Invoke();
                return true;
            }
            return false;
        }

        public bool ShiftDown()
        {
            if (currentGear > 1)
            {
                currentGear--;
                OnShiftDown?.Invoke();
                return true;
            }
            else if (currentGear == 1)
            {
                currentGear = 0; // Neutral
                return true;
            }
            return false;
        }

        public void ToggleTransmissionMode()
        {
            transmissionMode = (transmissionMode == TransmissionMode.Automatic) ? TransmissionMode.Manual : TransmissionMode.Automatic;
        }
    }
}
