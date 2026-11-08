using System;
using UnityEngine;
using F1Sim.Physics;

namespace F1Sim.Audio
{
    [RequireComponent(typeof(AudioSource))]
    public class F1AudioSynthesizer : MonoBehaviour
    {
        [Header("Engine Target")]
        public VehicleDynamics vehicle;

        [Header("Audio Settings")]
        public AudioSource audioSource;
        [Range(0.0f, 1.0f)] public float masterVolume = 0.80f;
        [Range(0.0f, 1.0f)] public float engineVolume = 0.85f;
        [Range(0.0f, 1.0f)] public float turboVolume = 0.70f;
        [Range(0.0f, 1.0f)] public float sfxVolume = 0.85f;
        public bool isMuted = false;
        public float sampleRate = 44100.0f;

        [Header("One-Shot Sound Effects")]
        public AudioClip upshiftPopClip;
        public AudioClip downshiftBlipClip;
        public AudioClip uiClickClip;

        private AudioSource sfxSource;
        private double phase0 = 0.0;
        private double phase1 = 0.0;
        private double phase2 = 0.0;
        private double turboPhase = 0.0;

        private void Awake()
        {
            audioSource = GetComponent<AudioSource>();
            if (audioSource != null)
            {
                audioSource.playOnAwake = true;
                audioSource.loop = true;
                audioSource.spatialBlend = 0.0f;
            }

            // Separate child GameObject for one-shot SFX so OnAudioFilterRead never conflicts
            Transform childSfx = transform.Find("SFX_Audio_Channel");
            GameObject sfxObj = (childSfx != null) ? childSfx.gameObject : new GameObject("SFX_Audio_Channel");
            sfxObj.transform.SetParent(transform, false);
            sfxSource = sfxObj.GetComponent<AudioSource>();
            if (sfxSource == null) sfxSource = sfxObj.AddComponent<AudioSource>();
            sfxSource.playOnAwake = false;
            sfxSource.spatialBlend = 0.0f;

            SynthesizePlaceholderClips();
        }

        private void Start()
        {
            if (vehicle != null && vehicle.powertrain != null)
            {
                vehicle.powertrain.OnShiftUp += PlayUpshiftPop;
                vehicle.powertrain.OnShiftDown += PlayDownshiftBlip;
            }
        }

        private void SynthesizePlaceholderClips()
        {
            int sr = 44100;
            // 1. Upshift exhaust pop (65 ms low-passed impulse + pitch drop)
            if (upshiftPopClip == null)
            {
                int len = Mathf.RoundToInt(sr * 0.065f);
                float[] samples = new float[len];
                for (int i = 0; i < len; i++)
                {
                    float t = i / (float)len;
                    float env = Mathf.Sin(Mathf.Min(1.0f, i / 180.0f) * Mathf.PI * 0.5f) * Mathf.Exp(-t * 6.5f);
                    float freq = Mathf.Lerp(180.0f, 75.0f, t);
                    samples[i] = Mathf.Sin(2.0f * Mathf.PI * freq * (i / (float)sr)) * env * 0.6f;
                }
                upshiftPopClip = AudioClip.Create("Synth_UpshiftPop", len, 1, sr, false);
                upshiftPopClip.SetData(samples, 0);
            }

            // 2. Downshift throttle blip (85 ms rev chirp)
            if (downshiftBlipClip == null)
            {
                int len = Mathf.RoundToInt(sr * 0.085f);
                float[] samples = new float[len];
                for (int i = 0; i < len; i++)
                {
                    float t = i / (float)len;
                    float env = Mathf.Sin(t * Mathf.PI);
                    float freq = Mathf.Lerp(320.0f, 540.0f, Mathf.Sin(t * Mathf.PI));
                    samples[i] = Mathf.Sin(2.0f * Mathf.PI * freq * (i / (float)sr)) * env * 0.5f;
                }
                downshiftBlipClip = AudioClip.Create("Synth_DownshiftBlip", len, 1, sr, false);
                downshiftBlipClip.SetData(samples, 0);
            }
        }

        private void OnAudioFilterRead(float[] data, int channels)
        {
            if (isMuted || masterVolume <= 0.001f || vehicle == null)
            {
                Array.Clear(data, 0, data.Length);
                return;
            }

            float rpm = vehicle.state.rpm;
            float throttle = vehicle.state.throttle;

            double f0 = Math.Max(50.0, rpm / 20.0);
            double f1 = 2.0 * f0;
            double f2 = 3.0 * f0;
            double fTurbo = f0 * 5.5;

            double inc0 = (2.0 * Math.PI * f0) / sampleRate;
            double inc1 = (2.0 * Math.PI * f1) / sampleRate;
            double inc2 = (2.0 * Math.PI * f2) / sampleRate;
            double incTurbo = (2.0 * Math.PI * fTurbo) / sampleRate;

            float gain = (0.28f + 0.55f * throttle) * masterVolume;

            for (int i = 0; i < data.Length; i += channels)
            {
                phase0 += inc0;
                phase1 += inc1;
                phase2 += inc2;
                turboPhase += incTurbo;

                if (phase0 > 2.0 * Math.PI) phase0 -= 2.0 * Math.PI;
                if (phase1 > 2.0 * Math.PI) phase1 -= 2.0 * Math.PI;
                if (phase2 > 2.0 * Math.PI) phase2 -= 2.0 * Math.PI;
                if (turboPhase > 2.0 * Math.PI) turboPhase -= 2.0 * Math.PI;

                float wave = (float)(
                    (0.52 * Math.Sin(phase0) + 0.28 * Math.Sin(phase1 + 0.3) + 0.14 * Math.Sin(phase2 + 0.7)) * engineVolume
                    + (0.08 * throttle * Math.Sin(turboPhase)) * turboVolume
                );

                float sample = Mathf.Clamp(wave * gain, -1.0f, 1.0f);

                for (int c = 0; c < channels; c++)
                {
                    data[i + c] = sample;
                }
            }
        }

        public void PlayUpshiftPop()
        {
            if (!isMuted && upshiftPopClip != null && sfxSource != null)
                sfxSource.PlayOneShot(upshiftPopClip, 0.75f * masterVolume * sfxVolume);
        }

        public void PlayDownshiftBlip()
        {
            if (!isMuted && downshiftBlipClip != null && sfxSource != null)
                sfxSource.PlayOneShot(downshiftBlipClip, 0.70f * masterVolume * sfxVolume);
        }
    }
}
