using System;
using BilliardsPhysicsEngine.Core;
using BilliardsUnityAdapter.Runtime;
using UnityEngine;

namespace BilliardsUnityAdapter.Training;

public class TrainingModeController : MonoBehaviour
{
    [Serializable]
    private struct TrainingShotPreset
    {
        public string Name;
        public int BallId;
        public Vector3 AimDirection;
        public float Impulse;
        public Vector2 TipOffset;
        public float CueElevation;
    }

    [SerializeField] private UnityPhysicsRuntimeController? runtimeController;
    [SerializeField] private KeyCode resetRackKey = KeyCode.R;
    [SerializeField] private KeyCode repeatShotKey = KeyCode.T;
    [SerializeField] private bool hotkeysEnabled = true;
    [SerializeField] private TrainingShotPreset[] presets = Array.Empty<TrainingShotPreset>();

    private void Awake()
    {
        runtimeController ??= UnityPhysicsRuntimeController.Instance;
    }

    private void Update()
    {
        if (!hotkeysEnabled || runtimeController is null)
        {
            return;
        }

        if (Input.GetKeyDown(resetRackKey))
        {
            runtimeController.ResetToDefaultRack();
        }

        if (Input.GetKeyDown(repeatShotKey))
        {
            runtimeController.RepeatLastShot();
        }

        for (int i = 0; i < Mathf.Min(9, presets.Length); i++)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1 + i))
            {
                FirePreset(i);
                break;
            }
        }
    }

    private void FirePreset(int i)
    {
        if (runtimeController is null || i < 0 || i >= presets.Length)
        {
            return;
        }

        TrainingShotPreset preset = presets[i];
        CueShotInput shot = new(
            preset.BallId,
            new PhysVector3(preset.AimDirection.x, preset.AimDirection.y, preset.AimDirection.z),
            preset.Impulse,
            new PhysVector2(preset.TipOffset.x, preset.TipOffset.y),
            preset.CueElevation,
            0.525,
            0.006,
            18.0);

        runtimeController.ApplyCueShot(shot);
    }
}
