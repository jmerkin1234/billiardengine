using System;
using UnityEngine;

namespace BilliardsUnityAdapter.Compatibility;

/// <summary>
/// Legacy-compatible pocket event surface.
/// Runtime controller emits pocket notifications; this shim rebroadcasts GameObject+pocket data.
/// </summary>
public class PocketTriggerShim : MonoBehaviour
{
    public static event Action<GameObject, int>? OnBallPocketed;

    private void OnEnable()
    {
        Runtime.UnityPhysicsRuntimeController.OnBallPocketed += HandlePocketed;
    }

    private void OnDisable()
    {
        Runtime.UnityPhysicsRuntimeController.OnBallPocketed -= HandlePocketed;
    }

    private static void HandlePocketed(GameObject ball, int pocketId)
    {
        OnBallPocketed?.Invoke(ball, pocketId);
    }
}
