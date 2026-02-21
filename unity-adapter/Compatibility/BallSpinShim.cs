using BilliardsPhysicsEngine.Core;
using BilliardsUnityAdapter.Runtime;
using UnityEngine;

namespace BilliardsUnityAdapter.Compatibility;

/// <summary>
/// Compatibility shim for reading spin from engine state.
/// </summary>
public class BallSpinShim : MonoBehaviour
{
    public float SideSpin
    {
        get
        {
            if (TryGetState(out BallState state))
            {
                return (float)state.AngularVelocity.Y;
            }

            return 0f;
        }
    }

    public Vector3 AngularVelocity
    {
        get
        {
            if (TryGetState(out BallState state))
            {
                return new Vector3((float)state.AngularVelocity.X, (float)state.AngularVelocity.Y, (float)state.AngularVelocity.Z);
            }

            return Vector3.zero;
        }
    }

    private bool TryGetState(out BallState state)
    {
        state = default;
        return UnityPhysicsRuntimeController.Instance is not null &&
               UnityPhysicsRuntimeController.Instance.TryGetBallState(gameObject, out state);
    }
}
