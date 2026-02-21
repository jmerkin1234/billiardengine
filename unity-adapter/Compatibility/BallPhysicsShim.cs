using BilliardsPhysicsEngine.Core;
using BilliardsUnityAdapter.Runtime;
using UnityEngine;

namespace BilliardsUnityAdapter.Compatibility;

/// <summary>
/// Compatibility shim mirroring legacy BallPhysics responsibilities while runtime controller owns simulation.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class BallPhysicsShim : MonoBehaviour
{
    [SerializeField] private bool allowLegacyFallback = true;

    private Rigidbody? _rb;

    public Vector3 Velocity
    {
        get
        {
            if (TryGetEngineState(out BallState state))
            {
                return new Vector3((float)state.Velocity.X, (float)state.Velocity.Y, (float)state.Velocity.Z);
            }

            return _rb is null ? Vector3.zero : _rb.linearVelocity;
        }
    }

    public Vector3 AngularVelocity
    {
        get
        {
            if (TryGetEngineState(out BallState state))
            {
                return new Vector3((float)state.AngularVelocity.X, (float)state.AngularVelocity.Y, (float)state.AngularVelocity.Z);
            }

            return _rb is null ? Vector3.zero : _rb.angularVelocity;
        }
    }

    private void Awake()
    {
        _rb = GetComponent<Rigidbody>();
    }

    public void ApplyImpulse(Vector3 impulse)
    {
        if (UnityPhysicsRuntimeController.Instance is not null && UnityPhysicsRuntimeController.Instance.TryApplyImpulse(gameObject, impulse))
        {
            return;
        }

        if (allowLegacyFallback && _rb is not null)
        {
            _rb.AddForce(impulse, ForceMode.Impulse);
        }
    }

    public void StopMotion()
    {
        if (_rb is not null)
        {
            _rb.linearVelocity = Vector3.zero;
            _rb.angularVelocity = Vector3.zero;
        }
    }

    private bool TryGetEngineState(out BallState state)
    {
        state = default;
        return UnityPhysicsRuntimeController.Instance is not null &&
               UnityPhysicsRuntimeController.Instance.TryGetBallState(gameObject, out state);
    }
}
