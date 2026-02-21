using BilliardsPhysicsEngine.Core;
using UnityEngine;

namespace BilliardsUnityAdapter.Runtime;

/// <summary>
/// Shot adapter from UI/aim systems to CueShotInput.
/// </summary>
public class UnityCueStrikeAdapter : MonoBehaviour
{
    [SerializeField] private UnityPhysicsRuntimeController? runtimeController;
    [SerializeField] private GameObject? cueBall;
    [SerializeField] private float powerToImpulseScale = 0.13f;
    [SerializeField] private float spinMultiplier = 18f;
    [SerializeField] private float cueElevationDeg = 0f;
    [SerializeField] private float cueMass = 0.525f;
    [SerializeField] private float tipRadius = 0.006f;

    [Header("Contact Offset")]
    [SerializeField] private float verticalOffset;
    [SerializeField] private float horizontalOffset;
    [SerializeField] private float maxOffsetFraction = 0.7f;

    private const float BallRadius = 0.028575f;

    private void Awake()
    {
        runtimeController ??= UnityPhysicsRuntimeController.Instance;
        if (cueBall is null)
        {
            GameObject found = GameObject.FindGameObjectWithTag("CueBall");
            if (found is not null)
            {
                cueBall = found;
            }
        }
    }

    public bool SubmitShot(Vector3 aimDirection, float gameplayPower)
    {
        if (runtimeController is null || cueBall is null)
        {
            return false;
        }

        if (!runtimeController.TryGetBallId(cueBall, out int ballId))
        {
            return false;
        }

        double maxOffset = BallRadius * maxOffsetFraction;
        double tipX = Mathf.Clamp(horizontalOffset, (float)-maxOffset, (float)maxOffset);
        double tipY = Mathf.Clamp(verticalOffset, (float)-maxOffset, (float)maxOffset);

        CueShotInput input = new(
            ballId,
            new PhysVector3(aimDirection.x, aimDirection.y, aimDirection.z),
            Mathf.Max(0f, gameplayPower * powerToImpulseScale),
            new PhysVector2(tipX, tipY),
            cueElevationDeg,
            cueMass,
            tipRadius,
            spinMultiplier);

        return runtimeController.ApplyCueShot(input);
    }

    public void SetContactOffset(float vertical, float horizontal)
    {
        verticalOffset = vertical;
        horizontalOffset = horizontal;
    }

    public void ResetContactOffset()
    {
        verticalOffset = 0f;
        horizontalOffset = 0f;
    }
}
