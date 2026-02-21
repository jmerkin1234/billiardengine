using BilliardsPhysicsEngine.Core;
using BilliardsPhysicsEngine.Prediction;
using BilliardsUnityAdapter.Runtime;
using UnityEngine;

namespace BilliardsUnityAdapter.Training;

public class GhostPathRenderer : MonoBehaviour
{
    [SerializeField] private UnityPhysicsRuntimeController? runtimeController;
    [SerializeField] private int shotBallId = 0;
    [SerializeField] private Vector3 aimDirection = new(0f, 0f, -1f);
    [SerializeField] private float impulse = 0.65f;
    [SerializeField] private Vector2 tipOffset = Vector2.zero;
    [SerializeField] private float refreshInterval = 0.2f;
    [SerializeField] private float horizonSeconds = 4f;

    private LineRenderer? _line;
    private float _nextRefresh;

    private void Awake()
    {
        runtimeController ??= UnityPhysicsRuntimeController.Instance;
        _line = GetComponent<LineRenderer>();
        if (_line is null)
        {
            _line = gameObject.AddComponent<LineRenderer>();
            _line.material = new Material(Shader.Find("Sprites/Default"));
            _line.startWidth = 0.01f;
            _line.endWidth = 0.01f;
        }
    }

    private void LateUpdate()
    {
        if (runtimeController is null || _line is null || Time.time < _nextRefresh)
        {
            return;
        }

        _nextRefresh = Time.time + refreshInterval;

        CueShotInput shot = new(
            shotBallId,
            new PhysVector3(aimDirection.x, aimDirection.y, aimDirection.z),
            impulse,
            new PhysVector2(tipOffset.x, tipOffset.y),
            0.0,
            0.525,
            0.006,
            18.0);

        PredictionSettings settings = new(horizonSeconds, 1.0 / 90.0, 256, false);
        TrajectoryPrediction prediction = runtimeController.PredictShot(shot, settings);

        if (!prediction.TryGetPoints(shotBallId, out var points) || points.Count < 2)
        {
            _line.positionCount = 0;
            return;
        }

        _line.positionCount = points.Count;
        for (int i = 0; i < points.Count; i++)
        {
            _line.SetPosition(i, new Vector3((float)points[i].X, (float)points[i].Y + 0.002f, (float)points[i].Z));
        }
    }
}
