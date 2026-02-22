using System.Collections.Generic;
using BilliardsPhysicsEngine.Core;

namespace BilliardsPhysicsEngine.Prediction;

public sealed class BallTrajectory
{
    public int BallId { get; init; }
    public PhysVector3[] Points { get; init; } = [];
    public bool Pocketed { get; init; }
}

public sealed class TrajectoryPrediction
{
    private readonly Dictionary<int, List<PhysVector3>> _pointsByBall = new();
    private readonly HashSet<int> _pocketed = new();

    public double SimulatedSeconds { get; set; }
    public int FirstContactBallId { get; set; } = -1;
    public PhysVector3 FirstContactPoint { get; private set; } = PhysVector3.Zero;
    public double FirstContactTimeSeconds { get; private set; } = -1.0;
    public bool HasFirstContact => FirstContactBallId >= 0 && FirstContactTimeSeconds >= 0.0;

    public void AddPoint(int ballId, in PhysVector3 point)
    {
        if (!_pointsByBall.TryGetValue(ballId, out List<PhysVector3>? points))
        {
            points = new List<PhysVector3>(256);
            _pointsByBall[ballId] = points;
        }

        points.Add(point);
    }

    public void SetPocketed(int ballId) => _pocketed.Add(ballId);

    public bool IsPocketed(int ballId) => _pocketed.Contains(ballId);

    public void SetFirstContact(int ballId, in PhysVector3 point, double timeSeconds)
    {
        if (HasFirstContact)
        {
            return;
        }

        FirstContactBallId = ballId;
        FirstContactPoint = point;
        FirstContactTimeSeconds = timeSeconds;
    }

    public bool TryGetPoints(int ballId, out IReadOnlyList<PhysVector3> points)
    {
        if (_pointsByBall.TryGetValue(ballId, out List<PhysVector3>? list))
        {
            points = list;
            return true;
        }

        points = [];
        return false;
    }

    public BallTrajectory[] ToArray()
    {
        BallTrajectory[] result = new BallTrajectory[_pointsByBall.Count];
        int index = 0;
        foreach ((int ballId, List<PhysVector3> points) in _pointsByBall)
        {
            result[index++] = new BallTrajectory
            {
                BallId = ballId,
                Points = points.ToArray(),
                Pocketed = _pocketed.Contains(ballId)
            };
        }

        return result;
    }
}
