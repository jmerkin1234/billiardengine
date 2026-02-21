using System.Collections.Generic;
using BilliardsPhysicsEngine.Config;
using BilliardsPhysicsEngine.Core;
using BilliardsPhysicsEngine.Dynamics;
using BilliardsPhysicsEngine.Prediction;

namespace BilliardsPhysicsEngine;

public sealed class BilliardPhysicsEngine
{
    private readonly BilliardsPhysicsWorld _world = new();
    private PhysicsProfile _profile = new();
    private TableGeometry _table;
    private RackPreset? _lastRack;

    public void Initialize(TableGeometry table, PhysicsProfile profile, IReadOnlyList<BallInitState> balls)
    {
        _table = table;
        _profile = profile;
        _world.Initialize(table, profile, balls);
        _lastRack = new RackPreset("Initial", balls as BallInitState[] ?? new List<BallInitState>(balls).ToArray());
    }

    public TableGeometry CreateEightFootTable(double surfaceY = 0.0) => TableGeometry.CreateEightFoot(surfaceY);

    public RackPreset CreateEightBallRack(int cueBallId, IReadOnlyList<int> objectBallIds)
        => RackPreset.EightBall(_table, _profile.BallRadius, _profile.BallMass, cueBallId, objectBallIds);

    public TableGeometry GetTableGeometry() => _table;

    public bool CueStrike(CueShotInput strikeParams) => _world.ApplyCueShot(strikeParams);

    public SimulationFrame Step(double deltaTime)
    {
        _world.Step(deltaTime);
        return _world.GetCurrentFrame();
    }

    public bool IsAtRest() => _world.GetCurrentFrame().IsAtRest;

    public BallState[] GetBallStates() => _world.GetCurrentFrame().Balls;

    public TrajectoryPrediction GetTrajectoryPrediction(CueShotInput shot, PredictionSettings settings)
        => _world.PredictShot(shot, settings);

    public void ResetToLastRack()
    {
        if (_lastRack is not null)
        {
            _world.ResetToRack(_lastRack);
        }
    }

    public void ResetToRack(RackPreset rack)
    {
        _lastRack = rack;
        _world.ResetToRack(rack);
    }

    public bool TrySetBallPosition(int ballId, in PhysVector3 position, bool clearMotion)
        => _world.TrySetBallPosition(ballId, position, clearMotion);

    public bool TryApplyImpulse(int ballId, in PhysVector3 impulse)
        => _world.TryApplyImpulse(ballId, impulse);

    public bool TryInjectSpin(int ballId, in PhysVector3 deltaAngularVelocity)
        => _world.TryInjectSpin(ballId, deltaAngularVelocity);

    public bool TryGetBallState(int ballId, out BallState state)
        => _world.TryGetBallState(ballId, out state);
}
