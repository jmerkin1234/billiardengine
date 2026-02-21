using System;

namespace BilliardsPhysicsEngine.Core;

public readonly struct SimulationFrame
{
    public double SimulationTime { get; }
    public BallState[] Balls { get; }
    public PhysicsEvent[] Events { get; }
    public bool IsAtRest { get; }
    public int MovingBallCount { get; }

    public SimulationFrame(double simulationTime, BallState[] balls, PhysicsEvent[] events, bool isAtRest, int movingBallCount)
    {
        SimulationTime = simulationTime;
        Balls = balls;
        Events = events;
        IsAtRest = isAtRest;
        MovingBallCount = movingBallCount;
    }

    public static SimulationFrame Empty => new(0.0, Array.Empty<BallState>(), Array.Empty<PhysicsEvent>(), true, 0);
}
