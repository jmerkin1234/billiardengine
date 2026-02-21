namespace BilliardsPhysicsEngine.Core;

public readonly struct BallInitState
{
    public int Id { get; }
    public PhysVector3 Position { get; }
    public PhysVector3 Velocity { get; }
    public PhysVector3 AngularVelocity { get; }
    public double Radius { get; }
    public double Mass { get; }
    public bool InPlay { get; }
    public bool IsPocketed { get; }
    public bool IsCueBall { get; }

    public BallInitState(
        int id,
        PhysVector3 position,
        double radius,
        double mass,
        bool isCueBall,
        PhysVector3 velocity,
        PhysVector3 angularVelocity,
        bool inPlay,
        bool isPocketed)
    {
        Id = id;
        Position = position;
        Radius = radius;
        Mass = mass;
        IsCueBall = isCueBall;
        Velocity = velocity;
        AngularVelocity = angularVelocity;
        InPlay = inPlay;
        IsPocketed = isPocketed;
    }

    public static BallInitState Create(int id, PhysVector3 position, double radius, double mass, bool isCueBall)
        => new(id, position, radius, mass, isCueBall, PhysVector3.Zero, PhysVector3.Zero, true, false);
}
