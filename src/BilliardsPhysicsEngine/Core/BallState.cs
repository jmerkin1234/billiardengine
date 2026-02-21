namespace BilliardsPhysicsEngine.Core;

public struct BallState
{
    public int Id;
    public PhysVector3 Position;
    public PhysVector3 Velocity;
    public PhysVector3 AngularVelocity;
    public double Radius;
    public double Mass;
    public bool InPlay;
    public bool IsPocketed;
    public bool IsCueBall;
    public double RestTimer;
    public double SlidingBlend;

    public double HorizontalSpeed
    {
        get
        {
            PhysVector3 horizontal = new(Velocity.X, 0.0, Velocity.Z);
            return horizontal.Length;
        }
    }

    public static BallState FromInit(in BallInitState init)
    {
        return new BallState
        {
            Id = init.Id,
            Position = init.Position,
            Velocity = init.Velocity,
            AngularVelocity = init.AngularVelocity,
            Radius = init.Radius,
            Mass = init.Mass,
            InPlay = init.InPlay,
            IsPocketed = init.IsPocketed,
            IsCueBall = init.IsCueBall,
            RestTimer = 0.0,
            SlidingBlend = 0.0
        };
    }
}
