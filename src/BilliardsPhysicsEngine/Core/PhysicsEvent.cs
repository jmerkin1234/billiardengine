namespace BilliardsPhysicsEngine.Core;

public enum PhysicsEventType
{
    ShotStarted,
    BallBallContact,
    RailContact,
    Pocketed,
    AllBallsStopped
}

public readonly struct PhysicsEvent
{
    public PhysicsEventType Type { get; }
    public double Time { get; }
    public int BallAId { get; }
    public int BallBId { get; }
    public int PocketId { get; }
    public PhysVector3 Position { get; }
    public PhysVector3 Normal { get; }

    public PhysicsEvent(
        PhysicsEventType type,
        double time,
        int ballAId,
        int ballBId,
        int pocketId,
        PhysVector3 position,
        PhysVector3 normal)
    {
        Type = type;
        Time = time;
        BallAId = ballAId;
        BallBId = ballBId;
        PocketId = pocketId;
        Position = position;
        Normal = normal;
    }
}
