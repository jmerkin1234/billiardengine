namespace BilliardsPhysicsEngine.Core;

public readonly struct CueShotInput
{
    public int BallId { get; }
    public PhysVector3 AimDirection { get; }
    public double LinearImpulse { get; }
    public PhysVector2 TipOffset { get; }
    public double CueElevationDeg { get; }
    public double CueMass { get; }
    public double TipRadius { get; }
    public double SpinMultiplier { get; }

    public CueShotInput(
        int ballId,
        PhysVector3 aimDirection,
        double linearImpulse,
        PhysVector2 tipOffset,
        double cueElevationDeg,
        double cueMass,
        double tipRadius,
        double spinMultiplier)
    {
        BallId = ballId;
        AimDirection = aimDirection;
        LinearImpulse = linearImpulse;
        TipOffset = tipOffset;
        CueElevationDeg = cueElevationDeg;
        CueMass = cueMass;
        TipRadius = tipRadius;
        SpinMultiplier = spinMultiplier;
    }
}
