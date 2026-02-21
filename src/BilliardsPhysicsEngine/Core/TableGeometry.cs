using System;

namespace BilliardsPhysicsEngine.Core;

public readonly struct PocketGeometry
{
    public int PocketId { get; }
    public PhysVector3 Position { get; }
    public double MouthRadius { get; }
    public double DropRadius { get; }
    public double ShelfDepth { get; }
    public double JawRestitution { get; }
    public double JawFriction { get; }

    public PocketGeometry(
        int pocketId,
        PhysVector3 position,
        double mouthRadius,
        double dropRadius,
        double shelfDepth,
        double jawRestitution,
        double jawFriction)
    {
        PocketId = pocketId;
        Position = position;
        MouthRadius = mouthRadius;
        DropRadius = dropRadius;
        ShelfDepth = shelfDepth;
        JawRestitution = jawRestitution;
        JawFriction = jawFriction;
    }
}

public readonly struct TableGeometry
{
    public PhysVector2 MinXZ { get; }
    public PhysVector2 MaxXZ { get; }
    public double SurfaceY { get; }
    public double CushionNoseHeight { get; }
    public PocketGeometry[] Pockets { get; }

    public TableGeometry(PhysVector2 minXZ, PhysVector2 maxXZ, double surfaceY, double cushionNoseHeight, PocketGeometry[] pockets)
    {
        MinXZ = minXZ;
        MaxXZ = maxXZ;
        SurfaceY = surfaceY;
        CushionNoseHeight = cushionNoseHeight;
        Pockets = pockets ?? Array.Empty<PocketGeometry>();
    }

    public double Width => MaxXZ.X - MinXZ.X;
    public double Length => MaxXZ.Y - MinXZ.Y;

    public PhysVector3 ClampBallCenter(in PhysVector3 position, double radius)
    {
        double x = Math.Clamp(position.X, MinXZ.X + radius, MaxXZ.X - radius);
        double z = Math.Clamp(position.Z, MinXZ.Y + radius, MaxXZ.Y - radius);
        return new PhysVector3(x, SurfaceY + radius, z);
    }

    public static TableGeometry CreateEightFoot(double surfaceY)
    {
        double width = 1.1176;
        double length = 2.2352;
        double halfW = width * 0.5;
        double halfL = length * 0.5;

        PocketGeometry[] pockets =
        {
            new(0, new PhysVector3(-halfW, surfaceY, -halfL), 0.080, 0.050, 0.020, 0.70, 0.25),
            new(1, new PhysVector3(0.0,   surfaceY, -halfL), 0.085, 0.052, 0.020, 0.70, 0.25),
            new(2, new PhysVector3(halfW, surfaceY, -halfL), 0.080, 0.050, 0.020, 0.70, 0.25),
            new(3, new PhysVector3(-halfW, surfaceY, halfL), 0.080, 0.050, 0.020, 0.70, 0.25),
            new(4, new PhysVector3(0.0,    surfaceY, halfL), 0.085, 0.052, 0.020, 0.70, 0.25),
            new(5, new PhysVector3(halfW,  surfaceY, halfL), 0.080, 0.050, 0.020, 0.70, 0.25)
        };

        return new TableGeometry(new PhysVector2(-halfW, -halfL), new PhysVector2(halfW, halfL), surfaceY, 0.037, pockets);
    }
}
