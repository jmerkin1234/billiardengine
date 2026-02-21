using System;

namespace BilliardsPhysicsEngine.Core;

public struct PhysVector3
{
    public double X;
    public double Y;
    public double Z;

    public PhysVector3(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    public static PhysVector3 Zero => new(0.0, 0.0, 0.0);
    public static PhysVector3 Up => new(0.0, 1.0, 0.0);
    public static PhysVector3 Down => new(0.0, -1.0, 0.0);

    public double Length => Math.Sqrt((X * X) + (Y * Y) + (Z * Z));
    public double LengthSquared => (X * X) + (Y * Y) + (Z * Z);

    public PhysVector3 Normalized
    {
        get
        {
            double len = Length;
            if (len <= 1e-12)
            {
                return Zero;
            }

            return this / len;
        }
    }

    public PhysVector3 WithY(double y) => new(X, y, Z);

    public static double Dot(in PhysVector3 a, in PhysVector3 b) => (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);

    public static PhysVector3 Cross(in PhysVector3 a, in PhysVector3 b)
        => new(
            (a.Y * b.Z) - (a.Z * b.Y),
            (a.Z * b.X) - (a.X * b.Z),
            (a.X * b.Y) - (a.Y * b.X));

    public static PhysVector3 operator +(in PhysVector3 a, in PhysVector3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    public static PhysVector3 operator -(in PhysVector3 a, in PhysVector3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    public static PhysVector3 operator -(in PhysVector3 v) => new(-v.X, -v.Y, -v.Z);
    public static PhysVector3 operator *(in PhysVector3 v, double s) => new(v.X * s, v.Y * s, v.Z * s);
    public static PhysVector3 operator *(double s, in PhysVector3 v) => new(v.X * s, v.Y * s, v.Z * s);
    public static PhysVector3 operator /(in PhysVector3 v, double s) => new(v.X / s, v.Y / s, v.Z / s);
}
