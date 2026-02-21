using System;

namespace BilliardsPhysicsEngine.Core;

public struct PhysVector2
{
    public double X;
    public double Y;

    public PhysVector2(double x, double y)
    {
        X = x;
        Y = y;
    }

    public static PhysVector2 Zero => new(0.0, 0.0);

    public double Length => Math.Sqrt((X * X) + (Y * Y));
    public double LengthSquared => (X * X) + (Y * Y);

    public PhysVector2 Normalized
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

    public static double Dot(in PhysVector2 a, in PhysVector2 b) => (a.X * b.X) + (a.Y * b.Y);

    public static PhysVector2 operator +(in PhysVector2 a, in PhysVector2 b) => new(a.X + b.X, a.Y + b.Y);
    public static PhysVector2 operator -(in PhysVector2 a, in PhysVector2 b) => new(a.X - b.X, a.Y - b.Y);
    public static PhysVector2 operator -(in PhysVector2 v) => new(-v.X, -v.Y);
    public static PhysVector2 operator *(in PhysVector2 v, double s) => new(v.X * s, v.Y * s);
    public static PhysVector2 operator *(double s, in PhysVector2 v) => new(v.X * s, v.Y * s);
    public static PhysVector2 operator /(in PhysVector2 v, double s) => new(v.X / s, v.Y / s);
}
