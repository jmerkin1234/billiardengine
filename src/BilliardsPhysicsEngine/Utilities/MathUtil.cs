using System;

namespace BilliardsPhysicsEngine.Utilities;

public static class MathUtil
{
    public static double Clamp(double value, double min, double max)
    {
        if (value < min)
        {
            return min;
        }

        if (value > max)
        {
            return max;
        }

        return value;
    }

    public static double Clamp01(double value) => Clamp(value, 0.0, 1.0);

    public static double SafeSqrt(double value) => value <= 0.0 ? 0.0 : Math.Sqrt(value);

    public static double DegToRad(double degrees) => degrees * (Math.PI / 180.0);
    public static double RadToDeg(double radians) => radians * (180.0 / Math.PI);
}
