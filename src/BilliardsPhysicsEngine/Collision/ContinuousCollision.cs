using BilliardsPhysicsEngine.Core;

namespace BilliardsPhysicsEngine.Collision;

public static class ContinuousCollision
{
    public static bool TrySphereSphereTimeOfImpact(
        in PhysVector3 posA,
        in PhysVector3 velA,
        double radiusA,
        in PhysVector3 posB,
        in PhysVector3 velB,
        double radiusB,
        double maxTime,
        out double toi)
    {
        PhysVector3 relPos = posB - posA;
        PhysVector3 relVel = velB - velA;
        double radii = radiusA + radiusB;

        double a = PhysVector3.Dot(relVel, relVel);
        double b = 2.0 * PhysVector3.Dot(relPos, relVel);
        double c = PhysVector3.Dot(relPos, relPos) - (radii * radii);

        if (c <= 0.0)
        {
            toi = 0.0;
            return true;
        }

        if (a <= 1e-12)
        {
            toi = double.PositiveInfinity;
            return false;
        }

        double discriminant = (b * b) - (4.0 * a * c);
        if (discriminant < 0.0)
        {
            toi = double.PositiveInfinity;
            return false;
        }

        double sqrtD = System.Math.Sqrt(discriminant);
        double t0 = (-b - sqrtD) / (2.0 * a);
        double t1 = (-b + sqrtD) / (2.0 * a);

        double candidate = double.PositiveInfinity;
        if (t0 >= 0.0)
        {
            candidate = t0;
        }
        else if (t1 >= 0.0)
        {
            candidate = t1;
        }

        if (double.IsInfinity(candidate) || candidate > maxTime)
        {
            toi = double.PositiveInfinity;
            return false;
        }

        toi = candidate;
        return true;
    }
}
