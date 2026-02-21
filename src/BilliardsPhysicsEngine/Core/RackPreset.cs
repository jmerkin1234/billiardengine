using System;
using System.Collections.Generic;

namespace BilliardsPhysicsEngine.Core;

public sealed class RackPreset
{
    public string Name { get; }
    public BallInitState[] InitialStates { get; }

    public RackPreset(string name, BallInitState[] initialStates)
    {
        Name = string.IsNullOrWhiteSpace(name) ? "Rack" : name;
        InitialStates = initialStates ?? Array.Empty<BallInitState>();
    }

    public static RackPreset EightBall(TableGeometry table, double radius, double mass, int cueBallId, IReadOnlyList<int> objectBallIds)
    {
        List<BallInitState> states = new(16);
        PhysVector3 cuePosition = new(0.0, table.SurfaceY + radius, table.Length * 0.25);
        states.Add(BallInitState.Create(cueBallId, cuePosition, radius, mass, true));

        double rowSpacing = radius * 2.0 * 0.99;
        double colSpacing = radius * 2.0 * 1.02;
        PhysVector3 apex = new(0.0, table.SurfaceY + radius, -table.Length * 0.25);

        int idIndex = 0;
        for (int row = 0; row < 5; row++)
        {
            int ballsThisRow = row + 1;
            double rowZ = apex.Z - (row * rowSpacing);
            double xStart = -((ballsThisRow - 1) * 0.5 * colSpacing);

            for (int c = 0; c < ballsThisRow; c++)
            {
                if (idIndex >= objectBallIds.Count)
                {
                    break;
                }

                int id = objectBallIds[idIndex++];
                PhysVector3 pos = new(xStart + (c * colSpacing), table.SurfaceY + radius, rowZ);
                states.Add(BallInitState.Create(id, pos, radius, mass, false));
            }
        }

        return new RackPreset("EightBall", states.ToArray());
    }
}
