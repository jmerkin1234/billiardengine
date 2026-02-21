namespace BilliardsPhysicsEngine.Core;

public readonly struct PredictionSettings
{
    public double HorizonSeconds { get; }
    public double SampleIntervalSeconds { get; }
    public int MaxSamplesPerBall { get; }
    public bool IncludePocketedBalls { get; }

    public PredictionSettings(double horizonSeconds, double sampleIntervalSeconds, int maxSamplesPerBall, bool includePocketedBalls)
    {
        HorizonSeconds = horizonSeconds;
        SampleIntervalSeconds = sampleIntervalSeconds;
        MaxSamplesPerBall = maxSamplesPerBall;
        IncludePocketedBalls = includePocketedBalls;
    }

    public static PredictionSettings Default => new(4.0, 1.0 / 120.0, 300, false);
}
