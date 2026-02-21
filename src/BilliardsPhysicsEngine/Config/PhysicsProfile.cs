namespace BilliardsPhysicsEngine.Config;

public sealed class PhysicsProfile
{
    public int SimulationHz { get; set; } = 480;
    public int MaxSubstepsPerFrame { get; set; } = 4;

    public double Gravity { get; set; } = 9.81;

    public double BallRadius { get; set; } = 0.028575;
    public double BallMass { get; set; } = 0.17;
    public double BallRestitution { get; set; } = 0.96;
    public double BallContactFriction { get; set; } = 0.055;

    public double SlidingFriction { get; set; } = 0.20;
    public double RollingFriction { get; set; } = 0.010;
    public double SlipToRollThreshold { get; set; } = 0.025;
    public double SpinDecayRate { get; set; } = 0.080;
    public double SideSpinDecayRate { get; set; } = 0.160;

    public double RailRestitution { get; set; } = 0.85;
    public double RailFriction { get; set; } = 0.18;
    public double RailTangentialRetention { get; set; } = 0.97;
    public double RailEnglishInfluence { get; set; } = 0.003;
    public double RailAxialSpinRetention { get; set; } = 0.94;
    public double RailSideSpinInversionRetention { get; set; } = 0.62;

    public double PocketJawRejectSpeed { get; set; } = 1.2;
    public double PocketEntryAssist { get; set; } = 0.12;

    public double RestLinearThreshold { get; set; } = 0.010;
    public double RestAngularThreshold { get; set; } = 0.12;
    public double RestConfirmSeconds { get; set; } = 0.15;

    public int MaxCollisionIterationsPerStep { get; set; } = 16;
    public double CcdEpsilon { get; set; } = 0.00005;
    public int PostCollisionPositionIterations { get; set; } = 6;
    public double PenetrationSlop { get; set; } = 0.00001;
    public double PenetrationCorrectionFactor { get; set; } = 1.0;

    public double FixedStep => 1.0 / SimulationHz;
}
