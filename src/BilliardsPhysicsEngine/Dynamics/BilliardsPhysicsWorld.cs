using System;
using System.Collections.Generic;
using BilliardsPhysicsEngine.Collision;
using BilliardsPhysicsEngine.Config;
using BilliardsPhysicsEngine.Core;
using BilliardsPhysicsEngine.Prediction;

namespace BilliardsPhysicsEngine.Dynamics;

public sealed class BilliardsPhysicsWorld
{
    private enum CollisionKind
    {
        None,
        BallBall,
        Rail
    }

    private readonly struct CollisionCandidate
    {
        public CollisionKind Kind { get; init; }
        public double Time { get; init; }
        public int BallA { get; init; }
        public int BallB { get; init; }
        public PhysVector3 Normal { get; init; }
        public PhysVector3 ContactPoint { get; init; }
    }

    private TableGeometry _table;
    private PhysicsProfile _profile = new();
    private BallState[] _balls = Array.Empty<BallState>();

    private readonly Dictionary<int, int> _indexByBallId = new();
    private readonly List<PhysicsEvent> _stepEvents = new(64);
    private readonly List<PhysicsEvent> _substepEvents = new(32);
    private readonly List<PhysicsEvent> _externalEvents = new(16);

    private double _simulationTime;
    private double _accumulator;
    private bool _shotInProgress;
    private bool _allStoppedEventFired;
    private int _cueBallId = -1;

    private SimulationFrame _lastFrame = SimulationFrame.Empty;

    public bool IsInitialized => _balls.Length > 0;
    public double FixedStep => _profile.FixedStep;

    public void Initialize(TableGeometry table, PhysicsProfile profile, IReadOnlyList<BallInitState> initialStates)
    {
        _table = table;
        _profile = profile ?? new PhysicsProfile();
        _indexByBallId.Clear();

        _balls = new BallState[initialStates.Count];
        for (int i = 0; i < initialStates.Count; i++)
        {
            BallState state = BallState.FromInit(initialStates[i]);
            state.Position = new PhysVector3(state.Position.X, _table.SurfaceY + state.Radius, state.Position.Z);
            _balls[i] = state;
            _indexByBallId[state.Id] = i;
            if (state.IsCueBall)
            {
                _cueBallId = state.Id;
            }
        }

        _simulationTime = 0.0;
        _accumulator = 0.0;
        _shotInProgress = false;
        _allStoppedEventFired = false;
        _stepEvents.Clear();
        _substepEvents.Clear();
        _externalEvents.Clear();
        _lastFrame = BuildFrame();
    }

    public bool ApplyCueShot(CueShotInput shotInput)
    {
        if (!IsInitialized)
        {
            return false;
        }

        if (!_indexByBallId.TryGetValue(shotInput.BallId, out int ballIndex))
        {
            return false;
        }

        if (shotInput.LinearImpulse <= 0.0 || shotInput.AimDirection.LengthSquared < 1e-12)
        {
            return false;
        }

        BallState ball = _balls[ballIndex];
        if (!ball.InPlay || ball.IsPocketed)
        {
            return false;
        }

        PhysVector3 direction = shotInput.AimDirection.WithY(0.0).Normalized;
        PhysVector3 impulse = direction * shotInput.LinearImpulse;
        ball.Velocity += impulse / ball.Mass;

        double maxOffset = Math.Max(0.0001, ball.Radius * 0.7);
        double offsetX = Math.Clamp(shotInput.TipOffset.X, -maxOffset, maxOffset);
        double offsetY = Math.Clamp(shotInput.TipOffset.Y, -maxOffset, maxOffset);

        PhysVector3 right = PhysVector3.Cross(PhysVector3.Up, direction).Normalized;
        PhysVector3 topBack = right * ((offsetY / maxOffset) * shotInput.SpinMultiplier);
        PhysVector3 side = PhysVector3.Up * ((offsetX / maxOffset) * shotInput.SpinMultiplier);

        double elevationFactor = Math.Clamp(shotInput.CueElevationDeg / 85.0, 0.0, 1.0);
        PhysVector3 masse = PhysVector3.Cross(direction, PhysVector3.Up) * (elevationFactor * shotInput.SpinMultiplier * 0.35);

        double impulseFactor = Math.Clamp(shotInput.LinearImpulse / 0.8, 0.0, 4.0);
        ball.AngularVelocity += (topBack + side + masse) * impulseFactor;
        ball.SlidingBlend = 1.0;
        ball.RestTimer = 0.0;

        _balls[ballIndex] = ball;
        _shotInProgress = true;
        _allStoppedEventFired = false;
        _externalEvents.Add(new PhysicsEvent(PhysicsEventType.ShotStarted, _simulationTime, shotInput.BallId, -1, -1, PhysVector3.Zero, direction));
        return true;
    }

    public void Step(double dtRealTime)
    {
        if (!IsInitialized || dtRealTime <= 0.0)
        {
            _lastFrame = BuildFrame();
            return;
        }

        _accumulator += dtRealTime;
        _stepEvents.Clear();

        if (_externalEvents.Count > 0)
        {
            _stepEvents.AddRange(_externalEvents);
            _externalEvents.Clear();
        }

        int substeps = 0;
        while (_accumulator >= FixedStep && substeps < Math.Max(1, _profile.MaxSubstepsPerFrame))
        {
            _substepEvents.Clear();
            StepFixed(FixedStep);
            _stepEvents.AddRange(_substepEvents);
            _accumulator -= FixedStep;
            substeps++;
        }

        _lastFrame = BuildFrame();
    }

    public SimulationFrame GetCurrentFrame() => _lastFrame;

    public TrajectoryPrediction PredictShot(CueShotInput shot, PredictionSettings settings)
    {
        PredictionSettings cfg = settings;
        if (cfg.HorizonSeconds <= 0.0)
        {
            cfg = PredictionSettings.Default;
        }

        BilliardsPhysicsWorld preview = Clone();
        TrajectoryPrediction prediction = new();
        double shotStartTime = preview._simulationTime;

        for (int i = 0; i < preview._balls.Length; i++)
        {
            prediction.AddPoint(preview._balls[i].Id, preview._balls[i].Position);
        }

        if (!preview.ApplyCueShot(shot))
        {
            return prediction;
        }

        double simulated = 0.0;
        int firstContactBallId = -1;

        while (simulated < cfg.HorizonSeconds)
        {
            preview.Step(cfg.SampleIntervalSeconds);
            SimulationFrame frame = preview.GetCurrentFrame();
            simulated += cfg.SampleIntervalSeconds;

            for (int i = 0; i < frame.Balls.Length; i++)
            {
                BallState ball = frame.Balls[i];
                if (!cfg.IncludePocketedBalls && ball.IsPocketed)
                {
                    continue;
                }

                if (!prediction.TryGetPoints(ball.Id, out IReadOnlyList<PhysVector3> points) || points.Count < cfg.MaxSamplesPerBall)
                {
                    prediction.AddPoint(ball.Id, ball.Position);
                }
            }

            for (int i = 0; i < frame.Events.Length; i++)
            {
                PhysicsEvent evt = frame.Events[i];
                if (evt.Type == PhysicsEventType.Pocketed)
                {
                    prediction.SetPocketed(evt.BallAId);
                }
                else if (evt.Type == PhysicsEventType.BallBallContact && firstContactBallId < 0)
                {
                    int resolved = ResolveFirstObjectBall(evt.BallAId, evt.BallBId);
                    if (resolved >= 0)
                    {
                        firstContactBallId = resolved;
                        double relativeContactTime = Math.Max(0.0, evt.Time - shotStartTime);
                        prediction.SetFirstContact(resolved, evt.Position, relativeContactTime);
                    }
                }
            }

            if (frame.IsAtRest)
            {
                break;
            }
        }

        prediction.SimulatedSeconds = simulated;
        prediction.FirstContactBallId = firstContactBallId;
        return prediction;
    }

    public void ResetToRack(RackPreset rack)
    {
        if (!IsInitialized || rack is null)
        {
            return;
        }

        for (int i = 0; i < rack.InitialStates.Length; i++)
        {
            BallInitState init = rack.InitialStates[i];
            if (!_indexByBallId.TryGetValue(init.Id, out int index))
            {
                continue;
            }

            BallState state = BallState.FromInit(init);
            state.Position = new PhysVector3(state.Position.X, _table.SurfaceY + state.Radius, state.Position.Z);
            _balls[index] = state;
        }

        _simulationTime = 0.0;
        _accumulator = 0.0;
        _shotInProgress = false;
        _allStoppedEventFired = false;
        _stepEvents.Clear();
        _substepEvents.Clear();
        _externalEvents.Clear();
        _lastFrame = BuildFrame();
    }

    public bool TryApplyImpulse(int ballId, in PhysVector3 impulse)
    {
        if (!_indexByBallId.TryGetValue(ballId, out int index))
        {
            return false;
        }

        BallState ball = _balls[index];
        if (!ball.InPlay || ball.IsPocketed)
        {
            return false;
        }

        ball.Velocity += impulse / ball.Mass;
        ball.SlidingBlend = 1.0;
        ball.RestTimer = 0.0;
        _balls[index] = ball;
        _shotInProgress = true;
        _allStoppedEventFired = false;
        return true;
    }

    public bool TryInjectSpin(int ballId, in PhysVector3 deltaAngularVelocity)
    {
        if (!_indexByBallId.TryGetValue(ballId, out int index))
        {
            return false;
        }

        BallState ball = _balls[index];
        if (!ball.InPlay || ball.IsPocketed)
        {
            return false;
        }

        ball.AngularVelocity += deltaAngularVelocity;
        ball.RestTimer = 0.0;
        _balls[index] = ball;
        return true;
    }

    public bool TrySetBallPosition(int ballId, in PhysVector3 position, bool clearMotion)
    {
        if (!_indexByBallId.TryGetValue(ballId, out int index))
        {
            return false;
        }

        BallState ball = _balls[index];
        if (!ball.InPlay || ball.IsPocketed)
        {
            return false;
        }

        ball.Position = _table.ClampBallCenter(position, ball.Radius);
        if (clearMotion)
        {
            ball.Velocity = PhysVector3.Zero;
            ball.AngularVelocity = PhysVector3.Zero;
            ball.RestTimer = _profile.RestConfirmSeconds;
        }

        _balls[index] = ball;
        return true;
    }

    public bool TryGetBallState(int ballId, out BallState state)
    {
        if (_indexByBallId.TryGetValue(ballId, out int index))
        {
            state = _balls[index];
            return true;
        }

        state = default;
        return false;
    }

    private BilliardsPhysicsWorld Clone()
    {
        BilliardsPhysicsWorld clone = new()
        {
            _table = _table,
            _profile = _profile,
            _balls = new BallState[_balls.Length],
            _simulationTime = _simulationTime,
            _accumulator = _accumulator,
            _shotInProgress = _shotInProgress,
            _allStoppedEventFired = _allStoppedEventFired,
            _cueBallId = _cueBallId,
            _lastFrame = _lastFrame
        };

        Array.Copy(_balls, clone._balls, _balls.Length);
        foreach ((int id, int index) in _indexByBallId)
        {
            clone._indexByBallId[id] = index;
        }

        return clone;
    }

    private int ResolveFirstObjectBall(int a, int b)
    {
        if (a == _cueBallId && b != _cueBallId)
        {
            return b;
        }

        if (b == _cueBallId && a != _cueBallId)
        {
            return a;
        }

        return -1;
    }

    private void StepFixed(double dt)
    {
        ApplyClothAndSpinDynamics(dt);
        IntegrateWithContinuousCollisions(dt);
        SolveResidualPenetrations();
        ApplyPocketInteractions(dt);
        SolveResidualPenetrations();
        UpdateRestState(dt);
        _simulationTime += dt;
    }

    private void ApplyClothAndSpinDynamics(double dt)
    {
        for (int i = 0; i < _balls.Length; i++)
        {
            BallState ball = _balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            PhysVector3 contactOffset = PhysVector3.Down * ball.Radius;
            PhysVector3 surfaceVelocity = PhysVector3.Cross(ball.AngularVelocity, contactOffset);
            PhysVector3 slip = ball.Velocity - surfaceVelocity;
            slip = new PhysVector3(slip.X, 0.0, slip.Z);

            double slipSpeed = slip.Length;
            if (slipSpeed > _profile.SlipToRollThreshold)
            {
                double decel = _profile.SlidingFriction * _profile.Gravity * dt;
                PhysVector3 horizontal = new(ball.Velocity.X, 0.0, ball.Velocity.Z);
                horizontal -= slip.Normalized * Math.Min(decel, slipSpeed);
                ball.Velocity = new PhysVector3(horizontal.X, 0.0, horizontal.Z);

                PhysVector3 frictionForce = -slip.Normalized * (_profile.SlidingFriction * ball.Mass * _profile.Gravity);
                PhysVector3 torque = PhysVector3.Cross(contactOffset, frictionForce);
                ball.AngularVelocity += torque * InverseSphereInertia(ball.Mass, ball.Radius) * dt;
                ball.SlidingBlend = 1.0;
            }
            else
            {
                PhysVector3 horizontal = new(ball.Velocity.X, 0.0, ball.Velocity.Z);
                double speed = horizontal.Length;
                if (speed > 0.0)
                {
                    double decel = _profile.RollingFriction * _profile.Gravity * dt;
                    horizontal -= horizontal.Normalized * Math.Min(decel, speed);
                }

                ball.Velocity = new PhysVector3(horizontal.X, 0.0, horizontal.Z);
                ball.SlidingBlend = Math.Max(0.0, ball.SlidingBlend - (6.0 * dt));
            }

            double axialDecay = Math.Clamp(1.0 - (_profile.SpinDecayRate * dt), 0.0, 1.0);
            double sideDecay = Math.Clamp(1.0 - (_profile.SideSpinDecayRate * dt), 0.0, 1.0);
            PhysVector3 angular = ball.AngularVelocity;
            angular.X *= axialDecay;
            angular.Z *= axialDecay;
            angular.Y *= sideDecay;

            if (Math.Abs(angular.X) < 1e-4) angular.X = 0.0;
            if (Math.Abs(angular.Y) < 1e-4) angular.Y = 0.0;
            if (Math.Abs(angular.Z) < 1e-4) angular.Z = 0.0;

            ball.AngularVelocity = angular;
            _balls[i] = ball;
        }
    }

    private void IntegrateWithContinuousCollisions(double dt)
    {
        double elapsed = 0.0;
        int iterations = 0;
        double epsilon = _profile.CcdEpsilon;

        while (elapsed < dt && iterations < Math.Max(1, _profile.MaxCollisionIterationsPerStep))
        {
            iterations++;
            double remaining = dt - elapsed;

            if (!FindEarliestCollision(remaining, out CollisionCandidate hit))
            {
                IntegrateAllBalls(remaining);
                elapsed = dt;
                break;
            }

            if (hit.Time > epsilon)
            {
                IntegrateAllBalls(hit.Time);
                elapsed += hit.Time;
            }

            if (hit.Kind == CollisionKind.BallBall)
            {
                ResolveBallBallCollision(hit.BallA, hit.BallB, hit.Normal, hit.ContactPoint);
            }
            else if (hit.Kind == CollisionKind.Rail)
            {
                ResolveRailCollision(hit.BallA, hit.Normal, hit.ContactPoint);
            }

            elapsed += epsilon;
            if (elapsed < dt)
            {
                IntegrateAllBalls(Math.Min(epsilon, dt - elapsed));
            }
        }

        if (elapsed < dt)
        {
            IntegrateAllBalls(dt - elapsed);
        }

        ClampInsidePlayArea();
    }

    private void IntegrateAllBalls(double dt)
    {
        if (dt <= 0.0)
        {
            return;
        }

        for (int i = 0; i < _balls.Length; i++)
        {
            BallState ball = _balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            ball.Position += ball.Velocity * dt;
            ball.Position.Y = _table.SurfaceY + ball.Radius;
            _balls[i] = ball;
        }
    }

    private bool FindEarliestCollision(double maxTime, out CollisionCandidate candidate)
    {
        candidate = new CollisionCandidate
        {
            Kind = CollisionKind.None,
            Time = double.PositiveInfinity,
            BallA = -1,
            BallB = -1,
            Normal = PhysVector3.Zero,
            ContactPoint = PhysVector3.Zero
        };

        bool found = false;

        for (int i = 0; i < _balls.Length; i++)
        {
            BallState a = _balls[i];
            if (!a.InPlay || a.IsPocketed)
            {
                continue;
            }

            for (int j = i + 1; j < _balls.Length; j++)
            {
                BallState b = _balls[j];
                if (!b.InPlay || b.IsPocketed)
                {
                    continue;
                }

                if (!ContinuousCollision.TrySphereSphereTimeOfImpact(a.Position, a.Velocity, a.Radius, b.Position, b.Velocity, b.Radius, maxTime, out double toi))
                {
                    continue;
                }

                if (toi < candidate.Time)
                {
                    PhysVector3 posA = a.Position + (a.Velocity * toi);
                    PhysVector3 posB = b.Position + (b.Velocity * toi);
                    PhysVector3 normal = (posB - posA).Normalized;

                    candidate = new CollisionCandidate
                    {
                        Kind = CollisionKind.BallBall,
                        Time = toi,
                        BallA = i,
                        BallB = j,
                        Normal = normal,
                        ContactPoint = posA + (normal * a.Radius)
                    };
                    found = true;
                }
            }

            if (TryFindRailCollision(i, maxTime, out CollisionCandidate railHit) && railHit.Time < candidate.Time)
            {
                candidate = railHit;
                found = true;
            }
        }

        return found;
    }

    private bool TryFindRailCollision(int ballIndex, double maxTime, out CollisionCandidate candidate)
    {
        candidate = default;
        BallState ball = _balls[ballIndex];
        double best = double.PositiveInfinity;
        bool found = false;
        PhysVector3 normal = PhysVector3.Zero;
        PhysVector3 point = PhysVector3.Zero;

        if (Math.Abs(ball.Velocity.X) > 1e-9)
        {
            if (ball.Velocity.X < 0.0)
            {
                double t = ((_table.MinXZ.X + ball.Radius) - ball.Position.X) / ball.Velocity.X;
                if (t >= 0.0 && t <= maxTime && t < best)
                {
                    best = t;
                    normal = new PhysVector3(1.0, 0.0, 0.0);
                    point = new PhysVector3(_table.MinXZ.X + ball.Radius, ball.Position.Y, ball.Position.Z + (ball.Velocity.Z * t));
                    found = true;
                }
            }
            else
            {
                double t = ((_table.MaxXZ.X - ball.Radius) - ball.Position.X) / ball.Velocity.X;
                if (t >= 0.0 && t <= maxTime && t < best)
                {
                    best = t;
                    normal = new PhysVector3(-1.0, 0.0, 0.0);
                    point = new PhysVector3(_table.MaxXZ.X - ball.Radius, ball.Position.Y, ball.Position.Z + (ball.Velocity.Z * t));
                    found = true;
                }
            }
        }

        if (Math.Abs(ball.Velocity.Z) > 1e-9)
        {
            if (ball.Velocity.Z < 0.0)
            {
                double t = ((_table.MinXZ.Y + ball.Radius) - ball.Position.Z) / ball.Velocity.Z;
                if (t >= 0.0 && t <= maxTime && t < best)
                {
                    best = t;
                    normal = new PhysVector3(0.0, 0.0, 1.0);
                    point = new PhysVector3(ball.Position.X + (ball.Velocity.X * t), ball.Position.Y, _table.MinXZ.Y + ball.Radius);
                    found = true;
                }
            }
            else
            {
                double t = ((_table.MaxXZ.Y - ball.Radius) - ball.Position.Z) / ball.Velocity.Z;
                if (t >= 0.0 && t <= maxTime && t < best)
                {
                    best = t;
                    normal = new PhysVector3(0.0, 0.0, -1.0);
                    point = new PhysVector3(ball.Position.X + (ball.Velocity.X * t), ball.Position.Y, _table.MaxXZ.Y - ball.Radius);
                    found = true;
                }
            }
        }

        if (!found)
        {
            return false;
        }

        candidate = new CollisionCandidate
        {
            Kind = CollisionKind.Rail,
            Time = best,
            BallA = ballIndex,
            BallB = -1,
            Normal = normal,
            ContactPoint = point
        };

        return true;
    }

    private void ResolveBallBallCollision(int iA, int iB, in PhysVector3 normalIn, in PhysVector3 contact)
    {
        BallState a = _balls[iA];
        BallState b = _balls[iB];
        if (!a.InPlay || !b.InPlay || a.IsPocketed || b.IsPocketed)
        {
            return;
        }

        PhysVector3 normal = new PhysVector3(normalIn.X, 0.0, normalIn.Z).Normalized;
        if (normal.LengthSquared < 1e-12)
        {
            normal = (b.Position - a.Position).Normalized;
        }

        PhysVector3 ra = normal * a.Radius;
        PhysVector3 rb = -normal * b.Radius;

        PhysVector3 velA = a.Velocity + PhysVector3.Cross(a.AngularVelocity, ra);
        PhysVector3 velB = b.Velocity + PhysVector3.Cross(b.AngularVelocity, rb);
        PhysVector3 relative = velA - velB;

        double vN = PhysVector3.Dot(relative, normal);
        if (vN <= 0.0)
        {
            return;
        }

        double invMassA = 1.0 / Math.Max(1e-9, a.Mass);
        double invMassB = 1.0 / Math.Max(1e-9, b.Mass);

        double jN = -((1.0 + _profile.BallRestitution) * vN) / (invMassA + invMassB);
        PhysVector3 normalImpulse = normal * jN;

        PhysVector3 tangentVelocity = relative - (normal * vN);
        PhysVector3 tangentImpulse = PhysVector3.Zero;
        double tangentMag = tangentVelocity.Length;
        if (tangentMag > 1e-9)
        {
            PhysVector3 tangent = tangentVelocity / tangentMag;
            double jT = -(PhysVector3.Dot(relative, tangent)) / (invMassA + invMassB);
            double jTMax = _profile.BallContactFriction * Math.Abs(jN);
            jT = Math.Clamp(jT, -jTMax, jTMax);
            tangentImpulse = tangent * jT;
        }

        PhysVector3 impulse = normalImpulse + tangentImpulse;
        a.Velocity += impulse * invMassA;
        b.Velocity -= impulse * invMassB;

        double invInertiaA = InverseSphereInertia(a.Mass, a.Radius);
        double invInertiaB = InverseSphereInertia(b.Mass, b.Radius);
        a.AngularVelocity += PhysVector3.Cross(ra, impulse) * invInertiaA;
        b.AngularVelocity += PhysVector3.Cross(rb, -impulse) * invInertiaB;

        a.RestTimer = 0.0;
        b.RestTimer = 0.0;
        a.SlidingBlend = 1.0;
        b.SlidingBlend = 1.0;

        double overlap = (a.Radius + b.Radius) - (b.Position - a.Position).Length;
        if (overlap > 0.0)
        {
            PhysVector3 sep = normal * ((overlap * 0.5) + _profile.CcdEpsilon);
            a.Position -= sep;
            b.Position += sep;
        }

        _balls[iA] = a;
        _balls[iB] = b;
        _substepEvents.Add(new PhysicsEvent(PhysicsEventType.BallBallContact, _simulationTime, a.Id, b.Id, -1, contact, normal));
    }

    private void ResolveRailCollision(int index, in PhysVector3 normalIn, in PhysVector3 contact)
    {
        BallState ball = _balls[index];
        if (!ball.InPlay || ball.IsPocketed)
        {
            return;
        }

        PhysVector3 normal = new PhysVector3(normalIn.X, 0.0, normalIn.Z).Normalized;
        if (normal.LengthSquared < 1e-12)
        {
            return;
        }

        double vN = PhysVector3.Dot(ball.Velocity, normal);
        if (vN >= 0.0)
        {
            return;
        }

        PhysVector3 normalVelocity = normal * vN;
        PhysVector3 tangentialVelocity = ball.Velocity - normalVelocity;

        PhysVector3 rebound = (-normalVelocity * _profile.RailRestitution) + (tangentialVelocity * _profile.RailTangentialRetention);
        PhysVector3 railDir = PhysVector3.Cross(PhysVector3.Up, normal).Normalized;
        rebound += railDir * (ball.AngularVelocity.Y * _profile.RailEnglishInfluence);

        ball.Velocity = rebound;
        ball.AngularVelocity = new PhysVector3(
            ball.AngularVelocity.X * _profile.RailAxialSpinRetention,
            -ball.AngularVelocity.Y * _profile.RailSideSpinInversionRetention,
            ball.AngularVelocity.Z * _profile.RailAxialSpinRetention);

        ball.RestTimer = 0.0;
        ball.SlidingBlend = 1.0;
        _balls[index] = ball;

        _substepEvents.Add(new PhysicsEvent(PhysicsEventType.RailContact, _simulationTime, ball.Id, -1, -1, contact, normal));
    }

    private void ClampInsidePlayArea()
    {
        for (int i = 0; i < _balls.Length; i++)
        {
            BallState ball = _balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            ball.Position = _table.ClampBallCenter(ball.Position, ball.Radius);
            _balls[i] = ball;
        }
    }

    private void SolveResidualPenetrations()
    {
        int iterations = Math.Max(1, _profile.PostCollisionPositionIterations);
        double slop = Math.Max(0.0, _profile.PenetrationSlop);
        double correctionFactor = Math.Clamp(_profile.PenetrationCorrectionFactor, 0.0, 1.0);

        for (int iter = 0; iter < iterations; iter++)
        {
            bool hadPenetration = false;

            for (int i = 0; i < _balls.Length; i++)
            {
                BallState a = _balls[i];
                if (!a.InPlay || a.IsPocketed)
                {
                    continue;
                }

                for (int j = i + 1; j < _balls.Length; j++)
                {
                    BallState b = _balls[j];
                    if (!b.InPlay || b.IsPocketed)
                    {
                        continue;
                    }

                    PhysVector3 delta = b.Position - a.Position;
                    double dist = delta.Length;
                    double minDist = a.Radius + b.Radius;
                    double penetration = minDist - dist;

                    if (penetration <= slop)
                    {
                        continue;
                    }

                    hadPenetration = true;
                    PhysVector3 normal;
                    if (dist > 1e-9)
                    {
                        normal = delta / dist;
                    }
                    else
                    {
                        normal = (a.Velocity - b.Velocity).WithY(0.0).Normalized;
                        if (normal.LengthSquared < 1e-12)
                        {
                            normal = new PhysVector3(1.0, 0.0, 0.0);
                        }
                    }

                    double correction = ((penetration - slop) * 0.5 * correctionFactor) + _profile.CcdEpsilon;
                    PhysVector3 separation = normal * correction;
                    a.Position -= separation;
                    b.Position += separation;

                    double invMassA = 1.0 / Math.Max(1e-9, a.Mass);
                    double invMassB = 1.0 / Math.Max(1e-9, b.Mass);
                    PhysVector3 relative = a.Velocity - b.Velocity;
                    double vN = PhysVector3.Dot(relative, normal);
                    if (vN > 0.0)
                    {
                        double jN = -vN / (invMassA + invMassB);
                        PhysVector3 impulse = normal * jN;
                        a.Velocity += impulse * invMassA;
                        b.Velocity -= impulse * invMassB;
                    }

                    a.RestTimer = 0.0;
                    b.RestTimer = 0.0;
                    _balls[i] = a;
                    _balls[j] = b;
                }
            }

            ClampInsidePlayArea();
            if (!hadPenetration)
            {
                break;
            }
        }
    }

    private void ApplyPocketInteractions(double dt)
    {
        if (_table.Pockets.Length == 0)
        {
            return;
        }

        for (int i = 0; i < _balls.Length; i++)
        {
            BallState ball = _balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            PhysVector2 ballXZ = new(ball.Position.X, ball.Position.Z);
            PhysVector2 velXZ = new(ball.Velocity.X, ball.Velocity.Z);
            double speed = velXZ.Length;

            for (int p = 0; p < _table.Pockets.Length; p++)
            {
                PocketGeometry pocket = _table.Pockets[p];
                PhysVector2 pocketXZ = new(pocket.Position.X, pocket.Position.Z);
                PhysVector2 toPocket = pocketXZ - ballXZ;
                double dist = toPocket.Length;

                if (dist <= pocket.DropRadius)
                {
                    ball.InPlay = false;
                    ball.IsPocketed = true;
                    ball.Velocity = PhysVector3.Zero;
                    ball.AngularVelocity = PhysVector3.Zero;
                    ball.Position = new PhysVector3(pocket.Position.X, _table.SurfaceY - 0.15, pocket.Position.Z);
                    _balls[i] = ball;

                    _substepEvents.Add(new PhysicsEvent(PhysicsEventType.Pocketed, _simulationTime, ball.Id, -1, pocket.PocketId, pocket.Position, PhysVector3.Zero));
                    break;
                }

                if (dist > pocket.MouthRadius || dist < 1e-9)
                {
                    continue;
                }

                PhysVector2 toPocketDir = toPocket / dist;
                double approach = PhysVector2.Dot(velXZ, toPocketDir);
                if (approach <= 0.0)
                {
                    continue;
                }

                if (speed > _profile.PocketJawRejectSpeed)
                {
                    PhysVector3 jawNormal = new(-toPocketDir.X, 0.0, -toPocketDir.Y);
                    double vn = PhysVector3.Dot(ball.Velocity, jawNormal);
                    if (vn < 0.0)
                    {
                        ball.Velocity -= (1.0 + pocket.JawRestitution) * vn * jawNormal;
                    }

                    ball.Velocity *= Math.Clamp(1.0 - (pocket.JawFriction * 0.5), 0.0, 1.0);
                    ball.SlidingBlend = 1.0;
                }
                else
                {
                    PhysVector3 assist = new(toPocketDir.X, 0.0, toPocketDir.Y);
                    ball.Velocity += assist * (_profile.PocketEntryAssist * dt);
                }

                _balls[i] = ball;
            }
        }
    }

    private void UpdateRestState(double dt)
    {
        int movingCount = 0;

        for (int i = 0; i < _balls.Length; i++)
        {
            BallState ball = _balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            bool moving = ball.HorizontalSpeed > _profile.RestLinearThreshold || ball.AngularVelocity.Length > _profile.RestAngularThreshold;
            if (moving)
            {
                movingCount++;
                ball.RestTimer = 0.0;
            }
            else
            {
                ball.RestTimer += dt;
                if (ball.RestTimer >= _profile.RestConfirmSeconds)
                {
                    ball.Velocity = PhysVector3.Zero;
                    ball.AngularVelocity = PhysVector3.Zero;
                    ball.RestTimer = _profile.RestConfirmSeconds;
                }
                else
                {
                    movingCount++;
                }
            }

            _balls[i] = ball;
        }

        if (_shotInProgress && movingCount == 0 && !_allStoppedEventFired)
        {
            _allStoppedEventFired = true;
            _shotInProgress = false;
            _substepEvents.Add(new PhysicsEvent(PhysicsEventType.AllBallsStopped, _simulationTime, -1, -1, -1, PhysVector3.Zero, PhysVector3.Zero));
        }
        else if (movingCount > 0)
        {
            _allStoppedEventFired = false;
        }
    }

    private SimulationFrame BuildFrame()
    {
        BallState[] balls = new BallState[_balls.Length];
        Array.Copy(_balls, balls, _balls.Length);

        PhysicsEvent[] events = _stepEvents.Count > 0 ? _stepEvents.ToArray() : Array.Empty<PhysicsEvent>();
        int moving = 0;

        for (int i = 0; i < balls.Length; i++)
        {
            BallState ball = balls[i];
            if (!ball.InPlay || ball.IsPocketed)
            {
                continue;
            }

            if (ball.HorizontalSpeed > _profile.RestLinearThreshold || ball.AngularVelocity.Length > _profile.RestAngularThreshold)
            {
                moving++;
            }
        }

        return new SimulationFrame(_simulationTime, balls, events, moving == 0, moving);
    }

    private static double InverseSphereInertia(double mass, double radius)
    {
        double inertia = 0.4 * mass * radius * radius;
        return inertia <= 1e-12 ? 0.0 : (1.0 / inertia);
    }
}
