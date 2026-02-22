using System.Diagnostics;
using System.Globalization;
using System.Text;
using System.Text.Json;
using BilliardsPhysicsEngine;
using BilliardsPhysicsEngine.Config;
using BilliardsPhysicsEngine.Core;

const double MaxPositionError = 0.03;
const double MaxAngleErrorDeg = 3.0;
const double MaxSettleErrorSeconds = 0.25;
const double DeterminismEpsilon = 1e-10;

RunnerOptions options = RunnerOptions.Parse(args);
string normalizedProfilePreset = NormalizeProfilePreset(options.ProfilePreset);
string fixturesPath = ResolveFixturePath(options.FixturePath);

if (!File.Exists(fixturesPath))
{
    Console.Error.WriteLine($"Fixture file not found: {fixturesPath}");
    Environment.Exit(2);
}

ShotFixture[] fixtures = JsonSerializer.Deserialize<ShotFixture[]>(File.ReadAllText(fixturesPath), new JsonSerializerOptions
{
    PropertyNameCaseInsensitive = true
}) ?? Array.Empty<ShotFixture>();

if (fixtures.Length == 0)
{
    Console.Error.WriteLine("No fixtures found.");
    Environment.Exit(2);
}

PhysicsProfile profile = new();
ApplyProfilePreset(profile, normalizedProfilePreset);
BilliardPhysicsEngine engine = new();
TableGeometry table = engine.CreateEightFootTable(0.0);

int cueBallId = 0;
List<int> objectIds = Enumerable.Range(1, 15).ToList();
RackPreset rack = RackPreset.EightBall(table, profile.BallRadius, profile.BallMass, cueBallId, objectIds);
engine.Initialize(table, profile, rack.InitialStates);

if (options.RecordMode)
{
    RunRecordMode(engine, rack, profile, fixtures, fixturesPath);
    Environment.Exit(0);
}

if (options.AnalyzeMode)
{
    RunAnalyzeMode(engine, rack, profile, fixtures);
    Environment.Exit(0);
}

if (options.SanityMode)
{
    Environment.Exit(RunSanityChecks(profile));
}

if (options.DeterminismRepeats > 0)
{
    int code = RunDeterminismCheck(engine, rack, profile, fixtures, options.DeterminismRepeats);
    if (code != 0)
    {
        Environment.Exit(code);
    }
}

if (options.StressShots > 0)
{
    int code = RunStressMode(engine, rack, profile, table, options.StressShots, options.RandomSeed);
    if (code != 0)
    {
        Environment.Exit(code);
    }
}

if (options.BenchmarkShots > 0)
{
    _ = RunBenchmarkMode(engine, rack, profile, options.BenchmarkShots, options.RandomSeed, options.PerfBudgetMs);
}

if (options.BenchmarkMatrixShots > 0)
{
    string reportPath = ResolveBenchmarkReportPath(options.BenchmarkReportPath);
    int code = RunBenchmarkMatrix(options.BenchmarkMatrixShots, options.RandomSeed, options.PerfBudgetMs, reportPath);
    Environment.Exit(code);
}

if (!options.ForceBaseline && normalizedProfilePreset != "physics")
{
    Console.WriteLine($"INFO baseline comparison skipped for non-default profile preset '{normalizedProfilePreset}'. Use --force-baseline to run anyway.");
    Environment.Exit(0);
}

int failed = RunBaselineComparison(engine, rack, profile, fixtures);
Environment.Exit(failed == 0 ? 0 : 1);

static int RunBaselineComparison(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, ShotFixture[] fixtures)
{
    int passed = 0;
    int failed = 0;

    foreach (ShotFixture fixture in fixtures)
    {
        SimResult result = Simulate(engine, rack, profile, fixture);
        if (!result.Success)
        {
            Console.WriteLine($"FAIL {fixture.Name}: {result.Error}");
            failed++;
            continue;
        }

        if (fixture.RequireRestByHorizon && !result.ReachedRest)
        {
            Console.WriteLine($"FAIL {fixture.Name}: did not reach rest by horizon ({fixture.HorizonSeconds:F2}s)");
            failed++;
            continue;
        }

        PhysVector3 expected = new(fixture.ExpectedFinalPosition[0], fixture.ExpectedFinalPosition[1], fixture.ExpectedFinalPosition[2]);
        double posErr = (result.FinalPosition - expected).Length;
        double angleErr = Math.Abs(NormalizeDeltaAngle(result.ExitAngleDeg, fixture.ExpectedExitAngleDeg));
        double settleErr = Math.Abs(result.SettleSeconds - fixture.ExpectedSettleSeconds);

        bool pass = posErr <= MaxPositionError && angleErr <= MaxAngleErrorDeg && settleErr <= MaxSettleErrorSeconds;
        if (pass)
        {
            passed++;
            Console.WriteLine($"PASS {fixture.Name}: pos={posErr:F4}m angle={angleErr:F2}deg settle={settleErr:F3}s");
        }
        else
        {
            failed++;
            Console.WriteLine($"FAIL {fixture.Name}: pos={posErr:F4}m angle={angleErr:F2}deg settle={settleErr:F3}s");
        }
    }

    Console.WriteLine($"Shot Suite Complete: Passed={passed} Failed={failed}");
    return failed;
}

static void RunRecordMode(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, ShotFixture[] fixtures, string fixturesPath)
{
    for (int i = 0; i < fixtures.Length; i++)
    {
        ShotFixture fixture = fixtures[i];
        SimResult result = Simulate(engine, rack, profile, fixture);
        if (!result.Success)
        {
            Console.WriteLine($"SKIP {fixture.Name}: {result.Error}");
            continue;
        }

        fixture.ExpectedFinalPosition = new[] { result.FinalPosition.X, result.FinalPosition.Y, result.FinalPosition.Z };
        fixture.ExpectedExitAngleDeg = result.ExitAngleDeg;
        fixture.ExpectedSettleSeconds = result.SettleSeconds;
        fixtures[i] = fixture;
        Console.WriteLine($"RECORDED {fixture.Name}");
    }

    File.WriteAllText(fixturesPath, JsonSerializer.Serialize(fixtures, new JsonSerializerOptions { WriteIndented = true }));
    Console.WriteLine($"Baseline recorded to {fixturesPath}");
}

static void RunAnalyzeMode(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, ShotFixture[] fixtures)
{
    Console.WriteLine("Analyze Mode");
    foreach (ShotFixture fixture in fixtures)
    {
        SimResult result = Simulate(engine, rack, profile, fixture);
        if (!result.Success)
        {
            Console.WriteLine($"{fixture.Name}: ERROR {result.Error}");
            continue;
        }

        Console.WriteLine($"{fixture.Name}: settle={result.SettleSeconds:F3}s reachedRest={result.ReachedRest} moving={result.FinalMovingCount} events={result.EventCount}");
    }
}

static int RunDeterminismCheck(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, ShotFixture[] fixtures, int repeats)
{
    Console.WriteLine($"Determinism Check: repeats={repeats}");

    int failures = 0;
    foreach (ShotFixture fixture in fixtures)
    {
        SimResult baseline = Simulate(engine, rack, profile, fixture);
        if (!baseline.Success)
        {
            Console.WriteLine($"FAIL {fixture.Name}: baseline sim error: {baseline.Error}");
            failures++;
            continue;
        }

        for (int i = 1; i < repeats; i++)
        {
            SimResult rerun = Simulate(engine, rack, profile, fixture);
            if (!rerun.Success)
            {
                Console.WriteLine($"FAIL {fixture.Name}: rerun {i} sim error: {rerun.Error}");
                failures++;
                break;
            }

            if (!FramesNearlyEqual(baseline.FinalFrame, rerun.FinalFrame, DeterminismEpsilon) ||
                Math.Abs(baseline.SettleSeconds - rerun.SettleSeconds) > DeterminismEpsilon)
            {
                Console.WriteLine($"FAIL {fixture.Name}: non-deterministic result detected at rerun {i}");
                failures++;
                break;
            }
        }

        if (failures == 0 || !Console.IsOutputRedirected)
        {
            Console.WriteLine($"PASS {fixture.Name}: deterministic across {repeats} runs");
        }
    }

    if (failures > 0)
    {
        Console.WriteLine($"Determinism Check Failed: {failures} case(s)");
        return 1;
    }

    Console.WriteLine("Determinism Check Passed");
    return 0;
}

static int RunStressMode(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, TableGeometry table, int shots, int seed)
{
    Console.WriteLine($"Stress Mode: shots={shots}, seed={seed}");

    Random rng = new(seed);
    int failures = 0;

    for (int i = 0; i < shots; i++)
    {
        engine.ResetToRack(rack);

        double angle = rng.NextDouble() * Math.PI * 2.0;
        PhysVector3 aim = new(Math.Sin(angle), 0.0, Math.Cos(angle));
        double impulse = 0.35 + (rng.NextDouble() * 0.95);

        double maxTip = profile.BallRadius * 0.65;
        PhysVector2 tipOffset = new(
            (rng.NextDouble() * 2.0 - 1.0) * maxTip,
            (rng.NextDouble() * 2.0 - 1.0) * maxTip);

        CueShotInput shot = new(
            0,
            aim,
            impulse,
            tipOffset,
            0.0,
            0.525,
            0.006,
            18.0);

        if (!engine.CueStrike(shot))
        {
            Console.WriteLine($"FAIL stress#{i}: shot rejected");
            failures++;
            continue;
        }

        double elapsed = 0.0;
        bool bad = false;
        while (elapsed < 12.0)
        {
            SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
            elapsed += 1.0 / profile.SimulationHz;

            if (!ValidateFrame(frame, table, out string reason))
            {
                Console.WriteLine($"FAIL stress#{i}: {reason}");
                failures++;
                bad = true;
                break;
            }

            if (frame.IsAtRest)
            {
                break;
            }
        }

        if (!bad && (i + 1) % 25 == 0)
        {
            Console.WriteLine($"stress progress: {i + 1}/{shots}");
        }
    }

    if (failures > 0)
    {
        Console.WriteLine($"Stress Mode Failed: {failures} failing shot(s)");
        return 1;
    }

    Console.WriteLine("Stress Mode Passed");
    return 0;
}

static BenchmarkResult RunBenchmarkMode(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, int shots, int seed, double perfBudgetMs)
{
    string budgetLabel = perfBudgetMs > 0.0
        ? $"{perfBudgetMs.ToString("F2", CultureInfo.InvariantCulture)}ms/shot"
        : "off";
    Console.WriteLine($"Benchmark Mode: shots={shots}, seed={seed}, profile={profile.SimulationHz}Hz, budget={budgetLabel}");

    Random rng = new(seed + 17);
    Stopwatch shotTimer = new();

    double totalMs = 0.0;
    double worstMs = 0.0;
    double totalSimSeconds = 0.0;
    long totalSteps = 0;
    int completedShots = 0;
    int rejectedShots = 0;
    List<double> shotTimesMs = new(shots);

    for (int i = 0; i < shots; i++)
    {
        engine.ResetToRack(rack);
        double angle = (-Math.PI * 0.1) + (rng.NextDouble() * Math.PI * 0.2);
        CueShotInput shot = new(
            0,
            new PhysVector3(Math.Sin(angle), 0.0, -Math.Cos(angle)),
            0.95,
            PhysVector2.Zero,
            0.0,
            0.525,
            0.006,
            18.0);

        if (!engine.CueStrike(shot))
        {
            rejectedShots++;
            continue;
        }

        shotTimer.Restart();
        double elapsed = 0.0;
        int steps = 0;
        while (elapsed < 12.0)
        {
            SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
            elapsed += 1.0 / profile.SimulationHz;
            steps++;
            if (frame.IsAtRest)
            {
                break;
            }
        }
        shotTimer.Stop();

        double ms = shotTimer.Elapsed.TotalMilliseconds;
        totalMs += ms;
        worstMs = Math.Max(worstMs, ms);
        totalSimSeconds += elapsed;
        totalSteps += steps;
        completedShots++;
        shotTimesMs.Add(ms);
    }

    if (completedShots == 0)
    {
        Console.WriteLine("Benchmark: no completed shots");
        return BenchmarkResult.Empty(profile.SimulationHz, shots, perfBudgetMs);
    }

    shotTimesMs.Sort();
    double avgMs = totalMs / completedShots;
    double p50Ms = Percentile(shotTimesMs, 0.50);
    double p95Ms = Percentile(shotTimesMs, 0.95);
    double p99Ms = Percentile(shotTimesMs, 0.99);
    double avgSimSeconds = totalSimSeconds / completedShots;
    double avgSteps = (double)totalSteps / completedShots;
    double msPerSimSecond = totalSimSeconds > 1e-9 ? totalMs / totalSimSeconds : 0.0;
    double realTimeFactor = totalMs > 1e-9 ? (totalSimSeconds * 1000.0) / totalMs : 0.0;

    Console.WriteLine($"Benchmark: avg={avgMs:F3}ms p50={p50Ms:F3}ms p95={p95Ms:F3}ms p99={p99Ms:F3}ms worst={worstMs:F3}ms");
    Console.WriteLine($"Benchmark Breakdown: completed={completedShots}/{shots} rejected={rejectedShots} avgSteps={avgSteps:F1} avgSim={avgSimSeconds:F3}s");
    Console.WriteLine($"Benchmark Throughput: msPerSimSecond={msPerSimSecond:F3} realtimeFactor={realTimeFactor:F2}x");

    int perfWarnCount = 0;
    if (perfBudgetMs > 0.0)
    {
        bool warn = false;
        if (avgMs > perfBudgetMs)
        {
            Console.WriteLine($"WARN perf: avg shot time {avgMs:F3}ms exceeds budget {perfBudgetMs:F3}ms");
            warn = true;
            perfWarnCount++;
        }

        if (p95Ms > perfBudgetMs * 1.25)
        {
            Console.WriteLine($"WARN perf: p95 shot time {p95Ms:F3}ms exceeds 125% budget");
            warn = true;
            perfWarnCount++;
        }

        if (worstMs > perfBudgetMs * 2.0)
        {
            Console.WriteLine($"WARN perf: worst shot time {worstMs:F3}ms exceeds 200% budget");
            warn = true;
            perfWarnCount++;
        }

        if (!warn)
        {
            Console.WriteLine("PASS perf: benchmark is within configured budget");
        }
        else if (profile.SimulationHz >= 480)
        {
            Console.WriteLine("Hint: try `--profile balanced` or `--profile debug240` for manual perf preset fallback.");
        }
    }

    return new BenchmarkResult(
        profile.SimulationHz,
        shots,
        completedShots,
        rejectedShots,
        avgMs,
        p50Ms,
        p95Ms,
        p99Ms,
        worstMs,
        avgSteps,
        avgSimSeconds,
        msPerSimSecond,
        realTimeFactor,
        perfBudgetMs,
        perfWarnCount);
}

static int RunBenchmarkMatrix(int shots, int seed, double perfBudgetMs, string reportPath)
{
    Console.WriteLine($"Benchmark Matrix: shots={shots}, seed={seed}");

    string[] presets = { "physics", "balanced", "debug240" };
    List<(string Preset, BenchmarkResult Result)> rows = new(presets.Length);
    List<int> objectIds = Enumerable.Range(1, 15).ToList();

    for (int i = 0; i < presets.Length; i++)
    {
        string preset = presets[i];
        Console.WriteLine($"--- preset: {preset} ---");
        PhysicsProfile profile = new();
        ApplyProfilePreset(profile, preset);
        BilliardPhysicsEngine engine = new();
        TableGeometry table = engine.CreateEightFootTable(0.0);
        RackPreset rack = RackPreset.EightBall(table, profile.BallRadius, profile.BallMass, 0, objectIds);
        engine.Initialize(table, profile, rack.InitialStates);

        BenchmarkResult result = RunBenchmarkMode(engine, rack, profile, shots, seed, perfBudgetMs);
        if (result.CompletedShots == 0)
        {
            Console.WriteLine($"FAIL benchmark-matrix: preset '{preset}' produced zero completed shots");
            return 1;
        }

        rows.Add((preset, result));
    }

    WriteBenchmarkMatrixReport(rows, shots, seed, perfBudgetMs, reportPath);
    Console.WriteLine($"Benchmark matrix report written to {reportPath}");
    return 0;
}

static void WriteBenchmarkMatrixReport(IReadOnlyList<(string Preset, BenchmarkResult Result)> rows, int shots, int seed, double perfBudgetMs, string reportPath)
{
    StringBuilder md = new();
    string budgetLabel = perfBudgetMs > 0.0
        ? $"{perfBudgetMs.ToString("F2", CultureInfo.InvariantCulture)} ms/shot"
        : "off";

    md.AppendLine("# Benchmark Matrix Report");
    md.AppendLine();
    md.AppendLine($"- Generated (UTC): `{DateTime.UtcNow.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture)} UTC`");
    md.AppendLine($"- Shots per preset: `{shots}`");
    md.AppendLine($"- Seed: `{seed}`");
    md.AppendLine($"- Perf budget: `{budgetLabel}`");
    md.AppendLine();
    md.AppendLine("| Preset | Hz | Completed | Avg ms | P95 ms | Worst ms | Avg steps | Avg sim (s) | ms/sim-s | RT factor | Budget |");
    md.AppendLine("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|");

    for (int i = 0; i < rows.Count; i++)
    {
        (string preset, BenchmarkResult result) = rows[i];
        string budgetStatus = perfBudgetMs <= 0.0
            ? "n/a"
            : (result.PerfWarnCount == 0 ? "pass" : $"warn ({result.PerfWarnCount})");

        md.AppendLine(
            $"| `{preset}` | {result.SimulationHz} | {result.CompletedShots}/{result.RequestedShots} | " +
            $"{result.AvgMs.ToString("F3", CultureInfo.InvariantCulture)} | " +
            $"{result.P95Ms.ToString("F3", CultureInfo.InvariantCulture)} | " +
            $"{result.WorstMs.ToString("F3", CultureInfo.InvariantCulture)} | " +
            $"{result.AvgSteps.ToString("F1", CultureInfo.InvariantCulture)} | " +
            $"{result.AvgSimSeconds.ToString("F3", CultureInfo.InvariantCulture)} | " +
            $"{result.MsPerSimSecond.ToString("F3", CultureInfo.InvariantCulture)} | " +
            $"{result.RealTimeFactor.ToString("F2", CultureInfo.InvariantCulture)}x | {budgetStatus} |");
    }

    md.AppendLine();
    md.AppendLine("## Notes");
    md.AppendLine("- `physics` is the default high-fidelity profile.");
    md.AppendLine("- `balanced` and `debug240` are manual fallback presets.");
    md.AppendLine("- No automatic fidelity downshift is applied by the engine.");

    string fullPath = Path.GetFullPath(reportPath);
    string? dir = Path.GetDirectoryName(fullPath);
    if (!string.IsNullOrWhiteSpace(dir))
    {
        Directory.CreateDirectory(dir);
    }

    File.WriteAllText(fullPath, md.ToString());
}

static void ApplyProfilePreset(PhysicsProfile profile, string preset)
{
    string normalized = NormalizeProfilePreset(preset);

    if (normalized == "physics")
    {
        profile.SimulationHz = 480;
        profile.MaxSubstepsPerFrame = 4;
        profile.MaxCollisionIterationsPerStep = 16;
        profile.PostCollisionPositionIterations = 6;
        return;
    }

    if (normalized == "balanced")
    {
        profile.SimulationHz = 360;
        profile.MaxSubstepsPerFrame = 4;
        profile.MaxCollisionIterationsPerStep = 14;
        profile.PostCollisionPositionIterations = 4;
        return;
    }

    if (normalized == "debug240")
    {
        profile.SimulationHz = 240;
        profile.MaxSubstepsPerFrame = 3;
        profile.MaxCollisionIterationsPerStep = 10;
        profile.PostCollisionPositionIterations = 3;
        return;
    }

    ApplyProfilePreset(profile, "physics");
}

static string NormalizeProfilePreset(string preset)
{
    string normalized = preset.Trim().ToLowerInvariant();
    if (normalized is "physics" or "physics480" or "default")
    {
        return "physics";
    }

    if (normalized is "balanced" or "balanced360")
    {
        return "balanced";
    }

    if (normalized is "debug240" or "debug" or "perf240")
    {
        return "debug240";
    }

    return "physics";
}

static double Percentile(IReadOnlyList<double> sortedValues, double p)
{
    if (sortedValues.Count == 0)
    {
        return 0.0;
    }

    if (sortedValues.Count == 1)
    {
        return sortedValues[0];
    }

    double clamped = Math.Clamp(p, 0.0, 1.0);
    double index = clamped * (sortedValues.Count - 1);
    int lo = (int)Math.Floor(index);
    int hi = (int)Math.Ceiling(index);
    if (lo == hi)
    {
        return sortedValues[lo];
    }

    double t = index - lo;
    return sortedValues[lo] + ((sortedValues[hi] - sortedValues[lo]) * t);
}

static int RunSanityChecks(PhysicsProfile profile)
{
    Console.WriteLine("Sanity Checks");
    const int TotalChecks = 6;
    int failures = 0;

    if (RunHeadOnCollisionSanity(profile, out string headOnReason))
    {
        Console.WriteLine("PASS sanity: head-on collision transfer");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: head-on collision transfer ({headOnReason})");
        failures++;
    }

    if (RunOverlapRecoverySanity(profile, out string overlapReason))
    {
        Console.WriteLine("PASS sanity: overlap recovery");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: overlap recovery ({overlapReason})");
        failures++;
    }

    if (RunRailBounceSanity(profile, out string railReason))
    {
        Console.WriteLine("PASS sanity: rail bounce response");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: rail bounce response ({railReason})");
        failures++;
    }

    if (RunPocketEventSanity(profile, out string pocketReason))
    {
        Console.WriteLine("PASS sanity: pocket event emission");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: pocket event emission ({pocketReason})");
        failures++;
    }

    if (RunShotLifecycleSanity(profile, out string lifecycleReason))
    {
        Console.WriteLine("PASS sanity: shot lifecycle events");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: shot lifecycle events ({lifecycleReason})");
        failures++;
    }

    if (RunPredictionIsolationSanity(profile, out string predictionReason))
    {
        Console.WriteLine("PASS sanity: prediction isolation");
    }
    else
    {
        Console.WriteLine($"FAIL sanity: prediction isolation ({predictionReason})");
        failures++;
    }

    Console.WriteLine($"Sanity Check Complete: Passed={TotalChecks - failures} Failed={failures}");
    return failures == 0 ? 0 : 1;
}

static bool RunHeadOnCollisionSanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);

    BallInitState[] states =
    {
        BallInitState.Create(0, new PhysVector3(0.0, table.SurfaceY + profile.BallRadius, 0.35), profile.BallRadius, profile.BallMass, true),
        BallInitState.Create(1, new PhysVector3(0.0, table.SurfaceY + profile.BallRadius, 0.18), profile.BallRadius, profile.BallMass, false)
    };

    engine.Initialize(table, profile, states);
    CueShotInput shot = new(
        0,
        new PhysVector3(0.0, 0.0, -1.0),
        0.75,
        PhysVector2.Zero,
        0.0,
        0.525,
        0.006,
        18.0);

    if (!engine.CueStrike(shot))
    {
        reason = "cue strike rejected";
        return false;
    }

    bool sawContact = false;
    for (int i = 0; i < profile.SimulationHz * 2; i++)
    {
        SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
        if (!ValidateNumericFrame(frame, out reason))
        {
            return false;
        }

        if (!sawContact)
        {
            for (int e = 0; e < frame.Events.Length; e++)
            {
                PhysicsEvent evt = frame.Events[e];
                if (evt.Type == PhysicsEventType.BallBallContact &&
                    ((evt.BallAId == 0 && evt.BallBId == 1) || (evt.BallAId == 1 && evt.BallBId == 0)))
                {
                    sawContact = true;
                    break;
                }
            }
        }

        if (!sawContact)
        {
            continue;
        }

        if (!TryGetBall(frame, 0, out BallState cue) || !TryGetBall(frame, 1, out BallState obj))
        {
            reason = "missing sanity balls";
            return false;
        }

        PhysVector3 shotDir = new(0.0, 0.0, -1.0);
        double cueForward = PhysVector3.Dot(cue.Velocity, shotDir);
        double objForward = PhysVector3.Dot(obj.Velocity, shotDir);
        double minDist = cue.Radius + obj.Radius - 1e-5;
        double dist = (cue.Position - obj.Position).Length;

        if (dist < minDist)
        {
            reason = $"overlap persists after impact (dist={dist:F5}, min={minDist:F5})";
            return false;
        }

        if (objForward <= 0.10)
        {
            reason = $"object did not receive forward speed (v={objForward:F4})";
            return false;
        }

        if (cueForward >= objForward)
        {
            reason = $"cue did not decelerate vs object (cue={cueForward:F4}, obj={objForward:F4})";
            return false;
        }

        reason = string.Empty;
        return true;
    }

    reason = "no 0-1 contact detected within horizon";
    return false;
}

static bool RunOverlapRecoverySanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);
    double overlap = 0.0015;

    BallInitState[] states =
    {
        BallInitState.Create(0, new PhysVector3(0.0, table.SurfaceY + profile.BallRadius, 0.0), profile.BallRadius, profile.BallMass, true),
        BallInitState.Create(1, new PhysVector3((profile.BallRadius * 2.0) - overlap, table.SurfaceY + profile.BallRadius, 0.0), profile.BallRadius, profile.BallMass, false)
    };

    engine.Initialize(table, profile, states);

    SimulationFrame frame = engine.Step(0.0);
    for (int i = 0; i < 30; i++)
    {
        frame = engine.Step(1.0 / profile.SimulationHz);
        if (!ValidateNumericFrame(frame, out reason))
        {
            return false;
        }
    }

    if (!TryGetBall(frame, 0, out BallState a) || !TryGetBall(frame, 1, out BallState b))
    {
        reason = "missing overlap sanity balls";
        return false;
    }

    double dist = (a.Position - b.Position).Length;
    double minDist = a.Radius + b.Radius - 1e-5;
    if (dist < minDist)
    {
        reason = $"overlap not resolved (dist={dist:F5}, min={minDist:F5})";
        return false;
    }

    reason = string.Empty;
    return true;
}

static bool RunRailBounceSanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);
    PhysVector3 initialVelocity = new(-1.25, 0.0, 0.20);

    BallInitState[] states =
    {
        new BallInitState(
            0,
            new PhysVector3(table.MinXZ.X + profile.BallRadius + 0.14, table.SurfaceY + profile.BallRadius, 0.0),
            profile.BallRadius,
            profile.BallMass,
            true,
            initialVelocity,
            PhysVector3.Zero,
            true,
            false)
    };

    engine.Initialize(table, profile, states);

    for (int i = 0; i < profile.SimulationHz * 2; i++)
    {
        SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
        if (!ValidateNumericFrame(frame, out reason))
        {
            return false;
        }

        bool hadRailContact = false;
        for (int e = 0; e < frame.Events.Length; e++)
        {
            if (frame.Events[e].Type == PhysicsEventType.RailContact && frame.Events[e].BallAId == 0)
            {
                hadRailContact = true;
                break;
            }
        }

        if (!hadRailContact)
        {
            continue;
        }

        if (!TryGetBall(frame, 0, out BallState ball))
        {
            reason = "missing rail sanity ball";
            return false;
        }

        if (ball.Velocity.X <= 0.0)
        {
            reason = $"rail bounce did not reverse X velocity (vx={ball.Velocity.X:F4})";
            return false;
        }

        if (ball.HorizontalSpeed > (new PhysVector3(initialVelocity.X, 0.0, initialVelocity.Z).Length * 1.05))
        {
            reason = $"rail bounce gained too much speed (speed={ball.HorizontalSpeed:F4})";
            return false;
        }

        reason = string.Empty;
        return true;
    }

    reason = "no rail contact detected within horizon";
    return false;
}

static bool RunPocketEventSanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);
    double y = table.SurfaceY + profile.BallRadius;

    BallInitState[] states =
    {
        BallInitState.Create(
            0,
            new PhysVector3(table.MaxXZ.X - profile.BallRadius, y, table.MaxXZ.Y - profile.BallRadius),
            profile.BallRadius,
            profile.BallMass,
            true)
    };

    engine.Initialize(table, profile, states);

    for (int i = 0; i < 20; i++)
    {
        SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
        if (!ValidateNumericFrame(frame, out reason))
        {
            return false;
        }

        bool pocketEvent = false;
        for (int e = 0; e < frame.Events.Length; e++)
        {
            PhysicsEvent evt = frame.Events[e];
            if (evt.Type == PhysicsEventType.Pocketed && evt.BallAId == 0)
            {
                pocketEvent = true;
                break;
            }
        }

        if (!pocketEvent)
        {
            continue;
        }

        if (!TryGetBall(frame, 0, out BallState ball))
        {
            reason = "missing pocket sanity ball";
            return false;
        }

        if (!ball.IsPocketed || ball.InPlay)
        {
            reason = "ball state not pocketed after pocket event";
            return false;
        }

        reason = string.Empty;
        return true;
    }

    reason = "no pocket event detected within horizon";
    return false;
}

static bool RunShotLifecycleSanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);
    double y = table.SurfaceY + profile.BallRadius;

    BallInitState[] states =
    {
        BallInitState.Create(0, new PhysVector3(0.0, y, 0.0), profile.BallRadius, profile.BallMass, true)
    };

    engine.Initialize(table, profile, states);
    CueShotInput shot = new(
        0,
        new PhysVector3(0.0, 0.0, -1.0),
        0.08,
        PhysVector2.Zero,
        0.0,
        0.525,
        0.006,
        18.0);

    if (!engine.CueStrike(shot))
    {
        reason = "cue strike rejected";
        return false;
    }

    bool sawShotStarted = false;
    bool sawAllStopped = false;

    for (int i = 0; i < profile.SimulationHz * 8; i++)
    {
        SimulationFrame frame = engine.Step(1.0 / profile.SimulationHz);
        if (!ValidateNumericFrame(frame, out reason))
        {
            return false;
        }

        for (int e = 0; e < frame.Events.Length; e++)
        {
            PhysicsEvent evt = frame.Events[e];
            if (evt.Type == PhysicsEventType.ShotStarted)
            {
                sawShotStarted = true;
            }
            else if (evt.Type == PhysicsEventType.AllBallsStopped)
            {
                sawAllStopped = true;
            }
        }

        if (sawShotStarted && sawAllStopped && frame.IsAtRest)
        {
            reason = string.Empty;
            return true;
        }
    }

    if (!sawShotStarted)
    {
        reason = "ShotStarted event never emitted";
        return false;
    }

    if (!sawAllStopped)
    {
        reason = "AllBallsStopped event never emitted";
        return false;
    }

    reason = "world did not settle after AllBallsStopped";
    return false;
}

static bool RunPredictionIsolationSanity(PhysicsProfile profile, out string reason)
{
    BilliardPhysicsEngine engine = new();
    TableGeometry table = TableGeometry.CreateEightFoot(0.0);
    double y = table.SurfaceY + profile.BallRadius;

    BallInitState[] states =
    {
        BallInitState.Create(0, new PhysVector3(0.0, y, 0.35), profile.BallRadius, profile.BallMass, true),
        BallInitState.Create(1, new PhysVector3(0.0, y, 0.20), profile.BallRadius, profile.BallMass, false)
    };

    engine.Initialize(table, profile, states);
    SimulationFrame before = engine.Step(0.0);

    CueShotInput shot = new(
        0,
        new PhysVector3(0.0, 0.0, -1.0),
        0.65,
        PhysVector2.Zero,
        0.0,
        0.525,
        0.006,
        18.0);

    var prediction = engine.GetTrajectoryPrediction(shot, PredictionSettings.Default);
    SimulationFrame after = engine.Step(0.0);

    if (!FramesNearlyEqual(before, after, DeterminismEpsilon))
    {
        reason = "prediction mutated live world state";
        return false;
    }

    if (prediction.SimulatedSeconds <= 0.0)
    {
        reason = "prediction simulated time is zero";
        return false;
    }

    if (!prediction.TryGetPoints(0, out IReadOnlyList<PhysVector3> cuePoints) || cuePoints.Count < 2)
    {
        reason = "prediction did not produce cue-ball path points";
        return false;
    }

    if (prediction.FirstContactBallId != 1)
    {
        reason = $"unexpected first contact ball id ({prediction.FirstContactBallId})";
        return false;
    }

    if (!prediction.HasFirstContact)
    {
        reason = "prediction did not expose first-contact metadata";
        return false;
    }

    if (!IsFinite(prediction.FirstContactPoint.X) || !IsFinite(prediction.FirstContactPoint.Y) || !IsFinite(prediction.FirstContactPoint.Z))
    {
        reason = "prediction first-contact point is non-finite";
        return false;
    }

    if (prediction.FirstContactTimeSeconds < 0.0 || prediction.FirstContactTimeSeconds > prediction.SimulatedSeconds + (1.0 / profile.SimulationHz))
    {
        reason = $"prediction first-contact time out of range ({prediction.FirstContactTimeSeconds:F4}s)";
        return false;
    }

    reason = string.Empty;
    return true;
}

static bool TryGetBall(in SimulationFrame frame, int ballId, out BallState state)
{
    for (int i = 0; i < frame.Balls.Length; i++)
    {
        if (frame.Balls[i].Id == ballId)
        {
            state = frame.Balls[i];
            return true;
        }
    }

    state = default;
    return false;
}

static SimResult Simulate(BilliardPhysicsEngine engine, RackPreset rack, PhysicsProfile profile, ShotFixture fixture)
{
    engine.ResetToRack(rack);

    CueShotInput shot = new(
        fixture.ShotBallId,
        new PhysVector3(fixture.AimDirection[0], fixture.AimDirection[1], fixture.AimDirection[2]),
        fixture.Impulse,
        new PhysVector2(fixture.TipOffset[0], fixture.TipOffset[1]),
        fixture.CueElevationDeg,
        fixture.CueMass,
        fixture.TipRadius,
        fixture.SpinMultiplier);

    if (!engine.CueStrike(shot))
    {
        return SimResult.Fail("cue strike rejected");
    }

    double simSeconds = 0.0;
    int eventCount = 0;
    SimulationFrame frame = engine.Step(0.0);

    while (simSeconds < fixture.HorizonSeconds)
    {
        frame = engine.Step(1.0 / profile.SimulationHz);
        simSeconds += 1.0 / profile.SimulationHz;
        eventCount += frame.Events.Length;

        if (!ValidateNumericFrame(frame, out string reason))
        {
            return SimResult.Fail(reason);
        }

        if (frame.IsAtRest)
        {
            break;
        }
    }

    for (int i = 0; i < frame.Balls.Length; i++)
    {
        if (frame.Balls[i].Id == fixture.TrackBallId)
        {
            BallState tracked = frame.Balls[i];
            double exitAngle = Math.Atan2(tracked.Velocity.X, tracked.Velocity.Z) * (180.0 / Math.PI);
            return SimResult.Ok(tracked.Position, exitAngle, simSeconds, frame.IsAtRest, frame.MovingBallCount, eventCount, frame);
        }
    }

    return SimResult.Fail($"tracked ball {fixture.TrackBallId} not found");
}

static bool ValidateFrame(SimulationFrame frame, TableGeometry table, out string reason)
{
    if (!ValidateNumericFrame(frame, out reason))
    {
        return false;
    }

    for (int i = 0; i < frame.Balls.Length; i++)
    {
        BallState ball = frame.Balls[i];
        if (!ball.InPlay || ball.IsPocketed)
        {
            continue;
        }

        double minX = table.MinXZ.X + ball.Radius - 1e-3;
        double maxX = table.MaxXZ.X - ball.Radius + 1e-3;
        double minZ = table.MinXZ.Y + ball.Radius - 1e-3;
        double maxZ = table.MaxXZ.Y - ball.Radius + 1e-3;

        if (ball.Position.X < minX || ball.Position.X > maxX || ball.Position.Z < minZ || ball.Position.Z > maxZ)
        {
            reason = $"ball {ball.Id} out of bounds ({ball.Position.X:F4}, {ball.Position.Z:F4})";
            return false;
        }
    }

    for (int i = 0; i < frame.Balls.Length; i++)
    {
        BallState a = frame.Balls[i];
        if (!a.InPlay || a.IsPocketed)
        {
            continue;
        }

        for (int j = i + 1; j < frame.Balls.Length; j++)
        {
            BallState b = frame.Balls[j];
            if (!b.InPlay || b.IsPocketed)
            {
                continue;
            }

            double dist = (a.Position - b.Position).Length;
            double minDist = (a.Radius + b.Radius) - 0.002;
            if (dist < minDist)
            {
                reason = $"overlap {a.Id}-{b.Id}: dist={dist:F5} min={minDist:F5}";
                return false;
            }
        }
    }

    reason = string.Empty;
    return true;
}

static bool ValidateNumericFrame(SimulationFrame frame, out string reason)
{
    for (int i = 0; i < frame.Balls.Length; i++)
    {
        BallState ball = frame.Balls[i];
        if (!IsFinite(ball.Position.X) || !IsFinite(ball.Position.Y) || !IsFinite(ball.Position.Z) ||
            !IsFinite(ball.Velocity.X) || !IsFinite(ball.Velocity.Y) || !IsFinite(ball.Velocity.Z) ||
            !IsFinite(ball.AngularVelocity.X) || !IsFinite(ball.AngularVelocity.Y) || !IsFinite(ball.AngularVelocity.Z))
        {
            reason = $"ball {ball.Id} has non-finite values";
            return false;
        }
    }

    reason = string.Empty;
    return true;
}

static bool IsFinite(double value) => !(double.IsNaN(value) || double.IsInfinity(value));

static bool FramesNearlyEqual(SimulationFrame a, SimulationFrame b, double epsilon)
{
    if (a.Balls.Length != b.Balls.Length)
    {
        return false;
    }

    for (int i = 0; i < a.Balls.Length; i++)
    {
        BallState x = a.Balls[i];
        BallState y = b.Balls[i];
        if (x.Id != y.Id || x.InPlay != y.InPlay || x.IsPocketed != y.IsPocketed)
        {
            return false;
        }

        if ((x.Position - y.Position).Length > epsilon ||
            (x.Velocity - y.Velocity).Length > epsilon ||
            (x.AngularVelocity - y.AngularVelocity).Length > epsilon)
        {
            return false;
        }
    }

    return true;
}

static string ResolveFixturePath(string? argPath)
{
    if (!string.IsNullOrWhiteSpace(argPath))
    {
        return Path.GetFullPath(argPath);
    }

    string cwdCandidate = Path.Combine(Environment.CurrentDirectory, "fixtures", "shot_suite_baseline.json");
    if (File.Exists(cwdCandidate))
    {
        return cwdCandidate;
    }

    return Path.Combine(AppContext.BaseDirectory, "fixtures", "shot_suite_baseline.json");
}

static string ResolveBenchmarkReportPath(string? argPath)
{
    if (!string.IsNullOrWhiteSpace(argPath))
    {
        return Path.GetFullPath(argPath);
    }

    string? repoRoot = TryFindRepositoryRoot();
    if (!string.IsNullOrWhiteSpace(repoRoot))
    {
        return Path.Combine(repoRoot, "docs", "BENCHMARK_MATRIX.md");
    }

    return Path.Combine(Environment.CurrentDirectory, "BENCHMARK_MATRIX.md");
}

static string? TryFindRepositoryRoot()
{
    string[] startPaths = { Environment.CurrentDirectory, AppContext.BaseDirectory };

    for (int i = 0; i < startPaths.Length; i++)
    {
        DirectoryInfo? dir = new DirectoryInfo(startPaths[i]);
        while (dir is not null)
        {
            bool hasRepoMarkers =
                File.Exists(Path.Combine(dir.FullName, "README.md")) &&
                Directory.Exists(Path.Combine(dir.FullName, "docs")) &&
                Directory.Exists(Path.Combine(dir.FullName, "src")) &&
                Directory.Exists(Path.Combine(dir.FullName, "tests"));

            if (hasRepoMarkers)
            {
                return dir.FullName;
            }

            dir = dir.Parent;
        }
    }

    return null;
}

static double NormalizeDeltaAngle(double a, double b)
{
    double delta = (a - b) % 360.0;
    if (delta > 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    return delta;
}

internal readonly struct SimResult
{
    public bool Success { get; }
    public string Error { get; }
    public PhysVector3 FinalPosition { get; }
    public double ExitAngleDeg { get; }
    public double SettleSeconds { get; }
    public bool ReachedRest { get; }
    public int FinalMovingCount { get; }
    public int EventCount { get; }
    public SimulationFrame FinalFrame { get; }

    private SimResult(
        bool success,
        string error,
        in PhysVector3 finalPosition,
        double exitAngleDeg,
        double settleSeconds,
        bool reachedRest,
        int finalMovingCount,
        int eventCount,
        in SimulationFrame finalFrame)
    {
        Success = success;
        Error = error;
        FinalPosition = finalPosition;
        ExitAngleDeg = exitAngleDeg;
        SettleSeconds = settleSeconds;
        ReachedRest = reachedRest;
        FinalMovingCount = finalMovingCount;
        EventCount = eventCount;
        FinalFrame = finalFrame;
    }

    public static SimResult Ok(
        in PhysVector3 finalPosition,
        double exitAngleDeg,
        double settleSeconds,
        bool reachedRest,
        int finalMovingCount,
        int eventCount,
        in SimulationFrame finalFrame)
        => new(true, string.Empty, finalPosition, exitAngleDeg, settleSeconds, reachedRest, finalMovingCount, eventCount, finalFrame);

    public static SimResult Fail(string error)
        => new(false, error, PhysVector3.Zero, 0.0, 0.0, false, 0, 0, SimulationFrame.Empty);
}

internal readonly struct BenchmarkResult
{
    public int SimulationHz { get; }
    public int RequestedShots { get; }
    public int CompletedShots { get; }
    public int RejectedShots { get; }
    public double AvgMs { get; }
    public double P50Ms { get; }
    public double P95Ms { get; }
    public double P99Ms { get; }
    public double WorstMs { get; }
    public double AvgSteps { get; }
    public double AvgSimSeconds { get; }
    public double MsPerSimSecond { get; }
    public double RealTimeFactor { get; }
    public double PerfBudgetMs { get; }
    public int PerfWarnCount { get; }

    public BenchmarkResult(
        int simulationHz,
        int requestedShots,
        int completedShots,
        int rejectedShots,
        double avgMs,
        double p50Ms,
        double p95Ms,
        double p99Ms,
        double worstMs,
        double avgSteps,
        double avgSimSeconds,
        double msPerSimSecond,
        double realTimeFactor,
        double perfBudgetMs,
        int perfWarnCount)
    {
        SimulationHz = simulationHz;
        RequestedShots = requestedShots;
        CompletedShots = completedShots;
        RejectedShots = rejectedShots;
        AvgMs = avgMs;
        P50Ms = p50Ms;
        P95Ms = p95Ms;
        P99Ms = p99Ms;
        WorstMs = worstMs;
        AvgSteps = avgSteps;
        AvgSimSeconds = avgSimSeconds;
        MsPerSimSecond = msPerSimSecond;
        RealTimeFactor = realTimeFactor;
        PerfBudgetMs = perfBudgetMs;
        PerfWarnCount = perfWarnCount;
    }

    public static BenchmarkResult Empty(int simulationHz, int requestedShots, double perfBudgetMs)
        => new(simulationHz, requestedShots, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, perfBudgetMs, 0);
}

internal sealed class RunnerOptions
{
    public bool RecordMode { get; private set; }
    public bool AnalyzeMode { get; private set; }
    public bool SanityMode { get; private set; }
    public int DeterminismRepeats { get; private set; }
    public int StressShots { get; private set; }
    public int BenchmarkShots { get; private set; }
    public int BenchmarkMatrixShots { get; private set; }
    public int RandomSeed { get; private set; } = 1337;
    public string ProfilePreset { get; private set; } = "physics";
    public double PerfBudgetMs { get; private set; }
    public bool ForceBaseline { get; private set; }
    public string? FixturePath { get; private set; }
    public string? BenchmarkReportPath { get; private set; }

    public static RunnerOptions Parse(string[] args)
    {
        RunnerOptions options = new();

        for (int i = 0; i < args.Length; i++)
        {
            string arg = args[i];
            switch (arg)
            {
                case "--record":
                    options.RecordMode = true;
                    break;

                case "--analyze":
                    options.AnalyzeMode = true;
                    break;

                case "--sanity":
                    options.SanityMode = true;
                    break;

                case "--determinism":
                    options.DeterminismRepeats = ReadIntArg(args, ref i, 3, minValue: 2);
                    break;

                case "--stress":
                    options.StressShots = ReadIntArg(args, ref i, 100, minValue: 1);
                    break;

                case "--benchmark":
                    options.BenchmarkShots = ReadIntArg(args, ref i, 200, minValue: 1);
                    break;

                case "--benchmark-matrix":
                    options.BenchmarkMatrixShots = ReadIntArg(args, ref i, 120, minValue: 1);
                    break;

                case "--seed":
                    options.RandomSeed = ReadIntArg(args, ref i, options.RandomSeed, minValue: int.MinValue);
                    break;

                case "--profile":
                    options.ProfilePreset = ReadStringArg(args, ref i, options.ProfilePreset);
                    break;

                case "--perf-budget-ms":
                    options.PerfBudgetMs = ReadDoubleArg(args, ref i, 0.0, minValue: 0.0);
                    break;

                case "--force-baseline":
                    options.ForceBaseline = true;
                    break;

                case "--benchmark-report":
                    if (i + 1 < args.Length)
                    {
                        options.BenchmarkReportPath = args[++i];
                    }
                    break;

                case "--fixtures":
                    if (i + 1 < args.Length)
                    {
                        options.FixturePath = args[++i];
                    }
                    break;
            }
        }

        return options;
    }

    private static int ReadIntArg(string[] args, ref int i, int fallback, int minValue)
    {
        if (i + 1 >= args.Length)
        {
            return fallback;
        }

        string candidate = args[i + 1];
        if (candidate.StartsWith("--", StringComparison.Ordinal))
        {
            return fallback;
        }

        i++;
        if (!int.TryParse(candidate, NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed))
        {
            return fallback;
        }

        return Math.Max(minValue, parsed);
    }

    private static double ReadDoubleArg(string[] args, ref int i, double fallback, double minValue)
    {
        if (i + 1 >= args.Length)
        {
            return fallback;
        }

        string candidate = args[i + 1];
        if (candidate.StartsWith("--", StringComparison.Ordinal))
        {
            return fallback;
        }

        i++;
        if (!double.TryParse(candidate, NumberStyles.Float | NumberStyles.AllowThousands, CultureInfo.InvariantCulture, out double parsed))
        {
            return fallback;
        }

        return Math.Max(minValue, parsed);
    }

    private static string ReadStringArg(string[] args, ref int i, string fallback)
    {
        if (i + 1 >= args.Length)
        {
            return fallback;
        }

        string candidate = args[i + 1];
        if (candidate.StartsWith("--", StringComparison.Ordinal))
        {
            return fallback;
        }

        i++;
        return candidate;
    }
}

internal sealed class ShotFixture
{
    public string Name { get; set; } = string.Empty;
    public int ShotBallId { get; set; }
    public int TrackBallId { get; set; }
    public double[] AimDirection { get; set; } = new double[3];
    public double Impulse { get; set; }
    public double[] TipOffset { get; set; } = new double[2];
    public double CueElevationDeg { get; set; }
    public double CueMass { get; set; } = 0.525;
    public double TipRadius { get; set; } = 0.006;
    public double SpinMultiplier { get; set; } = 18.0;
    public double HorizonSeconds { get; set; } = 8.0;
    public bool RequireRestByHorizon { get; set; } = false;
    public double[] ExpectedFinalPosition { get; set; } = new double[3];
    public double ExpectedExitAngleDeg { get; set; }
    public double ExpectedSettleSeconds { get; set; }
}
