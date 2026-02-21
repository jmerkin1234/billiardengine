using System;
using System.Collections.Generic;
using BilliardsPhysicsEngine;
using BilliardsPhysicsEngine.Config;
using BilliardsPhysicsEngine.Core;
using BilliardsPhysicsEngine.Prediction;
using UnityEngine;

namespace BilliardsUnityAdapter.Runtime;

/// <summary>
/// Engine-authoritative Unity bridge.
/// This file is intentionally standalone so it can be copied into a Unity project.
/// </summary>
public class UnityPhysicsRuntimeController : MonoBehaviour
{
    [Serializable]
    private sealed class BallBinding
    {
        public int Id;
        public bool IsCueBall;
        public GameObject GameObject = null!;
        public Transform Transform = null!;
        public Rigidbody? Rigidbody;
        public Collider? Collider;
        public Renderer[] Renderers = Array.Empty<Renderer>();
        public bool Pocketed;
    }

    [Header("Engine")]
    [SerializeField] private bool engineAuthoritative = true;
    [SerializeField] private bool autoInitializeOnAwake = true;
    [SerializeField] private bool autoDiscoverBalls = true;
    [SerializeField] private bool setRigidbodiesKinematic = true;
    [SerializeField] private bool applyVisualSpin = true;

    [Header("Config")]
    [SerializeField, Range(240, 500)] private int simulationHz = 480;
    [SerializeField] private Transform[] explicitBalls = Array.Empty<Transform>();
    [SerializeField] private Transform[] pocketCenters = Array.Empty<Transform>();

    private readonly Dictionary<int, BallBinding> _bindingById = new();
    private readonly Dictionary<GameObject, int> _idByGameObject = new();

    private BilliardPhysicsEngine? _engine;
    private PhysicsProfile? _profile;
    private RackPreset? _defaultRack;
    private CueShotInput? _lastShot;

    public static UnityPhysicsRuntimeController? Instance { get; private set; }

    public static event Action<GameObject, GameObject>? OnCueBallObjectBallContact;
    public static event Action<GameObject, int>? OnBallPocketed;
    public static event Action? OnAllBallsStopped;

    public bool IsEngineAuthoritative => engineAuthoritative;
    public bool IsInitialized => _engine is not null;
    public int CueBallId { get; private set; } = -1;

    private void Awake()
    {
        if (Instance is not null && Instance != this)
        {
            Destroy(this);
            return;
        }

        Instance = this;
        if (autoInitializeOnAwake)
        {
            Initialize();
        }
    }

    private void OnDestroy()
    {
        if (Instance == this)
        {
            Instance = null;
        }
    }

    private void FixedUpdate()
    {
        if (_engine is null)
        {
            return;
        }

        SimulationFrame frame = _engine.Step(Time.fixedDeltaTime);
        ApplyFrame(frame);
        EmitEvents(frame);
    }

    public void Initialize()
    {
        _profile = new PhysicsProfile { SimulationHz = simulationHz };
        _engine = new BilliardPhysicsEngine();

        List<BallBinding> bindings = DiscoverBalls();
        if (bindings.Count == 0)
        {
            Debug.LogError("[UnityPhysicsRuntimeController] No balls found");
            return;
        }

        TableGeometry table = BuildTableGeometry(bindings);
        List<BallInitState> initial = new(bindings.Count);

        _bindingById.Clear();
        _idByGameObject.Clear();

        for (int i = 0; i < bindings.Count; i++)
        {
            BallBinding binding = bindings[i];
            binding.Id = i;
            binding.IsCueBall = binding.GameObject.CompareTag("CueBall");
            _bindingById[i] = binding;
            _idByGameObject[binding.GameObject] = i;

            if (binding.IsCueBall)
            {
                CueBallId = i;
            }

            initial.Add(BallInitState.Create(
                i,
                new PhysVector3(binding.Transform.position.x, binding.Transform.position.y, binding.Transform.position.z),
                _profile.BallRadius,
                _profile.BallMass,
                binding.IsCueBall));

            if (binding.Rigidbody is not null)
            {
                binding.Rigidbody.mass = (float)_profile.BallMass;
                binding.Rigidbody.useGravity = false;
                binding.Rigidbody.linearVelocity = Vector3.zero;
                binding.Rigidbody.angularVelocity = Vector3.zero;
                binding.Rigidbody.detectCollisions = !engineAuthoritative;
                binding.Rigidbody.isKinematic = engineAuthoritative && setRigidbodiesKinematic;
            }
        }

        _engine.Initialize(table, _profile, initial);
        _defaultRack = new RackPreset("SceneInitial", initial.ToArray());
        _lastShot = null;

        ApplyFrame(_engine.Step(0.0));
    }

    public bool TryGetBallId(GameObject ball, out int ballId) => _idByGameObject.TryGetValue(ball, out ballId);

    public bool ApplyCueShot(CueShotInput shot)
    {
        if (_engine is null)
        {
            return false;
        }

        bool ok = _engine.CueStrike(shot);
        if (ok)
        {
            _lastShot = shot;
        }

        return ok;
    }

    public bool ApplyCueShot(GameObject cueBall, CueShotInput shot)
    {
        if (!TryGetBallId(cueBall, out int id))
        {
            return false;
        }

        CueShotInput resolved = new(
            id,
            shot.AimDirection,
            shot.LinearImpulse,
            shot.TipOffset,
            shot.CueElevationDeg,
            shot.CueMass,
            shot.TipRadius,
            shot.SpinMultiplier);

        return ApplyCueShot(resolved);
    }

    public bool RepeatLastShot() => _lastShot.HasValue && ApplyCueShot(_lastShot.Value);

    public TrajectoryPrediction PredictShot(CueShotInput shot, PredictionSettings settings)
    {
        if (_engine is null)
        {
            return new TrajectoryPrediction();
        }

        return _engine.GetTrajectoryPrediction(shot, settings);
    }

    public void ResetToDefaultRack()
    {
        if (_engine is null || _defaultRack is null)
        {
            return;
        }

        _engine.ResetToRack(_defaultRack);
        RestoreAllVisuals();
        ApplyFrame(_engine.Step(0.0));
    }

    public bool TrySetBallPosition(GameObject ball, Vector3 position, bool clearMotion)
    {
        if (_engine is null || !TryGetBallId(ball, out int id))
        {
            return false;
        }

        bool ok = _engine.TrySetBallPosition(id, new PhysVector3(position.x, position.y, position.z), clearMotion);
        if (ok)
        {
            ApplyFrame(_engine.Step(0.0));
        }

        return ok;
    }

    public bool TryApplyImpulse(GameObject ball, Vector3 impulse)
    {
        if (_engine is null || !TryGetBallId(ball, out int id))
        {
            return false;
        }

        return _engine.TryApplyImpulse(id, new PhysVector3(impulse.x, impulse.y, impulse.z));
    }

    public bool TryInjectSpin(GameObject ball, Vector3 deltaAngularVelocity)
    {
        if (_engine is null || !TryGetBallId(ball, out int id))
        {
            return false;
        }

        return _engine.TryInjectSpin(id, new PhysVector3(deltaAngularVelocity.x, deltaAngularVelocity.y, deltaAngularVelocity.z));
    }

    public bool TryGetBallState(GameObject ball, out BallState state)
    {
        state = default;
        if (_engine is null || !TryGetBallId(ball, out int id))
        {
            return false;
        }

        return _engine.TryGetBallState(id, out state);
    }

    private List<BallBinding> DiscoverBalls()
    {
        var list = new List<BallBinding>();
        var seen = new HashSet<Transform>();

        foreach (Transform t in explicitBalls)
        {
            if (t is null || !seen.Add(t))
            {
                continue;
            }

            list.Add(CreateBinding(t));
        }

        if (autoDiscoverBalls)
        {
            foreach (GameObject go in GameObject.FindGameObjectsWithTag("Ball"))
            {
                if (go is null || !seen.Add(go.transform))
                {
                    continue;
                }

                list.Add(CreateBinding(go.transform));
            }

            foreach (GameObject go in GameObject.FindGameObjectsWithTag("CueBall"))
            {
                if (go is null || !seen.Add(go.transform))
                {
                    continue;
                }

                list.Add(CreateBinding(go.transform));
            }
        }

        list.Sort((a, b) =>
        {
            if (a.IsCueBall != b.IsCueBall)
            {
                return a.IsCueBall ? -1 : 1;
            }

            return string.CompareOrdinal(a.GameObject.name, b.GameObject.name);
        });

        return list;
    }

    private static BallBinding CreateBinding(Transform t)
    {
        return new BallBinding
        {
            GameObject = t.gameObject,
            Transform = t,
            Rigidbody = t.GetComponent<Rigidbody>(),
            Collider = t.GetComponent<Collider>(),
            Renderers = t.GetComponentsInChildren<Renderer>(true),
            IsCueBall = t.CompareTag("CueBall")
        };
    }

    private TableGeometry BuildTableGeometry(IReadOnlyList<BallBinding> balls)
    {
        double surfaceY = 0.0;
        if (balls.Count > 0)
        {
            surfaceY = balls.Min(b => b.Transform.position.y) - (_profile?.BallRadius ?? 0.028575);
        }

        TableGeometry table = TableGeometry.CreateEightFoot(surfaceY);

        if (pocketCenters.Length > 0)
        {
            var pockets = new List<PocketGeometry>();
            for (int i = 0; i < pocketCenters.Length; i++)
            {
                Transform p = pocketCenters[i];
                if (p is null)
                {
                    continue;
                }

                pockets.Add(new PocketGeometry(
                    pockets.Count,
                    new PhysVector3(p.position.x, surfaceY, p.position.z),
                    0.080,
                    0.050,
                    0.020,
                    0.70,
                    0.25));
            }

            if (pockets.Count > 0)
            {
                table = new TableGeometry(table.MinXZ, table.MaxXZ, table.SurfaceY, table.CushionNoseHeight, pockets.ToArray());
            }
        }

        return table;
    }

    private void ApplyFrame(SimulationFrame frame)
    {
        for (int i = 0; i < frame.Balls.Length; i++)
        {
            BallState state = frame.Balls[i];
            if (!_bindingById.TryGetValue(state.Id, out BallBinding? binding))
            {
                continue;
            }

            binding.Transform.position = new Vector3((float)state.Position.X, (float)state.Position.Y, (float)state.Position.Z);

            if (applyVisualSpin && !state.IsPocketed)
            {
                float speed = (float)state.AngularVelocity.Length;
                if (speed > 0.0001f)
                {
                    Vector3 axis = new((float)state.AngularVelocity.X, (float)state.AngularVelocity.Y, (float)state.AngularVelocity.Z);
                    binding.Transform.rotation = Quaternion.AngleAxis(speed * Time.fixedDeltaTime * Mathf.Rad2Deg, axis.normalized) * binding.Transform.rotation;
                }
            }

            if (binding.Rigidbody is not null)
            {
                binding.Rigidbody.linearVelocity = new Vector3((float)state.Velocity.X, (float)state.Velocity.Y, (float)state.Velocity.Z);
                binding.Rigidbody.angularVelocity = new Vector3((float)state.AngularVelocity.X, (float)state.AngularVelocity.Y, (float)state.AngularVelocity.Z);
            }

            if (state.IsPocketed && !binding.Pocketed)
            {
                binding.Pocketed = true;
                SetBallVisible(binding, false);
            }
        }
    }

    private void EmitEvents(SimulationFrame frame)
    {
        bool firedAllStopped = false;

        for (int i = 0; i < frame.Events.Length; i++)
        {
            PhysicsEvent e = frame.Events[i];
            switch (e.Type)
            {
                case PhysicsEventType.BallBallContact:
                    if (_bindingById.TryGetValue(e.BallAId, out BallBinding? a) && _bindingById.TryGetValue(e.BallBId, out BallBinding? b))
                    {
                        if (a.IsCueBall && !b.IsCueBall)
                        {
                            OnCueBallObjectBallContact?.Invoke(a.GameObject, b.GameObject);
                        }
                        else if (b.IsCueBall && !a.IsCueBall)
                        {
                            OnCueBallObjectBallContact?.Invoke(b.GameObject, a.GameObject);
                        }
                    }
                    break;

                case PhysicsEventType.Pocketed:
                    if (_bindingById.TryGetValue(e.BallAId, out BallBinding? p))
                    {
                        OnBallPocketed?.Invoke(p.GameObject, e.PocketId);
                    }
                    break;

                case PhysicsEventType.AllBallsStopped:
                    firedAllStopped = true;
                    break;
            }
        }

        if (firedAllStopped)
        {
            OnAllBallsStopped?.Invoke();
        }
    }

    private static void SetBallVisible(BallBinding binding, bool visible)
    {
        if (binding.Collider is not null)
        {
            binding.Collider.enabled = visible;
        }

        if (binding.Rigidbody is not null)
        {
            binding.Rigidbody.detectCollisions = visible;
        }

        for (int i = 0; i < binding.Renderers.Length; i++)
        {
            binding.Renderers[i].enabled = visible;
        }
    }

    private void RestoreAllVisuals()
    {
        foreach (BallBinding binding in _bindingById.Values)
        {
            binding.Pocketed = false;
            SetBallVisible(binding, true);
        }
    }
}
