using System;
using System.Collections.Generic;
using System.IO;
using Games.Rebuilt.GameManagement;
using GameSystems.Management;
using Games.Rebuilt.Robots;
using Games.Rebuilt.Components;
using Games.Rebuilt.Scoring;
using Games.Rebuilt.GamePieceSystem;
using MoSimCore.BaseClasses.GameManagement;
using MoSimCore.Enums;
using System.Linq;
using RobotFramework.Components;
using RobotFramework.Controllers.Drivetrain;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.LowLevel;
using Object = UnityEngine.Object;

[InitializeOnLoad]
public static class GoldfirePlaytest
{
    private const string Query = "GoldfirePlaytest.Active";
    private const string RunHub = "GoldfirePlaytest.RunHub";
    private const string RunAdvanced = "GoldfirePlaytest.RunAdvanced";
    private const string RunFull = "GoldfirePlaytest.RunFull";
    private const string RunContactDiagnostics = "GoldfirePlaytest.RunContactDiagnostics";
    private const string RunClimb = "GoldfirePlaytest.RunClimb";
    private static Vector3 contactStart;
    private static float contactStartTime;
    private static float highestChassis;
    private static float highestTilt;
    private static float largestChassisSpeed;
    private static float extendedHook;
    private static bool fieldContactVerified;
    private static bool testedClimb;
    private static bool stowedTrenchBlocked;
    private static bool deployedTrenchCleared;
    private static Vector3 trenchStart;
    private static readonly float[] AdvancedRanges = { 3f, 4.9f, 3f, 3f };
    private static int advancedTrial;
    private static bool failedAdvanced;
    private static bool advancedLaunched;
    private static bool passLanded;
    private static Vector3 advancedTarget;
    private static Vector3 ballLanding;
    private static Vector3 releaseRobotVelocity;
    private static Vector3 releaseVelocity;
    private static Vector3 previousBallPosition;
    private static float releaseTime;
    private static float releaseLead;
    private static readonly float[] HubRanges = { 2f, 2.5f, 3.5f, 4.3f };
    private static int hubTrial;
    private static Vector3 hubPoint;
    private static Vector3 hubAimPoint;
    private static Vector3 hubShotDirection;
    private static Vector3 hubOpeningCrossing;
    private static bool crossedHubOpeningHeight;
    private static bool scoredHub;
    private static bool intakeFacingHub;
    private static float closestHubMiss;
    private static float maximumHeight;
    private static bool failedHub;
    private static bool failedMechanisms;
    private static double deadline;
    private static double stageStart;
    private static float slideStowed;
    private static int stage;
    private static Gamepad gamepad;
    private static Vector3 initialPosition;
    private static Transform carriage;
    private static AwtybotsV1 robot;
    private static Rigidbody fuel;
    private static float nearestZ;
    private static float furthestZ;
    private static float highestY;
    private static float fastest;
    private static bool pickedUp;
    private static int rampTrial;
    private static Vector3 lastIntakePosition;
    private static float intakeAxleHeight;
    private static float bumperPassHeight;
    private static float hopperEntryHeight;
    private static int intakeTraceFrame;
    private static Vector3 intakeHoldPosition;
    private static bool neutralIntakeStable;
    private static int containmentTrial;
    private static float containmentStarted;
    private static bool passedContainment = true;
    private static bool loadedStable;
    private static bool fuelStaysAboveFloor = true;
    private static readonly Vector2[] FloorDropSites =
    {
        new(-0.25f, -0.15f), new(0f, -0.28f), new(0.25f, -0.15f),
        new(-0.20f, 0.15f), new(0f, 0.25f), new(0.20f, 0.15f),
    };
    private static Rigidbody[] floorDropFuel;
    private static float[] lowestDropHeight;
    private static Collider[] coveredFloorColliders;
    private static readonly (string Name, Vector3 Position, Vector3 Velocity, float CrossingLimit)[] ContainmentTrials =
    {
        ("fixed side", new Vector3(0.22f, 0.34f, 0.09f), new Vector3(3.8f, 0f, 0f), 0.355f),
        ("sliding side", new Vector3(-0.22f, 0.36f, 0.53f), new Vector3(-3.8f, 0f, 0f), 0.341f),
        ("front intake pane", new Vector3(0.06f, 0.43f, 0.53f), new Vector3(0f, 0f, 3.8f), 0.655f),
        ("open top", new Vector3(0.22f, 0.74f, 0.20f), new Vector3(3.8f, 0f, 0f), 0.36f),
    };

    private static int pickupsPassed;
    private static double lastFeedTrace;
    private static bool hopperLaunch;
    private static int towerStation;
    private static float widestHopperRoute;
    private const float VortexThrough3To1 = 6784f / 3f;
    private const float IntakeShaftRpm = VortexThrough3To1 * 27f / 26f;
    private static bool collectedFuelLaunched;
    private static float reverseStarted;
    private static readonly Dictionary<Transform, Quaternion> previousVisualRotation = new();
    private static readonly HashSet<Transform> rotatingVisuals = new();

    static GoldfirePlaytest()
    {
        EditorApplication.update += Tick;
    }

    public static void AssembleAndStart()
    {
        GoldfireAuthoring.Assemble();
        Start();
    }

    public static void StartFull()
    {
        SessionState.SetBool(RunFull, true);
        Start();
    }

    public static void AssembleAndStartFull()
    {
        GoldfireAuthoring.Assemble();
        StartFull();
    }

    public static void StartContactDiagnostics()
    {
        SessionState.SetBool(RunContactDiagnostics, true);
        Start();
    }

    private static CapsuleCollider climbRung;

    private static void LogRungInHook(string label)
    {
        var stage = robot.transform.Find("Climber Slider");
        if (!climbRung || !stage) return;
        Vector3 origin = climbRung.transform.TransformPoint(climbRung.center);
        Vector3 notch = stage.TransformPoint(new Vector3(-0.324f, 0.518f, -0.096f));
        Vector3 axis = climbRung.transform.TransformDirection(climbRung.direction == 0 ? Vector3.right : climbRung.direction == 1 ? Vector3.up : Vector3.forward).normalized;
        Vector3 point = origin + axis * Vector3.Dot(notch - origin, axis);
        Vector3 local = stage.InverseTransformPoint(point);
        var body = robot.GetComponent<Rigidbody>();
        Debug.Log($"GOLDFIRE CLIMB DIAG {label}: rung in stage={local.ToString("F3")}, axis·right={Vector3.Dot(stage.right, axis):F2}, slide={stage.localPosition.y:F3}, chassis={body.position.ToString("F3")}, v={body.velocity.ToString("F2")}, latched={robot.GetComponentInChildren<FixedJoint>() != null}, grounded={robot.GetComponent<DriveController>().IsTouchingGround}, " +
            $"climbJoint target={stage.GetComponent<ConfigurableJoint>().targetVelocity.ToString("F3")} force={stage.GetComponent<ConfigurableJoint>().currentForce.ToString("F0")} " +
            $"latchForce={(robot.GetComponentInChildren<FixedJoint>() ? robot.GetComponentInChildren<FixedJoint>().currentForce.ToString("F0") : "-")} chassisMass={body.mass:F1} constraints={body.constraints}.");
    }

    public static void AssembleAndStartClimb()
    {
        GoldfireAuthoring.Assemble();
        SessionState.SetBool(RunClimb, true);
        Start();
    }

    private static void LogRobotPenetrations(string label)
    {
        var hits = new Collider[24];
        foreach (var part in robot.GetComponentsInChildren<Collider>())
        {
            if (!part.enabled || part.isTrigger) continue;
            int count = Physics.OverlapBoxNonAlloc(part.bounds.center, part.bounds.extents + Vector3.one * 0.005f, hits, Quaternion.identity, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < count; i++)
            {
                var other = hits[i];
                if (other.transform.IsChildOf(robot.transform) || other.GetComponentInParent<RebuiltGamePieceController>()) continue;
                if (Physics.GetIgnoreLayerCollision(part.gameObject.layer, other.gameObject.layer)) continue;
                if (Physics.ComputePenetration(part, part.transform.position, part.transform.rotation,
                        other, other.transform.position, other.transform.rotation, out var direction, out var distance))
                    Debug.Log($"GOLDFIRE CLIMB DIAG {label}: {part.name} ({part.transform.parent?.name}, layer {part.gameObject.layer}) touches {other.name} ({other.transform.parent?.name}, layer {other.gameObject.layer}) depth={distance:F4} dir={direction.ToString("F2")}.");
            }
        }
        foreach (var wheel in robot.GetComponentsInChildren<WheelCollider>())
            Debug.Log($"GOLDFIRE CLIMB DIAG {label}: wheel {wheel.transform.parent?.name}/{wheel.name} grounded={wheel.isGrounded} force={(wheel.GetGroundHit(out var hit) ? hit.force : 0f):F0} collider={(wheel.GetGroundHit(out hit) ? hit.collider.name : "-")}.");
    }

    public static void AssembleAndStartContactDiagnostics()
    {
        GoldfireAuthoring.Assemble();
        StartContactDiagnostics();
    }

    private static void LogTrenchContacts(string pose)
    {
        var body = robot.GetComponent<Rigidbody>();
        Debug.Log($"GOLDFIRE TRENCH DIAG {pose}: root={body.position.ToString("F3")}, velocity={body.velocity.ToString("F3")}, tilt={Vector3.Angle(robot.transform.up, Vector3.up):F1}deg.");
        var hits = new Collider[16];
        foreach (var part in robot.GetComponentsInChildren<Collider>())
        {
            if (!part.enabled || part.isTrigger || part.bounds.max.y < 0.45f)
                continue;
            if (part.layerOverridePriority > 0 && (part.includeLayers.value & (1 << 3)) == 0)
                continue;
            var b = part.bounds;
            int count = Physics.OverlapBoxNonAlloc(b.center, b.extents + Vector3.one * 0.012f, hits, Quaternion.identity, 1 << 3, QueryTriggerInteraction.Ignore);
            var touching = Enumerable.Range(0, count).Select(i => hits[i]).Where(hit => !hit.transform.IsChildOf(robot.transform))
                .Select(hit => $"{hit.name}({hit.bounds.min.y:F3})").ToArray();
            Debug.Log($"GOLDFIRE TRENCH DIAG {pose}: {part.name} parent={part.transform.parent.name} top={b.max.y:F3} xRange=({b.min.x:F3},{b.max.x:F3}) near=[{string.Join(", ", touching)}].");
        }
    }

    public static void AssembleAndStartHub()
    {
        GoldfireAuthoring.Assemble();
        StartHub();
    }

    public static void StartHub()
    {
        SessionState.SetBool(RunHub, true);
        Start();
    }

    public static void StartAdvanced()
    {
        SessionState.SetBool(RunAdvanced, true);
        Start();
    }

    public static void AssembleAndStartAdvanced()
    {
        GoldfireAuthoring.Assemble();
        StartAdvanced();
    }

    public static void Start()
    {
        var scene = EditorSceneManager.OpenScene("Assets/Scenes/Rebuilt/Rebuilt.unity");
        var controller = Object.FindFirstObjectByType<RobotSpawnController>(FindObjectsInactive.Include);
        var metadata = AssetDatabase.LoadAssetAtPath<RebuiltRobotMetadataSO>("Assets/Prefabs/Games/Rebuilt/Robots/Mods/AwtybotsRebuilt/GoldfireV1/GoldfireV1.asset");
        if (!controller || !metadata) throw new InvalidOperationException("REBUILT field or Goldfire metadata unavailable.");
        var settings = new SerializedObject(controller);
        settings.FindProperty("robotOverride").objectReferenceValue = metadata;
        settings.ApplyModifiedPropertiesWithoutUndo();
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        if (Gamepad.current == null) gamepad = InputSystem.AddDevice<Gamepad>("GoldfireSmokeController");
        SessionState.SetBool(Query, true);
        EditorApplication.EnterPlaymode();
    }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void ConfigureKeyboard()
    {
        if (!SessionState.GetBool(Query, false)) return;
        Application.runInBackground = true;
        InputSystem.settings.backgroundBehavior = InputSettings.BackgroundBehavior.IgnoreFocus;
        InputSystem.settings.editorInputBehaviorInPlayMode =
            InputSettings.EditorInputBehaviorInPlayMode.AllDeviceInputAlwaysGoesToGameView;
        if (Keyboard.current == null) InputSystem.AddDevice<Keyboard>("GoldfireSmokeKeyboard");
        if (Gamepad.current == null)
            gamepad = InputSystem.AddDevice<Gamepad>("GoldfireSmokeController");
    }

    private static void Tick()
    {
        if (!SessionState.GetBool(Query, false) || !EditorApplication.isPlaying)
            return;

        if (EditorApplication.isPaused) EditorApplication.isPaused = false;
        double now = EditorApplication.timeSinceStartup;
        if (deadline == 0)
            deadline = now + 75;
        if (now > deadline)
        {
            Complete($"Timed out at stage {stage}.", 1);
            return;
        }

        try
        {
            ObserveVisualMotion();
            switch (stage)
            {
                case 0:
                    robot = Object.FindFirstObjectByType<AwtybotsV1>();
                    if (!robot) return;
                    carriage = robot.transform.Find("Intake Rails/Intake Carriage");
                    if (!carriage) throw new InvalidOperationException("Could not find the intake carriage.");
                    gamepad = Gamepad.current ?? InputSystem.AddDevice<Gamepad>("GoldfireSmokeController");
                    var keyboard = Keyboard.current ?? InputSystem.AddDevice<Keyboard>("GoldfireSmokeKeyboard");
                    robot.GetComponent<PlayerInput>().SwitchCurrentControlScheme("Robot 1", gamepad, keyboard);

                    if (BaseGameManager.Instance.RobotState != RobotState.Enabled)
                    {
                        Object.FindFirstObjectByType<RebuiltTimerManager>().StartMatch();
                    }
                    Object.FindFirstObjectByType<RebuiltTimerManager>().PauseTimer();
                    Debug.Log("GOLDFIRE TEST: Goldfire V1 spawned in REBUILT.");
                    foreach (var pivot in robot.GetComponentsInChildren<Transform>().Where(part =>
                        part.name.EndsWith(" Visual Pivot", StringComparison.Ordinal) ||
                        part.name.EndsWith(" Roll Pivot", StringComparison.Ordinal)))
                        previousVisualRotation[pivot] = pivot.localRotation;
                    if (SessionState.GetBool(RunAdvanced, false))
                    {
                        stage = 30;
                        stageStart = now;
                        break;
                    }
                    if (SessionState.GetBool(RunHub, false))
                    {
                        stage = 20;
                        stageStart = now;
                        break;
                    }
                    if (SessionState.GetBool(RunClimb, false))
                    {
                        robot.SetGameplayInputEnabled(true);
                        robot.TranslateAction.actionMap.Enable();
                        InputSystem.EnableDevice(gamepad);
                        foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                            piece.gameObject.SetActive(false);
                        SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                        stage = 56;
                        stageStart = now;
                        break;
                    }
                    if (SessionState.GetBool(RunContactDiagnostics, false))
                    {
                        stage = 50;
                        stageStart = now;
                        break;
                    }
                    Next(now);
                    break;
                case 1:
                    if (BaseGameManager.Instance.RobotState != RobotState.Enabled || now - stageStart < 1.2) return;
                    robot.SetGameplayInputEnabled(true);
                    robot.TranslateAction.actionMap.Enable();
                    InputSystem.EnableDevice(gamepad);
                    initialPosition = robot.transform.position;
                    slideStowed = carriage.localPosition.z;
                    SetPad(new GamepadState { leftStick = Vector2.up });
                    Next(now);
                    break;
                case 2:
                    if (now - stageStart < 1.0) return;
                    var input = robot.GetComponent<PlayerInput>();
                    var motion = robot.TranslateAction.ReadValue<Vector2>();
                    var wheels = robot.GetComponentsInChildren<WheelCollider>();
                    var grounded = wheels.Count(wheel => wheel.GetGroundHit(out _));
                    var rb = robot.GetComponent<Rigidbody>();
                    Debug.Log($"GOLDFIRE TEST: pad stick={gamepad.leftStick.ReadValue()}; player input={motion}; scheme={input.currentControlScheme}; translate enabled={robot.TranslateAction.enabled}; translate controls={string.Join(", ", robot.TranslateAction.controls.Select(c => c.path + "@" + c.device.deviceId))}; paired={input.devices.Count}; gamepad identifier={gamepad.deviceId}; wheels on ground={grounded}/{wheels.Length}; spawn pos={initialPosition}; current={rb.position}; velocity={rb.velocity}; simulation time={Time.time:F2}.");
                    Debug.Log($"GOLDFIRE TEST: Wheel local positions {string.Join("; ", wheels.Select(w => w.name + "=" + w.transform.localPosition + " suspension " + w.suspensionDistance.ToString("F3")))}");
                    var offset = robot.transform.position - initialPosition;
                    var distance = new Vector2(offset.x, offset.z).magnitude;
                    Debug.Log($"GOLDFIRE TEST: Drive command moved {distance:F3} m.");
                    if (distance < 0.10f) throw new InvalidOperationException("Drive did not move the robot.");
                    SetPad(new GamepadState { leftTrigger = 1f });
                    Next(now);
                    break;
                case 3:
                    if (now - stageStart < 1.25) return;
                    var stroke = carriage.localPosition.z - slideStowed;
                    Debug.Log($"GOLDFIRE TEST: Intake deployed {stroke:F3} m along the rail.");
                    if (stroke < 0.24f) throw new InvalidOperationException("Intake did not reach deployment travel.");
                    CheckMechanicalSpeeds("intake", 1f, false);
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadDown));
                    Next(now);
                    break;
                case 4:
                    if (now - stageStart < 1.15) return;
                    SetPad(new GamepadState());
                    var remaining = carriage.localPosition.z - slideStowed;
                    Debug.Log($"GOLDFIRE TEST: Intake returned to {remaining:F3} m from stowed.");
                    if (Mathf.Abs(remaining) > 0.045f) throw new InvalidOperationException("Intake did not stow.");
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    Next(now);
                    break;
                case 5:
                    if (now - stageStart < 1.6) return;
                    var flywheels = robot.GetComponentsInChildren<FlywheelBehaviour>();
                    if (flywheels.Length != 2 || flywheels.Any(flywheel => Mathf.Abs(flywheel.TargetRpm + 1735f) > 10f))
                        throw new InvalidOperationException("Manual shot did not spin both flywheels.");
                    Debug.Log("GOLDFIRE TEST: Both flywheels accepted the 1735 RPM manual shot target.");
                    CheckMechanicalSpeeds("shot", -1f, true);
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    MoveTestRobot(new Vector3(6f, 0.018f, 2.55f), Quaternion.LookRotation(Vector3.left));
                    intakeHoldPosition = robot.GetComponent<Rigidbody>().position;
                    foreach (var panel in robot.GetComponentsInChildren<MeshFilter>().Where(item => item.name.Contains("smoked_pane")))
                    {
                        Transform frame = robot.transform;
                        var points = panel.sharedMesh.vertices.Select(vertex => frame.InverseTransformPoint(panel.transform.TransformPoint(vertex))).ToArray();
                        var bounds = new Bounds(points[0], Vector3.zero);
                        foreach (var point in points)
                            bounds.Encapsulate(point);
                        Debug.Log($"GOLDFIRE PANE: {panel.name} min={bounds.min.ToString("F4")} max={bounds.max.ToString("F4")}.");
                    }
                    SetPad(new GamepadState { leftTrigger = 1f });
                    Next(now);
                    break;
                case 6:
                    SetPad(new GamepadState { leftTrigger = 1f });
                    if (now - stageStart < 1.2) return;
                    if (rampTrial == 0)
                    {
                        Vector3 drift = robot.GetComponent<Rigidbody>().position - intakeHoldPosition;
                        var speed = robot.GetComponent<Rigidbody>().velocity;
                        neutralIntakeStable = new Vector2(drift.x, drift.z).magnitude < 0.04f
                            && new Vector2(speed.x, speed.z).magnitude < 0.025f;
                        Debug.Log($"GOLDFIRE TEST: Empty neutral intake displacement={drift.ToString("F4")} with translation={robot.TranslateAction.ReadValue<Vector2>()}, velocity={speed.ToString("F4")}, stable={neutralIntakeStable}.");
                    }
                    fuel = SpawnFuel(new Vector3(rampTrial == 0 ? 0.02f : 0.18f, 0.075f, 0.725f), -0.75f);
                    lastIntakePosition = robot.transform.InverseTransformPoint(fuel.position);
                    intakeAxleHeight = bumperPassHeight = hopperEntryHeight = float.NaN;
                    intakeTraceFrame = 0;

                    BeginObservation();
                    Next(now);
                    break;
                case 7:
                    ObserveFuel();
                    ObserveIntakeRoute();
                    SetPad(new GamepadState { leftTrigger = 1f });
                    if (now - stageStart < 1.65) return;
                    pickedUp = nearestZ < 0.35f && robot.transform.InverseTransformPoint(fuel.position).z < 0.36f
                        && intakeAxleHeight is >= 0.052f and <= 0.119f
                        && bumperPassHeight is >= 0.219f and <= 0.284f
                        && hopperEntryHeight is >= 0.238f and <= 0.299f;
                    Debug.Log($"GOLDFIRE TEST: Intake route {rampTrial}. Under axle Y={intakeAxleHeight:F3}, over bumper Y={bumperPassHeight:F3}, through hopper opening Y={hopperEntryHeight:F3}.");
                    Debug.Log($"GOLDFIRE TEST: Intake trial {rampTrial} floor Fuel closest Z={nearestZ:F3} m, peak Y={highestY:F3} m, peak speed={fastest:F2} m/s, final={robot.transform.InverseTransformPoint(fuel.position)}, rpm={string.Join(", ", carriage.GetComponentsInChildren<InteractionRoller>().Select(r => r.name + "=" + r.realRpm.ToString("F0")))}, slide={carriage.localPosition.z:F3}, picked up={pickedUp}.");
                    if (pickedUp) pickupsPassed++;
                    rampTrial++;
                    if (rampTrial < 2)
                    {
                        Object.Destroy(fuel.gameObject);
                        stage = 6;
                        stageStart = now;
                        break;
                    }
                    pickedUp = pickupsPassed == rampTrial;
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    BeginObservation();
                    towerStation = 0;
                    Next(now);
                    break;
                case 8:
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    ObserveFuel();
                    ObserveTowerRoute();
                    if (now - lastFeedTrace > 0.46)
                    {
                        var kicker = robot.GetComponentsInChildren<InteractionRoller>().First(roller => roller.name == "kicker_lower Physics");
                        var guide = robot.transform.Find("Indexer Fuel Contact");
                        var nearby = Physics.OverlapBox(guide.position, new Vector3(0.27f, 0.25f, 0.30f),
                            guide.rotation, 1 << 12, QueryTriggerInteraction.Ignore);
                        Debug.Log($"GOLDFIRE FEED TRACE: t={now - stageStart:F2}; position={robot.transform.InverseTransformPoint(fuel.position).ToString("F3")}; layerHits={nearby.Length}; kicker={kicker.TargetRpm:F1}; sleep={fuel.IsSleeping()}; kinematic={fuel.isKinematic}; atSpeed={string.Join(", ", robot.GetComponentsInChildren<FlywheelBehaviour>().Select(item => item.IsAtSpeed))}.");
                        lastFeedTrace = now;
                    }
                    if (now - stageStart < 2.8) return;
                    collectedFuelLaunched = towerStation == 3 && furthestZ > 0.52f && highestY > 0.60f && fastest > 2f;
                    Debug.Log($"GOLDFIRE TEST: Collected Fuel tower stations={towerStation}, furthest forward Z={furthestZ:F3} m, peak Y={highestY:F3} m, peak speed={fastest:F2} m/s, launched={collectedFuelLaunched}.");
                    Object.Destroy(fuel.gameObject);
                    fuel = SpawnFuel(new Vector3(-0.09f, 0.25f, 0.27f), -0.12f);
                    BeginObservation();
                    towerStation = 0;
                    widestHopperRoute = 0f;
                    stage = 10;
                    stageStart = now;
                    break;
                case 10:
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    ObserveFuel();
                    ObserveTowerRoute();
                    var hopperPosition = robot.transform.InverseTransformPoint(fuel.position);
                    if (hopperPosition.z is >= 0.108f and <= 0.202f && hopperPosition.y < 0.32f)
                        widestHopperRoute = Mathf.Max(widestHopperRoute, -hopperPosition.x);
                    if (now - stageStart < 2.4) return;
                    hopperLaunch = towerStation == 3 && widestHopperRoute >= 0.17f && furthestZ > 0.52f && highestY > 0.60f && fastest > 2f;
                    Debug.Log($"GOLDFIRE TEST: Hopper-feed Fuel tower stations={towerStation}, outside spindexer X={widestHopperRoute:F3}m, furthest forward Z={furthestZ:F3}m, peak Y={highestY:F3}m, peak speed={fastest:F2}m/s, final={robot.transform.InverseTransformPoint(fuel.position)}, launched={hopperLaunch}.");
                    CheckMechanicalSpeeds("feed", -1f, false);
                    SetPad(new GamepadState().WithButton(GamepadButton.South).WithButton(GamepadButton.DpadLeft));
                    reverseStarted = Time.fixedTime;
                    Next(now);
                    break;
                case 30:
                    if (BaseGameManager.Instance.RobotState != RobotState.Enabled || now - stageStart < 1.2) return;
                    robot.SetGameplayInputEnabled(true);
                    robot.TranslateAction.actionMap.Enable();
                    InputSystem.EnableDevice(gamepad);
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    Next(now);
                    break;
                case 31:
                    SetPad(new GamepadState());
                    var advancedAim = robot.GetComponent<RebuiltAutoAimController>();
                    var advancedBody = robot.GetComponent<Rigidbody>();
                    bool movingSetup = advancedTrial >= 2;
                    if (movingSetup)
                    {
                        advancedAim.SetDefaultMap(DefaultMap.Hub);
                        advancedTarget = advancedAim.GetCurrentTarget();
                    }
                    else
                    {
                        advancedAim.SetPassMapClosest();
                        advancedTarget = advancedAim.GetCurrentTarget();
                    }

                    float side = Mathf.Sign(advancedTarget.x);
                    Vector3 setupPos = movingSetup
                        ? new Vector3(advancedTarget.x, 0.015f, advancedTarget.z) + AdvancedRanges[advancedTrial] * new Vector3(side, 0f, advancedTrial == 2 ? 0.53f : -0.53f).normalized
                        : FindPassPosition(advancedTarget, AdvancedRanges[advancedTrial]);
                    Vector3 setupHeading = advancedTarget - setupPos;
                    setupHeading.y = 0f;
                    MoveTestRobot(setupPos, Quaternion.LookRotation(setupHeading, Vector3.up));
                    Debug.Log($"GOLDFIRE TEST: Advanced trial {advancedTrial} target={advancedTarget.ToString("F3")} robot={setupPos.ToString("F3")}.");
                    Next(now);
                    break;
                case 32:
                    if (now - stageStart < 0.8) return;
                    var setupBody = robot.GetComponent<Rigidbody>();
                    if (advancedTrial < 2)
                    {
                        var settledAim = robot.GetComponent<RebuiltAutoAimController>();
                        settledAim.SetPassMapClosest();
                        advancedTarget = settledAim.GetCurrentTarget();
                    }
                    var setupFacing = advancedTarget - setupBody.position;
                    setupFacing.y = 0f;
                    if (setupBody.position.y > 0.20f || Mathf.Abs(setupBody.position.x) > 7.8f || Mathf.Abs(setupBody.position.z) > 3.6f)
                        throw new InvalidOperationException($"The test robot did not settle on the carpet at {setupBody.position}.");
                    MoveTestRobot(setupBody.position, Quaternion.LookRotation(setupFacing, Vector3.up));
                    fuel = SpawnFuel(new Vector3(0.06f, 0.272f, 0.099f), -0.12f);
                    advancedLaunched = false;
                    scoredHub = false;
                    ballLanding = default;
                    releaseRobotVelocity = default;
                    releaseVelocity = default;
                    releaseTime = 0f;
                    passLanded = false;
                    maximumHeight = 0f;
                    closestHubMiss = float.PositiveInfinity;
                    previousBallPosition = fuel.position;
                    BeginObservation();
                    Next(now);
                    break;
                case 33:
                    bool moving = advancedTrial >= 2;
                    var state = new GamepadState
                    {
                        rightTrigger = advancedTrial == 0 ? 0f : 1f,
                        leftStick = moving && now - stageStart >= 0.42 ? new Vector2(advancedTrial == 2 ? 0.48f : -0.48f, 0f) : Vector2.zero,
                    };
                    if (advancedTrial == 0)
                        state = state.WithButton(GamepadButton.RightShoulder);
                    SetPad(state);
                    var liveBody = robot.GetComponent<Rigidbody>();
                    var currentPoint = fuel.position;
                    Vector3 relativePoint = robot.transform.InverseTransformPoint(currentPoint);
                    if (moving && now - lastFeedTrace > 0.48)
                    {
                        var movingAim = robot.GetComponent<RebuiltAutoAimController>();
                        var movingKicker = robot.GetComponentsInChildren<InteractionRoller>().First(roller => roller.name == "kicker_lower Physics");
                        var movingFlywheel = robot.GetComponentsInChildren<FlywheelBehaviour>().First();
                        Debug.Log($"GOLDFIRE TEST: Moving trace t={now - stageStart:F2}s robot={liveBody.position.ToString("F2")} v={liveBody.velocity.ToString("F2")} aim={movingAim.GetRotationError():F2} kicker={movingKicker.TargetRpm:F0} flywheel={movingFlywheel.CurrentRpm:F0}/{movingFlywheel.TargetRpm:F0}.");
                        lastFeedTrace = now;
                    }
                    maximumHeight = Mathf.Max(maximumHeight, currentPoint.y);
                    closestHubMiss = Mathf.Min(closestHubMiss, (currentPoint - advancedTarget).magnitude);
                    if (!advancedLaunched && relativePoint.z > 0.10f && relativePoint.y > 0.65f && fuel.velocity.magnitude > 3f)
                    {
                        advancedLaunched = true;
                        releaseTime = Time.fixedTime;
                        releaseRobotVelocity = liveBody.velocity;
                        releaseVelocity = fuel.velocity;
                        releaseLead = (robot.GetComponent<RebuiltAutoAimController>().GetCurrentTarget() - advancedTarget).magnitude;
                        Debug.Log($"GOLDFIRE TEST: Advanced trial {advancedTrial} release world={currentPoint.ToString("F3")} fuel v={releaseVelocity.ToString("F3")} robot v={releaseRobotVelocity.ToString("F3")} lead={releaseLead:F3}m.");
                    }

                    if (advancedLaunched && !passLanded && currentPoint.y < 0.10f && previousBallPosition.y >= 0.10f)
                    {
                        float alpha = Mathf.InverseLerp(previousBallPosition.y, currentPoint.y, 0.10f);
                        ballLanding = Vector3.Lerp(previousBallPosition, currentPoint, alpha);
                        passLanded = true;
                    }
                    previousBallPosition = currentPoint;

                    var advancedPiece = fuel.GetComponentInParent<RebuiltGamePieceController>();
                    scoredHub |= advancedPiece && advancedPiece.SlotIndex >= 0 && advancedPiece.scored;
                    if (now - stageStart < 5.0 && !(moving ? scoredHub : passLanded)) return;
                    var passError = ballLanding - advancedTarget;
                    passError.y = 0f;
                    Vector3 flatRobotVelocity = releaseRobotVelocity;
                    flatRobotVelocity.y = 0f;
                    var command = robot.GetComponentsInChildren<FlywheelBehaviour>().First().TargetRpm;
                    var advancedDistance = Vector2.Distance(new Vector2(advancedTarget.x, advancedTarget.z), new Vector2(liveBody.position.x, liveBody.position.z));
                    Debug.Log($"GOLDFIRE TEST: Advanced trial {advancedTrial} moving={moving} distance={advancedDistance:F2}m RPM={command:F0} launched={advancedLaunched} landed={passLanded} landing={ballLanding.ToString("F3")} landing error={passError.ToString("F3")} ({passError.magnitude:F2}m), peak={maximumHeight:F2}m hub score={scoredHub} closest={closestHubMiss:F2}m, robot release speed={flatRobotVelocity.magnitude:F2}m/s, flight={Time.fixedTime - releaseTime:F2}s.");
                    failedAdvanced |= moving ? !scoredHub || flatRobotVelocity.magnitude < 0.25f : !passLanded || passError.magnitude > 0.75f;
                    if (advancedPiece)
                        advancedPiece.gameObject.SetActive(false);
                    SetPad(new GamepadState());
                    advancedTrial++;
                    if (advancedTrial < AdvancedRanges.Length)
                    {
                        stage = 31;
                        stageStart = now;
                        break;
                    }
                    if (SessionState.GetBool(RunFull, false))
                    {
                        Debug.Log($"GOLDFIRE TEST: Advanced shot tests passed={!failedAdvanced}.");
                        stage = 50;
                        stageStart = now;
                        break;
                    }
                    Complete($"Advanced shot tests passed={!failedAdvanced}.", failedAdvanced || failedHub || failedMechanisms ? 1 : 0);
                    break;
                case 20:
                    if (BaseGameManager.Instance.RobotState != RobotState.Enabled || now - stageStart < 1.2) return;
                    robot.SetGameplayInputEnabled(true);
                    robot.TranslateAction.actionMap.Enable();
                    InputSystem.EnableDevice(gamepad);
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    Next(now);
                    break;
                case 21:
                    var aim = robot.GetComponent<RebuiltAutoAimController>();
                    aim.SetDefaultMap(DefaultMap.Hub);
                    var target = aim.GetCurrentTarget();
                    var ownHub = Object.FindObjectsByType<HubScoring>(FindObjectsSortMode.None)
                        .Single(item => item.Alliance == robot.Alliance);
                    var scoringVolumes = new SerializedObject(ownHub).FindProperty("occupyColliders");
                    var funnel = scoringVolumes.GetArrayElementAtIndex(0).objectReferenceValue as Collider;
                    hubPoint = funnel.bounds.center;
                    hubAimPoint = target;
                    float sideOffset = hubTrial < 2 ? 0f : hubTrial == 2 ? 0.63f : 0.89f;
                    var facing = (new Vector3(Mathf.Sign(target.x), 0f, sideOffset)).normalized;
                    var chassis = robot.GetComponent<Rigidbody>();
                    Vector3 hubSetup = new Vector3(target.x, 0.015f, target.z) + HubRanges[hubTrial] * facing;
                    MoveTestRobot(hubSetup, Quaternion.LookRotation(facing, Vector3.up));
                    Debug.Log($"GOLDFIRE TEST: Hub target={target.ToString("F3")}, scorer={hubPoint.ToString("F3")}, robot={chassis.position.ToString("F3")}, alliance zone={robot.InAllianceZone()}.");
                    Next(now);
                    break;
                case 22:
                    if (now - stageStart < 0.75) return;
                    var settledBody = robot.GetComponent<Rigidbody>();
                    var facingAfterSettling = settledBody.position - robot.GetComponent<RebuiltAutoAimController>().GetCurrentTarget();
                    facingAfterSettling.y = 0f;
                    hubShotDirection = -facingAfterSettling.normalized;
                    var hubHeading = Quaternion.LookRotation(hubTrial == 0 ? facingAfterSettling : -facingAfterSettling, Vector3.up);
                    if (hubTrial == 2)
                        hubHeading = Quaternion.AngleAxis(8f, Vector3.up) * hubHeading;
                    MoveTestRobot(settledBody.position, hubHeading);
                    fuel = SpawnFuel(new Vector3(0.06f, 0.272f, 0.099f), -0.12f);
                    previousBallPosition = fuel.position;
                    crossedHubOpeningHeight = false;
                    hubOpeningCrossing = default;
                    scoredHub = false;
                    intakeFacingHub = false;
                    closestHubMiss = float.PositiveInfinity;
                    maximumHeight = 0f;
                    BeginObservation();
                    Next(now);
                    break;
                case 23:
                    SetPad(new GamepadState { rightTrigger = 1f });
                    ObserveFuel();
                    var targetBearing = hubAimPoint - robot.transform.position;
                    targetBearing.y = 0f;
                    intakeFacingHub |= Vector3.Dot(robot.transform.forward, targetBearing.normalized) > 0.998f;
                    if (now - lastFeedTrace > 0.65)
                    {
                        var liveAim = robot.GetComponent<RebuiltAutoAimController>();
                        var liveKicker = robot.GetComponentsInChildren<InteractionRoller>().First(roller => roller.name == "kicker_lower Physics");
                        Debug.Log($"GOLDFIRE TEST: Hub trace range={HubRanges[hubTrial]:F1}m time={now - stageStart:F2}s pos={robot.transform.InverseTransformPoint(fuel.position).ToString("F3")} speed={fuel.velocity.magnitude:F2} robot={robot.transform.position.ToString("F2")} kicker={liveKicker.TargetRpm:F0} aim={liveAim.GetRotationError():F1}°.");
                        lastFeedTrace = now;
                    }
                    Vector3 hubSample = fuel.position;
                    if (!crossedHubOpeningHeight && previousBallPosition.y >= hubAimPoint.y && hubSample.y < hubAimPoint.y)
                    {
                        crossedHubOpeningHeight = true;
                        float alpha = Mathf.InverseLerp(previousBallPosition.y, hubSample.y, hubAimPoint.y);
                        hubOpeningCrossing = Vector3.Lerp(previousBallPosition, hubSample, alpha);
                    }
                    previousBallPosition = hubSample;
                    var ball = fuel.GetComponentInParent<RebuiltGamePieceController>();
                    scoredHub |= ball && ball.SlotIndex >= 0 && ball.scored;
                    closestHubMiss = Mathf.Min(closestHubMiss, (fuel.position - hubPoint).magnitude);
                    maximumHeight = Mathf.Max(maximumHeight, fuel.position.y);
                    if (now - stageStart < 4.2 && !scoredHub) return;
                    var shotFlywheels = robot.GetComponentsInChildren<FlywheelBehaviour>();
                    Debug.Log($"GOLDFIRE TEST: Hub {HubRanges[hubTrial]:F1}m scored={scoredHub}, peak Y={maximumHeight:F3}m, max speed={fastest:F2}m/s, closest scorer approach={closestHubMiss:F3}m, crossed rim height={crossedHubOpeningHeight}, opening longitudinal error={Vector3.Dot(hubOpeningCrossing - hubAimPoint, hubShotDirection):F2}m, shaft targets={string.Join(", ", shotFlywheels.Select(item => item.TargetRpm.ToString("F0")))}.");
                    Debug.Log($"GOLDFIRE TEST: Hub {HubRanges[hubTrial]:F1}m intake initially faced away={hubTrial == 0}, intake faced hub after auto-align={intakeFacingHub}.");
                    failedHub |= !scoredHub || !intakeFacingHub;
                    fuel.GetComponentInParent<RebuiltGamePieceController>().gameObject.SetActive(false);
                    SetPad(new GamepadState());
                    hubTrial++;
                    if (hubTrial < HubRanges.Length)
                    {
                        stage = 21;
                        stageStart = now;
                        break;
                    }
                    if (SessionState.GetBool(RunFull, false))
                    {
                        Debug.Log($"GOLDFIRE TEST: Hub range scoring passed={!failedHub}.");
                        stage = 31;
                        stageStart = now;
                        break;
                    }
                    Complete($"Hub range scoring passed={!failedHub}.", failedHub ? 1 : 0);
                    break;
                case 11:
                    SetPad(new GamepadState().WithButton(GamepadButton.South).WithButton(GamepadButton.DpadLeft));
                    if (Time.fixedTime - reverseStarted < 0.25f || now - stageStart < 0.30) return;
                    Debug.Log($"GOLDFIRE TEST: Reverse inputs south={gamepad.buttonSouth.isPressed}/{robot.FaceDown.IsPressed()}, d-pad-left={gamepad.dpad.left.isPressed}/{robot.DpadL.IsPressed()} match state={BaseGameManager.Instance.RobotState}.");
                    CheckMechanicalSpeeds("intake", -1f, false);
                    CheckMechanicalSpeeds("feed", 1f, false);
                    SetPad(new GamepadState());
                    var missed = previousVisualRotation.Keys.Where(pivot => !rotatingVisuals.Contains(pivot)).ToArray();
                    if (previousVisualRotation.Count != 30 || missed.Length > 0)
                        throw new InvalidOperationException($"Missing visual axle rotation ({previousVisualRotation.Count} pivots): {string.Join(", ", missed.Select(pivot => pivot.name))}.");
                    Debug.Log("GOLDFIRE TEST: Observed rotation around the local axle of all 30 roller, pulley, and swerve pivots.");
                    fuel.GetComponentInParent<RebuiltGamePieceController>().gameObject.SetActive(false);
                    Next(now);
                    break;
                case 50:
                    if (now - stageStart < 1.0) return;
                    robot.SetGameplayInputEnabled(true);
                    robot.TranslateAction.actionMap.Enable();
                    InputSystem.EnableDevice(gamepad);
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    MoveTestRobot(new Vector3(6.45f, 0.018f, 2.52f), Quaternion.LookRotation(Vector3.left));
                    SetPad(new GamepadState { leftTrigger = 1f });
                    Next(now);
                    break;
                case 51:
                    SetPad(new GamepadState { leftTrigger = 1f });
                    if (now - stageStart < 1.0) return;
                    for (int row = 0; row < 3; row++)
                    {
                        for (int column = -2; column <= 2; column++)
                            SpawnFuel(new Vector3(column * 0.151f, 0.075f, 0.89f + row * 0.154f), 0f);
                        for (int column = -2; column < 2; column++)
                            SpawnFuel(new Vector3((column + 0.5f) * 0.151f, 0.199f, 0.95f + row * 0.154f), 0f);
                    }
                    contactStart = robot.GetComponent<Rigidbody>().position;
                    highestChassis = highestTilt = 0f;
                    foreach (var member in robot.GetComponentsInChildren<Rigidbody>())
                        member.velocity = robot.transform.forward * 3.6f;
                    contactStartTime = Time.fixedTime;
                    Next(now);
                    break;
                case 52:
                    SetPad(new GamepadState { leftTrigger = 1f, leftStick = Vector2.up });
                    robot.GetComponent<DriveController>().OverrideInput(4.6f, 0f, 0f, DriveController.DrivePerspective.RobotRelative);
                    var pileBody = robot.GetComponent<Rigidbody>();
                    highestChassis = Mathf.Max(highestChassis, pileBody.position.y - contactStart.y);
                    highestTilt = Mathf.Max(highestTilt, Vector3.Angle(robot.transform.up, Vector3.up));
                    if (Time.fixedTime - contactStartTime < 0.74f) return;
                    Debug.Log($"GOLDFIRE TEST: Pile run, start 3.6 m/s, travel={Vector3.Dot(pileBody.position - contactStart, Vector3.left):F3}m, lift={highestChassis:F3}m, tilt={highestTilt:F1}deg, forward speed={Vector3.Dot(pileBody.velocity, Vector3.left):F2}m/s.");
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    MoveTestRobot(new Vector3(6.2f, 0.018f, 2.52f), Quaternion.LookRotation(Vector3.left));
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadDown));
                    Next(now);
                    break;
                case 53:
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadDown));
                    if (now - stageStart < 1.0) return;
                    contactStart = robot.GetComponent<Rigidbody>().position;
                    largestChassisSpeed = 0f;
                    Next(now);
                    break;
                case 54:
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    var spinBody = robot.GetComponent<Rigidbody>();
                    largestChassisSpeed = Mathf.Max(largestChassisSpeed, new Vector2(spinBody.velocity.x, spinBody.velocity.z).magnitude);
                    if (now - stageStart < 0.7) return;
                    Debug.Log($"GOLDFIRE TEST: Empty shooter movement={(spinBody.position - contactStart).ToString("F4")}m, peak horizontal speed={largestChassisSpeed:F4}m/s.");
                    fuel = SpawnFuel(new Vector3(0.06f, 0.272f, 0.099f), -0.12f);
                    contactStart = spinBody.position;
                    largestChassisSpeed = 0f;
                    BeginObservation();
                    Next(now);
                    break;
                case 55:
                    SetPad(new GamepadState().WithButton(GamepadButton.LeftShoulder));
                    ObserveFuel();
                    var shotBody = robot.GetComponent<Rigidbody>();
                    largestChassisSpeed = Mathf.Max(largestChassisSpeed, new Vector2(shotBody.velocity.x, shotBody.velocity.z).magnitude);
                    if (now - stageStart < 1.65) return;
                    Debug.Log($"GOLDFIRE TEST: Loaded shooter movement={(shotBody.position - contactStart).ToString("F4")}m, peak horizontal speed={largestChassisSpeed:F4}m/s, launched={furthestZ > 0.52f && highestY > 0.6f}.");
                    fuel.GetComponentInParent<RebuiltGamePieceController>().gameObject.SetActive(false);
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                    Next(now);
                    break;
                case 56:
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                    if (now - stageStart < 1.0) return;
                    var climbStage = robot.transform.Find("Climber Slider");
                    extendedHook = climbStage.localPosition.y;
                    var crossbar = carriage.Find("Hopper Hex Crossbar").GetComponent<CapsuleCollider>();
                    var frontPane = carriage.Find("Intake Front Lexan").GetComponent<BoxCollider>();
                    fieldContactVerified = (frontPane.includeLayers.value & (1 << 3)) != 0
                        && (crossbar.includeLayers.value & (1 << 12)) != 0
                        && crossbar.Raycast(new Ray(crossbar.transform.position + robot.transform.up * 0.10f, -robot.transform.up), out _, 0.2f);
                    Debug.Log($"GOLDFIRE TEST: Climber extended {extendedHook:F3}m, crossbar and field layer validated={fieldContactVerified}.");
                    {
                        var climbJoint = climbStage.GetComponent<ConfigurableJoint>();
                        var climbBody = climbStage.GetComponent<Rigidbody>();
                        var extendedFlag = typeof(AwtybotsV1).GetField("climberExtended", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)?.GetValue(robot);
                        Debug.Log($"GOLDFIRE CLIMB DIAG: extendedFlag={extendedFlag}, dpadUp={robot.DpadUp.IsPressed()}, targetVelocity={climbJoint.targetVelocity.ToString("F3")}, " +
                            $"bodyVelocity={climbBody.velocity.ToString("F3")}, kinematic={climbBody.isKinematic}, sleeping={climbBody.IsSleeping()}, yDrive damper={climbJoint.yDrive.positionDamper} max={climbJoint.yDrive.maximumForce}, " +
                            $"local={climbStage.localPosition.ToString("F3")}, axis={climbJoint.axis}, secondary={climbJoint.secondaryAxis}, state={BaseGameManager.Instance.RobotState}.");
                    }
                    foreach (var field in Object.FindObjectsByType<Collider>(FindObjectsSortMode.None).Where(candidate => candidate.enabled &&
                                 !candidate.isTrigger && candidate.gameObject.layer == 3 && candidate.bounds.size.y is > 0.025f and < 0.16f &&
                                 candidate.bounds.center.y is > 0.53f and < 0.84f && Mathf.Max(candidate.bounds.size.x, candidate.bounds.size.z) >= 0.67f))
                        Debug.Log($"GOLDFIRE FIELD: {field.name} parent={field.transform.parent.name}, type={field.GetType().Name}, center={field.bounds.center.ToString("F3")}, size={field.bounds.size.ToString("F3")}.");
                    SetPad(new GamepadState());
                    Next(now);
                    break;
                case 57:
                    if (now - stageStart < 0.14) return;
                    var support = Object.FindObjectsByType<CapsuleCollider>(FindObjectsSortMode.None)
                        .Where(candidate => candidate.enabled && !candidate.isTrigger && candidate.gameObject.layer == 3 &&
                            candidate.bounds.center.y is > 0.55f and < 0.79f &&
                            Mathf.Max(candidate.bounds.size.x, candidate.bounds.size.z) > 0.9f)
                        .OrderBy(candidate => (candidate.bounds.center - robot.transform.position).sqrMagnitude).FirstOrDefault();
                    if (!support)
                    {
                        Complete("Climber travel and crossbar checked. A tower rung was not found in the field.", 1);
                        return;
                    }
                    var bar = support.bounds.center;
                    Vector3 axis = support.bounds.size.x > support.bounds.size.z ? Vector3.right : Vector3.forward;
                    Vector3 front = Vector3.Cross(axis, Vector3.up);
                    // Goldfire's 34.7 in bumpers do not fit between the tower uprights, and its hook is on the
                    // left frame rail. Hook the rung end that overhangs an upright with the left bumper against it.
                    float cylinderHalf = Mathf.Max(support.bounds.size.x, support.bounds.size.z) / 2f - support.radius;
                    var setup = new[] { 1f, -1f }.SelectMany(end => new[] { front, -front }.Select(direction => (end, direction)))
                        .Where(option => Vector3.Dot(Quaternion.LookRotation(option.direction) * Vector3.left, axis * option.end) < -0.5f)
                        .SelectMany(option => Enumerable.Range(0, 13).Select(step =>
                        {
                            var rotation = Quaternion.LookRotation(option.direction);
                            float inset = 0.06f - step * 0.005f;
                            var hook = new Vector3(bar.x, 0.012f, bar.z) + axis * option.end * (cylinderHalf - inset);
                            var position = hook - rotation * new Vector3(-0.324f, 0f, -0.096f);
                            int obstructions = Physics.OverlapBox(position + Vector3.up * 0.30f, new Vector3(0.445f, 0.25f, 0.445f),
                                rotation, 1 << 3, QueryTriggerInteraction.Ignore).Count(hit => hit.name != "Carpet");
                            return (position, rotation, obstructions, inset);
                        }))
                        .OrderBy(option => option.obstructions).ThenByDescending(option => option.inset).First();
                    Debug.Log($"GOLDFIRE CLIMB DIAG: hook inset from rung end={setup.inset:F3}m.");
                    MoveTestRobot(setup.position, setup.rotation);
                    contactStart = setup.position;
                    Debug.Log($"GOLDFIRE TEST: Hook over rung {support.name} at {bar.ToString("F3")}, chassis {setup.position.ToString("F3")}, footprint obstructions={setup.obstructions}.");
                    climbRung = support;
                    foreach (var blocker in Physics.OverlapBox(setup.position + Vector3.up * 0.30f, new Vector3(0.445f, 0.25f, 0.445f),
                                 setup.rotation, 1 << 3, QueryTriggerInteraction.Ignore))
                        Debug.Log($"GOLDFIRE CLIMB DIAG: footprint blocker {blocker.name} parent={blocker.transform.parent?.name} type={blocker.GetType().Name} bounds={blocker.bounds.center.ToString("F3")} size={blocker.bounds.size.ToString("F3")}.");
                    foreach (var near in Physics.OverlapSphere(bar, 0.9f, 1 << 3, QueryTriggerInteraction.Ignore))
                        Debug.Log($"GOLDFIRE CLIMB DIAG: near rung {near.name} parent={near.transform.parent?.name} type={near.GetType().Name} bounds={near.bounds.center.ToString("F3")} size={near.bounds.size.ToString("F3")}.");
                    Next(now);
                    break;
                case 58:
                    if (now - stageStart < 0.19) return;
                    LogRungInHook("before retract");
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                    Next(now);
                    break;
                case 59:
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                    if (now - lastFeedTrace > 0.12)
                    {
                        lastFeedTrace = now;
                        LogRungInHook($"retracting t={now - stageStart:F2}");
                    }
                    if (now - stageStart < 1.0) return;
                    float raised = robot.GetComponent<Rigidbody>().position.y - contactStart.y;
                    bool hanging = raised > 0.045f && !robot.GetComponent<DriveController>().IsTouchingGround;
                    testedClimb = hanging;
                    var latch = robot.GetComponentInChildren<FixedJoint>();
                    var slide = robot.transform.Find("Climber Slider").localPosition.y;
                    Debug.Log($"GOLDFIRE TEST: Climber on physical rung lift={raised:F3}m, slide={slide:F3}m, latched={latch != null}, hanging={hanging}, chassis={robot.transform.position.ToString("F3")}.");
                    SetPad(new GamepadState());
                    if (SessionState.GetBool(RunClimb, false))
                    {
                        Complete($"Climb only: hanging={hanging}.", hanging ? 0 : 1);
                        return;
                    }
                    Next(now);
                    break;
                case 60:
                    if (now - stageStart < 0.12) return;
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp));
                    Next(now);
                    break;
                case 61:
                    if (now - stageStart < 0.24) return;
                    var overhead = Object.FindObjectsByType<BoxCollider>(FindObjectsSortMode.None)
                        .First(collider => collider.name == "Overhead" && collider.bounds.center.x > 0f && collider.bounds.center.z > 0f);
                    trenchStart = new Vector3(overhead.bounds.center.x + 1.02f, 0.018f, overhead.bounds.center.z);
                    MoveTestRobot(trenchStart, Quaternion.LookRotation(Vector3.left));
                    SetPad(new GamepadState());
                    Next(now);
                    break;
                case 62:
                    if (now - stageStart < 0.16) return;
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadUp).WithButton(GamepadButton.DpadDown));
                    Next(now);
                    break;
                case 63:
                    if (now - stageStart < 0.80) return;
                    SetPad(new GamepadState());
                    contactStartTime = Time.fixedTime;
                    Next(now);
                    break;
                case 64:
                    robot.GetComponent<DriveController>().OverrideInput(2.2f, 0f, 0f, DriveController.DrivePerspective.RobotRelative);
                    if (Time.fixedTime - contactStartTime < 1.15f) return;
                    float stowedX = robot.GetComponent<Rigidbody>().position.x;
                    stowedTrenchBlocked = stowedX > trenchStart.x - 0.95f;
                    Debug.Log($"GOLDFIRE TEST: Stock trench, stowed end X={stowedX:F3}m, blocked={stowedTrenchBlocked}.");
                    LogTrenchContacts("stowed");
                    MoveTestRobot(trenchStart, Quaternion.LookRotation(Vector3.left));
                    SetPad(new GamepadState { leftTrigger = 1f });
                    Next(now);
                    break;
                case 65:
                    SetPad(new GamepadState { leftTrigger = 1f });
                    if (now - stageStart < 0.80) return;
                    contactStartTime = Time.fixedTime;
                    Next(now);
                    break;
                case 66:
                    robot.GetComponent<DriveController>().OverrideInput(2.2f, 0f, 0f, DriveController.DrivePerspective.RobotRelative);
                    if (Time.fixedTime - contactStartTime < 1.15f) return;
                    float deployedX = robot.GetComponent<Rigidbody>().position.x;
                    deployedTrenchCleared = deployedX < trenchStart.x - 1.35f;
                    Debug.Log($"GOLDFIRE TEST: Stock trench, deployed end X={deployedX:F3}m, cleared={deployedTrenchCleared}.");
                    LogTrenchContacts("deployed");
                    MoveTestRobot(new Vector3(6.95f, 0.018f, 2.46f), Quaternion.LookRotation(Vector3.left));
                    SetPad(new GamepadState().WithButton(GamepadButton.DpadDown));
                    Next(now);
                    break;
                case 67:
                    if (now - stageStart < 0.62) return;
                    SetPad(new GamepadState());
                    contactStartTime = Time.fixedTime;
                    largestChassisSpeed = 0f;
                    Next(now);
                    break;
                case 68:
                    var benchmark = robot.GetComponent<DriveController>();
                    benchmark.OverrideInput(benchmark.GetMaxVelocity(), 0f, 0f, DriveController.DrivePerspective.RobotRelative);
                    var straightBody = robot.GetComponent<Rigidbody>();
                    largestChassisSpeed = Mathf.Max(largestChassisSpeed, Vector3.Dot(Vector3.left, straightBody.velocity));
                    if (Time.fixedTime - contactStartTime < 0.64f) return;
                    contactStart = straightBody.position;
                    contactStartTime = Time.fixedTime;
                    Next(now);
                    break;
                case 69:
                    if (Time.fixedTime - contactStartTime < 0.63f) return;
                    var stopped = robot.GetComponent<Rigidbody>();
                    float stoppingDistance = Vector3.Dot(stopped.position - contactStart, Vector3.left);
                    // Four Vortex drive motors at the V1 50 A limit give about 0.79 N·m each after free current.
                    // Through 5.08:1 at 93% on a 1.5 in wheel that is about 390 N on a 61 kg robot,
                    // or about 6.2 m/s². V1's 0.25 s open-loop ramp then predicts about 3.2 m/s at 0.64 s.
                    bool drivePassed = largestChassisSpeed > 2.8f && largestChassisSpeed < 3.6f && stoppingDistance < 1.5f;
                    Debug.Log($"GOLDFIRE TEST: MAXSwerve carpet launch 0.64s peak={largestChassisSpeed:F2}m/s, release traveled {stoppingDistance:F2}m in 0.63s, final speed={stopped.velocity.magnitude:F2}m/s.");
                    bool contactsPassed = fieldContactVerified && extendedHook > 0.17f && testedClimb && stowedTrenchBlocked && deployedTrenchCleared && drivePassed;
                    Complete($"Contacts and driving passed={contactsPassed}, climb={testedClimb}, trench poses={stowedTrenchBlocked}/{deployedTrenchCleared}, hopper={(!failedMechanisms)}, hub={(!failedHub)}, passing and moving shots={(!failedAdvanced)}.",
                        contactsPassed && !failedAdvanced && !failedHub && !failedMechanisms ? 0 : 1);
                    break;
                case 12:
                    var trial = ContainmentTrials[containmentTrial];
                    fuel = SpawnFuel(trial.Position, 0f);
                    fuel.velocity = robot.transform.TransformDirection(trial.Velocity);
                    containmentStarted = Time.fixedTime;
                    Next(now);
                    break;
                case 13:
                    if (Time.fixedTime - containmentStarted < 0.16f) return;
                    var wall = ContainmentTrials[containmentTrial];
                    var final = robot.transform.InverseTransformPoint(fuel.position);
                    float extent = Vector3.Dot(final, wall.Velocity.normalized);
                    bool crossed = extent > wall.CrossingLimit;
                    bool passed = containmentTrial == ContainmentTrials.Length - 1 ? crossed : !crossed;
                    passedContainment &= passed;
                    Debug.Log($"GOLDFIRE TEST: Containment {wall.Name} passed={passed}, final={final.ToString("F3")}.");
                    fuel.GetComponentInParent<RebuiltGamePieceController>().gameObject.SetActive(false);
                    if (++containmentTrial < ContainmentTrials.Length)
                    {
                        stage = 12;
                        stageStart = now;
                        break;
                    }
                    SetPad(new GamepadState { leftTrigger = 1f });
                    Next(now);
                    break;
                case 14:
                    SetPad(new GamepadState { leftTrigger = 1f });
                    if (now - stageStart < 0.5) return;
                    intakeHoldPosition = robot.GetComponent<Rigidbody>().position;
                    foreach (float x in new[] { -0.20f, 0f, 0.20f })
                    {
                        foreach (float z in new[] { 0.08f, 0.25f, 0.42f })
                            SpawnFuel(new Vector3(x, 0.225f, z), 0f);
                        SpawnFuel(new Vector3(x, 0.075f, 0.73f), -0.65f);
                    }
                    Next(now);
                    break;
                case 15:
                    SetPad(new GamepadState { leftTrigger = 1f });
                    foreach (var body in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                    {
                        var local = robot.transform.InverseTransformPoint(body.transform.position);
                        if (Mathf.Abs(local.x) < 0.19f && Mathf.Abs(local.z) < 0.25f && local.y < 0.17f)
                            fuelStaysAboveFloor = false;
                    }
                    if (now - stageStart < 1.9) return;
                    var loadedBody = robot.GetComponent<Rigidbody>();
                    var loadedDrift = loadedBody.position - intakeHoldPosition;
                    loadedStable = new Vector2(loadedDrift.x, loadedDrift.z).magnitude < 0.08f
                        && new Vector2(loadedBody.velocity.x, loadedBody.velocity.z).magnitude < 0.045f;
                    Debug.Log($"GOLDFIRE TEST: Loaded neutral intake displacement={loadedDrift.ToString("F4")}, velocity={loadedBody.velocity.ToString("F4")}, stable={loadedStable}, Fuel stayed above the plate={fuelStaysAboveFloor}.");
                    SetPad(new GamepadState());
                    foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                        piece.gameObject.SetActive(false);
                    Next(now);
                    break;
                case 16:
                    var floor = robot.transform.Find("Bottom Lexan Collision");
                    var mainPlate = floor.Find("Main Bottom Lexan").GetComponentsInChildren<BoxCollider>();
                    var lowerPlate = floor.Find("Lower Chassis Lexan").GetComponentsInChildren<BoxCollider>();
                    var batteryPlate = floor.Find("Lower Battery Lexan").GetComponentsInChildren<BoxCollider>();
                    var solidMain = FloorDropSites.All(site => HitsPlate(mainPlate, new Vector3(site.x, 0.60f, site.y), 0.13f)
                        || HitsPlate(batteryPlate, new Vector3(site.x, 0.60f, site.y), 0.075f));
                    var realNotches = !HitsPlate(mainPlate, new Vector3(0.30f, 0.60f, 0.30f), 0.13f)
                        && !HitsPlate(lowerPlate, new Vector3(-0.30f, 0.60f, 0.30f), 0.075f);
                    fuelStaysAboveFloor &= solidMain && realNotches
                        && HitsPlate(lowerPlate, new Vector3(0f, 0.60f, 0.26f), 0.075f)
                        && HitsPlate(batteryPlate, new Vector3(0.27f, 0.60f, -0.13f), 0.075f);
                    coveredFloorColliders = robot.GetComponentsInChildren<Collider>()
                        .Where(collider => collider.enabled && !collider.transform.IsChildOf(floor)
                            && ((collider.includeLayers.value & (1 << 12)) != 0 || collider.name == "Wheel_InternalCore")).ToArray();
                    foreach (var covered in coveredFloorColliders)
                        covered.enabled = false;
                    floorDropFuel = FloorDropSites.Select(site =>
                    {
                        var drop = SpawnFuel(new Vector3(site.x, 0.68f, site.y), 0f);
                        drop.velocity = -robot.transform.up * 5f;
                        return drop;
                    }).ToArray();
                    lowestDropHeight = Enumerable.Repeat(float.PositiveInfinity, floorDropFuel.Length).ToArray();
                    Next(now);
                    break;
                case 17:
                    for (int index = 0; index < floorDropFuel.Length; index++)
                    {
                        Vector3 local = robot.transform.InverseTransformPoint(floorDropFuel[index].position);
                        if (Mathf.Abs(local.x) <= 0.28f && Mathf.Abs(local.z) <= 0.30f)
                            lowestDropHeight[index] = Mathf.Min(lowestDropHeight[index], local.y);
                    }
                    if (now - stageStart < 0.7) return;
                    fuelStaysAboveFloor &= lowestDropHeight.All(height => height >= 0.135f && height < 0.26f);
                    foreach (var covered in coveredFloorColliders)
                        covered.enabled = true;
                    Debug.Log($"GOLDFIRE TEST: Six full-speed Lexan floor drops lowest center heights={string.Join(", ", lowestDropHeight.Select(height => height.ToString("F3")))}, plate coverage passed={fuelStaysAboveFloor}.");
                    var success = pickedUp && collectedFuelLaunched && hopperLaunch && passedContainment && neutralIntakeStable && loadedStable && fuelStaysAboveFloor;
                    string mechanismResults = $"Floor intake={pickedUp}; pickup-to-forward-shot={collectedFuelLaunched}; hopper-to-forward-shot={hopperLaunch}; Lexan containment and open top={passedContainment}; floor retained Fuel={fuelStaysAboveFloor}; neutral intake stable={neutralIntakeStable && loadedStable}.";
                    if (SessionState.GetBool(RunFull, false))
                    {
                        failedMechanisms = !success;
                        Debug.Log($"GOLDFIRE TEST: {mechanismResults}");
                        foreach (var piece in Object.FindObjectsByType<RebuiltGamePieceController>(FindObjectsSortMode.None))
                            piece.gameObject.SetActive(false);
                        SetPad(new GamepadState());
                        stage = 21;
                        stageStart = now;
                        break;
                    }
                    Complete(mechanismResults, success ? 0 : 1);
                    break;
            }
        }
        catch (Exception exception)
        {
            Complete(exception.ToString(), 1);
        }
    }

    private static bool HitsPlate(IEnumerable<BoxCollider> colliders, Vector3 point, float ceiling)
    {
        var ray = new Ray(robot.transform.TransformPoint(point), -robot.transform.up);
        return colliders.Any(collider => collider.Raycast(ray, out var hit, 0.6f)
            && robot.transform.InverseTransformPoint(hit.point).y <= ceiling);
    }

    private static Vector3 FindPassPosition(Vector3 target, float range)
    {
        Vector3 towardNeutral = -Mathf.Sign(target.x) * Vector3.right;
        foreach (float angle in new[] { -35f, 35f, -20f, 20f, -45f, 45f, 0f, -55f, 55f })
        {
            Vector3 facing = Quaternion.AngleAxis(angle, Vector3.up) * towardNeutral;
            Vector3 point = new Vector3(target.x, 0.015f, target.z) + facing * range;
            if (Mathf.Abs(point.x) > 7.6f || Mathf.Abs(point.z) > 3.50f || Mathf.Sign(point.z) != Mathf.Sign(target.z))
                continue;
            var obstacles = Physics.OverlapBox(point + Vector3.up * 0.37f, new Vector3(0.44f, 0.29f, 0.44f),
                Quaternion.LookRotation(facing, Vector3.up), ~0, QueryTriggerInteraction.Ignore);
            var field = obstacles.Where(item => !item.transform.IsChildOf(robot.transform)).ToArray();
            if (field.Length == 0)
                return point;
            Debug.Log($"GOLDFIRE TEST: Skipping occupied pass setup {point}. Contacts={string.Join(", ", field.Select(item => item.name))}.");
        }
        throw new InvalidOperationException($"Could not find a clear pass setup {range:F1}m from {target}.");
    }

    private static void MoveTestRobot(Vector3 position, Quaternion rotation)
    {
        var chassis = robot.GetComponent<Rigidbody>();
        var members = robot.GetComponentsInChildren<Rigidbody>();
        Vector3 oldPosition = chassis.position;
        Quaternion delta = rotation * Quaternion.Inverse(chassis.rotation);
        var positions = members.Select(member => position + delta * (member.position - oldPosition)).ToArray();
        var rotations = members.Select(member => delta * member.rotation).ToArray();
        for (int i = 0; i < members.Length; i++)
        {
            members[i].position = positions[i];
            members[i].rotation = rotations[i];
            members[i].velocity = Vector3.zero;
            members[i].angularVelocity = Vector3.zero;
        }
        Physics.SyncTransforms();
    }
    private static void ObserveVisualMotion()
    {
        // Read pose changes after MoSim's runtime Update rather than assuming
        // the visible geometry moved when a shaft accepted an RPM command.
        foreach (var pivot in previousVisualRotation.Keys.ToArray())
        {
            var previous = previousVisualRotation[pivot];
            var current = pivot.localRotation;
            previousVisualRotation[pivot] = current;
            if (Quaternion.Angle(previous, current) < 1f)
                continue;
            var relative = Quaternion.Inverse(previous) * current;
            if (Mathf.Abs(relative.y) > 0.004f || Mathf.Abs(relative.z) > 0.004f)
                throw new InvalidOperationException($"{pivot.name} rotated away from its axle.");
            rotatingVisuals.Add(pivot);
        }
    }

    private static void CheckMechanicalSpeeds(string mechanism, float sign, bool shot)
    {
        var rollers = robot.GetComponentsInChildren<InteractionRoller>();
        var names = mechanism switch
        {
            "intake" => new[] { "intake_roller_1_slide", "intake_roller_2_slide" },
            "feed" => new[] { "kicker_lower", "kicker_powered", "kicker_top" },
            _ => new[] { "backspin_top_left", "backspin_bottom_left", "backspin_top_right", "backspin_bottom_right" },
        };
        float expectedRpm = mechanism == "intake" ? IntakeShaftRpm : VortexThrough3To1;
        foreach (var name in names)
        {
            var roller = rollers.Single(item => item.name == name + " Physics");
            float expected = shot ? 1735f : expectedRpm * sign;
            if (Mathf.Abs(roller.TargetRpm - expected) > 2f)
                throw new InvalidOperationException($"{name} was commanded {roller.TargetRpm:F0} RPM; expected {expected:F0} RPM.");
        }
        if (mechanism == "feed")
        {
            foreach (var roller in rollers.Where(item => item.name.StartsWith("indexer_")))
            {
                float x = robot.transform.InverseTransformPoint(roller.transform.position).x;
                float expected = VortexThrough3To1 * Mathf.Sign(x) * sign;
                if (Mathf.Abs(roller.TargetRpm - expected) > 2f)
                    throw new InvalidOperationException($"{roller.name} was commanded {roller.TargetRpm:F0} RPM; expected {expected:F0} RPM.");
            }
        }
        foreach (var roller in rollers)
        {
            Transform pivot = roller.transform.Find(roller.name.Replace(" Physics", " Visual Pivot"));
            if (!pivot || Vector3.Dot(pivot.right, roller.transform.right) < 0.999f)
                throw new InvalidOperationException($"{roller.name} has a visual pivot that points away from its physics axle.");
        }
        Debug.Log($"GOLDFIRE TEST: {mechanism} shaft speeds and signs passed ({sign:F0}).");
    }

    private static Rigidbody SpawnFuel(Vector3 localPoint, float forwardSpeed)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>("Assets/Prefabs/Games/Rebuilt/Fuel.prefab");
        var world = GameObject.FindGameObjectWithTag("GamePieceWorld").transform;
        var instance = Object.Instantiate(prefab, robot.transform.TransformPoint(localPoint), Quaternion.identity, world);
        instance.name = "Goldfire single Fuel test";
        var piece = instance.GetComponentInChildren<Rigidbody>();
        piece.velocity = robot.transform.forward * forwardSpeed;
        return piece;
    }

    private static void ObserveIntakeRoute()
    {
        var p = robot.transform.InverseTransformPoint(fuel.position);
        var axle = carriage.GetComponentsInChildren<InteractionRoller>()
            .Single(roller => roller.name == "intake_roller_1_slide Physics");
        float axleZ = robot.transform.InverseTransformPoint(axle.transform.position).z;
        if (intakeTraceFrame++ % 24 == 0 && intakeTraceFrame <= 360)
        {
            var contact = robot.transform.Find("Intake Fuel Contact").GetComponent<AwtybotsV1IntakeContact>();
            var bumper = new SerializedObject(contact).FindProperty("frontBumper").objectReferenceValue as BoxCollider;
            var nearby = Physics.OverlapBox(contact.transform.position, new Vector3(0.29f, 0.18f, 0.25f),
                contact.transform.rotation, 1 << 12, QueryTriggerInteraction.Ignore);
            var touching = Physics.OverlapSphere(fuel.worldCenterOfMass, 0.076f, ~0, QueryTriggerInteraction.Ignore)
                .Where(c => c.attachedRigidbody != fuel).Select(c => c.name).Distinct();
            Debug.Log($"GOLDFIRE INTAKE TRACE: Fuel={p.ToString("F3")} axleZ={axleZ:F3} bumper={robot.transform.InverseTransformPoint(bumper.transform.position).ToString("F3")} rpm={axle.TargetRpm:F0} contacts={nearby.Count(c => c.attachedRigidbody == fuel)} touch={string.Join(",", touching)}.");
        }
        void Record(float station, ref float height)
        {
            if (float.IsNaN(height) && lastIntakePosition.z > station && p.z <= station)
            {
                float amount = Mathf.InverseLerp(lastIntakePosition.z, p.z, station);
                height = Mathf.Lerp(lastIntakePosition.y, p.y, amount);
            }
        }
        Record(axleZ, ref intakeAxleHeight);
        Record(0.4403f, ref bumperPassHeight);
        Record(0.315f, ref hopperEntryHeight);
        lastIntakePosition = p;
    }

    private static void ObserveTowerRoute()
    {
        var p = robot.transform.InverseTransformPoint(fuel.position);
        bool atNextStation = towerStation switch
        {
            0 => p.z is <= -0.055f and >= -0.16f && p.y is >= 0.16f and <= 0.25f,
            1 => p.y is >= 0.31f and <= 0.39f && p.z is <= -0.15f and >= -0.245f,
            2 => p.y is >= 0.45f and <= 0.55f && p.z is <= -0.15f and >= -0.245f,
            _ => false,
        };
        if (atNextStation)
            towerStation++;
    }

    private static void BeginObservation()
    {
        nearestZ = float.PositiveInfinity;
        furthestZ = float.NegativeInfinity;
        highestY = float.NegativeInfinity;
        fastest = 0f;
    }

    private static void ObserveFuel()
    {
        var relative = robot.transform.InverseTransformPoint(fuel.position);
        nearestZ = Mathf.Min(nearestZ, relative.z);
        furthestZ = Mathf.Max(furthestZ, relative.z);
        highestY = Mathf.Max(highestY, relative.y);
        fastest = Mathf.Max(fastest, fuel.velocity.magnitude);
    }

    private static void Next(double now)
    {
        stage++;
        stageStart = now;
    }

    private static void SetPad(GamepadState state)
    {
        // Let the player loop process the event, as it does for a real controller.
        // A manual InputSystem.Update() here can land between the robot's Update and
        // FixedUpdate and hide button presses from MoSimInput.WasTriggered().
        InputSystem.QueueStateEvent(gamepad, state);
    }

    private static void Complete(string outcome, int exit)
    {
        SessionState.SetBool(Query, false);
        SessionState.SetBool(RunHub, false);
        SessionState.SetBool(RunAdvanced, false);
        SessionState.SetBool(RunClimb, false);
        SessionState.SetBool(RunContactDiagnostics, false);
        Debug.Log($"GOLDFIRE TEST RESULT: {outcome}");
        EditorApplication.Exit(exit);
    }
}

