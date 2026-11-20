using Games.Rebuilt.Components;
using RobotFramework.Components;
using RobotFramework.Controllers.PidSystems;
using RobotFramework.Enums;
using UnityEngine;

public sealed class AwtybotsV3 : AwtybotsRebuiltRobot
{
    [Header("Intake pivot")]
    [SerializeField] private GenericJoint intakePivot;
    [SerializeField] private JointAxis intakeAxis = JointAxis.X;
    [SerializeField, Tooltip("Driven pivot angle in degrees relative to the stowed prefab pose.")]
    private float deployedIntakeAngle;
    [SerializeField] private PidConstants intakePid = new PidConstants(0.25f, 0f, 0f, 2.25f, 1f);

    // Seed speeds from the WPILib shot tables. Roller directions are configured on the prefab.
    [SerializeField] private RebuiltInterpolation hubShots = new RebuiltInterpolation(null, new[]
    {
        CurvePoint(2f, 1720f), CurvePoint(2.5f, 1955f), CurvePoint(3f, 2030f), CurvePoint(3.5f, 2160f),
        CurvePoint(4f, 2250f), CurvePoint(5.2048f, 2477f), CurvePoint(6f, 3000f)
    });

    [SerializeField] private RebuiltInterpolation passShots = new RebuiltInterpolation(null, new[]
    {
        CurvePoint(2f, 1940f), CurvePoint(2.5f, 2175f), CurvePoint(3f, 2250f), CurvePoint(3.5f, 2370f),
        CurvePoint(4f, 2500f), CurvePoint(5.2048f, 2737f), CurvePoint(6f, 3280f), CurvePoint(7f, 2800f),
        CurvePoint(8f, 2800f), CurvePoint(9f, 3000f), CurvePoint(10f, 3300f), CurvePoint(11f, 3750f),
        CurvePoint(12f, 4000f), CurvePoint(13f, 4000f), CurvePoint(14f, 4000f), CurvePoint(15f, 4000f)
    });

    private bool intakeDeployed;

    protected override void Init()
    {
        base.Init();
        if (intakePivot) intakePivot.SetPid(intakePid);
        Scheduler.AddSubsystems(UpdateMechanisms);
        Scheduler.AddLateRecursive(UpdatePids);
    }

    protected override void LateInit()
    {
        base.LateInit();
        InitializeShotCurves(hubShots, passShots);
    }

    private void UpdatePids()
    {
        if (intakePivot) intakePivot.UpdatePid(intakePid);
    }

    private void UpdateMechanisms()
    {
        bool stowing = StowAction;
        bool collecting = IntakeAction && !stowing;
        if (stowing)
        {
            intakeDeployed = false;
        }
        else if (collecting)
        {
            intakeDeployed = true;
        }

        if (intakePivot)
        {
            intakePivot.SetTargetAngle(intakeDeployed ? deployedIntakeAngle : 0f).withAxis(intakeAxis);
        }

        bool feed = UpdateShooter();
        SetFeeder(feed ? 1f : 0f);
        SetIntakeRollers(collecting || (feed && !stowing) ? 1f : 0f);
    }

    private bool UpdateShooter()
    {
        if (!CanAimShot || (!OuttakeAction && !AutoPass))
        {
            SetFlywheel(0f);
            return false;
        }

        bool passing = AutoPass || !InAllianceZone();
        Vector3 target = SelectAimTarget(passing);
        Aim.Aim();

        float distance = HorizontalDistance(target - transform.position);
        var shots = passing ? passShots : hubShots;
        bool atSpeed = SetFlywheel(shots.Interpolate(distance));

        float tolerance = passing ? 3f : HubAimTolerance(distance);
        return CanFeed && atSpeed && Mathf.Abs(Aim.GetRotationError()) <= tolerance;
    }

    private static float HubAimTolerance(float distance)
    {
        if (distance < 2f)
        {
            return 5f;
        }
        return distance < 3.5f ? 2f : 1f;
    }
}
