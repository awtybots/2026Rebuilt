using Games.Rebuilt.Components;
using Games.Rebuilt.Robots;
using MoSimCore.BaseClasses.GameManagement;
using MoSimCore.Enums;
using RobotFramework.Components;
using UnityEngine;

public abstract class AwtybotsRebuiltRobot : RebuiltRobotBase
{
    [Header("Intake rollers")]
    [SerializeField] private RollerGroup intakeRollers;
    [SerializeField] private float intakeRpm;

    [Header("Fuel feed")]
    [SerializeField] private RollerGroup hopperRollers;
    [SerializeField] private float hopperRpm;
    [SerializeField] private RollerGroup kickerRollers;
    [SerializeField] private float kickerRpm;

    [Header("Flywheel")]
    [SerializeField] private RollerGroup flywheelRollers;
    [SerializeField, Min(0f)] private float flywheelRpmScale = 1f;

    protected RebuiltAutoAimController Aim { get; private set; }
    protected bool CanAimShot => Aim && flywheelRollers && flywheelRpmScale > 0f;
    protected bool CanFeed => hopperRollers && kickerRollers && hopperRpm != 0f && kickerRpm != 0f;

    protected override void Init()
    {
        base.Init();

        Aim = GetComponent<RebuiltAutoAimController>();
        Scheduler.AddProtectedSystem(StopWhileDisabled);
    }

    protected void InitializeShotCurves(params RebuiltInterpolation[] curves)
    {
        if (!Aim || !flywheelRollers)
        {
            return;
        }

        foreach (var curve in curves)
        {
            curve.Initialize(this);
        }
    }

    protected Vector3 SelectAimTarget(bool passing)
    {
        if (passing)
        {
            Aim.SetPassMapClosest();
        }
        else
        {
            Aim.SetDefaultMap(DefaultMap.Hub);
        }
        return Aim.GetCurrentTarget();
    }

    protected bool SetFlywheel(float rpm)
    {
        float targetRpm = rpm * flywheelRpmScale;
        if (!flywheelRollers || targetRpm <= 0f)
        {
            flywheelRollers?.Stop();
            return false;
        }

        flywheelRollers.SetRpm(targetRpm);
        return flywheelRollers.AtSpeed();
    }

    protected void SetIntakeRollers(float direction)
    {
        intakeRollers?.SetRpm(direction * intakeRpm);
    }

    protected void SetFeeder(float direction)
    {
        hopperRollers?.SetRpm(direction * hopperRpm);
        kickerRollers?.SetRpm(direction * kickerRpm);
    }

    private void StopWhileDisabled()
    {
        if (BaseGameManager.Instance.RobotState == RobotState.Enabled)
        {
            return;
        }

        intakeRollers?.Stop();
        hopperRollers?.Stop();
        kickerRollers?.Stop();
        flywheelRollers?.Stop();
        ResetWhileDisabled();
    }

    protected virtual void ResetWhileDisabled() { }

    protected static float HorizontalDistance(Vector3 offset)
    {
        return new Vector2(offset.x, offset.z).magnitude;
    }

    protected static DistanceValue CurvePoint(float distanceMeters, float value)
    {
        return new DistanceValue { distance = distanceMeters, value = value };
    }
}
