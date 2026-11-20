using Games.Rebuilt.Components;
using RobotFramework.Components;
using RobotFramework.Controllers.PidSystems;
using RobotFramework.Enums;
using UnityEngine;

public sealed class AwtybotsV1 : AwtybotsRebuiltRobot
{
    private const float AimingDriveMultiplier = 0.4f;
    private const float HubFeedDelay = 0.5f;
    private const float HubAgitationDelay = 1.5f;
    private const float ManualAgitationDelay = 2.5f;
    private const float AgitationHalfCycle = 0.2f;
    private const float ManualShotRpm = 1735f;

    [Header("V1 intake slide")]
    [SerializeField, Tooltip("The carriage moves along the rail frame's local +Z axis.")]
    private GenericJoint intakeSlide;
    [SerializeField, Min(0f)] private float deployedIntakeInches = 12.25f;
    [SerializeField] private PidConstants intakeSlidePid = new PidConstants(8f, 0f, 0f, 1f, 1f);
    [SerializeField, Min(0f), Tooltip("Travel from stowed. Equal distances disable agitation.")]
    private float agitationExtendedInches;
    [SerializeField, Min(0f)] private float agitationRetractedInches;
    [SerializeField] private GenericJoint climberSlide;
    [SerializeField] private PidConstants climberPid = new PidConstants(10f, 0f, 0f, 1f, 1f);

    [Header("V1 shot tuning")]
    [SerializeField] private RebuiltInterpolation hubShots = new RebuiltInterpolation(null, new[]
    {
        CurvePoint(2f, 1460f), CurvePoint(2.5f, 1695f), CurvePoint(3f, 1810f), CurvePoint(3.5f, 1950f),
        CurvePoint(4f, 2080f), CurvePoint(5.2048f, 2317f), CurvePoint(6f, 2460f)
    });

    [SerializeField] private RebuiltInterpolation passShots = new RebuiltInterpolation(null, new[]
    {
        CurvePoint(2f, 1622.8225f), CurvePoint(3f, 1900f), CurvePoint(4f, 2200f), CurvePoint(5.2048f, 2296.918f)
    });

    [SerializeField, Tooltip("Scales the hardware pass table for Fuel landings in MoSim.")]
    private AnimationCurve passRpmScale = AnimationCurve.Linear(3f, 0.88f, 4.9f, 0.91f);

    [SerializeField] private RebuiltInterpolation flightTimes = new RebuiltInterpolation(null, new[]
    {
        CurvePoint(2f, 0.85f), CurvePoint(3f, 0.9f), CurvePoint(4f, 1.08f), CurvePoint(5f, 1.2f), CurvePoint(6f, 1.4f)
    });

    [SerializeField, Min(0f), Tooltip("Scales the time-of-flight lead. Zero disables movement compensation.")]
    private float flightTimeScale = 1f;

    private Rigidbody chassis;
    private bool intakeDeployed;
    private bool autoShootWasHeld;
    private bool passOutsideZone;
    private float hubReadySince = -1f;
    private float feedingSince = -1f;
    private float activeFeedAgitationDelay;
    private float agitationSince = -1f;
    private bool climberExtended;
    private FixedJoint heldRung;
    private readonly Collider[] nearbyRungs = new Collider[8];

    protected override void LateInit()
    {
        base.LateInit();
        if (intakeSlide)
        {
            intakeSlide.linearAxis = JointAxis.Z;
            intakeSlide.useStartingOffset = true;
            intakeSlide.SetPid(intakeSlidePid);
        }

        chassis = GetComponent<Rigidbody>();
        DriveController?.OverrideBrakeTime(0.25f, true);
        if (climberSlide)
        {
            climberSlide.linearAxis = JointAxis.Y;
            climberSlide.useStartingOffset = true;
            climberSlide.SetPid(climberPid);
        }
        InitializeShotCurves(hubShots, passShots, flightTimes);
        Scheduler.AddSubsystems(UpdateMechanisms);
    }

    private void UpdateMechanisms()
    {
        if (climberSlide)
        {
            if (DpadUp.WasTriggered())
            {
                climberExtended = !climberExtended;
                if (climberExtended && heldRung)
                {
                    Destroy(heldRung);
                    heldRung = null;
                }
            }
            if (!climberExtended && !heldRung)
                TryLatchRung();
            climberSlide.SetLinearTarget(climberExtended ? 7.64f : 0f);
        }
        bool autoShoot = OuttakeAction;
        if (autoShoot && !autoShootWasHeld)
        {
            passOutsideZone = !InAllianceZone();
        }
        autoShootWasHeld = autoShoot;

        bool aiming = autoShoot || AutoPass;
        bool passing = AutoPass || (autoShoot && passOutsideZone);
        DriveController?.SetDriveMp(aiming ? AimingDriveMultiplier : 1f);

        bool shotReady = UpdateShooter(aiming, passing);
        bool reversingFeed = DpadL;
        bool feedingShot = CanFeed && shotReady && !reversingFeed;
        float feedDirection = reversingFeed ? -1f : feedingShot || DpadR ? 1f : 0f;
        SetFeeder(CanFeed ? feedDirection : 0f);

        float agitationDelay = aiming ? (passing ? 0f : HubAgitationDelay) : ManualAgitationDelay;
        if (!feedingShot)
        {
            feedingSince = -1f;
        }
        else if (feedingSince < 0f || activeFeedAgitationDelay != agitationDelay)
        {
            feedingSince = Time.fixedTime;
            activeFeedAgitationDelay = agitationDelay;
        }

        bool agitating = RobotSpecial3 || (feedingShot && Time.fixedTime - feedingSince >= agitationDelay);
        UpdateIntake(feedingShot, agitating);
    }

    private void TryLatchRung()
    {
        Transform stage = climberSlide.transform;
        Vector3 notch = stage.TransformPoint(new Vector3(-0.324f, 0.518f, -0.096f));
        int count = Physics.OverlapSphereNonAlloc(notch, 0.070f, nearbyRungs,
            LayerMask.GetMask("Field"), QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            if (nearbyRungs[i] is not CapsuleCollider rung || !rung.name.Contains("Tower Rung"))
                continue;

            Vector3 localAxis = rung.direction == 0 ? Vector3.right : rung.direction == 1 ? Vector3.up : Vector3.forward;
            Vector3 origin = rung.transform.TransformPoint(rung.center);
            Vector3 extent = rung.transform.TransformVector(localAxis * (rung.height / 2f - rung.radius));
            Vector3 along = extent.normalized;
            if (Mathf.Abs(Vector3.Dot(stage.right, along)) < 0.90f)
                continue;

            Vector3 point = origin + along * Mathf.Clamp(Vector3.Dot(notch - origin, along), -extent.magnitude, extent.magnitude);
            Vector3 local = stage.InverseTransformPoint(point);
            if (Mathf.Abs(local.x + 0.324f) > 0.026f || Mathf.Abs(local.z + 0.096f) > 0.023f ||
                local.y < 0.474f || local.y > 0.527f)
                continue;

            // Only seat a rung that is inside the hook as it retracts.
            // Both anchors start at the point the bar already occupies.
            heldRung = stage.gameObject.AddComponent<FixedJoint>();
            heldRung.connectedBody = rung.attachedRigidbody;
            heldRung.autoConfigureConnectedAnchor = false;
            heldRung.anchor = local;
            heldRung.connectedAnchor = rung.attachedRigidbody ? rung.attachedRigidbody.transform.InverseTransformPoint(point) : point;
            heldRung.enableCollision = true;
            break;
        }
    }

    private bool UpdateShooter(bool aiming, bool passing)
    {
        if (!aiming)
        {
            hubReadySince = -1f;
            bool manualShoot = ManualShoot;
            bool atSpeed = SetFlywheel(manualShoot || RobotSpecial2 ? ManualShotRpm : 0f);
            return manualShoot && atSpeed;
        }

        if (!CanAimShot)
        {
            hubReadySince = -1f;
            SetFlywheel(0f);
            return false;
        }

        Vector3 target = CompensateForTravel(SelectAimTarget(passing));
        Aim.SetCustomTarget(target);

        Vector3 offset = target - transform.position;
        float distance = HorizontalDistance(offset);
        // Goldfire launches over the hopper toward the intake.
        float heading = Mathf.Atan2(offset.x, offset.z) * Mathf.Rad2Deg;
        Vector3 velocity = chassis ? chassis.velocity : Vector3.zero;
        float headingRate = (offset.x * velocity.z - offset.z * velocity.x)
            / Mathf.Max(distance * distance, 0.01f) * Mathf.Rad2Deg;
        Aim.Aim(heading, Mathf.Clamp(headingRate, -180f, 180f));

        var shots = passing ? passShots : hubShots;
        float rpm = shots.Interpolate(distance);
        bool ready = SetFlywheel(passing ? rpm * passRpmScale.Evaluate(distance) : rpm);
        float aimError = Mathf.Abs(Aim.GetRotationError());
        if (passing || !ready || aimError > 3f)
        {
            hubReadySince = -1f;
            return passing && ready && aimError <= 3f;
        }

        if (hubReadySince < 0f && aimError <= 1f)
        {
            hubReadySince = Time.fixedTime;
        }
        return hubReadySince >= 0f && Time.fixedTime - hubReadySince >= HubFeedDelay;
    }

    private Vector3 CompensateForTravel(Vector3 target)
    {
        if (!chassis || flightTimeScale <= 0f)
        {
            return target;
        }

        Vector3 velocity = chassis.velocity;
        velocity.y = 0f;
        Vector3 compensated = target;
        for (int i = 0; i < 4; i++)
        {
            float flightTime = flightTimes.Interpolate(HorizontalDistance(compensated - transform.position));
            compensated = target - velocity * (flightTime * flightTimeScale);
        }
        return compensated;
    }

    private void UpdateIntake(bool feedingShot, bool agitating)
    {
        bool stowing = StowAction;
        bool collecting = IntakeAction;
        if (stowing)
        {
            intakeDeployed = false;
        }
        else if (collecting)
        {
            intakeDeployed = true;
        }

        if (intakeSlide)
        {
            intakeSlide.SetLinearTarget(IntakeTravel(!stowing && !collecting && agitating));
        }

        float direction = FaceDown ? -1f : stowing || collecting || feedingShot ? 1f : 0f;
        SetIntakeRollers(direction);
    }

    private float IntakeTravel(bool agitating)
    {
        float extended = Mathf.Clamp(agitationExtendedInches, 0f, deployedIntakeInches);
        float retracted = Mathf.Clamp(agitationRetractedInches, 0f, deployedIntakeInches);
        if (!agitating || extended == retracted)
        {
            agitationSince = -1f;
            return intakeDeployed ? deployedIntakeInches : 0f;
        }

        if (agitationSince < 0f)
        {
            agitationSince = Time.fixedTime;
        }
        intakeDeployed = true;
        float phase = Mathf.Repeat(Time.fixedTime - agitationSince, 2f * AgitationHalfCycle);
        return phase < AgitationHalfCycle ? extended : retracted;
    }

    protected override void ResetWhileDisabled()
    {
        autoShootWasHeld = false;
        hubReadySince = -1f;
        feedingSince = -1f;
        agitationSince = -1f;
        DriveController?.SetDriveMp(1f);
    }
}
