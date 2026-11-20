using RobotFramework.Components;
using UnityEngine;

public sealed class AwtybotsV1ShooterContact : MonoBehaviour
{
    // Effective release angle estimated from V1's measured flight times.
    [SerializeField, Range(45f, 80f)] private float exitAngleDegrees = 65f;
    [SerializeField] private AnimationCurve contactEfficiency = AnimationCurve.Linear(1650f, 0.98f, 1925f, 0.925f);
    [SerializeField] private FlywheelBehaviour leftFlywheel;
    [SerializeField] private FlywheelBehaviour rightFlywheel;
    [SerializeField] private InteractionRoller leftBackspin;
    [SerializeField] private InteractionRoller rightBackspin;
    [SerializeField] private LayerMask fuelLayer;

    private readonly Collider[] pieces = new Collider[12];
    private Rigidbody chassis;

    private void Awake()
    {
        chassis = GetComponentInParent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (!leftFlywheel || !rightFlywheel || !leftBackspin || !rightBackspin ||
            (Mathf.Abs(leftFlywheel.CurrentRpm) < 500f && Mathf.Abs(rightFlywheel.CurrentRpm) < 500f))
        {
            return;
        }

        int count = Physics.OverlapBoxNonAlloc(transform.position, new Vector3(0.23f, 0.15f, 0.16f),
            pieces, transform.rotation, fuelLayer, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            ApplyShooterContact(pieces[i].attachedRigidbody);
            pieces[i] = null;
        }
    }

    private void ApplyShooterContact(Rigidbody fuel)
    {
        if (!fuel || fuel.isKinematic)
        {
            return;
        }

        Transform frame = chassis.transform;
        Vector3 point = frame.InverseTransformPoint(fuel.worldCenterOfMass);
        if (Mathf.Abs(point.x) > 0.23f || point.y < 0.54f || point.y > 0.71f ||
            point.z > -0.17f || point.z < -0.34f)
        {
            return;
        }

        Vector3 velocity = frame.InverseTransformDirection(
            fuel.velocity - chassis.GetPointVelocity(fuel.worldCenterOfMass));
        if (velocity.z >= -0.25f)
        {
            return;
        }

        bool leftLane = point.x < 0f;
        float flywheelRpm = Mathf.Abs((leftLane ? leftFlywheel : rightFlywheel).CurrentRpm);
        float backspinRpm = Mathf.Abs((leftLane ? leftBackspin : rightBackspin).realRpm);
        float surfaceSpeed = (flywheelRpm * 0.0508f + backspinRpm * 0.0254f) * (Mathf.PI / 60f);
        float angle = exitAngleDegrees * Mathf.Deg2Rad;
        Vector3 target = new Vector3(velocity.x * 0.3f, Mathf.Sin(angle) * surfaceSpeed, -Mathf.Cos(angle) * surfaceSpeed);
        Vector3 correction = target * contactEfficiency.Evaluate(flywheelRpm) - velocity;
        // The compliant outlet adds grip while the ball is inside the shooter.
        fuel.AddForce(frame.TransformDirection(Vector3.ClampMagnitude(correction, 2.7f)), ForceMode.VelocityChange);
    }
}