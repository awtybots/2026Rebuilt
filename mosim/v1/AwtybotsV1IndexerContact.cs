using RobotFramework.Components;
using UnityEngine;

public sealed class AwtybotsV1IndexerContact : MonoBehaviour
{
    [SerializeField] private InteractionRoller kicker;
    [SerializeField] private LayerMask fuelLayer;

    private readonly Collider[] nearbyFuel = new Collider[24];
    private Rigidbody chassis;

    private void Awake()
    {
        chassis = GetComponentInParent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (!kicker || kicker.TargetRpm >= 0f)
        {
            return;
        }

        int count = Physics.OverlapBoxNonAlloc(transform.position, new Vector3(0.27f, 0.25f, 0.30f),
            nearbyFuel, transform.rotation, fuelLayer, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            ApplyIndexerContact(nearbyFuel[i].attachedRigidbody);
            nearbyFuel[i] = null;
        }
    }

    private void ApplyIndexerContact(Rigidbody fuel)
    {
        if (!fuel || fuel.isKinematic)
        {
            return;
        }

        Vector3 position = chassis.transform.InverseTransformPoint(fuel.worldCenterOfMass);
        if (Mathf.Abs(position.x) > 0.27f || position.y < 0.19f || position.y > 0.64f ||
            position.z > 0.33f || position.z < -0.24f)
        {
            return;
        }

        // The kicker lifts Fuel from the hopper floor to one of the two flywheel lanes.
        float lane = position.x < 0f ? -0.1f : 0.1f;
        float rise = Mathf.InverseLerp(0.21f, 0.095f, position.z);
        float exitRise = Mathf.InverseLerp(0f, -0.19f, position.z);
        float height = Mathf.Lerp(0.277f, 0.514f, rise) + 0.1f * exitRise;
        float liftSpeed = Mathf.Clamp((height - position.y) * 17f, -0.6f, 2.6f);
        float advanceSpeed = Mathf.Lerp(0.6f, 1.7f, Mathf.InverseLerp(0.29f, 0.50f, position.y));
        Vector3 targetVelocity = new Vector3((lane - position.x) * 9f, liftSpeed, -advanceSpeed);
        Vector3 velocity = chassis.transform.InverseTransformDirection(
            fuel.velocity - chassis.GetPointVelocity(fuel.worldCenterOfMass));
        Vector3 acceleration = (targetVelocity - velocity) * 20f;
        acceleration.y += 8f;
        fuel.WakeUp();
        fuel.AddForce(chassis.transform.TransformDirection(Vector3.ClampMagnitude(acceleration, 58f)), ForceMode.Acceleration);
    }
}
