using RobotFramework.Components;
using UnityEngine;

public sealed class AwtybotsV1IntakeContact : MonoBehaviour
{
    [SerializeField] private InteractionRoller intakeRoller;
    [SerializeField] private BoxCollider frontBumper;
    [SerializeField] private LayerMask fuelLayer;

    private readonly Collider[] nearbyFuel = new Collider[24];
    private Rigidbody chassis;

    private void Awake()
    {
        chassis = frontBumper ? frontBumper.attachedRigidbody : null;
    }

    private void FixedUpdate()
    {
        if (!chassis || !intakeRoller || intakeRoller.TargetRpm <= 0f)
        {
            return;
        }

        Transform robot = chassis.transform;
        Vector3 roller = robot.InverseTransformPoint(intakeRoller.transform.position);
        Vector3 bumper = robot.InverseTransformPoint(frontBumper.transform.TransformPoint(frontBumper.center));
        float front = bumper.z + frontBumper.size.z / 2f;
        float top = bumper.y + frontBumper.size.y / 2f;
        if (roller.z - front < 0.135f)
        {
            return;
        }

        int count = Physics.OverlapBoxNonAlloc(transform.position, new Vector3(0.29f, 0.18f, 0.25f),
            nearbyFuel, transform.rotation, fuelLayer, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            ApplyIntakeContact(nearbyFuel[i].attachedRigidbody, roller, front, top, bumper.z - frontBumper.size.z / 2f);
            nearbyFuel[i] = null;
        }
    }

    private void ApplyIntakeContact(Rigidbody fuel, Vector3 roller, float bumperFront, float bumperTop, float bumperRear)
    {
        if (!fuel || fuel.isKinematic)
        {
            return;
        }

        Transform robot = chassis.transform;
        Vector3 position = robot.InverseTransformPoint(fuel.worldCenterOfMass);
        float height = IntakeHeight(position.z, roller.z, bumperFront, bumperTop, bumperRear);
        if (Mathf.Abs(position.x) > 0.28f || position.z < bumperRear - 0.11f ||
            position.z > roller.z + 0.093f || position.y > height + 0.053f)
        {
            return;
        }

        // The wheel pulls Fuel underneath first. Lift starts on the bumper
        // side of the axle and ends at the hopper floor.
        const float travelSpeed = 0.90f;
        float aheadHeight = IntakeHeight(position.z - 0.006f, roller.z, bumperFront, bumperTop, bumperRear);
        float pathLift = (aheadHeight - height) * travelSpeed / 0.006f;
        float liftSpeed = Mathf.Clamp(pathLift + (height - position.y) * 22f, -0.7f, 2.65f);
        Vector3 velocity = robot.InverseTransformDirection(fuel.velocity - chassis.GetPointVelocity(fuel.worldCenterOfMass));
        Vector3 acceleration = new Vector3(0f, liftSpeed - velocity.y, -travelSpeed - velocity.z) * 23f;
        acceleration.y += 8.8f;
        fuel.AddForce(robot.TransformDirection(Vector3.ClampMagnitude(acceleration, 56f)), ForceMode.Acceleration);
    }

    private static float IntakeHeight(float z, float rollerZ, float bumperFront, float bumperTop, float bumperRear)
    {
        float corner = Mathf.InverseLerp(rollerZ - 0.034f, bumperFront + 0.030f, z);
        float deck = Mathf.InverseLerp(bumperFront, bumperRear - 0.015f, z);
        return Mathf.SmoothStep(0.076f, bumperTop + 0.079f, corner) + Mathf.SmoothStep(0f, 0.039f, deck);
    }
}
