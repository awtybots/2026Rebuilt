using System.Collections.Generic;
using System.Linq;
using RobotFramework.Components;
using UnityEngine;

public sealed class AwtybotsV1IntakeContact : MonoBehaviour
{
    [SerializeField] private InteractionRoller intakeRoller;
    [SerializeField] private InteractionRoller upperRoller;
    [SerializeField] private BoxCollider frontBumper;
    [SerializeField] private LayerMask fuelLayer;

    private readonly Collider[] nearbyFuel = new Collider[24];
    private Rigidbody chassis;
    private Rigidbody carriage;
    private AwtybotsV1RollerClearance rollerClearance;

    private void Awake()
    {
        chassis = GetComponentInParent<Rigidbody>();
        carriage = intakeRoller ? intakeRoller.GetComponentInParent<Rigidbody>() : null;
    }

    private void Start()
    {
        rollerClearance = new AwtybotsV1RollerClearance(new[] { intakeRoller, upperRoller });
    }

    private void OnDisable() => rollerClearance?.Clear();

    private void FixedUpdate()
    {
        if (!chassis || !frontBumper || !intakeRoller || Mathf.Abs(intakeRoller.TargetRpm) < 1f)
        {
            rollerClearance?.EndStep();
            return;
        }

        Transform robot = chassis.transform;
        if (carriage && carriage.collisionDetectionMode != CollisionDetectionMode.ContinuousSpeculative)
            carriage.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        Vector3 roller = robot.InverseTransformPoint(intakeRoller.transform.position);
        Vector3 upper = upperRoller ? robot.InverseTransformPoint(upperRoller.transform.position) : roller;
        Vector3 bumper = robot.InverseTransformPoint(frontBumper.transform.TransformPoint(frontBumper.center));
        float front = bumper.z + frontBumper.size.z / 2f;
        float top = bumper.y + frontBumper.size.y / 2f;
        if (roller.z - front < 0.135f)
        {
            rollerClearance.EndStep();
            return;
        }

        int count = Physics.OverlapBoxNonAlloc(transform.position, new Vector3(0.29f, 0.18f, 0.25f),
            nearbyFuel, transform.rotation, fuelLayer, QueryTriggerInteraction.Ignore);
        float travelSpeed = Mathf.Sign(intakeRoller.TargetRpm) * 0.78f;
        for (int i = 0; i < count; i++)
        {
            ApplyIntakeContact(nearbyFuel[i], roller, upper, front, top, bumper.z - frontBumper.size.z / 2f, travelSpeed);
            nearbyFuel[i] = null;
        }
        rollerClearance.EndStep();
    }

    private void ApplyIntakeContact(Collider piece, Vector3 roller, Vector3 upper, float bumperFront, float bumperTop, float bumperRear, float travelSpeed)
    {
        Rigidbody fuel = piece.attachedRigidbody;
        if (!fuel || fuel.isKinematic)
        {
            return;
        }

        Transform robot = chassis.transform;
        Vector3 position = robot.InverseTransformPoint(fuel.worldCenterOfMass);
        float height = IntakeHeight(position.z, roller.z, bumperFront, bumperTop, bumperRear);
        float approachSpeed = travelSpeed > 0f ? Vector3.Dot(chassis.velocity, robot.forward) : 0f;
        var toUpper = new Vector2(position.y - upper.y, position.z - upper.z);
        bool stackedContact = approachSpeed > 1.15f && position.z >= bumperFront + 0.035f
            && position.y >= roller.y && toUpper.sqrMagnitude < 0.109f * 0.109f;
        if (Mathf.Abs(position.x) > 0.28f || position.z < bumperRear - 0.11f ||
            position.z > roller.z + 0.093f || (!stackedContact && position.y > height + 0.053f))
        {
            return;
        }

        rollerClearance.Compress(piece);
        // The wheel pulls Fuel underneath first. Lift starts on the bumper
        // side of the axle and ends at the hopper floor.
        float step = Mathf.Sign(travelSpeed) * 0.006f;
        float aheadHeight = IntakeHeight(position.z - step, roller.z, bumperFront, bumperTop, bumperRear);
        float pathLift = (aheadHeight - height) * Mathf.Abs(travelSpeed) / 0.006f;
        float liftSpeed = Mathf.Clamp(pathLift + (height - position.y) * 22f, -0.7f, 2.65f);
        if (stackedContact)
        {
            height = Mathf.Max(height, bumperTop + 0.079f);
            liftSpeed = Mathf.Clamp((height - position.y) * 17f, -0.55f, 2.6f);
            travelSpeed += Mathf.Clamp((approachSpeed - 1.15f) * 0.42f, 0f, 1.15f);
        }
        Vector3 velocity = robot.InverseTransformDirection(fuel.velocity - chassis.GetPointVelocity(fuel.worldCenterOfMass));
        Vector3 acceleration = new Vector3(0f, liftSpeed - velocity.y, -travelSpeed - velocity.z) * 28f;
        acceleration.y += 8.8f;
        fuel.AddForce(robot.TransformDirection(Vector3.ClampMagnitude(acceleration, stackedContact ? 90f : 65f)), ForceMode.Acceleration);
    }

    private static float IntakeHeight(float z, float rollerZ, float bumperFront, float bumperTop, float bumperRear)
    {
        float gap = Mathf.InverseLerp(rollerZ - 0.010f, bumperFront + 0.073f, z);
        float corner = Mathf.InverseLerp(bumperFront + 0.073f, bumperFront + 0.025f, z);
        float deck = Mathf.InverseLerp(bumperFront, bumperRear - 0.015f, z);
        return Mathf.SmoothStep(0.076f, bumperTop + 0.044f, gap)
            + Mathf.SmoothStep(0f, 0.035f, corner) + Mathf.SmoothStep(0f, 0.057f, deck);
    }
}

internal sealed class AwtybotsV1RollerClearance
{
    private readonly Collider[] cores;
    private HashSet<Collider> compressed = new();
    private HashSet<Collider> active = new();

    public AwtybotsV1RollerClearance(IEnumerable<InteractionRoller> rollers)
    {
        cores = rollers.Where(roller => roller)
            .Select(roller => roller.transform.Find("Wheel_InternalCore")?.GetComponent<Collider>())
            .Where(core => core).ToArray();
    }

    public void Compress(Collider piece)
    {
        if (active.Add(piece) && !compressed.Contains(piece))
            SetCompression(piece, true);
    }

    public void EndStep()
    {
        foreach (var piece in compressed)
        {
            if (active.Contains(piece))
                continue;
            if (IntersectsRoller(piece))
                active.Add(piece);
            else
                SetCompression(piece, false);
        }
        (compressed, active) = (active, compressed);
        active.Clear();
    }

    public void Clear()
    {
        foreach (var piece in compressed)
            SetCompression(piece, false);
        compressed.Clear();
        active.Clear();
    }

    private bool IntersectsRoller(Collider piece)
    {
        if (!piece || !piece.enabled || !piece.gameObject.activeInHierarchy)
            return false;
        foreach (var core in cores)
        {
            if (core && core.bounds.Intersects(piece.bounds) && Physics.ComputePenetration(
                core, core.transform.position, core.transform.rotation,
                piece, piece.transform.position, piece.transform.rotation, out _, out _))
                return true;
        }
        return false;
    }

    private void SetCompression(Collider piece, bool enabled)
    {
        if (!piece || !piece.enabled || !piece.gameObject.activeInHierarchy)
            return;
        // The guided force models the soft tread during capture. Allow that
        // contact to compress without disabling nearby panels or other Fuel.
        foreach (var core in cores)
        {
            if (core)
                Physics.IgnoreCollision(core, piece, enabled);
        }
    }
}
