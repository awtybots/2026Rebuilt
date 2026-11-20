using RobotFramework.Components;
using UnityEngine;

public sealed class AwtybotsV1IndexerContact : MonoBehaviour
{
    internal const float TowerExitHeight = 0.565f;
    private const float TowerEntryZ = 0.025f;

    // Each side of Goldfire follows this arc around the outside of its
    // spindexer (distance from robot centerline, forward Z).
    private static readonly Vector2[] HopperPath =
    {
        new(0.086f, 0.295f), new(0.121f, 0.252f), new(0.169f, 0.221f),
        new(0.208f, 0.179f), new(0.216f, 0.139f), new(0.207f, 0.115f),
        new(0.171f, 0.104f), new(0.133f, 0.084f), new(0.104f, 0.058f),
        new(0.100f, TowerEntryZ),
    };

    // (Forward Z, height Y) on the centerline of either CAD shooter lane.
    // The first section fits below the kickers. The bend follows the
    // inside of the tower guide into the gap between the flywheel and backspin.
    private static readonly Vector2[] TowerPath =
    {
        new(TowerEntryZ, 0.202f), new(-0.080f, 0.199f),
        new(-0.118f, 0.205f), new(-0.151f, 0.227f), new(-0.172f, 0.270f),
        new(-0.183f, 0.328f), new(-0.193f, 0.380f), new(-0.200f, 0.422f),
        new(-0.188f, 0.484f), new(-0.181f, 0.528f), new(-0.200f, 0.567f),
        new(-0.223f, 0.600f),
    };

    [SerializeField] private InteractionRoller kicker;
    [SerializeField] private InteractionRoller[] feedRollers = System.Array.Empty<InteractionRoller>();
    [SerializeField] private LayerMask fuelLayer;

    private readonly Collider[] nearbyFuel = new Collider[24];
    private Rigidbody chassis;
    private AwtybotsV1RollerClearance rollerClearance;

    private void Awake()
    {
        chassis = GetComponentInParent<Rigidbody>();
    }

    private void Start()
    {
        rollerClearance = new AwtybotsV1RollerClearance(feedRollers);
    }

    private void OnDisable() => rollerClearance?.Clear();

    private void FixedUpdate()
    {
        if (!kicker || Mathf.Abs(kicker.TargetRpm) < 1f)
        {
            rollerClearance?.EndStep();
            return;
        }

        int count = Physics.OverlapBoxNonAlloc(transform.position, new Vector3(0.27f, 0.25f, 0.30f),
            nearbyFuel, transform.rotation, fuelLayer, QueryTriggerInteraction.Ignore);
        for (int i = 0; i < count; i++)
        {
            ApplyIndexerContact(nearbyFuel[i], kicker.TargetRpm < 0f);
            nearbyFuel[i] = null;
        }
        rollerClearance.EndStep();
    }

    private void ApplyIndexerContact(Collider piece, bool feeding)
    {
        Rigidbody fuel = piece.attachedRigidbody;
        if (!fuel || fuel.isKinematic)
        {
            return;
        }

        Vector3 position = chassis.transform.InverseTransformPoint(fuel.worldCenterOfMass);
        if (Mathf.Abs(position.x) > 0.27f || position.y < 0.145f ||
            position.z > 0.33f || position.z < -0.255f || position.y >= TowerExitHeight)
        {
            return;
        }

        bool inHopper = position.z > TowerEntryZ;
        float side = position.x < 0f ? -1f : 1f;
        var path = inHopper ? HopperPath : TowerPath;
        var point = inHopper ? new Vector2(Mathf.Abs(position.x), position.z) : new Vector2(position.z, position.y);
        var track = ClosestPoint(path, point, out int segment);
        var error = track - point;
        if (error.sqrMagnitude > 0.145f * 0.145f || (inHopper && position.y > 0.35f))
        {
            return;
        }

        rollerClearance.Compress(piece);
        Vector2 ahead = LookAhead(path, track, segment, feeding);
        Vector2 tangent = (ahead - track).normalized;
        float speed = inHopper ? 0.78f : Mathf.Lerp(0.86f, 1.8f, Mathf.InverseLerp(0.24f, 0.57f, position.y));
        Vector2 motion = tangent * speed + error * 15f;
        float hopperLift = (HopperHeight(ahead.x) - HopperHeight(track.x)) * (speed / 0.034f);
        Vector3 targetVelocity = inHopper
            ? new Vector3(side * motion.x, Mathf.Clamp(hopperLift + (HopperHeight(track.x) - position.y) * 18f, -1.5f, 1.5f), motion.y)
            : new Vector3(Mathf.Clamp((side * 0.10f - position.x) * 8f, -0.7f, 0.7f), motion.y, motion.x);
        Vector3 velocity = chassis.transform.InverseTransformDirection(
            fuel.velocity - chassis.GetPointVelocity(fuel.worldCenterOfMass));
        Vector3 acceleration = (targetVelocity - velocity) * 20f;
        acceleration.y += 8f;
        fuel.WakeUp();
        fuel.AddForce(chassis.transform.TransformDirection(Vector3.ClampMagnitude(acceleration, 58f)), ForceMode.Acceleration);
    }

    private static float HopperHeight(float x) => Mathf.SmoothStep(0.202f, 0.267f, Mathf.InverseLerp(0.1f, 0.216f, x));

    private static Vector2 ClosestPoint(Vector2[] path, Vector2 position, out int segment)
    {
        float bestDistance = float.PositiveInfinity;
        var nearest = path[0];
        segment = 0;
        for (int i = 0; i < path.Length - 1; i++)
        {
            Vector2 start = path[i];
            Vector2 offset = path[i + 1] - start;
            Vector2 sample = start + offset * Mathf.Clamp01(Vector2.Dot(position - start, offset) / offset.sqrMagnitude);
            float distance = (position - sample).sqrMagnitude;
            if (distance < bestDistance)
            {
                bestDistance = distance;
                nearest = sample;
                segment = i;
            }
        }
        return nearest;
    }

    private static Vector2 LookAhead(Vector2[] path, Vector2 from, int segment, bool forward)
    {
        float distance = 0.034f;
        int direction = forward ? 1 : -1;
        for (int i = forward ? segment + 1 : segment; i >= 0 && i < path.Length; i += direction)
        {
            Vector2 offset = path[i] - from;
            float length = offset.magnitude;
            if (length >= distance)
            {
                return from + offset.normalized * distance;
            }
            distance -= length;
            from = path[i];
        }
        Vector2 finalDirection = forward ? path[^1] - path[^2] : path[0] - path[1];
        return from + finalDirection.normalized * distance;
    }
}
