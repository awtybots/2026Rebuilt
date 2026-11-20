using UnityEngine;

public sealed class AwtybotsV1WheelVisual : MonoBehaviour
{
    private Rigidbody chassis;
    private WheelCollider wheel;
    private float angle;

    private void Awake()
    {
        chassis = GetComponentInParent<Rigidbody>();
        wheel = GetComponentInParent<WheelCollider>();
        enabled = chassis && wheel;
    }

    private void LateUpdate()
    {
        // DriveController applies chassis forces directly, so the unloaded
        // WheelCollider RPM does not describe the tread speed on carpet.
        float speed = Vector3.Dot(chassis.GetPointVelocity(transform.position), wheel.transform.forward);
        angle = Mathf.Repeat(angle + speed / wheel.radius * Mathf.Rad2Deg * Time.deltaTime, 360f);
        transform.localRotation = Quaternion.AngleAxis(angle, Vector3.right);
    }
}
