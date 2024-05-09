using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static PoseSolver;

/// <summary>
/// Maintains a MotionFrame based on the real-time behaviour of the transform
/// </summary>
public class MotionFrameHelper : MonoBehaviour
{
    public MotionFrame Frame;

    private Vector3 previousPosition;
    private Quaternion previousRotation;
    private Vector3 previousVelocity;

    public Transform positionProvider;
    public Transform rotationProvider;

    private void Reset()
    {
        positionProvider = transform;
        rotationProvider = transform;
    }

    // Update is called once per frame
    void Update()
    {
        UpdateMotionFrame(positionProvider.position, rotationProvider.rotation);
    }

    public void UpdateMotionFrame(Vector3 position, Quaternion rotation)
    {
        var deltaPosition = position - previousPosition;
        var deltaRotation = rotation * Quaternion.Inverse(previousRotation);
        Vector3 deltaRotationAxis;
        float deltaRotationAngle;
        deltaRotation.ToAngleAxis(out deltaRotationAngle, out deltaRotationAxis);
        var angularVelocity = Quaternion.AngleAxis(deltaRotationAngle / Time.deltaTime, deltaRotationAxis);
        var velocity = deltaPosition / Time.deltaTime;
        var acceleration = velocity - previousVelocity;

        previousPosition = position;
        previousRotation = rotation;
        previousVelocity = velocity;

        Frame.position = position;
        Frame.rotation = rotation;
        Frame.acceleration = acceleration;
        Frame.angularVelocity = angularVelocity;
    }
}
