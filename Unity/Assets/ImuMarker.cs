using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Represents an Optical-IMU marker that can recieve Frames.
/// This class is the first to interpret the data type and is responsible for
/// performing any transforms to get the data into the Unity coordinate system.
/// </summary>
public class ImuMarker : MonoBehaviour
{
    public int Id;

    public Vector3 Position;

    public bool HasPosition => ((SystemTime - PositionTime) < 0.002f);

    public float PositionTime;

    public Vector3 Accelerometer;

    public Vector3 Gyroscope;

    public class InertialEvent : UnityEvent<Vector3,Vector3,float> { }

    public InertialEvent OnInertialFrame;

    public bool Integrate;

    public float SystemTime;

    private void Awake()
    {
        Position = transform.localPosition;
        PositionTime = 0;
        SystemTime = 0;
        OnInertialFrame = new InertialEvent();
    }

    public void ApplyFrame(StreamFrame frame)
    {
        switch (frame.Type)
        {
            case StreamType.OpticalPosition:
                Position = frame.Data;
                PositionTime = frame.Time;
                break;
            case StreamType.Accelerometer:

                // These lines transform the IMU data into the local coordinate
                // system.

                Accelerometer = frame.Data;

                // This rotation changes the basis of the IMU so that when it
                // has the physical rotation that we want to be Identity in the
                // real world, the measurements align with Identity in Unity.

                Accelerometer.x = -frame.Data.y;
                Accelerometer.y = -frame.Data.z;
                Accelerometer.z = -frame.Data.x;


                // These lines transform the IMU data into the local coordinate
                // system, and convert from g's to m/s^2.

                Accelerometer = Accelerometer * 9.8f;

                break;
            case StreamType.Gyroscope:
                Gyroscope = frame.Data;

                // These lines transform the IMU data into the local coordinate
                // system, and convert from degrees per second to radians per
                // second.

                // Unity uses a left-handed coordinate system, so positive
                // rotations are clockwise around the relevent axis.

                Gyroscope.x = -frame.Data.y;
                Gyroscope.y = -frame.Data.z;
                Gyroscope.z = -frame.Data.x;

                Gyroscope = Gyroscope * Mathf.Deg2Rad;

                if(Integrate)
                {
                    transform.rotation *= Quaternion.Euler(Gyroscope);
                }

                break;
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.matrix = Matrix4x4.identity;

        if (HasPosition)
        {
            Gizmos.color = Color.yellow;
        }
        else
        {
            Gizmos.color = Color.red;
        }
        Gizmos.DrawWireSphere(transform.position, 0.005f);
    }
}
