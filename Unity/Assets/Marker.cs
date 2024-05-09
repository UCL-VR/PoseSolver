using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Represents an Optical-IMU marker that can recieve frames.
/// </summary>
public class Marker : MonoBehaviour
{
    public int Id;

    public Vector3 Position;

    public float PositionAge;

    public Vector3 Accelerometer;

    private void Awake()
    {
        Position = transform.localPosition;
        PositionAge = 0;
    }

    public void OnFrame(StreamFrame frame)
    {
        switch (frame.Type)
        {
            case StreamType.OpticalPosition:
                Position = frame.Data;
                PositionAge = 0;
                break;
            case StreamType.Accelerometer:
                Accelerometer = frame.Data;
                break;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.localPosition = Position;
        PositionAge += Time.deltaTime;
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, 0.005f);

        Gizmos.color = Color.blue;
        Gizmos.DrawRay(transform.position, Accelerometer);
    }
}
