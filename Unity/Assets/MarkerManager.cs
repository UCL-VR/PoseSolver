using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Events;

public interface IStreamable
{
    UnityEvent<StreamFrame> OnStreamFrame { get; }
}

public enum StreamType : int
{
    Accelerometer = 1,
    Gyroscope = 2,
    OpticalPosition = 3
}

[StructLayout(LayoutKind.Sequential)]
public struct StreamFrame
{
    public int Marker;
    public float Time;
    public StreamType Type;
    public Vector3 Data;
}
