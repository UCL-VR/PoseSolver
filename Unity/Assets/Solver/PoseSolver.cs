using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct Pose
{
    public Vector3 Position;
    public Quaternion Rotation;

    public static Pose Identity
    {
        get
        {
            return new Pose()
            {
                Position = Vector3.zero,
                Rotation = Quaternion.identity
            };
        }
    }

    public static implicit operator Pose(Transform t)
    {
        Pose p;
        p.Position = t.position;
        p.Rotation = t.rotation;
        return p;
    }
}

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct ImuBiasParameters
{
    public Vector3 accelerometerBias;
    public Vector3 gyroscopeBias;
}

[Serializable]
[StructLayout(LayoutKind.Sequential)]
public struct MotionFrame
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector3 acceleration;
    public Quaternion angularVelocity;
};

public class PoseSolver : MonoBehaviour
{
    /// <summary>
    /// Gets the version - this is used as a test function for whether the DLL
    /// can be loaded.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern float getVersion();

    public float Version => getVersion();
}
