using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

using PosePtr = System.IntPtr;

public class PoseSolver : MonoBehaviour
{
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
    }

    // This first section is concerned with using the library

    /// <summary>
    /// Gets the version - this is used as a test function for whether the DLL
    /// can be loaded.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern float getVersion();

    /// <summary>
    /// Creates a new solver Scene object, that holds the entire problem and
    /// associated objects.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern float initialise();

    /// <summary>
    /// Run the solver for the problem in the current Scene
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern void solve();

    // This section is concerned with the pose type

    
    
    [DllImport("PoseSolver.dll")]
    public static extern PosePtr addPose(bool setParameterBlockConstant);

    [DllImport("PoseSolver.dll")]
    public static extern Pose getPose(IntPtr p);

    // This section is concerned with measurements

    /// <summary>
    /// Adds a measurement in world space relative to the absolute reference
    /// frame pose p.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern IntPtr addPointMeasurement(IntPtr pose, float dx, float dy, float dz, float wx, float wy, float wz);

    /// <summary>
    /// Updates the Point parameter block of the PointMeasurement m (the observed
    /// point). Both the offset and the point parameter blocks remain constant.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern void updatePointMeasurement(IntPtr measurement, float wx, float wy, float wz);

    // This next section is concerned with Denavit-Hartenberg Joints

    [DllImport("PoseSolver.dll")]
    public static extern float getJointAngle(IntPtr j);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr getJointStartPose(IntPtr j);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr getJointEndPose(IntPtr j);

    /// <summary>
    /// Adds a DH Joint to the kinematic chain. The theta parameter will be
    /// optimised. It should be initialised to a sensible value here. The joint
    /// will attach to a newly created pose. The return value is the reference
    /// to the joint.
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern IntPtr addDHJoint(IntPtr pose, float d, float th, float r, float a);

    /// <summary>
    /// Locks or unlocks a Dh Joint Angle (th)
    /// </summary>
    [DllImport("PoseSolver.dll")]
    public static extern void setDhJointParameterConstant(IntPtr joint, bool isConstant);


    // This section is concerned with unified hand models


    [DllImport("PoseSolver.dll")]
    public static extern IntPtr addHand1(Hand1Solver.HandParams hand, IntPtr pose);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr getHand1EndPose(IntPtr hand, Fingers finger);

    [DllImport("PoseSolver.dll")]
    internal static extern int getHand1Pose(IntPtr hand, [Out] double[] array);

    public float Version => getVersion();

    public virtual void Solve()
    {

    }
}
