using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using UCL.CASMS.DH;
using UnityEngine;

public enum Fingers : int
{
    Thumb = 0,
    Index = 1,
    Middle = 2,
    Ring = 3,
    Little = 4,
    Wrist = 5,
}

public static class FingersExtension
{
    public static Fingers GetFinger(string str)
    {
        if (str.Contains("Thumb"))
        {
            return Fingers.Thumb;
        }
        if (str.Contains("Index"))
        {
            return Fingers.Index;
        }
        if (str.Contains("Middle"))
        {
            return Fingers.Middle;
        }
        if (str.Contains("Ring"))
        {
            return Fingers.Ring;
        }
        if (str.Contains("Little"))
        {
            return Fingers.Little;
        }
        throw new ArgumentOutOfRangeException();
    }
}


/// <summary>
/// Estimes the pose of the Hand1 Kinematic Hand Model built into the Solver.
/// </summary>
public class Hand2Solver : MonoBehaviour
{
    // The Hand1 parameterisation is based on DH chains,
    // one per finger.

    [StructLayout(LayoutKind.Sequential)]
    public struct JointParams
    {
        public double d;
        public double theta;
        public double r;
        public double a;
        public double min;
        public double max;

        public static implicit operator JointParams(UCL.CASMS.DH.Joint j)
        {
            return new JointParams()
            {
                a = j.a * Mathf.Deg2Rad,
                d = j.d,
                r = j.r,
                theta = j.th * Mathf.Deg2Rad,
                min = j.min_th * Mathf.Deg2Rad,
                max = j.max_th * Mathf.Deg2Rad
            };
        }
    }

    // The following types are effectively manually unrolled versions of
    // the native array based ones.

    [StructLayout(LayoutKind.Sequential)]
    public struct ChainParams
    {
        public JointParams joint1;
        public JointParams joint2;
        public JointParams joint3;
        public JointParams joint4;
        public JointParams joint5;
        public JointParams joint6;

        public JointParams Joint(int i)
        {
            switch (i)
            {
                case 0:
                    return joint1;
                case 1:
                    return joint2;
                case 2:
                    return joint3;
                case 3:
                    return joint4;
                case 4:
                    return joint5;
                case 5:
                    return joint6;
            }
            throw new ArgumentException();
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HandParams
    {
        public ChainParams thumb;
        public ChainParams index;
        public ChainParams middle;
        public ChainParams ring;
        public ChainParams little;

        public ChainParams Chain(int i)
        {
            switch (i)
            {
                case 0:
                    return thumb;
                case 1:
                    return index;
                case 2:
                    return middle;
                case 3:
                    return ring;
                case 4:
                    return little;
            }
            throw new ArgumentException();
        }
    }

    private HandParams hand;
    private List<DHJointLink[]> fingerChains;
    private double[] angles;
    private IntPtr wristTransform;
    private IntPtr handPose;

    [DllImport("PoseSolver.dll")]
    public static extern float hand2_initialise();

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand2_addPose(bool setParameterBlockConstant);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand2_addHand(HandParams hand, IntPtr pose);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand2_getHandEndPose(IntPtr hand, Fingers finger);

    [DllImport("PoseSolver.dll")]
    internal static extern int hand2_getHandPose(IntPtr hand, [Out] double[] array);

    [DllImport("PoseSolver.dll")]
    public static extern void hand2_solve();

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand2_addPointMeasurement(IntPtr pose, float dx, float dy, float dz, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand2_updatePointMeasurement(IntPtr measurement, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand2_disablePointMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand2_addOrientationMeasurement(IntPtr pose, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand2_updateOrientationMeasurement(IntPtr measurement, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand2_disableOrientationMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern Pose hand2_getUnityPose(IntPtr p);

    private ChainParams GetChainParams(Transform root)
    {
        var nodes = root.GetComponentsInChildren<DHJointLink>();
        ChainParams p;
        p.joint1 = nodes[0].joint;
        p.joint2 = nodes[1].joint;
        p.joint3 = nodes[2].joint;
        p.joint4 = nodes[3].joint;
        p.joint5 = nodes[4].joint;
        p.joint6 = nodes[5].joint;
        return p;
    }

    private DHJointLink[] GetChainNodes(Transform root)
    {
        return root.GetComponentsInChildren<DHJointLink>();
    }

    private void Start()
    {
        Initialise();
    }

    public void Initialise()
    {
        // Get the joints (per-chain) in a representation amenable for loading
        // into the solver.

        // This implementation assumes that each finger has a separate GameObject,
        // directly below the Hand to which the Solver is added.

        hand.thumb = GetChainParams(transform.Find("Thumb"));
        hand.index = GetChainParams(transform.Find("Index"));
        hand.middle = GetChainParams(transform.Find("Middle"));
        hand.ring = GetChainParams(transform.Find("Ring"));
        hand.little = GetChainParams(transform.Find("Little"));

        fingerChains = new List<DHJointLink[]>();

        fingerChains.Add(GetChainNodes(transform.Find("Thumb")));
        fingerChains.Add(GetChainNodes(transform.Find("Index")));
        fingerChains.Add(GetChainNodes(transform.Find("Middle")));
        fingerChains.Add(GetChainNodes(transform.Find("Ring")));
        fingerChains.Add(GetChainNodes(transform.Find("Little")));

        angles = new double[30];

        // Create the entries in the problem

        hand2_initialise();

        wristTransform = hand2_addPose(false);   // The pose parameter for the root of the hand/wrist
        handPose = hand2_addHand(hand, wristTransform);
    }

    public class PointMeasurement
    {
        public IntPtr measurement;
        public void Update(Vector3 point)
        {
            hand2_updatePointMeasurement(measurement, point.x, point.y, point.z);
        }

        public void Remove()
        {
            hand2_disablePointMeasurement(measurement);
        }
    }

    public class OrientationMeasurement
    {
        public IntPtr measurement;

        public void Update(Quaternion q)
        {
            hand2_updateOrientationMeasurement(measurement, q.x, q.y, q.z, q.w);
        }

        public void Remove()
        {
            hand2_disableOrientationMeasurement(measurement);
        }
    }

    public PointMeasurement AddPointConstraint(Fingers finger, Vector3 point)
    {
        var m = new PointMeasurement();
        var endPoint = fingerChains[(int)finger].Last().Endpoint;
        var offset = endPoint.transform.InverseTransformPoint(point);
        var fingerTransform = hand2_getHandEndPose(handPose, finger);
        m.measurement = hand2_addPointMeasurement(fingerTransform, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    public PointMeasurement AddPointConstraint(Vector3 point)
    {
        var m = new PointMeasurement();
        var offset = transform.InverseTransformPoint(point);
        m.measurement = hand2_addPointMeasurement(wristTransform, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint()
    {
        var m = new OrientationMeasurement();
        m.measurement = hand2_addOrientationMeasurement(wristTransform, 0, 0, 0, 1);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint(Fingers finger)
    {
        var m = new OrientationMeasurement();
        var fingerTransform = hand2_getHandEndPose(handPose, finger);
        m.measurement = hand2_addOrientationMeasurement(fingerTransform, 0, 0, 0, 1);
        return m;
    }

    public Transform GetTransform(Fingers finger)
    {
        return fingerChains[(int)finger].Last().Endpoint.transform;
    }

    public Transform GetTransform()
    {
        return transform;
    }

    public void Update()
    {
        PerformanceProfiler.StartFrame();

        hand2_solve();

        PerformanceProfiler.EndFrame();

        // Apply the root transform
        var p = hand2_getUnityPose(wristTransform);
        transform.position = p.Position;
        transform.rotation = p.Rotation;

        // Project the current angles onto the DH Nodes in the Hand Object
        hand2_getHandPose(handPose, angles);
        for (int f = 0; f < 5; f++)
        {
            var offset = f * 6;
            var joints = fingerChains[f];
            for (int i = 0; i < 6; i++)
            {
                joints[i].joint.th = (float)angles[offset + i] * Mathf.Rad2Deg;
            }
        }
    }

    private void OnDrawGizmos()
    {
        if (!isActiveAndEnabled)
        {
            return;
        }

        Gizmos.color = Color.green;

        if (handPose != IntPtr.Zero)
        {
            var origin = hand2_getUnityPose(wristTransform);
            hand2_getHandPose(handPose, angles);

            var anglesList = "";
            foreach (var item in angles)
            {
                anglesList += item.ToString() + "\n";
            }

#if UNITY_EDITOR
            UnityEditor.Handles.color = Color.yellow;
            UnityEditor.Handles.Label(transform.position, $"{anglesList}");
#endif

            for (int i = 0; i < 5; i++)
            {
                var p = hand2_getUnityPose(hand2_getHandEndPose(handPose, (Fingers)i));
                Gizmos.DrawWireSphere(p.Position, 0.005f);
            }
            
            Gizmos.color = Color.green;

            for (int i = 0; i < 5; i++)
            {
                OnDrawChainGizmos(i, origin);
            }
        }
    }

    /// <summary>
    /// Draws a sequence of JointParams as lines, using the angles in angles to
    /// set the rotation of each joint.
    /// </summary>
    /// <remarks>
    /// To do this, the method keeps track of the last endpoint, and its rotation,
    /// as it progresses through the chain.
    /// </remarks>
    private void OnDrawChainGizmos(int chain, Pose origin)
    {
        var p = hand.Chain(chain);

        var rotation = origin.Rotation;
        var position = origin.Position;

        for (int i = 0; i < 6; i++)
        {
            var joint = p.Joint(i);
            var theta = angles[(chain * 6) + i];

            var offset = rotation * Vector3.up * (float)joint.d +
                rotation * (Quaternion.AngleAxis((float)theta * Mathf.Rad2Deg, Vector3.up) * Vector3.forward * (float)joint.r);

            Gizmos.DrawLine(position, position + offset);

            // Note that we do not include the final rotation around the normal
            // above since this doesn't affect the endpoint of the chain on which
            // it exists, but we do add it below since it affects subsequent
            // joints.

            rotation = rotation * (Quaternion.AngleAxis((float)theta * Mathf.Rad2Deg, Vector3.up) * Quaternion.AngleAxis((float)joint.a * Mathf.Rad2Deg, Vector3.forward));
            position = position + offset;
        }
    }
}
