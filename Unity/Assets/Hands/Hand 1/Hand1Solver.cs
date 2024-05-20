using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UCL.CASMS.DH;
using System.Runtime.InteropServices;
using System;

/// <summary>
/// Estimates the pose of a kinematic hand model based on Denavit-Hartenberg
/// parameters. This Component uses the DH Joints API of the solver. Solved
/// poses are projected back onto the Joint parameters.
/// </summary>
public class Hand1Solver : MonoBehaviour
{
    private class JointNode
    {
        public DHJointLink Component;
        public IntPtr JointPtr;
        public IntPtr StartTransformPtr;
        public IntPtr EndTransformPtr;

        public UCL.CASMS.DH.Joint Parameters => Component.joint;

        public JointNode(DHJointLink jl)
        {
            Component = jl;
        }
    }

    private class JointChain
    {
        public List<JointNode> nodes;

        public JointChain(DHJointLink rootLink)
        {
            nodes = rootLink.GetComponentsInChildren<DHJointLink>().Select(jl => new JointNode(jl)).ToList();
        }
    }

    public class PointMeasurement
    {
        public IntPtr measurement;
        
        public void Update(Vector3 p)
        {
            hand1_updatePointMeasurement(measurement, p.x, p.y, p.z);
        }

        public void Remove()
        {
            hand1_disablePointMeasurement(measurement);
        }
    }

    public class OrientationMeasurement
    {
        public IntPtr measurement;

        public void Update(Quaternion q)
        {
            hand1_updateOrientationMeasurement(measurement, q.x, q.y, q.z, q.w);
        }

        public void Remove()
        {
            hand1_disableOrientationMeasurement(measurement);
        }
    }

    // In the current DH implementation, branching is not supported so all
    // fingers are modelled as separate DH chains starting at the same transform.
    private List<JointChain> fingers = new List<JointChain>();

    /// <summary>
    /// When True, the origin (the transform of the wrist) will be fixed in place.
    /// </summary>
    public bool FixedOrigin = false;

    /// <summary>
    /// When True, the Joint Parameters are updated to match the pose (if False,
    /// only the Gizmos show the estimated pose).
    /// </summary>
    public bool Project = true;

    private System.IntPtr wristPose;

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_initialise();

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_addTransform(bool setParameterBlockConstant);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_addJoint(IntPtr pose, float d, float th, float r, float a);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_getJointStartTransform(IntPtr j);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_getJointEndTransform(IntPtr j);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_setJointParameterConstant(IntPtr joint, bool isConstant);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_setJointLimit(IntPtr joint, float min, float max);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_addPointMeasurement(IntPtr pose, float dx, float dy, float dz, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_updatePointMeasurement(IntPtr measurement, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_disablePointMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand1_addOrientationMeasurement(IntPtr pose, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_updateOrientationMeasurement(IntPtr measurement, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand1_disableOrientationMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern Pose hand1_getUnityTransform(IntPtr p);

    [DllImport("PoseSolver.dll")]
    public static extern float hand1_getJointAngle(IntPtr j);



    [DllImport("PoseSolver.dll")]
    public static extern void hand1_solve();

    private void Start()
    {
        Initialise();
    }

    /// <summary>
    /// Creates all the poses, joints and measurements involved in this hand.
    /// </summary>
    public void Initialise()
    {
        // Get the joints (per-chain) in a representation amenable for loading
        // into the solver.
        // This implementation assumes that each finger has a separate GameObject,
        // directly below the Hand to which the Solver is added.

        // Chains are assumed to be in the order of the Fingers enum in the Scene
        // Graph.

        foreach (Transform child in transform)
        {
            if (child.gameObject.activeSelf)
            {
                var chain = new JointChain(child.GetComponentInChildren<DHJointLink>());
                fingers.Add(chain);
            }
        }

        // Create the entries in the problem

        hand1_initialise();

        // The pose parameter block for the root of the hand/wrist

        wristPose = hand1_addTransform(FixedOrigin);

        // Create the parameter blocks for the joint-links making up the chains
        // of each finger.

        foreach (var chain in fingers)
        {
            for (int i = 0; i < chain.nodes.Count; i++)
            {
                var item = chain.nodes[i];
                item.StartTransformPtr = i == 0 ? wristPose : chain.nodes[i - 1].EndTransformPtr;
                item.JointPtr = hand1_addJoint(item.StartTransformPtr, item.Parameters.d, item.Parameters.th * Mathf.Deg2Rad, item.Parameters.r, item.Parameters.a * Mathf.Deg2Rad);
                item.EndTransformPtr = hand1_getJointEndTransform(item.JointPtr);
                if (item.Parameters.max_th == item.Parameters.min_th)
                {
                    hand1_setJointParameterConstant(item.JointPtr, true);
                }
                else
                {
                    hand1_setJointLimit(item.JointPtr, item.Parameters.min_th * Mathf.Deg2Rad, item.Parameters.max_th * Mathf.Deg2Rad);
                }
            }
        }
    }

    /// <summary>
    /// Returns the last DH Joint Constraint for the specified finger.
    /// </summary>
    private JointNode GetEndNode(Fingers finger)
    {
        return fingers[(int)finger].nodes.Last();
    }

    /// <summary>
    /// Adds a Point Measurement Cost Function to the Transform at the end of
    /// the finger chain, and returns a reference to an object that can be used
    /// to update it.
    /// </summary>
    public PointMeasurement AddPointConstraint(Fingers finger, Vector3 point)
    {
        var m = new PointMeasurement();
        var chainNode = GetEndNode(finger);
        var offset = chainNode.Component.Endpoint.transform.InverseTransformPoint(point);
        var pose = chainNode.EndTransformPtr;
        m.measurement = hand1_addPointMeasurement(pose, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    /// <summary>
    /// Adds a Point Measurement to the Transform at the wrist
    /// </summary>
    public PointMeasurement AddPointConstraint(Vector3 point)
    {
        var m = new PointMeasurement();
        var offset = transform.InverseTransformPoint(point);
        m.measurement = hand1_addPointMeasurement(wristPose, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint()
    {
        var m = new OrientationMeasurement();
        m.measurement = hand1_addOrientationMeasurement(wristPose, 0, 0, 0, 1);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint(Fingers finger)
    {
        var m = new OrientationMeasurement();
        var chainNode = GetEndNode(finger);
        var pose = chainNode.EndTransformPtr;
        m.measurement = hand1_addOrientationMeasurement(pose, 0, 0, 0, 1);
        return m;
    }

    public Transform GetTransform(Fingers finger)
    {
        var chainNode = GetEndNode(finger);
        return chainNode.Component.Endpoint.transform;
    }

    public Transform GetTransform()
    {
        return transform;
    }

    // Update is called once per frame
    void Update()
    {
        hand1_solve();

        // Project the solution onto the DH configuration
        if(Project)
        {
            // Update the wrist pose
            var p = hand1_getUnityTransform(wristPose);
            transform.position = p.Position;
            transform.rotation = p.Rotation;

            foreach (var chain in fingers)
            {
                foreach (var node in chain.nodes)
                {
                    node.Component.joint.th = hand1_getJointAngle(node.JointPtr) * Mathf.Rad2Deg;
                }
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (!isActiveAndEnabled)
        {
            return;
        }
        foreach (var chain in fingers)
        {
            Gizmos.color = Color.green;
            Gizmos.matrix = Matrix4x4.identity;

            var gizmoSize = 0.001f;

            // This Gizmos method draws the state of the solved system, without
            // mapping it back to any Unity Components or transforms (that is,
            // this shows the more 'pure' representation of what the solver
            // thinks is going on).

            Gizmos.DrawWireSphere(hand1_getUnityTransform(wristPose).Position, gizmoSize);
            foreach (var item in chain.nodes)
            {
                Gizmos.DrawWireSphere(hand1_getUnityTransform(item.EndTransformPtr).Position, gizmoSize);
            }

            foreach (var item in chain.nodes)
            {
                Gizmos.color = Color.green;
                var p = hand1_getUnityTransform(hand1_getJointStartTransform(item.JointPtr));
                var a = hand1_getJointAngle(item.JointPtr);
                Gizmos.DrawLine(p.Position,
                    p.Position + p.Rotation * Vector3.up * item.Parameters.d + p.Rotation * (Quaternion.AngleAxis(a * Mathf.Rad2Deg, Vector3.up) * Vector3.forward * item.Parameters.r)); // This line doesn't include the final rotation, because it won't affect the position, and it is part of the subsequent pose

                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(p.Position, p.Position + p.Rotation * Vector3.forward * 0.01f);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(p.Position, p.Position + p.Rotation * Vector3.up * 0.01f);

#if UNITY_EDITOR
                UnityEditor.Handles.color = Color.yellow;
                UnityEditor.Handles.Label(p.Position, $"{a}");
#endif

                // This snippet shows the orientation of the final pose (link end)

                Gizmos.color = Color.red;
                var end = hand1_getUnityTransform(chain.nodes.Last().EndTransformPtr);
                Gizmos.DrawLine(end.Position, end.Position + end.Rotation * Vector3.forward * 0.01f);
            }
        }
    }
}
