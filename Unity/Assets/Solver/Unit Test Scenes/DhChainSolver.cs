using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

using UCL.CASMS.DH;
using System;

/// <summary>
/// DHChain is a convenience object that holds the structure of a DHJointLink
/// hierarchy, along with references to complementary objects in the solver.
/// It can be created for a sequential chain of joints from the Unity Scene
/// Graph.
/// </summary>
public class DHChain
{
    /// <summary>
    /// The Pose3 that is the reference frame for the first joint. Can be shared.
    /// </summary>
    public IntPtr StartingPose; 

    /// <summary>
    /// List of all Nodes in the Chain. Nodes are in sequence from the start
    /// to the end, in order of dependency.
    /// </summary>
    public List<DHChainNode> Nodes = new List<DHChainNode>();

    public DHChain(GameObject start)
    {
        foreach (var item in DHSolverHelpers.GetJoints(start))
        {
            var node = new DHChainNode(this);
            node.Component = item;
            node.Parameters = node.Component.joint;

            if (Nodes.Count() > 0)
            {
                node.Prev = Nodes.Last();
                node.Prev.Next = node;
            }

            Nodes.Add(node);
        }
    }
}

// The graphics focused implementation of the DH chain in Unity does not allow
// branching, so all sequences of DH chains are one-dimensional.
public class DHChainNode
{
    public DHChainNode(DHChain chain)
    {
        this.chain = chain;
    }

    private DHChain chain;

    public DHChainNode Prev;
    public DHChainNode Next;

    public DHJointLink Component;
    public UCL.CASMS.DH.Joint Parameters; // The parameters of the Component

    public IntPtr JointR; // Reference to the object in solver 
    public IntPtr EndPoseR; // End frame pose

    public IntPtr StartPoseR => Prev != null ? Prev.EndPoseR : chain.StartingPose;
}

public class DHSolverHelpers
{
    /// <summary>
    /// Returns a sequence of DHJointLinks from the Unity Scene Graph in order
    /// </summary>
    public static IEnumerable<DHJointLink> GetJoints(GameObject start)
    {
        foreach (var j in start.GetComponents<DHJointLink>())
        {
            yield return j;
        }
        foreach (Transform child in start.transform)
        {
            foreach (var item in GetJoints(child.gameObject))
            {
                yield return item;
            }
        }
    }
}

public class TransformPointMeasurement
{
    public Vector3 point;
    public Vector3 offset;

    public IntPtr Ref;
    public IntPtr PoseR;

    public void Update(Vector3 point)
    {
        PoseSolver.updatePointMeasurement(Ref, point.x, point.y, point.z);
    }
}


/// <summary>
/// Solves a Denavit-Hartenberg chain with a single, possibly offset, measurement
/// within the reference frame of the end effector.
/// </summary>
public class DhChainSolver : PoseSolver
{
    public float GizmoSize = 0.25f;

    public DHJointLink Root;
    public Transform Measurement;
    public Vector3 MeasurementOffset;

    [NonSerialized]
    DHChain chain;

    public void CreateDHChain()
    {
        chain = new DHChain(Root.gameObject);
        AddDHChain(chain);
    }

    public void AddDHChain(DHChain chain)
    {
        chain.StartingPose = addPose(true);
        foreach (var item in chain.Nodes)
        {
            item.JointR = addDHJoint(item.StartPoseR, item.Parameters.d, item.Parameters.th * Mathf.Deg2Rad, item.Parameters.r, item.Parameters.a * Mathf.Deg2Rad);
            item.EndPoseR = getJointEndPose(item.JointR);
        }
    }

    public override void Solve()
    {
        initialise();
        CreateDHChain();
        addPointMeasurement(chain.Nodes.Last().EndPoseR, MeasurementOffset.x, MeasurementOffset.y, MeasurementOffset.z, Measurement.position.x, Measurement.position.y, Measurement.position.z);
        solve();
    }

    private void Update()
    {
        Solve();
    }

    /// <summary>
    /// Draws the DH Chain inside an OnDrawGizmos method. This draws items using
    /// the parameter blocks of the solver API, with just the structure taken
    /// from the chain (that is, this is a 'purer' representation of the estimated
    /// state than any projection into the Unity Components.
    /// </summary>
    public static void DrawGizmos(DHChain chain, float gizmoSize)
    {
        Gizmos.color = Color.green;
        Gizmos.matrix = Matrix4x4.identity;

        // This Gizmos method draws the state of the solved system, without
        // mapping it back to any Unity Components or transforms (that is,
        // this shows the more 'pure' representation of what the solver
        // thinks is going on).

        Gizmos.DrawWireSphere(getPose(chain.StartingPose).Position, gizmoSize);
        foreach (var item in chain.Nodes)
        {
            Gizmos.DrawWireSphere(getPose(item.EndPoseR).Position, gizmoSize);
        }

        foreach (var item in chain.Nodes)
        {
            Gizmos.color = Color.green;
            var p = getPose(getJointStartPose(item.JointR));
            var a = getJointAngle(item.JointR);
            Gizmos.DrawLine(p.Position, 
                p.Position + p.Rotation * Vector3.up * item.Parameters.d + p.Rotation * (Quaternion.AngleAxis(a * Mathf.Rad2Deg, Vector3.up) * Vector3.forward * item.Parameters.r));

            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(p.Position, p.Position + p.Rotation * Vector3.forward * 0.01f);

            Gizmos.color = Color.green;
            Gizmos.DrawLine(p.Position, p.Position + p.Rotation * Vector3.up * 0.01f);

#if UNITY_EDITOR
            UnityEditor.Handles.color = Color.yellow;
            UnityEditor.Handles.Label(p.Position, $"{a}");
#endif
        }
    }

    private void OnDrawGizmos()
    {
        if (chain != null)
        {
            DhChainSolver.DrawGizmos(chain, GizmoSize);

            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(Measurement.position, 0.0051f);

            // This snippet shows the orientation of the final pose, in case it
            // is relevant
            if(chain.Nodes.Count > 0) {
                var end = getPose(chain.Nodes.Last().EndPoseR);
                Gizmos.DrawLine(end.Position, end.Position + end.Rotation * Vector3.forward * 0.1f);
            }
        }
    }
}
