using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UCL.CASMS.DH;


/// <summary>
/// Estimates the pose of a kinematic hand model based on Denavit-Hartenberg
/// parameters. This Component uses the dh joint API of the solver. Solved
/// poses are projected back onto the Joint parameters.
/// </summary>
public class DHHandSolver : PoseSolver
{
    // In the current DH implementation, branching is not supported so all
    // fingers are modelled as separate DH chains starting at the same transform.
    private List<DHChain> chains = new List<DHChain>();

    // A lookup table of all scene nodes associated with a member of the DH Chain.
    private Dictionary<GameObject, DHChainNode> nodes = new Dictionary<GameObject, DHChainNode>();

    private void Start()
    {
        Initialise();
    }

    /// <summary>
    /// Creates all the poses, joints and measurements involved in this hand
    /// in the solver.
    /// </summary>
    public void Initialise()
    {
        // Get the joints (per-chain) in a representation amenable for loading
        // into the solver.

        // This implementation assumes that each finger has a separate GameObject,
        // directly below the Hand to which the Solver is added.

        // Chains are assumed to be in the order of the Fingers enum.

        foreach (Transform child in transform)
        {
            var dhRoot = child.GetComponentInChildren<DHJointLink>();
            var chain = new DHChain(dhRoot.gameObject);
            chains.Add(chain);
        }

        // Create the entries in the problem

        initialise();

        var pose = addPose(true);   // The pose parameter for the root of the hand/wrist

        foreach (var chain in chains)
        {
            chain.StartingPose = pose;
            AddDHChain(chain);
        }
    }

    /// <summary>
    /// Adds a DHChain to the Solver. Creating all the parameter blocks and
    /// constraints, and updating the DHChain object with references to the
    /// Solver resources.
    /// </summary>
    public void AddDHChain(DHChain chain)
    {
        if(chain.StartingPose == System.IntPtr.Zero)
        {
            throw new System.ArgumentOutOfRangeException();
        }

        foreach (var item in chain.Nodes)
        {
            item.JointR = addDHJoint(item.StartPoseR, item.Parameters.d, item.Parameters.th * Mathf.Deg2Rad, item.Parameters.r, item.Parameters.a * Mathf.Deg2Rad);
            item.EndPoseR = getJointEndPose(item.JointR);
            if(item.Parameters.max_th == item.Parameters.min_th)
            {
                setDhJointParameterConstant(item.JointR, true);
            }
            else
            {
                setDHJointLimit(item.JointR, item.Parameters.min_th * Mathf.Deg2Rad, item.Parameters.max_th * Mathf.Rad2Deg);
            }
            nodes[item.Component.gameObject] = item;
        }
    }

    /// <summary>
    /// Returns the last DH Joint Constraint for the specified finger.
    /// </summary>
    public DHChainNode GetEndNode(Fingers finger)
    {
        return chains[(int)finger].Nodes.Last();
    }

    /// <summary>
    /// Adds a Point Measurement to the Pose at the end of the Joint, and returns
    /// a reference to it.
    /// </summary>
    public TransformPointMeasurement AddPointMeasurement(DHChainNode chainNode, Vector3 point)
    {
        var m = new TransformPointMeasurement();
        m.point = point;
        m.offset = chainNode.Component.Endpoint.transform.InverseTransformPoint(m.point);
        m.PoseR = chainNode.EndPoseR;
        m.Ref = addPointMeasurement(m.PoseR, m.offset.x, m.offset.y, m.offset.z, m.point.x, m.point.y, m.point.z);

        return m;
    }

    // Update is called once per frame
    void Update()
    {
        solve();

        // Project the solution onto the DH configuration

    }

    private void OnDrawGizmos()
    {
        if (!isActiveAndEnabled)
        {
            return;
        }
        foreach (var item in chains)
        {
            DhChainSolver.DrawGizmos(item, 0.001f);
        }
    }
}
