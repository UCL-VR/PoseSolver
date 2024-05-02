using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// An observation of a location relative to a Node. The Node must sit on the
    /// sphere d around the measurement.
    /// </summary>
    public class PointMeasurement : IConstraint
    {
        public Node node;
        public Vector3 offset;
        public Vector3 position;

        public void Project()
        {
            node.SetPosition(position + (node.position - position).normalized * offset.magnitude);
            node.updated = true;
        }

        public Node Node => node;
    }

    public interface IConstraint
    {
        Node Node { get; }
        void Project();
    }

    public interface IJointConstraint
    {
        void Initialise(Node node, Node next);
        void Forwards(Node node, Node next, float d);
        void Backwards(Node node, Node next, float d);
        bool Enabled { get; }
        void DrawGizmos(Node node);
    }

    public class Node
    {
        public Transform transform;

        public Vector3 position;

        public Quaternion rotation;

        public Vector3 right => rotation * Vector3.right;
        public Vector3 up => rotation * Vector3.up;
        public Vector3 forwards => rotation * Vector3.forward;

        public Node(Transform t)
        {
            transform = t;
            position = t.position;
        }

        public List<Vector3> positions = new List<Vector3>();

        public List<Node> connections = new List<Node>(); // Edges

        public Dictionary<Node, FabrikJoint> joints = new Dictionary<Node, FabrikJoint>(); // Indexed by the Node at the center of rotation

        public int index;

        public bool updated;

        // The following flags define where the Node is updated in the solver.
        // These flags are mutually exclusive.

        public bool leaf; // Sits at the end of a chain
        public bool inter; // An intermediate joint in a serial chain
        public bool subbase; // Connects two graphs that should be solved in parallel
        public bool loopinter; // Connects a loop and a serial chain
        public bool root;

        public Node Next(Node previous)
        {
            if (connections.Count == 1)
            {
                return connections[0];
            }
            if (connections.Count == 2)
            {
                if (connections[0] == previous)
                {
                    return connections[1];
                }
                if (connections[1] == previous)
                {
                    return connections[0];
                }
            }
            if (connections.Count > 2)
            {
                throw new Exception("Next can only be called on interjoints");
            }
            throw new ArgumentOutOfRangeException();
        }

        public override string ToString()
        {
            if (transform)
            {
                return transform.name;
            }
            else
            {
                return "null";
            }
        }

        public int AddPosition()
        {
            return 0;
        }

        public void SetPosition(Vector3 position)
        {
            this.position = position;
            updated = true;
        }
    }

    /// <summary>
    /// A chain defines a route between two Nodes. The solver operates by
    /// iterating over the set of chains in the model.
    /// Where a chain is a loop, the first node appears twice: at the start
    /// and end of the chain.
    /// </summary>
    public class Chain : List<Node>
    {
        public List<float> d = new List<float>();

        // Currently only one joint per node per chain is supported
        public List<IJointConstraint> joints = new List<IJointConstraint>();

        // Stores the index of the position controlled by this chain in a subbase
        // for each node.
        public List<int> indices = new List<int>();

        public Node Start => this.First();
        public Node End => this.Last();

        public bool Loop;

        public void UpdateLengths()
        {
            this.d.Clear();
            for (int i = 0; i < Count - 1; i++)
            {
                this.d.Add((this[i].position - this[i + 1].position).magnitude);
            }
        }

        public void UpdateIndices()
        {
            this.indices.Clear();
            for (int i = 0; i < Count - 1; i++)
            {
                indices.Add(this[i].AddPosition());
            }
        }

        public List<Chain> subchains = new List<Chain>();
    }

    public class Subbase
    {
        public Node node;
        public List<Chain> chains;
        public List<Chain> loops;
        public List<Subbase> subbases; // Any subbases at the end of the chains. This is a convenience member.

        public Subbase(Node n)
        {
            node = n;
            chains = new List<Chain>();
            loops = new List<Chain>();
            subbases = new List<Subbase>();
        }

        public List<Vector3> positions = new List<Vector3>();

        public void UpdateSubbase()
        {
            var position = Vector3.zero;
            for (int i = 0; i < positions.Count; i++)
            {
                position += positions[i];
            }
            position /= (float)positions.Count();
            positions.Clear();

            node.position = position;
        }

        public override string ToString()
        {
            return node.ToString();
        }
    }

    public class Model
    {
        public List<Node> nodes;
        public Subbase root; // Root is always a subbase
    }

    [Serializable]
    public class DistanceConstraint
    {
        public Transform to;
        public Transform from;
    }

    public class FabrikSolver : MonoBehaviour
    {
        // Any nodes that may have a direct constraint applied must be in the
        // Effectors list. If the nodes do not have constraints applied in a
        // frame, they are considered to be typical interjoint nodes.
        // If nodes are not in the effectors list however, they cannot have
        // constraints applied.

        // When the Root is set, constraints are built from End Effectors
        // up the scene graph automatically. If the Root is not set, only
        // distance constraints are considered.

        public bool FixRoot;

        public Transform Root;
        public List<Transform> Effectors = new List<Transform>();
        public List<DistanceConstraint> Distances = new List<DistanceConstraint>();

        public Model model;

        public List<IConstraint> constraints = new List<IConstraint>();

        public int Iterations = 10;

        private Dictionary<string, Node> nodesByName = new Dictionary<string, Node>();
        private Dictionary<Transform, Node> nodesByTransform = new Dictionary<Transform, Node>();

        private void Awake()
        {
            MakeModel();
            CreateNodesLookup(model);
        }

        // Update is called once per frame
        void Update()
        {
            Solve();
        }

        public void MakeModel()
        {
            var builder = new ModelBuilder(Effectors, Distances, Root);
            model = builder.Build();
            UpdateOrientations(model.root);
            InitialiseJoints(model.root);
        }

        private void InitialiseJoints(Subbase sb)
        {
            foreach (var chain in sb.chains)
            {
                for (int i = 0; i < chain.Count - 1; i++)
                {
                    if (chain.joints[i] != null)
                    {
                        chain.joints[i].Initialise(chain[i], chain[i + 1]);
                    }
                }
            }

            foreach (var subbase in sb.subbases)
            {
                InitialiseJoints(subbase);
            }
        }

        /// <summary>
        /// Performs a Forward-Reaching Step for a Subbase. This means to iterate
        /// along each of the dependent chains, and then average the result,
        /// setting it as this subbase's position. If a chain terminates in a
        /// subbase, that subbase is solved first, and so on recursively.
        /// </summary>
        private void Forwards(Subbase subbase)
        {
            // Starting conditions for this subbase

            var node = subbase.node;
            var position = node.position;

            // Solve the position of any dependent subases first

            foreach (var sb in subbase.subbases)
            {
                Forwards(sb);
            }

            // Perform the inwards iteration of each chain, with each chain
            // resulting in a new position estimate. The initial conditions are
            // reset for each iteration. Some of these chains will start at the
            // subbasees solved above.

            foreach (var chain in subbase.chains)
            {
                node.position = position;
                Forwards(chain);
                subbase.positions.Add(node.position);
            }

            foreach (var loop in subbase.loops)
            {
                node.position = position;
                ForwardsLoop(loop);
                subbase.positions.Add(node.position);
            }

            // When the reaching is complete, average out the estimates to get
            // the new position for this subbase, which can be used as the
            // starting point of subsequent chains.

            subbase.UpdateSubbase();
        }

        /// <summary>
        /// Performs a Backwards-Reaching Step for a Subbase. This means to
        /// iterate from the Base constraining all segments of each chain until
        /// their outermost nodes. Once the chains have been iterated, the
        /// process repeats recursively starting at each descendent base.
        /// </summary>
        private void Backwards(Subbase subbase)
        {
            // Start by resolving all the loops

            foreach (var loop in subbase.loops)
            {
                BackwardsLoop(loop);
            }

            // Then move outwards through all the chains.

            foreach (var chain in subbase.chains)
            {
                Backwards(chain);
            }

            // Some of the chains may end in a subbase. Each subbase takes its
            // position directly from the node, which has already been set by
            // the chain, so we don't need to update that. We do need to update
            // that subbases chains from that new position however.

            foreach (var sb in subbase.subbases)
            {
                Backwards(sb);
            }
        }

        /// <summary>
        /// Moves forward through the model setting the forward and up vectors
        /// of each Node based on the current pose. This does not change any
        /// positions.
        /// </summary>
        private void UpdateOrientations(Subbase b)
        {
            foreach (var chain in b.chains)
            {
                UpdateOrientations(chain);
            }

            foreach (var subbase in b.subbases)
            {
                UpdateOrientations(subbase);
            }
        }

        private void UpdateOrientations(Chain c)
        {
            for (int i = 0; i < c.Count - 1; i++)
            {
                var node = c[i];
                var next = c[i + 1];

                next.rotation = Quaternion.LookRotation(next.position - node.position);
            }
        }

        private void Solve()
        {
            // The solver first applies each constraint directly to the Nodes.
            // The Nodes that have active constraints applied are the origins of
            // the Forwards iterations.

            foreach (var item in constraints)
            {
                item.Project();
            }

            var rootPosition = model.root.node.position;
            var rootRotation = model.root.node.rotation;

            for (int i = 0; i < Iterations; i++)
            {
                //model.root.node.rotation = model.root.node.transform.rotation;

                UpdateOrientations(model.root);

                Forwards(model.root);

//                model.root.node.rotation = model.root.node.transform.rotation;

                UpdateOrientations(model.root);

                // Apply any fixed constraints here...
                if (FixRoot)
                {
                    model.root.node.position = rootPosition;
                    //   model.root.node.rotation = rootRotation;
                }

                Backwards(model.root);
            }

            // Clear all the updated flags

            foreach (var node in model.nodes)
            {
                node.updated = false;
            }
        }

        private void ForwardsLoop(Chain loop)
        {
            // Phase 1

            var tmp = loop.End.position; // Store the initial value of p1
            Forwards(loop); // Update inter-joints from loop base (steps 1 to 3)
            Forwards(loop, 1); // With the temporary value of p1, update p3 again (step 4)

            // Phase 2

            loop.End.position = tmp;
            Backwards(loop, loop.Count - 1); // Update the inter-joints in the other direction (steps 5 & 6)

            // Phase 3

            Forwards(loop, 1); // Step 7
            Backwards(loop, 1); // Step 9

            // Update the positions of the interjoints based on the outwards
            // facing chains (steps 8 and 10).
            // This is done by walking chains that connect to nodes in the loop.
            // If a chain is not constrained by an effector, then it is skipped.

            foreach (var item in loop.subchains)
            {
                Forwards(item);
            }

            // Steps 11 to 16

            /*
            tmp = loop.Start.position; // Store the initial value of p1
            Forwards(loop); // Update inter-joints from loop base (steps 11 to 13)
            Forwards(loop, 1); // With the temporary value of p1, update p3 again (step 14)
            loop.Start.position = tmp;
            Backwards(loop, loop.Count - 1); // Update the inter-joints in the other direction (steps 15 & 16)
            */
        }

        private void BackwardsLoop(Chain loop)
        {
            Forwards(loop); 
            Forwards(loop, 1);
            foreach (var item in loop.subchains)
            {
                Backwards(item);
            }
        }

        private void Backwards(Chain chain, int num)
        {
            for (int i = 0; i < num; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                var d = chain.d[i];
                next.position = node.position + (next.position - node.position).normalized * d;
            }
        }

        private void Forwards(Chain chain, int num)
        {
            for (int i = chain.Count - 1; i > (chain.Count - num); i--)
            {
                var node = chain[i];
                var prev = chain[i - 1];
                var d = chain.d[i - 1];
                prev.position = node.position + (prev.position - node.position).normalized * d;
            }
        }

        private void Backwards(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                var d = chain.d[i];
                
                var joint = chain.joints[i]; // joint.Backwards updates next
                if (joint != null && joint.Enabled)
                {
                    joint.Backwards(node, next, d);
                }
                else
                {
                    next.position = node.position + (next.position - node.position).normalized * d;
                }
            }
        }

        private void Forwards(Chain chain)
        {
            for (int i = chain.Count - 1; i > 0; i--)
            {
                var node = chain[i];
                var prev = chain[i - 1];
                var d = chain.d[i - 1];

                // Constrain by the joints (if any)

                var joint = chain.joints[i - 1]; // joint.Forwards updates prev
                if (joint != null && joint.Enabled)
                {
                    joint.Forwards(prev, node, d);
                }
                else
                {
                    prev.position = node.position + (prev.position - node.position).normalized * d;
                }
            }
        }

        private void CreateNodesLookup(Model model)
        {
            foreach (var node in model.nodes)
            {
                nodesByName[node.transform.name] = node;
                nodesByTransform[node.transform] = node;
            }
        }

        public Node GetEffector(string name)
        {
            return nodesByName[name];
        }

        public bool HasEffector(string name)
        {
            return nodesByName.ContainsKey(name);
        }

        public Node GetEffector(Transform transform)
        {
            return nodesByTransform[transform];
        }

        public void AddConstraint(IConstraint m)
        {
            constraints.Add(m);
        }

        public void UpdateModelFromTransforms()
        {
            foreach (var item in model.nodes)
            {
                item.position = item.transform.position;
            }
        }
    }
}