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

    public class Node
    {
        public Transform transform;

        public Vector3 position;

        public Node(Transform t)
        {
            transform = t;
            position = t.position;
        }

        public List<Vector3> positions = new List<Vector3>();

        public List<Node> connections = new List<Node>(); // Edges

        public int index;

        public bool updated;

        // The following flags define where the Node is updated in the solver.
        // These flags are mutually exclusive.

        public bool leaf; // Sits at the end of a chain
        public bool inter; // An intermediate joint in a serial chain
        public bool subbase; // Connects two graphs that should be solved in parallel
        public bool loopinter; // Connects a loop and a serial chain

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

        // Stores the index of the position controlled by this chain in a subbase
        // for each node.
        public List<int> indices = new List<int>();

        public Node End => this.Last();
        public Node Start => this.First();

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

        public Subbase(Node n)
        {
            node = n;
            chains = new List<Chain>();
            loops = new List<Chain>();
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
    }

    public class Model
    {
        public List<Chain> chains;
        public List<Node> nodes;
        public List<Node> effectors;
        public List<Subbase> subbases;
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

        public Transform Root;
        public List<Transform> Effectors = new List<Transform>();
        public List<DistanceConstraint> Distances = new List<DistanceConstraint>();

        public Model model;

        public List<IConstraint> constraints = new List<IConstraint>();

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
        }

        public class SolveLayer
        {
            public List<Chain> chains;
            public List<Chain> loops;
            public List<Node> subbases;
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

            for (int i = 0; i < 10; i++)
            {
                foreach (var subbase in model.subbases)
                {
                    var node = subbase.node;
                    var position = node.position;

                    // Perform the backwards interation of each chain. The order is
                    // from the most distal chains of the subbases, to the least.

                    foreach (var chain in subbase.chains)
                    {
                        node.position = position;
                        ForwardsIf(chain);
                        subbase.positions.Add(node.position);
                    }

                    foreach (var loop in subbase.loops)
                    {

                        node.position = position;
                        ForwardsLoop(loop);
                        subbase.positions.Add(node.position);
                    }
                }

                // Update the sub-bases

                foreach (var subbase in model.subbases)
                {
                    subbase.UpdateSubbase();
                }

                // Apply any fixed constraints

                foreach (var subbase in model.subbases)
                {
                    // And go backwards...

                    foreach (var loop in subbase.loops)
                    {
                        BackwardsLoop(loop);
                    }

                    // And finally move through all the chains in the other direction,
                    // starting at the root.

                    foreach (var chain in subbase.chains)
                    {
                        Backwards(chain);
                    }
                }
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

            var tmp = loop.Start.position; // Store the initial value of p1
            Forwards(loop); // Update inter-joints from loop base (steps 1 to 3)
            Forwards(loop, 1); // With the temporary value of p1, update p3 again (step 4)

            // Phase 2

            loop.Start.position = tmp;
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
                ForwardsIf(item);
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

        private void Forwards(Chain chain, int num)
        {
            for (int i = 0; i < num; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                var d = chain.d[i];
                next.position = node.position + (next.position - node.position).normalized * d;
            }
        }

        private void Forwards(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                var d = chain.d[i];
                next.position = node.position + (next.position - node.position).normalized * d;
            }
        }

        private void ForwardsIf(Chain chain)
        {
            bool updated = false;

            foreach (var item in chain)
            {
                updated = item.updated;
                break;
            }

            if (!updated)
            {
                return;
            }

            for (int i = 0; i < chain.Count - 1; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                var d = chain.d[i];
                next.position = node.position + (next.position - node.position).normalized * d;
            }
        }

        private void Backwards(Chain chain, int num)
        {
            for (int i = chain.Count - 1; i > (chain.Count - num); i--)
            {
                var node = chain[i];
                var next = chain[i - 1];
                var d = chain.d[i - 1];
                next.position = node.position + (next.position - node.position).normalized * d;
            }
        }

        private void Backwards(Chain chain)
        {
            for (int i = chain.Count - 1; i > 0; i--)
            {
                var node = chain[i];
                var next = chain[i - 1];
                var d = chain.d[i - 1];
                next.position = node.position + (next.position - node.position).normalized * d;
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