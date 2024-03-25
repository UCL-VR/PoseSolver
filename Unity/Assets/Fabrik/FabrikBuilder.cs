using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public static class ListExtensions
    {
        public static void AddUnique<T>(this List<T> list, T node)
        {
            if(!list.Contains(node))
            {
                list.Add(node);
            }
        }
    }

    public class ModelBuilder
    {
        private List<Transform> effectors;
        private List<DistanceConstraint> distances;
        private Dictionary<Transform, Node> nodes;
        private Dictionary<Node, Subbase> subbases = new Dictionary<Node, Subbase>();
        private Dictionary<Node, List<Chain>> nodeToChains = new Dictionary<Node, List<Chain>>();
        private Model model;
        private Transform root;

        public ModelBuilder(List<Transform> effectors, List<DistanceConstraint> distances, Transform root)
        {
            this.effectors = new List<Transform>(effectors.Where(e => e != null));
            this.distances = new List<DistanceConstraint>(distances.Where(d => d.from != null && d.to != null));
            this.nodeToChains = new Dictionary<Node, List<Chain>>();
            this.root = root;
        }

        private Node GetCreateNode(Transform t)
        {
            if (!nodes.ContainsKey(t))
            {
                var node = new Node(t);
                nodes.Add(node.transform, node);

                model.nodes.Add(node);
                nodeToChains.Add(node, new List<Chain>());
            }
            return nodes[t];
        }

        private Subbase Subbase(Node n)
        {
            if(!subbases.ContainsKey(n))
            {
                var sb = new Subbase(n);
                subbases.Add(n, sb);
            }
            return subbases[n];
        }

        public Model Build()
        {
            model = new Model();
            model.nodes = new List<Node>();

            nodes = new Dictionary<Transform, Node>();

            // Build a graph of all the connections within the Model. The
            // number of edges will determine the type of Node.

            foreach (var effector in effectors)
            {
                var to = GetCreateNode(effector);
                do
                {
                    var from = GetCreateNode(to.transform.parent);
                    to.connections.AddUnique(from);
                    from.connections.AddUnique(to);
                    to = from;
                } while (to.transform != root);
            }

            foreach (var constraint in distances)
            {
                var to = GetCreateNode(constraint.to);
                var from = GetCreateNode(constraint.from);
                to.connections.AddUnique(from);
                from.connections.AddUnique(to);
            }

            // For each of the nodes, store the joints associated with it from
            // the scene graph too for later.

            foreach (var node in model.nodes)
            {
                node.joints[node] = null;

                foreach (var joint in node.transform.GetComponents<Joint>().Where(j => j.enabled))
                {
                    if (joint.Next)
                    {
                        node.joints[nodes[joint.Next]] = joint;
                    }
                    else
                    {
                        node.joints[node] = joint;
                    }
                }
            }

            // Update the type of node based on the edge count

            int i = 0;
            foreach (var node in model.nodes)
            {
                if (node.connections.Count == 1)
                {
                    node.leaf = true;
                }
                if (node.connections.Count == 2)
                {
                    node.inter = true;
                }
                if (node.connections.Count > 2)
                {
                    node.subbase = true;
                }
                node.index = i++;
            }

            if (root)
            {
                nodes[root].subbase = true;
                nodes[root].inter = false;
                nodes[root].root = true;
            }

            // Find all loops in the model. While in theory we can find and express
            // any size loop, the solver only supports three-edge loops.

            var loopBuilders = LoopHelper.FindLoops(model.nodes);
            List<Chain> loops = new List<Chain>(loopBuilders.Select(x => x.ToChain()));

            if (loops.Count > 0)
            {
                Debug.LogWarning("Loops detected in model. Loops are not properly supported.");
            }

            // Now build the chains. Chains are solved in a specific order
            // to ensure that loops and subbases are correctly estimated.

            // The list of nodes in chains goes from the outside towards the
            // root. So the start of the chain is the effector and the end
            // is the subbase.

            List<Node> knownEdges = new List<Node>();
            List<Subbase> knownSubbases = new List<Subbase>();

            // Collect all the chains emanating from leaf nodes

            var nodesToCheck = model.nodes.Where(n => n.leaf);

            do
            {
                List<Chain> chains = new List<Chain>();
                foreach (var n in nodesToCheck)
                {
                    foreach (var c in n.connections)
                    {
                        // Check that we have not already encountered the edge,
                        // as the connection lists are bidirectional.

                        if (knownEdges.Contains(c))
                        {
                            continue;
                        }

                        var chain = CreateChain(n, c);

                        // If a chain terminates at a loop, update the flags for
                        // that node to indicate it is part of the loop, and add
                        // the chain.

                        bool isPartOfLoop = false;

                        foreach (var loop in loops)
                        {
                            if (loop.Contains(chain.Start))
                            {
                                chain.Start.subbase = false;
                                chain.Start.loopinter = true;
                                loop.subchains.Add(chain);
                                isPartOfLoop = true;
                            }
                        }

                        if (!isPartOfLoop)
                        {
                            chains.Add(chain);
                        }
                    }
                }

                // Rotate the loops so that they start and end on subbases

                foreach (var item in loops)
                {
                    if(!item.First().subbase && item.Any(n => n.subbase))
                    {
                        item.RemoveAt(item.Count - 1);
                        while(!item.First().subbase)
                        {
                            Shift(item);
                        }
                        item.Add(item.First());
                        item.UpdateLengths();
                    }
                }

                // Chains cannot end at the root, because this is where all
                // chains must originate from.

                for (int c = chains.Count - 1; c >= 0; c--)
                {
                    if (chains[c].End.root)
                    {
                        chains.RemoveAt(c);
                    }
                }

                // For all chains and loops, find if they terminate at a subbase,
                // and if so add them as dependencies of that subbase.

                List<Node> nextNodes = new List<Node>();

                foreach (var chain in chains)
                {
                    if (chain.Start.subbase)
                    {
                        var sb = Subbase(chain.Start);
                        sb.chains.Add(chain);

                        // If the chain ends at a subbase, then add that
                        // subbase as a dependency as well.

                        if(chain.End.subbase)
                        {
                            sb.subbases.Add(Subbase(chain.End));
                        }

                        if(!knownSubbases.Contains(sb))
                        {
                            knownSubbases.Add(sb);
                            nextNodes.Add(sb.node);
                        }
                    }

                    foreach (var node in chain)
                    {
                        knownEdges.Add(node);
                    }
                }

                foreach (var loop in loops)
                {
                    foreach (var node in loop)
                    {
                        if (node.subbase)
                        {
                            var sb = Subbase(node);
                            if(!sb.loops.Contains(loop))
                            {
                                sb.loops.Add(loop);
                            }

                            if (!knownSubbases.Contains(sb))
                            {
                                knownSubbases.Add(sb);
                                nextNodes.Add(sb.node);
                            }
                        }

                        knownEdges.Add(node);
                    }
                }
                loops.Clear();

                nodesToCheck = nextNodes.Distinct();

            } while (nodesToCheck.Count() > 0);

            model.root = Subbase(GetCreateNode(root));

            SetOrientations(model);

            return model;
        }

        private void SetOrientations(Model model)
        {
            SetOrientations(model.root);
        }

        private void SetOrientations(Subbase subbase)
        {
            foreach (var item in subbase.subbases)
            {
                SetOrientations(item);
            }

            foreach (var item in subbase.chains)
            {
                SetOrientations(item);
            }
        }

        private void SetOrientations(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                var node = chain[i];
                var next = chain[i + 1];
                node.rotation = Quaternion.LookRotation(next.position - node.position);
            }
            chain[chain.Count - 1].rotation = chain[chain.Count - 2].rotation;
        }

        /// <summary>
        /// Creates a single chain emanating from the Node start, in the
        /// direction of Node next (where start is an inter-node). The chain
        /// will terminate when it reaches a subbase, or an end node.
        /// </summary>
        private Chain CreateChain(Node end, Node next)
        {
            var chain = new Chain();
            chain.Add(end);
            while (true)
            {
                chain.Add(next);

                if (next.inter)
                {
                    next = next.Next(chain[chain.Count - 2]);
                }
                else
                {
                    break;
                }
            }
            chain.Reverse();

            chain.UpdateLengths();

            // Add the joints. We only support one joint per node per chain for now.

            for (int i = 0; i < chain.Count - 1; i++)
            {
                if(chain[i].joints.ContainsKey(chain[i+1]))
                {
                    chain.joints.Add(chain[i].joints[chain[i + 1]]);
                }
                else
                {
                    chain.joints.Add(null);
                }
            }
            chain.joints.Add(chain.Last().joints[chain.Last()]);

            return chain;
        }

        /// <summary>
        /// Performs a circular shift of the array in-place
        /// </summary>
        void Shift(List<Node> a)
        {
            var tmp = a[0];
            for (int i = 0; i < a.Count - 1; i++)
            {
                a[i] = a[i + 1];
            }
            a[a.Count - 1] = tmp;
        }        
    }
}