using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public class ModelBuilder
    {
        private List<Transform> effectors;
        private List<DistanceConstraint> distances;
        private Dictionary<Transform, Node> nodes;
        private Dictionary<Node, Subbase> subbases = new Dictionary<Node, Subbase>();
        private Model model;
        private Transform root;

        public ModelBuilder(List<Transform> effectors, List<DistanceConstraint> distances, Transform root)
        {
            this.effectors = new List<Transform>(effectors.Where(e => e != null));
            this.distances = new List<DistanceConstraint>(distances.Where(d => d.from != null && d.to != null));
            this.root = root;
        }

        private Node GetCreateNode(Transform t)
        {
            if (!nodes.ContainsKey(t))
            {
                var node = new Node(t);
                nodes.Add(node.transform, node);
                model.nodes.Add(node);
            }
            return nodes[t];
        }

        private Subbase Subbase(Node n)
        {
            if(!subbases.ContainsKey(n))
            {
                var sb = new Subbase(n);
                subbases.Add(n, sb);
                model.subbases.Add(sb);
            }
            return subbases[n];
        }

        public Model Build()
        {
            model = new Model();
            model.chains = new List<Chain>();
            model.nodes = new List<Node>();
            model.effectors = new List<Node>();
            model.subbases = new List<Subbase>();

            nodes = new Dictionary<Transform, Node>();

            // Build a graph of all the connections within the Model. The
            // number of edges will determine the type of Node.

            foreach (var effector in effectors)
            {
                var to = GetCreateNode(effector);
                do
                {
                    var from = GetCreateNode(to.transform.parent);

                    to.connections.Add(from);
                    from.connections.Add(to);

                    to = from;
                } while (to.transform != root);
            }

            foreach (var constraint in distances)
            {
                var to = GetCreateNode(constraint.to);
                var from = GetCreateNode(constraint.from);

                to.connections.Add(from);
                from.connections.Add(to);
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
            }

            // Find all loops in the model. While in theory we can find and express
            // any size loop, the solver only supports three-edge loops.

            var loopBuilders = LoopHelper.FindLoops(model.nodes);
            List<Chain> loops = new List<Chain>(loopBuilders.Select(x => x.ToChain()));

            // Now build the chains. Chains are solved in a specific order
            // to ensure that loops and subbases are correctly estimated.

            // The list of nodes in chains goes from the outside towards the
            // root. So the start of the chain is the effector and the end
            // is the subbase.

            List<Node> knownEdges = new List<Node>();

            // Collect all the chains emanating from leaf nodes

            var nodesToCheck = model.nodes.Where(n => n.leaf);

            do
            {
                List<Chain> chains = new List<Chain>();
                foreach (var n in nodesToCheck)
                {
                    foreach (var c in n.connections)
                    {
                        // Check that the edge is not considered by an existing chain or loop

                        if (knownEdges.Contains(c))
                        {
                            continue;
                        }

                        var chain = CreateChain(n, c);

                        // If a chain terminates at a loop, update the flags for that node
                        // to indicate it is part of the loop, and add the chain.

                        bool isPartOfLoop = false;

                        foreach (var loop in loops)
                        {
                            if (loop.Contains(chain.End))
                            {
                                chain.End.subbase = false;
                                chain.End.loopinter = true;
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

                // For all chains and loops, find if they terminate at a subbase,
                // and if so add them as dependencies.
                List<Node> nextNodes = new List<Node>();

                foreach (var chain in chains)
                {
                    if (chain.End.subbase)
                    {
                        Subbase(chain.End).chains.Add(chain);
                        nextNodes.Add(chain.End);
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
                            nextNodes.Add(node);
                        }

                        knownEdges.Add(node);
                    }
                }
                loops.Clear();

                nodesToCheck = nextNodes.Distinct();

            } while (nodesToCheck.Count() > 0);

            return model;
        }

        /// <summary>
        /// Creates a single chain emanating from the Node start, in the
        /// direction of Node next (where start is an inter-node). The chain
        /// will terminate when it reaches a subbase, or an end node.
        /// </summary>
        private Chain CreateChain(Node start, Node next)
        {
            var chain = new Chain();
            chain.Add(start);
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

            chain.UpdateLengths();

            return chain;
        }

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