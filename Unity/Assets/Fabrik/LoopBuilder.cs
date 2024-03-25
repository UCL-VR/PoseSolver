using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public class LoopHelper : IEquatable<LoopHelper>
    {
        public List<Node> nodes;
        public int[] forward;
        public int[] backwards;

        public LoopHelper(List<Node> nodes)
        {
            this.nodes = nodes;
            forward = MakeHashArray(nodes);
            backwards = MakeHashArray(nodes.Reverse<Node>());
        }

        int[] MakeHashArray(IEnumerable<Node> nodes)
        {
            var f = nodes.Select(n => n.index).ToArray();
            var m = f.Min();
            while (f[0] != m)
            {
                Shift(f);
            }
            return f;
        }

        void Shift(int[] a)
        {
            var tmp = a[0];
            for (int i = 0; i < a.Length - 1; i++)
            {
                a[i] = a[i + 1];
            }
            a[a.Length - 1] = tmp;
        }

        public bool Equals(LoopHelper other)
        {
            return this == other;
        }

        public override bool Equals(object obj)
        {
            return this == (LoopHelper)obj;
        }

        public override int GetHashCode()
        {
            return this.forward.Sum();
        }

        public static bool operator ==(LoopHelper a, LoopHelper b)
        {
            return
                a.forward.SequenceEqual(b.forward)
                || a.backwards.SequenceEqual(b.backwards)
                || a.forward.SequenceEqual(b.backwards)
                || a.backwards.SequenceEqual(b.forward);
        }

        public static bool operator !=(LoopHelper a, LoopHelper b)
        {
            return !(a == b);
        }

        public Chain ToChain()
        {
            var chain = new Chain();
            chain.AddRange(nodes);
            chain.Add(nodes.First());
            chain.UpdateLengths();
            chain.Loop = true;
            return chain;
        }

        public static IEnumerable<LoopHelper> FindLoops(IEnumerable<Node> nodes)
        {
            List<LoopHelper> loopBuilders = new List<LoopHelper>();
            foreach (var n in nodes.Where(n => n.connections.Count > 1))
            {
                foreach (var loop in LoopHelper.FindLoops(n))
                {
                    if (!loopBuilders.Contains(loop))
                    {
                        loopBuilders.Add(loop);
                    }
                }
            }
            return loopBuilders;
        }

        private static List<LoopHelper> FindLoops(Node knot)
        {
            List<LoopHelper> loops = new List<LoopHelper>();
            foreach (var next in knot.connections)
            {
                var nodes = FindLoop(knot, knot, next, new List<Node>());
                if (nodes != null)
                {
                    var loop = new LoopHelper(nodes);
                    loops.Add(loop);
                }
            }
            return loops;
        }

        private static List<Node> FindLoop(Node knot, Node prev, Node node, List<Node> route)
        {
            // Perform a Depth First Traversal and check each node against
            // the starting one ('knot')

            var history = new List<Node>(route);
            history.Add(node);

            foreach (var edge in node.connections)
            {
                if (edge == prev)
                {
                    // This is a bidirectional graph, so don't search backwards
                    continue;
                }

                // We are only interested in true loops. If the search reaches
                // somewhere its been previously before the knot, then abandon
                // this route.

                if (route.Contains(edge))
                {
                    return null;
                }

                if (edge == knot) // There is an edge back to knot, so this node closes the loop
                {
                    history.Add(knot);
                    return history;
                }

                var loop = FindLoop(knot, node, edge, history);
                if (loop != null)
                {
                    return loop;
                }
            }

            return null;
        }
    }
}