using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Ubiq.Fabrik;

namespace Ubiq.Fabrik.Gizmo
{
    public class FabrikGizmos
    {
        static void DrawEdges(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                Gizmos.DrawLine(chain[i].position, chain[i + 1].position);
            }
        }

        static void DrawEdges(Subbase subbase)
        {
            Gizmos.color = Color.cyan;

            foreach (var chain in subbase.chains)
            {
                DrawEdges(chain);
            }

            foreach (var chain in subbase.loops)
            {
                foreach (var subchain in chain.subchains)
                {
                    DrawEdges(subchain);
                }
            }

            Gizmos.color = Color.green;

            foreach (var chain in subbase.loops)
            {
                DrawEdges(chain);
            }

            foreach (var sb in subbase.subbases)
            {
                DrawEdges(sb);
            }
        }

        static void DrawJoints(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                if (chain.joints[i] != null && chain.joints[i].Enabled)
                {
                    chain.joints[i].DrawGizmos(chain[i]);
                }
            }
        }

        static void DrawJoints(Subbase subbase)
        {
            Gizmos.color = Color.cyan;

            foreach (var chain in subbase.chains)
            {
                DrawJoints(chain);
            }

            Gizmos.color = Color.green;

            foreach (var sb in subbase.subbases)
            {
                DrawJoints(sb);
            }
        }

        [DrawGizmo(GizmoType.Selected | GizmoType.Active | GizmoType.NonSelected)]
        static void DrawGizmo(FabrikSolver component, GizmoType gizmoType)
        {
            if (!Application.isPlaying)
            {
                component.MakeModel();
            }

            var model = component.model;

            DrawEdges(model.root);

            // Draw the Nodes and their types

            foreach (var item in model.nodes)
            {
                if (item.inter)
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawSphere(item.position, 0.001f); // Unity bugs mean we need this call right after setting the colour
                }
                if (item.leaf)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(item.position, 0.001f);
                }
                if (item.subbase)
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawSphere(item.position, 0.001f);
                }
                if (item.loopinter)
                {
                    Gizmos.color = Color.green;
                    Gizmos.DrawSphere(item.position, 0.001f);
                }

                // Draw the orientations of the nodes

                float size = 0.001f;

                Gizmos.color = Color.red;
                Gizmos.DrawLine(item.position, item.position + item.right * size);

                Gizmos.color = Color.green;
                Gizmos.DrawLine(item.position, item.position + item.up * size); 

                Gizmos.color = Color.blue;
                Gizmos.DrawLine(item.position, item.position + item.forwards * size);

                UnityEditor.Handles.Label(item.position, item.transform.name);
            }

            DrawJoints(model.root);
        }
    }
}