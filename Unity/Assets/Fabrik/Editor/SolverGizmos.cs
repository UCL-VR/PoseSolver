using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

using Ubiq.Fabrik;

namespace Ubiq.Fabrik.Gizmo
{
    public class FabrikGizmos
    {

        static void DrawChain(Chain chain)
        {
            for (int i = 0; i < chain.Count - 1; i++)
            {
                Gizmos.DrawLine(chain[i].position, chain[i + 1].position);
            }
        }

        [DrawGizmo(GizmoType.Selected | GizmoType.Active | GizmoType.NonSelected)]
        static void DrawGizmoSolver(FabrikSolver component, GizmoType gizmoType)
        {
            if (!Application.isPlaying)
            {
                component.MakeModel();
            }

            var model = component.model;

            foreach (var subbase in model.subbases)
            {
                Gizmos.color = Color.cyan;

                foreach (var chain in subbase.chains)
                {
                    DrawChain(chain);
                }

                foreach (var chain in subbase.loops)
                {
                    foreach (var subchain in chain.subchains)
                    {
                        DrawChain(subchain);
                    }
                }

                Gizmos.color = Color.green;

                foreach (var chain in subbase.loops)
                {
                    DrawChain(chain);
                }
            }

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

                UnityEditor.Handles.Label(item.position, item.transform.name);
            }
        }
    }
}