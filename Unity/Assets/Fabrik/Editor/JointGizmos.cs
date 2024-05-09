using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using Ubiq.Fabrik;

namespace Ubiq.Fabrik.Gizmo
{
    public class JointGizmos
    {
       // [DrawGizmo(GizmoType.Selected | GizmoType.Active | GizmoType.NonSelected)]
        static void DrawGizmo(FabrikJoint component, GizmoType gizmoType)
        {
            //var hinge = component.Settings;
            //DrawGizmo(component.transform, hinge);
        }

        static void DrawGizmo(Transform node, HingeJoint settings)
        {
            Gizmos.color = Color.yellow;
           // Gizmos.DrawLine(node.position, node.position + settings.Hinge);


        }
    }
}