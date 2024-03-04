using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using System;

[CustomEditor(typeof(Socket1D))]
public class Socket1DEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as Socket1D;

        // To implement the constrains, each joint is broken down into a fixed
        // component and one or more variable rotation components.

        // That is, the graph at design time defines the Bind Pose, and the
        // 'constrints', in a sense, actually introduce freedoms to particular
        // transforms.



        // Get the forward vector of the parent

        var parentForward = component.transform.parent ? component.transform.parent.forward : Vector3.forward;

        // The parent'



        // Problem with Swing Twist Decomposition is that it is not signed...

        var angle = component.transform.localRotation.Twist(Vector3.right);

        EditorGUILayout.LabelField("Rotation", angle.ToString());
    }

    [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
    static void DrawGizmosSocket1D(Socket1D socket, GizmoType type)
    {
        Gizmos.matrix = socket.transform.localToWorldMatrix;

        // Create the arc

        List<Vector3> points = new List<Vector3>();

        for (float i = socket.range.x; i < socket.range.y; i += 0.5f)
        {
            var p = Quaternion.AngleAxis(i, Vector3.right) * Vector3.forward * 0.01f;
            points.Add(p);
        }

        var span = new ReadOnlySpan<Vector3>(points.ToArray());

        Gizmos.DrawLineStrip(span, false);
    }

    [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
    static void DrawGizmosSocket2D(Socket2D socket, GizmoType type)
    {
        Gizmos.matrix = socket.transform.localToWorldMatrix;

        // Create the arc

        List<Vector3> points = new List<Vector3>();

        for (float i = socket.range_x.x; i < socket.range_x.y; i += 0.5f)
        {
            var p = Quaternion.AngleAxis(i, Vector3.right) * Vector3.forward * 0.01f;
            points.Add(p);
        }
        Gizmos.DrawLineStrip(new ReadOnlySpan<Vector3>(points.ToArray()), false);

        points.Clear();

        for (float i = socket.range_y.x; i < socket.range_y.y; i += 0.5f)
        {
            var p = Quaternion.AngleAxis(i, Vector3.up) * Vector3.forward * 0.01f;
            points.Add(p);
        }
        Gizmos.DrawLineStrip(new ReadOnlySpan<Vector3>(points.ToArray()), false);
    }
}
