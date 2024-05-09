using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(Hand3Helper))]
public class Hand3HelperEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as Hand3Helper;

        if(GUILayout.Button("Initialise from DH"))
        {
            component.ImportFromDHParameters();
        }
    }

    [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
    static void DrawGizmos(Hand3Helper hand, GizmoType type)
    {
        
        
        Gizmos.matrix = Matrix4x4.identity;
        DrawChain(hand.transform);
    }

    static void DrawChain(Transform start)
    {
        for (int i = 0; i < start.childCount; i++)
        {
            var child = start.GetChild(i);
            Gizmos.DrawLine(start.position, child.position);
            DrawChain(child);
        }
    }
}
