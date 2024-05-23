using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MakeHumanDenavitHartenbergHand))]
public class MakeHumanDenavitHartenbergHandEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        var component = target as MakeHumanDenavitHartenbergHand;

        if(GUILayout.Button("Build"))
        {
            Undo.RecordObject(target, "Updated Transforms Map");
            component.BuildTransformMap();
        }

        if(component.hasMap)
        {
            GUILayout.Label("Ready");
        }
    }
}
