using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MakeHumanHand4))]
public class MakeHumanHand4Editor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        var component = target as MakeHumanHand4;

        if(GUILayout.Button("Build Map"))
        {
            Undo.RecordObject(target, "Updated Transforms Map");
            component.BuildTransformMap();
        }

        if (component.hasMap)
        {
            GUILayout.Label("Ready");
        }
    }
}
