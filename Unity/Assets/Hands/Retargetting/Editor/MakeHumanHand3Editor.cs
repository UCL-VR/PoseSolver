using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MakeHumanHand3))]
public class MakeHumanHand3Editor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        var component = target as MakeHumanHand3;

        if(GUILayout.Button("Build Map"))
        {
            component.BuildTransformMap();
        }

        if (component.hasMap)
        {
            GUILayout.Label("Ready");
        }
    }
}
