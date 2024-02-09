using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(PoseSolver), true)]
public class SolverEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();
        var component = target as PoseSolver;
        EditorGUILayout.HelpBox($"Version {component.Version}", MessageType.Info);
    }
}
