using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(SamplesExport))]
public class SamplesExportEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as SamplesExport;

        if (GUILayout.Button("Capture"))
        {
            var expected = System.IO.Path.Combine(Application.dataPath, "..");
            var filename = EditorUtility.SaveFilePanel("Save Samples", expected, "markers", "csv");

            if (!string.IsNullOrEmpty(filename))
            {
                component.Capture(filename);
            }
        }
    }
}
