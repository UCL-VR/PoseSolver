using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(PerformanceProfiler))]
public class PerformanceProfilerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if(GUILayout.Button("Capture"))
        {
            var expected = System.IO.Path.Combine(Application.dataPath, "..");
            var filename = EditorUtility.SaveFilePanel("Save Capture", expected, "timings", "csv");
            if (!string.IsNullOrEmpty(filename))
            {
                (target as PerformanceProfiler).Capture(filename);
            }
        }
    }
}
