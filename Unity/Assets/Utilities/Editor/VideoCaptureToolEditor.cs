using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(VideoCaptureTool))]
public class VideoCaptureToolEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as VideoCaptureTool;

        if (GUILayout.Button("Capture"))
        {
            var expected = System.IO.Path.Combine(Application.dataPath, "..", "Workspace");
            var filename = EditorUtility.SaveFilePanel("Save Capture", expected, "capture", "mp4");

            if (!string.IsNullOrEmpty(filename))
            {
                component.Capture(filename);
            }
        }
    }
}
