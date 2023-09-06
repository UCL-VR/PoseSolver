using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(CaptureStream))]
public class CaptureStreamEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as CaptureStream;

        if(!component.IsOpen)
        {
            if (GUILayout.Button("Load"))
            {
                component.Open();
            }
        }
        else
        {
            if (GUILayout.Button("Play"))
            {
                component.Play();
            }

            component.Frame = EditorGUILayout.IntSlider(component.Frame, 0, component.Frames);
        }

        EditorGUILayout.LabelField("Time", component.PlayTime.ToString());
    }

}
