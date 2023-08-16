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

        if(GUILayout.Button("Play"))
        {
            component.Play();
        }
    }

}
