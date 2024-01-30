using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MotionFrameHelper))]
public class MotionFrameHelperEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as MotionFrameHelper;

        var frame = component.Frame;

        EditorGUILayout.Vector3Field("Position", frame.position);
        //EditorGUILayout.Vector4Field("Rotation", frame.rotation);
        EditorGUILayout.Vector3Field("Acceleration", frame.acceleration);
    }
}
