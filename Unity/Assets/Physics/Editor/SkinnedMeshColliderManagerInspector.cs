using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(SkinnedMeshColliderManager))]
public class SkinnedMeshColliderManagerInspector : Editor {

    public override void OnInspectorGUI()
    {
        var target = serializedObject.targetObject as SkinnedMeshColliderManager;

        serializedObject.Update();

        EditorGUILayout.PropertyField(serializedObject.FindProperty("skinThreshold"));

        serializedObject.ApplyModifiedProperties();

        if(GUILayout.Button("Build"))
        {
            target.Build();
        }

        if (GUILayout.Button("Destroy Colliders"))
        {
            target.RemoveColliders();
        }
    }
}
