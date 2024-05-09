using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace UCL.CASMS.Haptics
{
    [CustomEditor(typeof(DHModelController), true, isFallback = true)]
    public class DHModelControllerEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DHModelController node = (DHModelController)target;

            if (GUILayout.Button("Reset"))
            {
                node.ResetModel();
            }
        }
    }
}