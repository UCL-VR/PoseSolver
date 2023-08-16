using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace UCL.CASMS.Haptics
{
    [CustomEditor(typeof(DHNodesController))]
    public class DHNodesControllerEditor : Editor
    {
        private void OnEnable()
        {

        }

        public override void OnInspectorGUI()
        {
            var target = serializedObject.targetObject as DHNodesController;

            string label = "No Model";

            if(target.Model != null)
            {
                label = "Current Model (type, name): " + target.Model.GetType().Name + ": " + target.Model.Name;
            }

            EditorGUILayout.LabelField(label);

            serializedObject.Update();

            //EditorGUILayout.PropertyField(serializedObject.FindProperty("Model"));

            serializedObject.ApplyModifiedProperties();
        }
    }
}