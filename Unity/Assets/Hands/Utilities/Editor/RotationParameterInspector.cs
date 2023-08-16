using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace UCL.CASMS.Haptics
{
    [CustomEditor(typeof(RotationParameter))]
    public class RotationParameterInspector : Editor
    {
        protected enum Limit
        {
            Min,
            Max,
            None
        }

        Limit editing = Limit.Min;

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.PropertyField(serializedObject.FindProperty("value"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("min"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("max"));

            serializedObject.ApplyModifiedProperties();

            editing = (Limit)EditorGUILayout.EnumPopup("Editing", editing);
        }

        void OnEnable()
        {
            Tools.hidden = true;
        }

        void OnDisable()
        {
            Tools.hidden = false;
        }


        // Use this for initialization
        private void OnSceneGUI()
        {
            var parameter = (target as RotationParameter);

            EditorGUI.BeginChangeCheck();

            Quaternion newvalue = Quaternion.identity;

            switch (editing)
            {
                case Limit.Min:
                    newvalue = parameter.min;
                    break;
                case Limit.Max:
                    newvalue = parameter.max;
                    break;
            }

            // to world

            newvalue = parameter.ParentRotation * newvalue;

            newvalue = Handles.RotationHandle(newvalue, parameter.transform.position);

            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(target, "Changed RotationParameter Limit");

                // to local

                if(parameter.transform.parent != null)
                {
                    newvalue = Quaternion.Inverse(parameter.ParentRotation) * newvalue;
                }

                switch (editing)
                {
                    case Limit.Min:
                        parameter.min = newvalue;
                        break;
                    case Limit.Max:
                        parameter.max = newvalue;
                        break;
                }

                parameter.Update();
            }
        }
    }
}