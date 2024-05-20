using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace Ubiq.Fabrik
{
    [CustomEditor(typeof(FabrikSolver))]
    public class FabrikSolverEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            if (Application.isEditor && !Application.isPlaying)
            {
                if (GUILayout.Button("Apply Model To Transforms"))
                {
                    var component = target as FabrikSolver;
                    component.UpdateTransformsFromModel();
                }
            }
        }
    }
}