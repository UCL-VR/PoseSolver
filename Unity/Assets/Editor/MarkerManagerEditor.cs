using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MarkerManager))]
public class MarkerManagerEditor : Editor
{
    struct PositionRotation
    {
        public Vector3 Position;
        public Quaternion Rotation;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        if (GUILayout.Button("Reset Transform"))
        {
            var transforms = new Dictionary<Transform, PositionRotation>();

            var component = target as MarkerManager;

            foreach(Transform child in component.transform)
            {
                transforms[child] = new PositionRotation()
                {
                    Position = child.transform.position,
                    Rotation = child.transform.rotation
                };
            }

            component.transform.position = Vector3.zero;
            component.transform.rotation = Quaternion.identity;

            foreach (var item in transforms)
            {
                item.Key.transform.position = item.Value.Position;
                item.Key.transform.rotation = item.Value.Rotation;
            }

        }
    }
}