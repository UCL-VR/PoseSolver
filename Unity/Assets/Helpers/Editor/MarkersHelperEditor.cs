using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(MarkersHelper))]
public class MarkersHelperEditor : Editor
{
    private void OnSceneGUI()
    {
        var component = target as MarkersHelper;

        var tmp = Vector3.zero;
        var i = 0f;

        foreach (var item in component.GetComponentsInChildren<ImuMarker>())
        {
            tmp += item.transform.position;
            i++;
        }

        tmp /= i;

        if (Event.current.button == 1)
        {
            SceneView.lastActiveSceneView.pivot = tmp;
        }

        var offset = component.transform.position - tmp;

        Vector3 position = tmp;
        Quaternion rotation = Quaternion.identity;
        Handles.TransformHandle(ref position, ref rotation);

        component.transform.position = position + offset;

        Vector3 axis;
        float angle;
        rotation.ToAngleAxis(out angle, out axis);

        component.transform.RotateAround(position, axis, angle);
    }
}
