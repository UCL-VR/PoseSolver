using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UCL.CASMS.DH;

[CustomEditor(typeof(DHJointLink))]
public class DHNodeEditor : Editor
{
    bool editPosition;

    Tool sceneTool = Tool.None;
    Tool ourTool = Tool.Move;

    void OnEnable()
    {
        sceneTool = Tools.current;
        Tools.current = Tool.None;
    }

    void OnDisable()
    {
        Tools.current = sceneTool;
    }

    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var buttonStyle = new GUIStyle(GUI.skin.button);

        editPosition = GUILayout.Toggle(editPosition, "Edit Position", buttonStyle);
        if (editPosition)
        {

        }
    }

    private void OnSceneGUI()
    {
        var component = target as DHJointLink;
       
        // Disable the in-built scene tool
        Tools.current = Tool.None;

        Repaint();
    }

    [DrawGizmo(GizmoType.Selected | GizmoType.NonSelected)]
    static void DrawGizmoForDHJointLink(DHJointLink scr, GizmoType gizmoType)
    {
        Vector3 position = scr.transform.position;

        Gizmos.matrix = scr.transform.localToWorldMatrix;

        var r = 0.005f;

        if ((gizmoType & GizmoType.Selected) != 0)
        {
            Handles.color = Color.yellow;
        }
        else
        {
            Handles.color = Color.white;
        }

        Handles.DrawWireDisc(scr.transform.position, scr.transform.up, r);

        Handles.color = Color.blue;

        Handles.DrawLine(scr.transform.position, scr.transform.position + scr.transform.forward * r);

        Handles.color = Color.white;

        Handles.DrawLine(scr.transform.position, scr.EndPosition);

        Handles.color = Color.green;

        Handles.DrawLine(scr.transform.position, scr.transform.position + scr.transform.up * 0.01f);

    }
}
