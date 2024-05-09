using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class TransformMap
{
    public Transform child;
    public Transform parent;
    public Vector3 relativePosition;
    public Quaternion relativeRotation;

    public TransformMap(Transform bone, Transform joint)
    {
        this.child = bone;
        this.parent = joint;
        relativePosition = joint.InverseTransformPoint(bone.position);
        relativeRotation = Quaternion.Inverse(joint.rotation) * bone.rotation;
    }

    public void Apply()
    {
        child.position = parent.TransformPoint(relativePosition);
        child.rotation = parent.rotation * relativeRotation;
    }
}

[DefaultExecutionOrder(3)] // This should run after the solver to always have the latest data
[ExecuteInEditMode]
public class RigidTransformMap : MonoBehaviour
{
    public bool ApplyInEditMode = false;

    [SerializeField]
    [HideInInspector]
    protected List<TransformMap> map;

    public bool hasMap => map != null && map.Count > 0;

    public void ApplyTransformMap()
    {
        foreach (var m in map)
        {
            m.Apply();
        }
    }
}
