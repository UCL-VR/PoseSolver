using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UCL.CASMS;
using System;

/// <summary>
/// Re-targets the Denavit Hartenberg hand to the MakeHuman Hand Mesh with the
/// Game rig. This Component should be applied to the root of the Hand Mesh.
/// </summary>
/// <remarks>
/// The recommended usage pattern is to create a variant prefab for a particular
/// calibration.
/// </remarks>
[DefaultExecutionOrder(3)] // This should run after the solver to always have the latest data
[ExecuteInEditMode]
public class MakeHumanDenavitHartenbergHand : MonoBehaviour
{
    public GameObject Hand;

    public bool MapPreview = false;

    public bool ApplyInEditMode = false;

    private string[][] boneMap = new string[][] { 
        new string[] { "index_01", "Index", "MCPFlex" },
        new string[] { "index_02", "Index", "PIP" },
        new string[] { "index_03", "Index", "DIP" },
        new string[] { "middle_01", "Middle", "MCPFlex" },
        new string[] { "middle_02", "Middle", "PIP" },
        new string[] { "middle_03", "Middle", "DIP" },
        new string[] { "pinky_01", "Little", "MCPFlex" },
        new string[] { "pinky_02", "Little", "PIP" },
        new string[] { "pinky_03", "Little", "DIP" },
        new string[] { "ring_01", "Ring", "MCPFlex" },
        new string[] { "ring_02", "Ring", "PIP" },
        new string[] { "ring_03", "Ring", "DIP" },
        new string[] { "thumb_02", "Thumb", "MCPFlex" },
        new string[] { "thumb_03", "Thumb", "IP" },
    };

    public bool hasMap => map != null && map.Count > 0;

    [Serializable]
    private class TransformMap
    {
        public Transform bone;
        public Transform joint;
        public Vector3 relativePosition;
        public Quaternion relativeRotation;

        public TransformMap(Transform bone, Transform joint)
        {
            this.bone = bone;
            this.joint = joint;
            relativePosition = joint.InverseTransformPoint(bone.position);
            relativeRotation = Quaternion.Inverse(joint.rotation) * bone.rotation;
        }

        public void Apply()
        {
            bone.position = joint.TransformPoint(relativePosition);
            bone.rotation = joint.rotation * relativeRotation;
        }
    }

    [SerializeField]
    [HideInInspector]
    private List<TransformMap> map;

    public void BuildTransformMap()
    {
        map = new List<TransformMap>();
        map.Add(new TransformMap(transform, Hand.transform));
        foreach (var bMap in boneMap)
        {
            map.Add(new TransformMap(FindMeshBone(bMap), FindHandJoint(bMap)));
        }
    }

    public void ApplyTransformMap()
    {
        foreach (var m in map)
        {
            m.Apply();
        }
    }

    void Start()
    {
        if (!hasMap)
        {
            BuildTransformMap();
        }
    }

    void Update()
    {
        if (Application.isPlaying || ApplyInEditMode)
        {
            ApplyTransformMap();
        }
    }

    private Transform FindMeshBone(string[] parms)
    {
        return transform.FindDeepPartial(parms[0]);
    }

    private Transform FindHandJoint(string[] parms)
    {
        // The first parameter finds the chain, then the second the individual bone
        return Hand.transform.Find(parms[1]).FindDeepPartial(parms[2]);
    }

    public void SnapHandBonesToJoints()
    {
        foreach (var map in boneMap)
        {
            var bone = FindMeshBone(map);
            var joint = FindHandJoint(map);
            bone.position = joint.position;
            bone.up = joint.forward; // This change in coordinate system is inherent to the MakeHuman hand model
        }
    }

    private void OnDrawGizmos()
    {
        // Draw all the relevant joints
      
        foreach (var map in boneMap)
        {
            var handTransform = FindMeshBone(map);
            var dhTransform = FindHandJoint(map);

            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(handTransform.position, 0.001f);

            Gizmos.color = Color.red;
            Gizmos.DrawLine(handTransform.position, dhTransform.position);
        }

    }
}
