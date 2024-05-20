using System.Collections;
using System.Collections.Generic;
using UCL.CASMS;
using Ubiq.Fabrik;
using UnityEngine;
using System;

[Serializable]
public class SplitTransformMap
{
    public Transform child;
    public Transform position;
    public Transform rotation;
    public Vector3 relativePosition;
    public Quaternion relativeRotation;

    public SplitTransformMap(Transform child, Transform position, Transform rotation)
    {
        this.child = child;
        this.position = position;
        this.rotation = rotation;
        relativePosition = Quaternion.Inverse(rotation.rotation) * (child.position - position.position);
        relativeRotation = Quaternion.Inverse(rotation.rotation) * child.rotation;
    }

    public void Apply()
    {
        child.position = (rotation.rotation * relativePosition) + position.position;
        child.rotation = rotation.rotation * relativeRotation;
    }
}

public class MakeHumanHand4 : MonoBehaviour
{
    public FabrikSolver Hand;

    public bool hasMap => map != null && map.Count > 0;
    public bool ApplyInEditMode = false;

    [SerializeField]
    [HideInInspector]
    protected List<SplitTransformMap> map;

    private string[][] positionsMap = new string[][]
    {
        new string[] { "index_01", "Index1" },
        new string[] { "index_02", "Index2" },
        new string[] { "index_03", "Index3" },
        new string[] { "middle_01", "Middle1"},
        new string[] { "middle_02", "Middle2" },
        new string[] { "middle_03", "Middle3" },
        new string[] { "pinky_01", "Little1" },
        new string[] { "pinky_02", "Little2" },
        new string[] { "pinky_03", "Little3" },
        new string[] { "ring_01", "Ring1" },
        new string[] { "ring_02", "Ring2" },
        new string[] { "ring_03", "Ring3" },
        new string[] { "thumb_01", "Thumb1" },
        new string[] { "thumb_02", "Thumb3" },
        new string[] { "thumb_03", "Thumb4" },
    };

    private string[][] rotationsMap = new string[][]
{
        new string[] { "index_01", "Index2" },
        new string[] { "index_02", "Index3" },
        new string[] { "index_03", "IndexTip" },
        new string[] { "middle_01", "Middle2"},
        new string[] { "middle_02", "Middle3" },
        new string[] { "middle_03", "MiddleTip" },
        new string[] { "pinky_01", "Little2" },
        new string[] { "pinky_02", "Little3" },
        new string[] { "pinky_03", "LittleTip" },
        new string[] { "ring_01", "Ring2" },
        new string[] { "ring_02", "Ring3" },
        new string[] { "ring_03", "RingTip" },
        new string[] { "thumb_01", "Thumb3" },
        new string[] { "thumb_02", "Thumb4" },
        new string[] { "thumb_03", "ThumbTip" },
};

    public void BuildTransformMap()
    {
        map = new List<SplitTransformMap>();
        map.Add(new SplitTransformMap(transform, Hand.transform, Hand.transform));

        for (int i = 0; i < positionsMap.Length; i++)
        {
            var bone = positionsMap[i][0];
            var p = positionsMap[i][1];
            var r = rotationsMap[i][1];
            map.Add(new SplitTransformMap(FindMeshBone(bone), FindHandJoint(p), FindHandJoint(r)));
        }
    }

    private Transform FindMeshBone(string name)
    {
        return transform.FindDeepPartial(name);
    }

    private Transform FindHandJoint(string name)
    {
        return Hand.transform.FindDeepPartial(name);
    }

    // Update is called once per frame
    void Update()
    {
        if (hasMap)
        {
            if (Application.isPlaying || ApplyInEditMode)
            {
                foreach (var m in map)
                {
                    m.Apply();
                }

                // Rotate thumb3 45 degrees around the y axis as a fix
                //var b1 = FindMeshBone("thumb_02");
                //b1.RotateAround(b1.position, b1.up, -90f);
                //var b2 = FindMeshBone("thumb_03");
                //b2.RotateAround(b2.position, b1.up, 90f);
            }
        }
    }
}
