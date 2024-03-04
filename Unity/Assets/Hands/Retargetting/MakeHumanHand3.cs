using System.Collections;
using System.Collections.Generic;
using UCL.CASMS;
using UnityEngine;

public class MakeHumanHand3 : RigidTransformMap
{
    public GameObject Hand;

    private string[][] boneMap = new string[][]
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
        new string[] { "thumb_01", "Thumb2" },
        new string[] { "thumb_02", "Thumb3" },
        new string[] { "thumb_03", "Thumb4" },
    };

    public void BuildTransformMap()
    {
        map = new List<TransformMap>();
        map.Add(new TransformMap(transform, Hand.transform));
        foreach (var bMap in boneMap)
        {
            map.Add(new TransformMap(FindMeshBone(bMap), FindHandJoint(bMap)));
        }
    }

    private Transform FindMeshBone(string[] parms)
    {
        return transform.FindDeepPartial(parms[0]);
    }

    private Transform FindHandJoint(string[] parms)
    {
        // The first parameter finds the chain, then the second the individual bone
        return Hand.transform.FindDeepPartial(parms[1]);
    }

    // Update is called once per frame
    void Update()
    {
        if (hasMap)
        {
            if (Application.isPlaying || ApplyInEditMode)
            {
                ApplyTransformMap();
            }
        }
    }
}
