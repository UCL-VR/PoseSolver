using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class SkinnedMeshColliderManager : MonoBehaviour {

    public new SkinnedMeshRenderer renderer;
    public List<MeshCollider> colliders;

    public float skinThreshold = 0.9f;

    private void Reset()
    {
        renderer = GetComponentInChildren<SkinnedMeshRenderer>();
        colliders = GetComponentsInChildren<MeshCollider>().ToList();
    }

    private void Awake()
    {
        
    }

    public void RemoveColliders()
    {
        foreach(var component in GetComponentsInChildren<MeshCollider>())
        {
            DestroyImmediate(component);
        }
    }

    public void Build()
    {
        var map = new VertexMap(renderer, skinThreshold);

        foreach (var bone in map.BoneMap)
        {
            var meshcollider = bone.Key.gameObject.GetComponent<MeshCollider>();
            if(meshcollider == null)
            {
                meshcollider = bone.Key.gameObject.AddComponent<MeshCollider>();
            }

            if(bone.Value.Length < 3)
            {
                DestroyImmediate(meshcollider);
                continue;
            }

            var mesh = new Mesh();
            mesh.vertices = bone.Value;
            mesh.SetIndices(new int[] { 1, 2, 3 }, MeshTopology.Triangles, 0);
            mesh.UploadMeshData(false);
        
            meshcollider.sharedMesh = mesh;

            meshcollider.cookingOptions = MeshColliderCookingOptions.CookForFasterSimulation | MeshColliderCookingOptions.EnableMeshCleaning | MeshColliderCookingOptions.WeldColocatedVertices;
            meshcollider.convex = true;
        }
    }

    private class VertexMap
    {
        public Dictionary<Transform, Vector3[]> BoneMap;

        public VertexMap(SkinnedMeshRenderer renderer, float threshold)
        {
            BoneMap = new Dictionary<Transform, Vector3[]>();

            var baked = new Mesh();
            renderer.BakeMesh(baked);

            // verts are relative to renderer.transform - put them in world space for later
            var vertices = baked.vertices.Select( v => renderer.gameObject.transform.TransformPoint(v) ).ToArray();

            Dictionary<int, List<Vector3>> boneIndexVertexMap = new Dictionary<int, List<Vector3>>();
            for (int i = 0; i < renderer.bones.Length; i++)
            {
                boneIndexVertexMap.Add(i, new List<Vector3>());
            }

            for (int i = 0; i < renderer.sharedMesh.boneWeights.Length; i++)
            {
                var set = renderer.sharedMesh.boneWeights[i];

                if(set.weight0 > threshold)
                {
                    boneIndexVertexMap[set.boneIndex0].Add(vertices[i]);    // found a skinned vertex! add it to the list for that bone.
                }
                if (set.weight1 > threshold)
                {
                    boneIndexVertexMap[set.boneIndex1].Add(vertices[i]);    
                }
                if (set.weight2 > threshold)
                {
                    boneIndexVertexMap[set.boneIndex2].Add(vertices[i]);    
                }
                if (set.weight3 > threshold)
                {
                    boneIndexVertexMap[set.boneIndex3].Add(vertices[i]);    
                }
            }

            // convert the array of indicies and world space vertices to array of transforms and local space vertices
            foreach (var pair in boneIndexVertexMap)
            {
                BoneMap.Add(
                    renderer.bones[pair.Key],
                    pair.Value.Select(v => renderer.bones[pair.Key].InverseTransformPoint(v)).ToArray()
                    );
            }
        }
    }

	
}
