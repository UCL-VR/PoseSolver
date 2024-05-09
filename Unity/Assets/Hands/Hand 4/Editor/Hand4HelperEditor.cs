using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Ubiq.Fabrik;

[CustomEditor(typeof(Hand4Helper))]
public class Hand4HelperEditor : Editor
{
    public override void OnInspectorGUI()
    {
        base.OnInspectorGUI();

        var component = target as Hand4Helper;
        if(GUILayout.Button("Initialise From Hand 3"))
        {
            InitialiseFromHand3(component.Hand3);
        }
    }

    IEnumerable<Transform> GetChildrenRecursive(Transform t)
    {
        yield return t;
        foreach (Transform item in t)
        {
            foreach (var child in GetChildrenRecursive(item))
            {
                yield return child;
            }
        }
    }

    Dictionary<string,Transform> ToDictionary(IEnumerable<Transform> transforms)
    {
        var d = new Dictionary<string, Transform>();
        foreach (var item in transforms)
        {
            d.Add(item.name, item);
        }
        return d;
    }

    void TryRemoveComponent<T>(Transform t) where T : MonoBehaviour
    {
        var c = t.GetComponent<T>();
        if(c)
        {
            DestroyImmediate(c);
        }
    }

    void InitialiseFromHand3(Hand3Solver hand3solver)
    {
        var hand3 = ToDictionary(GetChildrenRecursive(hand3solver.transform));
        var hand4 = ToDictionary(GetChildrenRecursive((target as Hand4Helper).transform));

        foreach (var pair in hand3)
        {
            if(hand4.ContainsKey(pair.Key))
            {
                var hand4transform = hand4[pair.Key];
                var hand3transform = pair.Value;

                TryRemoveComponent<Socket1D>(hand4transform);
                TryRemoveComponent<Socket2D>(hand4transform);
                TryRemoveComponent<FabrikJoint1D>(hand4transform);
                TryRemoveComponent<FabrikJoint2DB>(hand4transform);

                var socket1d = hand3transform.GetComponent<Socket1D>();
                if(socket1d)
                {
                    var joint = hand4transform.gameObject.AddComponent<FabrikJoint1D>();

                    joint.Axis = hand4transform.InverseTransformDirection(hand4transform.right);
                    joint.Forward = hand4transform.InverseTransformDirection(hand4transform.forward);
                    joint.Range = socket1d.range;
                }

                var socket2d = hand3transform.GetComponent<Socket2D>();
                if (socket2d)
                {
                    var joint = hand4transform.gameObject.AddComponent<FabrikJoint2DB>();

                    joint.range_x = socket2d.range_x;
                    joint.range_y = socket2d.range_y;
                }
            }
        }
    }
}
