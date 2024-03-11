using System.Collections;
using System.Collections.Generic;
using Ubiq.Fabrik;
using UnityEngine;

[DefaultExecutionOrder(2)]
public class Hand4Measurements : MonoBehaviour
{
    public GameObject Index;
    public GameObject Middle;

    private FabrikSolver solver;

    private class Measurement
    {
        public PointMeasurement m;
        public Transform t;

        public Measurement(Node node, Transform marker)
        {
            m = new PointMeasurement()
            {
                node = node,
                offset = marker.position - node.position,
                position = marker.position
            };
            t = marker;
        }

        public void Update()
        {
            m.position = t.position;
        }
    }

    private List<Measurement> measurements = new List<Measurement>();

    private void Awake()
    {
        solver = GetComponent<FabrikSolver>();
    }

    private void Start()
    {
        if(Index != null)
        {
            measurements.Add(new Measurement(solver.GetEffector("IndexTip"), Index.transform));
        }

        if(Middle != null)
        {
            measurements.Add(new Measurement(solver.GetEffector("MiddleTip"), Middle.transform));
        }

        foreach (var item in measurements)
        {
            solver.AddConstraint(item.m);
        }
    }

    private void Update()
    {
        foreach (var item in measurements)
        {
            item.Update();
        }
    }
}
