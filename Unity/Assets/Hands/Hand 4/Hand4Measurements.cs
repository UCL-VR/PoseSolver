using System.Collections;
using System.Collections.Generic;
using Ubiq.Fabrik;
using UnityEngine;

[DefaultExecutionOrder(2)]
public class Hand4Measurements : MonoBehaviour
{
    public GameObject Index;
    public GameObject Middle;
    public GameObject Ring;
    public GameObject Little;
    public GameObject Thumb;
    public GameObject Wrist;

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
        if(Index != null && solver.HasEffector("IndexTip"))
        {
            measurements.Add(new Measurement(solver.GetEffector("IndexTip"), Index.transform));
        }

        if(Middle != null && solver.HasEffector("MiddleTip"))
        {
            measurements.Add(new Measurement(solver.GetEffector("MiddleTip"), Middle.transform));
        }

        if (Ring != null && solver.HasEffector("RingTip"))
        {
            measurements.Add(new Measurement(solver.GetEffector("RingTip"), Ring.transform));
        }

        if (Little != null && solver.HasEffector("LittleTip"))
        {
            measurements.Add(new Measurement(solver.GetEffector("LittleTip"), Little.transform));
        }

        if (Thumb != null && solver.HasEffector("ThumbTip"))
        {
            measurements.Add(new Measurement(solver.GetEffector("ThumbTip"), Thumb.transform));
        }

        if (Wrist != null && solver.HasEffector(gameObject.name))
        {
            measurements.Add(new Measurement(solver.GetEffector(gameObject.name), Wrist.transform));
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
