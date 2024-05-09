using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(2)]
public class Hand2Measurements : MonoBehaviour
{
    private Hand2Solver solver;

    public GameObject Index;
    public GameObject Thumb;
    public GameObject Wrist;

    private Hand2Solver.PointMeasurement mIndex;
    private Hand2Solver.PointMeasurement mThumb;
    private Hand2Solver.PointMeasurement mWrist;


    private void Awake()
    {
        solver = GetComponent<Hand2Solver>();
    }

    // Start is called before the first frame update
    void Start()
    {
        mIndex = solver.AddConstraint(Fingers.Index, Index.transform.position);
        mThumb = solver.AddConstraint(Fingers.Thumb, Thumb.transform.position);
        mWrist = solver.AddConstraint(Wrist.transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        mIndex.Update(Index.transform.position);
        mThumb.Update(Thumb.transform.position);
        mWrist.Update(Wrist.transform.position);
    }
}
