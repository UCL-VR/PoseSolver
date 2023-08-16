using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(2)]
public class Hand1Measurements : MonoBehaviour
{
    private Hand1Solver solver;

    public GameObject Index;
    public GameObject Thumb;

    private TransformPointMeasurement mIndex;
    private TransformPointMeasurement mThumb;


    private void Awake()
    {
        solver = GetComponent<Hand1Solver>();
    }

    // Start is called before the first frame update
    void Start()
    {
        mIndex = solver.AddMeasurement(Fingers.Index, Index.transform.position);
        mThumb = solver.AddMeasurement(Fingers.Thumb, Thumb.transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        mIndex.Update(Index.transform.position);
        mThumb.Update(Thumb.transform.position);
    }
}
