using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS
{
    /// <summary>
    /// This is the measurements counterpart of the DH Hand solver.
    /// </summary>
    [DefaultExecutionOrder(2)]
    public class DHHandMeasurements : MonoBehaviour
    {
        private DHHandSolver solver;

        public GameObject Index;
        public GameObject Thumb;

        private TransformPointMeasurement mIndex;
        private TransformPointMeasurement mThumb;

        private void Awake()
        {
            solver = GetComponent<DHHandSolver>();
        }

        private void Start()
        {
            mIndex = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Index), Index.transform.position);
            mThumb = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Thumb), Thumb.transform.position);
        }

        private void Update()
        {
            mIndex.Update(Index.transform.position);
            mThumb.Update(Thumb.transform.position);
        }
    }
}
