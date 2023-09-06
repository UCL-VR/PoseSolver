using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS
{
    /// <summary>
    /// This is the measurements counterpart of the DH Hand solver.
    /// </summary>
    [DefaultExecutionOrder(2)] // Make sure that the solver is initialised before Start is called.
    public class DHHandMeasurements : MonoBehaviour
    {
        private DHHandSolver solver;
        
        public GameObject Thumb;
        public GameObject Index;
        public GameObject Middle;
        public GameObject Ring;
        public GameObject Little;
        public GameObject Wrist;

        private TransformPointMeasurement mIndex;
        private TransformPointMeasurement mThumb;
        private TransformPointMeasurement mWrist;
        private TransformPointMeasurement mLittle;
        private TransformPointMeasurement mMiddle;
        private TransformPointMeasurement mRing;

        private void Awake()
        {
            solver = GetComponent<DHHandSolver>();
        }

        private void Start()
        {
            if (Index)
            {
                mIndex = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Index), Index.transform.position);
            }
            if (Thumb)
            {
                mThumb = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Thumb), Thumb.transform.position);
            }
            if (Middle)
            {
                mMiddle = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Middle), Middle.transform.position);
            }
            if (Ring)
            {
                mRing = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Ring), Ring.transform.position);
            }
            if (Little)
            {
                mLittle = solver.AddPointMeasurement(solver.GetEndNode(Fingers.Little), Little.transform.position);
            }
            if (Wrist)
            {
                mWrist = solver.AddPointMeasurement(Wrist.transform.position);
            }
        }

        private void Update()
        {

            if (mThumb != null) 
            {
                mThumb.Update(Thumb.transform.position);
            }
            if (mIndex != null)
            {
                mIndex.Update(Index.transform.position);
            }
            if (mMiddle != null)
            {
                mMiddle.Update(Middle.transform.position);
            }
            if (mRing != null)
            {
                mRing.Update(Ring.transform.position);
            }
            if (mLittle != null)
            {
                mLittle.Update(Little.transform.position);
            }
            if (mWrist != null) 
            {
                mWrist.Update(Wrist.transform.position);
            }
        }
    }
}
