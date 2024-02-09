using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS
{
    /// <summary>
    /// This is the measurements counterpart of the DH Hand solver. This class
    /// creates a PointMeasurement Factor for each fingertip, and *updates* them
    /// each frame.
    /// </summary>
    [DefaultExecutionOrder(2)] // Make sure that the solver is initialised before Start is called.
    public class Hand1Measurements : MonoBehaviour
    {
        private Hand1Solver solver;
        
        public GameObject Thumb;
        public GameObject Index;
        public GameObject Middle;
        public GameObject Ring;
        public GameObject Little;
        public GameObject Wrist;

        internal Hand1Solver.PointMeasurement mIndex;
        internal Hand1Solver.PointMeasurement mThumb;
        internal Hand1Solver.PointMeasurement mWrist;
        internal Hand1Solver.PointMeasurement mLittle;
        internal Hand1Solver.PointMeasurement mMiddle;
        internal Hand1Solver.PointMeasurement mRing;

        private void Awake()
        {
            solver = GetComponent<Hand1Solver>();
        }

        private void Start()
        {
            if (Index)
            {
                mIndex = solver.AddPointConstraint((Fingers.Index), Index.transform.position);
            }
            if (Thumb)
            {
                mThumb = solver.AddPointConstraint((Fingers.Thumb), Thumb.transform.position);
            }
            if (Middle)
            {
                mMiddle = solver.AddPointConstraint((Fingers.Middle), Middle.transform.position);
            }
            if (Ring)
            {
                mRing = solver.AddPointConstraint((Fingers.Ring), Ring.transform.position);
            }
            if (Little)
            {
                mLittle = solver.AddPointConstraint((Fingers.Little), Little.transform.position);
            }
            if (Wrist)
            {
                mWrist = solver.AddPointConstraint(Wrist.transform.position);
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
