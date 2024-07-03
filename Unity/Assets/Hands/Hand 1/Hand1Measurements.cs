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
        
        public ImuMarker Thumb;
        public ImuMarker Index;
        public ImuMarker Middle;
        public ImuMarker Ring;
        public ImuMarker Little;
        public ImuMarker Wrist;

        private float ageThreshold = 0.0001f;

        public bool UseInertial = false;

        internal class ImuMarkerMeasurement
        {
            public ImuMarker marker;
            public Hand1Solver solver;
            public Fingers finger;

            public Hand1Solver.PointMeasurement position;
            public Hand1Solver.OrientationMeasurement orientation;

            public bool imu { get => marker.Integrate; set => marker.Integrate = value; }

            public bool wrist = false;

            public Transform fingerTransform => wrist ? solver.GetTransform() : solver.GetTransform(finger);

            public ImuMarkerMeasurement(ImuMarker marker, Hand1Solver solver, Fingers finger)
            {
                this.marker = marker;
                this.solver = solver;
                this.finger = finger;

                position = solver.AddPointConstraint(finger, marker.transform.position);
                orientation = solver.AddOrientationConstraint(finger);
                orientation.Remove();

                imu = false;
            }

            public ImuMarkerMeasurement(ImuMarker marker, Hand1Solver solver)
            {
                this.marker = marker;
                this.solver = solver;

                position = solver.AddPointConstraint(marker.transform.position);
                orientation = solver.AddOrientationConstraint(finger);
                orientation.Remove();

                imu = false;
                wrist = true;
            }
        }

        private List<ImuMarkerMeasurement> measurements = new List<ImuMarkerMeasurement>();

        private void Awake()
        {
            solver = GetComponent<Hand1Solver>();
        }

        private void Start()
        {
            if (Index)
            {
                measurements.Add(new ImuMarkerMeasurement(Index, solver, Fingers.Index));
            }
            if (Thumb)
            {
                measurements.Add(new ImuMarkerMeasurement(Thumb, solver, Fingers.Thumb));
            }
            if (Middle)
            {
                measurements.Add(new ImuMarkerMeasurement(Middle, solver, Fingers.Middle));
            }
            if (Ring)
            {
                measurements.Add(new ImuMarkerMeasurement(Ring, solver, Fingers.Ring));
            }
            if (Little)
            {
                measurements.Add(new ImuMarkerMeasurement(Little, solver, Fingers.Little));
            }
            if (Wrist)
            {
                measurements.Add(new ImuMarkerMeasurement(Wrist, solver));
            }
        }

        private void Update()
        {

            foreach (var m in measurements)
            {
                if (m.marker.PositionAge >= ageThreshold)
                {
                    m.position.Remove();

                    if (UseInertial && !m.imu && !m.wrist)
                    {
                        m.marker.transform.rotation = m.fingerTransform.rotation;
                        m.imu = true;
                    }

                    if (m.imu) // If we are using the IMU, and so this member should be updated each frame
                    {
                        m.orientation.Update(m.marker.transform.rotation);
                    }
                }
                else
                {
                    m.position.Update(m.marker.transform.position);

                    if (m.imu)
                    {
                        m.orientation.Remove();
                        m.imu = false;
                    }
                }
            }
        }
    }
}
