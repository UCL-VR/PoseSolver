using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(2)] // Make sure that the solver is initialised before Start is called.
public class Hand3Measurements : MonoBehaviour
{
    private Hand3Solver solver;

    public ImuMarker Thumb;
    public ImuMarker Index;
    public ImuMarker Middle;
    public ImuMarker Ring;
    public ImuMarker Little;
    public ImuMarker Wrist;

    public bool UseInertial = false;

    internal class ImuMarkerMeasurement
    {
        public ImuMarker marker;
        public Hand3Solver solver;
        public Fingers finger;

        public Hand3Solver.PointMeasurement position;
        public Hand3Solver.OrientationMeasurement orientation;

        public bool imu { get => marker.Integrate; set => marker.Integrate = value; }

        public bool wrist = false;

        public Transform fingerTransform => wrist ? solver.GetTransform() : solver.GetTransform(finger);

        public ImuMarkerMeasurement(ImuMarker marker, Hand3Solver solver, Fingers finger)
        {
            this.marker = marker;
            this.solver = solver;
            this.finger = finger;

            position = solver.AddPointConstraint(finger, marker.transform.position);
            orientation = solver.AddOrientationConstraint(finger);
            orientation.Remove(); 
            
            imu = false;
        }

        public ImuMarkerMeasurement(ImuMarker marker, Hand3Solver solver)
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
        solver = GetComponent<Hand3Solver>();
    }

    private void Start()
    {
        if (Index != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Index, solver, Fingers.Index));
        }
        if (Thumb != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Thumb, solver, Fingers.Thumb));
        }
        if (Middle != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Middle, solver, Fingers.Middle));
        }
        if (Ring != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Ring, solver, Fingers.Ring));
        }
        if (Little != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Little, solver, Fingers.Little));
        }
        if (Wrist != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Wrist, solver));
        }
    }

    private void Update()
    {
        foreach (var m in measurements)
        {
            if (!m.marker.HasPosition)
            {
                m.position.Remove();

                if(UseInertial && !m.imu && !m.wrist)
                {
                    m.marker.transform.rotation = m.fingerTransform.rotation; // Reset the rotation to the starting rotation for integration over the next few frames
                    m.imu = true;
                }

                if(m.imu) // If we are using the IMU, and so this member should be updated each frame
                {
                    m.orientation.Update(m.marker.transform.rotation);
                }
            }
            else
            {
                m.position.Update(m.marker.transform.position);

                if(m.imu)
                {
                    m.orientation.Remove();
                    m.imu = false;
                }
            }
        }
    }
}
