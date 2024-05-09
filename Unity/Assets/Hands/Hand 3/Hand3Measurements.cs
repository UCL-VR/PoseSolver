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

    internal class ImuMarkerMeasurement
    {
        public ImuMarker marker;
        public Hand3Solver.PointMeasurement measurement;

        public ImuMarkerMeasurement(ImuMarker marker, Hand3Solver.PointMeasurement measurement)
        {
            this.marker = marker;
            this.measurement = measurement;
        }
    }

    private List<ImuMarkerMeasurement> measurements = new List<ImuMarkerMeasurement>();

    private float ageThreshold = 0.100f;

    private void Awake()
    {
        solver = GetComponent<Hand3Solver>();
    }

    private void Start()
    {
        if (Index != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Index, solver.AddPointConstraint((Fingers.Index), Index.transform.position)));
        }
        if (Thumb != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Thumb, solver.AddPointConstraint((Fingers.Thumb), Thumb.transform.position)));
        }
        if (Middle != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Middle, solver.AddPointConstraint((Fingers.Middle), Middle.transform.position)));
        }
        if (Ring != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Ring, solver.AddPointConstraint((Fingers.Ring), Ring.transform.position)));
        }
        if (Little != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Little, solver.AddPointConstraint((Fingers.Little), Little.transform.position)));
        }
        if (Wrist != null)
        {
            measurements.Add(new ImuMarkerMeasurement(Wrist, solver.AddPointConstraint(Wrist.transform.position)));
        }
    }

    private void Update()
    {
        foreach (var m in measurements)
        {
            if (m.marker.PositionAge > ageThreshold)
            {
                m.measurement.Remove();
            }
            else
            {
                m.measurement.Update(m.marker.transform.position);
            }
        }
    }
}
