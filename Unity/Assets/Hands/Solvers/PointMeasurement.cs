using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UCL.CASMS.DH;

/// <summary>
/// A helper Component that allows a Point Measurement to be maintained through
/// the Unity Scene Graph. Point Measurements constrain a Pose3. These
/// measurements are 'immortal' in that they are continually updated and always
/// current so long as they exist.
/// </summary>
public class PointMeasurement : MonoBehaviour
{
    /// <summary>
    /// The GameObject in the Scene Graph that corresponds to the Paramter Block
    /// constrained by the Measurement. What this actually is depends on the Solver.
    /// </summary>
    public GameObject Joint;

    private TransformPointMeasurement measurement;

    private Measurements measurements;

    private void Start()
    {
        measurements = GetComponentInParent<Measurements>();
        measurement =  measurements.Solver.AddPointMeasurement(Joint, transform.position);
    }

    private void Update()
    {
        if (measurement != null)
        {
            measurement.Update(transform.position);
        }
    }

}
