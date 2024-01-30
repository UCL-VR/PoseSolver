using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PreIntegrationEstimator : PoseSolver
{
    IntPtr preIntegrationFactor;
    IntPtr orientationFactor;
    IntPtr begin;
    IntPtr end;
    IntPtr referencePose;

    public ImuMarker marker;

    public bool Calibrate = true;

    // Start is called before the first frame update
    void Start()
    {
        initialise();
        begin = addPose(true);
        end = addPose(false);
        referencePose = addPose(true);
        orientationFactor = addImuOrientationFactor(referencePose, begin);
        preIntegrationFactor = addPreIntegrationFactor(begin, end);
        marker.OnInertialFrame.AddListener(OnImuSample);
    }

    public void OnImuSample(Vector3 a, Vector3 g, float dt)
    {
        // The Imu Measurements are given in m/s^2 and radians/s. Unit conversions
        // take place in the Marker class.

        // Either the Imu transform is known or it is not - when it is known,
        // update the start pose & calibration. When external validity is lost,
        // begin to use it to estimate the local position.

        IntPtr displayPose = end;

        if (Calibrate)
        {
            setPoseVariable(begin);
            addImuOrientationMeasurement(orientationFactor, a.x, a.y, a.z, g.x, g.y, g.z, dt);
            displayPose = begin;
        }
        else
        {
            setPoseConstant(begin);
            addImuMeasurement(preIntegrationFactor, a.x, a.y, a.z, g.x, g.y, g.z, dt);
            displayPose = end;
        }

        solve();

        var pose = getPose(displayPose);
        transform.position = pose.Position;
        transform.rotation = pose.Rotation;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos()
    {
        Gizmos.matrix = Matrix4x4.identity;
        Gizmos.DrawWireCube(transform.position, Vector3.one * 0.1f);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + transform.up);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.forward);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(Vector3.zero, Vector3.down);
    }
}
