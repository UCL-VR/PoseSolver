using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OrientationIntegrator
{
    public Quaternion rotation;

    public OrientationIntegrator(ImuMarker marker)
    {
        rotation = Quaternion.identity;
        marker.OnInertialFrame.AddListener(OnInertialFrame);
    }

    void OnInertialFrame(Vector3 a, Vector3 g, float dt)
    {
        rotation *= Quaternion.Euler(g);
    }
}
