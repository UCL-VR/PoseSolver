using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ImuIntegration : MonoBehaviour
{
    public ImuMarker marker;

    // Start is called before the first frame update
    void Start()
    {
        marker.OnInertialFrame.AddListener(OnInertialFrame);
    }

    void OnInertialFrame(Vector3 a, Vector3 g, float dt)
    {
        transform.rotation *= Quaternion.Euler(g);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
