using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum StreamType
{
    Accelerometer,
    Gyroscope,
    OpticalPosition
}

public struct StreamFrame
{
    public int Marker;
    public float Time;
    public StreamType Type;
    public Vector3 Data;
}

public class MarkerManager : MonoBehaviour
{
    private CaptureStream stream;
    private Dictionary<int, Marker> markers = new Dictionary<int, Marker>();
    private Vector3 center;

    public float GizmoSize = 0.05f;
    public bool MoveToCenter;

    private void Awake()
    {
        stream = GetComponent<CaptureStream>();
        foreach (var item in GetComponentsInChildren<Marker>())
        {
            markers.Add(item.Id, item);
        }
    }

    void Start()
    {
        stream.OnStreamFrame.AddListener(OnStreamFrame);
    }

    void OnStreamFrame(StreamFrame frame)
    {
        if (!markers.ContainsKey(frame.Marker))
        {
            return;
        }
        var marker = markers[frame.Marker];
        marker.OnFrame(frame);
    }

    // Update is called once per frame
    void Update()
    {
        if (MoveToCenter)
        {
            center = Vector3.zero;
            float i = 0;
            foreach (var item in markers.Values)
            {
                if (item.Position != Vector3.zero)
                {
                    center += item.Position;
                    i++;
                }
            }
            if (i > 0)
            {
                center /= -i;
            }
            transform.position = center;
        }
    }

    private void OnDrawGizmos()
    {
        if (enabled)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(center, GizmoSize * 2);
        }
    }
}
