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

    private class Marker
    {
        public Vector3 Position;
        public Vector3 Position2;
        public Transform Transform;
    }

    private void Awake()
    {
        stream = GetComponent<CaptureStream>();
        foreach (Transform item in transform)
        {
            var tokens = item.name.Split();
            var id = int.Parse(tokens[0]);
            if (!markers.ContainsKey(id))
            {
                markers.Add(id, new Marker());
            }
            var marker = markers[id];
            marker.Transform = item;
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

        switch (frame.Type)
        {
            case StreamType.OpticalPosition:
                marker.Position = frame.Data;
                break;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(MoveToCenter)
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
        }
    }

    private void LateUpdate()
    {
        foreach (var item in markers.Values)
        {
            item.Position2 = item.Position + center;
            item.Transform.position = item.Position2;
        }
    }

    private void OnDrawGizmos()
    {
        if (enabled)
        {
            Gizmos.color = Color.yellow;
            foreach (var item in markers.Values)
            {
                Gizmos.DrawWireSphere(item.Position2, GizmoSize);
            }
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(center, 0.1f);
        }
    }
}
