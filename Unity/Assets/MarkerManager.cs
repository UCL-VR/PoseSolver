using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Events;

public interface IStreamable
{
    UnityEvent<StreamFrame> OnStreamFrame { get; }
}

public enum StreamType : int
{
    Accelerometer = 1,
    Gyroscope = 2,
    OpticalPosition = 3
}

[StructLayout(LayoutKind.Sequential)]
public struct StreamFrame
{
    public int Marker;
    public float Time;
    public StreamType Type;
    public Vector3 Data;
}

public class MarkerManager : MonoBehaviour
{
    private Dictionary<int, ImuMarker> markers = new Dictionary<int, ImuMarker>();
    private Vector3 center;

    public float GizmoSize = 0.05f;
    public bool MoveToCenter;

    private void Awake()
    {
        foreach (var item in GetComponentsInChildren<ImuMarker>())
        {
            markers[item.Id] = item;
        }
    }

    void Start()
    {
        if (GetComponent<CaptureStream>())
        {
            GetComponent<CaptureStream>().OnStreamFrame.AddListener(OnStreamFrame);
        }
        if (GetComponent<UdpStream>())
        {
            GetComponent<UdpStream>().OnStreamFrame.AddListener(OnStreamFrame);
        }
    }

    public void OnStreamFrame(StreamFrame frame)
    {
        if(!Application.isPlaying)
        {
            Awake();
        }

        if (!markers.ContainsKey(frame.Marker))
        {
            return;
        }
        var marker = markers[frame.Marker];
        marker.ApplyFrame(frame);
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

        foreach (var item in markers.Values)
        {
            item.PositionAge += Time.deltaTime;
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
