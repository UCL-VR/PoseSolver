using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Threading;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Receives live a binary stream of events via a UDP socket
/// </summary>
public class UdpStream : MonoBehaviour
{
    public int Port = 24693;

    [ReadOnly]
    public int Frames;

    const int FRAME_LENGTH_FLOATS = 6;

    private UdpClient listener;
    private float[] floats = new float[FRAME_LENGTH_FLOATS];
    private ConcurrentQueue<StreamFrame> frames;

    private Dictionary<int, ImuMarker> markers;

    private void Awake()
    {
        markers = new Dictionary<int, ImuMarker>();
        foreach (var item in GetComponentsInChildren<ImuMarker>())
        {
            markers[item.Id] = item;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        frames = new ConcurrentQueue<StreamFrame>();

        var worker = new Thread(async () =>
        {
            listener = new UdpClient(Port);
            while (true)
            {
                var result = await listener.ReceiveAsync();
                OnBytes(result.Buffer);
            }
        });
        worker.Start();

        StartCoroutine(EventCoroutine());
    }

    // Update is called once per frame
    void Update()
    {
        foreach (var item in markers.Values)
        {
            item.PositionAge += Time.deltaTime;
        }

        foreach (var item in markers.Values)
        {
            item.transform.localPosition = item.Position;
        }
    }

    private void OnBytes(byte[] packet)
    {
        Buffer.BlockCopy(packet, 0, floats, 0, sizeof(float) * FRAME_LENGTH_FLOATS);
        StreamFrame f;
        Vector3 data;
        f.Type = (StreamType)floats[0];
        f.Marker = (int)floats[1];
        f.Time = floats[2];
        data.x = floats[3];
        data.y = floats[4];
        data.z = floats[5];
        f.Data = data;
        frames.Enqueue(f);
    }

    private IEnumerator EventCoroutine()
    {
        while(true)
        {
            StreamFrame result;
            while(frames.TryDequeue(out result))
            {
                Frames++;
                if(markers.ContainsKey(result.Marker))
                {
                    markers[result.Marker].ApplyFrame(result);
                }
            }
            yield return 0;
        }
    }
}
