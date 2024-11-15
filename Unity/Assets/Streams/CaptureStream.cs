using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Events;
using static UnityEditor.Progress;

/// <summary>
/// Reads back a binary file captured by the tracker manager program.
/// </summary>
public class CaptureStream : MonoBehaviour
{
    public string CaptureFilename;

    public float PlayTime { get; private set; }
    public bool Playing { get; private set; }

    public float StartTime;
    public float EndTime;
    
    public UnityEvent<StreamFrame> OnStreamFrame { get; private set; }
    public UnityEvent OnFinished { get; private set; }

    public int Frames { get; private set; }
    public bool IsOpen => stream != null;
    public int FrameSize => sizeof(float) * 6; // This constant is defined by the capture application

    public float PlaybackSpeed = 1f;

    private BinaryReader reader;
    private FileStream stream;
    private Dictionary<int, ImuMarker> markers = new Dictionary<int, ImuMarker>();
    private float frameTime;

    private int frame;
    public int Frame
    {
        get => frame;
        set
        {
            if(value != frame)
            {
                // Individual Stream frames do not capture the entire state, so
                // when seeking move back a few hundred samples and fast forward

                var start = Mathf.Max(value - 5000, 0);
                var end = value;
                stream.Seek(start * FrameSize, SeekOrigin.Begin);
                frame = start;
                do
                {
                    var f = NextFrame();
                    PlayTime = f.Time;

                    if(!Application.isPlaying)
                    {
                        foreach (var item in GetComponentsInChildren<ImuMarker>())
                        {
                            if(item.Id == f.Marker)
                            {
                                item.ApplyFrame(f);
                                item.transform.localPosition = item.Position;
                            }
                        }
                    }

                } while (frame != end);
            }
        }
    }

    private void SeekTo(int f)
    {
        while (frame > f)
        {
            stream.Seek(-FrameSize, SeekOrigin.Current);
            frame--;

        }

    }

    public void Open()
    {
        stream = new FileStream(CaptureFilename, FileMode.Open, FileAccess.Read);
        reader = new BinaryReader(stream);
        frame = 0;
        PlayTime = 0;
        Frames = (int)stream.Length / FrameSize;
    }

    private void Awake()
    {
        OnFinished = new UnityEvent();
        foreach (var item in GetComponentsInChildren<ImuMarker>())
        {
            markers[item.Id] = item;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        Open();
    }

    public void Play()
    {
        Playing = true;
        PlayTime = StartTime;
        frameTime = PlayTime;
    }

    public void Pause()
    {
        Playing = false;
    }

    private StreamFrame NextFrame()
    {
        StreamFrame f;
        Vector3 data;

        f.Type = (StreamType)reader.ReadSingle();
        f.Marker = (int)reader.ReadSingle();
        f.Time = reader.ReadSingle();
        data.x = reader.ReadSingle();
        data.y = reader.ReadSingle();
        data.z = reader.ReadSingle();

        f.Data = data;

        switch (f.Type)
        {
            case StreamType.OpticalPosition:
                f.Data /= 1000;
                break;
        }

        frame++;

        return f;
    }

    void ApplyFrame(StreamFrame frame)
    {
        if (markers.ContainsKey(frame.Marker))
        {
            var marker = markers[frame.Marker];
            marker.ApplyFrame(frame);
            marker.transform.localPosition = marker.Position;
        }

        frameTime = frame.Time;

        foreach (var item in markers.Values)
        {
            item.SystemTime = frameTime;
        }
    }

    // Update is called once per frame
    void Update()
    {
        if(Playing)
        {
            var deltaTime = Time.deltaTime * PlaybackSpeed;            
            
            PlayTime += deltaTime;

            while (frameTime < PlayTime)
            {
                try
                {
                    var frame = NextFrame();

                    ApplyFrame(frame);

                    if(frameTime >= PlayTime)
                    {
                        break;
                    }

                    if(frameTime > EndTime && EndTime != 0)
                    {
                        throw new EndOfStreamException();
                    }
                }
                catch (EndOfStreamException)
                {
                    Playing = false;
                    OnFinished.Invoke();
                    break;
                }
            }
        }
    }
}
