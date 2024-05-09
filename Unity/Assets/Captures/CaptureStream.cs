using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Events;

/// <summary>
/// Reads back a binary file captured by the tracker manager program.
/// </summary>
public class CaptureStream : MonoBehaviour
{
    public string CaptureFilename;

    public float PlayTime { get; private set; }
    public bool Playing { get; private set; }

    public float StartTime;

    private BinaryReader reader;
    private FileStream stream;
    
    public UnityEvent<StreamFrame> OnStreamFrame { get; private set; }

    public int Frames { get; private set; }
    public bool IsOpen => stream != null;
    public int FrameSize => sizeof(float) * 7; // This constant is defined by the capture application

    public float PlaybackSpeed = 1f;

    private int frame;
    public int Frame
    {
        get => frame;
        set
        {
            if(value != frame)
            {
                // Stream frames are not self-contained, so when seeking move back a few hundred samples
                // and fast forward

                var start = Mathf.Max(value - 5000, 0);
                var end = value;
                stream.Seek(start * FrameSize, SeekOrigin.Begin);
                frame = start;
                do
                {
                    var f = NextFrame();
                    PlayTime = f.Time;
                    OnStreamFrame.Invoke(f);
                } while (frame != end);
            }
        }
    }

    private void Awake()
    {
        OnStreamFrame = new UnityEvent<StreamFrame>();
    }

    public void Open()
    {
        stream = new FileStream(CaptureFilename, FileMode.Open, FileAccess.Read);
        reader = new BinaryReader(stream);
        Frames = (int)stream.Length / FrameSize;
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
        StartCoroutine(ReadCoroutine());
    }

    public void Pause()
    {
        Playing = false;
    }

    private StreamFrame NextFrame()
    {
        StreamFrame f;

        var device = reader.ReadSingle();
        var marker = reader.ReadSingle();
        var type = reader.ReadSingle();

        f.Marker = (int)marker;

        f.Time = reader.ReadSingle();

        Vector3 data;
        data.x = reader.ReadSingle();
        data.y = reader.ReadSingle();
        data.z = reader.ReadSingle();

        f.Data = data;

        if (device == 1)
        {
            if (type == 1)
            {
                f.Type = StreamType.Accelerometer;
            }
            else
            {
                f.Type = StreamType.Gyroscope;
            }
        }
        else
        {
            f.Type = StreamType.OpticalPosition;
            f.Data /= 1000;
        }

        frame++;

        return f;
    }

    // Update is called once per frame
    void Update()
    {
        if(Playing)
        {
            PlayTime += Time.deltaTime * PlaybackSpeed;
        }
    }

    IEnumerator ReadCoroutine()
    {
        while(Playing)
        {
            var frame = NextFrame();

            while(frame.Time > PlayTime)
            {
                yield return 0;
            }

            OnStreamFrame.Invoke(frame);
        }
    }
}
