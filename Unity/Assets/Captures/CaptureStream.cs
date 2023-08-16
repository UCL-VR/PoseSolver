using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Events;

public class CaptureStream : MonoBehaviour
{
    public string CaptureFilename;

    public float PlayTime { get; private set; }
    public bool Playing { get; private set; }

    private BinaryReader reader;

    public UnityEvent<StreamFrame> OnStreamFrame { get; private set; }

    private void Awake()
    {
        OnStreamFrame = new UnityEvent<StreamFrame>();
        reader = new BinaryReader(new FileStream(CaptureFilename, FileMode.Open, FileAccess.Read));
    }

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(ReadCoroutine());
    }

    public void Play()
    {
        Playing = true;
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

        return f;
    }

    // Update is called once per frame
    void Update()
    {
        if(Playing)
        {
            PlayTime += Time.deltaTime;
        }
    }

    IEnumerator ReadCoroutine()
    {
        while(true)
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
