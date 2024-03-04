using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Events;

[RequireComponent(typeof(CaptureStream))]
[DefaultExecutionOrder(1000)]
public class VideoCaptureTool : MonoBehaviour
{
    public int fps = 300;
    public int captureFps = 60;

    private CaptureStream capture;
    private string workspace;
    private int counter;
    private bool capturing;
    private float frameTime;

    public UnityEvent OnVideoFrame;
    public UnityEvent BeginCapture;

    public string Workspace => workspace;

    // Make a video at the end like so:
    //  ffmpeg -framerate 60 -pattern_type sequence -i %01d.png out.mp4

    private void Awake()
    {
        OnVideoFrame = new UnityEvent();
        BeginCapture = new UnityEvent();
    }

    // Start is called before the first frame update
    void Start()
    {
        capture = GetComponent<CaptureStream>();
        capture.OnFinished.AddListener(OnStreamEnd);
        workspace = Path.Combine(Application.dataPath, "..", "Workspace");
        Directory.CreateDirectory(workspace);
    }

    // Update is called once per frame
    void LateUpdate()
    {
        if (capturing)
        {
            frameTime += Time.deltaTime;
            var capturePeriod = (1f / captureFps);
            if (frameTime >= capturePeriod)
            {
                OnVideoFrame.Invoke();
                ScreenCapture.CaptureScreenshot($"{workspace}\\{counter++}.png");
                frameTime = 0;
            }
        }
    }

    public void Capture(string filename)
    {
        // Create the folder structure
        workspace = Path.Combine(Path.GetDirectoryName(filename), Path.GetFileNameWithoutExtension(filename));
        Directory.CreateDirectory(workspace);

        Debug.Log("Beginning Capture");

        Time.captureDeltaTime = 1f / (float)fps;
        counter = 0;
        capturing = true;
        capture.PlaybackSpeed = 1f;
        capture.Play();

        BeginCapture.Invoke();
    }

    private void OnStreamEnd()
    {
        capturing = false;
        Time.captureDeltaTime = 0;
        Debug.Log("Finished Capture Playback");
    }
}
