using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using UnityEngine;

public class PerformanceProfiler : MonoBehaviour
{
    public int fps = 120;

    private Stopwatch stopwatch;
    private List<long> timings;
    private bool capturing;
    private static PerformanceProfiler singleton;
    private CaptureStream capture;
    private string filename;

    private void Awake()
    {
        stopwatch = new Stopwatch();
        timings = new List<long>();
        singleton = this;
    }

    // Start is called before the first frame update
    void Start()
    {
        capture = GetComponent<CaptureStream>();
        capture.OnFinished.AddListener(OnStreamEnd);
    }

    public void Capture(string filename)
    {
        UnityEngine.Debug.Log("Begin Performance Profile");
        this.filename = filename;
        Directory.CreateDirectory(Path.GetDirectoryName(filename));
        capturing = true;
        Time.captureDeltaTime = 1f / (float)fps;
        capture.PlaybackSpeed = 1f;
        capture.Play();
    }

    private void OnStreamEnd()
    {
        if (capturing)
        {
            capturing = false;
            Time.captureDeltaTime = 0;
            UnityEngine.Debug.Log("Finished Performance Profile");

            if (!string.IsNullOrEmpty(filename) && enabled)
            {
                string s = "";
                foreach (var item in timings)
                {
                    s += item + "\r\n";
                }
                System.IO.File.WriteAllText(filename, s);
            }

            UnityEditor.EditorApplication.isPlaying = false;
        }
    }

    private void startFrame()
    {
        stopwatch.Restart();
    }

    private void stopFrame()
    {
        timings.Add(stopwatch.ElapsedMilliseconds);
    }

    public static void StartFrame()
    {
        if (singleton)
        {
            singleton.startFrame();
        }
    }

    public static void EndFrame()
    {
        if (singleton)
        {
            singleton.stopFrame();
        }
    }
}
