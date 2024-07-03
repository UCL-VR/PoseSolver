using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using UnityEngine;

public class PerformanceProfiler : MonoBehaviour
{
    private Stopwatch stopwatch;
    private List<long> timings;
    private static PerformanceProfiler singleton;

    public string filename;

    private void Awake()
    {
        stopwatch = new Stopwatch();
        timings = new List<long>();
        singleton = this;
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
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

    private void OnDestroy()
    {
        if (!string.IsNullOrEmpty(filename) && enabled)
        {
            string s = "";
            foreach (var item in timings)
            {
                s += item + "\r\n";
            }
            System.IO.File.WriteAllText(filename, s);
        }
    }
}
