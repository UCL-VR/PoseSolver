using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Events;

[RequireComponent(typeof(CaptureStream))]
[DefaultExecutionOrder(1000)]
public class SamplesExport : MonoBehaviour
{
    public int fps = 300;

    private CaptureStream capture;
    private int counter;
    private bool capturing;
    private float frameTime;

    private Wrapper writer;

    public List<Transform> Markers;
    public Transform Wrist;

    // Start is called before the first frame update
    void Start()
    {
        capture = GetComponent<CaptureStream>();
        capture.OnFinished.AddListener(OnStreamEnd);
    }

    class Wrapper
    {
        TextWriter writer;

        public Wrapper(TextWriter writer)
        {
            this.writer = writer;
        }

        public void Write(float value, bool lineEnd = false)
        {
            if (lineEnd)
            {
                writer.Write(value + "\n");
            }
            else
            {
                writer.Write(value + ", ");
            }
        }

        public void Dispose()
        {
            writer.Dispose();
            writer = null;
        }
    }

    // Update is called once per frame
    void LateUpdate()
    {
        if (capturing)
        {
            frameTime += Time.deltaTime;
            foreach (var item in Markers)
            {
                writer.Write(item.position.x);
                writer.Write(item.position.y);
                writer.Write(item.position.z);
            }
            writer.Write(Wrist.position.x);
            writer.Write(Wrist.position.y);
            writer.Write(Wrist.position.z);
            writer.Write(Wrist.rotation.x);
            writer.Write(Wrist.rotation.y);
            writer.Write(Wrist.rotation.z);
            writer.Write(Wrist.rotation.w, true);
        }
    }

    public void Capture(string filename)
    {
        writer = new Wrapper(File.CreateText(filename));

        UnityEngine.Debug.Log("Beginning Samples Export");

        Time.captureDeltaTime = 1f / (float)fps;
        counter = 0;
        capturing = true;
        capture.PlaybackSpeed = 1f;
        capture.Play();
    }

    private void OnStreamEnd()
    {
        capturing = false;
        Time.captureDeltaTime = 0;
        UnityEngine.Debug.Log("Finished Samples Playback");

        UnityEditor.EditorApplication.isPlaying = false;

        End();
    }

    private void End()
    {
        writer.Dispose();
        writer = null;
    }
}
