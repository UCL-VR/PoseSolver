using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

// Sits next to the Video Capture Tool to get data about the hand during fitting.

public class ExperimentCapture : MonoBehaviour
{
    public SkinnedMeshRenderer hand;

    private VideoCaptureTool captureTool;
    private Transform[] bones;
    private BinaryWriter transformStream;
    private string filename;

    private void Awake()
    {
        captureTool = GetComponent<VideoCaptureTool>();
        bones = hand.bones;
    }

    // Start is called before the first frame update
    void Start()
    {
        captureTool.OnVideoFrame.AddListener(OnVideoFrame);
        captureTool.BeginCapture.AddListener(BeginCapture);
        captureTool.EndCapture.AddListener(EndCapture);
    }

    private void BeginCapture()
    {
        filename = captureTool.Destination + ".bones.bin";
        transformStream = new BinaryWriter(new FileStream(filename, FileMode.Create));
        Debug.Log($"Num Bones: {bones.Length}");
    }

    private void OnVideoFrame()
    {
        foreach (var item in bones)
        {
            transformStream.Write(item.position.x);
            transformStream.Write(item.position.y);
            transformStream.Write(item.position.z);
        }
    }

    private void EndCapture()
    {
        if (transformStream != null)
        {
            transformStream.Close();
            transformStream.Dispose();
            transformStream = null;
        }
    }
}
