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
    }

    private void BeginCapture()
    {
        transformStream = new BinaryWriter(new FileStream(captureTool.Workspace + "/handBones.bin", FileMode.CreateNew));
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

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDestroy()
    {
        if (transformStream != null)
        {
            transformStream.Close();
            transformStream.Dispose();
            transformStream = null;
        }
    }
}
