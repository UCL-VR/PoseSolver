using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class StreamHelper
{
    private FileStream stream;
    private BinaryWriter writer;

    public StreamHelper(string filename)
    {
        writer = new BinaryWriter(new FileStream(filename, FileMode.Create));
    }

    public void Push(params Vector3[] v)
    {
        foreach (var item in v)
        {
            writer.Write(item.x);
            writer.Write(item.y);
            writer.Write(item.z);
        }
    }

    ~StreamHelper()
    {

    }
}
