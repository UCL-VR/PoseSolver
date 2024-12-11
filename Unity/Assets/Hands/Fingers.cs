using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum Fingers : int
{
    Thumb = 0,
    Index = 1,
    Middle = 2,
    Ring = 3,
    Little = 4,
    Wrist = 5,
}

public static class FingersExtension
{
    public static Fingers GetFinger(string str)
    {
        if (str.Contains("Thumb"))
        {
            return Fingers.Thumb;
        }
        if (str.Contains("Index"))
        {
            return Fingers.Index;
        }
        if (str.Contains("Middle"))
        {
            return Fingers.Middle;
        }
        if (str.Contains("Ring"))
        {
            return Fingers.Ring;
        }
        if (str.Contains("Little"))
        {
            return Fingers.Little;
        }
        throw new ArgumentOutOfRangeException();
    }
}