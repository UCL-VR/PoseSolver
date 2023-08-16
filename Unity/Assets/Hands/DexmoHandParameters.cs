using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.Haptics
{
    public interface IDexmoPoseController
    {
        void UpdateParameters(DexmoHandParameters parameters);
        void UpdateImmediate();
    }

    /// <summary>
    /// The parameters available to drive a hand from a Dexmo glove.
    /// Hand controllers should implement IDexmoHandParameters to recieve these.
    /// </summary>
    public struct DexmoHandParameters
    {
        public float IndexSpread;
        public float IndexBend;
        public float MiddleSpread;
        public float MiddleBend;
        public float RingSpread;
        public float RingBend;
        public float LittleSpread;
        public float LittleBend;
        public float ThumbRotation;
        public float ThumbSpread;
        public float ThumbBend;
    }
}