using System.Collections;
using System.Collections.Generic;

using Mathf = UnityEngine.Mathf;

namespace UCL.CASMS.DH
{
    /// <summary>
    /// Helper class that computes DH Parameters based on different reference systems
    /// </summary>
    public static class Parameter
    {
        /// <summary>
        /// Computes the length of the link (r) based on the x,y position of the endpoint
        /// </summary>
        /// <param name="x">Meters forward of the joint</param>
        /// <param name="y">Meters right of the joint</param>
        /// <returns>Joint length in meters</returns>
        public static float Length(float x, float y)
        {
            return Mathf.Sqrt(Mathf.Pow(x, 2f) + Mathf.Pow(y, 2f));
        }

        /// <summary>
        /// Computes the angle (theta) of the joint based on the x,y position of the endpoint
        /// </summary>
        /// <param name="x">Meters forward of the joint</param>
        /// <param name="y">Meters right of the joint</param>
        /// <returns>Angle in degrees clockwise around the joint pivot</returns>
        public static float Angle(float x, float y)
        {
            return Mathf.Atan2(y, x) * Mathf.Rad2Deg;
        }

        public static Joint MakeJoint(string name, float x, float y, float z)
        {
            Joint joint = new DH.Joint(name);
            joint.r = Length(x, y);
            joint.th = Angle(x, y);
            joint.d = z;
            return joint;
        }


    }
}