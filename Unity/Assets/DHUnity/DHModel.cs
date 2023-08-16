using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.DH
{
    /// <summary>
    /// A Robot described as a set of DH Joint Chains. It is expected this model will be built programatically, or
    /// loaded from a file, and used to drive a set of GameObjects.
    /// When using the DH classes, one should be mindful of Unity's serialisation behaviour. Classes are serialised 
    /// like structs, and null references are not supported. One component should host the model and the others 
    /// should reference this property dynamically. That Component is the DHNodesController.
    /// Any modifications should be limited to new methods or basic type properties. Complex constructors and 
    /// destructors should not be used.
    /// Subclasses can be used to help with initialisation, so long as they don't contain any properties themselves,
    /// as any object graph should be able to be serialised and restored from only the base types.
    /// </summary>
    [Serializable]
    public class Model
    {
        public string Name;
        public List<Chain> Chains;

        public Model(string name)
        {
            this.Name = name;
        }

        public Model()
        {

        }
    }

    /// <summary>
    /// A set of DH Joints implicitly connected end-to-end, forming a chain.
    /// </summary>
    [Serializable]
    public class Chain
    {
        public string Name;
        public List<Joint> Joints;

        public Chain(string name)
        {
            this.Name = name;
        }

        public Chain()
        {

        }
    }

    /// <summary>
    /// Contains the parameters for a Denavit-Hartenberg Joint/Link.
    /// By convention units are meters and degrees.
    /// Currently, the joint is limited around the major rotation axis only, and other axes are assumed to be fixed, 
    /// but this is not enforced.
    /// </summary>
    [Serializable]
    public class Joint
    {
        // In c# value types are initialised to their default (which for numbers is zero) so only non-zero parameters
        // need to be set

        public string Name;

        /// <summary>
        /// (Alpha) rotation around common normal (a.k.a Roll)
        /// </summary>
        public float a;

        /// <summary>
        /// (Theta) angle of joint (the main one) around the hinge axis (a.k.a Yaw)
        /// </summary>
        public float th;

        /// <summary>
        /// Offset along the hinge (lift)
        /// </summary>
        public float d;

        /// <summary>
        /// Link segment length (along the common normal)
        /// </summary>
        public float r;

        /// <summary>
        /// Limits on the rotation of the joint
        /// </summary>
        public float min_th;

        /// <summary>
        /// Limits on the rotation of the joint
        /// </summary>
        public float max_th;

        /// <summary>
        /// The pose angle between the min and max limits
        /// </summary>
        public float NormalisedPoseAngle
        {
            set
            {
                th = min_th + (max_th - min_th) * (1-value);
            }
            get
            {
                return (th + min_th) / (max_th - min_th);
            }
        }

        public Joint(string name)
        {
            this.Name = name;
        }

        public Joint()
        {

        }

        public void Endpoint(float x, float y, float z)
        {
            this.r = Parameter.Length(x, y);
            this.th = Parameter.Angle(x, y);
            this.d = z;
        }

        /// <summary>
        /// Set the parameters so that the endpoint appears x units forward, and y units right, from the parent
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        public void Endpoint(float x, float y)
        {
            this.r = Parameter.Length(x, y);
            this.th = Parameter.Angle(x, y);
        }

        public void LengthXY(float x, float y)
        {
            this.r = Parameter.Length(x, y);
        }

        public void LengthTwist(float r, float a)
        {
            this.r = r;
            this.a = a;
        }

        public void Twist(float a)
        {
            this.a = a;
        }

        public void Length(float r)
        {
            this.r = r;
        }

        public void Angle(float angle)
        {
            this.th = angle;
        }

        public void Translate(float d)
        {
            this.d = d;
        }

        /// <summary>
        /// Sets the translation so that the endpoint is angle degrees up or down in front of this joint. Warning: this
        /// method should only be called once the length has been set!
        /// </summary>
        /// <param name="angle"></param>
        public void Pitch(float angle)
        {
            float h = this.r;

            if(h == 0)
            {
                throw new Exception("Attempting to set pitch with a segment length of zero.");
            }

            this.d = Mathf.Sin(angle * Mathf.Deg2Rad) * h;
        }

        /// <summary>
        /// Sets the Limits of the joints motion relative to the current value. This is also not a clamp. The angle direction 
        /// is given by the sign, negative for counterclockwise, positive for clockwise, so these angles are the extent in those directions.
        /// </summary>
        /// <param name="below"></param>
        /// <param name="above"></param>
        public void RelativeLimits(float below, float above)
        {
            min_th = th + -below;
            max_th = th + above;
        }

    }
}
