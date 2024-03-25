using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Socket1D : MonoBehaviour
{
    public Vector2 range;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos()
    {

    }

    // https://www.gamedev.net/forums/topic/696882-swing-twist-interpolation-sterp-an-alternative-to-slerp/
    // https://www.euclideanspace.com/maths/geometry/rotations/for/decomposition/forum.htm
    // This implementation is credited to Michaele Norel
    public static void SwingTwistDecomposition(Quaternion q, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
    {
        Vector3 r = new Vector3(q.x, q.y, q.z);

        // singularity: rotation by 180 degree
        if (r.sqrMagnitude < Mathf.Epsilon)
        {
            Vector3 rotatedTwistAxis = q * twistAxis;
            Vector3 swingAxis =
              Vector3.Cross(twistAxis, rotatedTwistAxis);

            if (swingAxis.sqrMagnitude > Mathf.Epsilon)
            {
                float swingAngle =
                  Vector3.Angle(twistAxis, rotatedTwistAxis);
                swing = Quaternion.AngleAxis(swingAngle, swingAxis);
            }
            else
            {
                // more singularity: 
                // rotation axis parallel to twist axis
                swing = Quaternion.identity; // no swing
            }

            // always twist 180 degree on singularity
            twist = Quaternion.AngleAxis(180.0f, twistAxis);
            return;
        }

        // meat of swing-twist decomposition
        Vector3 p = Vector3.Project(r, twistAxis);
        twist = new Quaternion(p.x, p.y, p.z, q.w);
        twist = Quaternion.Normalize(twist);
        swing = q * Quaternion.Inverse(twist);
    }
}

public static class QuaternionExtensions
{
    public static void SwingTwistDecomposition(this Quaternion q, Vector3 twistAxis, out Quaternion swing, out Quaternion twist)
    {
        Vector3 r = new Vector3(q.x, q.y, q.z);

        // singularity: rotation by 180 degree
        if (r.sqrMagnitude < Mathf.Epsilon)
        {
            Vector3 rotatedTwistAxis = q * twistAxis;
            Vector3 swingAxis =
              Vector3.Cross(twistAxis, rotatedTwistAxis);

            if (swingAxis.sqrMagnitude > Mathf.Epsilon)
            {
                float swingAngle =
                  Vector3.Angle(twistAxis, rotatedTwistAxis);
                swing = Quaternion.AngleAxis(swingAngle, swingAxis);
            }
            else
            {
                // more singularity: 
                // rotation axis parallel to twist axis
                swing = Quaternion.identity; // no swing
            }

            // always twist 180 degree on singularity
            twist = Quaternion.AngleAxis(180.0f, twistAxis);
            return;
        }

        // meat of swing-twist decomposition
        Vector3 p = Vector3.Project(r, twistAxis);
        twist = new Quaternion(p.x, p.y, p.z, q.w);
        twist = Quaternion.Normalize(twist);
        swing = q * Quaternion.Inverse(twist);
    }

    public static float Twist(this Quaternion q, Vector3 twistAxis)
    {
        Vector3 r = new Vector3(q.x, q.y, q.z);

        Quaternion twist;

        // singularity: rotation by 180 degree
        if (r.sqrMagnitude < Mathf.Epsilon)
        {
            // always twist 180 degree on singularity
            return 180.0f;
        }

        // meat of swing-twist decomposition
        Vector3 p = Vector3.Project(r, twistAxis);
        twist = new Quaternion(p.x, p.y, p.z, q.w);
        twist = Quaternion.Normalize(twist);
        float angle;
        Vector3 axis;
        twist.ToAngleAxis(out angle, out axis);
        return angle;
    }
}
