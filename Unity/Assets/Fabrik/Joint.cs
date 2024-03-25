using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public enum JointTypes
    {
        Hinge
    }

    [Serializable]
    public class HingeJoint
    {
        /// <summary>
        /// The axis around which the Joint rotates. This is in the local space
        /// of the parent Node - that is, after any orientation is applied.
        /// </summary>
        public Vector3 Hinge;

        /// <summary>
        /// The Range in Degrees of the Joint. The limits are given relative to
        /// the starting rotation of the Line.
        /// </summary>
        public Vector2 Range;
    }

    public class Joint : MonoBehaviour, IJointConstraint
    {
        public Transform Next;
        public JointTypes Type;

        private void Reset()
        {
            Next = transform.GetChild(0);
        }

        public HingeJoint Settings;

        public bool apply = true;

        // Each Node has a Position and Rotation. The Rotation can be imagined
        // as a local rotation applied at the *end* of a segment, where the
        // parent's rotation is always aligned with the direction of the line.

        // In this way, Leaf nodes can also have Rotations, which can be
        // important for mapping to skeletons for skinning or creating IK targets
        // from devices such as VR controllers.

        // In the Forwards stage, the world position and rotation of node are
        // observed. The local rotation, and the position and rotation of prev
        // must be found, so that the final transform is as close as possible
        // to target.

        public void Forwards(Node node, Node prev, float d)
        {
            // Get the ideal rotation of prev. This is the rotation that moves
            // the line onto target, and twists, where necessary.

            var q = Quaternion.LookRotation(node.position - prev.position);

            Debug.DrawLine(prev.position, prev.position + q * Vector3.forward * 0.01f, Color.blue);

            // Get the local rotation around node, that goes from prev's rotation
            // (i.e. along the axis from prev to node) to the final rotation of
            // node. This is what will be constrained.

            var local = Quaternion.Inverse(q) * node.rotation;

            Debug.DrawLine(node.position, node.position + q * local * Vector3.forward * 0.01f, Color.yellow);

            if (apply)
            {
                // Limit the rotation around node to be within the constraints
                // Doing nothing here is the identity operation and results in a free joint

                // Decompose around the right vector

                Quaternion swing;
                Quaternion twist;
                SwingTwistDecomposition(local, Vector3.right, out swing, out twist);
                local = twist;
                //   local = swing * twist; // Identity operation

                // This shows different behaviour to SwingTwistDecomposition, but is not
                // intuitive to control so is unsuitable to be the implementation of
                // constraints.

                //var euler = local.eulerAngles;
                //euler.z = 0;
                //local = Quaternion.Euler(euler);

                Debug.DrawLine(node.position, node.position + q * local * Vector3.forward * 0.01f, Color.white);
            }

            // Find a new starting rotation of previous that makes sure node ends
            // up in the same place, even without all of local being applied.

            var q1 = node.rotation * Quaternion.Inverse(local);

            Debug.DrawLine(prev.position, prev.position + q1 * Vector3.forward * 0.01f, Color.green);

            // Set prev's position by finding the vector it will point to node
            // along, and projecting it back along this vector by d.

            var p3 = node.position + q1 * Vector3.back * d;

            Debug.DrawLine(p3, node.position, Color.red);

            prev.position = p3;
            prev.rotation = q1.normalized;

            node.rotation = (q1 * local).normalized;
        }

        public void Backwards(Node node, Node next, Node prev, float d)
        {
            if(prev != null)
            {
                var local = Quaternion.Inverse(prev.rotation) * node.rotation;

                // Limit local - doing nothing here is the identity operation and results in a free joint

                if (apply)
                {
                    Quaternion swing;
                    Quaternion twist;
                    SwingTwistDecomposition(local, Vector3.right, out swing, out twist);
                    local = swing * twist; // Identity operation - let the constraints be handled by Forwards for now
                }

                // And re-apply

                node.rotation = prev.rotation * local;
            }
            
            next.position = node.position + node.rotation * Vector3.forward * d;
        }

        private void OnDrawGizmos()
        {
            //UnityEditor.Handles.Label(transform.position, $"d0: {d0}\nd1:{d1}\no:{o}");
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
}