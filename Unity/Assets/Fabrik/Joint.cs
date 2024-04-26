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

    /// <summary>
    /// Joints are always defined at the center of rotation. GameObjects that
    /// branch may have multiple Joint Components.
    /// </summary>
    public class Joint : MonoBehaviour, IJointConstraint
    {
        public Transform Next;
        public JointTypes Type;

        public Vector3 Normal;

        private void Reset()
        {
            Next = transform.GetChild(0);
        }

        public HingeJoint Settings;

        public bool apply = true;

        // Orientation constraints are applied by decomposing the Position into
        // two Angles (Yaw and Pitch), relative to node, and limiting them to a
        // range.

        // The orientation of a Node is always aligned with the vector from its
        // parent towards it. This is the frame in which the angles of the child
        // are computed.

        // Both stages work by getting the local position of node next in its
        // parent node, constraining the local position, then transforming it
        // back into world space. However, whereas in Backwards this constrained
        // world space position is considered to be the new position of next,
        // in Forwards the difference between the new position and the old one
        // is applied to node.

        public void Forwards(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Compute two angles in orthogonal planes aligned with the parent's
            // orientation. We could also write a version of this that uses one
            // plane with an arbitrary angle.

            var a = Mathf.Atan2(localDirection.x, localDirection.z);
            var b = Mathf.Atan2(localDirection.y, localDirection.z);

            // Constrain a & b

            // a = 0;

            // Recompute the constrained local position. Some identities that may be helpful...
            // Atan2(0,1) == 0
            // Atan2(1,0) == 1
            // Cos(0) == 1
            // Sin(0) == 0
         
            // This is the identity operation for this Joint, for debugging purposes
            var localDirectionP = localDirection;

            localDirectionP.x = Mathf.Sin(a) * Mathf.Cos(b);
            localDirectionP.y = Mathf.Sin(b) * Mathf.Cos(a);
            localDirectionP.z = Mathf.Cos(b);
            localDirectionP.Normalize();

            // Both of the following snippets work...

            /*
            var plane = new Plane(Normal, 0); // Everything is done in local space
            localDirectionP = plane.ClosestPointOnPlane(localDirection).normalized;

            //localDirectionP = localDirection;
            //localDirectionP.y = 0;
            //localDirectionP.Normalize();
            */

            var localPositionP = localDirectionP * d;

            // Transform this constrained position back into world space

            var worldPositionP = (node.rotation * localPositionP) + node.position;

            // In the Forwards/Inwards step we should be updating node to
            // satisfy next, so get the offset between the original and
            // constrained positions and apply the inverse to node.

            var difference = next.position - worldPositionP;
            node.position += difference;
        }

        public void Backwards(Node node, Node next, Node prev, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Compute two angles in orthogonal planes aligned with the parent's
            // orientation. We could also write a version of this that uses one
            // plane with an arbitrary angle.

            var a = Mathf.Atan2(localDirection.x, localDirection.z);
            var b = Mathf.Atan2(localDirection.y, localDirection.z);

            // Constrain a & b

            // a = 0;

            // Recompute the constrained local position

            // This is the identity operation for this Joint, for debugging purposes
            var localDirectionP = localDirection;

            localDirectionP.x = Mathf.Sin(a) * Mathf.Cos(b);
            localDirectionP.y = Mathf.Sin(b) * Mathf.Cos(a);
            localDirectionP.z = Mathf.Cos(b);
            localDirectionP.Normalize();

            // Both of the following snippets work...

            /*
            localDirectionP = localDirection;
            localDirectionP.y = 0;
            localDirectionP.Normalize();

            var plane = new Plane(Normal, 0); // Everything is done in local space
            localDirectionP = plane.ClosestPointOnPlane(localDirection).normalized;
            */

            var localPositionP = localDirectionP * d;

            // This is the identity operation for this Joint, for debugging purposes
            // localPositionP = localDirection * d;

            // Transform this constrained position back into world space

            var worldPositionP = (node.rotation * localPositionP) + node.position;

            // In the Backwards/Outwards step we should be updating next to
            // satisfy node.

            next.position = worldPositionP;
        }


        private void OnDrawGizmos()
        {
            //UnityEditor.Handles.Label(transform.position, $"d0: {d0}\nd1:{d1}\no:{o}");
        }
    }
}