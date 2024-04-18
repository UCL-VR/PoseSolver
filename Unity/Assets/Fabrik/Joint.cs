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

        // Orientation constraints are applied by decomposing the Position into
        // two Angles (Yaw and Pitch), relative to prev, and limiting them to a
        // range.

        // The orientation of a Node is always aligned with the vector from its
        // parent towards it. This is the frame in which the angles of the child
        // are computed.

        // Both stages work by getting the local position of a node in its
        // parent, constraining the local position, then transforming it back
        // into world space. However, whereas in Backwards this constrained
        // world space position is considered to be the new position of the node,
        // in Forwards the difference between the new position and the old one
        // is applied to prev.

        public void Forwards(Node node, Node prev, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(prev.rotation) * (node.position - prev.position);
            var localDirection = localPosition.normalized;

            // Compute two angles in orthogonal planes aligned with the parent's
            // orientation. We could also write a version of this that uses one
            // plane with an arbitrary angle.

            var a = Mathf.Atan2(localDirection.x, localDirection.z);
            var b = Mathf.Atan2(localDirection.y, localDirection.z);

            // Constrain a & b

            a = 0;

            // Recompute the constrained local position
            // atan2(y,x) is 0 when x is 1 and y is 0, so cos should go in the component corresponding to the first argument of Atan2

            var localDirectionP = (new Vector3(Mathf.Sin(a), 0, Mathf.Cos(a) * 0.5f) + new Vector3(0, Mathf.Sin(b), Mathf.Cos(b) * 0.5f)).normalized;
            var localPositionP = localDirectionP * d;

            // Transform this constrained position back into world space

            var worldPositionP = (prev.rotation * localPositionP) + prev.position;

            // In the Forwards/Inwards step, we should be updating prev to
            // satisfy node, so get the offset between the original node position
            // and constrained node position and apply its inverse to prev.

            var difference = node.position - worldPositionP;
            prev.position += difference;
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

            a = 0;

            // Recompute the constrained local position

            var localDirectionP = (new Vector3(Mathf.Sin(a), 0, Mathf.Cos(a) * 0.5f) + new Vector3(0, Mathf.Sin(b), Mathf.Cos(b) * 0.5f)).normalized;
            var localPositionP = localDirectionP * d;

            // Transform this constrained position back into world space

            var worldPositionP = (node.rotation * localPositionP) + node.position;

            // In the Backwards/Outwards step, the new position is simply applied

            next.position = worldPositionP;
        }


        private void OnDrawGizmos()
        {
            //UnityEditor.Handles.Label(transform.position, $"d0: {d0}\nd1:{d1}\no:{o}");
        }
    }
}