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

        /// <summary>
        /// Update node based on the observed state of next
        /// </summary>
        public void Forwards(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Get the rotation as two angles that will be constrained

            var s = CartesianToSpherical(localDirection);

            // Constrain a & b

            if (localDirection.z < 0)
            {
                s.polar = Mathf.PI;
            }
            else
            {
                s.polar = 0;
            }

            DebugDrawText.Draw("\n\n\n" + s.ToString(), node.position);

            // This is the identity operation for this Joint, for debugging purposes
            var localDirectionP = localDirection;

            localDirectionP = SphericalToCartesian(s);

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

        /// <summary>
        /// Update next based on the observed state of node
        /// </summary>
        public void Backwards(Node node, Node next, Node prev, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Get the rotation as two angles that will be constrained

            var s = CartesianToSpherical(localDirection);

            // Constrain a & b

            if (localDirection.z < 0)
            {
                s.polar = Mathf.PI;
            }
            else
            {
                s.polar = 0;
            }

            // Recompute the constrained local position

            // This is the identity operation for this Joint, for debugging purposes
            var localDirectionP = localDirection;

            /*
            localDirectionP.x = Mathf.Clamp(localDirection.x, -0.2f, 0.2f);
            localDirectionP.y = Mathf.Clamp(localDirection.y, -0.3f, 0.3f);
            localDirectionP.z = Mathf.Clamp(localDirection.z, 0, 1f);
            localDirectionP.Normalize();
            */

            localDirectionP = SphericalToCartesian(s);

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

        public struct SphericalCoordinates
        {
            public float polar;
            public float elevation;

            public override string ToString()
            {
                return $"{polar} {elevation}";
            }
        }

        public SphericalCoordinates CartesianToSpherical(Vector3 p)
        {
            SphericalCoordinates coords;
            if (p.x == 0)
            {
                p.x = Mathf.Epsilon;
            }
            coords.polar = Mathf.Atan(p.x / p.z);
            if (p.z < 0)
            {
                coords.polar += Mathf.PI;
            }
            coords.elevation = Mathf.Asin(p.y);
            return coords;
        }

        public static Vector3 SphericalToCartesian(SphericalCoordinates coords)
        {
            Vector3 p;
            p.x = Mathf.Sin(coords.polar) * Mathf.Cos(coords.elevation);
            p.y = Mathf.Sin(coords.elevation);
            p.z = Mathf.Cos(coords.polar) * Mathf.Cos(coords.elevation);
            return p;
        }

        private void OnDrawGizmos()
        {
            //UnityEditor.Handles.Label(transform.position, $"d0: {d0}\nd1:{d1}\no:{o}");

            DebugDrawText.Draw();
        }

        public static class DebugDrawText
        {
            private static Dictionary<Vector3, string> messages = new Dictionary<Vector3, string>();
            private static int frame;

            public static void Draw(string message, Vector3 position)
            {
                messages.Add(position, message);
            }

            public static void Draw()
            {
                if (Time.frameCount != frame)
                {
                    foreach (var item in messages)
                    {
                        UnityEditor.Handles.Label(item.Key, item.Value);
                    }
                    messages.Clear();
                    frame = Time.frameCount;
                }
            }
        }

    }
}