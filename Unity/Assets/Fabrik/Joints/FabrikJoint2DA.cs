using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// Implements a Fabrik Joint with Two Degrees of Freedom based on Spherical
    /// Coordinates.
    /// </summary>
    public class FabrikJoint2DA : FabrikJoint
    {
        public Vector2 PolarRange;
        public Vector2 ElevationRange;

        /// <summary>
        /// Update node based on the observed state of next
        /// </summary>
        public override void Forwards(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Get the rotation as two angles that will be constrained

            var s = CartesianToSpherical(localDirection);

            // Constrain a & b

            if (PolarRange.sqrMagnitude > 0)
            {
                s.polar = Mathf.Clamp(s.polar, PolarRange.x, PolarRange.y);
            }
            if (ElevationRange.sqrMagnitude > 0)
            {
                s.elevation = Mathf.Clamp(s.elevation, ElevationRange.x, ElevationRange.y);
            }

            DebugDrawText.Draw("\n\n\n" + s.ToString(), node.position);

            var localDirectionP = SphericalToCartesian(s);
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
        public override void Backwards(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Get the rotation as two angles that will be constrained

            var s = CartesianToSpherical(localDirection);

            // Constrain a & b

            if (PolarRange.sqrMagnitude > 0)
            {
                s.polar = Mathf.Clamp(s.polar, PolarRange.x, PolarRange.y);
            }
            if (ElevationRange.sqrMagnitude > 0)
            {
                s.elevation = Mathf.Clamp(s.elevation, ElevationRange.x, ElevationRange.y);
            }

            // Recompute the constrained local position

            // Spherical Coordinates
            var localDirectionP = SphericalToCartesian(s);
            var localPositionP = localDirectionP * d;

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
    }
}