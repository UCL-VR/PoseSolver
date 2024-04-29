using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// Implements a Hinge Joint around an arbitrary Axis based on projection
    /// into a Plane.
    /// </summary>
    public class FabrikJoint1D : FabrikJoint
    {
        public Vector3 Axis;
        public Vector2 Range;

        protected override Vector3 GetNextPosition(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Recompute the constrained local position

            var plane = new Plane(Axis.normalized, 0);
            var localDirectionP = plane.ClosestPointOnPlane(localDirection).normalized;



            // Transform this constrained position back into world space

            var localPositionP = localDirectionP * d;
            var worldPositionP = (node.rotation * localPositionP) + node.position;

            return worldPositionP;
        }

        public override void DrawGizmos(Node node)
        {
            var scale = 0.01f;

            Gizmos.color = Color.white;
            Gizmos.DrawRay(node.position, node.rotation * Axis.normalized * scale);

            //Gizmos.DrawRay(node.position, node.ro)
        }
    }
}
