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
        public Vector3 Forward;
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

            // Get the angle around the axis

            var right = Vector3.Cross(Axis, Forward);

            var x = Vector3.Dot(localDirectionP, right);
            var y = Vector3.Dot(localDirectionP, Forward);
            var angle = Mathf.Atan2(x, y); // Nb the order of x and y is deliberate here, as this puts 0 on the forward axis (with +ve clockwise when looking down and vice versa)

            angle = Mathf.Clamp(angle, Range.x, Range.y);

            localDirectionP = FromAngle(angle);

            DebugDrawText(angle.ToString());

            // Transform this constrained position back into world space

            var localPositionP = localDirectionP * d;
            var worldPositionP = (node.rotation * localPositionP) + node.position;

            return worldPositionP;
        }

        private Vector3 FromAngle(float angle)
        {
            var right = Vector3.Cross(Axis, Forward);
            var x = Mathf.Sin(angle);
            var y = Mathf.Cos(angle);
            return right * x + Forward * y;
        }

        public override void DrawGizmos(Node node)
        {
            var scale = 0.01f;

            Gizmos.color = Color.white;
            //Gizmos.DrawRay(node.position, node.rotation * Axis.normalized * scale);

            Gizmos.color = Color.yellow;
            //Gizmos.DrawRay(node.position, node.rotation * Forward.normalized * scale);

            //Gizmos.DrawRay(node.position, node.rotation * Forward.normalized * scale);

            Gizmos.color = Color.yellow;
//            Gizmos.DrawRay(node.position, node.rotation * FromAngle(Range.x) * scale);
          //  Gizmos.DrawRay(node.position, node.rotation * FromAngle(Range.y) * scale);

            // Arc points

            List<Vector3> points = new List<Vector3>();

            points.Add(node.position);
            var step = 0.01f;
            for (float th = Range.x; th < Range.y ; th += step)
            {
                points.Add(node.position + node.rotation * FromAngle(th) * scale);
            }
            points.Add(node.position + node.rotation * FromAngle(Range.y) * scale);

            Gizmos.DrawLineStrip(new Span<Vector3>(points.ToArray()), true);

            base.DrawGizmos(node);
        }
    }
}
