using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public class FabrikJoint2DB : FabrikJoint
    {
        public Vector2 range_x;
        public Vector2 range_y; // In degrees

        private Plane[] planes = new Plane[4];

        protected override Vector3 GetNextPosition(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Constrain the angles as a sequence of projections onto planes
            // defining a frustrum

            var localDirectionP = localDirection;

            planes[0] = GetPlaneLeft();
            planes[1] = GetPlaneRight();
            planes[2] = GetPlaneUp();
            planes[3] = GetPlaneDown();

            for (int i = 0; i < 4; i++)
            {
                if (planes[i].GetSide(localDirection))
                {
                    localDirectionP = planes[i].ClosestPointOnPlane(localDirectionP);
                }
            }

            // Transform this constrained position back into world space

            var localPositionP = localDirectionP.normalized * d;
            var worldPositionP = (node.rotation * localPositionP) + node.position;

            return worldPositionP;
        }

        private Plane GetPlaneLeft()
        {
            return new Plane(Quaternion.AngleAxis(range_y.x - 90, Vector3.up) * Vector3.forward, 0);
        }

        private Plane GetPlaneRight()
        {
            return new Plane(Quaternion.AngleAxis(range_y.y + 90, Vector3.up) * Vector3.forward, 0);
        }

        private Plane GetPlaneUp()
        {
            return new Plane(Quaternion.AngleAxis(range_x.x - 90, Vector3.right) * Vector3.forward, 0);
        }

        private Plane GetPlaneDown()
        {
            return new Plane(Quaternion.AngleAxis(range_x.y + 90, Vector3.right) * Vector3.forward, 0);
        }

        private Vector3 FromAngle(float angle, Vector3 axis)
        {
            return Quaternion.AngleAxis(angle, axis) * Vector3.forward;
        }

        private List<Vector3> DrawArc(Vector2 range, Vector3 axis)
        {
            List<Vector3> points = new List<Vector3>();
            float stepSize = 1f;
            for (float i = range.x; i < range.y; i += stepSize)
            {
                points.Add(FromAngle(i, axis));
            }
            points.Add(FromAngle(range.y, axis));
            points.Add(Vector3.zero);
            return points;
        }

        public override void DrawGizmos(Node node)
        {
            //Todo: this breaks for constraints greater than 90 deg - though the constraints do work...

            Gizmos.color = Color.yellow;

            var X = DrawArc(range_x, Vector3.right);
            var Y = DrawArc(range_y, Vector3.up);

            for (int i = 0; i < X.Count; i++)
            {
                X[i] = node.rotation * (X[i] * gizmoScale) + node.position;
            }

            for (int i = 0; i < Y.Count; i++)
            {
                Y[i] = node.rotation * (Y[i] * gizmoScale) + node.position;
            }

            Gizmos.DrawLineStrip(new ReadOnlySpan<Vector3>(X.ToArray()), true);
            Gizmos.DrawLineStrip(new ReadOnlySpan<Vector3>(Y.ToArray()), true);

            base.DrawGizmos(node);
        }
    }
}