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

            planes[0] = GetPlaneLeft(node);
            planes[1] = GetPlaneRight(node);
            planes[2] = GetPlaneUp(node);
            planes[3] = GetPlaneDown(node);

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

        private Plane GetPlaneLeft(Node node)
        {
            return new Plane(node.rotation * Quaternion.AngleAxis((range_y.x - Mathf.PI * 0.5f * Mathf.Rad2Deg), Vector3.up) * Vector3.forward, 0);
        }

        private Plane GetPlaneRight(Node node)
        {
            return new Plane(node.rotation * Quaternion.AngleAxis((range_y.y + Mathf.PI * 0.5f * Mathf.Rad2Deg) , Vector3.up) * Vector3.forward, 0);
        }

        private Plane GetPlaneUp(Node node)
        {
            return new Plane(node.rotation * Quaternion.AngleAxis((range_x.x - Mathf.PI * 0.5f * Mathf.Rad2Deg) , Vector3.right) * Vector3.forward, 0);
        }

        private Plane GetPlaneDown(Node node)
        {
            return new Plane(node.rotation * Quaternion.AngleAxis((range_x.y + Mathf.PI * 0.5f * Mathf.Rad2Deg) , Vector3.right) * Vector3.forward, 0);
        }

        public override void DrawGizmos(Node node)
        {
            //Todo: this breaks for constraints greater than 90 deg - though the constraints do work...

            Gizmos.color = Color.yellow;

            planes[0] = GetPlaneLeft(node);
            planes[1] = GetPlaneUp(node);
            planes[2] = GetPlaneRight(node);
            planes[3] = GetPlaneDown(node);

            var directions = new Vector3[4];

            for (int i = 0; i < 4; i++)
            {
                directions[i] = planes[i].ClosestPointOnPlane(Vector3.forward).normalized;
            }

            var positions = new Vector3[4];

            var front = new Plane(Vector3.back, gizmoScale * 0.8f);

            for (int i = 0; i < 4; i++)
            {
                var ray = new Ray(Vector3.zero, directions[i]);
                float d;
                front.Raycast(ray, out d);
                positions[i] = ray.GetPoint(d);
            }

            positions[0].y = positions[1].y;
            positions[1].x = positions[2].x;
            positions[2].y = positions[3].y;
            positions[3].x = positions[0].x;

            for (int i = 0; i < 4; i++)
            {
                positions[i] += node.position;
            }

            foreach (var item in positions)
            {
                Gizmos.DrawLine(node.position, item);
            }

            Gizmos.DrawLineStrip(new ReadOnlySpan<Vector3>(positions), true);

            base.DrawGizmos(node);
        }
    }
}