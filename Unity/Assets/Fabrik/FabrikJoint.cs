using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// Implements the base class for all Fabrik Joints. Joints are always
    /// defined at the center of rotation. GameObjects that branch may have
    /// multiple Joint Components, but their Next members should all be
    /// different.
    /// This class should only be used for debugging - use one of the subclases
    /// to actually constrain a chain.
    /// </summary>
    public class FabrikJoint : MonoBehaviour, IJointConstraint
    {        
        public Transform Next;

        private void Reset()
        {
            Next = transform.GetChild(0);
        }

        public virtual bool Enabled => enabled;

        public virtual void Initialise(Node node, Node next)
        {
        }

        // Orientation constraints are applied by getting the local direction of
        // a child node, and limiting this to be within a rotation. How this is
        // done is up to the subclass.

        // The orientation of a Node is always aligned with the vector from its
        // parent towards it. This is the frame in which the angles of the child
        // are computed.

        // Both stages work by getting the local position of node next in its
        // parent node, constraining the local position, then transforming it
        // back into world space. However, whereas in Backwards this constrained
        // world space position is considered to be the new position of next,
        // in Forwards the difference between the new position and the old one
        // is applied to node.

        // This superclass implements an identity operation - that is no
        // constraints are applied, but the nodes are updated based on
        // localDirection (instead of the traditional Fabrik algorithm),
        // providing a skeleton on which to implement joint constraints.
        // In most cases, subclasses only need to override GetNextPosition.

        /// <summary>
        /// Get the position of next given the constraints of node. How this
        /// position is used to update node or next, depends on whether it is
        /// used in Forwards or Backwards.
        /// </summary>
        protected virtual Vector3 GetNextPosition(Node node, Node next, float d)
        {
            // Transform into local space and get the direction with which to
            // compute the local orientation from position.

            var localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
            var localDirection = localPosition.normalized;

            // Constrain localDirection here...

            // This is the identity operation - we take localDirection as-is

            var localDirectionP = localDirection;
            var localPositionP = localDirectionP * d;

            // Transform this constrained position back into world space

            var worldPositionP = (node.rotation * localPositionP) + node.position;

            return worldPositionP;
        }

        /// <summary>
        /// Update node based on the observed state of next
        /// </summary>
        public virtual void Forwards(Node node, Node next, float d)
        {
            // Get the position of next that satisfies the constraints of node,
            // in world space.

            var nextPosition = GetNextPosition(node, next, d);

            // In the Forwards/Inwards step we should be updating node to
            // satisfy next, so get the offset between the original and
            // constrained positions and apply the inverse to node.

            var difference = next.position - nextPosition;
            node.position += difference;
        }

        /// <summary>
        /// Update next based on the observed state of node
        /// </summary>
        public virtual void Backwards(Node node, Node next, float d)
        {
            // Get the position of next that satisfies the constraints of node,
            // in world space.

            var nextPosition = GetNextPosition(node, next, d);

            // In the Backwards/Outwards step we should be updating next to
            // satisfy node.

            next.position = nextPosition;
        }

        public virtual void DrawGizmos(Node node)
        {
            var s = "";
            foreach (var item in messages)
            {
                s += "\n" + item;
            }
            messages.Clear();
            UnityEditor.Handles.Label(node.position, s);
        }

        protected void DebugDrawText(string message)
        {
            messages.Add(message);
        }

        private List<string> messages = new List<string>();
        protected float gizmoScale = 0.01f;
    }
}