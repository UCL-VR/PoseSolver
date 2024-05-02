using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// Positions are rigidly attached to their parent
    /// </summary>
    public class Rigid : FabrikJoint
    {
        Vector3 localPosition;

        public override void Initialise(Node node, Node next)
        {
            localPosition = Quaternion.Inverse(node.rotation) * (next.position - node.position);
        }

        protected override Vector3 GetNextPosition(Node node, Node next, float d)
        {
            var worldPositionP = (node.rotation * localPosition) + node.position;
            return worldPositionP;
        }
    }
}