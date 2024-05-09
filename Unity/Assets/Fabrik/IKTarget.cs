using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    public class IKTarget : MonoBehaviour, IConstraint
    {
        public Transform EndEffector;
        public Node Node { get; private set; }

        public void Project()
        {
            Node.position = transform.position;
            Node.updated = true;
        }

        private void Start()
        {
            var solver = EndEffector.GetComponentInParent<FabrikSolver>();
            Node = solver.GetEffector(EndEffector);
            solver.AddConstraint(this);
        }
    }
}