using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Ubiq.Fabrik
{
    /// <summary>
    /// Estimates the Orientation of a FABRIK root from three points
    /// </summary>
    [RequireComponent(typeof(FabrikSolver))]
    public class FabrikOrientation : MonoBehaviour
    {
        public Transform point1;
        public Transform point2;
        public Transform point3;

        public Vector3 Forward => (point2.position - point1.position).normalized;
        public Vector3 Right => (point3.position - point1.position).normalized;
        public Vector3 Up => Vector3.Cross(Right, Forward).normalized;
        public Quaternion Rotation => Quaternion.LookRotation(Forward, Up);
        public Vector3 Position => point1.position;

        private FabrikSolver solver;
        private Vector3 localPosition;
        private Quaternion localRotation;

        private void Awake()
        {
            solver = GetComponent<FabrikSolver>();

        }

        // Start is called before the first frame update
        void Start()
        {
            var node = solver.model.root.node;
            localRotation = Quaternion.Inverse(Rotation) * Quaternion.identity; // Replace with node.rotation, if we ever initialise it in Awake
            localPosition = Quaternion.Inverse(Rotation) * (node.position - Position);
            solver.OnUpdateRoot.AddListener(OnUpdateRoot);
        }

        void OnUpdateRoot(Node root)
        {
            root.position = Position + Rotation * localPosition;
            root.rotation = Rotation * localRotation;
        }


        // Update is called once per frame
        void Update()
        {

        }
    }
}
