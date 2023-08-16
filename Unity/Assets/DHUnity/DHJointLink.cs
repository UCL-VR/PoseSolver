using UnityEngine;

namespace UCL.CASMS.DH
{
    /// <summary>
    /// Represents both a Joint and a Link in a Denavit-Hartenberg Chain, which
    /// maintain the Unity Transforms involved.
    /// 
    /// While it is possible to represent a the Joints of a DH Chain with a
    /// sequence of traditional transform matrices, the centers of rotations
    /// of the joints will not line up with what would be expected of a
    /// traditional armature/skeleton.
    /// 
    /// Each DHJointLink controls the transforms of both the starting and ending
    /// transforms. This means that branching is not supported.
    /// </summary>
    [ExecuteInEditMode]
    public class DHJointLink : MonoBehaviour
    {
        [SerializeField]
        public Joint joint;

        public System.IntPtr Reference { get; set; }

        public GameObject Endpoint
        {
            get
            {
                if (endpoint == null)
                {
                    foreach (Transform t in transform)
                    {
                        if (t.gameObject.GetComponent<DHJointLink>())
                        {
                            endpoint = t.gameObject;
                            break;
                        }
                    }
                }

                // Create the endpoint if we really can't find one
                if (endpoint == null)
                { 
                    endpoint = new GameObject("Endpoint");
                    endpoint.transform.parent = this.transform;
                }

                return endpoint;
            }
        }
        [SerializeField]
        private GameObject endpoint;

        public float Distance
        {
            get
            {
                return Mathf.Sqrt(joint.r * joint.r + joint.d * joint.d);
            }
        }

        // Update is called once per frame
        public void Update()
        {
            // update the name of this node (in case we are a child of another gameobject currently called endpoint)

            if (joint.Name != null)
            {
                name = joint.Name;
            }
            else
            {
                joint.Name = name;
            }

            SetTransform();
        }

        /// <summary>
        /// Updates the transform of this node with the specified DH parameters
        /// </summary>
        void SetTransform()
        {
            var a = 0f;

            if (transform.parent != null)
            {
                var parentnode = transform.parent.gameObject.GetComponent<DHJointLink>();
                if (parentnode != null)
                {
                    a = parentnode.joint.a;
                }
            }

            // Update this node's rotation
            transform.localRotation = Quaternion.AngleAxis(a, Vector3.forward) * Quaternion.AngleAxis(joint.th, Vector3.up);

            // Update the endpoints position
            Endpoint.transform.localPosition = (Vector3.forward * joint.r) + (Vector3.up * joint.d);

            // Does the endpoint need its inclination adjusted, or will it do it itself?
            var remotenode = Endpoint.GetComponent<DHJointLink>();
            if (remotenode == null)
            {
                Endpoint.transform.localRotation = Quaternion.AngleAxis(joint.a, Vector3.forward);
            }
        }

        /// <summary>
        /// Updates the DH parameters of this node based on the actual transforms
        /// </summary>
        void GetTransform()
        {
            joint.th = transform.localRotation.eulerAngles.y;
            joint.a = Endpoint.transform.localRotation.eulerAngles.z;
            joint.r = Endpoint.transform.localPosition.z;
            joint.d = Endpoint.transform.localPosition.y;
        }

        /// <summary>
        /// Updates the Joint parameters so that the Endpoint matches the provided position
        /// </summary>
        /// <param name="position"></param>
        public void SetPosition(Vector3 position)
        {
            if(Parent)
            {
                Parent.SetEndPosition(position);
            }
        }

        public void SetEndPosition(Vector3 position)
        {
            var p = transform.InverseTransformPoint(position);

            // DH joints must point torwards the endpoint.
            // First rotate the joint so that it does so.

            var rotation = Quaternion.FromToRotation(transform.forward, position - transform.position);
            var around = GetRotationAroundAxis(rotation, transform.up);

            float angle;
            Vector3 axis;
            around.ToAngleAxis(out angle, out axis);

            joint.th -= angle;

            SetTransform();


            // When we can simply set the local offsets in the up and forward vectors.

            joint.r = p.z;
            joint.d = p.y;
        }

        // Credit Luke Hutchison, Minorlogic
        private static Quaternion GetRotationAroundAxis(Quaternion rotation, Vector3 direction)
        {
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            var dotProd = Vector3.Dot(direction, rotationAxis);

            Vector3 projection = direction * dotProd;

            Quaternion twist = new Quaternion(
                    projection.x, projection.y, projection.z, rotation.w).normalized;
            if (dotProd < 0.0)
            {
                twist.x = -twist.x;
                twist.y = -twist.y;
                twist.z = -twist.z;
                twist.w = -twist.w;
            }
            return twist;
        }

        public Vector3 EndPosition => transform.position + transform.forward * joint.r + transform.up * joint.d;

        public DHJointLink Parent
        {
            get
            {
                if (!transform.parent)
                {
                    return null;
                }
                else
                {
                    return transform.parent.GetComponent<DHJointLink>();
                }
            }
        }
    }
}
