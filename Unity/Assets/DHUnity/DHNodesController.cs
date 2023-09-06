using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.Haptics
{
    /// <summary>
    /// Creates a GameObject graph representing the DH chains in model, and keeps its transforms up to date
    /// with the DH link parameters.
    /// Each DHNode maintains a reference to a Joint, so this graph can be used to update the model.
    /// The Model should be provided by a DHModelController that has overridden the GetModel() method.
    /// </summary>
    [ExecuteInEditMode]
    public class DHNodesController : MonoBehaviour
    {
        public DH.Model Model
        {
            get
            {
                var controller = GetComponent<IDHModelController>();
                if (controller != null)
                {
                    return controller.GetModel();
                }
                return null;
            }
        }

        // Update is called once per frame
        void Update()
        {
            /*
            if (Model != null)
            {
                foreach (var digit in Model.Chains)
                {
                    UpdateChain(digit);
                }
            }
            */
        }

        void UpdateChain(DH.Chain chain)
        {
            var start = this.transform.Find(chain.Name);
            if(start == null)
            {
                var startgameobject = new GameObject(chain.Name);
                start = startgameobject.transform;
                start.SetParent(this.transform, false);
            }

            UpdateChain(start, chain.Joints);
        }

        void UpdateChain(Transform parent, IEnumerable<DH.Joint> joints)
        {
            GameObject lastEndpoint = null;

            // make sure there are nodes the chain, and they have the right references

            foreach (var joint in joints)
            {
                if(lastEndpoint == null)
                {
                    // find the starting node, or create one
                    var lastEndpointTransform = parent.Find(joint.Name);
                    if(lastEndpointTransform == null)
                    {
                        var newStartingNode = new GameObject(joint.Name);
                        newStartingNode.transform.parent = parent;
                        lastEndpointTransform = newStartingNode.transform;
                    }

                    lastEndpoint = lastEndpointTransform.gameObject;
                }

                // this should be the node for the current joint
                var node = lastEndpoint.GetComponent<DH.DHJointLink>();
                if (node == null)
                {
                    node = lastEndpoint.AddComponent<DH.DHJointLink>();
                }

                node.joint = joint;
                lastEndpoint = node.Endpoint;
            }
        }

        /// <summary>
        /// Applies all the DH Joint parameters to the DHNode GameObject transforms immediately (as in, before the method returns)
        /// </summary>
        public void UpdateImmediate()
        {
            UpdateImmediate(gameObject);
        }

        // breadth first traversal.
        private static void UpdateImmediate(GameObject parent)
        {
            foreach(DH.DHJointLink node in parent.GetComponents<DH.DHJointLink>())
            {
                node.Update();
            }
            foreach(Transform child in parent.transform)
            {
                UpdateImmediate(child.gameObject);
            }
        }


    }
}
