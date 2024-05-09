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
