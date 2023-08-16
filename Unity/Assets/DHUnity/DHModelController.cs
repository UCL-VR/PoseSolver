using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.Haptics
{
    public interface IDHModelController
    {
        void ResetModel();
        DH.Model GetModel();
    }

    /// <summary>
    /// The DHModelController allows a DH Model to be built programatically, as opposed to
    /// loaded from a file.
    /// To use this, subclass DHModelController and override ResetModel().
    /// A Custom Inspector has already been provided, but it can be overridden as well.
    /// </summary>
    [RequireComponent(typeof(DHNodesController))]
    public abstract class DHModelController : MonoBehaviour, IDHModelController
    {
        public virtual void ResetModel()
        {

        }

        public abstract DH.Model GetModel();
    }
}