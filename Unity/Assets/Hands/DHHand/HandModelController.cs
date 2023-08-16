using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UCL.CASMS.DH;

namespace UCL.CASMS.Haptics
{
    /// <summary>
    /// Convenience class to host a HandModel as a Unity Component so references can be passed around in the editor.
    /// </summary>
    public class HandModelController : DHModelController, IDexmoPoseController
    {
        public HandModel Model;

        private void OnEnable()
        {
            Model = new HandModel();
        }

        public override void ResetModel()
        {
            Model = new HandModel();
        }

        public override Model GetModel()
        {
            return Model;
        }

        public void UpdateImmediate()
        {
            if(GetComponent<DHNodesController>() != null)
            {
                GetComponent<DHNodesController>().UpdateImmediate();
            }
        }

        public void UpdateParameters(DexmoHandParameters parameters)
        {
            Model.Thumb.CMCabduc.NormalisedPoseAngle = parameters.ThumbRotation;
            Model.Thumb.CMCflex.NormalisedPoseAngle = parameters.ThumbSpread;
            Model.Thumb.MCPflex.NormalisedPoseAngle = parameters.ThumbBend;
            Model.Thumb.IP.NormalisedPoseAngle = parameters.ThumbBend;
            
            Model.Index.MCPabduc.NormalisedPoseAngle = parameters.IndexSpread;
            Model.Index.MCPflex.NormalisedPoseAngle = parameters.IndexBend;
            Model.Index.PIP.NormalisedPoseAngle = parameters.IndexBend;
            Model.Index.DIP.NormalisedPoseAngle = parameters.IndexBend;
            
            Model.Middle.MCPabduc.NormalisedPoseAngle = parameters.MiddleSpread;
            Model.Middle.MCPflex.NormalisedPoseAngle = parameters.MiddleBend;
            Model.Middle.PIP.NormalisedPoseAngle = parameters.MiddleBend;
            Model.Middle.DIP.NormalisedPoseAngle = parameters.MiddleBend;
            
            Model.Ring.MCPabduc.NormalisedPoseAngle = parameters.RingSpread;
            Model.Ring.MCPflex.NormalisedPoseAngle = parameters.RingBend;
            Model.Ring.PIP.NormalisedPoseAngle = parameters.RingBend;
            Model.Ring.DIP.NormalisedPoseAngle = parameters.RingBend;
            
            Model.Little.MCPabduc.NormalisedPoseAngle = parameters.LittleSpread;
            Model.Little.MCPflex.NormalisedPoseAngle = parameters.LittleBend;
            Model.Little.PIP.NormalisedPoseAngle = parameters.LittleBend;
            Model.Little.DIP.NormalisedPoseAngle = parameters.LittleBend;
        }
    }
}