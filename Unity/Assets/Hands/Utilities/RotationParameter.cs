using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.Haptics
{
    [ExecuteInEditMode]
    public class RotationParameter : MonoBehaviour
    {
        public Quaternion min;
        public Quaternion max;

        [Range(0,1)]
        public float value;

        private void Reset()
        {
            min = transform.localRotation;
            max = transform.localRotation;
        }

        public Quaternion ParentRotation
        {
            get
            {
                if(transform.parent != null)
                {
                    return transform.parent.rotation;
                }
                else
                {
                    return Quaternion.identity;
                }
            }
        }

        // Update is called once per frame
        public void Update()
        {
            transform.localRotation = Quaternion.Slerp(min, max, value);    
        }
    }
}