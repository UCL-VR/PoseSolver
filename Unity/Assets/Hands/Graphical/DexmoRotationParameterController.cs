using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UCL.CASMS.Haptics
{
    public class DexmoRotationParameterController : MonoBehaviour, IDexmoPoseController
    {
        public void UpdateImmediate()
        {
            index0.Update();
            index1.Update();
            index2.Update();
            index3.Update();
            middl0.Update();
            middl1.Update();
            middl2.Update();
            middl3.Update();
            ring_0.Update();
            ring_1.Update();
            ring_2.Update();
            ring_3.Update();
            pinky0.Update();
            pinky1.Update();
            pinky2.Update();
            pinky3.Update();
            thumb1.Update();
            thumb2.Update();
            thumb3.Update();
        }

        public void UpdateParameters(DexmoHandParameters parameters)
        {
            try
            {
                index0.value = parameters.IndexSpread;
                index1.value = parameters.IndexBend;
                index2.value = parameters.IndexBend;
                index3.value = parameters.IndexBend;
                middl0.value = parameters.MiddleSpread;
                middl1.value = parameters.MiddleBend;
                middl2.value = parameters.MiddleBend;
                middl3.value = parameters.MiddleBend;
                ring_0.value = parameters.RingSpread;
                ring_1.value = parameters.RingBend;
                ring_2.value = parameters.RingBend;
                ring_3.value = parameters.RingBend;
                pinky0.value = parameters.LittleSpread;
                pinky1.value = parameters.LittleBend;
                pinky2.value = parameters.LittleBend;
                pinky3.value = parameters.LittleBend;
                thumb1.value = parameters.ThumbSpread;
                thumb2.value = parameters.ThumbBend;
                thumb3.value = parameters.ThumbBend;
            }
            catch (NullReferenceException e)
            {
                Awake(); // try to fix the missing references
                throw e; // still throw it in case this is a persistent error
            }
        }

        [SerializeField] [HideInInspector] private RotationParameter index0;
        [SerializeField] [HideInInspector] private RotationParameter index1;
        [SerializeField] [HideInInspector] private RotationParameter index2;
        [SerializeField] [HideInInspector] private RotationParameter index3;

        [SerializeField] [HideInInspector] private RotationParameter middl0;
        [SerializeField] [HideInInspector] private RotationParameter middl1;
        [SerializeField] [HideInInspector] private RotationParameter middl2;
        [SerializeField] [HideInInspector] private RotationParameter middl3;

        [SerializeField] [HideInInspector] private RotationParameter ring_0;
        [SerializeField] [HideInInspector] private RotationParameter ring_1;
        [SerializeField] [HideInInspector] private RotationParameter ring_2;
        [SerializeField] [HideInInspector] private RotationParameter ring_3;

        [SerializeField] [HideInInspector] private RotationParameter pinky0;
        [SerializeField] [HideInInspector] private RotationParameter pinky1;
        [SerializeField] [HideInInspector] private RotationParameter pinky2;
        [SerializeField] [HideInInspector] private RotationParameter pinky3;

        [SerializeField] [HideInInspector] private RotationParameter thumb1;
        [SerializeField] [HideInInspector] private RotationParameter thumb2;
        [SerializeField] [HideInInspector] private RotationParameter thumb3;

        void Awake()
        {
            index0 = transform.FindDeep("index0").GetComponent<RotationParameter>();
            index1 = transform.FindDeep("index1").GetComponent<RotationParameter>();
            index2 = transform.FindDeep("index2").GetComponent<RotationParameter>();
            index3 = transform.FindDeep("index3").GetComponent<RotationParameter>();
            middl0 = transform.FindDeep("middle0").GetComponent<RotationParameter>();
            middl1 = transform.FindDeep("middle1").GetComponent<RotationParameter>();
            middl2 = transform.FindDeep("middle2").GetComponent<RotationParameter>();
            middl3 = transform.FindDeep("middle3").GetComponent<RotationParameter>();
            ring_0 = transform.FindDeep("ring0").GetComponent<RotationParameter>();
            ring_1 = transform.FindDeep("ring1").GetComponent<RotationParameter>();
            ring_2 = transform.FindDeep("ring2").GetComponent<RotationParameter>();
            ring_3 = transform.FindDeep("ring3").GetComponent<RotationParameter>();
            pinky0 = transform.FindDeep("pinky0").GetComponent<RotationParameter>();
            pinky1 = transform.FindDeep("pinky1").GetComponent<RotationParameter>();
            pinky2 = transform.FindDeep("pinky2").GetComponent<RotationParameter>();
            pinky3 = transform.FindDeep("pinky3").GetComponent<RotationParameter>();
            thumb1 = transform.FindDeep("thumb1").GetComponent<RotationParameter>();
            thumb2 = transform.FindDeep("thumb2").GetComponent<RotationParameter>();
            thumb3 = transform.FindDeep("thumb3").GetComponent<RotationParameter>();
        }

        void Reset()
        {
            Awake();
        }

        // Use this for initialization
        void Start()
        {

        }

        // Update is called once per frame
        void Update()
        {

        }
    }
}