using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static PoseSolver;

namespace UCL.CASMS
{
    /// <summary>
    /// Given a set of Reference Poses and ImuMarkers, estimate the Imu biases for
    /// each Imu.
    /// </summary>
    [DefaultExecutionOrder(3)] // This must run after DHHandMeasurements
    public class ImuBiasEstimator : MonoBehaviour
    {
        public ImuMarker Index;

        private DHHandMeasurements measurements;

        private ImuBias indexCalibration;

        private void Awake()
        {
            measurements = GetComponent<DHHandMeasurements>();
        }

        void Start()
        {
            if(Index && measurements.Index)
            {
                var stream = new StreamHelper(@"D:\UCL\TrackerFusion\Matlab\biasEstimates.bin");

                List<IntPtr> factors = new List<IntPtr>();
                var indexMotion = Index.GetComponent<MotionFrameHelper>();
                indexCalibration.Ref = addImuBiasParameters();
                Index.OnInertialFrame.AddListener((accelerometer, gyro, time) =>
                {
                    //factors.Add(addImuBiasFactor(indexCalibration.Ref, indexMotion.Frame, accelerometer, gyro));

                    if(factors.Count > 150)
                    {
                        var f = factors[0];
                        factors.RemoveAt(0);
                        removeImuBiasFactor(f);
                    }
                    
                    stream.Push(accelerometer, gyro, Index.Position, indexMotion.Frame.acceleration, indexMotion.Frame.angularVelocity.eulerAngles, indexMotion.rotationProvider.rotation.eulerAngles, new Vector3(time,0,0));
                });
            }
        }

        // Update is called once per frame
        void FixedUpdate()
        {
//            var bias = getImuBiasParameters(indexCalibration.Ref);

        }
    }
}