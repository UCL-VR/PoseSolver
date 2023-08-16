using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UCL.CASMS.DH;
using UnityEngine;

/// <summary>
/// Estimes the pose of the Hand1 Kinematic Hand Model built into the Solver.
/// </summary>
public class Hand1Solver : PoseSolver
{
    // The Hand1 parameterisation is based on DH chains,
    // one per finger.

    public enum Fingers : int 
    {
        Thumb = 0,
        Index = 1,
        Middle = 2,
        Ring = 3,
        Little = 4
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct JointParams
    {
        public double d;
        public double theta;
        public double r;
        public double a;

        public static implicit operator JointParams(UCL.CASMS.DH.Joint j)
        {
            return new JointParams()
            {
                a = j.a * Mathf.Deg2Rad,
                d = j.d,
                r = j.r,
                theta = j.th * Mathf.Deg2Rad
            };
        }
    }

    // The following types are effectively manually unrolled versions of
    // the native array based ones.

    [StructLayout(LayoutKind.Sequential)]
    public struct ChainParams
    {
        public JointParams joint1;
        public JointParams joint2;
        public JointParams joint3;
        public JointParams joint4;
        public JointParams joint5;
        public JointParams joint6;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct HandParams
    {
        public ChainParams thumb;
        public ChainParams index;
        public ChainParams middle;
        public ChainParams ring;
        public ChainParams little;
    }

    private HandParams hand;

    private ChainParams GetChainParams(Transform root)
    {
        var nodes = root.GetComponentsInChildren<DHJointLink>();
        ChainParams p;
        p.joint1 = nodes[0].joint;
        p.joint2 = nodes[1].joint;
        p.joint3 = nodes[2].joint;
        p.joint4 = nodes[3].joint;
        p.joint5 = nodes[4].joint;
        p.joint6 = nodes[5].joint;
        return p;
    }

    private DHJointLink[] Thumb;
    private DHJointLink[] Index;
    private DHJointLink[] Middle;
    private DHJointLink[] Ring;
    private DHJointLink[] Little;

    private List<DHJointLink[]> FingerChains;

    private double[] angles;

    private DHJointLink[] GetChainNodes(Transform root)
    {
        return root.GetComponentsInChildren<DHJointLink>();
    }

    private IntPtr startPose;
    private IntPtr hand1;

    public Hand1Measurements Measurements;

    public void Initialise()
    {
        // Get the joints (per-chain) in a representation amenable for loading
        // into the solver.

        // This implementation assumes that each finger has a separate GameObject,
        // directly below the Hand to which the Solver is added.

        hand.thumb = GetChainParams(transform.Find("Thumb"));
        hand.index = GetChainParams(transform.Find("Index"));
        hand.middle = GetChainParams(transform.Find("Middle"));
        hand.ring = GetChainParams(transform.Find("Ring"));
        hand.little = GetChainParams(transform.Find("Little"));

        FingerChains = new List<DHJointLink[]>();

        FingerChains.Add(GetChainNodes(transform.Find("Thumb")));
        FingerChains.Add(GetChainNodes(transform.Find("Index")));
        FingerChains.Add(GetChainNodes(transform.Find("Middle")));
        FingerChains.Add(GetChainNodes(transform.Find("Ring")));
        FingerChains.Add(GetChainNodes(transform.Find("Little")));

        angles = new double[30];

        // Create the entries in the problem

        initialise();

        startPose = addPose(true);   // The pose parameter for the root of the hand/wrist
        hand1 = addHand1(hand, startPose);

        if (Measurements)
        {
            Measurements.OnInitialise();
        }
    }

    /*
    public TransformPointMeasurement AddMeasurement(Fingers finger)
    {
        addPointMeasurement(getHand1EndPose(hand1, finger));
    }
    */


    private void Start()
    {
        Initialise();
    }

    public void Update()
    {
        solve();

        // Project onto hand

        getHand1Pose(hand1, angles);

        for (int f = 0; f < 5; f++)
        {
            var offset = f * 6;
            var joints = FingerChains[f];
            for (int i = 0; i < 6; i++)
            {
                joints[i].joint.th = (float)angles[offset + i] * Mathf.Rad2Deg;
            }
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;

        if (hand1 != IntPtr.Zero)
        {
            for (int i = 0; i < 5; i++)
            {
                var p = getPose(getHand1EndPose(hand1, (Fingers)i));
                Gizmos.DrawWireSphere(p.Position, 0.005f);



            }
        }
    }

    private void OnDrawChainGizmos(ChainParams p)
    {
        var start = Vector3.zero;



    }
}
