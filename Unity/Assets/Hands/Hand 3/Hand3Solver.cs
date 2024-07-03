using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

public class Hand3Solver : MonoBehaviour
{
    private Dictionary<Transform, IntPtr> transformNodes;
    private Dictionary<Fingers, Transform> fingertips;
    private System.IntPtr wristPose;

    public class PointMeasurement
    {
        public IntPtr measurement;

        public void Update(Vector3 p)
        {
            hand3_updatePointMeasurement(measurement, p.x, p.y, p.z);
        }

        public void Remove()
        {
            hand3_disablePointMeasurement(measurement);
        }
    }

    public class OrientationMeasurement
    {
        public IntPtr measurement;

        public void Update(Quaternion q)
        {
            hand3_updateOrientationMeasurement(measurement, q.x, q.y, q.z, q.w);
        }

        public void Remove()
        {
            hand3_disableOrientationMeasurement(measurement);
        }
    }

    void Start()
    {
        Initialise();
    }

    void Update()
    {
        PerformanceProfiler.StartFrame();

        hand3_solve();

        PerformanceProfiler.EndFrame();

        foreach (var item in transformNodes)
        {
            var t = hand3_getUnityTransform(item.Value);
            item.Key.position = t.Position;
            item.Key.rotation = t.Rotation;
        }
    }

    private void Initialise()
    {
        // Add the wrist pose

        hand3_initialise();
        wristPose = hand3_addTransform(transform);

        // For each finger chain, add the transforms and the constraints

        fingertips = new Dictionary<Fingers, Transform>();
        transformNodes = new Dictionary<Transform, IntPtr>();
        transformNodes[transform] = wristPose;

        for (int i = 0; i < transform.childCount; i++)
        {
            AddNode(transform.GetChild(i));
        }
    }

    private void AddNode(Transform node)
    {
        var parentTransfom = transformNodes[node.parent];
        var transform = hand3_addTransform(node);

        transformNodes[node] = transform;

        var socket1d = node.GetComponent<Socket1D>();
        var socket2d = node.GetComponent<Socket2D>();

        if (socket1d && socket1d.range.magnitude > 0)
        {
            hand3_addSocket1D(parentTransfom, transform, socket1d.range * Mathf.Deg2Rad);
        }
        else if(socket2d && socket2d)
        {
            hand3_addSocket2D(parentTransfom, transform, socket2d.range_x * Mathf.Deg2Rad, socket2d.range_y * Mathf.Deg2Rad);
        }
        else
        {
            // If there are no constraints, then assume that the linkage is rigid
            // (this is for development, when the model is complete it should rarely
            // if ever occur, as if two transforms are rigid they can be simplified
            // into one).

            hand3_addRigid(parentTransfom, transform);
        }

        if(node.childCount > 0)
        {
            AddNode(node.GetChild(0));
        }
        else
        {
            fingertips[FingersExtension.GetFinger(node.name)] = node;
        }
    }

    public PointMeasurement AddPointConstraint(Fingers finger, Vector3 point)
    {
        var m = new PointMeasurement();
        var transform = fingertips[finger];
        var offset = transform.InverseTransformPoint(point);
        var ptr = transformNodes[transform];
        m.measurement = hand3_addPointMeasurement(ptr, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    public PointMeasurement AddPointConstraint(Vector3 point)
    {
        var m = new PointMeasurement();
        var offset = transform.InverseTransformPoint(point);
        m.measurement = hand3_addPointMeasurement(wristPose, offset.x, offset.y, offset.z, point.x, point.y, point.z);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint(Fingers finger)
    {
        var m = new OrientationMeasurement();
        var transform = fingertips[finger];
        var ptr = transformNodes[transform];
        m.measurement = hand3_addOrientationMeasurement(ptr, 0, 0, 0, 1);
        return m;
    }

    public OrientationMeasurement AddOrientationConstraint()
    {
        var m = new OrientationMeasurement();
        m.measurement = hand3_addOrientationMeasurement(wristPose, 0, 0, 0, 1);
        return m;
    }

    public Transform GetTransform(Fingers finger)
    {
        return fingertips[finger];
    }

    public Transform GetTransform()
    {
        return transform;
    }

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_initialise();

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addTransform(Pose positionrotation);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addSocket1D(IntPtr from, IntPtr to, Vector2 range);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addSocket2D(IntPtr from, IntPtr to, Vector2 range_x, Vector2 range_y);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addRigid(IntPtr from, IntPtr to);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addPointMeasurement(IntPtr pose, float dx, float dy, float dz, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_updatePointMeasurement(IntPtr measurement, float wx, float wy, float wz);

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_disablePointMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern IntPtr hand3_addOrientationMeasurement(IntPtr pose, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_updateOrientationMeasurement(IntPtr measurement, float x, float y, float z, float w);

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_disableOrientationMeasurement(IntPtr measurement);

    [DllImport("PoseSolver.dll")]
    public static extern void hand3_solve();

    [DllImport("PoseSolver.dll")]
    public static extern Pose hand3_getUnityTransform(IntPtr p);
}
