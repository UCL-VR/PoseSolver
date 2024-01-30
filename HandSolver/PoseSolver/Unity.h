#pragma once

#include <Eigen/Dense>
#include "Pose.h"
#include "Imu.h"

#pragma pack(push,1) // This whole file is defining structs for interop

namespace Unity 
{
    struct Pose
    {
        float x;
        float y;
        float z;
        float qx;
        float qy;
        float qz;
        float qw;

        // We cannot have constructors and be compatible with the C ABI

        static Pose ToUnityPose(hs::Pose3d* pose)
        {
            Pose p;
            p.x = pose->Position().x();
            p.y = pose->Position().y();
            p.z = pose->Position().z();
            p.qx = pose->Rotation().toQuaternion().x();
            p.qy = pose->Rotation().toQuaternion().y();
            p.qz = pose->Rotation().toQuaternion().z();
            p.qw = pose->Rotation().toQuaternion().w();
            return p;
        }
    };

    struct Vector3
    {
        float x;
        float y;
        float z;

        operator Eigen::Vector3d() const
        {
            return Eigen::Vector3d(x, y, z);
        }

        Vector3() 
        {
            x = 0;
            y = 0;
            z = 0;
        }

        Vector3(const Eigen::Vector3d v)
        {
            x = v.x();
            y = v.y();
            z = v.z();
        }
    };

    struct Quaternion
    {
        float x;
        float y;
        float z;
        float w;

        operator Eigen::Quaterniond() const
        {
            return Eigen::Quaterniond(w, x, y, z);
        }
    };

    struct MotionFrame
    {
        Vector3 position; // The instantaneous world space position
        Quaternion rotation; // The instantaneous world space rotation
        Vector3 acceleration; // The change in velocity between the last timestep and this one, as a Vector
        Quaternion angularVelocity; // The change in rotation between the last timestep and this one, as a Quaternion

        operator hs::MotionFrame() const 
        {
            hs::MotionFrame f;
            f.pose = hs::Pose3d(position, rotation);
            f.acceleration = acceleration;
            f.angularVelocity = angularVelocity;
            return f;
        }
    };

    struct ImuBiasParameters
    {
        Vector3 accelerometerBias;
        Vector3 gyroscopeBias;

        ImuBiasParameters() { }

        ImuBiasParameters(const hs::ImuBiasParameters f) 
        {
            accelerometerBias = f.accelerometerBias;
            gyroscopeBias = f.gyroscopeBias;
        }
    };
}

#pragma pack(pop)