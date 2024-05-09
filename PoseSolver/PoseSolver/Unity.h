#pragma once

#include <Eigen/Dense>
#include "Transforms.h"
#include "Imu.h"

#pragma pack(push,1) // This whole file is defining structs for interop

namespace unity 
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

        static Pose ToUnityPose(transforms::Transformd* pose)
        {
            Pose p;
            p.x = (float)pose->Position().x();
            p.y = (float)pose->Position().y();
            p.z = (float)pose->Position().z();
            p.qx = (float)pose->Rotation().toQuaternion().x();
            p.qy = (float)pose->Rotation().toQuaternion().y();
            p.qz = (float)pose->Rotation().toQuaternion().z();
            p.qw = (float)pose->Rotation().toQuaternion().w();
            return p;
        }

        Eigen::Vector3d Position()
        {
            return Eigen::Vector3d(x, y, z);
        }

        Eigen::Quaterniond Rotation()
        {
            return Eigen::Quaterniond(qw, qx, qy, qz);
        }
    };

    struct Vector2
    {
        float x;
        float y;

        operator Eigen::Vector2d() const
        {
            return Eigen::Vector2d(x, y);
        }

        Vector2()
        {
            x = 0;
            y = 0;
        }

        Vector2(const Eigen::Vector2d v)
        {
            x = (float)v.x();
            y = (float)v.y();
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
            x = (float)v.x();
            y = (float)v.y();
            z = (float)v.z();
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

        operator motion::MotionFrame() const
        {
            motion::MotionFrame f;
            f.pose = transforms::Transformd(position, rotation);
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

        ImuBiasParameters(const imu::ImuBiasParameters f) 
        {
            accelerometerBias = f.accelerometerBias;
            gyroscopeBias = f.gyroscopeBias;
        }
    };
}

#pragma pack(pop)