#pragma once

#include <Eigen/Dense>
#include "Pose.h"

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

        static Pose ToPose(hs::Pose3d* pose)
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
}