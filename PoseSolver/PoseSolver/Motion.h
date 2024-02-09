#pragma once

#include <Eigen/Dense>
#include "Transforms.h"

namespace motion {

    /// <summary>
    /// A convenience type usually utilised by Imu-based factors, that describes
    /// the immediate Pose of a body, but also its acceleration and rotational
    /// velocity.
    /// </summary>
    struct MotionFrame
    {
        /// <summary>
        /// Pose in world space at this time
        /// </summary>
        transforms::Transformd pose;

        /// <summary>
        /// The acceleration in world space of the frame at this time
        /// </summary>
        Eigen::Vector3d acceleration;

        /// <summary>
        /// The angular velocity (change in rotation) in world space of the frame at this time
        /// </summary>
        transforms::Rodriguesd angularVelocity;
    };
}