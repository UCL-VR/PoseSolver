#include "pch.h"
#include "Hand5.h"
#include "Export.h"
#include "Unity.h"
#include "PointMeasurement.h"
#include "Transforms.h"

using namespace hand5;
using namespace transforms;

class Hand5Scene
{
public:
    ceres::Problem problem;
    std::vector<observations::PointMeasurementBase*> measurements;
};

#pragma optimize("",off)

static Hand5Scene* scene;

EXPORT void hand5_initialise()
{
    scene = new Hand5Scene();
}

// For Hand5, this is only used to add the wrist transform

EXPORT Transformd* hand5_addPose(bool setParameterBlockConstant)
{
    auto p = new transforms::Transformd();
    scene->problem.AddParameterBlock(p->parameterBlock(), transforms::Transformd::Dimension);
    if (setParameterBlockConstant)
    {
        scene->problem.SetParameterBlockConstant(p->parameterBlock());
    }
    return p;
}

EXPORT Hand* hand5_addHand(Hand::HandParams params, transforms::Transformd* pose)
{
    // Break the packed parameters out into a sequence of chains

    auto hand = new Hand(params, pose);

    // Joint limits are the only constraints the hand has inherently.
    // Point and Orientation observations are added per finger.
    Hand::JointLimits limits(hand);

    scene->problem.AddResidualBlock(
        limits.costFunction(),
        nullptr,
        limits.parameterBlocks()
    );

    return hand;
}

EXPORT void hand5_getHandPose(Hand* hand, double* angles)
{
    memcpy(angles, hand->theta, sizeof(double) * 30);
}

EXPORT void hand5_solve()
{
    using namespace ceres;

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    Solver::Summary summary;

    ceres::Solve(options, &scene->problem, &summary);
}

EXPORT observations::PointMeasurement* hand5_addTransformPointMeasurement(transforms::Transformd* pose, float dx, float dy, float dz, float wx, float wy, float wz)
{
    auto m = new observations::PointMeasurement();
    m->pose = pose;
    m->offset = Eigen::Vector3d(dx, dy, dz);
    m->point = Eigen::Vector3d(wx, wy, wz);
    m->residualBlockId = scene->problem.AddResidualBlock(
        m->costFunction(),
        nullptr,
        m->parameterBlocks()
    );
    scene->problem.SetParameterBlockConstant(m->offsetParameterBlock());
    scene->problem.SetParameterBlockConstant(m->pointParameterBlock());
    return m;
}

EXPORT hand5::Hand::PointMeasurement* hand5_addFingerPointMeasurement(Hand* hand, int finger, float dx, float dy, float dz, float wx, float wy, float wz)
{
    auto m = new hand5::Hand::PointMeasurement(hand, (Hand::Finger)finger);
    m->offset = Eigen::Vector3d(dx, dy, dz);
    m->point = Eigen::Vector3d(wx, wy, wz);
    m->residualBlockId = scene->problem.AddResidualBlock(
        m->costFunction(),
        nullptr,
        m->parameterBlocks()
    );
    scene->problem.SetParameterBlockConstant(m->offsetParameterBlock());
    scene->problem.SetParameterBlockConstant(m->pointParameterBlock());
    return m;
}

EXPORT void hand5_updatePointMeasurement(observations::PointMeasurementBase* measurement, float wx, float wy, float wz)
{
    if (measurement->residualBlockId == nullptr) {
        measurement->residualBlockId = scene->problem.AddResidualBlock(
            measurement->costFunction(),
            nullptr,
            measurement->parameterBlocks()
        );
    }
    measurement->point = Eigen::Vector3d(wx, wy, wz);
}

EXPORT void hand5_disablePointMeasurement(observations::PointMeasurementBase* measurement)
{
    if (measurement->residualBlockId != nullptr) {
        scene->problem.RemoveResidualBlock(measurement->residualBlockId);
        measurement->residualBlockId = nullptr;
    }
}

EXPORT observations::OrientationMeasurement* hand5_addOrientationMeasurement(hand5::Hand::Finger finger, float qx, float qy, float qz, float qw)
{
    auto m = new observations::OrientationMeasurement();
    /*
    m->pose = pose;
    m->orientation = pose->Rotation();
    m->residualBlockId = scene->problem.AddResidualBlock(
        m->costFunction(),
        nullptr,
        m->parameterBlocks()
    );
    scene->problem.SetParameterBlockConstant(m->orientationParameterBlock());
    */
    return m;
}

EXPORT void hand5_updateOrientationMeasurement(observations::OrientationMeasurement* measurement, float qx, float qy, float qz, float qw)
{
    /*
    if (measurement->residualBlockId == nullptr) {
        measurement->residualBlockId = scene->problem.AddResidualBlock(
            measurement->costFunction(),
            nullptr,
            measurement->parameterBlocks()
        );
    }
    measurement->orientation = Eigen::Quaterniond(qw, qx, qy, qz);
    */
}

EXPORT void hand5_disableOrientationMeasurement(observations::OrientationMeasurement* measurement)
{
    /*
    if (measurement->residualBlockId != nullptr) {
        scene->problem.RemoveResidualBlock(measurement->residualBlockId);
        measurement->residualBlockId = nullptr;
    }
    */
}

EXPORT unity::Pose hand5_getUnityPose(Transformd* p)
{
    return unity::Pose::ToUnityPose(p);
}

EXPORT unity::Pose hand5_getUnityFingerPose(hand5::Hand* hand, hand5::Hand::Finger finger)
{
    auto transform = hand->getEndPose(finger);
    return unity::Pose::ToUnityPose(&transform);
}