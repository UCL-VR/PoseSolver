#include "pch.h"
#include "Hand2.h"
#include "Export.h"
#include "Unity.h"
#include "PointMeasurement.h"
#include "Transforms.h"

using namespace hand2;
using namespace transforms;

class Hand2Scene
{
public:
    ceres::Problem problem;
};

Hand2Scene* scene;

EXPORT void hand2_initialise()
{
    scene = new Hand2Scene();
}

EXPORT Hand* hand2_addHand(Hand::HandParams params, transforms::Transformd* pose)
{
    // Break the packed parameters out into a sequence of chains

    auto hand = new hand2::Hand(params, pose);

    scene->problem.AddResidualBlock(
        hand->costFunction(),
        nullptr,
        hand->parameterBlocks()
    );

    return hand;
}

EXPORT Transformd* hand2_addPose(bool setParameterBlockConstant)
{
    auto p = new transforms::Transformd();
    scene->problem.AddParameterBlock(p->parameterBlock(), transforms::Transformd::Dimension);
    if (setParameterBlockConstant)
    {
        scene->problem.SetParameterBlockConstant(p->parameterBlock());
    }
    return p;
}

EXPORT Transformd* hand2_getHandEndPose(Hand* hand, int finger)
{
    return hand->getEndPose((Hand::Finger)finger);
}

EXPORT void hand2_getHandPose(Hand* hand, double* angles)
{
    memcpy(angles, hand->theta, sizeof(double) * 30);
}

EXPORT void hand2_solve()
{
    using namespace ceres;

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    Solver::Summary summary;

    ceres::Solve(options, &scene->problem, &summary);
}

EXPORT observations::PointMeasurement* hand2_addPointMeasurement(transforms::Transformd* pose, float dx, float dy, float dz, float wx, float wy, float wz)
{
    auto m = new observations::PointMeasurement();
    m->pose = pose;
    m->offset = Eigen::Vector3d(dx, dy, dz);
    m->point = Eigen::Vector3d(wx, wy, wz);
    scene->problem.AddResidualBlock(
        m->costFunction(),
        nullptr,
        m->parameterBlocks()
    );
    scene->problem.SetParameterBlockConstant(m->offsetParameterBlock());
    scene->problem.SetParameterBlockConstant(m->pointParameterBlock());
    return m;
}

EXPORT void hand2_updatePointMeasurement(observations::PointMeasurement* m, float wx, float wy, float wz)
{
    m->point = Eigen::Vector3d(wx, wy, wz);
}

EXPORT unity::Pose hand2_getUnityPose(transforms::Transformd* p)
{
    return unity::Pose::ToUnityPose(p);
}