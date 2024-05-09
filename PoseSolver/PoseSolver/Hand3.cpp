#include "pch.h"
#include "Hand3.h"
#include "Export.h"
#include "Unity.h"
#include "PointMeasurement.h"
#include "Transforms.h"
#include "Joints.h"

using namespace hand3;
using namespace transforms;

class Hand3Scene
{
public:
    ceres::Problem problem;
};

Hand3Scene* scene;

EXPORT void hand3_initialise()
{
    scene = new Hand3Scene();
}

EXPORT Transformd* hand3_addTransform(unity::Pose intialPose)
{
    auto p = new transforms::Transformd(intialPose.Position(), intialPose.Rotation());
    scene->problem.AddParameterBlock(p->parameterBlock(), transforms::Transformd::Dimension);
    scene->problem.SetParameterBlockVariable(p->parameterBlock());
    return p;
}

EXPORT void hand3_addSocket1D(Transformd* from, Transformd* to, unity::Vector2 range)
{
    auto j = new joints::Axis1D(from, to, range);
    scene->problem.AddResidualBlock(
        j->costFunction(),
        nullptr,
        j->parameterBlocks()
    );
}

EXPORT void hand3_addSocket2D(Transformd* from, Transformd* to, unity::Vector2 range_x, unity::Vector2 range_y)
{
    auto j = new joints::Axis2D(from, to, range_x, range_y);
    scene->problem.AddResidualBlock(
        j->costFunction(),
        nullptr,
        j->parameterBlocks()
    );
}

EXPORT void hand3_addRigid(Transformd* from, Transformd* to)
{
    auto j = new joints::Rigid(from, to);
    scene->problem.AddResidualBlock(
        j->costFunction(),
        nullptr,
        j->parameterBlocks()
    );
}

EXPORT observations::PointMeasurement* hand3_addPointMeasurement(Transformd* pose, float dx, float dy, float dz, float wx, float wy, float wz)
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

EXPORT void hand3_updatePointMeasurement(observations::PointMeasurement* measurement, float wx, float wy, float wz) 
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

EXPORT void hand3_disablePointMeasurement(observations::PointMeasurement* measurement)
{
    if (measurement->residualBlockId != nullptr) {
        scene->problem.RemoveResidualBlock(measurement->residualBlockId);
        measurement->residualBlockId = nullptr;
    }
}

EXPORT void hand3_solve()
{
    using namespace ceres;

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    Solver::Summary summary;

    ceres::Solve(options, &scene->problem, &summary);
}

EXPORT unity::Pose hand3_getUnityTransform(transforms::Transformd* p)
{
    return unity::Pose::ToUnityPose(p);
}