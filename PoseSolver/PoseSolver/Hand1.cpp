#include "pch.h"
#include "Hand1.h"
#include "Export.h"
#include "Unity.h"
#include "PointMeasurement.h"
#include "Transforms.h"

class Hand1Scene
{
public:
	ceres::Problem problem;
};

Hand1Scene* scene;

EXPORT void hand1_initialise()
{
	scene = new Hand1Scene();
}

/// <summary>
/// Adds a six-dof Transform parameter block to represent an absolute
/// joint configuration.
/// </summary>
EXPORT transforms::Transformd* hand1_addTransform(bool setParameterBlockConstant)
{
	auto p = new transforms::Transformd();
	scene->problem.AddParameterBlock(p->parameterBlock(), transforms::Transformd::Dimension);
	if (setParameterBlockConstant)
	{
		scene->problem.SetParameterBlockConstant(p->parameterBlock());
	}
	return p;
}

/// <summary>
/// Adds a Denavit-Hartenberg Joint-Link pair to the kinematic chain of the
/// solver scene.
/// A valid transforms (e.g. created with addPose()) must be provided as the starting
/// frame. The end transforms will be created automatically.
/// 
/// While many constraints for different times will be added, this method need
/// only be called once to define the kinematics of the problem. Instances of
/// the Factor will be created as needed as new measurements are added.
/// 
/// The one constraint here is that joints must be provided in order.
/// </summary>
EXPORT dh::Joint* hand1_addJoint(transforms::Transformd* referenceFrame, float d, float th, float r, float a)
{
    auto joint = new dh::Joint();

    joint->a = a;
    joint->theta = th;
    joint->r = r;
    joint->d = d;

    // Let the joint store its own end reference frame for now

    joint->start = referenceFrame;
    joint->end = new transforms::Transformd();

    scene->problem.AddResidualBlock(
        joint->costFunction(),
        nullptr,
        joint->parameterBlocks()
    );

    return joint;
}

/// <summary>
/// Adds a range to the Denavit-Hartenberg Joint variable theta. If a JointLimit
/// is not applied, the joint may take on any rotation. Limits are enforced via
/// a Cost Function. Once created, the limit cannot be altered.
/// </summary>
EXPORT void hand1_setJointLimit(dh::Joint* joint, float min, float max)
{
    //https://github.com/ceres-solver/ceres-solver/issues/187
    // Scale of 5 for radians

    auto limit = new dh::JointLimit(joint, min, max, 10.0);
    scene->problem.AddResidualBlock(
        limit->costFunction(),
        nullptr,
        limit->parameterBlocks()
    );
}

/// <summary>
/// Sets whether or not the DH Joint Angle is a constant. DH joints have only
/// one optimisable parameter, so setting this to true effectively turns the
/// joint rigid.
/// </summary>
EXPORT void hand1_setJointParameterConstant(dh::Joint* joint, bool isConstant)
{
    if (isConstant) {
        scene->problem.SetParameterBlockConstant(joint->parameterBlock());
    }
    else {
        scene->problem.SetParameterBlockVariable(joint->parameterBlock());
    }
}

EXPORT transforms::Transformd* hand1_getJointStartTransform(dh::Joint* joint)
{
    return joint->start;
}

EXPORT transforms::Transformd* hand1_getJointEndTransform(dh::Joint* joint)
{
    return joint->end;
}

EXPORT float hand1_getJointAngle(dh::Joint* joint)
{
    return (float)joint->theta;
}

EXPORT unity::Pose hand1_getUnityTransform(transforms::Transformd* p)
{
    return unity::Pose::ToUnityPose(p);
}

EXPORT observations::PointMeasurement* hand1_addPointMeasurement(transforms::Transformd* pose, float dx, float dy, float dz, float wx, float wy, float wz)
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

EXPORT void hand1_updatePointMeasurement(observations::PointMeasurement* m, float wx, float wy, float wz)
{
    m->point = Eigen::Vector3d(wx, wy, wz);
}

EXPORT void hand1_solve()
{
    using namespace ceres;

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;
    Solver::Summary summary;

    ceres::Solve(options, &scene->problem, &summary);
}