// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"

#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#define GLOG_NO_ABBREVIATED_SEVERITIES

#define EXPORT extern "C" __declspec(dllexport) 

#include <ceres/ceres.h>
#include <ceres/problem.h>

#include "Unity.h"
#include "DenavitHartenberg.h"
#include "Pose.h"
#include "Hand1.h"

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
        break;
    }
    return TRUE;
}


// The DH solver scene

class Scene
{
public:
    ceres::Problem problem;
    std::vector<hs::Pose3d*> poses;
    std::vector<dh::Joint*> dhJointTable;
    std::vector<hs::PointMeasurement*> pointMeasurements;
};

Scene* scene;

EXPORT void initialise()
{
    scene = new Scene();
}

/// <summary>
/// Adds a Denavit-Hartenberg Joint-Link pair to the kinematic chain of the
/// solver scene.
/// A valid pose (e.g. created with addPose()) must be provided as the starting
/// frame. The end pose will be created automatically.
/// 
/// While many constraints for different times will be added, this method need
/// only be called once to define the kinematics of the problem. Instances of
/// the Factor will be created as needed as new measurements are added.
/// 
/// The one constraint here is that joints must be provided in order.
/// </summary>
EXPORT dh::Joint* addDHJoint(hs::Pose3d* referenceFrame, float d, float th, float r, float a)
{
    auto joint = new dh::Joint();

    joint->a = a;
    joint->theta = th;
    joint->r = r;
    joint->d = d;

    // Let the joint store its own end reference frame for now

    joint->start = referenceFrame;
    joint->end = new hs::Pose3d();

    scene->poses.push_back(joint->end);
    scene->dhJointTable.push_back(joint);

    scene->problem.AddResidualBlock(
        joint->costFunction(),
        nullptr,
        joint->parameterBlocks()
    );

    return joint;
}

/// <summary>
/// Sets whether or not the DH Joint Angle is a constant. DH joints have only
/// one optimisable parameter, so setting this to true effectively turns the
/// joint rigid.
/// </summary>
EXPORT void setDhJointParameterConstant(dh::Joint* joint, bool isConstant)
{
    if (isConstant) {
        scene->problem.SetParameterBlockConstant(joint->parameterBlock());
    }
    else {
        scene->problem.SetParameterBlockVariable(joint->parameterBlock());
    }
}

// Adds a six-dof pose parameter block
EXPORT hs::Pose3d* addPose(bool setParameterBlockConstant)
{
    auto p = new hs::Pose3d();
    scene->poses.push_back(p);
    if (setParameterBlockConstant)
    {
        scene->problem.AddParameterBlock(p->parameterBlock(), hs::Pose3d::Dimension);
        scene->problem.SetParameterBlockConstant(p->parameterBlock());
    }
    return p;
}

// Gets the value of the pose with index p
EXPORT Unity::Pose getPose(hs::Pose3d* p)
{
    return Unity::Pose::ToPose(p);
}

EXPORT float getJointAngle(dh::Joint* joint)
{
    return joint->theta;
}

EXPORT hs::Pose3d* getJointEndPose(dh::Joint* joint)
{
    return joint->end;
}

EXPORT hs::Pose3d* getJointStartPose(dh::Joint* joint)
{
    return joint->start;
}

EXPORT hs::PointMeasurement* addPointMeasurement(hs::Pose3d* pose, float dx, float dy, float dz, float wx, float wy, float wz)
{
    auto m = new hs::PointMeasurement();
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

    scene->pointMeasurements.push_back(m);
    return m;
}

// 

/// <summary>
/// Updates the Point parameter block of the PointMeasurement m (the observed
/// point). Both the offset and the point parameter blocks remain constant.
/// </summary>
EXPORT void updatePointMeasurement(hs::PointMeasurement* m, float wx, float wy, float wz)
{
    m->point = Eigen::Vector3d(wx, wy, wz);
}

EXPORT hs::Hand1* addHand1(hs::Hand1::HandParams params, hs::Pose3d* pose)
{
    // Break the packed parameters out into a sequence of chains

    auto hand = new hs::Hand1(params, pose);

    scene->problem.AddResidualBlock(
        hand->costFunction(),
        nullptr,
        hand->parameterBlocks()
    );

    return hand;
}

EXPORT hs::Pose3d* getHand1EndPose(hs::Hand1* hand, int finger)
{
    return hand->getEndPose((hs::Hand1::Finger)finger);
}

EXPORT void getHand1Pose(hs::Hand1* hand, double* angles)
{
    memcpy(angles, hand->theta, sizeof(double) * 30);
}

EXPORT void solve()
{
    using namespace ceres;

    Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;

    ceres::Solve(options, &scene->problem, &summary);
}

EXPORT float getVersion()
{
    return 0.01f;
}

EXPORT void HelloWorld()
{
}