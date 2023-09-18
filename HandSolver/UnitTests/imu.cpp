#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <Imu.h>

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace hs;
using namespace Eigen;

#define TOL 1e-4
#define EXPECT_VECTOR(v1,v2) \
	EXPECT_NEAR(v1.x(),v2.x(),TOL);\
	EXPECT_NEAR(v1.y(),v2.y(),TOL);\
	EXPECT_NEAR(v1.z(),v2.z(),TOL);

void CheckRotation(hs::Rodriguesd a, Eigen::AngleAxisd b);

TEST(IMU, PreIntegration1) {

	// Tests the preintegration factor with a simple acceleration orthogonal
	// to gravity. The IMU sample is integrated over one second. The 
	// implementation integrates the velocity before position, so the final 
	// position is the same as the acceleration. All other transforms are 
	// identity.

	Pose3d origin;
	Pose3d end;
		
	auto preIntegrationFactor = new hs::PreIntegrationFactor(&origin, &end);

	// Create some simulated measurements. These are in the world frame.

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d acceleration = gravity + motion;

	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), 1.0);

	Problem problem;

	preIntegrationFactor->addToProblem(problem);

	// Prevent the solver moving the origin.

	problem.SetParameterBlockConstant(origin.parameterBlock());

	// For this test, the calibration is set as Identity

	problem.SetParameterBlockConstant(preIntegrationFactor->localPositionParameter());
	problem.SetParameterBlockConstant(preIntegrationFactor->localRotationParameter());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->bodyVelocity->data());

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), motion);
}


TEST(IMU, PreIntegration2)
{
	// Tests the preintegration factor with an acceleration orthogonal to
	// gravity, and a rotation around the gravitational axis.
	// The inertial frame remains the same as the world frame. The end
	// pose should be the same as the acceleration and rotation (as they
	// are integrated over one second).

	Pose3d origin;
	Pose3d end;

	auto preIntegrationFactor = new hs::PreIntegrationFactor(&origin, &end);

	// Create some simulated measurements

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d acceleration = gravity + motion;
	Vector3d rotation(0, 1.5708, 0);

	preIntegrationFactor->addSample(acceleration, rotation, 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(origin.parameterBlock());

	// For this test, the calibration is set as Identity

	problem.SetParameterBlockConstant(preIntegrationFactor->localPositionParameter());
	problem.SetParameterBlockConstant(preIntegrationFactor->localRotationParameter());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->bodyVelocity->data());

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), motion);
	CheckRotation(end.Rotation(), AngleAxisd(rotation.y(), Vector3d::UnitY()));
}


TEST(IMU, PreIntegration3)
{
	// Tests the preintegration factor with a simple acceleration but where the
	// inertial frame is rotated with respect to the world frame.
	// The end pose should be the acceleration integrated over one second, with
	// no world rotation change.

	Pose3d origin;
	Pose3d end;

	auto preIntegrationFactor = new hs::PreIntegrationFactor(&origin, &end);

	preIntegrationFactor->localRotation = AngleAxisd(1.5708, Vector3d::UnitX());

	// Create some simulated measurements, given in the world frame

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);

	Vector3d acceleration = (preIntegrationFactor->localRotation * gravity) + (preIntegrationFactor->localRotation * motion);

	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(origin.parameterBlock());

	// For this test, the calibration is fixed

	problem.SetParameterBlockConstant(preIntegrationFactor->localPositionParameter());
	problem.SetParameterBlockConstant(preIntegrationFactor->localRotationParameter());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->bodyVelocity->data());

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), motion);
	CheckRotation(end.Rotation(), AngleAxisd(0, Vector3d::UnitY()));
}

TEST(IMU, PreIntegration4)
{
	// Tests the preintegration factor with a change in inertial position and
	// acceleration, with a non-zero inertial frame rotation.
	// The end pose should be the integration of the acceleration and rotations
	// given in the world frame.

	Pose3d origin;
	Pose3d end;

	auto preIntegrationFactor = new hs::PreIntegrationFactor(&origin, &end);

	// Define the local inertial frame

	preIntegrationFactor->localRotation = AngleAxisd(1.5708, Vector3d::UnitX());

	// The simulated inertial measurements, given in the world frame

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d rotation(0, 1.5780, 0);

	Vector3d acceleration = (preIntegrationFactor->localRotation * gravity) + (preIntegrationFactor->localRotation * motion);

	preIntegrationFactor->addSample(acceleration, (preIntegrationFactor->localRotation * rotation), 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(origin.parameterBlock());

	// For this test, the calibration is fixed

	problem.SetParameterBlockConstant(preIntegrationFactor->localPositionParameter());
	problem.SetParameterBlockConstant(preIntegrationFactor->localRotationParameter());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->bodyVelocity->data());

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = false;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), motion);
	CheckRotation(end.Rotation(), AngleAxisd(rotation.y(), Vector3d::UnitY()));
}
