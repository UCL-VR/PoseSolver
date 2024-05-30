#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <Imu.h>
#include <Motion.h>

#include "MockImu.h"

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using namespace Eigen;

#define TOL 1e-4
#define EXPECT_VECTOR(expected,actual) \
	EXPECT_NEAR(expected.x(),actual.x(),TOL);\
	EXPECT_NEAR(expected.y(),actual.y(),TOL);\
	EXPECT_NEAR(expected.z(),actual.z(),TOL);

void CheckRotation(transforms::Rodriguesd a, Eigen::AngleAxisd b);
void CheckRotation(transforms::Rodriguesd a, Eigen::Quaterniond b);
void CheckRotation(transforms::Rodriguesd a, transforms::Rodriguesd b);

void CheckPose(transforms::Transformd& p, Eigen::Vector3d position, Eigen::AngleAxisd rotation)
{
	EXPECT_VECTOR(p.Position(), position);
	CheckRotation(p.Rotation(), rotation);
}

void CheckVector(Vector3d expected, Vector3d actual, double tol)
{
	EXPECT_NEAR(expected.x(), actual.x(), tol); 
	EXPECT_NEAR(expected.y(), actual.y(), tol);
	EXPECT_NEAR(expected.z(), actual.z(), tol);
}

Eigen::Quaterniond Euler(double x, double y, double z)
{
	return Eigen::Quaterniond(
		Eigen::AngleAxisd(x, Vector3d::UnitX()) *
		Eigen::AngleAxisd(y, Vector3d::UnitY()) *
		Eigen::AngleAxisd(z, Vector3d::UnitZ())
	);
}

// Returns an AngleAxis representation of X by Y by Z (the same order as used
// in the preintegration factor).
Eigen::AngleAxisd FromEuler(Eigen::Vector3d d)
{
	return Eigen::AngleAxisd((
		Eigen::AngleAxisd(d.x(), Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(d.y(), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(d.z(), Eigen::Vector3d::UnitZ())
		));
}

TEST(IMU, PreIntegration1) {

	using namespace transforms;
	// Tests the preintegration factor with a simple acceleration orthogonal
	// to gravity. The IMU sample is integrated over one second. The inertial
	// frame is the same as the navigation frame. The implementation integrates
	// the velocity before position, so the final position is the same as the
	// acceleration.

	Transformd begin;
	Transformd end;
		
	auto preIntegrationFactor = new imu::PreIntegrationFactor(&begin, &end);

	// Create some simulated measurements. These are in the world frame.

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d acceleration = gravity + motion;

	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), 1.0);

	Problem problem;

	preIntegrationFactor->addToProblem(problem);

	// Prevent the solver moving the begin.

	problem.SetParameterBlockConstant(begin.parameterBlock());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->velocity->data());

	// Run the solver!
	Solver::Options options;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), -motion); //TODO:: why is this reversed?
}


TEST(IMU, PreIntegration2)
{
	using namespace transforms;

	// Tests the preintegration factor with an acceleration orthogonal to
	// gravity, and a rotation around the gravitational axis.
	// The inertial frame remains the same as the world frame. The end
	// pose should be the same as the acceleration and rotation (as they
	// are integrated over one second).

	Transformd begin;
	Transformd end;

	auto preIntegrationFactor = new imu::PreIntegrationFactor(&begin, &end);

	// Create some simulated measurements

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d acceleration = gravity + motion;
	Vector3d rotation(0, 1.5708, 0);

	preIntegrationFactor->addSample(acceleration, rotation, 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(begin.parameterBlock());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->velocity->data());

	// Run the solver!
	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), -motion); //TODO:: why is this reversed?
	CheckRotation(end.Rotation(), AngleAxisd(rotation.y(), Vector3d::UnitY()));
}


TEST(IMU, PreIntegration3)
{
	using namespace transforms;

	// Tests the preintegration factor with a simple acceleration, but where the
	// inertial frame is rotated with respect to the navigation frame.
	// As the dps is zero, the rotation should not change, and the position
	// should be the motion.

	AngleAxisd initialRotation(1.5708, Vector3d::UnitX());

	Transformd begin(initialRotation);
	Transformd end;

	auto preIntegrationFactor = new imu::PreIntegrationFactor(&begin, &end);

	// Create some simulated measurements, given in the world (navigation) frame

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);

	// Transform these measurements into the the inertial frame (the inverse of
	// the initial transform).

	Vector3d acceleration = (initialRotation.inverse() * gravity) + (initialRotation.inverse() * motion);

	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(begin.parameterBlock());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->velocity->data());

	// Run the solver!
	Solver::Options options;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), -motion); //TODO:: why is this reversed?
	CheckRotation(end.Rotation(), initialRotation);
}

TEST(IMU, PreIntegration4)
{
	using namespace transforms;

	// Tests the preintegration factor with a change in inertial position and
	// acceleration, with a non-zero inertial frame rotation.
	// The end pose should be the integration of the acceleration and rotations
	// given in the world frame.

	AngleAxisd initialRotation(1.5708, Vector3d::UnitX());

	Transformd begin(initialRotation);
	Transformd end;

	auto preIntegrationFactor = new imu::PreIntegrationFactor(&begin, &end);

	// The simulated inertial measurements, given in the world frame

	Vector3d gravity(0, -9.8, 0);
	Vector3d motion(0, 0, 0.5);
	Vector3d rotation(0, 1.5780, 0);

	// Transform them into the inertial frame (inverse of the initial transform)

	Vector3d acceleration = (initialRotation.inverse() * gravity) + (initialRotation.inverse() * motion);

	preIntegrationFactor->addSample(acceleration, (initialRotation.inverse() * rotation), 1.0);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(begin.parameterBlock());

	// And the initial body velocity is considered zero

	problem.SetParameterBlockConstant(preIntegrationFactor->velocity->data());

	// Run the solver!
	Solver::Options options;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = false;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), -motion); //TODO:: why is this reversed?

	// The expected rotation is the sequential concatenation of each rotation

	CheckRotation(end.Rotation(), (FromEuler(rotation) * initialRotation));
}

TEST(IMU, GravityIntegration)
{
	using namespace transforms;

	// This checks whether the gravity computation is correct. The integration
	// should match that of the acceleration.
	// When the IMU measures nothing but gravity, the estimated position should
	// be unchanged.

	Transformd start;
	Transformd end;

	auto preIntegrationFactor = new imu::PreIntegrationFactor(&start, &end);

	double dt = 0.01;

	Vector3d gravity(0, -9.8, 0);

	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(gravity, Vector3d::Zero(), dt);

	EXPECT_VECTOR(preIntegrationFactor->deltaGravityPosition, preIntegrationFactor->deltaPosition);

	Problem problem;
	preIntegrationFactor->addToProblem(problem);
	problem.SetParameterBlockConstant(start.parameterBlock());
	problem.SetParameterBlockConstant(preIntegrationFactor->velocity->data());
	Solver::Options options;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_VECTOR(end.Position(), Vector3d::Zero());
}

TEST(IMU, DISABLED_CalibrationFactor1)
{
	using namespace transforms;
	using namespace imu;
	using namespace motion;

	// Tests if the ImuCalibrationFactor can recover the relative orientation
	// of the Imu when it starts with no initial orientation.

	Problem problem;
	ImuCalibrationParameters calibration;
	MotionFrame reference;

	// Make calibration non-zero so we test if the solver comes up
	// with the correct answer of zero

	calibration.accelerometerBias = Vector3d(0.1, 0.2, 0.1);

	// Synthetic acceleration and rotation measurements
	Vector3d gravity(0, -9.8, 0);
	Vector3d acceleration(0, 0, 0);
	Vector3d rotation(0, 0, 0);
	Transformd trueImuPose;
	Vector3d measuredAcceleration = trueImuPose.Rotation().inverse() * gravity;

	auto factor = new ImuCalibrationFactor(problem, &calibration, reference, measuredAcceleration, rotation);

	Solver::Options options;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	// In this sanity-test check, gravity should counteract the estimate and
	// everything else should be close to zero. Be aware that the bias and local
	// transforms can counter-compensate so the tolerances here aren't too low.

	CheckVector(calibration.accelerometerBias, Vector3d::Zero(), 0.05);
	CheckVector(calibration.gyroscopeBias, Vector3d::Zero(), 0.05);
	CheckVector(calibration.local.Position(), Vector3d::Zero(), 0.01);
}

motion::MotionFrame convert(MockImu::State state)
{
	motion::MotionFrame f;
	f.acceleration = state.linearAcceleration;
	f.angularVelocity = state.angularVelocity;
	f.pose = transforms::Transformd(state.position, state.rotation);
	return f;
}

TEST(IMU, DISABLED_CalibrationFactor2)
{
	using namespace transforms;
	using namespace motion;
	using namespace imu;

	// Using the mock-IMU, see if the CalibrationFactor can recover the local
	// transform and biases from a sequence of Imu samples and reference 
	// observations.

	Problem problem;

	// The calibration is estimated by multiple calibration factors and estimates
	// the local offset and biases.

	ImuCalibrationParameters calibration;

	// Set up the synthetic Imu

	// Remember in the current version there is no support for position offsets!

	Transformd local(Vector3d(0, 0, 0.0), Euler(0.1, 2, 0));

	MockImu::Settings settings; // Default
	MockImu imu(Trajectory(trajectory1), settings, local.Position(), local.Rotation().toQuaternion(), 1.0);

	// Create a factor for each Imu sample and solve

	for (size_t i = 0; i < imu.frames.size(); i++)
	{
		MotionFrame reference = convert(imu.frames[i].reference);
		auto inertial = imu.frames[i].imu;
		auto factor = new ImuCalibrationFactor(problem, &calibration, reference, inertial.linearAcceleration, inertial.angularVelocityDps);

		std::cout << std::endl;
		imu.frames[i].reference.ToString1(std::cout);
		std::cout << std::endl;
		imu.frames[i].imu.ToString1(std::cout);
	}

	Solver::Options options;
	Solver::Summary summary;

	options.minimizer_progress_to_stdout = true;

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	// Compare the calibration with the synthetic imu settings

	CheckVector(local.Position(), calibration.local.Position(), 0.01);
	CheckRotation(local.Rotation(), calibration.local.Rotation());

	// For now the biases are zero

	CheckVector(Vector3d::Zero(), calibration.accelerometerBias, 0.01);
	CheckVector(Vector3d::Zero(), calibration.gyroscopeBias, 0.01);
}


TEST(IMU, OrientationFactor)
{
	using namespace transforms;

	// Tests if the ImuOrientationFactor can recover the relative orientation
	// of the Imu when it starts with no initial motion.

	AngleAxisd initialRotation(0.9, Vector3d::UnitX());
	Transformd referenceFrame;
	Transformd imuFrame;

	// Simulated inertial measurements

	Vector3d gravity = initialRotation.inverse() * Vector3d(0, -9.8, 0);
	Vector3d motion(0, 0, 0);
	Vector3d rotation(0, 0, 0);
	Vector3d acceleration = gravity + motion;

	auto orientationFactor = new imu::ImuOrientationFactor(&referenceFrame, &imuFrame);
	orientationFactor->addSample(acceleration, rotation, 1.0);

	Problem problem;
	orientationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(referenceFrame.parameterBlock());

	Solver::Options options;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = false;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	CheckRotation(orientationFactor->start->Rotation(), initialRotation);
}

TEST(IMU, DISABLED_WithOrientation)
{
	using namespace transforms;

	// Tests with initial orientation, calibrated using an ImuOrientationFactor 

	Transformd referenceFrame;
	Transformd start;
	Transformd end;

	// Simulated inertial measurements

	AngleAxisd initialRotation(0.9, Vector3d::UnitX());
	Vector3d gravity = initialRotation.inverse() * Vector3d(0, -9.8, 0);
	Vector3d motion(0, 0, 0);
	Vector3d rotation(0, 0, 0);
	Vector3d acceleration = gravity + motion;
	double dt = 0.01;

	// The problem description

	auto orientationFactor = new imu::ImuOrientationFactor(&referenceFrame, &start);

	Problem problem;
	Solver::Options options;
	options.gradient_check_relative_precision = 1e-5; // This is verified by manually checking the differences
	options.check_gradients = false;
	Solver::Summary summary;

	// The first step is to add an IMU sample when its pose is known, and solve

	orientationFactor->addToProblem(problem);

	problem.SetParameterBlockConstant(referenceFrame.parameterBlock());
	problem.SetParameterBlockVariable(start.parameterBlock());
	orientationFactor->addSample(acceleration, rotation, dt);
	orientationFactor->addSample(acceleration, rotation, dt);
	orientationFactor->addSample(acceleration, rotation, dt);

	// Before solving, all poses should be in their default states

	CheckPose(referenceFrame, Vector3d::Zero(), AngleAxisd());
	CheckPose(start, Vector3d::Zero(), AngleAxisd());
	CheckPose(end, Vector3d::Zero(), AngleAxisd());

	// After solving for the calibration, the start pose should be updated.
	// The end pose doesn't matter.

	ceres::Solve(options, &problem, &summary);

	CheckPose(referenceFrame, Vector3d::Zero(), AngleAxisd());
	CheckPose(start, Vector3d::Zero(), initialRotation);

	// The second step is to do the imu integration. The gravity vector will be
	// off-axis because the imu is rotated. During the solve this should be
	// compensated for, leaving the end position at 0,0,0.

	auto preIntegrationFactor = new imu::PreIntegrationFactor(&start, &end);
	preIntegrationFactor->addToProblem(problem);

	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);
	preIntegrationFactor->addSample(acceleration, Vector3d::Zero(), dt);

	problem.SetParameterBlockConstant(start.parameterBlock());
	ceres::Solve(options, &problem, &summary);

	EXPECT_VECTOR(end.Position(), Vector3d::Zero());
}