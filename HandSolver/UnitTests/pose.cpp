#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <Pose.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#define TOL 1e-4
#define PI 3.1415926535897932385
#define RAD(v1) (v1 * PI/180.0)

#define EXPECT_VECTOR(v1,v2) \
	EXPECT_NEAR(v1.x(),v2.x(),TOL);\
	EXPECT_NEAR(v1.y(),v2.y(),TOL);\
	EXPECT_NEAR(v1.z(),v2.z(),TOL);

TEST(Pose3, Composition) {

	// This test checks that the Pose3 Composition operator is working correctly

	using namespace Eigen;
	using namespace hs;

	const auto r1 = Pose3d(Vector3d(10, 0, 0)) * Pose3d(Vector3d(10, 0, 0));
	EXPECT_EQ(r1.Position().x(), 20.0);

	const auto r2 = Pose3d(Vector3d(10, 10, 0)) * Pose3d(Vector3d(10, 0, 0));
	EXPECT_EQ(r2.Position().x(), 20.0);
	EXPECT_EQ(r2.Position().y(), 10.0);

	const auto r3 = Pose3d(AngleAxisd(1.5708, Vector3d::UnitX())) * Pose3d(Vector3d(0, 10, 0));
	EXPECT_NEAR(r3.Position().z(), 10.0, TOL);
	EXPECT_NEAR(r3.Position().y(), 0.0, TOL);
}

TEST(Pose3, VectorOperator) 
{
	using namespace Eigen;
	using namespace hs;

	// This contains a number of tests of the vector transform overload.
	// Add more edge cases as they are found.
	
	Vector3d v;

	// An offset with no rotation should be straightforward vector addition

	v = Pose3d(Vector3d(1, 2, 3)) * Vector3d::Zero();
	EXPECT_VECTOR(v, Vector3d(1, 2, 3));

	v = Pose3d(Vector3d(1, 2, 3)) * Vector3d(9, 7, 4);
	EXPECT_VECTOR(v, Vector3d(1 + 9, 2 + 7, 3 + 4));

	// A pure rotation of a zero length vector should do nothing

	v = Pose3d(AngleAxisd(1.2, Vector3d::UnitX())) * Vector3d::Zero();
	EXPECT_VECTOR(v, Vector3d::Zero());

	// A rotation of 90 degrees around the Y axis should turn a unit length
	// vector in the x axis into one in the z axis

	v = Pose3(AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, Vector3d::UnitZ());

	// The offset should follow the rotation, 

	v = Pose3<double>(Vector3d::UnitZ(), AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, (Vector3d::UnitZ() * 2));

	v = Pose3<double>(Vector3d::UnitX() * 10, AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, (Vector3d::UnitZ() + Vector3d::UnitX() * 10));
}

TEST(Pose3, BetweenGradients)
{
	// Check the gradients of the Pose3Between functor against finite difference
	// using ceres' GradientChecker.

	hs::Pose3d x1(Eigen::Vector3d(10, 0, 0), Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitX()));
	hs::Pose3d x2;


	const ceres::CostFunction* cost_function = hs::PoseFunctions::Between::costFunction();
	const std::vector<const ceres::Manifold*> manifolds {nullptr, nullptr};
	ceres::NumericDiffOptions options;

	ceres::GradientChecker gc(cost_function, &manifolds, options);

	ceres::GradientChecker::ProbeResults probeResults;

	double** parameters = new double* [] { x1.parameterBlock(), x2.parameterBlock() };

	if (!gc.Probe(parameters, 0.000001, &probeResults))
	{
		FAIL() << probeResults.error_log;
	}
}

TEST(Pose3, Between) {

	// Can Ceres estimate the value of a Pose3, given that it should be coincident
	// with another.

	hs::Pose3d x1(Eigen::Vector3d(10, 0, 0), Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitX()));
	hs::Pose3d x2;

	Problem problem;

	/*
	std::cout << "x1" << " " << x1.ToString() << std::endl;
	std::cout << "x2" << " " << x2.ToString() << std::endl;
	*/

	problem.AddResidualBlock(hs::PoseFunctions::Between::costFunction(), nullptr, { x1.parameterBlock(), x2.parameterBlock()});

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	// Print residuals too

	hs::PoseFunctions::Between f;
	Eigen::Vector<double, 6> residuals;
	f(x1.parameterBlock(), x2.parameterBlock(), residuals.data());

	/*
	std::cout << "x1" << " " << x1.ToString() << std::endl;
	std::cout << "x2" << " " << x2.ToString() << std::endl;
	std::cout << "r" << residuals << std::endl;
	*/
}

#pragma optimize("", on)

TEST(Pose3, FixedBlocks)
{
	hs::Pose3d x;
	hs::Pose3d z(Eigen::Vector3d(10, 1, 0));

	Problem problem;

	// Constraints x to z

	problem.AddResidualBlock(
		hs::PoseFunctions::Between::costFunction(),
		nullptr,
		{ x.parameterBlock(), z.parameterBlock()}
	);

	problem.SetParameterBlockConstant(z.parameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	EXPECT_NEAR(x.Position().x(), 10, TOL);
	EXPECT_NEAR(x.Position().y(), 1, TOL);
	EXPECT_NEAR(x.Position().z(), 0, TOL);
}

TEST(Pose3, Measurement)
{
	hs::Pose3d x;
	hs::Pose3d z(Eigen::Vector3d(10, 1, 0));

	Problem problem;

	// Constraints x to z

	problem.AddResidualBlock(
		hs::PoseMeasurement::costFunction(z),
		nullptr,
		{ x.parameterBlock() }
	);

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;
	Solve(options, &problem, &summary);

	EXPECT_NEAR(x.Position().x(), 10, TOL);
	EXPECT_NEAR(x.Position().y(), 1, TOL);
	EXPECT_NEAR(x.Position().z(), 0, TOL);
}


TEST(Pose3, PointObservation)
{
	// Tests the PointObservation in a number of configurations

	using namespace hs;
	using namespace Eigen;
	using namespace ceres;

	Vector3d observation1(10, 21, 9);
	Vector3d observation2 = observation1 + Vector3d(sin(1), cos(1), 0);

	Pose3d x1;

	auto p1 = new PointMeasurement(); // Do not allocate this on the stack because the solver will take ownership of it later
	p1->pose = &x1;

	// The measurement for this test is a simple coincident measure

	p1->point = observation1;

	Problem problem;
	
	problem.AddResidualBlock(
		p1->costFunction(),
		nullptr,
		p1->parameterBlocks()
	);

	// Optimise only the pose

	problem.SetParameterBlockConstant(p1->pointParameterBlock());
	problem.SetParameterBlockConstant(p1->offsetParameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_NEAR((x1.Position() - observation1).norm(), 0, TOL);
	EXPECT_NEAR((p1->point - observation1).norm(), 0, TOL);
	EXPECT_NEAR((p1->offset - Vector3d::Zero()).norm(), 0, TOL);
	
	// In this second stage, we add another observation, with a local offset.
	// The only way to resolve this, is for the solver to add a rotation
	// to the pose3.

	auto p2 = new PointMeasurement();
	p2->pose = &x1;
	p2->point = observation2;
	p2->offset = Vector3d::UnitX();

	problem.AddResidualBlock(
		p2->costFunction(),
		nullptr,
		p2->parameterBlocks()
	);

	problem.SetParameterBlockConstant(p2->pointParameterBlock());
	problem.SetParameterBlockConstant(p2->offsetParameterBlock());

	// Solve again

	ceres::Solve(options, &problem, &summary);

	auto pose_distance = (x1.Position() - observation1).norm();
	auto observation2_world_projection = (x1 * Vector3d::UnitX());
	auto observation2_world_projection_error = (observation2_world_projection - observation2).norm();
	auto angle = AngleAxisd(x1.Rotation().toQuaternion());
	auto observation2_local_projection = angle * Vector3d::UnitX();

	EXPECT_NEAR(pose_distance, 0, TOL);
	EXPECT_NEAR(observation2_world_projection_error, 0, TOL);
}