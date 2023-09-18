#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <glog/logging.h>
#include <DenavitHartenberg.h>

#define TOL 1e-2

// EXPECT_ANGLE has a default precision of 1 degree
#define EXPECT_ANGLE(a1,a2) \
	EXPECT_NEAR(a1,a2,0.0174533)

#define EXPECT_VECTOR(v1,v2) \
	EXPECT_NEAR(v1.x(),v2.x(),TOL);\
	EXPECT_NEAR(v1.y(),v2.y(),TOL);\
	EXPECT_NEAR(v1.z(),v2.z(),TOL);

TEST(DenavitHartenberg, Single) {

	// This test checks the behaviour of a Denavit Hartenberg Single Joint
	// cost functor.
	// It does this by connecting two Pose parameter blocks with a faulty
	// prior and seeing if the solver can work out the correct joint angle.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	// Create two poses, offset by a unit vector, and connected by a DH joint
	// that is pointing in the wrong direction (away from x2)

	auto x1 = new Pose3d();
	auto x2 = new Pose3d(Vector3d::UnitZ());
	auto joint = new dh::Joint();

	joint->theta = 1.5; // Start with a wrong value and see if it can get the right one
	joint->start = x1;
	joint->end = x2;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	// Fix x1 (the reference frame) and x2, the end frame, meaning the solver
	// must optimise theta to resolve the chain.

	problem.SetParameterBlockConstant(x1->parameterBlock());
	problem.SetParameterBlockConstant(x2->parameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);
	EXPECT_ANGLE(joint->theta, 0);

	// Run the test again but this time move x2 in another direction. The local
	// joint axis is always Y, facing along Z, so the test position must match
	// this convention for the angles to be comparable.

	x2->Rotation() = AngleAxisd(2.34, Vector3d::UnitY());
	x2->Position() = x2->Rotation() * Vector3d::UnitZ();

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);
	EXPECT_ANGLE(joint->theta, 2.34);
}

TEST(DenavitHartenberg, StartFromZero) {

	// This test is almost identical to the above, but checks if the joint can
	// solve when it starts at zero.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	// Create two poses, offset by a unit vector, and connected by a DH joint
	// that is pointing in the wrong direction (away from x2)

	auto x1 = new Pose3d();
	auto x2 = new Pose3d(AngleAxisd(2.34, Vector3d::UnitY()) * Vector3d::UnitZ(), AngleAxisd(2.34, Vector3d::UnitY()));
	auto joint = new dh::Joint();

	joint->theta = 0.0;
	joint->start = x1;
	joint->end = x2;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	// Fix x1 (the reference frame) and x2, the end frame, meaning the solver
	// must optimise theta to resolve the chain.

	problem.SetParameterBlockConstant(x1->parameterBlock());
	problem.SetParameterBlockConstant(x2->parameterBlock());

	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);
	EXPECT_ANGLE(joint->theta, 2.34);
}

TEST(DenavitHartenberg, CoincidentPointObservation) {

	// This test checks the behaviour of a single Denavit Hartenberg cost functor
	// when connecting a cost block with an observation.
	// The observation is a 3d point (no rotation) point with no offset.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	auto x1 = new Pose3d();
	auto x2 = new Pose3d();

	// Constrain x2 to x1 via single dh joint

	auto joint = std::make_shared<dh::Joint>();
	joint->a = 0;
	joint->d = 0;
	joint->r = 1;
	joint->theta = 1.5; // Start with a wrong value and see if it can get the right one
	joint->start = x1;
	joint->end = x2;

	// x2 is unintialised in this test and will be solved for

	auto expected_observation_point = AngleAxisd(2.3, Vector3d::UnitY()) * Vector3d::UnitZ();

	auto m = new PointMeasurement();
	m->pose = x2;
	m->offset = Vector3d::Zero();
	m->point = expected_observation_point;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	problem.AddResidualBlock(
		m->costFunction(),
		nullptr,
		m->parameterBlocks()
	);

	// Fix the reference frame, and the observation

	problem.SetParameterBlockConstant(x1->parameterBlock());
	problem.SetParameterBlockConstant(m->pointParameterBlock());
	problem.SetParameterBlockConstant(m->offsetParameterBlock());


	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);
	EXPECT_NEAR(joint->theta, 2.3, TOL);
	EXPECT_VECTOR(x2->Position(), expected_observation_point);
}

TEST(DenavitHartenberg, OffsetPointObservation) {

	// This test checks the behaviour of a single Denavit Hartenberg cost functor
	// when connecting a cost block with an observation.
	// The observation is a 3d point (no rotation) point with a small offset.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	auto x1 = new Pose3d();
	auto x2 = new Pose3d();

	// Constrain x2 to x1 via single dh joint

	auto joint = std::make_shared<dh::Joint>();
	joint->theta = 1.5; // Start with a wrong value and see if it can get the right one
	joint->start = x1;
	joint->end = x2;

	// x2 is unintialised in this test and will be solved for


	// When setting the offset we must take care, because while x1 can have any
	// rotation, x2's is determined by the joint.
	// Therefore, the observation must consider where x2 will be when estimated
	// correctly for the solver to have a chance (either the offset, or the
	// point).

	auto expected_offset = Vector3d::Ones();
	auto expected_x2_pose = Pose3d(AngleAxisd(1.7, Vector3d::UnitY())) * Pose3d(Vector3d::UnitZ());
	auto expected_observation_point = expected_x2_pose * expected_offset;

	auto m = new PointMeasurement();
	m->pose = x2;
	m->offset = expected_offset;
	m->point = expected_observation_point;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	problem.AddResidualBlock(
		m->costFunction(),
		nullptr,
		m->parameterBlocks()
	);

	// Fix the reference frame, and the observation

	problem.SetParameterBlockConstant(x1->parameterBlock());
	problem.SetParameterBlockConstant(m->pointParameterBlock());
	problem.SetParameterBlockConstant(m->offsetParameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);
	EXPECT_NEAR(joint->theta, 1.7, TOL);
	EXPECT_VECTOR(x2->Position(), expected_x2_pose.Position());
}

TEST(DenavitHartenberg, MultiChainOnly) {

	// This test creates a chain of two Denavit Hartenberg single joint functors,
	// connecting three poses. The first and final pose have coincident observations.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	auto x1 = new Pose3d();
	auto x2 = new Pose3d();
	auto x3 = new Pose3d();

	auto joint1 = std::make_shared<dh::Joint>();
	joint1->start = x1;
	joint1->end = x2;
	joint1->theta = 0.1;

	auto joint2 = std::make_shared<dh::Joint>();
	joint2->start = x2;
	joint2->end = x3;
	joint2->theta = 0.9;

	auto m1 = new PointMeasurement();
	m1->pose = x1;

	auto m2 = new PointMeasurement();
	m2->pose = x3;
	m2->point = Vector3d(0.5, 0, 0.8);

	Problem problem;

	problem.AddResidualBlock(
		joint1->costFunction(),
		nullptr,
		joint1->parameterBlocks()
	);

	problem.AddResidualBlock(
		joint2->costFunction(),
		nullptr,
		joint2->parameterBlocks()
	);

	problem.AddResidualBlock(
		m1->costFunction(),
		nullptr,
		m1->parameterBlocks()
	);

	problem.AddResidualBlock(
		m2->costFunction(),
		nullptr,
		m2->parameterBlocks()
	);

	// Fix the reference frame, and the observation

	problem.SetParameterBlockConstant(m1->pointParameterBlock());
	problem.SetParameterBlockConstant(m1->offsetParameterBlock());
	problem.SetParameterBlockConstant(m2->pointParameterBlock());
	problem.SetParameterBlockConstant(m2->offsetParameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_VECTOR(x3->Position(), m2->point);

	EXPECT_NEAR((x1->Position() - x2->Position()).norm(), 1, TOL);
	EXPECT_NEAR((x2->Position() - x3->Position()).norm(), 1, TOL);
}

TEST(DenavitHartenberg, MultiChainMultiPointObservation) {

	// *Note this test is broken because the end effector orientation will be
	// constrained by the kinematics of the chain but thats not reflected in
	// the test paramters*

	// This test creates a chain of two Denavit Hartenberg single joint functors,
	// connecting three poses. The final pose has two observations that require
	// a rotation to resolve.

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	auto x1 = new Pose3d();
	auto x2 = new Pose3d();
	auto x3 = new Pose3d();

	auto joint1 = std::make_shared<dh::Joint>();
	joint1->start = x1;
	joint1->end = x2;
	joint1->theta = 0.1;

	auto joint2 = std::make_shared<dh::Joint>();
	joint2->start = x2;
	joint2->end = x3;
	joint2->theta = 0.9;

	// This is the pose of the end effector in world space that the solver
	// should reach.
	// It should be used to work out the world space observations.

	auto expected_x3_pose = new Pose3(Vector3d(0.5, 0, 0.9), AngleAxisd(0.8, Vector3d::UnitY()));

	auto mx31 = new PointMeasurement();
	mx31->pose = x3;
	mx31->offset = Vector3d(0.1, 0, 0.2);
	mx31->point = (*expected_x3_pose) * mx31->offset;

	auto mx32 = new PointMeasurement();
	mx32->pose = x3;
	mx32->offset = Vector3d(0.5, 1, 0.1);
	mx32->point = (*expected_x3_pose) * mx32->offset;

	Problem problem;

	problem.AddResidualBlock(
		joint1->costFunction(),
		nullptr,
		joint1->parameterBlocks()
	);

	problem.AddResidualBlock(
		joint2->costFunction(),
		nullptr,
		joint2->parameterBlocks()
	);

	problem.AddResidualBlock(
		mx31->costFunction(),
		nullptr,
		mx31->parameterBlocks()
	);

	problem.AddResidualBlock(
		mx32->costFunction(),
		nullptr,
		mx32->parameterBlocks()
	);

	// Fix the reference frame, and the observation

	problem.SetParameterBlockConstant(mx31->pointParameterBlock());
	problem.SetParameterBlockConstant(mx31->offsetParameterBlock());
	problem.SetParameterBlockConstant(mx32->pointParameterBlock());
	problem.SetParameterBlockConstant(mx32->offsetParameterBlock());
	problem.SetParameterBlockConstant(x1->parameterBlock());

	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	// Has the end effector ended up in the right place?
	EXPECT_VECTOR(x3->Position(), expected_x3_pose->Position());

	// Are the joint lengths satisified?
	EXPECT_NEAR((x1->Position() - x2->Position()).norm(), 1, TOL);
	EXPECT_NEAR((x2->Position() - x3->Position()).norm(), 1, TOL);

}

TEST(DenavitHartenberg, JointLimit) {

	// This test checks the behaviour of a Denavit Hartenberg JointLimits object

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	// Create two poses, offset by a unit vector, and connected by a DH joint
	// that is pointing in the wrong direction (away from x2)

	auto joint = new dh::Joint();

	joint->start = new Pose3d();
	joint->end = new Pose3d();
	joint->theta = 0.0;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	problem.SetParameterBlockConstant(joint->start->parameterBlock());

	// (We use measurements instead of setting end directly, so we don't have to
	// work out the appropriate orientation)

	auto measurement = new PointMeasurement();
	measurement->point = (AngleAxisd(2.2, Vector3d::UnitY()) * Vector3d::UnitZ());
	measurement->pose = joint->end;

	problem.AddResidualBlock(
		measurement->costFunction(),
		nullptr,
		measurement->parameterBlocks()
	);

	problem.SetParameterBlockConstant(measurement->offsetParameterBlock());
	problem.SetParameterBlockConstant(measurement->pointParameterBlock());


	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	// First verify that the problem above is solvable

	EXPECT_ANGLE(joint->theta, 2.2);

	// Now add the limits and solve again

	auto limits = new JointLimit(joint, 1.0, 1.5, 10.0);
	limits->addToProblem(problem);

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	// The limits in this case conflict with the ideal solution. The actual
	// result should be halfway between them.

	EXPECT_GE(joint->theta, 1.0);
	EXPECT_LE(joint->theta, 2.1);
}

TEST(DenavitHartenberg, JointManifold) {

	// This test checks the behaviour of a Denavit Hartenberg JointLimits object

	using namespace Eigen;
	using namespace hs;
	using namespace dh;
	using namespace ceres;

	// Create two poses, offset by a unit vector, and connected by a DH joint
	// that is pointing in the wrong direction (away from x2)

	auto joint = new dh::Joint();

	joint->start = new Pose3d();
	joint->end = new Pose3d();
	joint->theta = 0.0;

	Problem problem;

	problem.AddResidualBlock(
		joint->costFunction(),
		nullptr,
		joint->parameterBlocks()
	);

	problem.SetParameterBlockConstant(joint->start->parameterBlock());

	// (We use measurements instead of setting end directly, so we don't have to
	// work out the appropriate orientation)

	auto measurement = new PointMeasurement();
	measurement->point = (AngleAxisd(2.2, Vector3d::UnitY()) * Vector3d::UnitZ());
	measurement->pose = joint->end;

	problem.AddResidualBlock(
		measurement->costFunction(),
		nullptr,
		measurement->parameterBlocks()
	);

	problem.SetParameterBlockConstant(measurement->offsetParameterBlock());
	problem.SetParameterBlockConstant(measurement->pointParameterBlock());


	Solver::Options options;
	options.linear_solver_type = ceres::DENSE_QR;
	options.minimizer_progress_to_stdout = false;
	Solver::Summary summary;

	//ceres::Solve(options, &problem, &summary);

	// First verify that the problem above is solvable

	EXPECT_ANGLE(joint->theta, 2.2);

	// Now add the limits and solve again

	auto functor = new JointManifoldFunctor(1.0, 1.5);
	auto manifold = new AutoDiffManifold<JointManifoldFunctor, 1, 1>(functor);
	problem.SetManifold(joint->parameterBlock(), manifold);

	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, ceres::TerminationType::CONVERGENCE);

	EXPECT_GE(joint->theta, 1.5);
	EXPECT_LE(joint->theta, 2.1);
}