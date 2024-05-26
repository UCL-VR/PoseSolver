#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <ceres/ceres.h>
#include <ceres/gradient_checker.h>
#include <glog/logging.h>
#include <Transforms.h>
#include <PointMeasurement.h>

using namespace Eigen;
using namespace ceres;

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

void CheckAngle(double a)
{
	if (a > PI * 0.5)
	{
		a = PI - a;
	}
	EXPECT_NEAR(a, 0, 0.0174533); // Expect accurate to 1 degree
}

void CheckRotation(transforms::Rodriguesd a, Eigen::AngleAxisd b)
{
	CheckAngle(a.toQuaternion().angularDistance(Eigen::Quaterniond(b)));
}

void CheckRotation(transforms::Rodriguesd a, Eigen::Quaterniond b)
{
	CheckAngle(a.toQuaternion().angularDistance(b));
}

void CheckRotation(transforms::Rodriguesd a, transforms::Rodriguesd b)
{
	CheckAngle(a.toQuaternion().angularDistance(b.toQuaternion()));
}

TEST(Transform, Composition) {

	using namespace transforms;

	// This test checks that the Transform Composition operator is working correctly
	
	// Two offsets only in the same axis

	const auto r1 = Transformd(Vector3d(10, 0, 0)) * Transformd(Vector3d(10, 0, 0));
	EXPECT_VECTOR(r1.Position(), Vector3d(20, 0, 0));

	// Two offsets in different axes

	const auto r2 = Transformd(Vector3d(10, 10, 0)) * Transformd(Vector3d(10, 0, 0));
	EXPECT_VECTOR(r2.Position(), Vector3d(20, 10, 0));

	// Offset and rotation

	const auto r3 = Transformd(AngleAxisd(1.5708, Vector3d::UnitX())) * Transformd(Vector3d(0, 10, 0));
	EXPECT_VECTOR(r3.Position(), Vector3d(0, 0, 10));
}

TEST(Transform, Inverse)
{
	using namespace transforms;

	// Checks the inverse method of Transform

	auto position = Vector3d(1.5, 2, 9);
	auto rotation = AngleAxis(0.74, Vector3d(0.7, 0.2, 0.9));
	auto transform = Transformd(position, rotation);

	EXPECT_VECTOR(Vector3d::Zero(), (transform * transform.inverse() * Vector3d::Zero()));
	EXPECT_VECTOR(Vector3d::Zero(), (transform.inverse() * transform * Vector3d::Zero()));
}

TEST(Transform, VectorOperator) 
{
	using namespace transforms;

	// This contains a number of tests of the vector transform overload.
	// Add more edge cases as they are found.
	
	Vector3d v;

	// An offset with no rotation should be straightforward vector addition

	v = Transformd(Vector3d(1, 2, 3)) * Vector3d::Zero();
	EXPECT_VECTOR(v, Vector3d(1, 2, 3));

	v = Transformd(Vector3d(1, 2, 3)) * Vector3d(9, 7, 4);
	EXPECT_VECTOR(v, Vector3d(1 + 9, 2 + 7, 3 + 4));

	// A pure rotation of a zero length vector should do nothing

	v = Transformd(AngleAxisd(1.2, Vector3d::UnitX())) * Vector3d::Zero();
	EXPECT_VECTOR(v, Vector3d::Zero());

	// A rotation of 90 degrees around the Y axis should turn a unit length
	// vector in the x axis into one in the z axis

	v = Transformd(AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, Vector3d::UnitZ());

	// The offset should follow the rotation, 

	v = Transformd(Vector3d::UnitZ(), AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, (Vector3d::UnitZ() * 2));

	v = Transformd(Vector3d::UnitX() * 10, AngleAxisd(-RAD(90), Vector3d::UnitY())) * Vector3d::UnitX();
	EXPECT_VECTOR(v, (Vector3d::UnitZ() + Vector3d::UnitX() * 10));
}

struct Between1 {
	template <typename T>
	bool operator()(const T* const a, const T* const b, T* residual) const {

		auto posea = transforms::Transform<T>::Map(a);
		auto poseb = transforms::Transform<T>::Map(b);
		transforms::Transform<T>::Between(posea, poseb, residual);

		return true;
	}

	static void AddToProblem(Problem& problem, transforms::Transformd& x1, transforms::Transformd& x2)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<
			Between1,
			transforms::Transformd::Residuals,
			transforms::Transformd::Dimension,
			transforms::Transformd::Dimension >(new Between1()),
			nullptr,
			{
				x1.parameterBlock(),
				x2.parameterBlock()
			}
		);
	}
};

TEST(Transform, Between) {

	// Checks the Between Method of Transform in a Cost Functor, where the poses
	// are mapped between two Parameter Blocks

	transforms::Transformd x1(Eigen::Vector3d(10, 0, 0), Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitX()));
	transforms::Transformd x2;

	Problem problem;

	Between1::AddToProblem(problem, x1, x2);

	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

        // Output the results
	std::cout << summary.FullReport() << std::endl;
	std::cout << "Estimated solution " << x2.ToString() << std::endl;

	EXPECT_EQ(summary.termination_type, 0);
	EXPECT_VECTOR(x1.Position(), x2.Position());
	CheckRotation(x2.Rotation(), x2.Rotation());
}

TEST(Transform, FixedBlocks)
{
	// Checks that the solver can optimise one Pose to match another using the
	// Between method, when one paramter block is fixed

	transforms::Transformd x1;
	transforms::Transformd x2(Eigen::Vector3d(10, 1, 0));

	Problem problem;

	// Constraints x to z

	Between1::AddToProblem(problem, x1, x2);
	problem.SetParameterBlockConstant(x2.parameterBlock());

	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;
	
	ceres::Solve(options, &problem, &summary);

        // Output the results
        //std::cout << summary.FullReport() << std::endl;
        //std::cout << "Estimated solution " << x2.ToString() << std::endl;

	EXPECT_EQ(summary.termination_type, 0);
	EXPECT_VECTOR(x2.Position(), Eigen::Vector3d(10, 1, 0));
	EXPECT_VECTOR(x1.Position(), x2.Position());
}

// In this Cost Functor, the fixed pose is cast using the Cast<> method

struct Between2 {
	template <typename T>
	bool operator()(const T* const a, T* residual) const {
		transforms::Transform<T>::Between(transforms::Transform<T>::Map(a), b.Cast<T>(), residual);
		return true;
	}

	transforms::Transformd b;

	Between2(transforms::Transformd pose):b(pose){ }

	static void AddToProblem(Problem& problem, transforms::Transformd& x1, transforms::Transformd x2)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<
			Between2,
			transforms::Transformd::Residuals,
			transforms::Transformd::Dimension >(new Between2(x2)),
			nullptr,
			{
				x1.parameterBlock(),
			}
		);
	}
};

TEST(Transform, Measurement1)
{
	// Tests using a fixed Pose as a measurement. This uses the Between2 factor,
	// which Maps one Pose parameter block, and Casts a fixed Transform type to the
	// Jets

	transforms::Transformd x1;
	transforms::Transformd x2(Eigen::Vector3d(10, 1, 0));

	Problem problem;
	Between2::AddToProblem(problem, x1, x2);

	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, 0);
	EXPECT_VECTOR(x1.Position(), x2.Position());
}

struct Between3 {
	template <typename T>
	bool operator()(const T* const position, const T* const rotation, T* residual) const {
		
		transforms::Transform<T> p1 = transforms::Transform<T>(Eigen::Vector3<T>::Map(position), transforms::Rodrigues<T>::Map(rotation));
		transforms::Transform<T> p2 = b.Cast<T>();
		transforms::Transform<T>::Between(p1, p2, residual);
		return true;
	}

	transforms::Transformd b;

	Between3(transforms::Transformd pose) :b(pose) { }

	static void AddToProblem(Problem& problem, Eigen::Vector3d& position, transforms::Rodriguesd& rotation, transforms::Transformd x2)
	{
		problem.AddResidualBlock(
			new ceres::AutoDiffCostFunction<
			Between3,
			transforms::Transformd::Residuals,
			3,
			3 >(new Between3(x2)),
			nullptr,
			{
				(double*)(&position),
				(double*)(&rotation)
			}
		);
	}
};

TEST(Transform, Components)
{
	// Tests whether a Transform can be mapped from a position and rotation
	// separately in a Cost Function

	transforms::Transformd x1(Eigen::Vector3d(9, 7, 1.5), Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.7, 0.7, 0)));
	Eigen::Vector3d position;
	transforms::Rodriguesd rotation;

	Problem problem;
	Between3::AddToProblem(problem, position, rotation, x1);

	Solver::Options options;
	options.check_gradients = true;
        options.gradient_check_relative_precision = 10;//e-7;
        
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, 0);
	EXPECT_VECTOR(x1.Position(), position);
	CheckRotation(x1.Rotation(), rotation);
}


TEST(Transform, PointObservation)
{
	using namespace transforms;
	using namespace observations;

	// Tests the PointObservation with a simple offset

	Vector3d observation(10, 21, 9);
	Transformd x;
	auto p1 = new PointMeasurement(&x); // Do not allocate this on the stack because the solver will take ownership of it later

	p1->point = observation;

	Problem problem;
	p1->addToProblem(problem);

	// Optimise only the pose

	problem.SetParameterBlockConstant(p1->pointParameterBlock());
	problem.SetParameterBlockConstant(p1->offsetParameterBlock());

	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;

	ceres::Solve(options, &problem, &summary);

	EXPECT_VECTOR(x.Position(), observation);
	EXPECT_VECTOR(p1->point, observation);
	EXPECT_VECTOR(p1->offset, Vector3d::Zero());
}

TEST(Transform, PointObservation2)
{
	using namespace transforms;
	using namespace observations;

	// Tests the PointObservation, where two PointObservations are attached to
	// one Pose.
	// The solver must rotate the pose in this case to satisfy both constraints.

	Transformd x;

	AngleAxisd rotation(AngleAxisd(0.7, Vector3d::UnitY()));
	Vector3d observation1(10, 21, 9);
	Vector3d observation2 = observation1 + (rotation * Vector3d::UnitZ());
	
	auto p1 = new PointMeasurement(&x);
	auto p2 = new PointMeasurement(&x);
	 
	p1->point = observation1;
	p2->point = observation2;

	p1->offset = Vector3d::Zero();
	p2->offset = Vector3d::UnitZ(); // This is the same length as observation2 - observation1, but in a different direction.

	Problem problem;

	p1->addToProblem(problem);
	p2->addToProblem(problem);

	problem.SetParameterBlockConstant(p1->pointParameterBlock());
	problem.SetParameterBlockConstant(p1->offsetParameterBlock());

	problem.SetParameterBlockConstant(p2->pointParameterBlock());
	problem.SetParameterBlockConstant(p2->offsetParameterBlock());

	Solver::Options options;
	options.check_gradients = false;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, 0);

	EXPECT_VECTOR(x.Position(), observation1);
	EXPECT_VECTOR(p1->point, observation1);
	EXPECT_VECTOR(p2->point, observation2);
	EXPECT_VECTOR((x * p2->offset), observation2);
	CheckRotation(x.Rotation(), rotation);
}

// Can the solver match the angle of the pose?
struct AngleAxisCostFunctor
{
	template<typename T>
	bool operator()(const T* const a, const T* const b, T* residual) const {

		Eigen::Map<Eigen::Vector3<T>> r(&residual[0]);
		auto rotation = (transforms::Rodrigues<T>)Eigen::AngleAxis<T>(*b, Eigen::Vector3<T>::UnitX()); // This invokes the conversion operator
		r = (transforms::Transform<T>::Map(a).Rotation().inverse() * rotation).toVector();
		return true;
	}
};

TEST(Transform, AxisAngleCost)
{
	// This test checks whether the axis-angle conversion operator can be used
	// in a cost function

	using namespace transforms;
	using namespace Eigen;
	using namespace ceres;

	Transformd x1(AngleAxisd(1.5, Eigen::Vector3d::UnitX()));
	double angle = 0.000;

	Problem problem;

	problem.AddResidualBlock(
		new AutoDiffCostFunction<
		AngleAxisCostFunctor,
		3,
		6,
		1>(
			new AngleAxisCostFunctor()
		),
		nullptr,
		{
			x1.parameterBlock(),
			&angle
		}
	);

	problem.SetParameterBlockConstant(x1.parameterBlock());
		
	Solver::Options options;
	options.check_gradients = true;
	Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	EXPECT_EQ(summary.termination_type, 0);
}

