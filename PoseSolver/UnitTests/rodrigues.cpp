#include "pch.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES
#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <glog/logging.h>
#include <ceres/gradient_checker.h>
#include "Transforms.h"

#define TOL 1e-4

#define EXPECT_Q(val) \
	EXPECT_NEAR(val, 0, TOL)

TEST(Rodrigues, DefaultConstructor) {

	using namespace transforms;
	Rodriguesd r; 
	EXPECT_EQ(r.data[0],0);
	EXPECT_EQ(r.data[1],0);
	EXPECT_EQ(r.data[2],0);
}

TEST(Rodrigues, QuaternionConversion) {

	// Tests if we can perform a lossless conversion to and from Quaternions,
	// using all constructors, operators and methods.

	using namespace transforms;
	using namespace Eigen;

	auto axis = Vector3d(0.1, 0.4, 0.7).normalized();
	auto q = Quaterniond(AngleAxisd(1.4, axis));

	Rodriguesd r;
	r = q;

	Rodrigues qr(q);

	EXPECT_EQ(r.data[0], qr.data[0]);
	EXPECT_EQ(r.data[1], qr.data[1]);
	EXPECT_EQ(r.data[2], qr.data[2]);

	auto rq = qr.toQuaternion();

	EXPECT_NEAR(q.angularDistance(rq), 0, TOL);
}

TEST(Rodrigues, AxisAngleConversion) {

	// Tests if we can perform a lossless conversion to and from AxisAngle,
	// using all constructors, operators and methods.

	// The evaluation here uses quaternions as there is no direct conversion to
	// axis angle.

	using namespace transforms;
	using namespace Eigen;

	auto axis = Vector3d(0.1, 0.4, 0.7).normalized();
	auto aa = (AngleAxisd(1.4, axis));

	// First check that both constructor and assignment operators have the same
	// behaviour.

	Rodriguesd r;
	r = aa;
	
	Rodrigues ar(aa);

	EXPECT_EQ(r.data[0], ar.data[0]);
	EXPECT_EQ(r.data[1], ar.data[1]);
	EXPECT_EQ(r.data[2], ar.data[2]);

	// Sanity check that we are actually writing something.
	EXPECT_GT(r.data[0], 0);

	// Then check different angle and axis combinations

	aa = (AngleAxisd(1.4, axis));
	r = aa;
	EXPECT_NEAR(Quaterniond(aa).angularDistance(r.toQuaternion()), 0, TOL);

	aa = (AngleAxisd(6.9, axis)); // > 2pi
	r = aa;
	EXPECT_NEAR(Quaterniond(aa).angularDistance(r.toQuaternion()), 0, TOL);

	aa = (AngleAxisd(-1.578, Vector3d::UnitY())); // < 0
	r = aa;
	EXPECT_NEAR(Quaterniond(aa).angularDistance(r.toQuaternion()), 0, TOL);
}

struct QuaternionCostFunctor {

	QuaternionCostFunctor(Eigen::Quaterniond q)
	{
		this->q = q;
	}

	Eigen::Quaterniond q;

	template<typename T>
	bool operator()(const T* const r, T* residuals) const
	{
		// This cost functor tests whether the toQuaternion() method can be used

		using namespace transforms;
		using namespace Eigen;

		Map<Vector3<T>> rr(residuals);
		rr = Rodrigues<T>(Rodrigues<T>::Map(r).toQuaternion().inverse() * q.cast<T>()).toVector();

		return true;
	}
};

TEST(Rodrigues, AutoDiff) {

	// Tests if Ceres can auto diff the type using Jets

	using namespace transforms;
	using namespace Eigen;
	using namespace ceres;

	auto q = Quaterniond(AngleAxisd(1.4, Vector3d(0.1, 0.4, 0.7).normalized()));
	Rodriguesd r;

	const ceres::CostFunction* cost_function =
		new AutoDiffCostFunction<
			QuaternionCostFunctor,
				3,
				3
		>(
			new QuaternionCostFunctor(q)
		);

	const std::vector<const ceres::Manifold*> manifolds {nullptr, nullptr};
	NumericDiffOptions options;

	GradientChecker gc(cost_function, &manifolds, options);

	GradientChecker::ProbeResults probeResults;

	double** parameters = new double* [] { r.data };

	if (!gc.Probe(parameters, 0.000001, &probeResults))
	{
		FAIL() << probeResults.error_log;
	}
}

TEST(Rodrigues, Composition) {

	// Tests if Ceres can auto diff the type using Jets

	using namespace transforms;
	using namespace Eigen;
	using namespace ceres;

	auto q1 = Quaterniond(AngleAxisd(1.47, Vector3d(0.1, 0.4, 0.7).normalized()));
	auto q2 = Quaterniond(AngleAxisd(6.8, Vector3d(0.8, 0.1, 0.7).normalized()));
	auto q3 = Quaterniond(AngleAxisd(-2.2, Vector3d(0.6, 0.1, 0.2).normalized()));
	auto q4 = Quaterniond(AngleAxisd(-9.2, Vector3d(0.26, 0.1, 0.2).normalized()));

	auto r1 = Rodrigues(q1);
	auto r2 = Rodrigues(q2);
	auto r3 = Rodrigues(q3);
	auto r4 = Rodrigues(q4);

	// todo: finish this...

	EXPECT_Q(
		q1.angularDistance(r1.toQuaternion())
	);
	
	EXPECT_Q(
		(q1 * q2).angularDistance((r1 * r2).toQuaternion())
	);

	EXPECT_Q(
		(q1 * q2 * q3).angularDistance((r1 * r2 * r3).toQuaternion())
	);

	EXPECT_Q(
		(q1 * q2 * q3 * q4).angularDistance((r1 * r2 * r3 * r4).toQuaternion())
	);

}

TEST(Rodrigues, Inverse) {

	// Tests if Ceres can auto diff the type using Jets

	using namespace transforms;
	using namespace Eigen;
	using namespace ceres;

	auto q1 = Quaterniond(AngleAxisd(1.47, Vector3d(0.1, 0.4, 0.7).normalized()));
	auto q2 = Quaterniond(AngleAxisd(6.8, Vector3d(0.8, 0.1, 0.7).normalized()));
	auto q3 = Quaterniond(AngleAxisd(-2.2, Vector3d(0.6, 0.1, 0.2).normalized()));
	auto q4 = Quaterniond(AngleAxisd(-9.2, Vector3d(0.26, 0.1, 0.2).normalized()));

	auto r1 = Rodrigues(q1);
	auto r2 = Rodrigues(q2);
	auto r3 = Rodrigues(q3);
	auto r4 = Rodrigues(q4);

	EXPECT_Q(q1.angularDistance(r1.toQuaternion()));

	EXPECT_Q(
		(q1 * q2.inverse()).angularDistance((r1 * r2.inverse()).toQuaternion())
	);

	EXPECT_Q(
		(q1.inverse() * q2 * q3).angularDistance((r1.inverse() * r2 * r3).toQuaternion())
	);

	EXPECT_Q(
		(q1 * q2.inverse() * q3 * q4.inverse()).angularDistance((r1 * r2.inverse() * r3 * r4.inverse()).toQuaternion())
	);

}