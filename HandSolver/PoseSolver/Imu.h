#pragma once

#include "Pose.h"

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/problem.h>

namespace hs {

	// Based on:
	// Lupton, Todd, and Salah Sukkarieh. ‘Efficient Integration of Inertial 
	// Observations into Visual SLAM without Initialization’. 
	// 2009 IEEE / RSJ International Conference on Intelligent Robots and 
	// Systems, October 2009, 1547–52.
	// https://doi.org/10.1109/IROS.2009.5354267.

	/// <summary>
	/// Implements a pre-integration cost between two frames at t0 and t1, based on
	/// imu measurements and a calibration pose.
	/// </summary>
	class PreIntegrationFactor
	{
	public:
		// The inertial frame at the beginning of the integration period in world
		// space.

		hs::Pose3d* start;

		// The inertial frame at the end of the integration period in world 
		// space.

		hs::Pose3d* end;

		// The estimated velocity of the inertial frame, in world space, at the
		// start.

		Eigen::Vector3d* velocity;

		// A set of properties describing the total change in state over the
		// integration period.

		Eigen::Vector3d deltaPosition;
		Eigen::Vector3d deltaVelocity;
		hs::Rodriguesd deltaRotation;

		Eigen::Vector3d deltaGravityPosition;
		Eigen::Vector3d deltaGravityVelocity;

		// The elapsed time across the delta parameters above. This is the variable
		// used in the Cost evaluation, though some convenience parameters are also
		// defined.

		double time;

		PreIntegrationFactor(Pose3d* begin, Pose3d* end) {
			this->start = begin;
			this->end = end;
			velocity = new Eigen::Vector3d(0, 0, 0);
			deltaPosition.setZero();
			deltaVelocity.setZero();
			deltaRotation.setIdentity();
			deltaGravityPosition.setZero();
			deltaGravityVelocity.setZero();
			time = 0;
		}

		void addSample(Eigen::Vector3d a, Eigen::Vector3d d, double dt) {	
			deltaVelocity = deltaVelocity + (deltaRotation * a * dt);
			deltaPosition = deltaPosition + deltaVelocity * dt;
			deltaRotation = deltaRotation * (Eigen::AngleAxisd(d.x() * dt, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(d.y() * dt, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(d.z() * dt, Eigen::Vector3d::UnitZ()));

			// Accumulates what the gravity would be in the world frame

			deltaGravityVelocity = deltaGravityVelocity + Eigen::Vector3d(0, -9.8, 0) * dt;
			deltaGravityPosition = deltaGravityPosition + deltaGravityVelocity * dt;

			time += dt;
		}

		// Implements a Cost Function where the local offsets are maintained as
		// parameter blocks.
		// The residual is end (i.e. a Pose3d)

		struct CostFunctor1 {

			PreIntegrationFactor* factor;

			CostFunctor1(PreIntegrationFactor* factor)
			{
				this->factor = factor;
			}

			template<typename T>
			bool operator()(
				const T* const before,
				const T* const after,
				const T* const velocity,
				T* residuals) const {

				using namespace hs;
				using namespace Eigen;

				// The starting position 
				auto start = Pose3<T>::Map(before);

				// Apply the transforms to the body
				Pose3<T> end = start;

				// The change in inertial position, transformed into the
				// navigation frame based on the initial rotation.
				end.Position() -= start.Rotation() * factor->deltaPosition.cast<T>();

				// The expected change due to the existing velocity that won't
				// be measured in the inertial frame.
				end.Position() += Vector3<T>::Map(velocity) * factor->time;

				// Removal of the effects due to gravity over the inertial frame
				end.Position() -= -factor->deltaGravityPosition.cast<T>();

				// The change in rotation measured by the imu. 
				end.Rotation() = end.Rotation() * factor->deltaRotation.Cast<T>();

				// These updated parameters are compared with the estimate of
				// the pose at end to get the residuals.
				Pose3<T>::Between(end, Pose3<T>::Map(after), residuals);

				return true;
			}

			enum {
				Dimension1 = hs::Pose3d::Dimension,
				Dimension2 = hs::Pose3d::Dimension,
				Dimension3 = 3,
				Residuals = hs::Pose3d::Residuals
			};
		};

		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor1,
				CostFunctor1::Residuals,
				CostFunctor1::Dimension1,
				CostFunctor1::Dimension2,
				CostFunctor1::Dimension3>(
					new CostFunctor1(this)
				);
		};

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ 
				start->parameterBlock(), 
				end->parameterBlock(), 
				(double*)velocity
			});
		}

		void addToProblem(ceres::Problem& problem) {
			problem.AddResidualBlock(
				costFunction(),
				nullptr,
				parameterBlocks()
			);
		}
	};

	/// <summary>
	/// Convenience class collecting parameter blocks describing the calibration
	/// of a single IMU.
	/// </summary>
#pragma pack(push, 1)
	struct ImuBiasParameters
	{
		Eigen::Vector3d accelerometerBias;
		Eigen::Vector3d gyroscopeBias;

		enum {
			Size = 6
		};
	};
#pragma pack(pop)

	/// <summary>
	/// A factor to estimate the accelerometer and gyroscope bias based on the
	/// observed motion of an Imu. One of these factors should exist for each
	/// frame the Imu can be fully observed.
	/// </summary>
	class ImuBiasFactor
	{
	public:
		/// <summary>
		/// A snapshot of the strapdown frame at this timestamp. This includes
		/// the current position, as well as the linear and angular velocity
		/// and accelerations.
		/// </summary>
		MotionFrame reference;

		/// <summary>
		/// The measured acceleration of the IMU at this timestamp in m/s^2.
		/// </summary>
		Eigen::Vector3d acceleration;

		/// <summary>
		/// The measured rotation of the IMU at this timestamp in radians per second.
		/// </summary>
		Eigen::Vector3d rotation;

		/// <summary>
		/// The bias in the accelerometer. The factor will estimate this.
		/// </summary>
		Eigen::Vector3d* accelerometerBias;

		/// <summary>
		/// The bias in the gyroscope. The factor will estimate this.
		/// </summary>
		Eigen::Vector3d* gyroscopeBias;

		ceres::ResidualBlockId id;

		ImuBiasFactor(ceres::Problem& problem, ImuBiasParameters* calibration, MotionFrame reference, Eigen::Vector3d a, Eigen::Vector3d d)
		{
			this->accelerometerBias = &calibration->accelerometerBias;
			this->gyroscopeBias = &calibration->gyroscopeBias;
			this->acceleration = a;
			this->rotation = d;
			addToProblem(problem);
		}

		struct CostFunctor
		{
			ImuBiasFactor* factor;

			CostFunctor(ImuBiasFactor* factor)
			{
				this->factor = factor;
			}

			template<typename T>
			bool operator()(
				const T* const accelerometerBias,
				const T* const gyroscopeBias,
				T* residuals) const {

				using namespace hs;
				using namespace Eigen;

				// This cost functor optimises for the biases of the Imu

				// The estimated pose
				auto s = factor->reference.pose.Cast<T>();

				// Estimated gravity in inertial frame
				auto g = s.Rotation().inverse() * Vector3d(0, -9.8, 0).cast<T>();

				// Reference acceleration in inertial frame
				auto a = s.Rotation().inverse() * factor->reference.acceleration.cast<T>();

				// Reference angular velocity in inertial frame
				auto d = s.Rotation().inverse() * factor->reference.angularVelocity.Cast<T>();

				// The estimated biases
				auto ba = Vector3<T>::Map(accelerometerBias);
				auto bg = Vector3<T>::Map(gyroscopeBias);

				// The difference between estimated and measured acceleration
				Vector3<T>::Map(&residuals[0]) = factor->acceleration - (g + a + ba);

				// The difference between the estimated and measured rotations,
				// note that this works by composing the measured euler angles
				// into a quaternion, as opposed to trying to decompose the
				// quaternion, as this has much less potential for ambiguities
				// and numerical errors.

				// Measured angular velocity minus bias (Estimated true angular velocity)
				auto inertialAngles = (factor->rotation.cast<T>() - bg);

				// Measured angular velocity as quaternion
				auto localRotation = hs::Rodrigues<T>(
					Eigen::AngleAxis<T>(inertialAngles.x(), Vector3d::UnitX().cast<T>()) *
					Eigen::AngleAxis<T>(inertialAngles.y(), Vector3d::UnitY().cast<T>()) *
					Eigen::AngleAxis<T>(inertialAngles.z(), Vector3d::UnitZ().cast<T>())
					);

				// The difference between the reference angular velocity, and reference angular velocity
				Vector3<T>::Map(&residuals[3]) = (localRotation.inverse() * d).toVector();

				return true;
			}

			enum {
				Dimension1 = 3,
				Dimension2 = 3,
				Residuals = 6
			};
		};

		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor,
				CostFunctor::Residuals,
				CostFunctor::Dimension1,
				CostFunctor::Dimension2>(
					new CostFunctor(this)
					);
		};

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({
				accelerometerBias->data(),
				gyroscopeBias->data()
				}
			);
		}

		void addToProblem(ceres::Problem& problem) {
			id = problem.AddResidualBlock(
				costFunction(),
				nullptr,
				parameterBlocks()
			);
		}
	};

	/// <summary>
	/// Convenience class collecting parameter blocks describing the calibration
	/// of a single IMU.
	/// </summary>
	struct ImuCalibrationParameters
	{
		hs::Pose3d local;
		Eigen::Vector3d accelerometerBias;
		Eigen::Vector3d gyroscopeBias;
	};

	/// <summary>
	/// A factor to estimate the bias and rotation between the IMU navigation
	/// frame and another (moving) frame in the world.
	/// The IMU is assumed to be coincident with the reference frame, and they
	/// are separated only by a rotation.
	/// Each instance of the factor captures a snapshot in time - ideally there
	/// would be a factor for each IMU reading.
	/// </summary>
	class ImuCalibrationFactor
	{
	public:
		/// <summary>
		/// The transform from the strapdown frame to the inertial frame. The
		/// factor will estimate this.
		/// </summary>
		hs::Pose3d* local;

		/// <summary>
		/// A snapshot of the strapdown frame at this timestamp. This includes
		/// the current position, as well as the linear and angular velocity
		/// and accelerations.
		/// </summary>
		MotionFrame reference;

		/// <summary>
		/// The measured acceleration of the IMU at this timestamp in m/s^2.
		/// </summary>
		Eigen::Vector3d acceleration;

		/// <summary>
		/// The measured rotation of the IMU at this timestamp in radians per second.
		/// </summary>
		Eigen::Vector3d rotation;

		/// <summary>
		/// The bias in the accelerometer. The factor will estimate this.
		/// </summary>
		Eigen::Vector3d* accelerometerBias;

		/// <summary>
		/// The bias in the gyroscope. The factor will estimate this.
		/// </summary>
		Eigen::Vector3d* gyroscopeBias;

		ImuCalibrationFactor(ceres::Problem& problem, ImuCalibrationParameters* calibration, MotionFrame reference, Eigen::Vector3d a, Eigen::Vector3d d)
		{
			this->local = &calibration->local;
			this->accelerometerBias = &calibration->accelerometerBias;
			this->gyroscopeBias = &calibration->gyroscopeBias;
			this->acceleration = a;
			this->rotation = d;
			addToProblem(problem);
		}

		struct CostFunctor 
		{
			ImuCalibrationFactor* factor;

			CostFunctor(ImuCalibrationFactor* factor)
			{
				this->factor = factor;
			}

			template<typename T>
			bool operator()(
				const T* const local,
				const T* const accelerometerBias,
				const T* const gyroscopeBias,
				T* residuals) const {

				using namespace hs;
				using namespace Eigen;

				// This cost functor optimises for the bias and orientation of
				// the IMU at the sampled timestamps.

				// The estimated pose (the local offset composed with the known strapdown frame)
				auto s = factor->reference.pose.Cast<T>() * Pose3<T>::Map(local);

				// Estimated gravity in inertial frame
				auto g = s.Rotation().inverse() * Vector3d(0, -9.8, 0).cast<T>();

				// Reference acceleration in inertial frame
				auto a = s.Rotation().inverse() * factor->reference.acceleration.cast<T>();

				// Reference angular velocity in inertial frame
				auto d = s.Rotation().inverse() * factor->reference.angularVelocity.Cast<T>();

				// The estimated biases
				auto ba = Vector3<T>::Map(accelerometerBias);
				auto bg = Vector3<T>::Map(gyroscopeBias);

				// The difference between estimated and measured acceleration
				Vector3<T>::Map(&residuals[0]) = factor->acceleration - (g + a + ba);

				// The difference between the estimated and measured rotations,
				// note that this works by composing the measured euler angles
				// into a quaternion, as opposed to trying to decompose the
				// quaternion, as this has much less potential for ambiguities
				// and numerical errors.
				
				// Measured angular velocity minus bias (Estimated true angular velocity)
				auto inertialAngles = (factor->rotation.cast<T>() - bg);

				// Measured angular velocity as quaternion
				auto localRotation = hs::Rodrigues<T>(
					Eigen::AngleAxis<T>(inertialAngles.x(), Vector3d::UnitX().cast<T>()) *
					Eigen::AngleAxis<T>(inertialAngles.y(), Vector3d::UnitY().cast<T>()) *
					Eigen::AngleAxis<T>(inertialAngles.z(), Vector3d::UnitZ().cast<T>())
				); 

				// The difference between the reference angular velocity, and reference angular velocity
				Vector3<T>::Map(&residuals[3]) = (localRotation.inverse() * d).toVector();

				// The position parameters should match the reference frame exactly
				Vector3<T>::Map(&residuals[6]) = factor->reference.pose.Cast<T>().Position() - s.Position();

				return true;
			}

			enum {
				Dimension1 = hs::Pose3d::Dimension,
				Dimension2 = 3,
				Dimension3 = 3,
				Residuals = 10
			};
		};

		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor,
				CostFunctor::Residuals,
				CostFunctor::Dimension1,
				CostFunctor::Dimension2,
				CostFunctor::Dimension3>(
					new CostFunctor(this)
				);
		};

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({
				local->parameterBlock(),
				accelerometerBias->data(),
				gyroscopeBias->data()
				}
			);
		}

		void addToProblem(ceres::Problem& problem) {
			problem.AddResidualBlock(
				costFunction(),
				nullptr,
				parameterBlocks()
			);
		}

	};

	/// <summary>
	/// A factor to estimate the rotation between the IMU navigation frame and
	/// another (moving) frame in the world.
	/// The IMU is assumed to be coincident with the reference frame, and they
	/// are separated only by a rotation.
	/// </summary>
	class ImuOrientationFactor
	{
	public:
		/// <summary>
		/// The center of rotation of the IMU in world space.
		/// </summary>
		hs::Pose3d* reference;

		/// <summary>
		/// The absolute transform of the IMU estimated in world space. This
		/// is the equivalent of the start pose in the pre-integration factor.
		/// The local transform is given by the difference between reference
		/// and this.
		/// </summary>
		hs::Pose3d* start;

		Eigen::Vector3d acceleration;

		ImuOrientationFactor(hs::Pose3d* reference, hs::Pose3d* start)
		{
			this->reference = reference;
			this->start = start;
		}

		void addSample(Eigen::Vector3d a, Eigen::Vector3d d, double dt) {
			acceleration = a;
		}

		// Implements a Cost Function where the local offsets are maintained as
		// parameter blocks.
		// The residual is end (i.e. a Pose3d)

		struct CostFunctor1 {

			ImuOrientationFactor* factor;

			CostFunctor1(ImuOrientationFactor* factor)
			{
				this->factor = factor;
			}

			template<typename T>
			bool operator()(
				const T* const reference,
				const T* const start,
				T* residuals) const {

				using namespace hs;
				using namespace Eigen;

				// The estimated pose`
				auto s = Pose3<T>::Map(start);

				// Estimate gravity
				auto g = s.Rotation().inverse() * Vector3d(0, -9.8, 0).cast<T>();

				// Estimate acceleration
				auto a = Vector3<T>((T)0, (T)0, (T)0);

				// The residuals are the difference between estimated and measured acceleration
				Vector3<T>::Map(&residuals[0]) = factor->acceleration - (g + a);

				// The position parameters should match the reference frame exactly
				Vector3<T>::Map(&residuals[3]) = Pose3<T>::Map(reference).Position() - s.Position();

				return true;
			}

			enum {
				Dimension1 = hs::Pose3d::Dimension,
				Dimension2 = hs::Pose3d::Dimension,
				Residuals = hs::Pose3d::Residuals
			};
		};

		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor1,
				CostFunctor1::Residuals,
				CostFunctor1::Dimension1,
				CostFunctor1::Dimension2>(
					new CostFunctor1(this)
				);
		};

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({
				reference->parameterBlock(),
				start->parameterBlock()
				}
			);
		}

		void addToProblem(ceres::Problem& problem) {
			problem.AddResidualBlock(
				costFunction(),
				nullptr,
				parameterBlocks()
			);
		}

	};
}