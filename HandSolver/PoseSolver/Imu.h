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
		// The body frame at t0. This should be constrainted with another
		// measurment or set to const.

		hs::Pose3d* bodyStart;

		// The body frame at t1. This is what will be estimated based on the
		// IMU measurements added to the factor.

		hs::Pose3d* bodyEnd;

		// The estimated velocity of the body at bodyStart. This must be set for
		// the estimation to work. It can either be set externally or maintained
		// here.

		Eigen::Vector3d* bodyVelocity;

		// The local transform from the body frames to the inertial frame.
		// This must be calibrated before the factor can be used to estimate motion
		// based on Imu data.
		// The parameter blocks for these are stored in this class.

		hs::Rodriguesd localRotation;
		Eigen::Vector3d localPosition;

		Eigen::Vector3d deltaPosition;
		Eigen::Vector3d deltaVelocity;
		hs::Rodriguesd deltaRotation;

		Eigen::Vector3d deltaGravity;

		// The elapsed time across the delta parameters above. This is the variable
		// used in the Cost evaluation, though some convenience parameters are also
		// defined.

		double time;

		PreIntegrationFactor(Pose3d* begin, Pose3d* end) {
			bodyStart = begin;
			bodyEnd = end;
			bodyVelocity = new Eigen::Vector3d(0, 0, 0);
			localRotation.setIdentity();
			localPosition.setZero();
			deltaPosition.setZero();
			deltaVelocity.setZero();
			deltaRotation.setIdentity();
			deltaGravity.setZero();
			time = 0;
		}

		void addSample(Eigen::Vector3d a, Eigen::Vector3d d, double dt) {	
			deltaVelocity = deltaVelocity + (deltaRotation * a * dt);
			deltaPosition = deltaPosition + deltaVelocity * dt;
			deltaRotation = deltaRotation * (Eigen::AngleAxisd(d.x() * dt, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(d.y() * dt, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(d.z() * dt, Eigen::Vector3d::UnitZ()));

			// Accumulates what the gravity would be in the world frame

			deltaGravity = deltaGravity + Eigen::Vector3d(0, -9.8, 0) * dt * dt;

			time += dt;
		}

		// Implements a Cost Function where the local offsets are maintained as
		// parameter blocks.
		// The residual is bodyEnd (i.e. a Pose3d)

		struct CostFunctor1 {

			PreIntegrationFactor* factor;

			CostFunctor1(PreIntegrationFactor* factor)
			{
				this->factor = factor;
			}

			template<typename T>
			bool operator()(
				const T* const bodyStart,
				const T* const bodyEnd,
				const T* const bodyVelocity,
				const T* const localPosition,
				const T* const localRotation,
				T* residuals) const {

				using namespace hs;
				using namespace Eigen;

				// Set up the mappings to the parameter blocks

				// The starting position 
				auto start = Pose3<T>::Map(bodyStart);

				auto bodyFrameToInertialFrame = Pose3<T>(Vector3<T>::Map(localPosition), Rodrigues<T>::Map(localRotation));
				auto changeInInertialFrame = Pose3d(factor->deltaPosition, factor->deltaRotation).Cast<T>();

				// Move the transforms from the inertial frame into the body frame by undoing the local strapdown transform
				Pose3<T> inertialChangeInBodyFrame = bodyFrameToInertialFrame.inverse() * changeInInertialFrame;

				// Apply the transforms to the body
				Pose3<T> end = start;

				// The change in position measured by the imu (double integration)
				end.Position() += inertialChangeInBodyFrame.Position();

				// The expected change due to the existing velocity that won't be measured in the inertial frame
				end.Position() += Vector3<T>::Map(bodyVelocity) * factor->time;

				// Removal of the effects due to gravity over the inertial frame
				end.Position() += -factor->deltaGravity.cast<T>();

				// The change in rotation measured by the imu. 
				// This is not the composition of rotations, but the inertial
				// rotation with its basis transformed into the body frame.

				// Starting rotation in the inertial frame, add the delta, 
				// back into body frame...

				auto inertialStart = start.Rotation() * factor->localRotation.Cast<T>();
				auto inertialEndFrame = inertialStart * factor->deltaRotation.Cast<T>();
				auto bodyEndFrame = factor->localRotation.Cast<T>().inverse() * inertialStart;

				end.Rotation() = bodyEndFrame;

				// These updated parameters are compared with the estimate of the 
				// pose at end to get the residuals
				auto expected = Pose3<T>::Map(bodyEnd);

				Pose3<T>::Between(end, expected, residuals);

				return true;
			}

			enum {
				Dimension1 = hs::Pose3d::Dimension,
				Dimension2 = hs::Pose3d::Dimension,
				Dimension3 = 3,
				Dimension4 = 3,
				Dimension5 = 3,
				Residuals = hs::Pose3d::Residuals
			};
		};

		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor1,
				CostFunctor1::Residuals,
				CostFunctor1::Dimension1,
				CostFunctor1::Dimension2,
				CostFunctor1::Dimension3,
				CostFunctor1::Dimension4,
				CostFunctor1::Dimension5>(
					new CostFunctor1(this)
				);
		};

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ 
				bodyStart->parameterBlock(), 
				bodyEnd->parameterBlock(), 
				(double*)bodyVelocity,
				(double*)&localPosition,
				(double*)&localRotation
			});
		}

		double* localPositionParameter() {
			return (double*)& localPosition;
		}

		double* localRotationParameter() {
			return (double*)&localRotation;
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