#pragma once

#include "Transforms.h"

#include <vector>

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/problem.h>

using namespace transforms;

// Various common joints of differing degrees of freedom. All of these joints
// operate exclusively on the transforms namespace.

namespace joints
{
	template<typename T>
	T softRange(const T& theta, T min, T max)
	{
		auto s = (T)10.0;
		auto x = theta * s;
		auto y_max = 1.0 / (1.0 + exp(-(x - (T)max * s - 6.0)));
		auto y_min = 1.0 / (1.0 + exp(-((T)min * s - x - 6.0)));
		return (y_min + y_max);
	}

	/// <summary>
	/// Implements a hinge joint about the x-axis, with limits
	/// </summary>
	class Axis1D
	{
		// This is the angle around the x axis of the joint.
		double angle; 
		Transformd bind;
		Transformd* start;
		Transformd* end;
		Eigen::Vector2d range;

	public:
		Axis1D(transforms::Transformd* start, transforms::Transformd* end, Eigen::Vector2d range):
			start(start), end(end), range(range)
		{
			// The initial configuration defines the bind pose of the joint.
			// This joint will permit an additional rotation, after the bind
			// pose, around the x-axis.

			bind = start->inverse() * *end;

			angle = 0;
		}

		struct CostFunctor {
			
			Axis1D* j;

			CostFunctor(Axis1D* joint) : j(joint) {}

			template<typename T>
			bool operator()(const T* const start, const T* const end, const T* const theta, T* residuals) const {

				// An Axis1D joint connects two transforms, start and end.
				// Expressed as a cost, end should be equal to the starting pose,
				// perturbed by the fixed transform, perturbed by the parameterised
				// rotation around the axis.

				auto localRotation = Eigen::AngleAxis<T>(*theta, Eigen::Vector3<T>((T)1.0, (T)0.0, (T)0.0));

				transforms::Transform<T>::Between(
					transforms::Transform<T>::Map(start) 
					* j->bind.Cast<T>()
					* Transform(localRotation), 
					transforms::Transform<T>::Map(end), 
					residuals);
				residuals[6] = softRange<T>(*theta, (T)j->range.x(), (T)j->range.y());

				return true;
			}

			enum {
				Residuals = Transformd::Residuals + 1,
				Dimension1 = Transformd::Dimension,
				Dimension2 = Transformd::Dimension,
				Dimension3 = 1
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
		}

		double* parameterBlock() {
			return &angle;
		}

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ start->parameterBlock(), end->parameterBlock(), parameterBlock() });
		}
	};

	/// <summary>
	/// Implements a ball joint about the x & y axis, with limits. The rotation
	/// in the y-axis is applied first.
	/// </summary>
	class Axis2D
	{
		// This is the angle around the x axis of the joint.
		double angles[2];
		Transformd bind;
		Transformd* start;
		Transformd* end;
		Eigen::Vector2d range_x;
		Eigen::Vector2d range_y;

	public:
		Axis2D(transforms::Transformd* start, transforms::Transformd* end, Eigen::Vector2d range_x, Eigen::Vector2d range_y) :
			start(start), end(end), range_x(range_x), range_y(range_y)
		{
			// The initial configuration defines the bind pose of the joint.
			// This joint will permit an additional rotation, after the bind
			// pose, around the x-axis.

			bind = start->inverse() * *end;

			angles[0] = 0;
			angles[1] = 0;
		}

		struct CostFunctor {

			Axis2D* j;

			CostFunctor(Axis2D* joint) : j(joint) {}

			template<typename T>
			bool operator()(const T* const start, const T* const end, const T* const angles, T* residuals) const {

				// An Axis1D joint connects two transforms, start and end.
				// Expressed as a cost, end should be equal to the starting pose,
				// perturbed by the fixed transform, perturbed by the parameterised
				// rotation around the axis.

				auto localRotationX = Eigen::AngleAxis<T>(angles[0], Eigen::Vector3<T>((T)1.0, (T)0.0, (T)0.0));
				auto localRotationY = Eigen::AngleAxis<T>(angles[1], Eigen::Vector3<T>((T)0.0, (T)1.0, (T)0.0));

				transforms::Transform<T>::Between(
					transforms::Transform<T>::Map(start)
					* j->bind.Cast<T>()
					* Transform(localRotationY)
					* Transform(localRotationX),
					transforms::Transform<T>::Map(end),
					residuals);
				residuals[6] = softRange<T>(angles[0], (T)j->range_x.x(), (T)j->range_x.y());
				residuals[7] = softRange<T>(angles[1], (T)j->range_y.x(), (T)j->range_y.y());

				return true;
			}

			enum {
				Residuals = Transformd::Residuals + 2,
				Dimension1 = Transformd::Dimension,
				Dimension2 = Transformd::Dimension,
				Dimension3 = 2
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
		}

		double* parameterBlock() {
			return angles;
		}

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ start->parameterBlock(), end->parameterBlock(), parameterBlock() });
		}
	};

	class Rigid 
	{
		Transformd* start;
		Transformd* end;
		Transformd bind;

	public:
		Rigid(Transformd* start, Transformd* end):start(start),end(end)
		{
			bind = start->inverse() * *end;
		}

		struct CostFunctor{

			Rigid* j;

			CostFunctor(Rigid* j):j(j) { }

			template <typename T>
			bool operator()(const T* const a, const T* const b, T* residual) const {

				auto posea = transforms::Transform<T>::Map(a);
				auto poseb = transforms::Transform<T>::Map(b);
				transforms::Transform<T>::Between(posea * j->bind.Cast<T>(), poseb, residual);
				return true;
			}

			enum {
				Residuals = Transformd::Residuals,
				Dimension1 = Transformd::Dimension,
				Dimension2 = Transformd::Dimension,
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
		}

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ start->parameterBlock(), end->parameterBlock()});
		}
	};
}