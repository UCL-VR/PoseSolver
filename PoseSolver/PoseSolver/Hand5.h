#pragma once

#include "Transforms.h"
#include "DenavitHartenberg.h"
#include "PointMeasurement.h"

namespace hand5 {

	using namespace transforms;

	/// <summary>
	/// Hand5 is one of the PoseSolver's unified Hand Models. These models have a
	/// fixed structure that model the transforms of a hand with a single parameter block
	/// from a Transform.
	/// Hand solves the hand using the typical DH Joints, similar to Hand2, but the
	/// fingertips are not represented by parameter blocks. Instead Hand5 has dedicated
	/// cost functions for fingertip observations.
	/// </summary>
	class Hand
	{
	public:
		struct JointParams
		{
			double d;
			double theta;
			double r;
			double a;
			double min;
			double max;
		};

		struct ChainParams
		{
			JointParams joints[6];
		};

		struct HandParams
		{
			ChainParams chains[5];
		};

		Hand(HandParams p, Transformd* start) :params(p),start(start)
		{
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 6; j++)
				{
					getParameterBlock((Finger)i)[j] = p.chains[i].joints[j].theta;
					getUpperBounds((Finger)i)[j] = p.chains[i].joints[j].max;
					getLowerBounds((Finger)i)[j] = p.chains[i].joints[j].min;
				}
			}
		}

		enum Finger {
			Thumb = 0,
			Index = 1,
			Middle = 2,
			Ring = 3,
			Little = 4
		};

		Transformd* start;

		// There are six angles per finger, and five fingers. These along with the wrist
		// make up the parameter blocks for this hand.
		double theta[30];

		// The ranges for each pose angle
		double lowerBounds[30];
		double upperBounds[30];

		// The parameters returned here are pointers to the fingers theta
		// parameter chain.
		// Each variable dh joint will be placed sequentially.

		double* getParameterBlock(Finger f) {
			return &theta[f * 6];
		}

		double* getUpperBounds(Finger f) {
			return &upperBounds[f * 6];
		}

		double* getLowerBounds(Finger f) {
			return &lowerBounds[f * 6];
		}

		const ChainParams& getChain(Finger f){
			return params.chains[(int)f];
		}

		template<typename T>
		Transform<T> getFingerPose(const Transform<T>& wrist, const T* const parameters, const ChainParams& c) const
		{
			auto chain = c.joints;

			transforms::Transform<T> p; // Identity
			for (size_t i = 0; i < 6; i++)
			{
				p = p *
					transforms::Transform<T>(Eigen::Vector3<T>((T)0, (T)chain[i].d, (T)0)) *
					transforms::Transform<T>(Eigen::AngleAxis<T>(parameters[i], Eigen::Vector3<T>::UnitY())) *
					transforms::Transform<T>(Eigen::Vector3<T>((T)0, (T)0, (T)chain[i].r)) *
					transforms::Transform<T>(Eigen::AngleAxis<T>((T)chain[i].a, Eigen::Vector3<T>::UnitZ()));
			}
			return wrist * p;
		}

		Transformd getEndPose(Finger finger)
		{
			return getFingerPose(*start, getParameterBlock(finger), getChain(finger));
		}

	private:
		HandParams params;

	public:
		struct JointLimits {
			Hand* hand;
			JointLimits(Hand* hand) : hand(hand) {}

			struct CostFunctor {
				Hand* hand;
				double scale = 10.0;

				CostFunctor(Hand* hand) : hand(hand) {}

				template<typename T>
				T evaluateJointLimit(const T* const angle, double upper, double lower) const {

					// It is not usually OK to include conditions in such evaluations
					// because they introduce discontinuities in the Jet's derivative
					// computations. However, in this case the lower bounds and upper
					// bounds are constant, so this is effectively the same as using
					// two different functions at compile time.

					if (upper == lower) {
						auto min = (T)lower;
						return *angle - min;
					}
					else {
						auto max = (T)upper * scale;
						auto min = (T)lower * scale;
						auto x = *angle * scale;
						auto y_max = 1.0 / (1.0 + exp(-(x - max - 6.0)));
						auto y_min = 1.0 / (1.0 + exp(-(min - x - 6.0)));
						return (y_min + y_max);
					}
				}

				template<typename T>
				void evaluateJointLimits(const T* const finger, const double* upper, const double* lower, T* residuals) const {
					for (size_t i = 0; i < 6; i++) {
						residuals[i] = evaluateJointLimit(&finger[i], upper[i], lower[i]);
					}
				}

				enum {
					Residuals = 30,
					Dimension1 = 6, // Five fingers with six angles each
					Dimension2 = 6,
					Dimension3 = 6,
					Dimension4 = 6,
					Dimension5 = 6,
				};

			public:
				template<typename T>
				bool operator()(
					const T* const finger1,
					const T* const finger2,
					const T* const finger3,
					const T* const finger4,
					const T* const finger5,
					T* residuals) const
				{
					evaluateJointLimits<T>(finger1, hand->getUpperBounds((Finger)0), hand->getLowerBounds((Finger)0), &residuals[0]);
					evaluateJointLimits<T>(finger2, hand->getUpperBounds((Finger)1), hand->getLowerBounds((Finger)1), &residuals[6]);
					evaluateJointLimits<T>(finger3, hand->getUpperBounds((Finger)2), hand->getLowerBounds((Finger)2), &residuals[12]);
					evaluateJointLimits<T>(finger4, hand->getUpperBounds((Finger)3), hand->getLowerBounds((Finger)3), &residuals[18]);
					evaluateJointLimits<T>(finger5, hand->getUpperBounds((Finger)4), hand->getLowerBounds((Finger)4), &residuals[24]);
					return true;
				}
			};

			ceres::CostFunction* costFunction() {
				return new ceres::AutoDiffCostFunction<
					CostFunctor,
					CostFunctor::Residuals,
					CostFunctor::Dimension1,
					CostFunctor::Dimension2,
					CostFunctor::Dimension3,
					CostFunctor::Dimension4,
					CostFunctor::Dimension5>(
						new CostFunctor(this->hand)
					);
			}

			std::vector<double*> parameterBlocks() {
				return std::vector<double*>({
						hand->getParameterBlock((Finger)0),
						hand->getParameterBlock((Finger)1),
						hand->getParameterBlock((Finger)2),
						hand->getParameterBlock((Finger)3),
						hand->getParameterBlock((Finger)4),
					});
			}
		};

		class PointMeasurement : public observations::PointMeasurementBase {
		private:
			Hand* hand;
			Finger finger;

		public:
			PointMeasurement(Hand* hand, Finger finger) : hand(hand), finger(finger)
			{
				point.setZero();
				offset.setZero();
				residualBlockId = nullptr;
			}
			
			template<typename T>
			bool operator()(
				const T* const wrist,
				const T* const finger,
				const T* const point,
				const T* const offset,
				T* residuals) const
			{
				Eigen::Map<Eigen::Vector3<T>> r(residuals);
				r = (hand->getFingerPose(Transform<T>::Map(wrist), finger, hand->getChain(this->finger)) * Eigen::Vector3<T>(offset)) - Eigen::Vector3<T>(point);
				return true;
			}

			enum {
				Residuals = 3, // Difference between estimated point and actual point 
				Dimension1 = Transformd::Dimension, // The pose of the wrist
				Dimension2 = 6, // The angles of the finger
				Dimension3 = 3, // Observed Point
				Dimension4 = 3 // Observed Offset
			};

		public:
			ceres::CostFunction* costFunction() {
				return new ceres::AutoDiffCostFunction<
					PointMeasurement,
					Residuals,
					Dimension1,
					Dimension2,
					Dimension3,
					Dimension4
				>(
					this,
					ceres::Ownership::DO_NOT_TAKE_OWNERSHIP
				);
			}

			std::vector<double*> parameterBlocks() {
				return {
					hand->start->parameterBlock(),
					hand->getParameterBlock(finger),
					point.data(),
					offset.data()
				};
			}
		};
	};
}