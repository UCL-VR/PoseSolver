#pragma once

#include "Transforms.h"
#include "DenavitHartenberg.h"

namespace hand2 {

	using namespace transforms;

	/// <summary>
	/// Hand2 is one of the PoseSolver's unified Hand Models. These models have a
	/// fixed structure that model the transforms of a hand with a single parameter block
	/// from a Transform.
	/// Hand solves the hand using the typical DH Joints, but where all joints are
	/// baked into a single cost function.
	/// </summary>
	class Hand
	{
		// All finger joint chains are independent, so in theory each chain could be
		// solved as a separate cost function.
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
					getParams((Finger)i)[j] = p.chains[i].joints[j].theta;
					getUpperBounds((Finger)i)[j] = p.chains[i].joints[j].max;
					getLowerBounds((Finger)i)[j] = p.chains[i].joints[j].min;
				}

				ends[i] = new Transformd();
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
		Transformd* ends[5];

		// There are six thetas per five fingers (for now, we will add fixed thetas later...)
		double theta[30];

		double lowerBounds[30];
		double upperBounds[30];

		Transformd* getEndPose(Finger finger)
		{
			return ends[(int)finger];
		}

		// These methods below should match

		// The parameters returned here are pointers to the fingers theta
		// parameter chain.
		// Each variable dh joint will be placed sequentially.

		double* getParams(Finger f) {
			return &theta[f * 6];
		}

		double* getUpperBounds(Finger f) {
			return &upperBounds[f * 6];
		}

		double* getLowerBounds(Finger f) {
			return &lowerBounds[f * 6];
		}

		template<typename T>
		const T* getParams(const T* const p, Finger f) const {
			return &p[f * 6]; // six joints per finger
		}

		template<typename T>
		Transform<T> getFingerPose(const T* const handParameterBlock, Finger finger) const
		{
			auto chain = params.chains[finger].joints;
			auto chainAngles = getParams(handParameterBlock, finger);

			transforms::Transform<T> p; // Identity

			for (size_t i = 0; i < 6; i++)
			{
				p = p *
					transforms::Transform<T>(Eigen::Vector3<T>((T)0, (T)chain[i].d, (T)0)) *
					transforms::Transform<T>(Eigen::AngleAxis<T>(chainAngles[i], Eigen::Vector3<T>::UnitY())) *
					transforms::Transform<T>(Eigen::Vector3<T>((T)0, (T)0, (T)chain[i].r)) *
					transforms::Transform<T>(Eigen::AngleAxis<T>((T)chain[i].a, Eigen::Vector3<T>::UnitZ()));
			}
			return p;
		}

	private:
		HandParams params;

		struct CostFunctor {
			Hand* hand;
			double scale = 10.0;

			CostFunctor(Hand* hand) :hand(hand) {}

			template<typename T>
			T* getPoseResiduals(T* residuals, Finger f) const {
				return &residuals[f * transforms::Transformd::Residuals];
			}

			template<typename T>
			T* getLimitsResiduals(T* residuals) const {
				return &residuals[(5 * transforms::Transformd::Residuals)];
			}

			template<typename T>
			T evaluateJointLimit(const T* const theta, int i) const {

				// It is not usually OK to include conditions in such evaluations
				// because they introduce discontinuities in the Jet's derivative
				// computations. However, in this case the lower bounds and upper
				// bounds are constant, so this is effectively the same as using
				// two different functions at compile time.

				if (hand->upperBounds[i] == hand->lowerBounds[i]) {
					auto min = (T)hand->lowerBounds[i];
					return theta[i] - min;
				}
				else{
					auto max = (T)hand->upperBounds[i] * scale;
					auto min = (T)hand->lowerBounds[i] * scale;
					auto x = theta[i] * scale;
					auto y_max = 1.0 / (1.0 + exp(-(x - max - 6.0)));
					auto y_min = 1.0 / (1.0 + exp(-(min - x - 6.0)));
					return (y_min + y_max);
				}
			}
			
			template<typename T>
			bool operator()(const T* const start,
				const T* const thumb,
				const T* const index,
				const T* const middle,
				const T* const ring,
				const T* const little,
				const T* const theta,
				T* residuals) const {

				Transform<T>::Between(Transform<T>::Map(start) * hand->getFingerPose(theta, Thumb),  Transform<T>::Map(thumb), getPoseResiduals(residuals, Thumb));
				Transform<T>::Between(Transform<T>::Map(start) * hand->getFingerPose(theta, Index),  Transform<T>::Map(index), getPoseResiduals(residuals, Index));
				Transform<T>::Between(Transform<T>::Map(start) * hand->getFingerPose(theta, Middle), Transform<T>::Map(middle), getPoseResiduals(residuals, Middle));
				Transform<T>::Between(Transform<T>::Map(start) * hand->getFingerPose(theta, Ring),   Transform<T>::Map(ring), getPoseResiduals(residuals, Ring));
				Transform<T>::Between(Transform<T>::Map(start) * hand->getFingerPose(theta, Little), Transform<T>::Map(little), getPoseResiduals(residuals, Little));

				auto LR = getLimitsResiduals(residuals);
				for (size_t i = 0; i < 30; i++){
					LR[i] = evaluateJointLimit(theta, (int)i);
				}

				return true;
			}

			enum {
				Residuals = (Transformd::Residuals * 5) + 30, // Five poses plus 30 joint limits
				Dimension1 = Transformd::Dimension, // Starting transforms
				Dimension2 = Transformd::Dimension, // Five finger end poses
				Dimension3 = Transformd::Dimension,
				Dimension4 = Transformd::Dimension,
				Dimension5 = Transformd::Dimension,
				Dimension6 = Transformd::Dimension,
				Dimension7 = 30,					// The angles
			};
		};

	public:
		ceres::CostFunction* costFunction() {
			return new ceres::AutoDiffCostFunction<
				CostFunctor,
				CostFunctor::Residuals,
				CostFunctor::Dimension1,
				CostFunctor::Dimension2,
				CostFunctor::Dimension3,
				CostFunctor::Dimension4,
				CostFunctor::Dimension5,
				CostFunctor::Dimension6,
				CostFunctor::Dimension7>(
					new CostFunctor(this)
				);
		}

		double* parameterBlock() {
			return theta;
		}

		std::vector<double*> parameterBlocks() {
			return std::vector<double*>({ start->parameterBlock(), 
				getEndPose(Thumb)->parameterBlock(),
				getEndPose(Index)->parameterBlock(),
				getEndPose(Middle)->parameterBlock(),
				getEndPose(Ring)->parameterBlock(),
				getEndPose(Little)->parameterBlock(),
				parameterBlock() 
			});
		}
	};
}