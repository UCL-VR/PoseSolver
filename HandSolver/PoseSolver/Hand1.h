#pragma once

#include "DenavitHartenberg.h"

namespace hs {

	/// <summary>
	/// Hand1 is one of the PoseSolver's unified Hand Models. These models have a
	/// fixed structure that model the pose of a hand with a single parameter block
	/// from a Pose3.
	/// Hand1 solves the hand using the typical DH Joints, but where all joints are
	/// baked into a single cost function.
	/// </summary>
	class Hand1
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
		};

		struct ChainParams
		{
			JointParams joints[6];
		};

		struct HandParams
		{
			ChainParams chains[5];
		};

		Hand1(HandParams p, hs::Pose3d* start) :params(p),start(start)
		{
			for (size_t i = 0; i < 5; i++)
			{
				for (size_t j = 0; j < 6; j++)
				{
					getParams((Finger)i)[j] = p.chains[i].joints[j].theta;
				}

				ends[i] = new Pose3d();
			}
		}

		enum Finger {
			Thumb = 0,
			Index = 1,
			Middle = 2,
			Ring = 3,
			Little = 4
		};

		hs::Pose3d* start;
		hs::Pose3d* ends[5];

		// There are six thetas per five fingers (for now, we will add fixed thetas later...)
		double theta[30];

		hs::Pose3d* getEndPose(Finger finger)
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

		template<typename T>
		const T* getParams(const T* const p, Finger f) const {
			return &p[f * 6]; // six joints per finger
		}

		template<typename T>
		hs::Pose3<T> getFingerPose(const T* const handParameterBlock, Finger finger) const
		{
			auto chain = params.chains[finger].joints;
			auto chainAngles = getParams(handParameterBlock, finger);

			hs::Pose3<T> p; // Identity

			for (size_t i = 0; i < 6; i++)
			{
				p = p *
					hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)chain[i].d, (T)0)) *
					hs::Pose3<T>(Eigen::AngleAxis<T>(chainAngles[i], Eigen::Vector3<T>::UnitY())) *
					hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)0, (T)chain[i].r)) *
					hs::Pose3<T>(Eigen::AngleAxis<T>((T)chain[i].a, Eigen::Vector3<T>::UnitZ()));
			}
			return p;
		}

	private:
		HandParams params;

		struct CostFunctor {
			Hand1* hand;

			CostFunctor(Hand1* hand) :hand(hand) {}

			template<typename T>
			T* getResiduals(T* residuals, Finger f) const {
				return &residuals[f * hs::Pose3d::Residuals];
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

				hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * hand->getFingerPose(theta, Thumb), hs::Pose3<T>::Map(thumb), getResiduals(residuals, Thumb));
				hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * hand->getFingerPose(theta, Index), hs::Pose3<T>::Map(index), getResiduals(residuals, Index));
				hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * hand->getFingerPose(theta, Middle), hs::Pose3<T>::Map(middle), getResiduals(residuals, Middle));
				hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * hand->getFingerPose(theta, Ring), hs::Pose3<T>::Map(ring), getResiduals(residuals, Ring));
				hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * hand->getFingerPose(theta, Little), hs::Pose3<T>::Map(little), getResiduals(residuals, Little));

				return true;
			}

			enum {
				Residuals = hs::Pose3d::Residuals * 5,
				Dimension1 = hs::Pose3d::Dimension, // Starting pose
				Dimension2 = hs::Pose3d::Dimension, // Five finger end poses
				Dimension3 = hs::Pose3d::Dimension,
				Dimension4 = hs::Pose3d::Dimension,
				Dimension5 = hs::Pose3d::Dimension,
				Dimension6 = hs::Pose3d::Dimension,
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