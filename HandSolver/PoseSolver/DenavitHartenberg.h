#pragma once

#include "Pose.h"

#include <vector>

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/problem.h>

namespace dh {

    struct JointParams
    {
        double d;
        double theta;
        double r;
        double a;
    };

    /// <summary>
    /// Represents a fixed-length chain of Denavit-Hartenberg Joints. The chain
    /// connects two Poses. Each joint can vary its rotation angle (theta) but
    /// all other parameters are fixed.
    /// </summary>
    template<int N> // The number of links in the chain
    class Chain
    {
    public:
        Chain(std::vector<JointParams> params)
        {
            for (size_t i = 0; i < N; i++)
            {
                this->theta = params[i].theta;
                d = params[i].d;
                r = params[i].r;
                a = params[i].a;
            }
        }

        hs::Pose3d* start;
        hs::Pose3d* end;

    private:
        double d[N];
        double r[N];
        double a[N];
        double theta[N];

        // Gets the pose of the entire chain (the pose between the starting
        // reference frame to the ending reference frame).
        template<typename T>
        hs::Pose3<T> getLocalPose(const T* const theta) const
        {
            hs::Pose3<T> p; // Identity
            for (size_t i = 0; i < N; i++)
            {
                p = p *
                    hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)d[i], (T)0)) *
                    hs::Pose3<T>(Eigen::AngleAxis<T>(theta[i], Eigen::Vector3<T>::UnitY())) *
                    hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)0, (T)r[i])) *
                    hs::Pose3<T>(Eigen::AngleAxis<T>((T)a[i], Eigen::Vector3<T>::UnitZ()));
            }
            return p;
        }

        struct CostFunctor {
            Chain* joint;

            CostFunctor(Chain* joint) : joint(joint) {}

            template<typename T>
            bool operator()(const T* const start, const T* const end, const T* const theta, T* residuals) const {
                hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * joint->getLocalPose(theta), hs::Pose3<T>::Map(end), residuals);
                return true;
            }

            enum {
                Residuals = hs::Pose3d::Residuals,
                Dimension1 = hs::Pose3d::Dimension,
                Dimension2 = hs::Pose3d::Dimension,
                Dimension3 = N
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
            return theta;
        }

        std::vector<double*> parameterBlocks() {
            return std::vector<double*>({ start->parameterBlock(), end->parameterBlock(), parameterBlock() });
        }
    };

    /// <summary>
    /// Represents a single Denavit-Hartenberg Joint and Link in a kinematic chain.
    /// The joint connects two Poses, with the joint rotation angle theta. The joint
    /// holds the parameter block for theta. The starting and ending reference frames
    /// are owned externally, however the type contains pointers to them itself.
    /// The start pose is the refernce frame the joint begins in, all offsets and
    /// rotations manifest in the end pose, which is the ending reference frame in
    /// DH conventions.
    /// </summary>
    class Joint
    {
    public:
        double d;
        double theta;
        double r;
        double a;

        hs::Pose3d* start;
        hs::Pose3d* end;

        Joint() {
            d = 0;
            theta = 0;
            r = 1;
            a = 0;
            start = nullptr;
            end = nullptr;
        }

        template<typename T>
        hs::Pose3<T> getLocalPose(const T* const theta) const
        {
            return
                hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)d, (T)0)) *
                hs::Pose3<T>(Eigen::AngleAxis<T>(*theta, Eigen::Vector3<T>::UnitY())) *
                hs::Pose3<T>(Eigen::Vector3<T>((T)0, (T)0, (T)r)) *
                hs::Pose3<T>(Eigen::AngleAxis<T>((T)a, Eigen::Vector3<T>::UnitZ()));
        }

        struct CostFunctor {
            Joint* joint;

            CostFunctor(Joint* dj) : joint(dj) {}

            template<typename T>
            bool operator()(const T* const start, const T* const end, const T* const theta, T* residuals) const {

                // A single DH joint connects two reference frames, start and end.
                // Expressed as a cost, end should equal the starting frame perturbed
                // by the joint variables (one of which here is also a parameter
                // block).

                hs::Pose3<T>::Between(hs::Pose3<T>::Map(start) * joint->getLocalPose(theta), hs::Pose3<T>::Map(end), residuals);

                return true;
            }

            enum {
                Residuals = hs::Pose3d::Residuals,
                Dimension1 = hs::Pose3d::Dimension,
                Dimension2 = hs::Pose3d::Dimension,
                Dimension3 = 1
            };
        };

        // Creates a cost function with three constants for the d, r and a variables
        // of the DH linkage.
        // The parameter blocks expected are the starting pose, end pose and theta.

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
            return &theta;
        }

        std::vector<double*> parameterBlocks() {
            return std::vector<double*>({ start->parameterBlock(), end->parameterBlock(), parameterBlock() });
        }
    };
       
    /// <summary>
    /// Represents the joint limits of a single joint
    /// </summary>
    class JointLimit
    {
    public:
        double min;
        double max;
        Joint* joint;
        double scale;

        JointLimit(Joint* joint, double min, double max, double scale)
        {
            this->joint = joint;
            this->scale = scale;
            this->min = min * scale;
            this->max = max * scale;
        }

        struct CostFunctor {
            double min;
            double max;
            double scale;

            CostFunctor(double min, double max, double scale) : min(min), max(max), scale(scale) {}

            template<typename T>
            bool operator()(const T* const theta, T* residuals) const {

                auto x = theta[0] * (T)scale;
                auto y_max = 1.0 / (1.0 + exp(-(x - (T)max - 6.0)));
                auto y_min = 1.0 / (1.0 + exp(-((T)min - x - 6.0)));
                residuals[0] = (y_min + y_max);
                return true;
            }

            enum {
                Residuals = 1,
                Dimension1 = 1,
            };
        };

        ceres::CostFunction* costFunction() {
            return new ceres::AutoDiffCostFunction<
                CostFunctor,
                CostFunctor::Residuals,
                CostFunctor::Dimension1>(
                    new CostFunctor(min, max, scale)
                );
        }

        std::vector<double*> parameterBlocks() {
            return std::vector<double*>({ joint->parameterBlock() });
        }

        void addToProblem(ceres::Problem& problem) {
            problem.AddResidualBlock(
                costFunction(),
                nullptr,
                parameterBlocks()
            );
        }
    };

    struct JointManifoldFunctor {
        double min;
        double max;

        JointManifoldFunctor(double min, double max) :min(min), max(max) {}

        template<typename T>
        bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
            if (x[0] > max)
            {
                x_plus_delta[0] = (T)max;
            }
            else if (x[0] < min)
            {
                x_plus_delta[0] = (T)min;
            }
            else
            {
                x_plus_delta[0] = x[0] + delta[0];
            }
            return true;
        }

        template <typename T>
        bool Minus(const T* y, const T* x, T* y_minus_x) const {

            if (x[0] > max)
            {
                y_minus_x[0] = (T)0;
            }
            else if (x[0] < min)
            {
                y_minus_x[0] = (T)0;
            }
            else
            {
                y_minus_x[0] = y[0] - x[0];
            }
            return true;
        }
    };
}