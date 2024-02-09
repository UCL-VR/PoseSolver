#pragma once

#include <Eigen/Dense>
#include "Transforms.h"

namespace observations {

    /// <summary>
    /// Constrains a Pose to be an exact distance to a Point in 3D space. As the
    /// Point does not have a rotation, this effectively constrains the Pose to sit
    /// on the sphere surrounding the point.
    /// </summary>
    struct PointMeasurement {

        Eigen::Vector3d point;
        Eigen::Vector3d offset;
        transforms::Transformd* pose;

        PointMeasurement() {
            point.setZero();
            offset.setZero();
            pose = nullptr;
        }

        PointMeasurement(transforms::Transformd* pose) {
            this->pose = pose;
            point.setZero();
            offset.setZero();
        }

        template<typename T>
        bool operator()(const T* const pose, const T* const point, const T* const offset, T* residual) const {
            using namespace Eigen;
            using namespace transforms;

            Map<Vector3<T>> r(residual);
            r = (transforms::Transform<T>::Map(pose) * Vector3<T>(offset)) - Vector3<T>(point);
            return true;
        }

        enum {
            Residuals = 3,
            Dimension1 = transforms::Transformd::Dimension,
            Dimension2 = 3,
            Dimension3 = 3
        };

        ceres::CostFunction* costFunction() {
            return new ceres::AutoDiffCostFunction<
                PointMeasurement,
                Residuals,
                Dimension1,
                Dimension2,
                Dimension3
            >(this);
        }

        std::vector<double*> parameterBlocks() {
            return {
                pose->parameterBlock(),
                point.data(),
                offset.data()
            };
        }

        double* pointParameterBlock() {
            return point.data();
        }

        double* offsetParameterBlock() {
            return offset.data();
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