#pragma once

#include <Eigen/Dense>
#include "Transforms.h"

namespace observations {

    struct MeasurementBase {
    public:
        ceres::ResidualBlockId residualBlockId;

        virtual ceres::CostFunction* costFunction() = 0;
        virtual std::vector<double*> parameterBlocks() = 0;
    };

    struct PointMeasurementBase : MeasurementBase {
    public:
        Eigen::Vector3d point;
        Eigen::Vector3d offset;
        

        double* pointParameterBlock() {
            return point.data();
        }

        double* offsetParameterBlock() {
            return offset.data();
        }
    };

    /// <summary>
    /// Constrains a Pose to be an exact distance to a Point in 3D space. As the
    /// Point does not have a rotation, this effectively constrains the Pose to sit
    /// on the sphere surrounding the point.
    /// </summary>
    struct PointMeasurement : public PointMeasurementBase {

        transforms::Transformd* pose;

        PointMeasurement() {
            point.setZero();
            offset.setZero();
            pose = nullptr;
            residualBlockId = nullptr;
        }

        PointMeasurement(transforms::Transformd* pose) {
            this->pose = pose;
            point.setZero();
            offset.setZero();
            residualBlockId = nullptr;
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
            >(this,
             ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
        }

        std::vector<double*> parameterBlocks() {
            return {
                pose->parameterBlock(),
                point.data(),
                offset.data()
            };
        }

        void addToProblem(ceres::Problem& problem) {
            residualBlockId = problem.AddResidualBlock(
                costFunction(),
                nullptr,
                parameterBlocks()
            );
        }
    };

    /// <summary>
    /// Constrains a Pose to have the exact Orientation in 3D space, regardless of
    /// it's position.
    /// </summary>
    struct OrientationMeasurement
    {
        transforms::Transformd* pose;
        transforms::Rodriguesd orientation;
        ceres::ResidualBlockId residualBlockId;

        OrientationMeasurement()
        {
            pose = nullptr;
            orientation.setIdentity();
            residualBlockId = nullptr;
        }

        OrientationMeasurement(transforms::Transformd* pose) {
            this->pose = pose;
            orientation.setIdentity();
            residualBlockId = nullptr;
        }

        template<typename T>
        bool operator()(const T* const pose, const T* const orientation, T* residual) const {
            using namespace Eigen;
            using namespace transforms;

            Map<Vector3<T>> r(residual);
            r = (transforms::Transform<T>::Map(pose).Rotation().inverse() * transforms::Rodrigues<T>::Map(orientation)).toVector();
            return true;
        }

        enum {
            Residuals = 3,
            Dimension1 = transforms::Transformd::Dimension,
            Dimension2 = 3,
        };

        ceres::CostFunction* costFunction() {
            return new ceres::AutoDiffCostFunction<
                OrientationMeasurement,
                Residuals,
                Dimension1,
                Dimension2
            >(this,
                ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
        }

        std::vector<double*> parameterBlocks() {
            return {
                pose->parameterBlock(),
                orientation.data,
            };
        }

        double* orientationParameterBlock() {
            return orientation.data;
        }

        void addToProblem(ceres::Problem& problem) {
            residualBlockId = problem.AddResidualBlock(
                costFunction(),
                nullptr,
                parameterBlocks()
            );
        }
    };
}