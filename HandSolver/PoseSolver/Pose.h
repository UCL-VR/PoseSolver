#pragma once

#define NOMINMAX // Stop minwindef.h overriding std::max, if it gets included somehow

#include <string>
#include <format>

#include <Eigen/Dense>

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/cost_function.h>

#pragma optimize("", off)

namespace hs {

    template<typename T>
    class Rodrigues
    {
    public:
        T data[3];

        // This class is based on 
        // http://link.springer.com/10.1007/s10851-017-0765-x
        // Howevever there are a couple of peculiarities, for one if the below
        // is multiplied by angle, which they should be according to eq 27, the
        // solution is worse than without. Additionally, the solution is poor
        // if eq 22 is used.

        Rodrigues(const Eigen::Quaternion<T> q)
        {
            *this = q;
        }

        Rodrigues(const Eigen::AngleAxis<T> a)
        {
            *this = a;
        }

        Rodrigues(T x, T y, T z)
        {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        Rodrigues()
        {
            this->setIdentity();
        }

        Rodrigues(const T* r)
        {
            data[0] = r[0];
            data[1] = r[1];
            data[2] = r[2];
        }

        // The most common way to use this type is to use it over an existing array
        static Rodrigues& Map(T* data)
        {
            return *((Rodrigues*)data);
        }

        static const Rodrigues& Map(const T* const data)
        {
            return *((Rodrigues*)data);
        }

        Rodrigues<T> inverse() const
        {
            return Rodrigues(-data[0], -data[1], -data[2]);
        }

        void setIdentity()
        {
            data[0] = (T)0.0;
            data[1] = (T)0.0;
            data[2] = (T)0.0;
        }

        T normSquared() const
        {
            return (data[0] * data[0]) + (data[1] * data[1]) + (data[2] * data[2]);
        }

        Eigen::Quaternion<T> toQuaternion() const 
        {
            using namespace Eigen;

            Vector3<T> v(data[0], data[1], data[2]);

            T n2 = normSquared();
            Vector3<T> s = (2.0 * v) / (1.0 + n2);

            return Eigen::Quaternion<T>(
                (1.0 - n2) / (1.0 + n2),
                s.x(),
                s.y(),
                s.z()
            );
        }

        Eigen::Vector3<T> toVector() const {
            return Eigen::Vector3<T>(data[0], data[1], data[2]);
        }

        Eigen::Map<Eigen::Vector3<T>> asVector() {
            return Eigen::Map<Eigen::Vector3<T>>(data);
        }

        // Assignment from Quaternion
        void operator = (const Eigen::Quaternion<T> q)
        {
            data[0] = q.x() / (1.0 + q.w());
            data[1] = q.y() / (1.0 + q.w());
            data[2] = q.z() / (1.0 + q.w());
        }

        // Assignment from AxisAngle
        void operator = (const Eigen::AngleAxis<T> a)
        {
            using namespace Eigen;
            Eigen::Map<Vector3<T>> v(data);

            if (abs(a.angle()) <= DBL_EPSILON) // Because we divide by angle below
            {
                v = Eigen::Vector3<T>::Zero();
            }
            else
            {
                v = (tan(a.angle() / 4.0) / a.angle()) * (a.axis() * a.angle());
            }
        }

        // Composition operator
        Rodrigues<T> operator * (const Rodrigues<T>& r1) const
        {
            auto& r2 = *this; // (just so we can use the notation r1,r2 below...)
            
            // This method implements Eq 28 of Terzakis et al. (Note that the
            // operator order conventions of Eigen are swapped with respect
            // to the paper.)

            auto norm1 = r1.normSquared();
            auto norm2 = r2.normSquared();
            auto v1 = r1.toVector();
            auto v2 = r2.toVector();

            auto a = 2.0 * v1.cross(v2);
            auto b = 2.0 * v1.dot(v2);

            auto c = ((1.0 - norm2) * v1) + ((1.0 - norm1) * v2) - a;
            auto d = 1.0 + (norm1 * norm2) - b;

            Rodrigues<T> r;
            r.asVector() = c / d;
            return r;
        }

    };

    typedef Rodrigues<double> Rodriguesd;

    /// <summary>
    /// Pose3 represents a six-dof transform (or reference frame) in 3D space.
    /// </summary>
    template<typename T>
    class Pose3
    {
    public:

        // This type is expected to cast directly to and from parameter blocks,
        // so declare it directly.
        // (Even Eigen types that would expected to be aligned according to C++
        // rules are not necessarily in practice!)
        T coeffs[6];

        Pose3()
        {
            Position().setZero();
            Rotation().setIdentity();
        }

        Pose3(Eigen::Vector3<T> p, Eigen::AngleAxis<T> r)
        {
            Position() = p;
            Rotation() = r;
        }

        Pose3(Eigen::Vector3<T> p, Eigen::Quaternion<T> r)
        {
            Position() = p;
            Rotation() = r;
        }

        Pose3(Eigen::Vector3<T> p)
        {
            Position() = p;
            Rotation().setIdentity();
        }

        Pose3(Eigen::AngleAxis<T> r)
        {
            Position().setZero();
            Rotation() = r;
        }

        // Returns a reference to a Pose3, as seen over the parameter block a.
        // As the parameter block is const, the returned reference is also
        // necessarily constant, however, generally a Map of a Pose3 onto an
        // array does not have to be.

        static const Pose3<T>& Map(const T* const a)
        {
            return *((Pose3<T>*)a);
        }

        template<typename T2>
        Pose3<T2> Cast() const
        {
            Pose3<T2> p;
            p.coeffs[0] = (T2)coeffs[0];
            p.coeffs[1] = (T2)coeffs[1];
            p.coeffs[2] = (T2)coeffs[2];
            p.coeffs[3] = (T2)coeffs[3];
            p.coeffs[4] = (T2)coeffs[4];
            p.coeffs[5] = (T2)coeffs[5];
            return p;
        }

        // These next methods return an Eigen Map - a view directly of the
        // coefficients, which can be used to set the below properties.

        Eigen::Map<Eigen::Vector3<T>> Position()
        {
            return Eigen::Map<Eigen::Vector3<T>>(coeffs);
        }

        Rodrigues<T>& Rotation()
        {
            return Rodrigues<T>::Map(&coeffs[3]);
        }

        // These next methods return copies of the parameters - these are the
        // const versions of the above, used where the Pose3 must not be
        // modified.

        Eigen::Vector3<T> Position() const
        {
            return Eigen::Vector3<T>(coeffs[0], coeffs[1], coeffs[2]);
        }

        const Rodrigues<T> Rotation() const
        {
            return Rodrigues<T>(&coeffs[3]);
        }

        static void Between(const Pose3<T>& posea, const Pose3<T>& poseb, T* residual)
        {
            using namespace Eigen;

            Eigen::Map<Vector3<T>> pe(&residual[0]);
            Eigen::Map<Vector3<T>> qe(&residual[3]);

            pe = posea.Position() - poseb.Position();
            qe = (posea.Rotation().inverse() * poseb.Rotation()).toVector();
        }

        std::string ToString()
        {
            auto euler = Rotation().toQuaternion().toRotationMatrix().eulerAngles(0, 1, 2);

            return std::format("{} {} {} {} {} {}",
                Position().x(),
                Position().y(),
                Position().z(),
                euler.x(),
                euler.y(),
                euler.z()
            );
        }

        // Compose operator

        Pose3<T> operator * (const Pose3<T>& other) const
        {
            return Pose3<T>(
                Position() + (Rotation().toQuaternion() * other.Position()),
                (Rotation() * other.Rotation()).toQuaternion()
            );
        }

        // Helper overloads to transform common types in 3d space

        Eigen::Vector3<T> operator * (const Eigen::Vector3<T>& vector) const
        {
            return (Rotation().toQuaternion() * vector) + Position();
        }

        T* parameterBlock() 
        {
            return (T*)this;
        }

        enum {
            Dimension = 6,
            Residuals = 6
        };
    };

    typedef Pose3<double> Pose3d;

    // Functors

    // A measurement of a Pose3 - that is, a cost function that says the
    // associated parameter block should match the provided pose.

    struct PoseFunctions {

        /// <summary>
        /// The difference between two poses expressed as a Vector6. The difference in
        /// position is the Eucliden distance, and the rotation expressed as an Euler-
        /// Rodrigues vector.
        /// </summary>
        struct Between {
            template <typename T>
            bool operator()(const T* const a, const T* const b, T* residual) const {

                auto posea = hs::Pose3<T>::Map(a);
                auto poseb = hs::Pose3<T>::Map(b);
                hs::Pose3<T>::Between(posea, poseb, residual);

                return true;
            }

            enum {
                Residuals = 6,
                Dimension = Pose3d::Dimension,
            };

            // Returns the CostFunction represetnation of this Functor. Expects the caller
            // to take ownership.

            static ceres::CostFunction* costFunction() {
                return new ceres::AutoDiffCostFunction<
                    Between,
                    Residuals,
                    Dimension,
                    Dimension>(
                        new Between());
            }
        };

    };

    struct PoseMeasurement {

        const Pose3d& a;

        PoseMeasurement(const Pose3d& p) :a(p)
        {
        }

        template<typename T>
        bool operator()(const T* const b, T* residual) const
        {
            Pose3<T>::Between(a.Cast<T>(), Pose3<T>::Map(b), residual);
            return true;
        }

        enum {
            Residuals = 6,
            Dimension = Pose3d::Dimension
        };

        static ceres::CostFunction* costFunction(const Pose3d& a) {
            return new ceres::AutoDiffCostFunction<
                PoseMeasurement,
                PoseMeasurement::Residuals,
                PoseMeasurement::Dimension>(
                    new PoseMeasurement(a)
                );
        }
    };

    /// <summary>
    /// Connects a Pose with a Point in world space through a Vector.
    /// </summary>
    struct PointMeasurement {

        Eigen::Vector3d point;
        Eigen::Vector3d offset;
        Pose3d* pose;

        PointMeasurement() {
            point.setZero();
            offset.setZero();
            pose = nullptr;
        }

        template<typename T>
        bool operator()(const T* const pose, const T* const point, const T* const offset, T* residual) const {
            using namespace Eigen;

            Map<Vector3<T>> r(residual);
            r = Pose3<T>::Map(pose) * Vector3<T>(offset) - Vector3<T>(point);
            return true;
        }

        enum {
            Residuals = 3,
            Dimension1 = Pose3d::Dimension,
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
    };
}