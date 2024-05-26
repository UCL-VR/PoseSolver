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

namespace transforms {

    template<typename T>
    class Rodrigues
    {
    public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        T data[3];

        // This class is based on 
        // http://link.springer.com/10.1007/s10851-017-0765-x

        Rodrigues(const Eigen::Quaternion<T> q)
        {
            *this = q; // Invokes the assignment from Quaternion operator
        }

        Rodrigues(const Eigen::AngleAxis<T> a)
        {
            *this = a; // Invokes the assignment from Axis Angle operator
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

        template<typename T2>
        Rodrigues<T2> Cast() const
        {
            Rodrigues<T2> r;
            r.data[0] = (T2)data[0];
            r.data[1] = (T2)data[1];
            r.data[2] = (T2)data[2];
            return r;
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
            *this = Eigen::Quaternion<T>(a); // This calls the Quaternion assignment operator
        }

        // Composition operator
        Rodrigues<T> operator * (const Rodrigues<T>& r1) const
        {
            auto& r2 = *this; // (just so we can use the notation r1,r2 below...)
            
            // This method implements Eq 28 of Terzakis et al. (Note that the
            // operator order conventions of Eigen are swapped with respect
            // to the paper.)

            T norm1 = r1.normSquared();
            T norm2 = r2.normSquared();
            Eigen::Vector3<T> v1 = r1.toVector();
            Eigen::Vector3<T> v2 = r2.toVector();

            Eigen::Vector3<T> a = 2.0 * v1.cross(v2);
            T b = 2.0 * v1.dot(v2);

            Eigen::Vector3<T> c = ((1.0 - norm2) * v1) + ((1.0 - norm1) * v2) - a;
            T d = 1.0 + (norm1 * norm2) - b;

            Rodrigues<T> r;
            r.asVector() = c / d;
            return r;
        }

        // rotate vector operator (this converts to a quaternion)
        Eigen::Vector3<T> operator * (const Eigen::Vector3<T>& v) const
        {
            return toQuaternion() * v;
        }

        T* parameterBlock() {
            return (T*)this;
        }
    };

    typedef Rodrigues<double> Rodriguesd;

    /// <summary>
    /// Transform represents a six-dof transform (or reference frame) in 3D space.
    /// </summary>
    template<typename T>
    class Transform
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // This type is expected to cast directly to and from parameter blocks,
        // so declare it directly.
        // (Even Eigen types that would expected to be aligned according to C++
        // rules are not necessarily in practice!)
        T coeffs[6];

        Transform()
        {
            Position().setZero();
            Rotation().setIdentity();
        }

        Transform(Eigen::Vector3<T> p, Eigen::AngleAxis<T> r)
        {
            Position() = p;
            Rotation() = r;
        }

        Transform(Eigen::Vector3<T> p, Eigen::Quaternion<T> r)
        {
            Position() = p;
            Rotation() = r;
        }

        Transform(Eigen::Vector3<T> p, Rodrigues<T> r)
        {
            Position() = p;
            Rotation() = r;
        }

        Transform(Eigen::Vector3<T> p)
        {
            Position() = p;
            Rotation().setIdentity();
        }

        Transform(Eigen::AngleAxis<T> r)
        {
            Position().setZero();
            Rotation() = r;
        }

        // Returns a reference to a Transform, as seen over the parameter block a.
        // As the parameter block is const, the returned reference is also
        // necessarily constant, however, generally a Map of a Transform onto an
        // array does not have to be.

        static const Transform<T>& Map(const T* const a)
        {
            return *((Transform<T>*)a);
        }

        template<typename T2>
        Transform<T2> Cast() const
        {
            Transform<T2> p;
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
        // const versions of the above, used where the Transform must not be
        // modified.

        Eigen::Vector3<T> Position() const
        {
            return Eigen::Vector3<T>(coeffs[0], coeffs[1], coeffs[2]);
        }

        const Rodrigues<T> Rotation() const
        {
            return Rodrigues<T>(&coeffs[3]);
        }

        static void Between(const Transform<T>& posea, const Transform<T>& poseb, T* residual)
        {
            using namespace Eigen;

            Eigen::Map<Vector3<T>> pe(&residual[0]);
            Eigen::Map<Vector3<T>> qe(&residual[3]);

            pe = poseb.Position() - posea.Position();
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

        std::string ToString2()
        {
            return std::format("{} {} {} {} {} {} {}",
                Position().x(),
                Position().y(),
                Position().z(),
                Rotation().toQuaternion().x(),
                Rotation().toQuaternion().y(),
                Rotation().toQuaternion().z(),
                Rotation().toQuaternion().w()
            );
        }

        // Compose operator

        Transform<T> operator * (const Transform<T>& other) const
        {
            return Transform<T>(
                Position() + (Rotation().toQuaternion() * other.Position()),
                (Rotation() * other.Rotation()).toQuaternion()
            );
        }

        // Inverse operator

        Transform<T> inverse()
        {
            return Transform<T>(-(Rotation().inverse().toQuaternion() * Position()), Rotation().inverse());
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

    typedef Transform<double> Transformd;
}
