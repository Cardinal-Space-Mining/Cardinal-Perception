#pragma once

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/types.hpp>
#include <opencv2/core/quaternion.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>


namespace util
{
namespace geom
{
    #define ASSERT_NUMERIC_TYPE(T) static_assert(std::is_arithmetic<T>::value)
    #define ASSERT_FLOATING_TYPE(T) static_assert(std::is_floating_point<T>::value)

    template<typename T>
    struct Pose3
    {
        ASSERT_FLOATING_TYPE(T);
        using Vec_T = Eigen::Vector3<T>;
        using Quat_T = Eigen::Quaternion<T>;

        Vec_T vec = Eigen::Vector3<T>::Zero();
        Quat_T quat = Quat_T::Identity();
    };
    using Pose3f = Pose3<float>;
    using Pose3d = Pose3<double>;

    template<typename T>
    struct PoseTf3 : Pose3<T>
    {
        ASSERT_FLOATING_TYPE(T);
        using Vec_T = typename Pose3<T>::Vec_T;
        using Quat_T = typename Pose3<T>::Quat_T;
        using Tf_T = Eigen::Transform<T, 3, Eigen::Isometry>;

        Pose3<T> pose;
        Tf_T tf = Tf_T::Identity();
    };
    using PoseTf3f = PoseTf3<float>;
    using PoseTf3d = PoseTf3<double>;

    // vec3
    template<typename T> using eigen_vec3 = Eigen::Vector3<T>;
    template<typename T> using eigen_trl3 = Eigen::Translation<T, 3>;
    template<typename T> using cv_vec3 = cv::Vec<T, 3>;
    template<typename T> using cv_point = cv::Point3_<T>;
    using ros_vec3 = geometry_msgs::msg::Vector3;
    using ros_point = geometry_msgs::msg::Point;
    // vec4
    template<typename T> using eigen_vec4 = Eigen::Vector4<T>;
    template<typename T> using cv_vec4 = cv::Vec<T, 4>;
    // quat
    template<typename T> using eigen_quat = Eigen::Quaternion<T>;
    template<typename T> using cv_quat = cv::Quat<T>;
    using ros_quat = geometry_msgs::msg::Quaternion;
    // pose
    template<typename T> using eigen_tf3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using ros_tf3 = geometry_msgs::msg::Transform;
    using ros_pose = geometry_msgs::msg::Pose;
    template<typename T> using util_pose = util::geom::Pose3<T>;

    namespace cvt
    {
        #define MAKE_ACCESSORS(fname, param, opr) \
            inline static auto& fname(param& v) { return v.opr; } \
            inline static auto fname(const param& v) { return v.opr; }
        #define MAKE_TEMPLATE_ACCESSORS(fname, param, opr) \
            template<typename T> inline static auto& fname(param<T>& v) { return v.opr; } \
            template<typename T> inline static auto fname(const param<T>& v) { return v.opr; }

        template<typename T, typename U> inline
        void cast_assign(T& x, const U& y)
        {
            static_assert(std::is_convertible<T, U>::value);
            if constexpr(std::is_same<T, U>::value) x = y;
            else x = static_cast<T>(y);
        }

        namespace vec3
        {
            namespace traits
            {
                // Eigen::Vector3<T>
                MAKE_TEMPLATE_ACCESSORS(x, eigen_vec3, operator[](0))
                MAKE_TEMPLATE_ACCESSORS(y, eigen_vec3, operator[](1))
                MAKE_TEMPLATE_ACCESSORS(z, eigen_vec3, operator[](2))
                // Eigen::Translation3<T>
                MAKE_TEMPLATE_ACCESSORS(x, eigen_trl3, x())
                MAKE_TEMPLATE_ACCESSORS(y, eigen_trl3, y())
                MAKE_TEMPLATE_ACCESSORS(z, eigen_trl3, z())
                // cv::Vec<T, 3>
                MAKE_TEMPLATE_ACCESSORS(x, cv_vec3, operator[](0))
                MAKE_TEMPLATE_ACCESSORS(y, cv_vec3, operator[](1))
                MAKE_TEMPLATE_ACCESSORS(z, cv_vec3, operator[](2))
                // cv::Point3_<T>
                MAKE_TEMPLATE_ACCESSORS(x, cv_point, x)
                MAKE_TEMPLATE_ACCESSORS(y, cv_point, y)
                MAKE_TEMPLATE_ACCESSORS(z, cv_point, z)
                // geometry_msgs::msg::Vector3
                MAKE_ACCESSORS(x, ros_vec3, x)
                MAKE_ACCESSORS(y, ros_vec3, y)
                MAKE_ACCESSORS(z, ros_vec3, z)
                // geometry_msgs::msg::Point
                MAKE_ACCESSORS(x, ros_point, x)
                MAKE_ACCESSORS(y, ros_point, y)
                MAKE_ACCESSORS(z, ros_point, z)
            };

            template<
                typename A,
                typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::vec3::traits;
                cvt::cast_assign(x(a), x(b));
                cvt::cast_assign(y(a), y(b));
                cvt::cast_assign(z(a), z(b));
                return a;
            }
        };

        namespace quat
        {
            namespace traits
            {
                // Eigen::Quaternion<T>
                MAKE_TEMPLATE_ACCESSORS(w, eigen_quat, w())
                MAKE_TEMPLATE_ACCESSORS(x, eigen_quat, x())
                MAKE_TEMPLATE_ACCESSORS(y, eigen_quat, y())
                MAKE_TEMPLATE_ACCESSORS(z, eigen_quat, z())
                // cv::Quat<T>
                MAKE_TEMPLATE_ACCESSORS(w, cv_quat, w)
                MAKE_TEMPLATE_ACCESSORS(x, cv_quat, x)
                MAKE_TEMPLATE_ACCESSORS(y, cv_quat, y)
                MAKE_TEMPLATE_ACCESSORS(z, cv_quat, z)
                // geometry_msgs::msg::Quaternion
                MAKE_ACCESSORS(w, ros_quat, w)
                MAKE_ACCESSORS(x, ros_quat, x)
                MAKE_ACCESSORS(y, ros_quat, y)
                MAKE_ACCESSORS(z, ros_quat, z)
            };

            template<
                typename A,
                typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::quat::traits;
                cvt::cast_assign(w(a), w(b));
                cvt::cast_assign(x(a), x(b));
                cvt::cast_assign(y(a), y(b));
                cvt::cast_assign(z(a), z(b));
                return a;
            }
        };

        namespace vec4
        {
            template<typename T, typename U> inline // EIGEN to CV
            cv_vec4<T>& cvt(cv_vec4<T>& a, const eigen_vec4<U>& b)
            {
                cvt::cast_assign(a[0], b[0]);
                cvt::cast_assign(a[1], b[1]);
                cvt::cast_assign(a[2], b[2]);
                cvt::cast_assign(a[3], b[3]);
                return a;
            }
            template<typename T, typename U> inline // CV to EIGEN
            eigen_vec4<T>& cvt(eigen_vec4<T>& a, const cv_vec4<U>& b)
            {
                cvt::cast_assign(a[0], b[0]);
                cvt::cast_assign(a[1], b[1]);
                cvt::cast_assign(a[2], b[2]);
                cvt::cast_assign(a[3], b[3]);
                return a;
            }
        };

        namespace vecN
        {
            template<typename T, typename U, std::size_t N> inline
            Eigen::Matrix<T, N, 1>& cvt(Eigen::Matrix<T, N, 1>& a, const cv::Vec<U, N>& b)
            {
                for(std::size_t i = 0; i < N; i++)
                {
                    cvt::cast_assign(a[i], b[i]);
                }
                return a;
            }
            template<typename T, typename U, std::size_t N> inline
            cv::Vec<U, N>& cvt(cv::Vec<U, N>& a, const Eigen::Matrix<T, N, 1>& b)
            {
                for(std::size_t i = 0; i < N; i++)
                {
                    cvt::cast_assign(a[i], b[i]);
                }
                return a;
            }
        };

        namespace pose
        {
            namespace traits
            {
                // geometry_msgs::msg::Transform
                MAKE_ACCESSORS(vec_, ros_tf3, translation)
                MAKE_ACCESSORS(quat_, ros_tf3, rotation)
                // geometry_msgs::msg::Pose
                MAKE_ACCESSORS(vec_, ros_pose, position)
                MAKE_ACCESSORS(quat_, ros_pose, orientation)
                // util::geom::Pose3<T>
                MAKE_TEMPLATE_ACCESSORS(vec_, util_pose, vec)
                MAKE_TEMPLATE_ACCESSORS(quat_, util_pose, quat)
            };

            template<typename A, typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::pose::traits;
                vec3::cvt(vec_(a), vec_(b));
                quat::cvt(quat_(a), quat_(b));
                return a;
            }
            template<typename T, typename B>
            inline eigen_tf3<T>& cvt(eigen_tf3<T>& a, const B& b)
            {
                using namespace util::geom::cvt::pose::traits;
                eigen_trl3<T> t;
                eigen_quat<T> r;
                vec3::cvt(t, vec_(b));
                quat::cvt(r, quat_(b));
                a = t * r;
                return a;
            }
            template<typename A, typename T>
            inline A& cvt(A& a, const eigen_tf3<T>& b)
            {
                using namespace util::geom::cvt::pose::traits;
                eigen_vec3<T> t;
                eigen_quat<T> r;
                t = b.translation();
                r = b.rotation();
                vec3::cvt(vec_(a), t);
                quat::cvt(quat_(a), r);
                return a;
            }
            template<typename T, typename U>
            inline eigen_tf3<T>& cvt(eigen_tf3<T>& a, const eigen_tf3<U>& b)
            {
                static_assert(std::is_convertible<T, U>::value);
                if constexpr(std::is_same<T, U>::value) a = b;
                else a = b.template cast<T>();
                return a;
            }
        };

        #undef MAKE_ACCESSORS
        #undef MAKE_TEMPLATE_ACCESSORS


        namespace ops
        {
            #define MAKE_DOUBLE_TEMPLATED_OPS(type, tA, tB) \
                template<typename T, typename U> inline tA<T>& operator<<(tA<T>& a, const tB<U>& b) { return type::cvt(a, b); } \
                template<typename T, typename U> inline tB<T>& operator<<(tB<T>& a, const tA<U>& b) { return type::cvt(a, b); } \
                template<typename T, typename U> inline tB<T>& operator>>(const tA<T>& a, tB<U>& b) { return type::cvt(b, a); } \
                template<typename T, typename U> inline tA<T>& operator>>(const tB<T>& a, tA<U>& b) { return type::cvt(b, a); }
            #define MAKE_PRIMARY_TEMPLATED_OPS(type, tA, B) \
                template<typename T> inline tA<T>& operator<<(tA<T>& a, const B& b) { return type::cvt(a, b); } \
                template<typename T> inline B&     operator<<(B& a, const tA<T>& b) { return type::cvt(a, b); } \
                template<typename T> inline B&     operator>>(const tA<T>& a, B& b) { return type::cvt(b, a); } \
                template<typename T> inline tA<T>& operator>>(const B& a, tA<T>& b) { return type::cvt(b, a); }
            #define MAKE_STATIC_OPS(type, A, B) \
                inline A& operator<<(A& a, const B& b) { return type::cvt(a, b); } \
                inline B& operator<<(B& a, const A& b) { return type::cvt(a, b); } \
                inline B& operator>>(const A& a, B& b) { return type::cvt(b, a); } \
                inline A& operator>>(const B& a, A& b) { return type::cvt(b, a); }


            // vec3 ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, eigen_trl3)
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, cv_vec3)
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, cv_point)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_vec3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_vec3, ros_point)

            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_trl3, cv_vec3)
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_trl3, cv_point)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_trl3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_trl3, ros_point)

            MAKE_DOUBLE_TEMPLATED_OPS( vec3, cv_vec3, cv_point)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_vec3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_vec3, ros_point)

            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_point, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_point, ros_point)

            MAKE_STATIC_OPS(           vec3, ros_vec3, ros_point)

            // quat ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( quat, eigen_quat, cv_quat)
            MAKE_PRIMARY_TEMPLATED_OPS(quat, eigen_quat, ros_quat)

            MAKE_PRIMARY_TEMPLATED_OPS(quat, cv_quat, ros_quat)

            // vec4 ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( vec4, eigen_vec4, cv_vec4)

            // pose ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( pose, eigen_tf3, util_pose)
            MAKE_PRIMARY_TEMPLATED_OPS(pose, eigen_tf3, ros_tf3)
            MAKE_PRIMARY_TEMPLATED_OPS(pose, eigen_tf3, ros_pose)

            MAKE_PRIMARY_TEMPLATED_OPS(pose, util_pose, ros_tf3)
            MAKE_PRIMARY_TEMPLATED_OPS(pose, util_pose, ros_pose)

            MAKE_STATIC_OPS(           pose, ros_tf3, ros_pose)


            #undef MAKE_DOUBLE_TEMPLATED_OPS
            #undef MAKE_PRIMARY_TEMPLATED_OPS
            #undef MAKE_STATIC_OPS
        };
    };


// pose component ops

    template<typename T> inline
    void component_diff(
        geom::Pose3<T>& diff,
        const geom::Pose3<T>& from,
        const geom::Pose3<T>& to)
    {
        diff.vec = to.vec - from.vec;
        diff.quat = from.quat.inverse() * to.quat;
    }

// pose lerping

    template<typename float_T> inline
    void lerpSimple(
        geom::Pose3<float_T>& interp,
        const geom::Pose3<float_T>& diff,
        const float_T alpha)
    {
        lerpSimple<float_T>(interp, diff.vec, diff.quat, alpha);
    }

    template<typename float_T>
    void lerpSimple(
        geom::Pose3<float_T>& interp,
        const geom::eigen_vec3<float_T>& v,
        const geom::eigen_quat<float_T>& q,
        const float_T alpha)
    {
        ASSERT_FLOATING_TYPE(float_T);

        interp.vec = v * alpha;
        interp.quat = geom::eigen_vec3<float_T>::Identity().slerp(alpha, q);
    }

    template<typename float_T> inline
    void lerpCurvature(
        geom::Pose3<float_T>& interp,
        const geom::Pose3<float_T>& diff,
        const float_T alpha)
    {
        lerpCurvature<float_T>(interp, diff.vec, diff.quat, alpha);
    }

    template<typename float_T>
    void lerpCurvature(
        geom::Pose3<float_T>& interp,
        const geom::eigen_vec3<float_T>& v,
        const geom::eigen_quat<float_T>& q,
        float_T alpha)
    {
        ASSERT_FLOATING_TYPE(float_T);

        using Vec3 = geom::eigen_vec3<float_T>;
        using Quat = geom::eigen_quat<float_T>;
        using Mat3 = Eigen::Matrix3<float_T>;

        // https://github.com/wpilibsuite/allwpilib/blob/79dfdb9dc5e54d4f3e02fb222106c292e85f3859/wpimath/src/main/native/cpp/geometry/Pose3d.cpp#L80-L177

        const Quat qI = Quat::Identity();
        Mat3 omega = q.toRotationMatrix();
        Mat3 omega_sq = omega * omega;

        double theta = qI.angularDistance(q);
        double theta_sq = theta * theta;

        double A, B, C;
        if(std::abs(theta) < 1e-7)
        {
            C = (1. / 12.) + (theta_sq / 720.) + (theta_sq * theta_sq / 30240.);
        }
        else
        {
            A = std::sin(theta) / theta,
            B = (1. - std::cos(theta)) / theta_sq;
            C = (1. - A / (2. * B)) / theta_sq;
        }

        Mat3 V_inv = Mat3::Identity() - 0.5 * omega + C * omega_sq;

        Vec3 twist_translation = V_inv * v * alpha;
        Quat twist_rotation = qI.slerp(alpha, q);

        omega = twist_rotation;
        omega_sq = omega * omega;
        theta = qI.angularDistance(twist_rotation);
        theta_sq = theta * theta;

        if(std::abs(theta) < 1e-7)
        {
            const double theta_4 = theta_sq * theta_sq;

            A = (1. - theta_sq) / 6. + (theta_4 / 120.);
            B = (1. / 2.) - (theta_sq / 24.) + (theta_4 / 720.);
            C = (1. / 6.) - (theta_sq / 120.) + (theta_4 / 5040.);
        }
        else
        {
            A = std::sin(theta) / theta;
            B = (1. - std::cos(theta)) / theta_sq;
            C = (1. - A) / theta_sq;
        }

        Mat3 V = Mat3::Identity() + B * omega + C * omega_sq;
        Mat3 R = Mat3::Identity() + A * omega + B * omega_sq;

        interp.vec = V * twist_translation;
        interp.quat = R;
    }

    #undef ASSERT_NUMERIC_TYPE
    #undef ASSERT_FLOATING_TYPE
};
};