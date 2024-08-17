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

        Vec_T vec = Vec_T::Zero();
        Quat_T quat = Quat_T::Identity();
    };
    using Pose3f = Pose3<float>;
    using Pose3d = Pose3<double>;

    template<typename T>
    struct PoseTf3 : Pose3<T>
    {
        using typename Pose3<T>::Vec_T;
        using typename Pose3<T>::Quat_T;
        using Tf_T = Eigen::Transform<T, 3, Eigen::Isometry>;

        Tf_T tf = Tf_T::Identity();
    };
    using PoseTf3f = PoseTf3<float>;
    using PoseTf3d = PoseTf3<double>;

    /**
    Eigen:
        Vector3<fT>
        Quaternion<fT>
        Translation<fT>
        Transform<fT...> -- Isometry
        Vector4<fT>???
    CV:
        Vec3<fT>
        Quat<fT>
        Vec4<fT>
    RCL:
        Vector3
        Quaternion
        Transform, Pose
    Util:
        Pose3[tf]
    **/

    // v3
    template<typename T> using eigen_vec3 = Eigen::Vector3<T>;
    template<typename T> using eigen_trl3 = Eigen::Translation<T, 3>;
    template<typename T> using cv_vec3 = cv::Vec<T, 3>;
    template<typename T> using cv_point = cv::Point3_<T>;
    using ros_vec3 = geometry_msgs::msg::Vector3;
    using ros_point = geometry_msgs::msg::Point;
    // v4
    template<typename T> using eigen_vec4 = Eigen::Vector4<T>;
    template<typename T> using cv_vec4 = cv::Vec<T, 4>;
    // quat
    template<typename T> using eigen_quat = Eigen::Quaternion<T>;
    template<typename T> using cv_quat = cv::Quat<T>;
    using ros_quat = geometry_msgs::msg::Quaternion;
    // tf
    template<typename T> using eigen_tf3 = Eigen::Transform<T, 3, Eigen::Isometry>;
    using ros_tf3 = geometry_msgs::msg::Transform;
    using ros_pose = geometry_msgs::msg::Pose;
    template<typename T> using util_pose = util::geom::Pose3<T>;

    namespace cvt
    {
        // internal use only
        template<typename T, typename U> inline
        T to_(U v)
        {
            if constexpr(std::is_same<T, U>::value) return v;
            else return static_cast<T>(v);
        }
        template<typename T> inline
        double to_double(T v)
        {
            return to_<double>(v);
        }

        template<typename T, typename U> inline
        void cast_assign(T& x, const U& y)
        {
            if constexpr(std::is_same<T, U>::value) x = y;
            else x = static_cast<T>(y);
        }

    namespace vec3
    {
        namespace traits
        {
            #define MAKE_ACCESSORS(fname, param, opr) \
                inline static double& fname(param& v) { return v.opr; } \
                inline static double fname(const param& v) { return v.opr; }
            #define MAKE_TEMPLATE_ACCESSORS(fname, param, opr) \
                template<typename T> inline static T& fname(param<T>& v) { return v.opr; } \
                template<typename T> inline static T fname(const param<T>& v) { return v.opr; }

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
            template<typename...> typename A,
            template<typename...> typename B,
            typename... Ta,
            typename... Tb>
        inline A<Ta...>& cvt(A<Ta...>& a, const B<Tb...>& b)
        {
            using namespace cvt::vec3::traits;
            cvt::cast_assign(x(a), x(b));
            cvt::cast_assign(y(a), y(b));
            cvt::cast_assign(z(a), z(b));
            return a;
        }
        template<
            template<typename...> typename A,
            typename B,
            typename... Ta>
        inline A<Ta...>& cvt(A<Ta...>& a, const B& b)
        {
            using namespace cvt::vec3::traits;
            cvt::cast_assign(x(a), x(b));
            cvt::cast_assign(y(a), y(b));
            cvt::cast_assign(z(a), z(b));
            return a;
        }
        template<
            typename A,
            template<typename...> typename B,
            typename... Tb>
        inline A& cvt(A& a, const B<Tb...>& b)
        {
            using namespace cvt::vec3::traits;
            cvt::cast_assign(x(a), x(b));
            cvt::cast_assign(y(a), y(b));
            cvt::cast_assign(z(a), z(b));
            return a;
        }
        template<
            typename A,
            typename B>
        inline A& cvt(A& a, const B& b)
        {
            using namespace cvt::vec3::traits;
            cvt::cast_assign(x(a), x(b));
            cvt::cast_assign(y(a), y(b));
            cvt::cast_assign(z(a), z(b));
            return a;
        }

        template<typename A, typename B>
        inline A& operator<<(A& a, const B& b)
        {
            return vec3::cvt(a, b);
        }
        template<typename A, typename B>
        inline B& operator>>(const A& a, B& b)
        {
            return vec3::cvt(b, a);
        }
    };


        // template<typename T, typename rT>
        // struct vec3_traits
        // {
        //     static_assert(false, "No template specialization for 'vec3_traits' was found.");

        //     static rT& x(T&);
        //     static rT& y(T&);
        //     static rT& z(T&);
        //     static const rT& x(const T&);
        //     static const rT& y(const T&);
        //     static const rT& z(const T&);
        // };

        // template<typename fT>
        // struct vec3_traits<eigen_vec3<fT>, fT>
        // {
        //     inline static fT& x(eigen_vec3<fT>& v) { return v.x(); }
        //     inline static fT& y(eigen_vec3<fT>& v) { return v.y(); }
        //     inline static fT& z(eigen_vec3<fT>& v) { return v.z(); }
        //     inline static const fT& x(const eigen_vec3<fT>& v) { return v.x(); }
        //     inline static const fT& y(const eigen_vec3<fT>& v) { return v.y(); }
        //     inline static const fT& z(const eigen_vec3<fT>& v) { return v.z(); }
        // };

        // struct vec3_traits<ros_vec3, double>
        // {
        //     inline static double& x(ros_vec3& v) { return v.x; }
        //     inline static double& y(ros_vec3& v) { return v.y; }
        //     inline static double& z(ros_vec3& v) { return v.z; }
        //     inline static const double& x(const ros_vec3& v) { return v.x; }
        //     inline static const double& y(const ros_vec3& v) { return v.y; }
        //     inline static const double& z(const ros_vec3& v) { return v.z; }
        // };


    /** Vec3 */

        template<typename T> inline // CV to ROS
        ros_vec3& vec3_(ros_vec3& a, const cv_vec3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x = to_double(b[0]);
            // a.y = to_double(b[1]);
            // a.z = to_double(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_vec3& vec3_(ros_vec3& a, const eigen_vec3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x = to_double(b[0]);
            // a.y = to_double(b[1]);
            // a.z = to_double(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_point& vec3_(ros_point& a, const eigen_vec3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x = to_double(b[0]);
            // a.y = to_double(b[1]);
            // a.z = to_double(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_vec3<T>& vec3_(eigen_vec3<T>& a, const ros_vec3& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a[0] = to_<T>(b.x);
            // a[1] = to_<T>(b.y);
            // a[2] = to_<T>(b.z);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_vec3<T>& vec3_(eigen_vec3<T>& a, const ros_point& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a[0] = to_<T>(b.x);
            // a[1] = to_<T>(b.y);
            // a[2] = to_<T>(b.z);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // ROS to EIGEN (translation from tf)
        eigen_vec3<T>& vec3_(eigen_vec3<T>& a, const ros_tf3& b)
        {
            return cvt::vec3_(a, b.translation);
        }
        template<typename T> inline // ROS to EIGEN (position from pose)
        eigen_vec3<T>& vec3_(eigen_vec3<T>& a, const ros_pose& b)
        {
            return cvt::vec3_(a, b.position);
        }
        template<typename T, typename U> inline // CV to EIGEN
        eigen_vec3<T>& vec3_(eigen_vec3<T>& a, const cv_vec3<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            // a.x() = to_<T>(b[0]);
            // a.y() = to_<T>(b[1]);
            // a.z() = to_<T>(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }

    /** Translation3 */

        template<typename T, typename U> inline // CV to EIGEN
        eigen_trl3<T>& trl3(eigen_trl3<T>& a, const cv_vec3<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            // a.x() = to_<T>(b[0]);
            // a.y() = to_<T>(b[1]);
            // a.z() = to_<T>(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T, typename U> inline // EIGEN to EIGEN
        eigen_trl3<T>& trl3(eigen_trl3<T>& a, const eigen_vec3<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            // a.x() = to_<T>(b[0]);
            // a.y() = to_<T>(b[1]);
            // a.z() = to_<T>(b[2]);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_trl3<T>& trl3(eigen_trl3<T>& a, const ros_vec3& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x() = to_<T>(b.x);
            // a.y() = to_<T>(b.y);
            // a.z() = to_<T>(b.z);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_trl3<T>& trl3(eigen_trl3<T>& a, const ros_point& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x() = to_<T>(b.x);
            // a.y() = to_<T>(b.y);
            // a.z() = to_<T>(b.z);
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_vec3& trl3(ros_vec3& a, const eigen_trl3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x = to_<T>(b.x());
            // a.y = to_<T>(b.y());
            // a.z = to_<T>(b.z());
            // return a;
            return vec3::cvt(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_point& trl3(ros_point& a, const eigen_trl3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            // a.x = to_<T>(b.x());
            // a.y = to_<T>(b.y());
            // a.z = to_<T>(b.z());
            // return a;
            return vec3::cvt(a, b);
        }

        template<typename T> inline // EIGEN (tf) to ROS
        ros_vec3& vec3_(ros_vec3& a, const eigen_tf3<T>& b)
        {
            eigen_trl3<T> t;
            t = b.translation();

            // return cvt::trl3(a, t);
            return vec3::cvt(a, b);
        }
        template<typename T> inline // EIGEN (tf) to ROS
        ros_point& vec3_(ros_point& a, const eigen_tf3<T>& b)
        {
            eigen_trl3<T> t;
            t = b.translation();

            // return cvt::trl3(a, t);
            return vec3::cvt(a, b);
        }

    /** Quaternion */

        template<typename T> inline // CV to ROS
        ros_quat& quat(ros_quat& a, const cv_quat<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            a.w = to_double(b.w);
            a.x = to_double(b.x);
            a.y = to_double(b.y);
            a.z = to_double(b.z);
            return a;
        }
        template<typename T> inline // EIGEN to ROS
        ros_quat& quat(ros_quat& a, const eigen_quat<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            a.w = to_double(b.w());
            a.x = to_double(b.x());
            a.y = to_double(b.y());
            a.z = to_double(b.z());
            return a;
        }
        template<typename T> inline // EIGEN (tf) to ROS
        ros_quat& quat(ros_quat& a, const eigen_tf3<T>& b)
        {
            eigen_quat<T> r;
            r = b.rotation();

            return cvt::quat(a, r);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_quat<T>& quat(eigen_quat<T>& a, const ros_quat& b)
        {
            ASSERT_FLOATING_TYPE(T);
            a.w() = to_<T>(b.w);
            a.x() = to_<T>(b.x);
            a.y() = to_<T>(b.y);
            a.z() = to_<T>(b.z);
            return a;
        }
        template<typename T> inline // ROS to EIGEN (rotation from tf)
        eigen_quat<T>& quat(eigen_quat<T>& a, const ros_tf3& b)
        {
            return cvt::quat(a, b.rotation);
        }
        template<typename T> inline // ROS to EIGEN (orientation from pose)
        eigen_quat<T>& quat(eigen_quat<T>& a, const ros_pose& b)
        {
            return cvt::quat(a, b.orientation);
        }
        template<typename T, typename U> inline
        eigen_quat<T>& quat(eigen_quat<T>& a, const cv_quat<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            a.w() = to_<T>(b.w);
            a.x() = to_<T>(b.x);
            a.y() = to_<T>(b.y);
            a.z() = to_<T>(b.z);
            return a;
        }

    /** Vec4 */

        template<typename T, typename U> inline // EIGEN to CV
        cv_vec4<T>& vec4(cv_vec4<T>& a, const eigen_vec4<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            a[0] = to_<T>(b[0]);
            a[1] = to_<T>(b[1]);
            a[2] = to_<T>(b[2]);
            a[3] = to_<T>(b[3]);
            return a;
        }
        template<typename T, typename U> inline // CV to EIGEN
        eigen_vec4<T>& vec4(eigen_vec4<T>& a, const cv_vec4<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);
            a[0] = to_<T>(b[0]);
            a[1] = to_<T>(b[1]);
            a[2] = to_<T>(b[2]);
            a[3] = to_<T>(b[3]);
            return a;
        }

    /** Transform3 */

        template<typename T> inline // ROS to EIGEN (full tf)
        eigen_tf3<T>& tf3(eigen_tf3<T>& a, const ros_tf3& b)
        {
            ASSERT_FLOATING_TYPE(T);

            eigen_vec3<T> t;
            eigen_quat<T> r;

            cvt::vec3_(t, b);
            cvt::quat(r, b);

            a = t * r;
            return a;
        }
        template<typename T> inline // EIGEN to ROS (translation)
        ros_tf3& tf3(ros_tf3& a, const eigen_vec3<T>& b)
        {
            cvt::vec3_(a.translation, b);
            return a;
        }
        template<typename T> inline // EIGEN to ROS (rotation)
        ros_tf3& tf3(ros_tf3& a, const eigen_quat<T>& b)
        {
            cvt::quat(a.rotation, b);
            return a;
        }
        template<typename T> inline // EIGEN to ROS (full tf)
        ros_tf3& tf3(ros_tf3& a, const eigen_tf3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);

            eigen_vec3<T> t;
            eigen_quat<T> r;

            t = b.translation();
            r = b.rotation();

            cvt::tf3(a, t);
            cvt::tf3(a, r);

            return a;
        }

    /** Pose */

        template<typename T> inline // ROS to EIGEN (full tf)
        eigen_tf3<T>& pose(eigen_tf3<T>& a, const ros_pose& b)
        {
            ASSERT_FLOATING_TYPE(T);

            eigen_vec3<T> t;
            eigen_quat<T> r;

            cvt::vec3_(t, b);
            cvt::quat(r, b);

            a = t * r;
            return a;
        }
        template<typename T> inline // EIGEN to ROS (translation)
        ros_pose& pose(ros_pose& a, const eigen_vec3<T>& b)
        {
            cvt::vec3_(a.position, b);
            return a;
        }
        template<typename T> inline // EIGEN to ROS (rotation)
        ros_pose& pose(ros_pose& a, const eigen_quat<T>& b)
        {
            cvt::quat(a.orientation, b);
            return a;
        }
        template<typename T> inline // EIGEN to ROS (full tf)
        ros_pose& pose(ros_pose& a, const eigen_tf3<T>& b)
        {
            ASSERT_FLOATING_TYPE(T);

            eigen_vec3<T> t;
            eigen_quat<T> r;

            t = b.translation();
            r = b.rotation();

            cvt::pose(a, t);
            cvt::pose(a, r);

            return a;
        }

        template<typename T, typename U> inline // UTIL (pose) to EIGEN (full tf)
        eigen_tf3<T>& pose(eigen_tf3<T>& a, const util_pose<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);

            if constexpr(std::is_same<T, U>::value)
            {
                a = eigen_trl3<T>{ b.vec } * b.quat;
            }
            else
            {
                eigen_trl3<T> t;
                eigen_quat<T> r;

                cvt::trl3(t, b.vec);
                cvt::quat(r, b.quat);

                a = t * r;
            }
            return a;
        }
        template<typename T, typename U> inline // UTIL (pose) to EIGEN (full tf)
        util_pose<T>& pose(util_pose<T>& a, const eigen_tf3<U>& b)
        {
            ASSERT_FLOATING_TYPE(T);
            ASSERT_FLOATING_TYPE(U);

            if constexpr(std::is_same<T, U>::value)
            {
                a.vec = b.translation();
                a.quat = b.rotation();
            }
            else
            {
                // eigen_trl3<T> t;
                // eigen_quat<T> r;

                // t = b.translation();
                // r = b.rotation();

                // TODO

                // a = t * r;
            }
            return a;
        }

    /** Operator<< wrappers */

        template<typename T> inline // CV to ROS
        ros_vec3& operator<<(ros_vec3& a, const cv_vec3<T>& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_vec3& operator<<(ros_vec3& a, const eigen_vec3<T>& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_point& operator<<(ros_point& a, const eigen_vec3<T>& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_vec3<T>& operator<<(eigen_vec3<T>& a, const ros_vec3& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_vec3<T>& operator<<(eigen_vec3<T>& a, const ros_point& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // ROS to EIGEN (translation from tf)
        eigen_vec3<T>& operator<<(eigen_vec3<T>& a, const ros_tf3& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // ROS to EIGEN (position from pose)
        eigen_vec3<T>& operator<<(eigen_vec3<T>& a, const ros_pose& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T, typename U> inline // CV to EIGEN
        eigen_vec3<T>& operator<<(eigen_vec3<T>& a, const cv_vec3<U>& b)
        {
            return cvt::vec3_(a, b);
        }

        template<typename T> inline // EIGEN (tf) to ROS
        ros_vec3& operator<<(ros_vec3& a, const eigen_tf3<T>& b)
        {
            return cvt::vec3_(a, b);
        }
        template<typename T> inline // EIGEN (tf) to ROS
        ros_point& operator<<(ros_point& a, const eigen_tf3<T>& b)
        {
            return cvt::vec3_(a, b);
        }

        template<typename T, typename U> inline // CV to EIGEN
        eigen_trl3<T>& operator<<(eigen_trl3<T>& a, const cv_vec3<U>& b)
        {
            return cvt::trl3(a, b);
        }
        template<typename T, typename U> inline // EIGEN to EIGEN
        eigen_trl3<T>& operator<<(eigen_trl3<T>& a, const eigen_vec3<U>& b)
        {
            return cvt::trl3(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_trl3<T>& operator<<(eigen_trl3<T>& a, const ros_vec3& b)
        {
            return cvt::trl3(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_trl3<T>& operator<<(eigen_trl3<T>& a, const ros_point& b)
        {
            return cvt::trl3(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_vec3& operator<<(ros_vec3& a, const eigen_trl3<T>& b)
        {
            return cvt::trl3(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_point& operator<<(ros_point& a, const eigen_trl3<T>& b)
        {
            return cvt::trl3(a, b);
        }

        template<typename T> inline // CV to ROS
        ros_quat& operator<<(ros_quat& a, const cv_quat<T>& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T> inline // EIGEN to ROS
        ros_quat& operator<<(ros_quat& a, const eigen_quat<T>& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T> inline // ROS to EIGEN
        eigen_quat<T>& operator<<(eigen_quat<T>& a, const ros_quat& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T> inline // ROS to EIGEN (rotation from tf)
        eigen_quat<T>& operator<<(eigen_quat<T>& a, const ros_tf3& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T> inline // ROS to EIGEN (orientation from pose)
        eigen_quat<T>& operator<<(eigen_quat<T>& a, const ros_pose& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T, typename U> inline // CV to EIGEN
        eigen_quat<T>& operator<<(eigen_quat<T>& a, const cv_quat<U>& b)
        {
            return cvt::quat(a, b);
        }
        template<typename T> inline // EIGEN (tf) to ROS
        ros_quat& operator<<(ros_quat& a, const eigen_tf3<T>& b)
        {
            return cvt::quat(a, b);
        }

        template<typename T, typename U> inline // EIGEN to CV
        cv_vec4<T>& operator<<(cv_vec4<T>& a, const eigen_vec4<U>& b)
        {
            return cvt::vec4(a, b);
        }
        template<typename T, typename U> inline // CV to EIGEN
        eigen_vec4<T>& operator<<(eigen_vec4<T>& a, const cv_vec4<U>& b)
        {
            return cvt::vec4(a, b);
        }

        template<typename T> inline // ROS to EIGEN (full tf)
        eigen_tf3<T>& operator<<(eigen_tf3<T>& a, const ros_tf3& b)
        {
            return cvt::tf3(a, b);
        }
        template<typename T> inline // EIGEN to ROS (translation)
        ros_tf3& operator<<(ros_tf3& a, const eigen_vec3<T>& b)
        {
            return cvt::tf3(a, b);
        }
        template<typename T> inline // EIGEN to ROS (rotation)
        ros_tf3& operator<<(ros_tf3& a, const eigen_quat<T>& b)
        {
            return cvt::tf3(a, b);
        }
        template<typename T> inline // EIGEN to ROS (full tf)
        ros_tf3& operator<<(ros_tf3& a, const eigen_tf3<T>& b)
        {
            return cvt::tf3(a, b);
        }

        template<typename T> inline // ROS to EIGEN (full tf)
        eigen_tf3<T>& operator<<(eigen_tf3<T>& a, const ros_pose& b)
        {
            return cvt::pose(a, b);
        }
        template<typename T> inline // EIGEN to ROS (translation)
        ros_pose& operator<<(ros_pose& a, const eigen_vec3<T>& b)
        {
            return cvt::pose(a, b);
        }
        template<typename T> inline // EIGEN to ROS (rotation)
        ros_pose& operator<<(ros_pose& a, const eigen_quat<T>& b)
        {
            return cvt::pose(a, b);
        }
        template<typename T> inline // EIGEN to ROS (full tf)
        ros_pose& operator<<(ros_pose& a, const eigen_tf3<T>& b)
        {
            return cvt::pose(a, b);
        }

#undef ASSERT_NUMERIC_TYPE
#undef ASSERT_FLOATING_TYPE
    };

};
};
