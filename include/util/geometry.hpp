/*******************************************************************************
*   Copyright (C) 2024-2026 Cardinal Space Mining Club                         *
*                                                                              *
*                                 ;xxxxxxx:                                    *
*                                ;$$$$$$$$$       ...::..                      *
*                                $$$$$$$$$$x   .:::::::::::..                  *
*                             x$$$$$$$$$$$$$$::::::::::::::::.                 *
*                         :$$$$$&X;      .xX:::::::::::::.::...                *
*                 .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :               *
*                :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.              *
*               :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.              *
*              ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::               *
*               X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.               *
*                .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                *
*                 X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                  *
*                 $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                    *
*                 $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                    *
*                 $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                    *
*                 X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                     *
*                 $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                    *
*               x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                   *
*              +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                  *
*               +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                   *
*                :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                    *
*                ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                     *
*               ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                            *
*               ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                               *
*               :;;;;;;;;;;;;.  :$$$$$$$$$$X                                   *
*                .;;;;;;;;:;;    +$$$$$$$$$                                    *
*                  .;;;;;;.       X$$$$$$$:                                    *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*******************************************************************************/

#pragma once

#include <type_traits>

// #ifndef GEOM_UTIL_USE_EIGEN  // eigen required
//     #define GEOM_UTIL_USE_EIGEN 1
// #endif
#ifndef GEOM_UTIL_USE_OPENCV
    #define GEOM_UTIL_USE_OPENCV 0
#endif
#ifndef GEOM_UTIL_USE_ROS
    #define GEOM_UTIL_USE_ROS 1
#endif
#ifndef GEOM_UTIL_USE_GTSAM
    #define GEOM_UTIL_USE_GTSAM 0
#endif

// #if GEOM_UTIL_USE_EIGEN
#include <Eigen/Core>
#include <Eigen/Geometry>
// #endif
#if GEOM_UTIL_USE_OPENCV
    #include <opencv2/core/types.hpp>
    #include <opencv2/core/quaternion.hpp>
#endif
#if GEOM_UTIL_USE_ROS
    #include <geometry_msgs/msg/vector3.hpp>
    #include <geometry_msgs/msg/point.hpp>
    #include <geometry_msgs/msg/quaternion.hpp>
    #include <geometry_msgs/msg/transform.hpp>
    #include <geometry_msgs/msg/pose.hpp>
#endif
#if GEOM_UTIL_USE_GTSAM
    #include <gtsam/geometry/Rot3.h>
    #include <gtsam/geometry/Pose3.h>
    #include <gtsam/geometry/Point3.h>
#endif


// clang-format off

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
        using Scalar_T = T;
        using Vec_T = Eigen::Vector3<T>;
        using Quat_T = Eigen::Quaternion<T>;
        using Trl_T = Eigen::Translation<T, 3>;

        Vec_T vec;
        Quat_T quat;

        inline Trl_T vec_trl() const { return Trl_T{ this->vec }; }
        inline Trl_T vec_ntrl() const { return Trl_T{ -this->vec }; }

        // template<bool Init_Zero = true>
        // inline Pose3()
        // {
        //     if constexpr(Init_Zero)
        //     {
        //         this->vec.setZero();
        //         this->quat.setIdentity();
        //     }
        // }
    };
    using Pose3f = Pose3<float>;
    using Pose3d = Pose3<double>;

    template<typename T>
    struct PoseTf3
    {
        ASSERT_FLOATING_TYPE(T);
        using Scalar_T = T;
        using Vec_T = typename Pose3<T>::Vec_T;
        using Quat_T = typename Pose3<T>::Quat_T;
        using Tf_T = Eigen::Transform<T, 3, Eigen::Isometry>;

        Pose3<T> pose;
        Tf_T tf;

        // template<bool Init_Zero = true>
        // inline PoseTf3()
        // {
        //     if constexpr(Init_Zero)
        //     {
        //         this->pose.vec.setZero();
        //         this->pose.quat.setIdentity();
        //         this->tf = Tf_T::Identity();
        //     }
        // }
    };
    using PoseTf3f = PoseTf3<float>;
    using PoseTf3d = PoseTf3<double>;

    template<typename T> using util_pose = util::geom::Pose3<T>;
    template<typename T> using util_posetf = util::geom::PoseTf3<T>;

    template<typename T> using eigen_vec3 = Eigen::Vector3<T>;
    template<typename T> using eigen_trl3 = Eigen::Translation<T, 3>;
    template<typename T> using eigen_vec4 = Eigen::Vector4<T>;
    template<typename T> using eigen_quat = Eigen::Quaternion<T>;
    template<typename T> using eigen_tf3 = Eigen::Transform<T, 3, Eigen::Isometry>;

#if GEOM_UTIL_USE_OPENCV
    template<typename T> using cv_vec3 = cv::Vec<T, 3>;
    template<typename T> using cv_point = cv::Point3_<T>;
    template<typename T> using cv_vec4 = cv::Vec<T, 4>;
    template<typename T> using cv_quat = cv::Quat<T>;
#endif
#if GEOM_UTIL_USE_ROS
    using ros_vec3 = geometry_msgs::msg::Vector3;
    using ros_point = geometry_msgs::msg::Point;
    using ros_quat = geometry_msgs::msg::Quaternion;
    using ros_tf3 = geometry_msgs::msg::Transform;
    using ros_pose = geometry_msgs::msg::Pose;
#endif
#if GEOM_UTIL_USE_GTSAM
    using gtsam_vec3 = gtsam::Vector3;
    using gtsam_quat = gtsam::Quaternion;
    using gtsam_pose = gtsam::Pose3;
#endif

    namespace cvt
    {
        #define MAKE_ACCESSORS(fname, param, opr) \
            inline static auto& fname(param& v) { return v.opr; } \
            inline static auto fname(const param& v) { return v.opr; }
        #define MAKE_TEMPLATE_ACCESSORS(fname, param, opr) \
            template<typename T> inline static auto& fname(param<T>& v) { return v.opr; } \
            template<typename T> inline static auto fname(const param<T>& v) { return v.opr; }

        template<typename T, typename U>
        inline void castOrAssign(T& x, const U& y)
        {
            static_assert(std::is_convertible<T, U>::value);

            if constexpr(std::is_same<T, U>::value) x = y;
            else                                    x = static_cast<T>(y);
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
            #if GEOM_UTIL_USE_OPENCV
                // cv::Vec<T, 3>
                MAKE_TEMPLATE_ACCESSORS(x, cv_vec3, operator[](0))
                MAKE_TEMPLATE_ACCESSORS(y, cv_vec3, operator[](1))
                MAKE_TEMPLATE_ACCESSORS(z, cv_vec3, operator[](2))
                // cv::Point3_<T>
                MAKE_TEMPLATE_ACCESSORS(x, cv_point, x)
                MAKE_TEMPLATE_ACCESSORS(y, cv_point, y)
                MAKE_TEMPLATE_ACCESSORS(z, cv_point, z)
            #endif
            #if GEOM_UTIL_USE_ROS
                // geometry_msgs::msg::Vector3
                MAKE_ACCESSORS(x, ros_vec3, x)
                MAKE_ACCESSORS(y, ros_vec3, y)
                MAKE_ACCESSORS(z, ros_vec3, z)
                // geometry_msgs::msg::Point
                MAKE_ACCESSORS(x, ros_point, x)
                MAKE_ACCESSORS(y, ros_point, y)
                MAKE_ACCESSORS(z, ros_point, z)
            #endif
            #if GEOM_UTIL_USE_GTSAM
                // gtsam::Vector3 & gtsam::Point3 (aliases)
                MAKE_ACCESSORS(x, gtsam_vec3, operator[](0))
                MAKE_ACCESSORS(y, gtsam_vec3, operator[](1))
                MAKE_ACCESSORS(z, gtsam_vec3, operator[](2))
            #endif
            };

            template<
                typename A,
                typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::vec3::traits;
                cvt::castOrAssign(x(a), x(b));
                cvt::castOrAssign(y(a), y(b));
                cvt::castOrAssign(z(a), z(b));
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
            #if GEOM_UTIL_USE_OPENCV
                // cv::Quat<T>
                MAKE_TEMPLATE_ACCESSORS(w, cv_quat, w)
                MAKE_TEMPLATE_ACCESSORS(x, cv_quat, x)
                MAKE_TEMPLATE_ACCESSORS(y, cv_quat, y)
                MAKE_TEMPLATE_ACCESSORS(z, cv_quat, z)
            #endif
            #if GEOM_UTIL_USE_ROS
                // geometry_msgs::msg::Quaternion
                MAKE_ACCESSORS(w, ros_quat, w)
                MAKE_ACCESSORS(x, ros_quat, x)
                MAKE_ACCESSORS(y, ros_quat, y)
                MAKE_ACCESSORS(z, ros_quat, z)
            #endif
            #if GEOM_UTIL_USE_GTSAM
                // gtsam::Quaternion
                MAKE_ACCESSORS(w, gtsam_quat, w())
                MAKE_ACCESSORS(x, gtsam_quat, x())
                MAKE_ACCESSORS(y, gtsam_quat, y())
                MAKE_ACCESSORS(z, gtsam_quat, z())
            #endif
            };

            template<
                typename A,
                typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::quat::traits;
                cvt::castOrAssign(w(a), w(b));
                cvt::castOrAssign(x(a), x(b));
                cvt::castOrAssign(y(a), y(b));
                cvt::castOrAssign(z(a), z(b));
                return a;
            }
        };

        namespace vec4
        {
        #if GEOM_UTIL_USE_OPENCV
            template<typename T, typename U> inline // EIGEN to CV
            cv_vec4<T>& cvt(cv_vec4<T>& a, const eigen_vec4<U>& b)
            {
                cvt::castOrAssign(a[0], b[0]);
                cvt::castOrAssign(a[1], b[1]);
                cvt::castOrAssign(a[2], b[2]);
                cvt::castOrAssign(a[3], b[3]);
                return a;
            }
            template<typename T, typename U> inline // CV to EIGEN
            eigen_vec4<T>& cvt(eigen_vec4<T>& a, const cv_vec4<U>& b)
            {
                cvt::castOrAssign(a[0], b[0]);
                cvt::castOrAssign(a[1], b[1]);
                cvt::castOrAssign(a[2], b[2]);
                cvt::castOrAssign(a[3], b[3]);
                return a;
            }
        #endif
        };

        namespace vecN
        {
        #if GEOM_UTIL_USE_OPENCV
            template<typename T, typename U, std::size_t N> inline
            Eigen::Matrix<T, N, 1>& cvt(Eigen::Matrix<T, N, 1>& a, const cv::Vec<U, N>& b)
            {
                for(std::size_t i = 0; i < N; i++)
                {
                    cvt::castOrAssign(a[i], b[i]);
                }
                return a;
            }
            template<typename T, typename U, std::size_t N> inline
            cv::Vec<U, N>& cvt(cv::Vec<U, N>& a, const Eigen::Matrix<T, N, 1>& b)
            {
                for(std::size_t i = 0; i < N; i++)
                {
                    cvt::castOrAssign(a[i], b[i]);
                }
                return a;
            }
        #endif
        };

        namespace pose
        {
            namespace traits
            {
            #if GEOM_UTIL_USE_ROS
                // geometry_msgs::msg::Transform
                MAKE_ACCESSORS(vec_, ros_tf3, translation)
                MAKE_ACCESSORS(quat_, ros_tf3, rotation)
                // geometry_msgs::msg::Pose
                MAKE_ACCESSORS(vec_, ros_pose, position)
                MAKE_ACCESSORS(quat_, ros_pose, orientation)
            #endif
                // util::geom::Pose3<T>
                MAKE_TEMPLATE_ACCESSORS(vec_, util_pose, vec)
                MAKE_TEMPLATE_ACCESSORS(quat_, util_pose, quat)
            };

            // generic ---------------------------------------------------------
            template<typename A, typename B>
            inline A& cvt(A& a, const B& b)
            {
                using namespace util::geom::cvt::pose::traits;
                vec3::cvt(vec_(a), vec_(b));
                quat::cvt(quat_(a), quat_(b));
                return a;
            }
            template<typename T>
            inline T& cvt(T& a, const T& b)
            {
                a = b;
                return a;
            }

            // eigen isometry --------------------------------------------------
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

        #if GEOM_UTIL_USE_GTSAM
            // gtsam -----------------------------------------------------------
            template<typename B>
            inline gtsam_pose& cvt(gtsam_pose& a, const B& b)
            {
                using namespace util::geom::cvt::pose::traits;
                gtsam_vec3 t;
                gtsam_quat r;
                vec3::cvt(t, vec_(b));
                quat::cvt(r, quat_(b));
                a = gtsam::Pose3(gtsam::Rot3(r), t);
                return a;
            }
            template<typename A>
            inline A& cvt(A& a, const gtsam_pose& b)
            {
                using namespace util::geom::cvt::pose::traits;
                vec3::cvt(vec_(a), b.translation());
                quat::cvt(quat_(a), b.rotation().toQuaternion());
                return a;
            }

            // gtsam and eigen isometry ----------------------------------------
            template<typename T>
            inline eigen_tf3<T>& cvt(eigen_tf3<T>& a, const gtsam_pose& b)
            {
                using namespace util::geom::cvt::pose::traits;
                eigen_trl3<T> t;
                eigen_quat<T> r;
                vec3::cvt(t, b.translation());
                quat::cvt(r, b.rotation().toQuaternion());
                a = t * r;
                return a;
            }
            template<typename T>
            inline gtsam_pose& cvt(gtsam_pose& a, const eigen_tf3<T>& b)
            {
                using namespace util::geom::cvt::pose::traits;
                gtsam_vec3 t;
                gtsam_quat r;
                eigen_vec3<T> et;
                eigen_quat<T> er;
                et = b.translation();
                er = b.rotation();
                vec3::cvt(t, et);
                quat::cvt(r, er);
                a = gtsam::Pose3(gtsam::Rot3(r), t);
                return a;
            }
        #endif

            // PoseTf3 ---------------------------------------------------------
            template<typename F1, typename F2>
            inline util_posetf<F1>& cvt(util_posetf<F1>& a, const util_posetf<F2>& b)
            {
                cvt(a.pose, b.pose);
                a.tf = b.tf.template cast<F1>();
                return a;
            }
            template<typename F>
            inline util_posetf<F>& cvt(util_posetf<F>& a, const util_posetf<F>& b)
            {
                a = b;
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
            #define MAKE_TEMPLATED_CASTING_OPS(type, t) \
                template<typename T, typename U> inline t<T>& operator<<(t<T>& a, const t<U>& b) { return type::cvt(a, b); } \
                template<typename T, typename U> inline t<U>& operator>>(const t<T>& a, t<U>& b) { return type::cvt(b, a); }


            // vec3 ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, eigen_trl3)
        #if GEOM_UTIL_USE_OPENCV
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, cv_vec3)
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_vec3, cv_point)
        #endif
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_vec3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_vec3, ros_point)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_vec3, gtsam_vec3)
        #endif

        #if GEOM_UTIL_USE_OPENCV
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_trl3, cv_vec3)
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, eigen_trl3, cv_point)
        #endif
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_trl3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_trl3, ros_point)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, eigen_trl3, gtsam_vec3)
        #endif

    #if GEOM_UTIL_USE_OPENCV
            MAKE_DOUBLE_TEMPLATED_OPS( vec3, cv_vec3, cv_point)
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_vec3, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_vec3, ros_point)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_vec3, gtsam_vec3)
        #endif

        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_point, ros_vec3)
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_point, ros_point)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(vec3, cv_point, gtsam_vec3)
        #endif
    #endif

    #if GEOM_UTIL_USE_ROS
            MAKE_STATIC_OPS(           vec3, ros_vec3, ros_point)
        #if GEOM_UTIL_USE_GTSAM
            MAKE_STATIC_OPS(           vec3, ros_vec3, gtsam_vec3)

            MAKE_STATIC_OPS(           vec3, ros_point, gtsam_vec3)
        #endif
    #endif

            // quat ------------------------------------------------
        #if GEOM_UTIL_USE_OPENCV
            MAKE_DOUBLE_TEMPLATED_OPS( quat, eigen_quat, cv_quat)
        #endif
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(quat, eigen_quat, ros_quat)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(quat, eigen_quat, gtsam_quat)
        #endif

    #if GEOM_UTIL_USE_OPENCV
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(quat, cv_quat, ros_quat)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(quat, cv_quat, gtsam_quat)
        #endif
    #endif

        #if GEOM_UTIL_USE_ROS & GEOM_UTIL_USE_GTSAM
            MAKE_STATIC_OPS(           quat, ros_quat, gtsam_quat)
        #endif

        #if GEOM_UTIL_USE_OPENCV
            // vec4 ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( vec4, eigen_vec4, cv_vec4)
        #endif

            // pose ------------------------------------------------
            MAKE_DOUBLE_TEMPLATED_OPS( pose, eigen_tf3, util_pose)
        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(pose, eigen_tf3, ros_tf3)
            MAKE_PRIMARY_TEMPLATED_OPS(pose, eigen_tf3, ros_pose)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(pose, eigen_tf3, gtsam_pose)
        #endif

        #if GEOM_UTIL_USE_ROS
            MAKE_PRIMARY_TEMPLATED_OPS(pose, util_pose, ros_tf3)
            MAKE_PRIMARY_TEMPLATED_OPS(pose, util_pose, ros_pose)
        #endif
        #if GEOM_UTIL_USE_GTSAM
            MAKE_PRIMARY_TEMPLATED_OPS(pose, util_pose, gtsam_pose)
        #endif

    #if GEOM_UTIL_USE_ROS
            MAKE_STATIC_OPS(           pose, ros_tf3, ros_pose)
        #if GEOM_UTIL_USE_GTSAM
            MAKE_STATIC_OPS(           pose, ros_tf3, gtsam_pose)
        #endif
    #endif

        #if GEOM_UTIL_USE_ROS & GEOM_UTIL_USE_GTSAM
            MAKE_STATIC_OPS(           pose, ros_pose, gtsam_pose)
        #endif

            // util pose casting -----------------------------------
            MAKE_TEMPLATED_CASTING_OPS(pose, util_pose)
            MAKE_TEMPLATED_CASTING_OPS(pose, util_posetf)


            #undef MAKE_DOUBLE_TEMPLATED_OPS
            #undef MAKE_PRIMARY_TEMPLATED_OPS
            #undef MAKE_STATIC_OPS
            #undef MAKE_TEMPLATED_CASTING_OPS
        };
    };


// pose component ops

    /** Simple difference between pose components. THIS IS NOT THE RELATIVE DIFFERENCE (see relativeDiff()) */
    template<typename T> inline
    void componentDiff(
        geom::Pose3<T>& diff,
        const geom::Pose3<T>& from,
        const geom::Pose3<T>& to)
    {
        diff.vec = to.vec - from.vec;
        diff.quat = from.quat.inverse() * to.quat;
    }

    /** Get the difference in poses relative to the "from" coordinate frame (manifold difference) */
    template<typename T> inline
    void relativeDiff(
        geom::Pose3<T>& diff,
        const geom::Pose3<T>& from,
        const geom::Pose3<T>& to)
    {
        componentDiff<T>(diff, from, to);
        diff.vec = from.quat.inverse()._transformVector(diff.vec);
    }
    /** Append poses that are relative to each other (resulting pose is in the external frame of reference) */
    template<typename T> inline
    void compose(
        geom::Pose3<T>& out,
        const geom::Pose3<T>& base,
        const geom::Pose3<T>& relative)
    {
        out.vec = base.vec + base.quat._transformVector(relative.vec);
        out.quat = relative.quat * base.quat;
    }

    template<typename T> inline
    void inverse(
        geom::Pose3<T>& out,
        const geom::Pose3<T>& in)
    {
        out.quat = in.quat.inverse();
        out.vec = out.quat._transformVector(-in.vec);
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
        interp.quat = geom::eigen_quat<float_T>::Identity().slerp(alpha, q);
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
