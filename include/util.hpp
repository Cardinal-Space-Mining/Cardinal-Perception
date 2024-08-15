#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2/time.h"

#include <string>
#include <chrono>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace util
{
    template <typename T>
    struct identity
    {
        typedef T type;
    };

    template <typename T>
    void declare_param(rclcpp::Node * node, const std::string param_name, T & param,
                       const typename identity<T>::type & default_value)
    {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }


    inline tf2::TimePoint toTf2TimePoint(const builtin_interfaces::msg::Time& t)
    {
        return tf2::TimePoint{
            std::chrono::seconds{ t.sec } +
            std::chrono::nanoseconds{ t.nanosec } };
    }
    inline tf2::TimePoint toTf2TimePoint(const rclcpp::Time& t)
    {
        return tf2::TimePoint{ std::chrono::nanoseconds{ t.nanoseconds() } };
    }
    inline tf2::TimePoint toTf2TimePoint(double t_secs)
    {
        return tf2::timeFromSec(t_secs);
    }

    inline double toFloatSeconds(const builtin_interfaces::msg::Time& t)
    {
        return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
    }
    inline double toFloatSeconds(const rclcpp::Time& t)
    {
        return static_cast<double>(t.nanoseconds()) * 1e-9;
    }
    inline double toFloatSeconds(const tf2::TimePoint& t)
    {
        return tf2::timeToSec(t);
    }
    template<typename rep, typename period>
    inline double toFloatSeconds(const std::chrono::duration<rep, period>& dur)
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(dur).count();
    }

    template<typename clock, typename duration>
    inline builtin_interfaces::msg::Time toTimeStamp(const std::chrono::time_point<clock, duration>& t)
    {
        auto _t = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
        return builtin_interfaces::msg::Time{}
            .set__sec(_t / 1000000000)
            .set__nanosec(_t % 1000000000);
    }
    inline builtin_interfaces::msg::Time toTimeStamp(const rclcpp::Time& t)
    {
        return static_cast<builtin_interfaces::msg::Time>(t);
    }
    inline builtin_interfaces::msg::Time toTimeStamp(double t_secs)
    {
        return builtin_interfaces::msg::Time{}
            .set__sec(static_cast<builtin_interfaces::msg::Time::_sec_type>(t_secs))
            .set__nanosec(static_cast<builtin_interfaces::msg::Time::_nanosec_type>(fmod(t_secs, 1.) * 1e9));
    }


    template<typename float_t = double>
    Eigen::Transform<float_t, 3, Eigen::Isometry> lerpSimple(
        const Eigen::Transform<float_t, 3, Eigen::Isometry>& tf,
        float_t alpha)
    {
        static_assert(std::is_floating_point<float_t>::value);

        using Vec3 = Eigen::Vector3<float_t>;
        using Trl3 = Eigen::Translation<float_t, 3>;
        using Quat = Eigen::Quaternion<float_t>;
        using Iso3 = Eigen::Transform<float_t, 3, Eigen::Isometry>;

        Vec3 t = tf.translation();
        Quat q;
        q = tf.rotation();

        return Iso3{ Trl3{ t * alpha } * Quat::Identity().slerp(alpha, q) };
    }

    template<typename float_t = double>
    Eigen::Transform<float_t, 3, Eigen::Isometry> lerpCurvature(
        const Eigen::Transform<float_t, 3, Eigen::Isometry>& tf,
        float_t alpha)
    {
        static_assert(std::is_floating_point<float_t>::value);

        using Vec3 = Eigen::Vector3<float_t>;
        using Trl3 = Eigen::Translation<float_t, 3>;
        using Quat = Eigen::Quaternion<float_t>;
        using Mat3 = Eigen::Matrix3<float_t>;

        // https://github.com/wpilibsuite/allwpilib/blob/79dfdb9dc5e54d4f3e02fb222106c292e85f3859/wpimath/src/main/native/cpp/geometry/Pose3d.cpp#L80-L177

        const Quat qI = Quat::Identity();
        Vec3 u;
        Quat q;
        Mat3 omega = tf.rotation();
        Mat3 omega_sq = omega * omega;
        
        u = tf.translation();
        q = omega;

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

        Vec3 twist_translation = V_inv * u * alpha;
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

        Mat3 R = Mat3::Identity() + A * omega + B * omega_sq;
        Mat3 V = Mat3::Identity() + B * omega + C * omega_sq;
        Quat _q;
        _q = R;
        return Trl3{ V * twist_translation } * _q;
    }
}
