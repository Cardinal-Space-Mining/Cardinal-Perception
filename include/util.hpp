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
};
