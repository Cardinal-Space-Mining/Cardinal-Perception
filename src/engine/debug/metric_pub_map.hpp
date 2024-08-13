#pragma once

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>


template<typename...>
struct any_same : std::true_type {};
template<typename T, typename t>
struct any_same<T, t> : std::is_same<T, t> {};
template<typename T, typename t, typename... ts>
struct any_same<T, t, ts...> : std::conjunction<std::is_same<T, t>::value, any_same<T, ts...>::value> {};


template<typename Msg_T = std_msgs::msg::Float64>
class MetricPublisherMap
{
static_assert(any_same<Msg_T,
    std_msgs::msg::Bool,
    std_msgs::msg::Byte,
    std_msgs::msg::Char,
    std_msgs::msg::Float32,
    std_msgs::msg::Float64,
    std_msgs::msg::Int8,
    std_msgs::msg::Int16,
    std_msgs::msg::Int32,
    std_msgs::msg::Int64,
    std_msgs::msg::UInt8,
    std_msgs::msg::UInt16,
    std_msgs::msg::UInt32,
    std_msgs::msg::UInt64>::value)
public:

protected:


};
