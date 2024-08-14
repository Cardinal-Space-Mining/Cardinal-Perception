#pragma once

#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
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


// template<typename...>
// struct any_same : std::true_type {};
// template<typename T, typename t>
// struct any_same<T, t> : std::is_same<T, t> {};
// template<typename T, typename t, typename... ts>
// struct any_same<T, t, ts...> : std::conjunction<std::is_same<T, t>::value, any_same<T, ts...>::value> {};

template<typename T, typename... Ts>
static constexpr bool any_same_v = std::disjunction_v<std::is_same_v<T, Ts>...>;


template<typename Msg_T = std_msgs::msg::Float64>
class MetricPublisherMap_
{
// static_assert(any_same_v<typename Msg_T::Type,
//     std_msgs::msg::Bool::Type,
//     std_msgs::msg::Byte::Type,
//     std_msgs::msg::Char::Type,
//     std_msgs::msg::Int8::Type,
//     std_msgs::msg::Int16::Type,
//     std_msgs::msg::Int32::Type,
//     std_msgs::msg::Int64::Type,
//     std_msgs::msg::UInt8::Type,
//     std_msgs::msg::UInt16::Type,
//     std_msgs::msg::UInt32::Type,
//     std_msgs::msg::UInt64::Type,
//     std_msgs::msg::Float32::Type,
//     std_msgs::msg::Float64::Type>, "Message type must be a valid RCLCPP standard message!");
public:
    using Pub_T = typename rclcpp::Publisher<Msg_T>::SharedPtr;
    using Val_T = typename Msg_T::_data_type;   // all standard msgs have one member of this typename

public:
    inline MetricPublisherMap_(
        rclcpp::Node* n,
        std::string_view prefix = "",
        const rclcpp::QoS& qos = 1)
        : node{ n }, default_qos{ qos }, prefix{ prefix }, publishers{} {}
    MetricPublisherMap_(const MetricPublisherMap_&) = delete;
    ~MetricPublisherMap_() = default;

    // wraps the other addPub using the default QoS
    inline Pub_T addPub(std::string_view topic)
    {
        return this->addPub(topic, this->default_qos);
    }
    // create a publisher and add it to the map
    Pub_T addPub(std::string_view topic, const rclcpp::QoS& qos)
    {
        if(!this->node) return nullptr;

        std::string full;
        if(this->prefix.empty()) full = topic;
        else
        {
        #if __cpp_lib_string_view >= 202403 // from cppreference
            full = this->prefix + topic;
        #else
            full = this->prefix + std::string{ topic };
        #endif
        }

        try
        {
            std::unique_lock _lock{ this->mtx };
            auto ptr = this->publishers.insert({ std::string{ topic }, this->node->create_publisher<Msg_T>(full, qos) });
            if(ptr.second && ptr.first->second) return ptr.first->second;
        }
        catch(...) {}
        return nullptr;
    }
    // extract a publisher from its topic
    Pub_T findPub(std::string_view topic)
    {
        if(!this->node) return nullptr;

        std::shared_lock _lock{ this->mtx };
        auto search = this->publishers.find(std::string{ topic });
        _lock.unlock();

        if(search == this->publishers.end()) return nullptr;
        else return search->second;
    }
    // extract or add if not already present
    Pub_T getPub(std::string_view topic)
    {
        Pub_T p = this->findPub(topic);
        if(!p) return this->addPub(topic);
        return p;
    }

    // wraps getPub()
    inline Pub_T operator[](std::string_view topic)
    {
        return this->getPub(topic);
    }

    // publish a value to a topic
    template<typename T>
    void publish(std::string_view topic, T val)
    {
        static_assert(std::is_convertible<T, Val_T>::value);

        Pub_T p = this->getPub(topic);
        if(p) p->publish( Msg_T{}.set__data(static_cast<Val_T>(val)) );
    }

protected:
    rclcpp::Node* node = nullptr;
    rclcpp::QoS default_qos = 1;
    std::string prefix = "";
    std::unordered_map<std::string, Pub_T> publishers{};
    std::shared_mutex mtx;

};

using MetricPublisherMap = MetricPublisherMap_<>;
using FloatPublisherMap = MetricPublisherMap_<std_msgs::msg::Float64>;
using IntPublisherMap = MetricPublisherMap_<std_msgs::msg::Int64>;
using UintPublisherMap = MetricPublisherMap_<std_msgs::msg::UInt64>;
using BoolPublisherMap = MetricPublisherMap_<std_msgs::msg::Bool>;
