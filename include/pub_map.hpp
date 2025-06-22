/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
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
#include <std_msgs/msg/string.hpp>


namespace util
{

template<typename ros_T, typename primitive_T>
inline ros_T& to_ros(primitive_T& v)
{
    static_assert(
        std::is_same<typename ros_T::_data_type, primitive_T>::value &&
        sizeof(ros_T) == sizeof(primitive_T) &&
        alignof(ros_T) == alignof(primitive_T));

    return reinterpret_cast<ros_T&>(v);
}

inline std_msgs::msg::Bool& to_ros(bool& v)
{
    return reinterpret_cast<std_msgs::msg::Bool&>(v);
}
inline std_msgs::msg::Int64& to_ros(int64_t& v)
{
    return reinterpret_cast<std_msgs::msg::Int64&>(v);
}
inline std_msgs::msg::Float64& to_ros(double& v)
{
    return reinterpret_cast<std_msgs::msg::Float64&>(v);
}

template<typename ros_T, typename primitive_T>
inline ros_T to_ros_val(primitive_T v)
{
    static_assert(std::is_same<typename ros_T::_data_type, primitive_T>::value);

    return ros_T{}.set__data(v);
}

inline std_msgs::msg::Bool to_ros_val(bool v)
{
    return std_msgs::msg::Bool{}.set__data(v);
}
inline std_msgs::msg::Int64 to_ros_val(int64_t v)
{
    return std_msgs::msg::Int64{}.set__data(v);
}
inline std_msgs::msg::Float64 to_ros_val(double v)
{
    return std_msgs::msg::Float64{}.set__data(v);
}


// template<typename...>
// struct any_same : std::true_type {};
// template<typename T, typename t>
// struct any_same<T, t> : std::is_same<T, t> {};
// template<typename T, typename t, typename... ts>
// struct any_same<T, t, ts...> :
//     std::conjunction<std::is_same<T, t>::value, any_same<T, ts...>::value>{};

// template<typename T, typename... Ts>
// static constexpr bool any_same_v =
//  std::disjunction_v<std::is_same_v<T, Ts>...>;


template<typename Msg_T>
class PublisherMap
{
private:
    // template<typename X>
    // struct is_std_msg
    // {
    // private:
    //     struct _True
    //     {
    //         char v[1];
    //     };
    //     struct _False
    //     {
    //         char v[2];
    //     };

    //     template<typename Y>
    //     static _True test(typename X::_data_type*);
    //     template<typename Y>
    //     static _False test(...);

    // public:
    //     static const bool value = sizeof(test<X>(nullptr)) == sizeof(_True);
    // };

public:
    using Pub_T = typename rclcpp::Publisher<Msg_T>::SharedPtr;

    static constexpr bool Is_Std_Msg = std::disjunction_v<
        std::is_same<Msg_T, std_msgs::msg::Bool>,
        std::is_same<Msg_T, std_msgs::msg::Byte>,
        std::is_same<Msg_T, std_msgs::msg::Char>,
        std::is_same<Msg_T, std_msgs::msg::Int8>,
        std::is_same<Msg_T, std_msgs::msg::Int16>,
        std::is_same<Msg_T, std_msgs::msg::Int32>,
        std::is_same<Msg_T, std_msgs::msg::Int64>,
        std::is_same<Msg_T, std_msgs::msg::UInt8>,
        std::is_same<Msg_T, std_msgs::msg::UInt16>,
        std::is_same<Msg_T, std_msgs::msg::UInt32>,
        std::is_same<Msg_T, std_msgs::msg::UInt64>,
        std::is_same<Msg_T, std_msgs::msg::Float32>,
        std::is_same<Msg_T, std_msgs::msg::Float64>,
        std::is_same<Msg_T, std_msgs::msg::String> >;
    // using Val_T = typename std::conditional<
    //     Is_Std_Msg,
    //     typename Msg_T::_data_type,
    //     void>::type;  // all standard msgs have one member of this typename

public:
    inline PublisherMap(
        rclcpp::Node* n,
        std::string_view prefix = "",
        const rclcpp::QoS& qos = rclcpp::SensorDataQoS{}) :
        node{n},
        default_qos{qos},
        prefix{prefix},
        publishers{}
    {
    }
    PublisherMap(const PublisherMap&) = delete;
    ~PublisherMap() = default;

public:
    // wraps the other addPub using the default QoS
    inline Pub_T addPub(std::string_view topic)
    {
        return this->addPub(topic, this->default_qos);
    }

    // create a publisher and add it to the map
    Pub_T addPub(std::string_view topic, const rclcpp::QoS& qos)
    {
        if (!this->node)
        {
            return nullptr;
        }

        std::string full;
        if (this->prefix.empty())
        {
            full = topic;
        }
        else
        {
#if __cpp_lib_string_view >= 202403  // from cppreference
            full = this->prefix + topic;
#else
            full = this->prefix + std::string{topic};
#endif
        }

        std::lock_guard _lock{this->mtx};
        auto ptr = this->publishers.insert(
            {std::string{topic},
             this->node->template create_publisher<Msg_T>(full, qos)});
        if (ptr.second && ptr.first->second)
        {
            return ptr.first->second;
        }
        return nullptr;
    }

    // extract a publisher from its topic
    Pub_T findPub(std::string_view topic)
    {
        if (!this->node)
        {
            return nullptr;
        }

        std::shared_lock _lock{this->mtx};
        auto search = this->publishers.find(std::string{topic});
        _lock.unlock();

        if (search == this->publishers.end())
        {
            return nullptr;
        }
        else
        {
            return search->second;
        }
    }

    // extract or add if not already present
    Pub_T getPub(std::string_view topic)
    {
        Pub_T p = this->findPub(topic);
        if (!p)
        {
            return this->addPub(topic);
        }
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
        Pub_T p = this->getPub(topic);
        if (!p)
        {
            return;
        }

        if constexpr (std::is_same<T, Msg_T>::value)
        {
            p->publish(val);
        }
        else if constexpr (std::is_convertible<T, Msg_T>::value)
        {
            p->publish(static_cast<Msg_T>(val));
        }
        else if constexpr (
            Is_Std_Msg &&
            std::is_convertible<T, typename Msg_T::_data_type>::value)
        {
            p->publish(
                Msg_T{}.set__data(
                    static_cast<typename Msg_T::_data_type>(val)));
        }
    }

protected:
    rclcpp::Node* node = nullptr;
    rclcpp::QoS default_qos = 1;
    std::string prefix = "";
    std::unordered_map<std::string, Pub_T> publishers{};
    std::shared_mutex mtx;
};

using FloatPublisherMap = PublisherMap<std_msgs::msg::Float64>;
using IntPublisherMap = PublisherMap<std_msgs::msg::Int64>;
using UintPublisherMap = PublisherMap<std_msgs::msg::UInt64>;
using BoolPublisherMap = PublisherMap<std_msgs::msg::Bool>;

};  // namespace util
