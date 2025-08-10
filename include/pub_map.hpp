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
#include <memory>
#include <string>
#include <string_view>
#include <shared_mutex>
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

template<typename ros_T, typename primitive_T>
inline ros_T to_ros_val(primitive_T v)
{
    static_assert(std::is_same<typename ros_T::_data_type, primitive_T>::value);

    return ros_T{}.set__data(v);
}


template<typename T, typename MsgT, typename = void>
struct is_convertible_to_std_msg : std::false_type
{
};

template<typename T, typename MsgT>
struct is_convertible_to_std_msg<
    T,
    MsgT,
    std::enable_if_t<
        std::disjunction_v<
            std::is_same<MsgT, std_msgs::msg::Bool>,
            std::is_same<MsgT, std_msgs::msg::Byte>,
            std::is_same<MsgT, std_msgs::msg::Char>,
            std::is_same<MsgT, std_msgs::msg::Int8>,
            std::is_same<MsgT, std_msgs::msg::Int16>,
            std::is_same<MsgT, std_msgs::msg::Int32>,
            std::is_same<MsgT, std_msgs::msg::Int64>,
            std::is_same<MsgT, std_msgs::msg::UInt8>,
            std::is_same<MsgT, std_msgs::msg::UInt16>,
            std::is_same<MsgT, std_msgs::msg::UInt32>,
            std::is_same<MsgT, std_msgs::msg::UInt64>,
            std::is_same<MsgT, std_msgs::msg::Float32>,
            std::is_same<MsgT, std_msgs::msg::Float64>,
            std::is_same<MsgT, std_msgs::msg::String> >,
        void> > : std::is_convertible<T, typename MsgT::_data_type>
{
};



template<typename Msg_T>
class PubMap
{
public:
    using MsgT = Msg_T;
    using SharedPubT = typename rclcpp::Publisher<MsgT>::SharedPtr;

public:
    inline PubMap(
        rclcpp::Node* n,
        std::string_view prefix = "",
        const rclcpp::QoS& qos = rclcpp::SensorDataQoS{}) :
        node{n},
        default_qos{qos},
        prefix{prefix},
        publishers{}
    {
    }
    PubMap(const PubMap&) = delete;
    ~PubMap() = default;

public:
    // wraps the other addPub using the default QoS
    inline SharedPubT addPub(std::string_view topic)
    {
        return this->addPub(topic, this->default_qos);
    }

    // create a publisher and add it to the map
    SharedPubT addPub(std::string_view topic, const rclcpp::QoS& qos)
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
             this->node->template create_publisher<MsgT>(full, qos)});
        if (ptr.second && ptr.first->second)
        {
            return ptr.first->second;
        }
        return nullptr;
    }

    // extract a publisher from its topic
    SharedPubT findPub(std::string_view topic)
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
    SharedPubT getPub(std::string_view topic)
    {
        SharedPubT p = this->findPub(topic);
        if (!p)
        {
            return this->addPub(topic);
        }
        return p;
    }

    // wraps getPub()
    inline SharedPubT operator[](std::string_view topic)
    {
        return this->getPub(topic);
    }

    // publish a value to a topic
    template<typename T>
    void publish(std::string_view topic, T val)
    {
        SharedPubT p = this->getPub(topic);
        if (!p)
        {
            return;
        }

        if constexpr (std::is_same<T, MsgT>::value)
        {
            p->publish(val);
        }
        else if constexpr (std::is_convertible<T, MsgT>::value)
        {
            p->publish(static_cast<MsgT>(val));
        }
        else if constexpr (is_convertible_to_std_msg<T, MsgT>::value)
        {
            p->publish(
                MsgT{}.set__data(static_cast<typename MsgT::_data_type>(val)));
        }
    }

protected:
    rclcpp::Node* node = nullptr;
    rclcpp::QoS default_qos = 1;
    std::string prefix = "";

    std::unordered_map<std::string, SharedPubT> publishers{};
    std::shared_mutex mtx;
    //
};


template<>
class PubMap<void>
{
    template<typename MsgT>
    using Pub = rclcpp::Publisher<MsgT>;
    template<typename MsgT>
    using SharedPub = typename Pub<MsgT>::SharedPtr;

    using SharedVoidPtr = std::shared_ptr<void>;

public:
    inline PubMap(
        rclcpp::Node* n,
        std::string_view prefix = "",
        const rclcpp::QoS& qos = 1) :
        node{n},
        default_qos{qos},
        prefix{prefix},
        pubs{}
    {
    }
    PubMap(const PubMap&) = delete;
    ~PubMap() = default;

public:
    template<typename MsgT>
    inline SharedPub<MsgT> addPub(std::string_view topic)
    {
        return this->addPub<MsgT>(topic, this->default_qos);
    }
    template<typename MsgT>
    inline SharedPub<MsgT> addPub(
        std::string_view topic,
        const rclcpp::QoS& qos)
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

        std::lock_guard lock_{this->mtx};
        auto iter = this->pubs.insert(
            {std::string(topic),
             this->node->template create_publisher<MsgT>(full, qos)});
        if (iter.second && iter.first->second)
        {
            return std::static_pointer_cast<Pub<MsgT>>(iter.first->second);
        }
        return nullptr;
    }

    template<typename MsgT>
    SharedPub<MsgT> findPub(std::string_view topic)
    {
        if (!this->node)
        {
            return nullptr;
        }

        std::shared_lock lock_{this->mtx};
        auto iter = this->pubs.find(std::string(topic));
        lock_.unlock();

        if (iter == this->pubs.end())
        {
            return nullptr;
        }
        else
        {
            return std::static_pointer_cast<Pub<MsgT>>(iter->second);
        }
    }

    template<typename MsgT>
    SharedPub<MsgT> getPub(std::string_view topic)
    {
        SharedPub<MsgT> ptr =
            std::static_pointer_cast<Pub<MsgT>>(this->findPub<MsgT>(topic));
        if (!ptr)
        {
            return this->addPub<MsgT>(topic);
        }
        return ptr;
    }

    template<typename MsgT>
    inline void publish(std::string_view topic, const MsgT& val)
    {
        this->publish<MsgT, MsgT>(topic, val);
    }

    template<typename MsgT, typename T>
    void publish(std::string_view topic, const T& val)
    {
        SharedPub<MsgT> pub = this->getPub<MsgT>(topic);
        if (!pub)
        {
            return;
        }

        if constexpr (std::is_same<T, MsgT>::value)
        {
            pub->publish(val);
            return;
        }
        if constexpr (std::is_convertible<T, MsgT>::value)
        {
            pub->publish(static_cast<MsgT>(val));
            return;
        }
        if constexpr (is_convertible_to_std_msg<T, MsgT>::value)
        {
            pub->publish(
                MsgT{}.set__data(static_cast<typename MsgT::_data_type>(val)));
            return;
        }
    }

protected:
    rclcpp::Node* node = nullptr;
    rclcpp::QoS default_qos = 1;
    std::string prefix = "";

    std::unordered_map<std::string, SharedVoidPtr> pubs;
    std::shared_mutex mtx;
    //
};

using GenericPubMap = PubMap<void>;

};  // namespace util
