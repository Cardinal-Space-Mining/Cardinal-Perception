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

#include "std_utils.hpp"


namespace util
{

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
            std::is_same<MsgT, std_msgs::msg::String>>,
        void>> : std::is_convertible<T, typename MsgT::_data_type>
{
};


class PubMapBase
{
public:
    using RclNode = rclcpp::Node;
    using RclQoS = rclcpp::QoS;

    template<typename MsgT>
    using Pub = rclcpp::Publisher<MsgT>;
    template<typename MsgT>
    using SharedPub = typename Pub<MsgT>::SharedPtr;

    template<typename T>
    using StringMap = std::unordered_map<
        std::string,
        T,
        util::TransparentStringHash,
        util::TransparentStringEq>;

protected:
    inline PubMapBase(RclNode& n, std::string_view prefix, const RclQoS& qos) :
        node{n},
        default_qos{qos},
        prefix{
            // empty prefix should be left alone because in some cases it is nice to exploit namespace-local topics
            prefix.empty() || prefix.back() == '/' ? prefix
                                                   : std::string(prefix) + "/"}
    {
    }

protected:
    inline std::string formatFullTopic(std::string_view topic)
    {
        // ASSERT TOPIC IS NOT EMPTY!?
        return this->prefix +
               std::string(topic.front() == '/' ? topic.substr(1) : topic);
    }

protected:
    RclNode& node;
    RclQoS default_qos;
    std::string prefix;
};



template<typename Msg_T>
class PubMap : public PubMapBase
{
public:
    using MsgT = Msg_T;
    using SharedPubT = SharedPub<MsgT>;

public:
    /* Construct a publisher map.
     * Prefix may be relative or absolute and gets appended to topics in all method calls. */
    inline PubMap(
        RclNode& n,
        std::string_view prefix = "",
        const RclQoS& qos = rclcpp::SensorDataQoS{}) :
        PubMapBase{n, prefix, qos},
        publishers{}
    {
    }
    PubMap(const PubMap&) = delete;
    ~PubMap() = default;

public:
    /* Add a publisher on the provided topic, using the default QoS */
    inline SharedPubT addPub(std::string_view topic)
    {
        return this->addPub(topic, this->default_qos);
    }

    /* Add a publisher on the provided topic, using the provided QoS.
     * Does not recreate publisher if the topic is already used. */
    SharedPubT addPub(std::string_view topic, const RclQoS& qos)
    {
        std::lock_guard lock{this->mtx};

        auto it = this->publishers.find(topic);
        if (it != this->publishers.end())
        {
            return it->second;
        }

        auto ptr = this->publishers.emplace(
            topic,
            this->node.template create_publisher<MsgT>(
                this->formatFullTopic(topic),
                qos));
        if (ptr.second && ptr.first->second)
        {
            return ptr.first->second;
        }

        return nullptr;
    }

    /* Find and return a publisher if it exists for the provided topic,
     * otherwise returns nullptr. */
    SharedPubT findPub(std::string_view topic)
    {
        std::shared_lock lock{this->mtx};

        auto search = this->publishers.find(topic);
        if (search == this->publishers.end())
        {
            return nullptr;
        }
        else
        {
            return search->second;
        }
    }

    /* Get the publisher for the provided topic if it exists,
     * otherwise creates a new one with the default QoS. */
    SharedPubT getPub(std::string_view topic)
    {
        SharedPubT p = this->findPub(topic);
        if (!p)
        {
            return this->addPub(topic);
        }
        return p;
    }

    /* Operator wrapper for getPub() */
    inline SharedPubT operator[](std::string_view topic)
    {
        return this->getPub(topic);
    }

    /* Publish a value on the provided topic, which is created if not already present. */
    template<typename T>
    void publish(std::string_view topic, T val)
    {
        SharedPubT p = this->getPub(topic);
        if (!p)
        {
            return;
        }

        if constexpr (std::is_same_v<T, MsgT>)
        {
            p->publish(val);
        }
        else if constexpr (std::is_convertible_v<T, MsgT>)
        {
            p->publish(static_cast<MsgT>(val));
        }
        else if constexpr (is_convertible_to_std_msg<T, MsgT>::value)
        {
            p->publish(
                MsgT{}.set__data(static_cast<typename MsgT::_data_type>(val)));
        }
        else
        {
            static_assert(
                std::is_same_v<T, MsgT> ||
                std::is_convertible_v<T, MsgT> ||
                is_convertible_to_std_msg<T, MsgT>::value, "Error: incompatible types!");
        }
    }

protected:
    StringMap<SharedPubT> publishers{};
    std::shared_mutex mtx;
};


template<>
class PubMap<void> : public PubMapBase
{
    using SharedBasePtr = std::shared_ptr<rclcpp::PublisherBase>;

public:
    /* Construct a publisher map.
     * Prefix may be relative or absolute and gets appended to topics in all method calls. */
    inline PubMap(
        RclNode& n,
        std::string_view prefix = "",
        const RclQoS& qos = 1) :
        PubMapBase{n, prefix, qos},
        publishers{}
    {
    }
    PubMap(const PubMap&) = delete;
    ~PubMap() = default;

public:
    /* Add a publisher on the provided topic, using the default QoS */
    template<typename MsgT>
    inline SharedPub<MsgT> addPub(std::string_view topic)
    {
        return this->addPub<MsgT>(topic, this->default_qos);
    }

    /* Add a publisher on the provided topic, using the provided QoS.
     * Does not recreate publisher if the topic is already used. */
    template<typename MsgT>
    inline SharedPub<MsgT> addPub(std::string_view topic, const RclQoS& qos)
    {
        std::lock_guard lock{this->mtx};

        // Avoid additional calls to create_publisher
        auto it = this->publishers.find(topic);
        if (it != this->publishers.end())
        {
            return std::dynamic_pointer_cast<Pub<MsgT>>(it->second);
        }

        auto iter = this->publishers.emplace(
            topic,
            this->node.template create_publisher<MsgT>(
                this->formatFullTopic(topic),
                qos));
        if (iter.second && iter.first->second)
        {
            return std::dynamic_pointer_cast<Pub<MsgT>>(iter.first->second);
        }

        return nullptr;
    }

    /* Find and return a publisher if it exists for the provided topic,
     * otherwise returns nullptr. */
    template<typename MsgT>
    SharedPub<MsgT> findPub(std::string_view topic)
    {
        std::shared_lock lock{this->mtx};

        auto iter = this->publishers.find(topic);
        if (iter == this->publishers.end())
        {
            return nullptr;
        }
        else
        {
            return std::dynamic_pointer_cast<Pub<MsgT>>(iter->second);
        }
    }

    /* Get the publisher for the provided topic if it exists,
     * otherwise creates a new one with the default QoS. */
    template<typename MsgT>
    SharedPub<MsgT> getPub(std::string_view topic)
    {
        SharedPub<MsgT> ptr = this->findPub<MsgT>(topic);
        if (!ptr)
        {
            return this->addPub<MsgT>(topic);
        }
        return ptr;
    }

    /* Publish a value on the provided topic, which is created if not already present. */
    template<typename MsgT>
    inline void publish(std::string_view topic, const MsgT& val)
    {
        this->publish<MsgT, MsgT>(topic, val);
    }

    /* Publish a value on the provided topic, which is created if not already present. */
    template<typename MsgT, typename T>
    void publish(std::string_view topic, const T& val)
    {
        SharedPub<MsgT> pub = this->getPub<MsgT>(topic);
        if (!pub)
        {
            return;
        }

        if constexpr (std::is_same_v<T, MsgT>)
        {
            pub->publish(val);
            return;
        }
        else if constexpr (std::is_convertible_v<T, MsgT>)
        {
            pub->publish(static_cast<MsgT>(val));
            return;
        }
        else if constexpr (is_convertible_to_std_msg<T, MsgT>::value)
        {
            pub->publish(
                MsgT{}.set__data(static_cast<typename MsgT::_data_type>(val)));
            return;
        }
        else
        {
            static_assert(
                std::is_same_v<T, MsgT> ||
                std::is_convertible_v<T, MsgT> ||
                is_convertible_to_std_msg<T, MsgT>::value, "Error: incompatible types!");
        }
    }

protected:
    StringMap<SharedBasePtr> publishers;
    std::shared_mutex mtx;
};

using GenericPubMap = PubMap<void>;

};  // namespace util
