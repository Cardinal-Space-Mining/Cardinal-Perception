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

#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>


namespace util
{

namespace ros_aliases
{

using RclNode = rclcpp::Node;
using RclTimer = rclcpp::TimerBase::SharedPtr;

template<typename T>
using SharedPub = typename rclcpp::Publisher<T>::SharedPtr;
template<typename T>
using SharedSub = typename rclcpp::Subscription<T>::SharedPtr;
template<typename T>
using SharedSrv = typename rclcpp::Service<T>::SharedPtr;

#define BUILD_MSG_ALIAS(pkg, name)    using name##Msg = pkg::msg::name;
#define BUILD_SRV_ALIAS(pkg, name)    using name##Srv = pkg::srv::name;
#define BUILD_STD_MSG_ALIAS(name)     BUILD_MSG_ALIAS(std_msgs, name)
#define BUILD_SENSORS_MSG_ALIAS(name) BUILD_MSG_ALIAS(sensor_msgs, name)
#define BUILD_GEOM_MSG_ALIAS(name)    BUILD_MSG_ALIAS(geometry_msgs, name)
#define BUILD_BUILTIN_MSG_ALIAS(name) BUILD_MSG_ALIAS(builtin_interfaces, name)

};  // namespace ros_aliases


template<typename T>
struct identity
{
    typedef T type;
};

template<typename T>
inline void declare_param(
    rclcpp::Node* node,
    const std::string param_name,
    T& param,
    const typename identity<T>::type& default_value)
{
    node->declare_parameter(param_name, default_value);
    node->get_parameter(param_name, param);
}
template<typename T>
inline void declare_param(
    rclcpp::Node& node,
    const std::string param_name,
    T& param,
    const typename identity<T>::type& default_value)
{
    node.declare_parameter(param_name, default_value);
    node.get_parameter(param_name, param);
}
template<typename T>
inline T declare_and_get_param(
    rclcpp::Node& node,
    const std::string param_name,
    const T& default_value)
{
    node.declare_parameter(param_name, default_value);
    return node.get_parameter_or(param_name, default_value);
}


template<typename ros_T, typename primitive_T>
inline ros_T to_ros_val(primitive_T v)
{
    static_assert(std::is_same<typename ros_T::_data_type, primitive_T>::value);

    return ros_T{}.set__data(v);
}

};  // namespace util
