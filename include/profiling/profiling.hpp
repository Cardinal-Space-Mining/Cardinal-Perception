#pragma once

#include <rclcpp/rclcpp.hpp>


#define PROFILE_MODE_DISABLED 0  // all publishing gets compiled out
#define PROFILE_MODE_BASIC    1  // only basic metrics get published
#define PROFILE_MODE_ADVANCED 2  // all profiling metrics get published

#define PROFILING_DEFAULT_TOPIC "trace_notifications"
#define PROFILING_DEFAULT_QOS                                           \
    rclcpp::QoS(                                                        \
        rclcpp::QoSInitialization(                                      \
            rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST, \
            10))                                                        \
        .keep_last(1)                                                   \
        .reliable()                                                     \
        .durability_volatile()

#ifndef PROFILING_MODE
    #define PROFILING_MODE PROFILE_MODE_BASIC
#endif
#ifndef PROFILING_BUFFER_SIZE
    #define PROFILING_BUFFER_SIZE 0
#endif

namespace util
{
namespace profiling
{

void init(
    rclcpp::Node& node,
    const char* topic = PROFILING_DEFAULT_TOPIC,
    const rclcpp::QoS& qos = PROFILING_DEFAULT_QOS);
void flush();
void notify(const char* label);
void notify(const char* label1, const char* label2);

};  // namespace profiling
};  // namespace util

#if PROFILING_MODE >= PROFILE_MODE_ADVANCED
    #define PROFILING_NOTIFY(label) util::profiling::notify(#label)
    #define PROFILING_NOTIFY2(label1, label2)       \
        util::profiling::notify(#label1, #label2)
#else
    #define PROFILING_NOTIFY(...)
    #define PROFILING_NOTIFY2(...)
#endif
#if PROFILING_MODE >= PROFILE_MODE_BASIC
    #define PROFILING_INIT(node, ...)                          \
        util::profiling::init(node __VA_OPT__(, ) __VA_ARGS__)
    #define PROFILING_FLUSH()             util::profiling::flush()
    #define PROFILING_NOTIFY_BASIC(label) util::profiling::notify(#label)
    #define PROFILING_NOTIFY2_BASIC(label1, label2) \
        util::profiling::notify(#label1, #label2)
#else
    #define PROFILING_INIT(...)
    #define PROFILING_NOTIFY_BASIC(...)
    #define PROFILING_NOTIFY2_BASIC(...)
    #define PROFILING_FLUSH(...)
#endif
