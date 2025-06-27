#include "profiling/profiling.hpp"

#include <mutex>
#include <chrono>
#include <thread>
#include <vector>

#include <cardinal_perception/msg/trace_notification.hpp>
#include <cardinal_perception/msg/trace_notifications.hpp>


namespace util
{
namespace profiling
{

using TraceNotificationMsg = cardinal_perception::msg::TraceNotification;
using TraceNotificationsMsg = cardinal_perception::msg::TraceNotifications;


static struct
{
    rclcpp::Publisher<TraceNotificationsMsg>::SharedPtr pub{nullptr};
    std::hash<std::thread::id> id_hash;

#if PROFILING_BUFFER_SIZE > 0
    TraceNotificationsMsg buffer;
    std::mutex mtx;
#endif
    //
} ctx;


inline uint64_t getNs()
{
    return (std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count());
}
inline uint64_t getId() { return ctx.id_hash(std::this_thread::get_id()); }

inline void fillMsg(
    TraceNotificationMsg& msg,
    const char* label,
    uint64_t ns = getNs(),
    uint64_t id = getId())
{
    msg.label = label;
    msg.ns_since_epoch = ns;
    msg.thread_id = id;
}

void init(rclcpp::Node& node, const char* topic, const rclcpp::QoS& qos)
{
#if PROFILING_BUFFER_SIZE > 0
    ctx.mtx.lock();
#endif
    ctx.pub = node.create_publisher<TraceNotificationsMsg>(topic, qos);
#if PROFILING_BUFFER_SIZE > 0
    ctx.buffer.notifications.reserve(PROFILING_BUFFER_SIZE);
    ctx.mtx.unlock();
#endif
}
void flush()
{
#if PROFILING_BUFFER_SIZE > 0
    ctx.mtx.lock();
    ctx.pub->publish(ctx.buffer);
    ctx.buffer.notifications.clear();
    ctx.mtx.unlock();
#endif
}

void notify(const char* label)
{
#if PROFILING_BUFFER_SIZE > 0
    ctx.mtx.lock();
    static_assert(!"NOT IMPLEMENTED");
    ctx.mtx.unlock();
#else
    TraceNotificationsMsg msg;
    fillMsg(msg.notifications.emplace_back(), label);
    ctx.pub->publish(msg);
#endif
}
void notify(const char* label1, const char* label2)
{
#if PROFILING_BUFFER_SIZE > 0
    ctx.mtx.lock();
    static_assert(!"NOT IMPLEMENTED!");
    ctx.mtx.unlock();
#else
    TraceNotificationsMsg msg;
    msg.notifications.resize(2);
    const uint64_t ns = getNs();
    const uint64_t id = getId();

    fillMsg(msg.notifications[0], label1, ns, id);
    fillMsg(msg.notifications[1], label2, ns, id);
    ctx.pub->publish(msg);
#endif
}

};  // namespace profiling
};  // namespace util
