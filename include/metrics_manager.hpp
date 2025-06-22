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
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <functional>
#include <string_view>
#include <type_traits>
#include <unordered_map>

#include "cardinal_perception/msg/thread_metrics.hpp"

#include "pub_map.hpp"
#include "stats/stats.hpp"


namespace csm
{
namespace perception
{

template<typename Key_T, typename Clock_T = std::chrono::system_clock>
class MetricsManager
{
    using ThreadMetricsMsg = cardinal_perception::msg::ThreadMetrics;

public:
    using KeyT = Key_T;
    using ClockT = Clock_T;
    using TimePoint = typename ClockT::time_point;
    using ProcDurationT = std::pair<double, TimePoint>;
    using ProcDurationsMap = std::unordered_map<KeyT, ProcDurationT>;
    using ThreadDurationsMap =
        std::unordered_map<std::thread::id, ProcDurationsMap>;
    using StatisticsMap = std::unordered_map<KeyT, util::proc::ThreadMetrics>;

    using KeyArgT = typename std::
        conditional<(sizeof(KeyT) > sizeof(uintptr_t)), KeyT&, KeyT>::type;

public:
    inline MetricsManager() = default;
    inline ~MetricsManager() = default;

    TimePoint registerProcStart(const KeyArgT key);
    TimePoint registerProcEnd(const KeyArgT key, bool track_stats = false);

    void publishThreadMetrics(
        util::PublisherMap<ThreadMetricsMsg>& pub_map,
        std::function<std::string_view(const KeyArgT)> topic_map);

    ThreadDurationsMap& getThreadDurations(std::unique_lock<std::mutex>& lock);
    const StatisticsMap& readProcStatistics(
        std::unique_lock<std::mutex>& lock) const;

protected:
    ProcDurationT* getProcDurations(const KeyArgT key);

protected:
    ThreadDurationsMap thread_proc_durations;
    StatisticsMap proc_stats;
    mutable std::mutex mtx;
};



template<typename K, typename C>
typename MetricsManager<K, C>::ProcDurationT*
    MetricsManager<K, C>::getProcDurations(const KeyArgT key)
{
    std::thread::id id_ = std::this_thread::get_id();

    auto ptr = this->thread_proc_durations.find(id_);
    if (ptr == this->thread_proc_durations.end())
    {
        auto x = this->thread_proc_durations.insert({id_, {}});
        if (!x.second)
        {
            return nullptr;
        }
        else
        {
            ptr = x.first;
        }
    }

    ProcDurationsMap& durs = ptr->second;
    auto times_ptr = durs.find(key);
    if (times_ptr == durs.end())
    {
        auto x = durs.insert({
            key,
            {0., TimePoint::min()}
        });
        if (!x.second)
        {
            return nullptr;
        }
        else
        {
            times_ptr = x.first;
        }
    }

    return &times_ptr->second;
}

template<typename K, typename C>
typename MetricsManager<K, C>::TimePoint
    MetricsManager<K, C>::registerProcStart(const KeyArgT key)
{
    std::lock_guard<std::mutex> lock_{this->mtx};
    auto tp = ClockT::now();

    auto times = this->getProcDurations(key);
    if (!times)
    {
        return tp;
    }

    if (times->second <= TimePoint::min())
    {
        times->second = tp;
    }

    return tp;
}

template<typename K, typename C>
typename MetricsManager<K, C>::TimePoint MetricsManager<K, C>::registerProcEnd(
    const KeyArgT key,
    bool track_stats)
{
    std::lock_guard<std::mutex> lock_{this->mtx};
    auto tp = ClockT::now();

    auto times = this->getProcDurations(key);
    if (!times)
    {
        return tp;
    }

    if (times->second > TimePoint::min())
    {
        if (track_stats)
        {
            auto ptr = this->proc_stats.find(key);
            if (ptr == this->proc_stats.end())
            {
                auto x = this->proc_stats.insert({key, {}});
                if (x.second)
                {
                    ptr = x.first;
                }
            }
            if (ptr != this->proc_stats.end())
            {
                ptr->second.addSample(times->second, tp);
            }
        }

        times->first +=
            std::chrono::duration<double>(tp - times->second).count();
        times->second = TimePoint::min();
    }

    return tp;
}

template<typename K, typename C>
void MetricsManager<K, C>::publishThreadMetrics(
    util::PublisherMap<ThreadMetricsMsg>& pub_map,
    std::function<std::string_view(const KeyArgT)> topic_map)
{
    std::lock_guard<std::mutex> lock_{this->mtx};

    ThreadMetricsMsg msg;

    for (const auto& proc : this->proc_stats)
    {
        std::string_view topic = topic_map(proc.first);
        if (!topic.empty())
        {
            msg.delta_t = static_cast<float>(proc.second.last_comp_time);
            msg.avg_delta_t = static_cast<float>(proc.second.avg_comp_time);
            msg.avg_freq = static_cast<float>(1. / proc.second.avg_call_delta);
            msg.iterations = proc.second.samples;

            pub_map.publish(topic, msg);
        }
    }
}

template<typename K, typename C>
typename MetricsManager<K, C>::ThreadDurationsMap&
    MetricsManager<K, C>::getThreadDurations(std::unique_lock<std::mutex>& lock)
{
    if (lock.mutex() != &this->mtx)
    {
        lock = std::unique_lock<std::mutex>{this->mtx};
    }

    return this->thread_proc_durations;
}

template<typename K, typename C>
const typename MetricsManager<K, C>::StatisticsMap&
    MetricsManager<K, C>::readProcStatistics(
        std::unique_lock<std::mutex>& lock) const
{
    if (lock.mutex() != &this->mtx)
    {
        lock = std::unique_lock<std::mutex>{this->mtx};
    }

    return this->proc_stats;
}

};  // namespace perception
};  // namespace csm
