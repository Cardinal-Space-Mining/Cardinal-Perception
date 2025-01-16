/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#pragma once

#include <stdio.h>

#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <string.h>

#include <sys/times.h>

#ifndef CORE_STATISTICS_STRICT_PARSING
#define CORE_STATISTICS_STRICT_PARSING  0
#endif


namespace util
{
namespace proc
{

    template<size_t Rolling_Sample_Max = 0>
    struct ThreadMetrics_
    {
        double addSample(
            const std::chrono::system_clock::time_point& start,
            const std::chrono::system_clock::time_point& end);

        double last_comp_time{0.}, avg_comp_time{0.}, max_comp_time{0.}, avg_call_delta{0.};
        size_t samples{0};

    protected:
        std::chrono::system_clock::time_point last_call_time;
        std::mutex mtx;

    };
    using ThreadMetrics = ThreadMetrics_<>;



    std::string cpuBrandString();
    size_t numProcessors();
    double cpuFreq(size_t p_num = 0);
    void getProcessStats(double& resident_set_mb, size_t& num_threads);

    struct ProcessMetrics
    {
        ProcessMetrics();
        void update();

        double last_cpu_percent{0.}, avg_cpu_percent{0.}, max_cpu_percent{0.};

    protected:
        clock_t last_cpu, last_sys_cpu, last_user_cpu;
        size_t cpu_samples{ 0 }, num_processors{ 0 };

    };



    class CoreStats
    {
    public:
        using _jiffies = uint64_t;
        enum class State
        {
            USER, NICE, SYSTEM,
            IDLE, IOWAIT, IRQ,
            SOFTIRQ, STEAL,
            GUEST, GUEST_NICE,
            TOTAL
        };

    public:
        inline CoreStats(bool p_all = false) : parse_all(p_all) {}
        CoreStats(const CoreStats&) = delete;
        CoreStats(CoreStats&&) = default;

        inline void configParseAll(bool val) { this->parse_all = val; }
        inline bool validCore(size_t c) const { return c < this->individual.size(); }

        inline _jiffies getIdle(size_t i = IMMEDIATE) const
        {
            return getStates<State::IDLE, State::IOWAIT>(i);
        }
        inline _jiffies getCoreIdle(size_t c, size_t i = IMMEDIATE) const
        {
            return getCoreStates<State::IDLE, State::IOWAIT>(c, i);
        }

        inline _jiffies getActive(size_t i = IMMEDIATE) const
        {
            return getStates<State::USER, State::NICE, State::SYSTEM, State::IRQ, State::SOFTIRQ, State::STEAL, State::GUEST, State::GUEST_NICE>(i);
        }
        inline _jiffies getCoreActive(size_t c, size_t i = IMMEDIATE) const
        {
            return getCoreStates<State::USER, State::NICE, State::SYSTEM, State::IRQ, State::SOFTIRQ, State::STEAL, State::GUEST, State::GUEST_NICE>(c, i);
        }

        inline _jiffies getTotal(size_t i = IMMEDIATE) const
        {
            return getIdle(i) + getActive(i);
        }
        inline _jiffies getCoreTotal(size_t c, size_t i = IMMEDIATE) const
        {
            return getCoreIdle(c, i) + getCoreActive(c, i);
        }

        template<State S>
        _jiffies getState(size_t = IMMEDIATE) const;
        template<State S>
        _jiffies getCoreState(size_t c, size_t = IMMEDIATE) const;
        template<State S = State::TOTAL, State... states>
        _jiffies getStates(size_t = IMMEDIATE) const;
        template<State S = State::TOTAL, State... states>
        _jiffies getCoreStates(size_t c, size_t = IMMEDIATE) const;

        float fromLast();
        float fromLastCore(size_t c);
        void fromLastAll(std::vector<float>&);

        template<typename rep, typename period>
        float inlineAverage(std::chrono::duration<rep,period>);
        template<typename rep, typename period>
        float inlineAverageCore(std::chrono::duration<rep,period>, size_t c);
        template<typename rep, typename period>
        void inlineAverageAll(std::chrono::duration<rep,period>, std::vector<float>&);


    protected:
        void updateBuff();

        using CoreBuff = std::array<_jiffies, static_cast<size_t>(State::TOTAL)>;
        enum
        {
            IMMEDIATE = 0,
            REFERENCE = 1
        };

        std::array<CoreBuff, 2> main;
        std::vector<std::array<CoreBuff, 2> > individual;
        char head[4];
        bool parse_all;

    };

};
};



template<size_t Rolling_Sample_Max>
double util::proc::ThreadMetrics_<Rolling_Sample_Max>::addSample(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    std::lock_guard<std::mutex> _lock{ this->mtx };

    const double
        call_diff = std::chrono::duration_cast<std::chrono::duration<double>>(start - this->last_call_time).count(),
        comp_time = std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

    this->last_call_time = start;
    this->last_comp_time = comp_time;

    const size_t sample_weight = (Rolling_Sample_Max > 0) ? std::min(Rolling_Sample_Max, this->samples) : this->samples;

    this->avg_comp_time = (this->avg_comp_time * sample_weight + comp_time) / (sample_weight + 1);
    if(this->samples != 0)
    {
        this->avg_call_delta = (this->avg_call_delta * sample_weight + call_diff) / (sample_weight + 1);
    }
    this->samples++;

    if(comp_time > this->max_comp_time) this->max_comp_time = comp_time;

    return comp_time;
}



template<util::proc::CoreStats::State S>
util::proc::CoreStats::_jiffies util::proc::CoreStats::getState(size_t i) const
{
    // if constexpr(s == State::TOTAL) {
    // 	return this->getTotal(i);
    // }
    return this->main[i][static_cast<size_t>(S)];
}

template<util::proc::CoreStats::State S>
util::proc::CoreStats::_jiffies util::proc::CoreStats::getCoreState(size_t c, size_t i) const
{
    // if constexpr(s == State::TOTAL) {
    // 	return this->getCoreTotal(c, i);
    // }
    if(this->validCore(c))
    {
        return this->individual[c][i][static_cast<size_t>(S)];
    }
    return 0U;
}

template<util::proc::CoreStats::State S, util::proc::CoreStats::State... states>
util::proc::CoreStats::_jiffies util::proc::CoreStats::getStates(size_t i) const
{
    if constexpr(S != State::TOTAL)
    {
        return this->getState<S>(i) + this->getStates<states...>(i);
    }
    return 0U;
}

template<util::proc::CoreStats::State S, util::proc::CoreStats::State... states>
util::proc::CoreStats::_jiffies util::proc::CoreStats::getCoreStates(size_t c, size_t i) const
{
    if constexpr(S != State::TOTAL)
    {
        return this->getCoreState<S>(c, i) + this->getCoreStates<states...>(c, i);
    }
    return 0U;
}

template<typename rep, typename period>
float util::proc::CoreStats::inlineAverage(std::chrono::duration<rep,period> dt)
{
    this->updateBuff();
    std::this_thread::sleep_for(dt);
    return this->fromLast();
}

template<typename rep, typename period>
float util::proc::CoreStats::inlineAverageCore(std::chrono::duration<rep,period> dt, size_t c)
{
    this->updateBuff();
    std::this_thread::sleep_for(dt);
    return this->fromLastCore(c);
}

template<typename rep, typename period>
void util::proc::CoreStats::inlineAverageAll(std::chrono::duration<rep,period> dt, std::vector<float>& out)
{
    this->updateBuff();
    std::this_thread::sleep_for(dt);
    this->fromLastAll(out);
}
