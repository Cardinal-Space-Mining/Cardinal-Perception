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

#include <stdio.h>

#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <mutex>
#include <string.h>

#include <sys/times.h>


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

    double last_comp_time{0.};
    double avg_comp_time{0.};
    double max_comp_time{0.};
    double avg_call_delta{0.};
    size_t samples{0};

protected:
    std::chrono::system_clock::time_point last_call_time;
    // std::mutex mtx;
};
using ThreadMetrics = ThreadMetrics_<>;



std::string cpuBrandString();
size_t numProcessors();
double cpuFreq(size_t p_num = 0);
void getProcessStats(double& resident_set_mb, size_t& num_threads);

#ifdef HAS_SENSORS
double readCpuTemp();
#endif

struct ProcessMetrics
{
    ProcessMetrics();
    void update();

    double last_cpu_percent{0.};
    double avg_cpu_percent{0.};
    double max_cpu_percent{0.};

protected:
    clock_t last_cpu, last_sys_cpu, last_user_cpu;
    size_t cpu_samples{0}, num_processors{0};
};

};  // namespace proc
};  // namespace util



template<size_t Rolling_Sample_Max>
double util::proc::ThreadMetrics_<Rolling_Sample_Max>::addSample(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    // std::lock_guard<std::mutex> _lock{ this->mtx };

    const double call_diff =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            start - this->last_call_time)
            .count();
    const double comp_time =
        std::chrono::duration_cast<std::chrono::duration<double>>(end - start)
            .count();

    this->last_call_time = start;
    this->last_comp_time = comp_time;

    const size_t sample_weight =
        (Rolling_Sample_Max > 0) ? std::min(Rolling_Sample_Max, this->samples)
                                 : this->samples;

    this->avg_comp_time =
        (this->avg_comp_time * sample_weight + comp_time) / (sample_weight + 1);
    if (this->samples != 0)
    {
        this->avg_call_delta =
            (this->avg_call_delta * sample_weight + call_diff) /
            (sample_weight + 1);
    }
    this->samples++;

    if (comp_time > this->max_comp_time)
    {
        this->max_comp_time = comp_time;
    }

    return comp_time;
}
