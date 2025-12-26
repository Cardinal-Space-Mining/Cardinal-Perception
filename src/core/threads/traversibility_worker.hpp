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

#include <config.hpp>

#include <atomic>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <modules/traversibility_gen.hpp>

#include <util/pub_map.hpp>
#include <util/synchronization.hpp>

#include <imu_integrator.hpp>

#include "shared_resources.hpp"
#include "../perception_presets.hpp"


namespace csm
{
namespace perception
{

class TraversibilityWorker
{
    friend class PerceptionNode;

    using RclNode = rclcpp::Node;

public:
    TraversibilityWorker(RclNode& node, const ImuIntegrator<>& imu_sampler);
    ~TraversibilityWorker();

public:
    void configure(const std::string& odom_frame);

    util::ResourcePipeline<TraversibilityResources>& getInput();
    void connectOutput(
        util::ResourcePipeline<PathPlanningResources>& path_planning_resources);
    void connectOutput(
        util::ResourcePipeline<MiningEvalResources>& mining_eval_resources);

    void startThreads();
    void stopThreads();

protected:
    void traversibility_thread_worker();
    void traversibility_callback(TraversibilityResources& buff);

protected:
    RclNode& node;
    const ImuIntegrator<>& imu_sampler;
    util::GenericPubMap pub_map;

    std::string odom_frame;

    std::atomic<bool> threads_running{false};

    TraversibilityGenerator<TraversibilityPointType, TraversibilityMetaType>
        trav_gen;
    util::ResourcePipeline<TraversibilityResources> traversibility_resources;
    util::ResourcePipeline<PathPlanningResources>* path_planning_resources{
        nullptr};
    util::ResourcePipeline<MiningEvalResources>* mining_eval_resources{nullptr};
    std::thread traversibility_thread;
};

};  // namespace perception
};  // namespace csm
