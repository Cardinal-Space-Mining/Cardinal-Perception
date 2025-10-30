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

#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#include <modules/path_planner.hpp>

#include <pub_map.hpp>
#include <synchronization.hpp>

#include "shared_resources.hpp"
#include "../perception_presets.hpp"


namespace csm
{
namespace perception
{

class PathPlanningWorker
{
    friend class PerceptionNode;

    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;

    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;

public:
    PathPlanningWorker(RclNode& node, const Tf2Buffer& tf_buffer);
    ~PathPlanningWorker();

public:
    void configure(const std::string& odom_frame);

    void accept(
        const UpdatePathPlanSrv::Request::SharedPtr& req,
        const UpdatePathPlanSrv::Response::SharedPtr& resp);

    ResourcePipeline<PathPlanningResources>& getInput();

    void startThreads();
    void stopThreads();

protected:
    void path_planning_thread_worker();
    void path_planning_callback(PathPlanningResources& buff);

protected:
    RclNode& node;
    const Tf2Buffer& tf_buffer;
    util::GenericPubMap pub_map;

    std::string odom_frame;

    std::atomic<bool> threads_running{false};
    std::atomic<bool> srv_enable_state{false};

    PathPlanner<TraversibilityPointType, TraversibilityMetaType> path_planner;
    ResourcePipeline<PoseStampedMsg> pplan_target_notifier;
    ResourcePipeline<PathPlanningResources> path_planning_resources;
    std::thread path_planning_thread;
};

};  // namespace perception
};  // namespace csm
