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

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <csm_metrics/stats.hpp>

#include <cardinal_perception/msg/tags_transform.hpp>
#include <cardinal_perception/srv/update_mining_eval_mode.hpp>
#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#include <util.hpp>
#include <pub_map.hpp>

#include "threads/imu_worker.hpp"
#include "threads/mapping_worker.hpp"
#include "threads/mining_eval_worker.hpp"
#include "threads/localization_worker.hpp"
#include "threads/path_planning_worker.hpp"
#include "threads/traversibility_worker.hpp"

#include "perception_presets.hpp"


namespace csm
{
namespace perception
{

struct PerceptionConfig;

class PerceptionNode : public rclcpp::Node
{
protected:
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using TagsTransformMsg = cardinal_perception::msg::TagsTransform;

    using SetBoolSrv = std_srvs::srv::SetBool;
    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;
    using UpdateMiningEvalSrv = cardinal_perception::srv::UpdateMiningEvalMode;

    using ProcessStatsCtx = csm::metrics::ProcessStats;

public:
    PerceptionNode();
    ~PerceptionNode();
    DECLARE_IMMOVABLE(PerceptionNode)

    void shutdown();

protected:
    void getParams(PerceptionConfig& cfg);
    void initPubSubs(PerceptionConfig& cfg);
    void printStartup(PerceptionConfig& cfg);

private:
    // --- TRANSFORM UTILITEIS -------------------------------------------------
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // --- CORE COMPONENTS -----------------------------------------------------
    ImuWorker imu_worker;
    LocalizationWorker localization_worker;
    MappingWorker mapping_worker;
    TraversibilityWorker traversibility_worker;
    PathPlanningWorker path_planning_worker;
    MiningEvalWorker mining_eval_worker;

    // --- SUBSCRIPTIONS/SERVICES/PUBLISHERS -----------------------------------
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
    rclcpp::Subscription<PointCloudMsg>::SharedPtr scan_sub;
    IF_TAG_DETECTION_ENABLED(
        rclcpp::Subscription<TagsTransformMsg>::SharedPtr detections_sub;)

    rclcpp::Service<SetBoolSrv>::SharedPtr alignment_state_service;
    rclcpp::Service<UpdatePathPlanSrv>::SharedPtr path_plan_service;
    rclcpp::Service<UpdateMiningEvalSrv>::SharedPtr mining_eval_service;

    rclcpp::TimerBase::SharedPtr proc_stats_timer;

    util::GenericPubMap generic_pub;

    // --- METRICS -------------------------------------------------------------
    ProcessStatsCtx process_stats;
};

};  // namespace perception
};  // namespace csm
