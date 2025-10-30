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

#include <geometry_msgs/msg/pose_array.hpp>

#include <cardinal_perception/srv/update_mining_eval_mode.hpp>

#include <pub_map.hpp>
#include <synchronization.hpp>

#include "shared_resources.hpp"
#include "../perception_presets.hpp"


namespace csm
{
namespace perception
{

class MiningEvalWorker
{
    friend class PerceptionNode;

    using RclNode = rclcpp::Node;
    using Tf2Buffer = tf2_ros::Buffer;

    using PoseArrayMsg = geometry_msgs::msg::PoseArray;

    using UpdateMiningEvalSrv = cardinal_perception::srv::UpdateMiningEvalMode;

public:
    MiningEvalWorker(
        RclNode& node,
        const Tf2Buffer& tf_buffer);
    ~MiningEvalWorker();

public:
    void configure(const std::string& odom_frame);  // TODO

    void accept(
        const UpdateMiningEvalSrv::Request::SharedPtr& req,
        const UpdateMiningEvalSrv::Response::SharedPtr& resp);

    ResourcePipeline<MiningEvalResources>& getInput();

    void startThreads();
    void stopThreads();

protected:
    void mining_eval_thread_worker();
    void mining_eval_callback(MiningEvalResources& buff);

protected:
    RclNode& node;
    const Tf2Buffer& tf_buffer;
    util::GenericPubMap pub_map;

    std::string odom_frame;

    std::atomic<bool> threads_running{false};
    std::atomic<bool> srv_enable_state{false};

    ResourcePipeline<PoseArrayMsg> query_notifier;
    ResourcePipeline<MiningEvalResources> mining_eval_resources;
    std::thread mining_eval_thread;
};

};  // namespace perception
};  // namespace csm
