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

#include "mining_eval_worker.hpp"

#include <limits>

#include <Eigen/Core>

#include <cardinal_perception/msg/mining_eval_results.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>


using Vec3f = Eigen::Vector3f;
using Iso3f = Eigen::Isometry3f;
using Box3f = Eigen::AlignedBox3f;

using MiningEvalResultsMsg = cardinal_perception::msg::MiningEvalResults;

using namespace util::geom::cvt::ops;


namespace csm
{
namespace perception
{

MiningEvalWorker::MiningEvalWorker(RclNode& node, const Tf2Buffer& tf_buffer) :
    node{node},
    tf_buffer{tf_buffer},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

MiningEvalWorker::~MiningEvalWorker() { this->stopThreads(); }

void MiningEvalWorker::configure(const std::string& odom_frame)
{
    this->odom_frame = odom_frame;
}

void MiningEvalWorker::accept(
    const UpdateMiningEvalSrv::Request::SharedPtr& req,
    const UpdateMiningEvalSrv::Response::SharedPtr& resp)
{
    if (req->completed)
    {
        this->srv_enable_state = false;
        resp->query_id = -1;
    }
    else
    {
        this->srv_enable_state = true;
        {
            auto& buff = this->query_notifier.lockInput();
            if (!buff)
            {
                buff = std::make_shared<Query>();
            }
            buff->poses = req->queries;
            buff->eval_width = req->eval_width;
            buff->eval_height = req->eval_height;
            resp->query_id = buff->id = this->query_count.load();
            this->query_notifier.unlockInputAndNotify(buff);
        }
        this->query_count++;
    }
}

ResourcePipeline<MiningEvalResources>& MiningEvalWorker::getInput()
{
    return this->mining_eval_resources;
}

void MiningEvalWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->mining_eval_thread =
            std::thread{&MiningEvalWorker::mining_eval_thread_worker, this};
    }
}

void MiningEvalWorker::stopThreads()
{
    this->threads_running = false;

    this->query_notifier.notifyExit();
    this->mining_eval_resources.notifyExit();
    if (this->mining_eval_thread.joinable())
    {
        this->mining_eval_thread.join();
    }
}

void MiningEvalWorker::mining_eval_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mining evaluation thread started successfully.");

    do
    {
        if (this->srv_enable_state.load())
        {
            auto& buff = this->mining_eval_resources.waitNewestResource();
            if (!this->threads_running.load())
            {
                break;
            }

            buff.query = this->query_notifier.aquireNewestOutput();

            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(mining_eval);
            this->mining_eval_callback(buff);
            PROFILING_NOTIFY_ALWAYS(mining_eval);
        }
        else
        {
            this->query_notifier.waitNewestResource();
        }
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Mining evaluation thread exited cleanly.");
}


void MiningEvalWorker::mining_eval_callback(MiningEvalResources& buff)
{
    auto query_ptr = std::static_pointer_cast<Query>(buff.query);

    MiningEvalResultsMsg msg;
    msg.query_id = query_ptr->id;
    msg.ranges.resize(query_ptr->poses.poses.size(), 0.f);

    Iso3f odom_to_ref_tf = Iso3f::Identity();
    const auto& header = query_ptr->poses.header;
    if (header.frame_id != this->odom_frame)
    {
        try
        {
            odom_to_ref_tf << this->tf_buffer
                                  .lookupTransform(
                                      header.frame_id,
                                      this->odom_frame,
                                      util::toTf2TimePoint(header.stamp))
                                  .transform;
        }
        catch (const std::exception& e)
        {
            return;
        }
    }

    Box3f eval_bound;
    eval_bound.min().x() = 0.f;
    eval_bound.min().y() = query_ptr->eval_width * -0.5f;
    eval_bound.min().z() = query_ptr->eval_height * -0.5f;
    eval_bound.max().x() = std::numeric_limits<float>::infinity();
    eval_bound.max().y() = query_ptr->eval_width * 0.5f;
    eval_bound.max().z() = query_ptr->eval_height * 0.5f;

    for (size_t i = 0; i < query_ptr->poses.poses.size(); i++)
    {
        Iso3f pose_tf;
        // (ref to pose tf)[4x4] * (odom to ref tf)[4x4] --> (odom to pose tf)[4x4]
        pose_tf =
            (pose_tf << query_ptr->poses.poses[i]).inverse() * odom_to_ref_tf;

        msg.ranges[i] = std::numeric_limits<float>::infinity();
        for (const auto& pt : buff.avoid_points)
        {
            Vec3f pt_ = (pose_tf * pt.getVector4fMap()).template head<3>();

            if(eval_bound.contains(pt_) && pt_.x() < msg.ranges[i])
            {
                msg.ranges[i] = pt_.x();
            }
        }
    }

    this->pub_map.publish("mining_evaluation", msg);
}

};  // namespace perception
};  // namespace csm
