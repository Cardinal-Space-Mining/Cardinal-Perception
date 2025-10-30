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

#include "traversibility_worker.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>


using namespace util::geom::cvt::ops;

using Vec3f = Eigen::Vector3f;
using Vec3d = Eigen::Vector3d;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;


namespace csm
{
namespace perception
{

TraversibilityWorker::TraversibilityWorker(
    RclNode& node,
    const ImuIntegrator<>& imu_sampler) :
    node{node},
    imu_sampler{imu_sampler},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

TraversibilityWorker::~TraversibilityWorker() { this->stopThreads(); }

void TraversibilityWorker::configure(const std::string& odom_frame)
{
    this->odom_frame = odom_frame;
}

ResourcePipeline<TraversibilityResources>& TraversibilityWorker::getInput()
{
    return this->traversibility_resources;
}
void TraversibilityWorker::connectOutput(
    ResourcePipeline<PathPlanningResources>& path_planning_resources)
{
    this->path_planning_resources = &path_planning_resources;
}
void TraversibilityWorker::connectOutput(
    ResourcePipeline<MiningEvalResources>& mining_eval_resources)
{
    this->mining_eval_resources = &mining_eval_resources;
}

void TraversibilityWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->traversibility_thread = std::thread{
            &TraversibilityWorker::traversibility_thread_worker,
            this};
    }
}

void TraversibilityWorker::stopThreads()
{
    this->threads_running = false;

    this->traversibility_resources.notifyExit();
    if (this->traversibility_thread.joinable())
    {
        this->traversibility_thread.join();
    }
}

void TraversibilityWorker::traversibility_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Traversibility thread started successfully.");

    do
    {
        auto& buff = this->traversibility_resources.waitNewestResource();
        if (!this->threads_running.load())
        {
            return;
        }

        PROFILING_SYNC();
        PROFILING_NOTIFY_ALWAYS(traversibility);
        this->traversibility_callback(buff);
        PROFILING_NOTIFY_ALWAYS(traversibility);
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Traversibility thread exited cleanly.");
}


void TraversibilityWorker::traversibility_callback(
    TraversibilityResources& buff)
{
    PROFILING_NOTIFY(traversibility_preproc);

    thread_local Vec3f env_grav_vec{0.f, 0.f, 1.f};
    if (this->imu_sampler.hasSamples())
    {
        double stddev, delta_r;
        const Vec3d grav_vec =
            this->imu_sampler.estimateGravity(0.5, &stddev, &delta_r);
        if (stddev < 1. && delta_r < 0.01)
        {
            env_grav_vec =
                (buff.base_to_odom.tf * grav_vec.template cast<float>())
                    .normalized();
        }
    }

    PROFILING_NOTIFY2(traversibility_preproc, traversibility_gen_proc);

    this->trav_gen.processMapPoints(
        buff.points,
        buff.bounds_min,
        buff.bounds_max,
        env_grav_vec,
        buff.base_to_odom.pose.vec);

    PROFILING_NOTIFY2(traversibility_gen_proc, traversibility_export);

    pcl::PointCloud<TraversibilityPointType>::Ptr trav_points =
        std::make_shared<pcl::PointCloud<TraversibilityPointType>>();
    pcl::PointCloud<TraversibilityMetaType>::Ptr trav_meta =
        std::make_shared<pcl::PointCloud<TraversibilityMetaType>>();

    this->trav_gen.swapPoints(*trav_points);
    this->trav_gen.swapMetaDataList(trav_meta->points);

    if (this->path_planning_resources)
    {
        auto& x = this->path_planning_resources->lockInput();
        x.stamp = buff.stamp;
        x.bounds_min = buff.bounds_min;
        x.bounds_max = buff.bounds_max;
        x.base_to_odom = buff.base_to_odom;
        x.points = trav_points;
        x.points_meta = trav_meta;
        this->path_planning_resources->unlockInputAndNotify(x);
    }
    if (this->mining_eval_resources)
    {
        auto& x = this->mining_eval_resources->lockInput();
        x.stamp = buff.stamp;
        x.bounds_min = buff.bounds_min;
        x.bounds_max = buff.bounds_max;
        x.points = trav_points;
        x.points_meta = trav_meta;
        this->mining_eval_resources->unlockInputAndNotify(x);
    }

    PROFILING_NOTIFY2(traversibility_export, traversibility_debpub);

    pcl::PointCloud<pcl::PointXYZINormal> trav_debug_cloud;
    trav_debug_cloud.points.resize(trav_points->size());
    for (size_t i = 0; i < trav_points->size(); i++)
    {
        auto& out = trav_debug_cloud.points[i];
        out.getVector3fMap() = trav_points->points[i].getVector3fMap();
        out.getNormalVector3fMap() =
            trav_meta->points[i].getNormalVector3fMap();
        out.curvature = trav_meta->points[i].curvature;
        out.intensity = trav_meta->points[i].trav_weight();
    }
    trav_debug_cloud.height = 1;
    trav_debug_cloud.width = trav_debug_cloud.points.size();
    trav_debug_cloud.is_dense = true;

    try
    {
        PointCloudMsg output;
        pcl::toROSMsg(trav_debug_cloud, output);
        output.header.stamp = util::toTimeStamp(buff.stamp);
        output.header.frame_id = this->odom_frame;
        this->pub_map.publish("traversibility_points", output);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->node.get_logger(),
            "[TRAVERSIBILITY]: Failed to publish debug cloud -- what():\n\t%s",
            e.what());
    }

    PROFILING_NOTIFY(traversibility_debpub);
}

};  // namespace perception
};  // namespace csm
