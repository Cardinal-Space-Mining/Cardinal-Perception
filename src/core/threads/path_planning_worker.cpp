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

#include "path_planning_worker.hpp"

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <csm_metrics/profiling.hpp>

#include <util.hpp>
#include <geometry.hpp>
// #include <modules/map_octree.hpp>


using namespace util::geom::cvt::ops;

using Vec3f = Eigen::Vector3f;

using PathMsg = nav_msgs::msg::Path;
using PointCloudMsg = sensor_msgs::msg::PointCloud2;


// namespace csm::perception
// {

// template<typename Point_T, typename Meta_T>
// class PlannningMap :
//     public MapOctree<pcl::PointXYZI, MAP_OCTREE_DEFAULT, PlanningMap>
// {
//     static_assert(pcl::traits::has_xyz<Point_T>::value);
//     static_assert(util::traits::has_trav_weight<Meta_T>::value);

// private:
//     using PointT = Point_T;
//     using MetaT = Meta_T;
//     using PointCloudT = pcl::PointCloud<PointT>;
//     using MetaCloudT = pcl::PointCloud<MetaT>;

//     using Vec3f = Eigen::Vector3f;

// public:
//     PlanningMap(double voxel_res);
//     ~PlanningMap() = default;

// public:
//     void update(
//         const PointCloudT& points,
//         const MetaCloudT& points_meta,
//         const Vec3f& bound_min,
//         const Vec3f& bound_max);

// protected:
// };

// };  // namespace csm::perception


namespace csm
{
namespace perception
{

PathPlanningWorker::PathPlanningWorker(
    RclNode& node,
    const Tf2Buffer& tf_buffer) :
    node{node},
    tf_buffer{tf_buffer},
    pub_map{&node, PERCEPTION_TOPIC(""), PERCEPTION_PUBSUB_QOS}
{
}

PathPlanningWorker::~PathPlanningWorker() { this->stopThreads(); }

void PathPlanningWorker::configure(const std::string& odom_frame)
{
    this->odom_frame = odom_frame;
}

void PathPlanningWorker::accept(
    const UpdatePathPlanSrv::Request::SharedPtr& req,
    const UpdatePathPlanSrv::Response::SharedPtr& resp)
{
    if (req->completed)
    {
        this->srv_enable_state = false;
    }
    else
    {
        this->srv_enable_state = true;
        this->pplan_target_notifier.updateAndNotify(req->target);
    }

    resp->running = this->srv_enable_state;
}

ResourcePipeline<PathPlanningResources>& PathPlanningWorker::getInput()
{
    return this->path_planning_resources;
}

void PathPlanningWorker::startThreads()
{
    if (!this->threads_running)
    {
        this->threads_running = true;
        this->path_planning_thread =
            std::thread{&PathPlanningWorker::path_planning_thread_worker, this};
    }
}

void PathPlanningWorker::stopThreads()
{
    this->threads_running = false;

    this->pplan_target_notifier.notifyExit();
    this->path_planning_resources.notifyExit();
    if (this->path_planning_thread.joinable())
    {
        this->path_planning_thread.join();
    }
}

void PathPlanningWorker::path_planning_thread_worker()
{
    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Path planning thread started successfully.");

    do
    {
        if (this->srv_enable_state.load())
        {
            auto& buff = this->path_planning_resources.waitNewestResource();
            if (!this->threads_running.load())
            {
                break;
            }

            buff.target = this->pplan_target_notifier.aquireNewestOutput();

            PROFILING_SYNC();
            PROFILING_NOTIFY_ALWAYS(path_planning);
            this->path_planning_callback(buff);
            PROFILING_NOTIFY_ALWAYS(path_planning);
        }
        else
        {
            this->pplan_target_notifier.waitNewestResource();
        }
    }  //
    while (this->threads_running.load());

    RCLCPP_INFO(
        this->node.get_logger(),
        "[CORE]: Path planning thread exited cleanly.");
}


void PathPlanningWorker::path_planning_callback(PathPlanningResources& buff)
{
    if (buff.target.header.frame_id != this->odom_frame)
    {
        try
        {
            auto tf = this->tf_buffer.lookupTransform(
                this->odom_frame,
                buff.target.header.frame_id,
                // util::toTf2TimePoint(buff.stamp));
                tf2::timeFromSec(0));

            // TODO: ensure tf stamp is not wildly out of date

            tf2::doTransform(buff.target, buff.target, tf);
        }
        catch (const std::exception& e)
        {
            RCLCPP_INFO(
                this->node.get_logger(),
                "[PATH PLANNING CALLBACK]: Failed to transform target pose from '%s' to '%s'\n\twhat(): %s",
                buff.target.header.frame_id.c_str(),
                this->odom_frame.c_str(),
                e.what());
            return;
        }
    }

    Vec3f odom_target;
    odom_target << buff.target.pose.position;

    std::vector<Vec3f> path;

    if (!this->path_planner.solvePath(
            buff.base_to_odom.pose.vec,
            odom_target,
            buff.bounds_min,
            buff.bounds_max,
            buff.points,
            buff.points_meta,
            path))
    {
        return;
    }

    PathMsg path_msg;
    path_msg.header.frame_id = this->odom_frame;
    path_msg.header.stamp = util::toTimeStamp(buff.stamp);

    path_msg.poses.reserve(path.size());
    for (const Vec3f& kp : path)
    {
        PoseStampedMsg& pose = path_msg.poses.emplace_back();
        pose.pose.position << kp;
        pose.header.frame_id = this->odom_frame;
    }

    this->pub_map.publish("planned_path", path_msg);
    this->pub_map.publish("pplan_target", buff.target);
}

};  // namespace perception
};  // namespace csm
