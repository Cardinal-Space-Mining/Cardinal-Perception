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

#include "../config.hpp"
#include <point_def.hpp>    // needs to come before PCL includes when using custom types!

#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "cardinal_perception/msg/tags_transform.hpp"
#include "cardinal_perception/msg/process_metrics.hpp"
#include "cardinal_perception/msg/thread_metrics.hpp"
#include "cardinal_perception/msg/trajectory_filter_debug.hpp"
#include "cardinal_perception/srv/update_path_planning_mode.hpp"

#include <stats/stats.hpp>

#include <util.hpp>
#include <pub_map.hpp>
#include <geometry.hpp>
#include <ldrf_detector.hpp>
#include <synchronization.hpp>

#include "mapping.hpp"
#include "metrics.hpp"
#include "odometry.hpp"
#include "transform_sync.hpp"

#include "perception_presets.hpp"


namespace csm
{
namespace perception
{

class PerceptionNode :
    public rclcpp::Node
{
protected:
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
    using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
    using PathMsg = nav_msgs::msg::Path;

    using TagsTransformMsg = cardinal_perception::msg::TagsTransform;
    using ThreadMetricsMsg = cardinal_perception::msg::ThreadMetrics;
    using ProcessMetricsMsg = cardinal_perception::msg::ProcessMetrics;
    using TrajectoryFilterDebugMsg = cardinal_perception::msg::TrajectoryFilterDebug;

    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;

public:
    using OdomPointType = csm::perception::OdomPointType;
    using MappingPointType = csm::perception::MappingPointType;
    using FiducialPointType = csm::perception::FiducialPointType;
    using CollisionPointType = csm::perception::CollisionPointType;
    using RayDirectionType = csm::perception::RayDirectionType;
    using TraversibilityPointType = csm::perception::TraversibilityPointType;
    using TraversibilityMetaType = csm::perception::TraversibilityMetaType;

    using OdomPointCloudType = pcl::PointCloud<OdomPointType>;
    using MappingPointCloudType = pcl::PointCloud<MappingPointType>;
    using TraversibilityPointCloudType = pcl::PointCloud<TraversibilityPointType>;
    using TraversibilityMetaCloudType = pcl::PointCloud<TraversibilityMetaType>;

    using ClockType = std::chrono::system_clock;

protected:
    enum class ProcType : size_t
    {
        IMU_CB = 0,
        SCAN_CB,
        DET_CB,
        FID_CB,
        MAP_CB,
        TRAV_CB,
        PPLAN_CB,
        HANDLE_METRICS,
        MISC,
        NUM_ITEMS
    };

protected:
    #if TAG_DETECTION_ENABLED
    struct TagDetection
    {
        using Ptr = std::shared_ptr<TagDetection>;
        using ConstPtr = std::shared_ptr<const TagDetection>;

        util::geom::Pose3d pose;

        double time_point, pix_area, avg_range, rms;
        size_t num_tags;

        inline operator util::geom::Pose3d&() { return this->pose; }
    };
    #endif
    #if LFD_ENABLED
    struct FiducialResources
    {
        util::geom::PoseTf3f lidar_to_base;
        PointCloudMsg::ConstSharedPtr raw_scan;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
        uint32_t iteration_count;
    };
    #endif
    #if MAPPING_ENABLED
    struct MappingResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        PointCloudMsg::ConstSharedPtr raw_scan;
        OdomPointCloudType lo_buff;
        #if PERCEPTION_USE_NULL_RAY_DELETION
        std::vector<RayDirectionType> null_vecs;
        #endif
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
    };
    #endif
    #if TRAVERSABILITY_ENABLED
    struct TraversabilityResources
    {
        double stamp;
        Eigen::Vector3f search_min, search_max;
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        MappingPointCloudType::Ptr points;
    };
    #endif
    #if PATH_PLANNING_ENABLED
    struct PathPlanningResources
    {
        double stamp;
        Eigen::Vector3f local_bound_min, local_bound_max;
        util::geom::PoseTf3f base_to_odom;
        PoseStampedMsg target;
        TraversibilityPointCloudType trav_points;
        TraversibilityMetaCloudType trav_meta;
    };
    #endif

public:
    PerceptionNode();
    ~PerceptionNode();
    DECLARE_IMMOVABLE(PerceptionNode)

    void shutdown();

protected:
    void getParams();
    void initPubSubs();

    void handleStatusUpdate();
    void publishMetrics(double mem_usage, size_t n_threads, double cpu_temp);

    void imu_worker(const ImuMsg::SharedPtr& imu);
    IF_TAG_DETECTION_ENABLED(
    void detection_worker(const TagsTransformMsg::ConstSharedPtr& det); )
    void odometry_worker();
    IF_LFD_ENABLED(
    void fiducial_worker(); )
    IF_MAPPING_ENABLED(
    void mapping_worker(); )
    IF_TRAVERSABILITY_ENABLED(
    void traversability_worker(); )
    IF_PATH_PLANNING_ENABLED(
    void path_planning_worker(); )

private:
    int preprocess_scan(
        const PointCloudMsg::ConstSharedPtr& scan,
        util::geom::PoseTf3f& lidar_to_base_tf,
        OdomPointCloudType& lo_cloud,
        std::vector<RayDirectionType>& null_vecs,
        pcl::Indices& nan_indices,
        pcl::Indices& remove_indices );

    void scan_callback_internal(const PointCloudMsg::ConstSharedPtr& scan);
    IF_LFD_ENABLED(
    void fiducial_callback_internal(FiducialResources& buff); )
    IF_MAPPING_ENABLED(
    void mapping_callback_internal(MappingResources& buff); )
    IF_TRAVERSABILITY_ENABLED(
    void traversibility_callback_internal(TraversabilityResources& buff); )
    IF_PATH_PLANNING_ENABLED(
    void path_planning_callback_internal(PathPlanningResources& buffer); )

private:
// --- TRANSFORM UTILITEIS ----------------------------------------------------
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

// --- CORE COMPONENTS --------------------------------------------------------
    ImuIntegrator imu_samples;
    LidarOdometry lidar_odom;
    IF_LFD_ENABLED(
    LidarFiducialDetector<FiducialPointType> fiducial_detector; )
    IF_MAPPING_ENABLED(
    EnvironmentMap<MappingPointType, CollisionPointType> environment_map; )
    #if TAG_DETECTION_ENABLED
    TransformSynchronizer<TagDetection> transform_sync;
    #else
    TransformSynchronizer<util::geom::Pose3d> transform_sync;
    #endif

// --- SUBSCRIPTIONS/SERVICES/PUBLISHERS --------------------------------------
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
    rclcpp::Subscription<PointCloudMsg>::SharedPtr scan_sub;
    IF_TAG_DETECTION_ENABLED(
    rclcpp::Subscription<TagsTransformMsg>::SharedPtr detections_sub; )

    rclcpp::Service<UpdatePathPlanSrv>::SharedPtr path_plan_service;

    rclcpp::Publisher<TwistStampedMsg>::SharedPtr velocity_pub;
    rclcpp::Publisher<ProcessMetricsMsg>::SharedPtr proc_metrics_pub;
    rclcpp::Publisher<TrajectoryFilterDebugMsg>::SharedPtr traj_filter_debug_pub;
    rclcpp::Publisher<PathMsg>::SharedPtr path_plan_pub;

    util::FloatPublisherMap metrics_pub;
    util::PublisherMap<PointCloudMsg> scan_pub;
    util::PublisherMap<PoseStampedMsg> pose_pub;
    util::PublisherMap<ThreadMetricsMsg> thread_metrics_pub;

// --- FRAME IDS --------------------------------------------------------------
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

// --- STATE VARS -------------------------------------------------------------
    struct
    {
        // std::atomic<bool> has_rebiased{ false };
        std::atomic<bool> pplan_enabled{ true };
        std::atomic<bool> threads_running{ true };

        ClockType::time_point last_print_time, last_frames_time;
        std::mutex print_mtx;
    }
    state;

// --- PARAMETERIZED CONFIGS --------------------------------------------------
    struct
    {
        double metrics_pub_freq;
        IF_TAG_DETECTION_ENABLED( int tag_usage_mode; )

        Eigen::Vector3f
            base_link_crop_min,
            base_link_crop_max;
        bool use_crop_filter;

        double map_export_horizontal_range;
        double map_export_vertical_range;
    }
    param;

// --- MULTITHREADING RESOURCES -----------------------------------------------
    struct
    {
        ResourcePipeline<PointCloudMsg::ConstSharedPtr> odometry_resources;
        IF_LFD_ENABLED(
        ResourcePipeline<FiducialResources> fiducial_resources; )
        IF_MAPPING_ENABLED(
        ResourcePipeline<MappingResources> mapping_resources; )
        IF_TRAVERSABILITY_ENABLED(
        ResourcePipeline<TraversabilityResources> traversibility_resources; )
        IF_PATH_PLANNING_ENABLED(
        ResourcePipeline<PoseStampedMsg> pplan_target_notifier;
        ResourcePipeline<PathPlanningResources> path_planning_resources; )

        std::vector<std::thread> threads;
    }
    mt;

// --- METRICS ----------------------------------------------------------------
    struct
    {
        util::proc::ProcessMetrics process_utilization;
        MetricsManager<ProcType> manager;
    }
    metrics;

};

};
};
