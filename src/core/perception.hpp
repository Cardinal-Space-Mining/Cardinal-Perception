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

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <csm_metrics/stats.hpp>

#include <cardinal_perception/msg/tags_transform.hpp>
#include <cardinal_perception/msg/trajectory_filter_debug.hpp>
#include <cardinal_perception/srv/update_mining_eval_mode.hpp>
#include <cardinal_perception/srv/update_path_planning_mode.hpp>

#include <modules/kfc_map.hpp>
#include <modules/lidar_odom.hpp>
#include <modules/map_octree.hpp>
#include <modules/lf_detector.hpp>
#include <modules/path_planner.hpp>
#include <modules/traversibility_gen.hpp>

#include <util.hpp>
#include <pub_map.hpp>
#include <geometry.hpp>
#include <imu_integrator.hpp>
#include <transform_sync.hpp>
#include <synchronization.hpp>
#include <scan_preprocessor.hpp>

#include "perception_presets.hpp"


namespace csm
{
namespace perception
{

template<typename PointT, typename CollisionPointT>
using SparseMap = KFCMap<
    PointT,
    MapOctree<PointT, MAP_OCTREE_STORE_NORMALS>,
    CollisionPointT>;


class PerceptionNode : public rclcpp::Node
{
protected:
    using Float64Msg = std_msgs::msg::Float64;
    using ImuMsg = sensor_msgs::msg::Imu;
    using PointCloudMsg = sensor_msgs::msg::PointCloud2;
    using PoseStampedMsg = geometry_msgs::msg::PoseStamped;

    using TagsTransformMsg = cardinal_perception::msg::TagsTransform;
    using TrajectoryFilterDebugMsg =
        cardinal_perception::msg::TrajectoryFilterDebug;

    using UpdatePathPlanSrv = cardinal_perception::srv::UpdatePathPlanningMode;
    using UpdateMiningEvalSrv = cardinal_perception::srv::UpdateMiningEvalMode;

    using ProcessStatsCtx = csm::metrics::ProcessStats;

public:
    using OdomPointType = csm::perception::OdomPointType;
    using MappingPointType = csm::perception::MappingPointType;
    using FiducialPointType = csm::perception::FiducialPointType;
    using CollisionPointType = csm::perception::CollisionPointType;
    using RayDirectionType = csm::perception::RayDirectionType;
    using SphericalDirectionPointType =
        csm::perception::SphericalDirectionPointType;
    using TimestampPointType = csm::perception::TimestampPointType;
    using TraversibilityPointType = csm::perception::TraversibilityPointType;
    using TraversibilityMetaType = csm::perception::TraversibilityMetaType;

    using OdomPointCloudType = pcl::PointCloud<OdomPointType>;
    using MappingPointCloudType = pcl::PointCloud<MappingPointType>;
    using TraversibilityPointCloudType =
        pcl::PointCloud<TraversibilityPointType>;
    using TraversibilityMetaCloudType = pcl::PointCloud<TraversibilityMetaType>;

    using ClockType = std::chrono::system_clock;

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
#if TRAVERSIBILITY_ENABLED
    struct TraversibilityResources
    {
        double stamp;
        Eigen::Vector3f search_min, search_max;
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        MappingPointCloudType points;
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
    void getParams(void* = nullptr);
    void initPubSubs(void* = nullptr);
    void printStartup(void* = nullptr);

    void imu_worker(const ImuMsg::SharedPtr& imu);
    IF_TAG_DETECTION_ENABLED(
        void detection_worker(const TagsTransformMsg::ConstSharedPtr& det);)
    void odometry_worker();
    IF_LFD_ENABLED(void fiducial_worker();)
    IF_MAPPING_ENABLED(void mapping_worker();)
    IF_TRAVERSIBILITY_ENABLED(void traversibility_worker();)
    IF_PATH_PLANNING_ENABLED(void path_planning_worker();)

private:
    void scan_callback_internal(const PointCloudMsg::ConstSharedPtr& scan);
    IF_LFD_ENABLED(void fiducial_callback_internal(FiducialResources& buff);)
    IF_MAPPING_ENABLED(void mapping_callback_internal(MappingResources& buff);)
    IF_TRAVERSIBILITY_ENABLED(
        void traversibility_callback_internal(TraversibilityResources& buff);)
    IF_PATH_PLANNING_ENABLED(
        void path_planning_callback_internal(PathPlanningResources& buffer);)

private:
    // --- TRANSFORM UTILITEIS -------------------------------------------------
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // --- CORE COMPONENTS -----------------------------------------------------
    ImuIntegrator<> imu_samples;
    ScanPreprocessor<
        OdomPointType,
        RayDirectionType,
        SphericalDirectionPointType,
        TimestampPointType>
        scan_preproc;
    LidarOdometry<OdomPointType> lidar_odom;
    IF_LFD_ENABLED(LidarFiducialDetector<FiducialPointType> fiducial_detector;)
    IF_MAPPING_ENABLED(
        SparseMap<MappingPointType, CollisionPointType> sparse_map;)
#if TAG_DETECTION_ENABLED
    TransformSynchronizer<TagDetection> transform_sync;
#else
    TransformSynchronizer<util::geom::Pose3d> transform_sync;
#endif
    IF_TRAVERSIBILITY_ENABLED(
        TraversibilityGenerator<TraversibilityPointType, TraversibilityMetaType>
            trav_gen;)
    IF_PATH_PLANNING_ENABLED(
        PathPlanner<TraversibilityPointType, TraversibilityMetaType>
            path_planner;)

    // --- SUBSCRIPTIONS/SERVICES/PUBLISHERS -----------------------------------
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub;
    rclcpp::Subscription<PointCloudMsg>::SharedPtr scan_sub;
    IF_TAG_DETECTION_ENABLED(
        rclcpp::Subscription<TagsTransformMsg>::SharedPtr detections_sub;)

    rclcpp::Service<UpdatePathPlanSrv>::SharedPtr path_plan_service;

    rclcpp::TimerBase::SharedPtr proc_stats_timer;

    util::GenericPubMap generic_pub;
    util::PubMap<Float64Msg> metrics_pub;
    util::PubMap<PointCloudMsg> scan_pub;
    util::PubMap<PoseStampedMsg> pose_pub;

    // --- FRAME IDS -----------------------------------------------------------
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    // --- STATE VARS ----------------------------------------------------------
    struct
    {
        // std::atomic<bool> has_rebiased{ false };
        std::atomic<bool> pplan_enabled{false};
        std::atomic<bool> threads_running{true};
    }  //
    state;

    // --- PARAMETERIZED CONFIGS -----------------------------------------------
    struct
    {
        IF_TAG_DETECTION_ENABLED(int tag_usage_mode;)

        double map_crop_horizontal_range;
        double map_crop_vertical_range;
        double map_export_horizontal_range;
        double map_export_vertical_range;
    }  //
    param;

    // --- MULTITHREADING RESOURCES --------------------------------------------
    struct
    {
        ResourcePipeline<PointCloudMsg::ConstSharedPtr> odometry_resources;
        IF_LFD_ENABLED(ResourcePipeline<FiducialResources> fiducial_resources;)
        IF_MAPPING_ENABLED(
            ResourcePipeline<MappingResources> mapping_resources;)
        IF_TRAVERSIBILITY_ENABLED(
            ResourcePipeline<TraversibilityResources> traversibility_resources;)
        IF_PATH_PLANNING_ENABLED(
            ResourcePipeline<PoseStampedMsg> pplan_target_notifier;
            ResourcePipeline<PathPlanningResources> path_planning_resources;)

        std::vector<std::thread> threads;
    }  //
    mt;

    // --- METRICS -------------------------------------------------------------
    ProcessStatsCtx process_stats;
};

};  // namespace perception
};  // namespace csm
