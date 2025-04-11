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
#include <point_def.hpp>    // needs to come before PCL includes when using custom types

#include <array>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <memory>
#include <utility>
#include <functional>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// #include <nav_msgs/msg/path.hpp>

#include "cardinal_perception/msg/tags_transform.hpp"
#include "cardinal_perception/msg/process_metrics.hpp"
#include "cardinal_perception/msg/thread_metrics.hpp"
#include "cardinal_perception/msg/trajectory_filter_debug.hpp"

#include <nano_gicp/nano_gicp.hpp>
#include <stats/stats.hpp>

#include <util.hpp>
#include <synchronization.hpp>
#include <pub_map.hpp>
#include <geometry.hpp>
#include <trajectory_filter.hpp>
#include <map_octree.hpp>
#include <ldrf_detector.hpp>

#include "odometry.hpp"
#include "mapping.hpp"
#include "transform_sync.hpp"


#ifndef PERCEPTION_PRINT_STATUS_DISPLAY
#define PERCEPTION_PRINT_STATUS_DISPLAY 1
#endif

#ifndef PERCEPTION_PUBLISH_GRAV_ESTIMATION
#define PERCEPTION_PUBLISH_GRAV_ESTIMATION 1
#endif

#ifndef PERCEPTION_PUBLISH_LIO_DEBUG
#define PERCEPTION_PUBLISH_LIO_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_LFD_DEBUG
#define PERCEPTION_PUBLISH_LFD_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_TRJF_DEBUG
#define PERCEPTION_PUBLISH_TRJF_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_TRAV_DEBUG
#define PERCEPTION_PUBLISH_TRAV_DEBUG 1
#endif

#ifndef PERCEPTION_PUBLISH_FULL_MAP
#define PERCEPTION_PUBLISH_FULL_MAP 0
#endif

#ifndef PERCEPTION_USE_SCAN_DESKEW
#define PERCEPTION_USE_SCAN_DESKEW 1
#endif

#ifndef PERCEPTION_ENABLE_MAPPING
#define PERCEPTION_ENABLE_MAPPING 1
#endif

#ifndef PERCEPTION_ENABLE_TRAVERSABILITY
#define PERCEPTION_ENABLE_TRAVERSABILITY (PERCEPTION_ENABLE_MAPPING)
#endif

#ifndef PERCEPTION_USE_TAG_DETECTION_PIPELINE
#define PERCEPTION_USE_TAG_DETECTION_PIPELINE 0
#endif
#ifndef PERCEPTION_USE_LFD_PIPELINE
#define PERCEPTION_USE_LFD_PIPELINE (!PERCEPTION_USE_TAG_DETECTION_PIPELINE)
#endif


#if ((PERCEPTION_USE_TAG_DETECTION_PIPELINE) && (PERCEPTION_USE_LFD_PIPELINE))
static_assert(false, "Tag detection and lidar fiducial pipelines are mutually exclusive. You may only enable one at a time.");
#endif
#if ((PERCEPTION_ENABLE_TRAVERSABILITY) && !(PERCEPTION_ENABLE_MAPPING))
#undef PERCEPTION_ENABLE_TRAVERSABILITY
#define PERCEPTION_ENABLE_TRAVERSABILITY 0
#endif


namespace csm
{
namespace perception
{

#if PERCEPTION_ENABLE_MAPPING > 0
    #define IF_MAPPING_ENABLED(...) __VA_ARGS__
    #define MAPPING_ENABLED 1
#else
    #define IF_MAPPING_ENABLED(...)
    #define MAPPING_ENABLED 0
#endif
#if PERCEPTION_ENABLE_TRAVERSABILITY > 0
    #define IF_TRAVERSABILITY_ENABLED(...) __VA_ARGS__
    #define TRAVERSABILITY_ENABLED 1
#else
    #define IF_TRAVERSABILITY_ENABLED(...)
    #define TRAVERSABILITY_ENABLED 0
#endif

#if PERCEPTION_USE_TAG_DETECTION_PIPELINE > 0
    #define IF_TAG_DETECTION_ENABLED(...) __VA_ARGS__
    #define TAG_DETECTION_ENABLED 1
#else
    #define IF_TAG_DETECTION_ENABLED(...)
    #define TAG_DETECTION_ENABLED 0
#endif
#if PERCEPTION_USE_LFD_PIPELINE > 0
    #define IF_LFD_ENABLED(...) __VA_ARGS__
    #define LFD_ENABLED 1
#else
    #define IF_LFD_ENABLED(...)
    #define LFD_ENABLED 0
#endif


class PerceptionNode :
    public rclcpp::Node
{
public:
    using OdomPointType = csm::perception::OdomPointType;
    using MappingPointType = csm::perception::MappingPointType;
    using FiducialPointType = csm::perception::FiducialPointType;
    using CollisionPointType = csm::perception::CollisionPointType;

    using OdomPointCloudType = pcl::PointCloud<OdomPointType>;
    using MappingPointCloudType = pcl::PointCloud<MappingPointType>;

    using ClockType = std::chrono::system_clock;

public:
    PerceptionNode();
    ~PerceptionNode();
    DECLARE_IMMOVABLE(PerceptionNode)

    void shutdown();

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
        sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
        uint32_t iteration_count;
    };
    #endif
    #if MAPPING_ENABLED
    struct MappingResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
        OdomPointCloudType lo_buff;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
    };
    #endif
    #if TRAVERSABILITY_ENABLED
    struct TraversabilityResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        MappingPointCloudType::Ptr points;
        double stamp;
    };
    #endif

protected:
    void getParams();
    void initPubSubs();

    void handleStatusUpdate();
    void publishMetrics(double mem_usage, size_t n_threads);

    void imu_worker(const sensor_msgs::msg::Imu::SharedPtr& imu);
    void odometry_worker();

    IF_TAG_DETECTION_ENABLED(   void detection_worker(
                                    const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det ); )
    IF_LFD_ENABLED(             void fiducial_worker(); )
    IF_MAPPING_ENABLED(         void mapping_worker(); )
    IF_TRAVERSABILITY_ENABLED(  void traversability_worker(); )

private:
    int preprocess_scan(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan,
        util::geom::PoseTf3f& lidar_to_base_tf,
        OdomPointCloudType& lo_cloud,
        pcl::Indices& nan_indices,
        pcl::Indices& remove_indices );

    void scan_callback_internal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);

    IF_LFD_ENABLED(             void fiducial_callback_internal(FiducialResources& buff); )
    IF_MAPPING_ENABLED(         void mapping_callback_internal(MappingResources& buff); )
    IF_TRAVERSABILITY_ENABLED(  void traversibility_callback_internal(TraversabilityResources& buff); )

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    ImuIntegrator imu_samples;
    LidarOdometry lidar_odom;
    IF_LFD_ENABLED( LidarFiducialDetector<FiducialPointType> fiducial_detector; )
    IF_MAPPING_ENABLED( EnvironmentMap<MappingPointType, CollisionPointType> environment_map; )
    #if TAG_DETECTION_ENABLED
    TransformSynchronizer<TagDetection> transform_sync;
    #else
    TransformSynchronizer<util::geom::Pose3d> transform_sync;
    #endif

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    IF_TAG_DETECTION_ENABLED(
    rclcpp::Subscription<cardinal_perception::msg::TagsTransform>::SharedPtr detections_sub; )

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub, map_cloud_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<cardinal_perception::msg::ProcessMetrics>::SharedPtr proc_metrics_pub;
    rclcpp::Publisher<cardinal_perception::msg::TrajectoryFilterDebug>::SharedPtr traj_filter_debug_pub;

    util::FloatPublisherMap metrics_pub;
    util::PublisherMap<geometry_msgs::msg::PoseStamped> pose_pub;
    util::PublisherMap<sensor_msgs::msg::PointCloud2> scan_pub;
    util::PublisherMap<cardinal_perception::msg::ThreadMetrics> thread_metrics_pub;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    struct
    {
        // std::atomic<bool> has_rebiased{ false };
        std::atomic<bool> threads_running{ true };

        std::mutex print_mtx;
        ClockType::time_point last_print_time, last_frames_time;
    }
    state;

    struct
    {
        double metrics_pub_freq;
        IF_TAG_DETECTION_ENABLED( int use_tag_detections; )
        // bool rebias_tf_pub_prereq;
        // bool rebias_scan_pub_prereq;

        bool publish_odom_debug;

        bool use_crop_filter;
        Eigen::Vector3f crop_min, crop_max;

        double map_export_horizontal_range;
        double map_export_vertical_range;
    }
    param;

    struct
    {
        ResourcePipeline<sensor_msgs::msg::PointCloud2::ConstSharedPtr> odometry_resources;
        IF_LFD_ENABLED( ResourcePipeline<FiducialResources> fiducial_resources; )
        IF_MAPPING_ENABLED( ResourcePipeline<MappingResources> mapping_resources; )
        IF_TRAVERSABILITY_ENABLED( ResourcePipeline<TraversabilityResources> traversibility_resources; )

        std::vector<std::thread> threads;
    }
    mt;

private:
    enum class ProcType : size_t
    {
        IMU_CB = 0,
        SCAN_CB,
        #if TAG_DETECTION_ENABLED
        DET_CB,
        #endif
        #if LFD_ENABLED
        FID_CB,
        #endif
        #if MAPPING_ENABLED
        MAP_CB,
        #endif
        #if TRAVERSABILITY_ENABLED
        TRAV_CB,
        #endif
        HANDLE_METRICS,
        MISC,
        NUM_ITEMS
    };
    struct ProcDurationArray :
        std::array<
            std::pair<double, ClockType::time_point>,
            static_cast<size_t>(ProcType::NUM_ITEMS) >
    {
        inline ProcDurationArray()
        {
            this->fill({ 0., ClockType::time_point::min() });
        }
        ~ProcDurationArray() = default;
    };

    struct
    {
        util::proc::ProcessMetrics process_utilization;

        util::proc::ThreadMetrics imu_thread, scan_thread;
        IF_TAG_DETECTION_ENABLED(   util::proc::ThreadMetrics det_thread; )
        IF_LFD_ENABLED(             util::proc::ThreadMetrics fiducial_thread; )
        IF_MAPPING_ENABLED(         util::proc::ThreadMetrics mapping_thread; )
        IF_TRAVERSABILITY_ENABLED(  util::proc::ThreadMetrics trav_thread; )

        std::unordered_map<std::thread::id, ProcDurationArray> thread_metric_durations;
        std::mutex thread_procs_mtx;
    }
    metrics;

private:
    void appendThreadProcTime(ProcType type, double dt);

    ClockType::time_point appendMetricStartTime(ProcType type);
    ClockType::time_point appendMetricStopTime(ProcType type);

    template<bool Mode>
    friend ClockType::time_point appendMetricTimeCommon(PerceptionNode*, ProcType);

};

};
};
