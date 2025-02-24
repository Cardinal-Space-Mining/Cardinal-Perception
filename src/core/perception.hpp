/*******************************************************************************
*   Copyright (C) 2024-2025 Cardinal Space Mining Club                         *
*                                                                              *
*   Unless required by applicable law or agreed to in writing, software        *
*   distributed under the License is distributed on an "AS IS" BASIS,          *
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
*   See the License for the specific language governing permissions and        *
*   limitations under the License.                                             *
*                                                                              *
*                                ;xxxxxxx:                                     *
*                               ;$$$$$$$$$       ...::..                       *
*                               $$$$$$$$$$x   .:::::::::::..                   *
*                            x$$$$$$$$$$$$$$::::::::::::::::.                  *
*                        :$$$$$&X;      .xX:::::::::::::.::...                 *
*                .$$Xx++$$$$+  :::.     :;:   .::::::.  ....  :                *
*               :$$$$$$$$$  ;:      ;xXXXXXXXx  .::.  .::::. .:.               *
*              :$$$$$$$$: ;      ;xXXXXXXXXXXXXx: ..::::::  .::.               *
*             ;$$$$$$$$ ::   :;XXXXXXXXXXXXXXXXXX+ .::::.  .:::                *
*              X$$$$$X : +XXXXXXXXXXXXXXXXXXXXXXXX; .::  .::::.                *
*               .$$$$ :xXXXXXXXXXXXXXXXXXXXXXXXXXXX.   .:::::.                 *
*                X$$X XXXXXXXXXXXXXXXXXXXXXXXXXXXXx:  .::::.                   *
*                $$$:.XXXXXXXXXXXXXXXXXXXXXXXXXXX  ;; ..:.                     *
*                $$& :XXXXXXXXXXXXXXXXXXXXXXXX;  +XX; X$$;                     *
*                $$$: XXXXXXXXXXXXXXXXXXXXXX; :XXXXX; X$$;                     *
*                X$$X XXXXXXXXXXXXXXXXXXX; .+XXXXXXX; $$$                      *
*                $$$$ ;XXXXXXXXXXXXXXX+  +XXXXXXXXx+ X$$$+                     *
*              x$$$$$X ;XXXXXXXXXXX+ :xXXXXXXXX+   .;$$$$$$                    *
*             +$$$$$$$$ ;XXXXXXx;;+XXXXXXXXX+    : +$$$$$$$$                   *
*              +$$$$$$$$: xXXXXXXXXXXXXXX+      ; X$$$$$$$$                    *
*               :$$$$$$$$$. +XXXXXXXXX;      ;: x$$$$$$$$$                     *
*               ;x$$$$XX$$$$+ .;+X+      :;: :$$$$$xX$$$X                      *
*              ;;;;;;;;;;X$$$$$$$+      :X$$$$$$&.                             *
*              ;;;;;;;:;;;;;x$$$$$$$$$$$$$$$$x.                                *
*              :;;;;;;;;;;;;.  :$$$$$$$$$$X                                    *
*               .;;;;;;;;:;;    +$$$$$$$$$                                     *
*                 .;;;;;;.       X$$$$$$$:                                     *
*                                                                              *
*******************************************************************************/

#pragma once

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

#ifndef USE_TAG_DETECTION_PIPELINE
#define USE_TAG_DETECTION_PIPELINE 0
#endif
#ifndef USE_LFD_PIPELINE
#define USE_LFD_PIPELINE (!USE_TAG_DETECTION_PIPELINE)
#endif
#if USE_TAG_DETECTION_PIPELINE && USE_LFD_PIPELINE
static_assert(false, "Tag detection and lidar fiducial pipelines are mutually exclusive");
#endif


namespace csm
{
namespace perception
{

#if USE_TAG_DETECTION_PIPELINE > 0
    #define IF_TAG_DETECTION_ENABLED(x) x
    #define TAG_DETECTION_ENABLED 1
#else
    #define IF_TAG_DETECTION_ENABLED(...)
    #define TAG_DETECTION_ENABLED 0
#endif
#if USE_LFD_PIPELINE > 0
    #define IF_LFD_ENABLED(x) x
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
    struct MappingResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
        pcl::PointCloud<OdomPointType> lo_buff;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
    };
    struct TraversibilityResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        pcl::PointCloud<MappingPointType>::Ptr points;
        double stamp;
    };

protected:
    void getParams();
    void initPubSubs();

    void handleStatusUpdate();
    void publishMetrics(double mem_usage, size_t n_threads);

IF_TAG_DETECTION_ENABLED(
    void detection_worker(const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det); )
    void imu_worker(const sensor_msgs::msg::Imu::SharedPtr& imu);
    void odometry_worker();
IF_LFD_ENABLED(
    void fiducial_worker(); )
    void mapping_worker();
    void traversibility_worker();

    void scan_callback_internal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
IF_LFD_ENABLED(
    void fiducial_callback_internal(FiducialResources& buff); )
    void mapping_callback_internal(MappingResources& buff);
    void traversibility_callback_internal(TraversibilityResources& buff);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    LidarOdometry lidar_odom;
    EnvironmentMap<MappingPointType, CollisionPointType> environment_map;
    LidarFiducialDetector<FiducialPointType> fiducial_detector;
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
    IF_TAG_DETECTION_ENABLED(
        int use_tag_detections; )
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
        ResourcePipeline<MappingResources> mapping_resources;
    IF_LFD_ENABLED(
        ResourcePipeline<FiducialResources> fiducial_resources; )
        ResourcePipeline<TraversibilityResources> traversibility_resources;

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
        MAP_CB,
    #if LFD_ENABLED
        FID_CB,
    #endif
        TRAV_CB,
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
        util::proc::ThreadMetrics imu_thread, scan_thread, mapping_thread, trav_thread;
    IF_TAG_DETECTION_ENABLED(
        util::proc::ThreadMetrics det_thread; )
    IF_LFD_ENABLED(
        util::proc::ThreadMetrics fiducial_thread; )
        util::proc::ProcessMetrics process_utilization;

        std::unordered_map<std::thread::id, ProcDurationArray> thread_metric_durations;
        std::mutex thread_procs_mtx;
    }
    metrics;

private:
    void appendThreadProcTime(ProcType type, double dt);

    ClockType::time_point appendMetricStartTime(ProcType type);
    ClockType::time_point appendMetricStopTime(ProcType type);

    template<bool S>
    friend ClockType::time_point appendMetricTimeCommon(PerceptionNode*, ProcType);

};

};
};
