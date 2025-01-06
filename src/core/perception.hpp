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

#ifndef USE_GTSAM_PGO
#define USE_GTSAM_PGO 0
#endif
#if USE_GTSAM_PGO > 0
#define GEOM_UTIL_USE_GTSAM 1
#else
#define GEOM_UTIL_USE_GTSAM 0
#endif

#ifndef ENABLE_TAG_DETECTION
#define ENABLE_TAG_DETECTION 0
#endif

#include "point_def.hpp"
#include "util.hpp"
#include "synchronization.hpp"
#include "pub_map.hpp"
#include "geometry.hpp"
#include "trajectory_filter.hpp"
#include "map_octree.hpp"

#include "mapping.hpp"

#include "cardinal_perception/msg/tags_transform.hpp"
#include "cardinal_perception/msg/process_metrics.hpp"
#include "cardinal_perception/msg/thread_metrics.hpp"
#include "cardinal_perception/msg/trajectory_filter_debug.hpp"

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
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>
#include <condition_variable>

#include <sys/times.h>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <nav_msgs/msg/path.hpp>

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nano_gicp/nano_gicp.hpp>
#include <stats/stats.hpp>
// #include <ikd_tree/ikd_tree.hpp>


namespace csm
{
namespace perception
{

#if ENABLE_TAG_DETECTION > 0
    #define IF_TAG_DETECTION_ENABLED(x) x
    #define TAG_DETECTION_ENABLED 1
#else
    #define IF_TAG_DETECTION_ENABLED(...)
    #define TAG_DETECTION_ENABLED 0
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

    void shutdown();

protected:
    class LidarOdometry
    {
        friend PerceptionNode;
        using PointType = PerceptionNode::OdomPointType;
        using PointCloudType = pcl::PointCloud<PointType>;

        static_assert(std::is_same<PointType, pcl::PointXYZ>::value);

    public:
        LidarOdometry(PerceptionNode* inst);
        ~LidarOdometry() = default;

    public:
        struct IterationStatus
        {
            union
            {
                struct
                {
                    union
                    {
                        struct
                        {
                            bool odom_updated : 1;
                            bool keyframe_init : 1;
                            bool new_keyframe : 1;
                        };
                        uint32_t status_bits;
                    };
                    uint32_t total_keyframes;
                };
                int64_t data;
            };

            inline IterationStatus(int64_t v = 0) : data{ v } {}
            inline operator int64_t() const { return this->data; }
            inline operator bool() const { return static_cast<bool>(this->data); }
        };

    public:
        void processImu(const sensor_msgs::msg::Imu& imu);
        IterationStatus processScan(
            const PointCloudType& scan,
            double stamp,
            util::geom::PoseTf3f& odom_tf );

        void publishDebugScans(IterationStatus proc_status);

    protected:
        void getParams();

        bool preprocessPoints(const PointCloudType& scan);
        void initializeInputTarget();
        void setInputSources();

        void initializeDLO();
        void gravityAlign();

        void getNextPose();
        void integrateIMU();

        void propagateS2S(const Eigen::Matrix4f& T);
        void propagateS2M();

        void setAdaptiveParams(const PointCloudType& scan);

        void transformCurrentScan();
        void updateKeyframes();
        void computeConvexHull();
        void computeConcaveHull();
        void pushSubmapIndices(
            const std::vector<float>& dists,
            int k,
            const std::vector<int>& frames );
        void getSubmapKeyframes();

    protected:
        struct XYZd // TODO: use Eigen::Vector3d
        {
            double x;
            double y;
            double z;
        };
        struct ImuBias
        {
            XYZd gyro;
            XYZd accel;
        };
        struct ImuMeas
        {
            double stamp;
            XYZd ang_vel;
            XYZd lin_accel;
        };
        struct OrientMeas
        {
            double stamp;
            Eigen::Quaterniond quat;
        };

    private:
        PerceptionNode* pnode;

        PointCloudType::Ptr current_scan, current_scan_t;
        PointCloudType::Ptr target_cloud;
        PointCloudType scratch_cloud;

        pcl::CropBox<PointType> crop;
        pcl::VoxelGrid<PointType> vf_scan;
        pcl::VoxelGrid<PointType> vf_submap;

        pcl::ConvexHull<PointType> convex_hull;
        pcl::ConcaveHull<PointType> concave_hull;
        std::vector<int> keyframe_convex;
        std::vector<int> keyframe_concave;

        PointCloudType::Ptr keyframe_cloud;
        PointCloudType::Ptr keyframe_points;
        std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, PointCloudType::Ptr>> keyframes;  // TODO: use kdtree for positions
        std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

        PointCloudType::Ptr submap_cloud;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;
        std::unordered_set<int> submap_kf_idx_curr;
        std::unordered_set<int> submap_kf_idx_prev;

        nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
        nano_gicp::NanoGICP<PointType, PointType> gicp;

        // std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> trajectory;

        boost::circular_buffer<ImuMeas> imu_buffer; // TODO: use std::deque and make this a TSQ
        util::tsq::TSQ<Eigen::Quaterniond> orient_buffer;

        struct
        {
            std::atomic<bool> dlo_initialized;
            std::atomic<bool> imu_calibrated;
            std::atomic<bool> submap_hasChanged;

            uint32_t num_keyframes;

            double range_avg_lpf;
            double range_stddev_lpf;
            double adaptive_voxel_size;

            double first_imu_time;
            double curr_frame_stamp;
            double prev_frame_stamp;

            Eigen::Vector3f origin;

            Eigen::Matrix4f T;
            Eigen::Matrix4f T_s2s, T_s2s_prev;

            Eigen::Vector3f pose;
            Eigen::Matrix3f rotSO3;
            Eigen::Quaternionf rotq, last_rotq;

            Eigen::Matrix4f imu_SE3;

            ImuBias imu_bias;
            ImuMeas imu_meas;

            std::mutex imu_mtx, scan_mtx;
        }
        state;

        struct
        {
            bool gravity_align_;

            double keyframe_thresh_dist_;
            double keyframe_thresh_rot_;

            int submap_knn_;
            int submap_kcv_;
            int submap_kcc_;
            double submap_concave_alpha_;

            bool initial_pose_use_;
            Eigen::Vector3d initial_position_;
            Eigen::Quaterniond initial_orientation_;

            bool vf_scan_use_;
            double vf_scan_res_;

            bool vf_submap_use_;
            double vf_submap_res_;

            double adaptive_voxel_range_coeff_;
            double adaptive_voxel_stddev_coeff_;
            double adaptive_voxel_offset_;
            double adaptive_voxel_floor_;
            double adaptive_voxel_ceil_;
            double adaptive_voxel_precision_;

            bool adaptive_params_use_;
            double adaptive_params_lpf_coeff_;

            bool imu_use_;
            bool imu_use_orientation_;
            int imu_calib_time_;
            int imu_buffer_size_;

            int gicp_min_num_points_;

            int gicps2s_k_correspondences_;
            double gicps2s_max_corr_dist_;
            int gicps2s_max_iter_;
            double gicps2s_transformation_ep_;
            double gicps2s_euclidean_fitness_ep_;
            int gicps2s_ransac_iter_;
            double gicps2s_ransac_inlier_thresh_;

            int gicps2m_k_correspondences_;
            double gicps2m_max_corr_dist_;
            int gicps2m_max_iter_;
            double gicps2m_transformation_ep_;
            double gicps2m_euclidean_fitness_ep_;
            int gicps2m_ransac_iter_;
            double gicps2m_ransac_inlier_thresh_;
        }
        param;

        static bool comparatorImu(const ImuMeas& m1, const ImuMeas& m2) { return (m1.stamp < m2.stamp); };

    };

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

    struct MappingResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
        pcl::PointCloud<OdomPointType> lo_buff;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
    };
    struct FiducialResources
    {
        util::geom::PoseTf3f lidar_to_base, base_to_odom;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr raw_scan;
        std::shared_ptr<const pcl::Indices> nan_indices, remove_indices;
    };
    struct TraversibilityResources
    {
        pcl::PointCloud<MappingPointType>::Ptr points;
    };

protected:
    void getParams();
    void initPubSubs();

    void handleStatusUpdate();
    void publishMetrics(double mem_usage, size_t n_threads);
    void sendTf(const builtin_interfaces::msg::Time& stamp, bool needs_lock = false);

IF_TAG_DETECTION_ENABLED(
    void detection_worker(const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det); )
    void imu_worker(const sensor_msgs::msg::Imu::SharedPtr imu);
    void odometry_worker();
    void mapping_worker();
    void fiducial_worker();
    void traversibility_worker();

    void scan_callback_internal(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
    void mapping_callback_internal(MappingResources& buff);
    void fiducial_callback_internal(FiducialResources& buff);
    void traversibility_callback_internal(TraversibilityResources& buff);

private:
    LidarOdometry lidar_odom;
IF_TAG_DETECTION_ENABLED(
    TrajectoryFilter<TagDetection> trajectory_filter; )
    EnvironmentMap<MappingPointType, CollisionPointType> environment_map;
    FiducialMap<FiducialPointType, CollisionPointType> fiducial_map;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
IF_TAG_DETECTION_ENABLED(
    rclcpp::Subscription<cardinal_perception::msg::TagsTransform>::SharedPtr detections_sub; )

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub, map_cloud_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Publisher<cardinal_perception::msg::ProcessMetrics>::SharedPtr proc_metrics_pub;
IF_TAG_DETECTION_ENABLED(
    rclcpp::Publisher<cardinal_perception::msg::TrajectoryFilterDebug>::SharedPtr traj_filter_debug_pub; )

    util::FloatPublisherMap metrics_pub;
    // util::PublisherMap<geometry_msgs::msg::PoseStamped> pose_pub;
    util::PublisherMap<sensor_msgs::msg::PointCloud2> scan_pub;
    util::PublisherMap<cardinal_perception::msg::ThreadMetrics> thread_metrics_pub;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    struct
    {
        std::atomic<bool> has_rebiased{ false };
        std::atomic<bool> threads_running{ true };

        util::geom::PoseTf3d map_tf, odom_tf;
        double last_odom_stamp;

        std::mutex tf_mtx, print_mtx;
        ClockType::time_point last_print_time, last_frames_time;
    }
    state;

    struct
    {
        double metrics_pub_freq;
    IF_TAG_DETECTION_ENABLED(
        int use_tag_detections; )
        bool rebias_tf_pub_prereq;
        bool rebias_scan_pub_prereq;

        bool publish_odom_debug;

        bool use_crop_filter;
        Eigen::Vector3f crop_min, crop_max;
    }
    param;

    struct
    {
        ResourcePipeline<sensor_msgs::msg::PointCloud2::ConstSharedPtr> odometry_resources;
        ResourcePipeline<MappingResources> mapping_resources;
        ResourcePipeline<FiducialResources> fiducial_resources;
        ResourcePipeline<TraversibilityResources> traversibility_resources;

        std::vector<std::thread> threads;
    }
    mt;

    enum class ProcType : size_t
    {
        IMU_CB = 0,
        SCAN_CB,
    #if TAG_DETECTION_ENABLED
        DET_CB,
    #endif
        MAP_CB,
        FID_CB,
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
        util::proc::ThreadMetrics imu_thread, scan_thread, mapping_thread, fiducial_thread, trav_thread;
    IF_TAG_DETECTION_ENABLED(
        util::proc::ThreadMetrics det_thread; )
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
