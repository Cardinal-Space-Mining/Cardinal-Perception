#pragma once

#ifndef USE_GTSAM_PGO
#define USE_GTSAM_PGO 0
#endif
#if USE_GTSAM_PGO > 0
#define GEOM_UTIL_USE_GTSAM 1
#else
#define GEOM_UTIL_USE_GTSAM 0
#endif

#include "point_def.hpp"
#include "util.hpp"
#include "synchronization.hpp"
#include "pub_map.hpp"
#include "geometry.hpp"
#include "trajectory_filter.hpp"
#include "stats.hpp"

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

#include <sys/times.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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

#if USE_GTSAM_PGO > 0
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#endif

#include <nano_gicp/nano_gicp.hpp>
#include <ikd_tree/ikd_tree.hpp>


struct TagDetection
{
    using Ptr = std::shared_ptr<TagDetection>;
    using ConstPtr = std::shared_ptr<const TagDetection>;

    util::geom::Pose3d pose;

    double time_point, pix_area, avg_range, rms;
    size_t num_tags;

    inline operator util::geom::Pose3d&() { return this->pose; }
};

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode();
    ~PerceptionNode() = default;

protected:
    class DLOdom
    {
    friend PerceptionNode;
    public:
        using PointType = cardinal_perception::PointType;

        DLOdom(PerceptionNode* inst);
        ~DLOdom() = default;

    public:
        int64_t processScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan, util::geom::PoseTf3d& odom_tf);
        void processImu(const sensor_msgs::msg::Imu& imu);
        void iterateMapping(Eigen::Vector3d lvp_offset = Eigen::Vector3d::Zero());

        const pcl::PointCloud<PointType>::Ptr& filteredScan();
        void publishFilteredScan(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub);
        void publishDebugScans();

    protected:
        void getParams();

        void preprocessPoints();
        void initializeInputTarget();
        void setInputSources();

        void initializeDLO();
        void gravityAlign();

        void getNextPose();
        void integrateIMU();

        void propagateS2S(const Eigen::Matrix4d& T);
        void propagateS2M();

        void setAdaptiveParams();

        void transformCurrentScan();
        void updateKeyframes();
        void computeConvexHull();
        void computeConcaveHull();
        void pushSubmapIndices(const std::vector<float>& dists, int k, const std::vector<int>& frames);
        void getSubmapKeyframes();

    protected:
        struct XYZd
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

        // pcl::PointCloud<PointType>::Ptr source_cloud;
        pcl::PointCloud<PointType>::Ptr current_scan, current_scan_t;
        pcl::PointCloud<PointType>::Ptr filtered_scan, filtered_scan_t;
        pcl::PointCloud<PointType>::Ptr target_cloud;

        pcl::CropBox<PointType> crop;
        pcl::VoxelGrid<PointType> vf_scan;
        pcl::VoxelGrid<PointType> vf_submap;

        pcl::ConvexHull<PointType> convex_hull;
        pcl::ConcaveHull<PointType> concave_hull;
        std::vector<int> keyframe_convex;
        std::vector<int> keyframe_concave;

        // pcl::PointCloud<PointType>::Ptr keyframes_cloud;
        pcl::PointCloud<PointType>::Ptr keyframe_cloud;
        std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Quaterniond>, pcl::PointCloud<PointType>::Ptr>> keyframes;  // TODO: use kdtree for positions
        std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

        pcl::PointCloud<PointType>::Ptr submap_cloud;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;
        std::vector<int> submap_kf_idx_curr;
        std::vector<int> submap_kf_idx_prev;

        nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
        nano_gicp::NanoGICP<PointType, PointType> gicp;

        struct
        {
            // ikd::IKDTree<cardinal_perception::CollisionPointType> collision_kdtree;
            pcl::KdTreeFLANN<cardinal_perception::CollisionPointType> collision_kdtree;
            pcl::PointCloud<cardinal_perception::CollisionPointType>::Ptr submap_ranges;
            pcl::PointCloud<PointType>::Ptr map_cloud, submap_vox;

            std::mutex mtx;
        }
        mapping;

        // std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> trajectory;

        boost::circular_buffer<ImuMeas> imu_buffer;
        util::tsq::TSQ<Eigen::Quaterniond> orient_buffer;

        struct
        {
            std::atomic<bool> dlo_initialized;
            std::atomic<bool> imu_calibrated;
            std::atomic<bool> submap_hasChanged;
            std::atomic<bool> can_iterate_map;

            int num_keyframes;

            // rclcpp::Time scan_stamp;

            double range_avg_lpf;
            double range_stddev_lpf;
            double adaptive_voxel_size;

            double first_imu_time;
            double curr_frame_stamp;
            double prev_frame_stamp;

            // std::string curr_scan_frame;

            Eigen::Vector3d origin;

            Eigen::Matrix4d T;
            Eigen::Matrix4d T_s2s, T_s2s_prev;

            // Eigen::Vector3d pose_s2s;
            // Eigen::Matrix3d rotSO3_s2s;
            // Eigen::Quaterniond rotq_s2s;

            Eigen::Vector3d pose;
            Eigen::Matrix3d rotSO3;
            Eigen::Quaterniond rotq, last_rotq;

            Eigen::Matrix4d imu_SE3;

            ImuBias imu_bias;
            ImuMeas imu_meas;

            std::mutex imu_mtx, scan_mtx;
        }
        state;

        struct
        {
            bool publish_debug_scans_;

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

            bool crop_use_;
            Eigen::Vector4f crop_min_, crop_max_;

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

            double mapping_valid_range_;
            double mapping_frustum_search_radius_;
            double mapping_delete_range_thresh_;
            double mapping_voxel_size_;
        }
        param;

        static bool comparatorImu(const ImuMeas& m1, const ImuMeas& m2) { return (m1.stamp < m2.stamp); };

    };

    void getParams();
    void initPGO();

    void sendTf(const builtin_interfaces::msg::Time& stamp, bool needs_lock = false);

    void handleStatusUpdate();

    void detection_callback(const cardinal_perception::msg::TagsTransform::ConstSharedPtr& det);
    void scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    rclcpp::CallbackGroup::SharedPtr mt_callback_group;
    rclcpp::Subscription<cardinal_perception::msg::TagsTransform>::SharedPtr detections_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub;
#if USE_GTSAM_PGO > 0
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
#endif
    rclcpp::Publisher<cardinal_perception::msg::ProcessMetrics>::SharedPtr proc_metrics_pub;
    rclcpp::Publisher<cardinal_perception::msg::ThreadMetrics>::SharedPtr imu_metrics_pub, det_metrics_pub, scan_metrics_pub;
    rclcpp::Publisher<cardinal_perception::msg::TrajectoryFilterDebug>::SharedPtr traj_filter_debug_pub;
    FloatPublisherMap metrics_pub;
    PublisherMap<geometry_msgs::msg::PoseStamped> pose_pub;
    PublisherMap<sensor_msgs::msg::PointCloud2> scan_pub;

    DLOdom lidar_odom;
    TrajectoryFilter<TagDetection> trajectory_filter;

#if USE_GTSAM_PGO > 0
    struct
    {
        gtsam::NonlinearFactorGraph factor_graph;
        std::shared_ptr<gtsam::ISAM2> isam;

        gtsam::Values init_estimate, isam_estimate;

        std::vector<size_t> keyframe_state_indices;
        gtsam::Pose3 last_odom;
        size_t next_state_idx = 0;

        nav_msgs::msg::Path trajectory_buff;

        std::mutex mtx;
    }
    pgo;
#endif

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    struct
    {
        std::atomic<bool> has_rebiased{ false };

        util::geom::PoseTf3d map_tf, odom_tf;
        double last_odom_stamp;

        std::mutex tf_mtx, print_mtx;
        std::chrono::system_clock::time_point last_print_time, last_frames_time;
    }
    state;

    struct
    {
        double metrics_pub_freq;
        int use_tag_detections;
        bool rebias_tf_pub_prereq;
        bool rebias_scan_pub_prereq;
    }
    param;

    enum class ProcType : size_t
    {
        DET_CB = 0,
        SCAN_CB,
        IMU_CB,
        HANDLE_METRICS,
        MISC,
        NUM_ITEMS
    };
    struct
    {
        util::proc::ThreadMetrics imu_thread, det_thread, scan_thread;
        util::proc::ProcessMetrics process_utilization;

        std::unordered_map<std::thread::id, std::array<double, (size_t)ProcType::NUM_ITEMS>> thread_proc_times;
        std::mutex thread_procs_mtx;
    }
    metrics;

private:
    void appendThreadProcTime(ProcType type, double dt);

};
