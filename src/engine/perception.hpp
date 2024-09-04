#pragma once

#include "common.hpp"
#include "util.hpp"
#include "synchronization.hpp"
#include "pub_map.hpp"
#include "geometry.hpp"

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

#include <image_transport/image_transport.hpp>

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <nano_gicp/nano_gicp.hpp>


struct TagDescription
{
    using Ptr = std::shared_ptr<TagDescription>;
    using ConstPtr = std::shared_ptr<const TagDescription>;

    std::array<cv::Point3f, 4>
        world_corners,
        rel_corners;

    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Vector4d plane;

    static Ptr fromRaw(const std::vector<double>& world_corner_pts);
};
struct TagDetection
{
    using Ptr = std::shared_ptr<TagDetection>;
    using ConstPtr = std::shared_ptr<const TagDetection>;

    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;

    double time_point, pix_area, avg_range, rms;
    size_t num_tags;
};

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode();
    ~PerceptionNode() = default;

protected:
    class CameraSubscriber
    {
    public:
        CameraSubscriber() = default;
        // CameraSubscriber(PerceptionNode* inst, const std::string& img_topic, const std::string& info_topic);
        CameraSubscriber(const CameraSubscriber& ref);
        ~CameraSubscriber() = default;

        void initialize(PerceptionNode* inst, const std::string& img_topic, const std::string& info_topic);

    public:
        PerceptionNode* pnode = nullptr;

        image_transport::Subscriber image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

        // SpinBuffer<cv::Mat> dbg_frame;
        Synchronized<cv::Mat> dbg_frame;
        cv::Mat1d calibration = cv::Mat1d::zeros(3, 3);
        cv::Mat1d distortion = cv::Mat1d::zeros(1, 5);

        std::atomic<bool> valid_calib = false;
        // bool valid_calib = false;

    private:
        void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img);
        void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

    };

    class DLOdom
    {
    friend PerceptionNode;
    public:
        using PointType = cardinal_perception::PointType;

        DLOdom(PerceptionNode* inst);
        ~DLOdom() = default;

    public:
        void getParams();

        int64_t processScan(
            const sensor_msgs::msg::PointCloud2::SharedPtr& scan,
            pcl::PointCloud<PointType>::Ptr& filtered_scan,
            util::geom::PoseTf3d& odom_tf);
        void processImu(const sensor_msgs::msg::Imu::SharedPtr& imu);

        void preprocessPoints();
        void initializeInputTarget();
        void setInputSources();

        void initializeDLO();
        void gravityAlign();

        void getNextPose();
        void integrateIMU();

        void propagateS2S(Eigen::Matrix4d T);
        void propagateS2M();

        void setAdaptiveParams();

        void transformCurrentScan();
        void updateKeyframes();
        void computeConvexHull();
        void computeConcaveHull();
        void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
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

    private:
        PerceptionNode* pnode;

        pcl::PointCloud<PointType>::Ptr source_cloud;
        pcl::PointCloud<PointType>::Ptr current_scan;
        pcl::PointCloud<PointType>::Ptr current_scan_t;
        pcl::PointCloud<PointType>::Ptr export_scan;
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
        std::vector<std::pair<std::pair<Eigen::Vector3d, Eigen::Quaterniond>, pcl::PointCloud<PointType>::Ptr>> keyframes;
        std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

        pcl::PointCloud<PointType>::Ptr submap_cloud;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;
        std::vector<int> submap_kf_idx_curr;
        std::vector<int> submap_kf_idx_prev;

        nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
        nano_gicp::NanoGICP<PointType, PointType> gicp;

        // std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> trajectory;

        boost::circular_buffer<ImuMeas> imu_buffer;

        struct
        {
            std::atomic<bool> dlo_initialized;
            std::atomic<bool> imu_calibrated;
            std::atomic<bool> submap_hasChanged;

            int num_keyframes;

            // rclcpp::Time scan_stamp;

            double first_imu_time;
            double curr_frame_stamp;
            double prev_frame_stamp;

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

            bool adaptive_params_use_;

            bool imu_use_;
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

        static bool comparatorImu(ImuMeas m1, ImuMeas m2) { return (m1.stamp < m2.stamp); };

    };

    class TagDetector
    {
    friend PerceptionNode;
    public:
        TagDetector(PerceptionNode* inst);
        ~TagDetector() = default;

    protected:
        void getParams();

        // template<bool enable_debug = true>
        void processImg(
            const sensor_msgs::msg::Image::ConstSharedPtr& img,
            PerceptionNode::CameraSubscriber& sub,
            std::vector<TagDetection::Ptr>& detections);

    private:
        PerceptionNode* pnode;

        std::unordered_map<int, TagDescription::ConstPtr> obj_tag_corners;
        cv::Ptr<cv::aruco::Dictionary> aruco_dict;
        cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

    };

    void getParams();
    void initPGO();
    void initMetrics();

    void sendTf(const builtin_interfaces::msg::Time& stamp, bool needs_lock = false);

    void handleStatusUpdate();
    void handleDebugFrame();

    void scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& scan);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    image_transport::ImageTransport img_transport;

    rclcpp::CallbackGroup::SharedPtr mt_callback_group;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    std::vector<CameraSubscriber> camera_subs;

    image_transport::Publisher debug_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    FloatPublisherMap metrics_pub;
    PublisherMap<geometry_msgs::msg::PoseStamped> pose_pub;

    DLOdom lidar_odom;
    TagDetector tag_detection;

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

    std::deque<TagDetection::Ptr> alignment_queue;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

    struct
    {
        std::atomic<bool> any_new_frames{ false };
        std::atomic<bool> dlo_in_progress{ false };

        util::geom::PoseTf3d map_tf, odom_tf;
        double last_odom_stamp;

        std::mutex tf_mtx, alignment_mtx, print_mtx, frames_mtx;
        std::chrono::system_clock::time_point last_print_time, last_frames_time;
    }
    state;

    struct
    {
        double status_max_print_freq;
        double img_debug_max_pub_freq;
    }
    param;

    struct
    {
        Eigen::AlignedBox3d filter_bbox;

        double fitness_oob_weight;
        double fitness_rms_weight;
        double thresh_max_linear_diff_velocity;
        double thresh_max_angular_diff_velocity;
        double thresh_min_tags_per_range;
        double thresh_max_rms_per_tag;
        double thresh_min_pix_area;

        double covariance_linear_base_coeff;
        double covariance_linear_range_coeff;
        double covariance_angular_base_coeff;
        double covariance_angular_range_coeff;
    }
    tag_filtering;

    struct ThreadMetrics
    {
        std::chrono::system_clock::time_point last_call_time;
        double last_comp_time{0.}, avg_comp_time{0.}, max_comp_time{0.}, avg_call_delta{0.};
        size_t samples{0};
        std::mutex mtx;

        double addSample(
            const std::chrono::system_clock::time_point& start,
            const std::chrono::system_clock::time_point& end);
    };

    enum class ProcType : size_t
    {
        IMG_CB = 0,
        INFO_CB,
        SCAN_CB,
        IMU_CB,
        HANDLE_DBG_FRAME,
        HANDLE_METRICS,
        MISC,
        NUM_ITEMS
    };
    struct
    {
        // static
        std::string cpu_type;
        size_t num_processors;
        // cached
        clock_t last_cpu, last_sys_cpu, last_user_cpu;
        // cpu utilization
        double avg_cpu_percent{0.}, max_cpu_percent{0.};
        size_t avg_cpu_samples{0};
        // callbacks
        ThreadMetrics imu_thread, info_thread, img_thread, scan_thread;

        std::unordered_map<std::thread::id, std::array<double, (size_t)ProcType::NUM_ITEMS>> thread_proc_times;
        std::mutex thread_procs_mtx;
    }
    metrics;

private:
    void appendThreadProcTime(ProcType type, double dt);

};
