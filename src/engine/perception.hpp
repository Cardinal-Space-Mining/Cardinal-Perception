#pragma once

#include "util.h"

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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

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

#include <nano_gicp/nano_gicp.hpp>


struct TagDescription
{
    using Ptr = std::shared_ptr<TagDescription>;
    using ConstPtr = std::shared_ptr<const TagDescription>;

    std::array<cv::Point3f, 4>
        world_corners,
        rel_corners;

    union
    {
        struct
        {
            double x, y, z;
            double qw, qx, qy, qz, qww;
            double a, b, c, d;
        };
        struct
        {
            double translation[3];
            double rotation[5];
            double plane[4];
        };
    };

    static Ptr fromRaw(const std::vector<double>& world_corner_pts);
};
struct TagDetection
{
    using Ptr = std::shared_ptr<TagDetection>;
    using ConstPtr = std::shared_ptr<const TagDetection>;

    union
    {
        struct
        {
            double x, y, z;
            double qw, qx, qy, qz, qww;
        };
        struct
        {
            double translation[3];
            double quat_wxzy[5];
        };
        struct
        {
            double translation_[4];
            double quat_xyzw[4];
        };
    };

    double time_point, tags_area, avg_range, rms;
};

class DLOdom;
class TagDetector;

class PerceptionNode : public rclcpp::Node
{
friend DLOdom;
friend TagDetector;
public:
    PerceptionNode();

protected:
    class CameraSubscriber
    {
    public:
        CameraSubscriber(PerceptionNode* inst, const std::string& img_topic, const std::string& info_topic);

    protected:
        PerceptionNode* pnode;

        image_transport::Subscriber image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

        cv::Mat dbg_frame;
        cv::Mat1d calibration = cv::Mat1d::zeros(3, 3);
        cv::Mat1d distortion = cv::Mat1d::zeros(1, 5);

        std::atomic<bool> valid_calib = false;

    private:
        void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img);
        void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

    };

    void getParams();
    void initMetrics();
    // void updateMetrics();

    void handleStatusUpdate();
    void handleDebugFrame();

    void scan_callback(const sensor_msgs::msg::ConstSharedPtr& scan);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    image_transport::ImageTransport img_transport;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    std::vector<CameraSubscriber> camera_subs;

    image_transport::Publisher debug_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_scan_pub;

    DLOdom lidar_odom;
    TagDetector tag_detection;

    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;

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

    struct
    {
        // static
        std::string cpu_type;
        size_t numProcessors;
        // cached
        clock_t lastCPU, lastSysCPU, lastUserCPU;
        // cpu utilization
        double avg_cpu_percent, max_cpu_percent;
        size_t avg_cpu_samples;
        // dlo
        std::mutex dlo_stats_mtx;
        double dlo_last_comp_time, dlo_avg_comp_time, dlo_max_comp_time;
        size_t dlo_avg_comp_samples;
        // tags
        std::mutex tags_stats_mtx;
        double tags_last_comp_time, tags_avg_comp_time, tags_max_comp_time;
        size_t tags_avg_comp_stamples;
    }
    metrics;

};

class DLOdom
{
friend PerceptionNode;
public:
    using PointType = pcl::PointXYZ;

    DLOdom(PerceptionNode* inst);

protected:
    void getParams();

    void processScan(const sensor_msgs::msg::PointCloud2::SharedPtr& scan);
    void processImu(const sensor_msgs::msg::Imu::SharedPtr& imu);

    void preprocessPoints();
    void initializeInputTarget();
    void setInputSources();

    void initializeDLO();
    void gravityAlign();

    void getNextPose();
    void integrateIMU();

    void propagateS2S(Eigen::Matrix4f T);
    void propagateS2M();

    void setAdaptiveParams();

    void transformCurrentScan();
    void updateKeyframes();
    void computeConvexHull();
    void computeConcaveHull();
    void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
    void getSubmapKeyframes();

private:
    PerceptionNode* pnode;

    Eigen::Vector3f origin;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
    std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes;
    std::vector<std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>> keyframe_normals;

    std::atomic<bool> dlo_initialized;
    std::atomic<bool> imu_calibrated;

    pcl::PointCloud<PointType>::Ptr export_scan;
    pcl::PointCloud<PointType>::Ptr current_scan;
    pcl::PointCloud<PointType>::Ptr current_scan_t;

    pcl::PointCloud<PointType>::Ptr keyframes_cloud;
    pcl::PointCloud<PointType>::Ptr keyframe_cloud;
    int num_keyframes;

    pcl::ConvexHull<PointType> convex_hull;
    pcl::ConcaveHull<PointType> concave_hull;
    std::vector<int> keyframe_convex;
    std::vector<int> keyframe_concave;

    pcl::PointCloud<PointType>::Ptr submap_cloud;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;

    std::vector<int> submap_kf_idx_curr;
    std::vector<int> submap_kf_idx_prev;
    std::atomic<bool> submap_hasChanged;

    pcl::PointCloud<PointType>::Ptr source_cloud;
    pcl::PointCloud<PointType>::Ptr target_cloud;

    rclcpp::Time scan_stamp;

    double curr_frame_stamp;
    double prev_frame_stamp;
    std::vector<double> comp_times;

    nano_gicp::NanoGICP<PointType, PointType> gicp_s2s;
    nano_gicp::NanoGICP<PointType, PointType> gicp;

    pcl::CropBox<PointType> crop;
    pcl::VoxelGrid<PointType> vf_scan;
    pcl::VoxelGrid<PointType> vf_submap;

    Eigen::Matrix4f T;
    Eigen::Matrix4f T_s2s, T_s2s_prev;
    Eigen::Quaternionf q_final;

    Eigen::Vector3f pose_s2s;
    Eigen::Matrix3f rotSO3_s2s;
    Eigen::Quaternionf rotq_s2s;

    Eigen::Vector3f pose;
    Eigen::Matrix3f rotSO3;
    Eigen::Quaternionf rotq;

    Eigen::Matrix4f imu_SE3;

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
    }
    imu_bias;

    struct ImuMeas
    {
        double stamp;
        XYZd ang_vel;
        XYZd lin_accel;
    }
    imu_meas;

    boost::circular_buffer<ImuMeas> imu_buffer;
    std::mutex mtx_imu;

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
        Eigen::Vector3f initial_position_;
        Eigen::Quaternionf initial_orientation_;

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
