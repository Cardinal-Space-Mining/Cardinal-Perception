#pragma once

#include "util.hpp"
#include "geometry.hpp"
#include "pub_map.hpp"
#include "stats.hpp"

#include "cardinal_perception/msg/tags_transform.hpp"

#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_transport/image_transport.hpp>


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

    std::string
        frame_id,
        base_frame;

    bool is_static;

    static Ptr fromRaw(const std::vector<double>& world_corner_pts, const std::vector<std::string>& frames, bool is_static);
};

class TagDetector : public rclcpp::Node
{
public:
    TagDetector();
    ~TagDetector() = default;

protected:
    class CameraSubscriber
    {
    friend class TagDetector;
    public:
        CameraSubscriber() = default;
        CameraSubscriber(const CameraSubscriber& ref);
        ~CameraSubscriber() = default;

        void initialize(
            TagDetector* inst,
            const std::string& img_topic,
            const std::string& info_topic);

    private:
        void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img);
        void info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

    protected:
        TagDetector* node;

        image_transport::Subscriber image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
        image_transport::Publisher debug_img_pub;

        cv::Mat1d calibration = cv::Mat1d::zeros(3, 3);
        cv::Mat1d distortion = cv::Mat1d::zeros(1, 5);

        std::string cam_frame_override;
        std::string base_tf_frame;
        util::geom::Pose3d offset;

        bool valid_calib = false;

    };

    void getParams();

    void processImg(
        const sensor_msgs::msg::Image::ConstSharedPtr& img,
        TagDetector::CameraSubscriber& sub);
    void updateStats(
        const std::chrono::system_clock::time_point& start,
        const std::chrono::system_clock::time_point& end);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    image_transport::ImageTransport img_transport;

    rclcpp::CallbackGroup::SharedPtr mt_callback_group;
    std::vector<CameraSubscriber> camera_subs;

    rclcpp::Publisher<cardinal_perception::msg::TagsTransform>::SharedPtr detection_pub;

    std::unordered_map<int, TagDescription::ConstPtr> tag_descriptions;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

    util::proc::ThreadMetrics detection_cb_metrics;
    util::proc::ProcessMetrics process_metrics;
    FloatPublisherMap metrics_pub;

    struct
    {
        int publish_best_tf;
        int export_best_detection;
        int export_all_detections;
        bool enable_debug_stream;
        bool publish_individual_tag_solution_tfs;
        bool publish_group_solution_tfs;
    }
    param;

    struct
    {
        Eigen::AlignedBox3d filter_bbox;
        bool use_bounds;

        double thresh_min_tags_per_range;
        double thresh_max_rms_per_tag;
        double thresh_min_pix_area;
    }
    filtering;

};
