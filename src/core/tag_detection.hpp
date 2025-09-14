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

#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <optional>

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

#include <csm_metrics/stats.hpp>
#include <csm_metrics/msg/task_stats.hpp>
#include <csm_metrics/msg/process_stats.hpp>

#include <cardinal_perception/msg/tags_transform.hpp>

#include <util.hpp>
#include <geometry.hpp>
#include <pub_map.hpp>



namespace csm
{
namespace perception
{

struct TagDescription
{
    using Ptr = std::shared_ptr<TagDescription>;
    using ConstPtr = std::shared_ptr<const TagDescription>;
    using Optional = std::optional<TagDescription>;
    using CornersArray = std::array<cv::Point3f, 4>;

    CornersArray world_corners;
    CornersArray rel_corners;

    Eigen::Quaterniond rotation;
    Eigen::Vector4d plane;
    Eigen::Vector3d translation;

    std::string frame_id;
    std::string base_frame;

    bool is_static;

    static Optional fromRaw(
        const std::vector<double>& world_corner_pts,
        const std::vector<std::string>& frames,
        bool is_static);
};


class TagDetector : public rclcpp::Node
{
protected:
    using TagsTransformMsg = cardinal_perception::msg::TagsTransform;
    using TaskStatsMsg = csm_metrics::msg::TaskStats;
    using ProcessStatsMsg = csm_metrics::msg::ProcessStats;
    using ImageMsg = sensor_msgs::msg::Image;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

    using ClockType = std::chrono::system_clock;

protected:
    class CameraSubscriber
    {
        friend class TagDetector;

    public:
        CameraSubscriber(
            TagDetector* inst,
            const std::vector<std::string>& param_buf,
            const std::vector<double>& offset_pose);
        ~CameraSubscriber() = default;
        DECLARE_IMMOVABLE(CameraSubscriber)

    private:
        void img_callback(const ImageMsg::ConstSharedPtr& img);
        void info_callback(const CameraInfoMsg::ConstSharedPtr& info);

    protected:
        TagDetector* node;

        rclcpp::Subscription<CameraInfoMsg>::SharedPtr info_sub;
        image_transport::Subscriber image_sub;
        image_transport::Publisher debug_img_pub;

        cv::Mat1d calibration = cv::Mat1d::zeros(3, 3);
        cv::Mat1d distortion = cv::Mat1d::zeros(1, 5);

        std::string cam_frame_override;
        std::string base_tf_frame;
        util::geom::Pose3d offset;

        bool valid_calib = false;
    };

public:
    TagDetector();
    ~TagDetector() = default;
    DECLARE_IMMOVABLE(TagDetector)

public:
    void getParams();

    void processImg(const ImageMsg::ConstSharedPtr& img, CameraSubscriber& sub);
    void updateStats(
        const ClockType::time_point& start,
        const ClockType::time_point& end);

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    image_transport::ImageTransport img_transport;

    rclcpp::CallbackGroup::SharedPtr mt_callback_group;
    std::vector<std::unique_ptr<CameraSubscriber>> camera_subs;

    util::GenericPubMap generic_pub;

    std::unordered_map<int, TagDescription> tag_descriptions;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

    csm::metrics::TaskStats detection_cb_metrics;
    csm::metrics::ProcessStats process_metrics;

private:
    struct
    {
        int publish_best_tf;
        int export_best_detection;
        bool export_debug_detections;
        bool enable_debug_stream;
        bool publish_individual_tag_solution_tfs;
        bool publish_group_solution_tfs;

    } param;

    struct
    {
        Eigen::AlignedBox3d filter_bbox;
        bool use_bounds;

        double thresh_min_tags_per_range;
        double thresh_max_rms_per_tag;
        double thresh_min_pix_area;
        double thresh_max_coplanar_dist;

    } filtering;
};

};  // namespace perception
};  // namespace csm
