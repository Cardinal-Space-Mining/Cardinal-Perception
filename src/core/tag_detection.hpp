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

#include "util.hpp"
#include "geometry.hpp"
#include "pub_map.hpp"
#include <stats/stats.hpp>

#include "cardinal_perception/msg/tags_transform.hpp"
#include "cardinal_perception/msg/process_metrics.hpp"
#include "cardinal_perception/msg/thread_metrics.hpp"

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


namespace csm
{
namespace perception
{

struct TagDescription
{
    using Ptr = std::shared_ptr<TagDescription>;
    using ConstPtr = std::shared_ptr<const TagDescription>;
    using Optional = std::optional<TagDescription>;

    std::array<cv::Point3f, 4>
        world_corners,
        rel_corners;

    Eigen::Quaterniond rotation;
    Eigen::Vector4d plane;

    std::string
        frame_id,
        base_frame;

    Eigen::Vector3d translation;

    bool is_static;

    static Optional fromRaw(
        const std::vector<double>& world_corner_pts,
        const std::vector<std::string>& frames,
        bool is_static );
};

class TagDetector :
    public rclcpp::Node
{
public:
    TagDetector();
    ~TagDetector() = default;
    DECLARE_IMMOVABLE(TagDetector)

protected:
    class CameraSubscriber
    {
        friend class TagDetector;
    public:
        CameraSubscriber(
            TagDetector* inst,
            const std::vector<std::string>& param_buf,
            const std::vector<double>& offset_pose );
        ~CameraSubscriber() = default;
        DECLARE_IMMOVABLE(CameraSubscriber)

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
    std::vector<std::unique_ptr<CameraSubscriber>> camera_subs;

    rclcpp::Publisher<cardinal_perception::msg::TagsTransform>::SharedPtr detection_pub, debug_pub;
    rclcpp::Publisher<cardinal_perception::msg::ProcessMetrics>::SharedPtr proc_metrics_pub;
    rclcpp::Publisher<cardinal_perception::msg::ThreadMetrics>::SharedPtr detection_metrics_pub;

    std::unordered_map<int, TagDescription> tag_descriptions;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;

    util::proc::ThreadMetrics detection_cb_metrics;
    util::proc::ProcessMetrics process_metrics;
    // FloatPublisherMap metrics_pub;

    struct
    {
        int publish_best_tf;
        int export_best_detection;
        bool export_debug_detections;
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
        double thresh_max_coplanar_dist;
    }
    filtering;

};

};
};
