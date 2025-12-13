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

#include "tag_detection.hpp"

#include <sstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifndef USE_CV_BRIDGE_HPP
    #include <cv_bridge/cv_bridge.h>
#else
    #include <cv_bridge/cv_bridge.hpp>
#endif


using namespace util::geom::cvt::ops;

using TransformMsg = geometry_msgs::msg::Transform;
using TransformStampedMsg = geometry_msgs::msg::TransformStamped;


namespace csm
{
namespace perception
{

TagDescription::Optional TagDescription::fromRaw(
    const std::vector<double>& pts,
    const std::vector<std::string>& frames,
    bool is_static)
{
    if (pts.size() < 12 || frames.empty())
    {
        return std::nullopt;
    }

    const cv::Point3d c0{pts[0], pts[1], pts[2]};
    const cv::Point3d c1{pts[3], pts[4], pts[5]};
    const cv::Point3d c2{pts[6], pts[7], pts[8]};
    const cv::Point3d c3{pts[9], pts[10], pts[11]};

    const cv::Vec3d                             // fill in each row
        x_full = c1 - c0,                       // need to get length later
        x = cv::normalize(x_full),              // x-axis
        y = cv::normalize(cv::Vec3d{c1 - c2}),  // y-axis
        z = x.cross(y);  // z-axis can be dervied from x and y

    // clang-format off
    const cv::Matx33d rmat{
        x[0], x[1], x[2],
        y[0], y[1], y[2],
        z[0], z[1], z[2] };  // rotation matrix from orthogonal axes
    // clang-format on

    TagDescription description;
    description.world_corners = {
        static_cast<cv::Point3f>(c0),
        static_cast<cv::Point3f>(c1),
        static_cast<cv::Point3f>(c2),
        static_cast<cv::Point3f>(c3)};

    const float half_len = static_cast<float>(cv::norm(x_full) / 2.);
    description.rel_corners = {
        cv::Point3f{-half_len, +half_len, 0.f},
        cv::Point3f{+half_len, +half_len, 0.f},
        cv::Point3f{+half_len, -half_len, 0.f},
        cv::Point3f{-half_len, -half_len, 0.f}
    };
    description.translation << (c0 + c2) / 2.;  // center point
    description.rotation << cv::Quatd::createFromRotMat(rmat);
    description.plane[0] = z[0];
    description.plane[1] = z[1];
    description.plane[2] = z[2];
    description.plane[3] =
        description.plane.block<3, 1>(0, 0).dot(description.translation);
    description.plane.normalize();

    description.frame_id = frames[0];
    if (frames.size() > 1)
    {
        description.base_frame = frames[1];
    }
    description.is_static = is_static;

    return description;
}

TagDetector::TagDetector() :
    Node("cardinal_perception_tag_detection"),
    tf_buffer{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)},
    tf_listener{tf_buffer},
    tf_broadcaster{*this},
    img_transport{std::shared_ptr<TagDetector>(this, [](auto*) {})},
    mt_callback_group{
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)},
    generic_pub{*this, "", rclcpp::SensorDataQoS{}},
    aruco_params{cv::aruco::DetectorParameters::create()}
{
    this->getParams();
}



TagDetector::CameraSubscriber::CameraSubscriber(
    TagDetector* inst,
    const std::vector<std::string>& param_buf,
    const std::vector<double>& offset_pose) :
    node(inst)
{
    if (!inst)
    {
        return;
    }
    if (param_buf.size() < 2)
    {
        return;
    }

    if (param_buf.size() > 2)
    {
        this->cam_frame_override = param_buf[2];
    }
    if (param_buf.size() > 3)
    {
        this->base_tf_frame = param_buf[3];
    }
    if (offset_pose.size() == 4)
    {
        this->offset.quat.w() = offset_pose[0];
        this->offset.quat.x() = offset_pose[1];
        this->offset.quat.y() = offset_pose[2];
        this->offset.quat.z() = offset_pose[3];
    }
    else if (offset_pose.size() >= 3)
    {
        this->offset.vec.x() = offset_pose[0];
        this->offset.vec.y() = offset_pose[1];
        this->offset.vec.z() = offset_pose[2];
        if (offset_pose.size() >= 7)
        {
            this->offset.quat.w() = offset_pose[3];
            this->offset.quat.x() = offset_pose[4];
            this->offset.quat.y() = offset_pose[5];
            this->offset.quat.z() = offset_pose[6];
        }
    }

    rclcpp::SubscriptionOptions ops{};
    ops.callback_group = this->node->mt_callback_group;

    this->image_sub = this->node->img_transport.subscribe(
        param_buf[0],
        rmw_qos_profile_sensor_data.depth,
        [this](const ImageMsg::ConstSharedPtr& img)
        { this->img_callback(img); },
        image_transport::ImageTransport::VoidPtr(),
        nullptr,
        ops);

    this->info_sub = this->node->create_subscription<CameraInfoMsg>(
        param_buf[1],
        rclcpp::SensorDataQoS{},
        [this](const CameraInfoMsg::ConstSharedPtr& info)
        { this->info_callback(info); },
        ops);

    this->debug_img_pub = this->node->img_transport.advertise(
        param_buf[0] + "/debug_output",
        rmw_qos_profile_sensor_data.depth);
}

void TagDetector::CameraSubscriber::img_callback(
    const ImageMsg::ConstSharedPtr& img)
{
    auto _start = std::chrono::system_clock::now();
    this->node->processImg(img, *this);
    auto _end = std::chrono::system_clock::now();
    this->node->updateStats(_start, _end);
}

void TagDetector::CameraSubscriber::info_callback(
    const CameraInfoMsg::ConstSharedPtr& info)
{
    if (!this->valid_calib && info->k.size() == 9 && info->d.size() >= 5)
    {
        this->calibration = cv::Mat(info->k, true).reshape(0, 3);
        this->distortion = cv::Mat(info->d, true).reshape(0, 1);

        this->valid_calib = true;
    }
}

void TagDetector::getParams()
{
    this->declare_parameter("image_transport", "raw");

    util::declare_param(
        this,
        "feature.publish_best_detection_tf",
        this->param.publish_best_tf,
        0);
    util::declare_param(
        this,
        "feature.export_best_detection",
        this->param.export_best_detection,
        1);
    util::declare_param(
        this,
        "feature.debug.export_all_detections",
        this->param.export_debug_detections,
        false);
    util::declare_param(
        this,
        "feature.debug.publish_stream",
        this->param.enable_debug_stream,
        true);
    util::declare_param(
        this,
        "feature.debug.publish_individual_tag_solution_tfs",
        this->param.publish_individual_tag_solution_tfs,
        true);
    util::declare_param(
        this,
        "feature.debug.publish_group_solution_tfs",
        this->param.publish_group_solution_tfs,
        true);

    std::vector<double> min, max;
    util::declare_param(this, "filtering.bounds_min", min, {});
    util::declare_param(this, "filtering.bounds_max", max, {});
    if (min.size() > 2 && max.size() > 2)
    {
        this->filtering.filter_bbox = Eigen::AlignedBox3d{
            Eigen::Vector3d{min.data()},
            Eigen::Vector3d{max.data()}};
    }
    util::declare_param(
        this,
        "filtering.use_bounds",
        this->filtering.use_bounds,
        true);
    util::declare_param(
        this,
        "filtering.thresh.min_tags_per_range",
        this->filtering.thresh_min_tags_per_range,
        0.5);
    util::declare_param(
        this,
        "filtering.thresh.max_rms_per_tag",
        this->filtering.thresh_max_rms_per_tag,
        0.1);
    util::declare_param(
        this,
        "filtering.thresh.min_sum_pix_area",
        this->filtering.thresh_min_pix_area,
        10000.);
    util::declare_param(
        this,
        "filtering.thresh.require_nonplanar_after",
        this->filtering.thresh_max_coplanar_dist,
        2.);

    int aruco_dict_id =
        cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
    util::declare_param(
        this,
        "aruco.predefined_family_idx",
        aruco_dict_id,
        aruco_dict_id);
    this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);

    std::vector<int64_t> tag_ids;
    util::declare_param(this, "aruco.tags.ids", tag_ids, {});

    const size_t n_tags = tag_ids.size();
    if (n_tags < 1)
    {
        return;
    }

    std::vector<double> corners_buff;
    corners_buff.reserve(12);
    std::vector<std::string> str_param_buff;
    str_param_buff.reserve(4);
    for (size_t i = 0; i < n_tags; i++)
    {
        corners_buff.clear();
        str_param_buff.clear();
        const int id = static_cast<int>(tag_ids[i]);
        bool is_static = true;

        util::declare_param(
            this,
            (std::ostringstream{} << "aruco.tags.tag" << id << "_corners").str(),
            corners_buff,
            {});
        util::declare_param(
            this,
            (std::ostringstream{} << "aruco.tags.tag" << id << "_frames").str(),
            str_param_buff,
            {});
        util::declare_param(
            this,
            (std::ostringstream{} << "aruco.tags.tag" << id << "_static").str(),
            is_static,
            true);

        auto description =
            TagDescription::fromRaw(corners_buff, str_param_buff, is_static);
        if (description.has_value())
        {
            tag_descriptions[id] = description.value();
        }
    }

    int n_streams = 0;
    util::declare_param(this, "streams.num_streams", n_streams, 0);
    std::vector<double> offset_pose;
    offset_pose.reserve(7);
    this->camera_subs.reserve(n_streams);
    for (int i = 0; i < n_streams; i++)
    {
        str_param_buff.clear();
        offset_pose.clear();

        std::ostringstream param;
        param << "streams.stream" << i;
        // config consists of string list: ["/img/topic", "/info/topic", "frame_override", "base_frame"]
        util::declare_param(this, param.str(), str_param_buff, {});
        if (str_param_buff.size() < 2)
        {
            continue;
        }

        param << "_offset";
        // offset to actual camera origin within camera frame
        util::declare_param(this, param.str(), offset_pose, {});

        this->camera_subs.emplace_back(
            std::make_unique<CameraSubscriber>(
                this,
                str_param_buff,
                offset_pose));
    }
}

void TagDetector::processImg(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    CameraSubscriber& sub)
{
    if (!sub.valid_calib)
    {
        return;
    }

    thread_local cv::Mat debug_frame;
    thread_local std::vector<std::vector<cv::Point2f>> tag_corners;
    thread_local std::vector<int> tag_ids;

    cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");
    if (this->param.enable_debug_stream)
    {
        cv::cvtColor(cv_img->image, debug_frame, CV_GRAY2BGR);
    }

    // detections.clear();
    tag_corners.clear();
    tag_ids.clear();

    // TODO: undistort (param as well)

    try
    {
        cv::aruco::detectMarkers(
            cv_img->image,
            this->aruco_dict,
            tag_corners,
            tag_ids,
            this->aruco_params);
    }
    catch (const std::exception& e)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "[TAG DETECTION]: Encountered exception while detecting markers!\n\twhat(): %s",
            e.what());
        return;
    }

    if (this->param.enable_debug_stream)
    {
        cv::aruco::drawDetectedMarkers(
            debug_frame,
            tag_corners,
            tag_ids,
            cv::Scalar{0, 255, 0});
    }

    const size_t n_detected = tag_ids.size();
    if (n_detected > 0 && n_detected == tag_corners.size())
    {
        const std::string& cam_frame_id = sub.cam_frame_override.empty()
                                              ? img->header.frame_id
                                              : sub.cam_frame_override;
        const std::string& cam_base_frame = sub.base_tf_frame;
        const bool need_cam_base_tf =
            !cam_base_frame.empty() && cam_base_frame != cam_frame_id;

        // effective transform which converts from lowest baselink frame to camera origin
        TransformStampedMsg cam_tf;
        cam_tf.transform << sub.offset;
        cam_tf.header.stamp = img->header.stamp;
        cam_tf.header.frame_id = cam_frame_id;

        // detections get lumped based on the lowest coordinate frame which they can be statically transformed into
        struct DetectionGroup
        {
            TagDescription primary_desc;
            Eigen::Vector4d detection_plane = Eigen::Vector4d::Zero();

            std::vector<cv::Point2f> img_points;
            std::vector<cv::Point3f> obj_points;
            std::vector<cv::Vec3d> tvecs, rvecs;
            std::vector<double> eerrors;
            std::vector<double> ranges;

            double sum_area = 0.;
            size_t n_matches = 0;
            bool all_coplanar = true;

            inline DetectionGroup()
            {
                img_points.reserve(8);
                obj_points.reserve(8);
            }
        };

        thread_local std::unordered_map<std::string, DetectionGroup>
            detection_groups;
        detection_groups.clear();
        detection_groups.reserve(n_detected);

        for (size_t i = 0; i < n_detected; i++)
        {
            auto search = this->tag_descriptions.find(tag_ids[i]);
            if (search != this->tag_descriptions.end())
            {
                const TagDescription& tag_description = search->second;

                const bool use_baselink_tf =
                    !tag_description.base_frame.empty() &&
                    tag_description.frame_id != tag_description.base_frame &&
                    this->tf_buffer.canTransform(
                        tag_description.frame_id,
                        tag_description.base_frame,
                        util::toTf2TimePoint(img->header.stamp));
                const std::string& target_frame =
                    use_baselink_tf ? tag_description.base_frame
                                    : tag_description.frame_id;

                auto itr = detection_groups.try_emplace(target_frame).first;
                if (itr == detection_groups.end())
                {
                    continue;
                }

                DetectionGroup& group = itr->second;
                auto& obj_corners = tag_description.world_corners;
                auto& rel_obj_corners = tag_description.rel_corners;
                auto& img_corners = tag_corners[i];

                if (use_baselink_tf)
                {
                    const size_t obj_points_off = group.obj_points.size();
                    group.obj_points.resize(group.obj_points.size() + 4);
                    try
                    {
                        auto tf = this->tf_buffer.lookupTransform(
                            tag_description.base_frame,
                            tag_description.frame_id,
                            util::toTf2TimePoint(img->header.stamp));

                        Eigen::Isometry3f _tf;
                        _tf << tf.transform;

                        Eigen::Vector3f p;
                        for (size_t i = 0; i < 4; i++)
                        {
                            group.obj_points[i + obj_points_off]
                                << (_tf * (p << obj_corners[i]));
                        }
                    }
                    catch (const std::exception& e)
                    {
                        continue;  // could handle by reverting to main tag frame?
                    }
                }
                else
                {
                    group.obj_points.insert(
                        group.obj_points.end(),
                        obj_corners.begin(),
                        obj_corners.end());
                }
                group.img_points.insert(
                    group.img_points.end(),
                    img_corners.begin(),
                    img_corners.end());

                if (++group.n_matches == 1)
                {
                    group.primary_desc = tag_description;
                }
                group.sum_area += cv::contourArea(img_corners);

                cv::solvePnPGeneric(
                    rel_obj_corners,
                    img_corners,
                    sub.calibration,
                    sub.distortion,
                    group.rvecs,
                    group.tvecs,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE,
                    cv::noArray(),
                    cv::noArray(),
                    group.eerrors);

                TransformStampedMsg dbg_tf;
                dbg_tf.header = cv_img->header;
                for (size_t s = 0; s < group.tvecs.size(); s++)
                {
                    cv::Vec3d& _rvec = group.rvecs[s];
                    cv::Vec3d& _tvec = group.tvecs[s];

                    group.ranges.push_back(cv::norm(_tvec));

                    if (this->param.publish_individual_tag_solution_tfs)
                    {
                        std::ostringstream cframe;
                        cframe << "tag_" << tag_ids[i] << '_' << s;

                        dbg_tf.child_frame_id = cframe.str();
                        dbg_tf.transform.translation << _tvec;
                        dbg_tf.transform.rotation
                            << cv::Quatd::createFromRvec(_rvec);
                        // apply camera origin offset
                        tf2::doTransform(dbg_tf, dbg_tf, cam_tf);

                        this->tf_broadcaster.sendTransform(dbg_tf);
                    }
                }

                if (group.all_coplanar)
                {
                    if (group.n_matches == 1)
                    {
                        group.detection_plane = tag_description.plane;
                    }
                    else if (
                        1. - std::abs(group.detection_plane.dot(
                                 tag_description.plane)) >
                        1e-6)
                    {
                        group.all_coplanar = false;
                    }
                }
            }
        }

        if (need_cam_base_tf)
        {
            try
            {
                auto tf = this->tf_buffer.lookupTransform(  // might need to swap order
                    cam_frame_id,
                    cam_base_frame,
                    util::toTf2TimePoint(cv_img->header.stamp) );    // camera baselink --> camera frame

                // current tf: camera baselink --> camera origin
                tf2::doTransform(cam_tf, cam_tf, tf);
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Error at line %d in fn %s: %s",
                    __LINE__,
                    __func__,
                    e.what());
            }
        }

        for (auto& itr : detection_groups)
        {
            const std::string& tags_frame = itr.first;
            DetectionGroup& group = itr.second;

            if (group.n_matches == 1)
            {
                Eigen::Isometry3d tf =
                    group.primary_desc.rotation *
                    Eigen::Translation3d{-group.primary_desc.translation};
                // TODO: make this simpler >>

                // convert each rvec,tvec to be relative to tags frame not center of tag
                const size_t n_solutions = group.tvecs.size();
                for (size_t i = 0; i < n_solutions; i++)
                {
                    Eigen::Quaterniond r;
                    Eigen::Translation3d t;
                    r << cv::Quatd::createFromRvec(group.rvecs[i]);
                    t << group.tvecs[i];
                    Eigen::Isometry3d _tf = t * (r * tf);

                    Eigen::Vector3d _t;
                    _t = _tf.translation();
                    group.tvecs[i] << _t;

                    Eigen::Quaterniond _r;
                    _r = _tf.rotation();
                    group.rvecs[i] =
                        cv::Quatd{_r.w(), _r.x(), _r.y(), _r.z()}.toRotVec();
                }
            }
            else if (group.n_matches > 1)
            {
                try
                {
                    cv::solvePnPGeneric(
                        group.obj_points,
                        group.img_points,
                        sub.calibration,
                        sub.distortion,
                        group.rvecs,
                        group.tvecs,
                        false,
                        (group.all_coplanar ? cv::SOLVEPNP_IPPE
                                            : cv::SOLVEPNP_SQPNP),
                        cv::noArray(),
                        cv::noArray(),
                        group.eerrors);
                }
                catch (const std::exception& e)
                {
                    continue;
                }
            }

            const size_t n_solutions = group.tvecs.size();
            if (n_solutions > 0 && n_solutions == group.rvecs.size())
            {
                TagsTransformMsg detection_buff;
                TagsTransformMsg best_detection;
                TagsTransformMsg best_filtered_detection;

                const bool publish_best_detection_tf =
                    this->param.publish_best_tf > 0;
                const bool publish_best_filtered_detection_tf =
                    this->param.publish_best_tf < 0;
                const bool export_best_detection =
                    this->param.export_best_detection > 0;
                const bool export_best_filtered_detection =
                    this->param.export_best_detection < 0;

                double avg_range = 0.;
                for (double r : group.ranges)
                {
                    avg_range += r;
                }
                avg_range /= group.ranges.size();

                for (size_t i = 0; i < n_solutions; i++)
                {
                    const cv::Vec3d& _rvec = group.rvecs[i];
                    const cv::Vec3d& _tvec = group.tvecs[i];

                    if (this->param.enable_debug_stream)
                    {
                        cv::drawFrameAxes(
                            debug_frame,
                            sub.calibration,
                            sub.distortion,
                            _rvec,
                            _tvec,
                            0.5f,
                            5);

                        // draw projected obj points for verification
                        std::vector<cv::Point2f> reproj_points;
                        cv::projectPoints(
                            group.obj_points,
                            _rvec,
                            _tvec,
                            sub.calibration,
                            sub.distortion,
                            reproj_points);
                        for (size_t i = 0; i < reproj_points.size() / 4; i++)
                        {
                            cv::line(
                                debug_frame,
                                reproj_points[i * 4 + 0],
                                reproj_points[i * 4 + 1],
                                cv::Scalar{0, 0, 255},
                                1);
                            cv::line(
                                debug_frame,
                                reproj_points[i * 4 + 1],
                                reproj_points[i * 4 + 2],
                                cv::Scalar{0, 0, 255},
                                1);
                            cv::line(
                                debug_frame,
                                reproj_points[i * 4 + 2],
                                reproj_points[i * 4 + 3],
                                cv::Scalar{0, 0, 255},
                                1);
                            cv::line(
                                debug_frame,
                                reproj_points[i * 4 + 3],
                                reproj_points[i * 4 + 0],
                                cv::Scalar{0, 0, 255},
                                1);
                        }
                    }

                    TransformStampedMsg& full_tf = detection_buff.estimated_tf;
                    TransformMsg& cam_baselink_to_tag_baselink =
                        full_tf.transform;
                    TransformMsg cam_to_tag;
                    cam_to_tag.translation << _tvec;
                    cam_to_tag.rotation << cv::Quatd::createFromRvec(
                        _rvec);  // transforms from camera origin --> tags baselink
                    tf2::doTransform(
                        cam_to_tag,
                        cam_baselink_to_tag_baselink,
                        cam_tf);  // camera baselink --> tags baselink

                    full_tf.header = cam_tf.header;
                    if (this->param.publish_group_solution_tfs)
                    {
                        full_tf.child_frame_id =
                            (std::ostringstream{} << tags_frame << "_e" << i)
                                .str();
                        this->tf_broadcaster.sendTransform(full_tf);
                    }

                    util::geom::Pose3d dynamic_entity_pose;
                    dynamic_entity_pose << cam_baselink_to_tag_baselink;
                    if (group.primary_desc.is_static)
                    {
                        /* The tags are in a static coordinate frame, thus our current transform is from a dynamic coordinate frame
                         * to a static frame, which can also be interpretted as the dynamic frame's pose with respect to the dynamic
                         * frame. We need to inverse the transform so that the dynamic frame ends up as a child, but can use
                         * the current transform as the entity's pose without modification. */
                        // NOTE: It turns out that ROS TransformStamp is defined backwards so this is actually wrong ^
                        util::geom::inverse(
                            dynamic_entity_pose,
                            dynamic_entity_pose);
                        full_tf.transform << dynamic_entity_pose;
                        full_tf.child_frame_id = cam_tf.header.frame_id;
                        full_tf.header.frame_id = tags_frame;
                    }
                    else
                    {
                        /* The tags are in a dynamic coordinate frame, thus our current transform is from a static
                         * frame to a dynamic frame, which can also be interpretted as the static frame's pose with respect
                         * to the dynammic frame. We want the dynamic frame's pose with respect to the static frame, so we
                         * take the inverse of the pose, although we don't need to inverse the transform which we will
                         * be publishing since the dynamic frame is already on the "child" end. */
                        // NOTE: It turns out that ROS TransformStamp is defined backwards so this is actually wrong ^
                        full_tf.child_frame_id = tags_frame;
                    }

                    detection_buff.pix_area = group.sum_area;
                    detection_buff.avg_range = avg_range;
                    detection_buff.rms = group.eerrors[i];
                    detection_buff.num_tags = group.n_matches;

                    const double tags_per_range = group.n_matches / avg_range,
                                 rms_per_tag =
                                     group.eerrors[i] / group.n_matches;
                    const bool in_bounds =
                        (!this->filtering.use_bounds ||
                         this->filtering.filter_bbox.isEmpty() ||
                         this->filtering.filter_bbox.contains(
                             dynamic_entity_pose.vec));
                    const bool tags_per_range_ok =
                        (tags_per_range >=
                         this->filtering.thresh_min_tags_per_range);
                    const bool rms_per_tag_ok =
                        (rms_per_tag <= this->filtering.thresh_max_rms_per_tag);
                    const bool pix_area_ok =
                        (group.sum_area >= this->filtering.thresh_min_pix_area);
                    const bool nonplanar_ok =
                        ((avg_range <=
                          this->filtering.thresh_max_coplanar_dist) ||
                         !group.all_coplanar);

                    detection_buff.filter_mask =
                        (in_bounds << 0) + (tags_per_range_ok << 1) +
                        (rms_per_tag_ok << 2) + (pix_area_ok << 3) +
                        (nonplanar_ok << 4);

                    if ((best_detection.num_tags == 0) ||
                        (detection_buff.rms < best_detection.rms))
                    {
                        best_detection = detection_buff;
                    }

                    if (nonplanar_ok && in_bounds && tags_per_range_ok &&
                        rms_per_tag_ok && pix_area_ok)
                    {
                        if (best_filtered_detection.num_tags == 0 ||
                            detection_buff.rms < best_filtered_detection.rms)
                        {
                            best_filtered_detection = detection_buff;
                        }
                    }

                    if (this->param.export_debug_detections)
                    {
                        this->generic_pub.publish("/tags_detector/debug", detection_buff);
                    }
                }

                if ((best_filtered_detection.num_tags > 0) &&
                    publish_best_filtered_detection_tf)
                {
                    this->tf_broadcaster.sendTransform(
                        best_filtered_detection.estimated_tf);
                }
                else if (
                    (best_detection.num_tags > 0) && publish_best_detection_tf)
                {
                    this->tf_broadcaster.sendTransform(
                        best_detection.estimated_tf);
                }
                if ((best_filtered_detection.num_tags > 0) &&
                    export_best_filtered_detection)
                {
                    this->generic_pub.publish("/cardinal_perception/tags_detections", best_filtered_detection);
                }
                else if ((best_detection.num_tags > 0) && export_best_detection)
                {
                    this->generic_pub.publish("/cardinal_perception/tags_detections", best_detection);
                }
            }
        }
    }

    if (this->param.enable_debug_stream)
    {
        sub.debug_img_pub.publish(
            cv_bridge::CvImage(img->header, "bgr8", debug_frame).toImageMsg());
    }
}

void TagDetector::updateStats(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    this->detection_cb_metrics.addSample(start, end);
    this->process_metrics.update();
    this->generic_pub.publish("/tags_detector/process_stats", this->process_metrics.toMsg());
    this->generic_pub.publish("/tags_detector/detection_cb_metrics", this->detection_cb_metrics.toMsg());
}

};  // namespace perception
};  // namespace csm
