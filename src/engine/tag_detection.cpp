#include "tag_detection.hpp"

#include <sstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef USE_LEGACY_CV_BRIDGE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif


using namespace util::geom::cvt::ops;

TagDescription::Ptr TagDescription::fromRaw(const std::vector<double>& pts, const std::vector<std::string>& frames, bool is_static)
{
    if(pts.size() < 12 || frames.size() < 1) return nullptr;
    Ptr _desc = std::make_shared<TagDescription>();

    const cv::Point3d
        *_0 = reinterpret_cast<const cv::Point3d*>(pts.data() + 0),
        *_1 = reinterpret_cast<const cv::Point3d*>(pts.data() + 3),
        *_2 = reinterpret_cast<const cv::Point3d*>(pts.data() + 6),
        *_3 = reinterpret_cast<const cv::Point3d*>(pts.data() + 9);

    cv::Matx33d rmat;	// rotation matrix from orthogonal axes
    cv::Vec3d			// fill in each row
        *a = reinterpret_cast<cv::Vec3d*>(rmat.val + 0),
        *b = reinterpret_cast<cv::Vec3d*>(rmat.val + 3),
        *c = reinterpret_cast<cv::Vec3d*>(rmat.val + 6);

    *a = *_1 - *_0;	// x-axis

    const double len = cv::norm(*a);
    const float half_len = static_cast<float>(len / 2.);

    *b = *_1 - *_2;	// y-axis
    *c = ( *a /= len ).cross( *b /= len );	// z-axis can be dervied from x and y

    _desc->world_corners = {
        static_cast<cv::Point3f>(*_0),
        static_cast<cv::Point3f>(*_1),
        static_cast<cv::Point3f>(*_2),
        static_cast<cv::Point3f>(*_3)
    };
    _desc->rel_corners = {
        cv::Point3f{ -half_len, +half_len, 0.f },
        cv::Point3f{ +half_len, +half_len, 0.f },
        cv::Point3f{ +half_len, -half_len, 0.f },
        cv::Point3f{ -half_len, -half_len, 0.f }
    };
    _desc->translation << (*_0 + *_2) / 2.;
    _desc->rotation << cv::Quatd::createFromRotMat(rmat);
    _desc->plane[0] = c->operator[](0);
    _desc->plane[1] = c->operator[](1);
    _desc->plane[2] = c->operator[](2);
    _desc->plane[3] = _desc->plane.block<3, 1>(0, 0).dot(_desc->translation);
    _desc->plane.normalize();

    _desc->frame_id = frames[0];
    if(frames.size() > 1) _desc->base_frame = frames[1];
    _desc->is_static = is_static;

    return _desc;
}

TagDetector::TagDetector() :
    Node("cardinal_perception_tag_detection"),
    tf_buffer{ std::make_shared<rclcpp::Clock>(RCL_ROS_TIME) },
    tf_listener{ tf_buffer },
    tf_broadcaster{ *this },
    img_transport{ std::shared_ptr<TagDetector>(this, [](auto*){}) },
    mt_callback_group{ this->create_callback_group(rclcpp::CallbackGroupType::Reentrant) },
    detection_pub{ this->create_publisher<cardinal_perception::msg::TagsTransform>("tags_detections", rclcpp::SensorDataQoS{}) },
    debug_pub{ this->create_publisher<cardinal_perception::msg::TagsTransform>("/tags_detector/debug", rclcpp::SensorDataQoS{}) },
    proc_metrics_pub{ this->create_publisher<cardinal_perception::msg::ProcessMetrics>("/tags_detector/process_metrics", rclcpp::SensorDataQoS{}) },
    detection_metrics_pub{ this->create_publisher<cardinal_perception::msg::ThreadMetrics>("/tags_detector/detection_cb_metrics", rclcpp::SensorDataQoS{}) },
    aruco_params{ cv::aruco::DetectorParameters::create() }
    // metrics_pub{ this, "/tags_detector/", 1 }
{
    this->getParams();
}

TagDetector::CameraSubscriber::CameraSubscriber(
    const CameraSubscriber& ref
) :
    node{ ref.node },
    calibration{ ref.calibration.clone() },
    distortion{ ref.distortion.clone() },
    cam_frame_override{ ref.cam_frame_override },
    base_tf_frame{ ref.base_tf_frame },
    offset{ ref.offset },
    valid_calib{ ref.valid_calib }
{}

void TagDetector::CameraSubscriber::initialize(
    TagDetector* inst,
    const std::string& img_topic,
    const std::string& info_topic)
{
    if(!inst) return;
    this->node = inst;

    rclcpp::SubscriptionOptions ops{};
    ops.callback_group = this->node->mt_callback_group;

    this->image_sub = this->node->img_transport.subscribe(
        img_topic, rmw_qos_profile_sensor_data.depth,
        [this](const image_transport::ImageTransport::ImageConstPtr & img){ this->img_callback(img); },
        image_transport::ImageTransport::VoidPtr(), nullptr, ops);
    this->info_sub = this->node->create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS{},
        [this](const image_transport::ImageTransport::CameraInfoConstPtr& info){ this->info_callback(info); }, ops);

    this->debug_img_pub = this->node->img_transport.advertise(img_topic + "/debug_output", rmw_qos_profile_sensor_data.depth);
}

void TagDetector::CameraSubscriber::img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& img)
{
    auto _start = std::chrono::system_clock::now();
    this->node->processImg(img, *this);
    auto _end = std::chrono::system_clock::now();
    this->node->updateStats(_start, _end);
}

void TagDetector::CameraSubscriber::info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
{
    if( !this->valid_calib &&
        info->k.size() == 9 &&
        info->d.size() >= 5 )
    {
        this->calibration = cv::Mat(info->k, true).reshape(0, 3);
        this->distortion = cv::Mat(info->d, true).reshape(0, 1);

        this->valid_calib = true;
    }
}

void TagDetector::getParams()
{
    this->declare_parameter("image_transport", "raw");

    util::declare_param(this, "feature.publish_best_detection_tf", this->param.publish_best_tf, 0);
    util::declare_param(this, "feature.export_best_detection", this->param.export_best_detection, 1);
    util::declare_param(this, "feature.debug.export_all_detections", this->param.export_debug_detections, false);
    util::declare_param(this, "feature.debug.publish_stream", this->param.enable_debug_stream, true);
    util::declare_param(this, "feature.debug.publish_individual_tag_solution_tfs", this->param.publish_individual_tag_solution_tfs, true);
    util::declare_param(this, "feature.debug.publish_group_solution_tfs", this->param.publish_group_solution_tfs, true);

    std::vector<double> min, max;
    util::declare_param(this, "filtering.bounds_min", min, {});
    util::declare_param(this, "filtering.bounds_max", max, {});
    if(min.size() > 2 && max.size() > 2)
    {
        this->filtering.filter_bbox = Eigen::AlignedBox3d{ *reinterpret_cast<Eigen::Vector3d*>(min.data()), *reinterpret_cast<Eigen::Vector3d*>(max.data()) };
    }
    util::declare_param(this, "filtering.use_bounds", this->filtering.use_bounds, true);
    util::declare_param(this, "filtering.thresh.min_tags_per_range", this->filtering.thresh_min_tags_per_range, 0.5);
    util::declare_param(this, "filtering.thresh.max_rms_per_tag", this->filtering.thresh_max_rms_per_tag, 0.1);
    util::declare_param(this, "filtering.thresh.min_sum_pix_area", this->filtering.thresh_min_pix_area, 10000.);
    util::declare_param(this, "filtering.thresh.require_nonplanar_after", this->filtering.thresh_max_coplanar_dist, 2.);

    int aruco_dict_id = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
    util::declare_param(this, "aruco.predefined_family_idx", aruco_dict_id, aruco_dict_id);
    this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);

    std::vector<double> tag_ids;
    util::declare_param(this, "aruco.tag_ids", tag_ids, {});

    const size_t n_tags = tag_ids.size();
    if(n_tags < 1) return;

    std::vector<double> corners_buff;
    corners_buff.reserve(12);
    std::vector<std::string> str_param_buff;
    str_param_buff.reserve(4);
    for(size_t i = 0; i < n_tags; i++)
    {
        corners_buff.clear();
        str_param_buff.clear();
        const int id = static_cast<int>(tag_ids[i]);
        bool is_static = true;

        util::declare_param(this, (std::ostringstream{} << "aruco.tag" << id << "_corners").str(), corners_buff, {});
        util::declare_param(this, (std::ostringstream{} << "aruco.tag" << id << "_frames").str(), str_param_buff, {});
        util::declare_param(this, (std::ostringstream{} << "aruco.tag" << id << "_static").str(), is_static, true);

        auto ptr = this->tag_descriptions.try_emplace(id, TagDescription::fromRaw(corners_buff, str_param_buff, is_static));
        if(!ptr.second || !ptr.first->second)
        {
            this->tag_descriptions.erase(id);
        }
    }

    int n_streams = 0;
    util::declare_param(this, "num_streams", n_streams, 0);
    std::vector<double> offset_pose;
    offset_pose.reserve(7);
    this->camera_subs.resize(n_streams);
    for(int i = 0; i < n_streams; i++)
    {
        str_param_buff.clear();
        offset_pose.clear();

        std::ostringstream param;
        param << "stream" << i;
        util::declare_param(this, param.str(), str_param_buff, {}); // config consists of string list: ["/img/topic", "/info/topic", "frame_override", "base_frame"]
        if(str_param_buff.size() < 2) continue;
        param << "_offset";
        util::declare_param(this, param.str(), offset_pose, {});    // offset to actual camera origin within camera frame

        auto& _sub = this->camera_subs[i];
        if(str_param_buff.size() > 2) _sub.cam_frame_override = str_param_buff[2];
        if(str_param_buff.size() > 3) _sub.base_tf_frame = str_param_buff[3];
        if(offset_pose.size() == 4)
        {
            _sub.offset.quat.w() = offset_pose[0];
            _sub.offset.quat.x() = offset_pose[1];
            _sub.offset.quat.y() = offset_pose[2];
            _sub.offset.quat.z() = offset_pose[3];
        }
        else if(offset_pose.size() >= 3)
        {
            _sub.offset.vec.x() = offset_pose[0];
            _sub.offset.vec.y() = offset_pose[1];
            _sub.offset.vec.z() = offset_pose[2];
            if(offset_pose.size() >= 7)
            {
                _sub.offset.quat.w() = offset_pose[3];
                _sub.offset.quat.x() = offset_pose[4];
                _sub.offset.quat.y() = offset_pose[5];
                _sub.offset.quat.z() = offset_pose[6];
            }
        }
        _sub.initialize(this, str_param_buff[0], str_param_buff[1]);
    }
}

void TagDetector::processImg(const sensor_msgs::msg::Image::ConstSharedPtr& img, CameraSubscriber& sub)
{
    if(!sub.valid_calib) return;

    thread_local cv::Mat debug_frame;
    thread_local std::vector<std::vector<cv::Point2f>> tag_corners;
    thread_local std::vector<int> tag_ids;

    cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");
    if(this->param.enable_debug_stream) cv::cvtColor(cv_img->image, debug_frame, CV_GRAY2BGR);

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
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "[TAG DETECTION]: Encountered exception while detecting markers!\n\twhat(): %s", e.what());
        return;
    }

    if(this->param.enable_debug_stream) cv::aruco::drawDetectedMarkers(debug_frame, tag_corners, tag_ids, cv::Scalar{0, 255, 0});

    const size_t n_detected = tag_ids.size();
    if(n_detected > 0 && n_detected == tag_corners.size())
    {

        const std::string& cam_frame_id = sub.cam_frame_override.empty() ? img->header.frame_id : sub.cam_frame_override;
        const std::string& cam_base_frame = sub.base_tf_frame;
        const bool need_cam_base_tf = !cam_base_frame.empty() && cam_base_frame != cam_frame_id;

        geometry_msgs::msg::TransformStamped cam_tf;   // effective transform which converts from lowest baselink frame to camera origin
        // util::geom::Pose3d _cam_tf;
        // util::geom::inverse(_cam_tf, sub.offset);
        // cam_tf.transform << _cam_tf;   // current tf: camera frame --> camera origin
        cam_tf.transform << sub.offset;
        cam_tf.header.stamp = img->header.stamp;
        cam_tf.header.frame_id = cam_frame_id;

        struct DetectionGroup   // detections get lumped based on the lowest coordinate frame which they can be statically transformed into
        {
            std::vector<cv::Point2f> img_points;
            std::vector<cv::Point3f> obj_points;
            std::vector<cv::Vec3d> tvecs, rvecs;
            std::vector<double> eerrors;
            std::vector<double> ranges;
            Eigen::Vector4d detection_plane = Eigen::Vector4d::Zero();
            TagDescription::ConstPtr primary_desc = nullptr;
            double sum_area = 0.;
            size_t n_matches = 0;
            bool all_coplanar = true;

            DetectionGroup()
            {
                img_points.reserve(8);
                obj_points.reserve(8);
            }
        };

        thread_local std::unordered_map<std::string, DetectionGroup> detection_groups;
        detection_groups.clear();
        detection_groups.reserve(n_detected);

        for(size_t i = 0; i < n_detected; i++)
        {
            auto search = this->tag_descriptions.find(tag_ids[i]);
            if(search != this->tag_descriptions.end())
            {
                TagDescription::ConstPtr& tag_description = search->second;

                const bool use_baselink_tf =
                    !tag_description->base_frame.empty() &&
                    tag_description->frame_id != tag_description->base_frame &&
                    this->tf_buffer.canTransform(
                        tag_description->frame_id,
                        tag_description->base_frame,
                        util::toTf2TimePoint(img->header.stamp) );
                const std::string& target_frame = use_baselink_tf ? tag_description->base_frame : tag_description->frame_id;

                auto itr = detection_groups.try_emplace(target_frame).first;
                if(itr == detection_groups.end()) continue;

                DetectionGroup& group = itr->second;
                auto& obj_corners = tag_description->world_corners;
                auto& rel_obj_corners = tag_description->rel_corners;
                auto& img_corners = tag_corners[i];

                if(use_baselink_tf)
                {
                    const size_t obj_points_off = group.obj_points.size();
                    group.obj_points.resize(group.obj_points.size() + 4);
                    try
                    {
                        auto tf = this->tf_buffer.lookupTransform(
                            tag_description->base_frame,
                            tag_description->frame_id,
                            util::toTf2TimePoint(img->header.stamp) );

                        Eigen::Isometry3f _tf;
                        _tf << tf.transform;

                        Eigen::Vector3f p;
                        for(size_t i = 0; i < 4; i++)
                        {
                            group.obj_points[i + obj_points_off] << (_tf * (p << obj_corners[i]));
                        }
                    }
                    catch(const std::exception& e)
                    {
                        continue;   // could handle by reverting to main tag frame?
                    }
                }
                else
                {
                    group.obj_points.insert(group.obj_points.end(), obj_corners.begin(), obj_corners.end());
                }
                group.img_points.insert(group.img_points.end(), img_corners.begin(), img_corners.end());

                if(++group.n_matches == 1)
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
                    group.eerrors );

                geometry_msgs::msg::TransformStamped dbg_tf;
                dbg_tf.header = cv_img->header;
                for(size_t s = 0; s < group.tvecs.size(); s++)
                {
                    cv::Vec3d& _rvec = group.rvecs[s];
                    cv::Vec3d& _tvec = group.tvecs[s];

                    group.ranges.push_back(cv::norm(_tvec));

                    if(this->param.publish_individual_tag_solution_tfs)
                    {
                        std::ostringstream cframe;
                        cframe << "tag_" << tag_ids[i] << '_' << s;

                        dbg_tf.child_frame_id = cframe.str();
                        dbg_tf.transform.translation << _tvec;
                        dbg_tf.transform.rotation << cv::Quatd::createFromRvec(_rvec);
                        tf2::doTransform(dbg_tf, dbg_tf, cam_tf);   // apply camera origin offset

                        this->tf_broadcaster.sendTransform(dbg_tf);
                    }
                }

                if(group.all_coplanar)
                {
                    if(group.n_matches == 1) group.detection_plane = tag_description->plane;
                    else if(1. - std::abs(group.detection_plane.dot(tag_description->plane)) > 1e-6) group.all_coplanar = false;
                }
            }
        }

        if(need_cam_base_tf)
        {
            try
            {
                auto tf = this->tf_buffer.lookupTransform(  // might need to swap order
                    cam_frame_id,
                    cam_base_frame,
                    util::toTf2TimePoint(cv_img->header.stamp));    // camera baselink --> camera frame

                tf2::doTransform(cam_tf, cam_tf, tf);   // current tf: camera baselink --> camera origin
                // cam_tf.header.frame_id = cam_base_frame;
            }
            catch(const std::exception& e) {}
        }

        for(auto itr = detection_groups.begin(); itr != detection_groups.end(); itr++)
        {
            const std::string& tags_frame = itr->first;
            DetectionGroup& group = itr->second;

            if(group.n_matches == 1)
            {
                Eigen::Isometry3d tf = group.primary_desc->rotation * Eigen::Translation3d{ -group.primary_desc->translation };
                // TODO: make this simpler >>

                // convert each rvec,tvec to be relative to tags frame not center of tag
                const size_t n_solutions = group.tvecs.size();
                for(size_t i = 0; i < n_solutions; i++)
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
                    group.rvecs[i] = cv::Quatd{ _r.w(), _r.x(), _r.y(), _r.z() }.toRotVec();
                }
            }
            else if(group.n_matches > 1)
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
                        (group.all_coplanar ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_SQPNP),
                        cv::noArray(),
                        cv::noArray(),
                        group.eerrors );
                }
                catch(const std::exception& e)
                {
                    continue;
                }
            }
            const size_t n_solutions = group.tvecs.size();
            if(n_solutions > 0 && n_solutions == group.rvecs.size())
            {
                cardinal_perception::msg::TagsTransform
                    detection_buff, best_detection, best_filtered_detection;

                const bool
                    publish_best_detection_tf = this->param.publish_best_tf > 0,
                    publish_best_filtered_detection_tf = this->param.publish_best_tf < 0,
                    export_best_detection = this->param.export_best_detection > 0,
                    export_best_filtered_detection = this->param.export_best_detection < 0;

                double avg_range = 0.;
                for(double r : group.ranges)
                {
                    avg_range += r;
                }
                avg_range /= group.ranges.size();

                for(size_t i = 0; i < n_solutions; i++)
                {
                    const cv::Vec3d& _rvec = group.rvecs[i];
                    const cv::Vec3d& _tvec = group.tvecs[i];

                    if(this->param.enable_debug_stream) cv::drawFrameAxes(debug_frame, sub.calibration, sub.distortion, _rvec, _tvec, 0.5f, 5);

                    geometry_msgs::msg::TransformStamped& full_tf = detection_buff.estimated_tf;
                    geometry_msgs::msg::Transform cam_to_tag, &cam_baselink_to_tag_baselink = full_tf.transform;
                    cam_to_tag.translation << _tvec;
                    cam_to_tag.rotation << cv::Quatd::createFromRvec(_rvec);    // transforms from camera origin --> tags baselink
                    tf2::doTransform(cam_to_tag, cam_baselink_to_tag_baselink, cam_tf); // camera baselink --> tags baselink

                    full_tf.header = cam_tf.header;
                    if(this->param.publish_group_solution_tfs)
                    {
                        full_tf.child_frame_id = (std::ostringstream{} << tags_frame << "_e" << i).str();
                        this->tf_broadcaster.sendTransform(full_tf);
                    }

                    util::geom::Pose3d dynamic_entity_pose;
                    dynamic_entity_pose << cam_baselink_to_tag_baselink;
                    if(group.primary_desc->is_static)
                    {
                        /* The tags are in a static coordinate frame, thus our current transform is from a dynamic coordinate frame
                         * to a static frame, which can also be interpretted as the dynamic frame's pose with respect to the dynamic
                         * frame. We need to inverse the transform so that the dynamic frame ends up as a child, but can use
                         * the current transform as the entity's pose without modification. */
                        // NOTE: It turns out that ROS TransformStamp is defined backwards so this is actually wrong ^
                        util::geom::inverse(dynamic_entity_pose, dynamic_entity_pose);
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

                    const double
                        tags_per_range = group.n_matches / avg_range,
                        rms_per_tag = group.eerrors[i] / group.n_matches;
                    const bool
                        in_bounds = !this->filtering.use_bounds || this->filtering.filter_bbox.isEmpty() ||
                            this->filtering.filter_bbox.contains(dynamic_entity_pose.vec),
                        tags_per_range_ok = tags_per_range >= this->filtering.thresh_min_tags_per_range,
                        rms_per_tag_ok = rms_per_tag <= this->filtering.thresh_max_rms_per_tag,
                        pix_area_ok = group.sum_area >= this->filtering.thresh_min_pix_area,
                        nonplanar_ok = avg_range <= this->filtering.thresh_max_coplanar_dist || !group.all_coplanar;

                    detection_buff.filter_mask =
                        (in_bounds << 0) +
                        (tags_per_range_ok << 1) +
                        (rms_per_tag_ok << 2) +
                        (pix_area_ok << 3) +
                        (nonplanar_ok << 4);

                    if((best_detection.num_tags == 0) || (detection_buff.rms < best_detection.rms))
                    {
                        best_detection = detection_buff;
                    }

                    if(nonplanar_ok && in_bounds && tags_per_range_ok && rms_per_tag_ok && pix_area_ok)
                    {
                        if(best_filtered_detection.num_tags == 0 || detection_buff.rms < best_filtered_detection.rms)
                        {
                            best_filtered_detection = detection_buff;
                        }
                    }

                    if(this->param.export_debug_detections)
                    {
                        this->debug_pub->publish(detection_buff);
                    }
                }

                if((best_filtered_detection.num_tags > 0) && publish_best_filtered_detection_tf)
                {
                    this->tf_broadcaster.sendTransform(best_filtered_detection.estimated_tf);
                }
                else if((best_detection.num_tags > 0) && publish_best_detection_tf)
                {
                    this->tf_broadcaster.sendTransform(best_detection.estimated_tf);
                }
                if((best_filtered_detection.num_tags > 0) && export_best_filtered_detection)
                {
                    this->detection_pub->publish(best_filtered_detection);
                }
                else if((best_detection.num_tags > 0) && export_best_detection)
                {
                    this->detection_pub->publish(best_detection);
                }
            }
        }

    }
    if(this->param.enable_debug_stream) sub.debug_img_pub.publish(cv_bridge::CvImage(img->header, "bgr8", debug_frame).toImageMsg());

}

void TagDetector::updateStats(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end)
{
    this->detection_cb_metrics.addSample(start, end);
    this->process_metrics.update();

    double mem;
    size_t threads;
    util::proc::getProcessStats(mem, threads);

    cardinal_perception::msg::ProcessMetrics pm;
    pm.cpu_percent = static_cast<float>(this->process_metrics.last_cpu_percent);
    pm.avg_cpu_percent = static_cast<float>(this->process_metrics.avg_cpu_percent);
    pm.mem_usage_mb = static_cast<float>(mem);
    pm.num_threads = static_cast<uint32_t>(threads);
    this->proc_metrics_pub->publish(pm);

    cardinal_perception::msg::ThreadMetrics tm;
    tm.delta_t = static_cast<float>(this->detection_cb_metrics.last_comp_time);
    tm.avg_delta_t = static_cast<float>(this->detection_cb_metrics.avg_comp_time);
    tm.avg_freq = static_cast<float>(1. / this->detection_cb_metrics.avg_call_delta);
    tm.iterations = this->detection_cb_metrics.samples;
    this->detection_metrics_pub->publish(tm);

    // this->metrics_pub.publish("process/cpu_percent", this->process_metrics.last_cpu_percent);
    // this->metrics_pub.publish("process/avg_cpu_percent", this->process_metrics.avg_cpu_percent);
    // this->metrics_pub.publish("process/mem_usage_mb", mem);
    // this->metrics_pub.publish("process/num_threads", (double)threads);

    // this->metrics_pub.publish("detection_cb/proc_time", this->detection_cb_metrics.last_comp_time);
    // this->metrics_pub.publish("detection_cb/avg_proc_time", this->detection_cb_metrics.avg_comp_time);
    // this->metrics_pub.publish("detection_cb/iterations", this->detection_cb_metrics.samples);
    // this->metrics_pub.publish("detection_cb/avg_freq", 1. / this->detection_cb_metrics.avg_call_delta);
}
