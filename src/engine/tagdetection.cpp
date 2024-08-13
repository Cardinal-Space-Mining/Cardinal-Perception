#include "./perception.hpp"
#include "conversions.hpp"

#include <sstream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgproc.hpp>


using namespace util::cvt;

TagDescription::Ptr TagDescription::fromRaw(const std::vector<double>& pts)
{
    if(pts.size() < 12) return nullptr;
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

    const double
        len = cv::norm(*a),
        half_len = len / 2.;

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
    reinterpret_cast<cv::Point3d&>(_desc->translation) = (*_0 + *_2) / 2.;
    reinterpret_cast<cv::Quatd&>(_desc->rotation) = cv::Quatd::createFromRotMat(rmat);
    _desc->qww = _desc->qw;
    reinterpret_cast<cv::Vec3d&>(_desc->plane) = *c;
    _desc->d = c->dot(reinterpret_cast<cv::Vec3d&>(_desc->translation));
    reinterpret_cast<cv::Vec4d&>(_desc->plane) /= cv::norm(reinterpret_cast<cv::Vec4d&>(_desc->plane));		// Eigen cast and normalize() causes crash :|

    return _desc;
}


PerceptionNode::TagDetector::TagDetector(PerceptionNode* inst) :
    pnode{ inst },
    aruco_params{ cv::aruco::DetectorParameters::create() }
{
    // RCLCPP_INFO(this->pnode->get_logger(), "TAGS CONSTRUCTOR INIT");
    this->getParams();
    // RCLCPP_INFO(this->pnode->get_logger(), "TAGS CONSTRUCTOR EXIT");
}

void PerceptionNode::TagDetector::getParams()
{
    if(!this->pnode) return;

    int aruco_dict_id = cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_APRILTAG_36h11;
    util::declare_param(this->pnode, "aruco.predefined_family_idx", aruco_dict_id, aruco_dict_id);
    this->aruco_dict = cv::aruco::getPredefinedDictionary(aruco_dict_id);

    std::vector<double> tag_ids;
    util::declare_param(this->pnode, "aruco.tag_ids", tag_ids, {});

    const size_t n_tags = tag_ids.size();
    if(n_tags < 1) return;

    std::vector<double> parm_buff;
    for(size_t i = 0; i < n_tags; i++)
    {
        parm_buff.clear();
        const int id = static_cast<int>(tag_ids[i]);

        std::ostringstream topic;
        topic << "aruco.tag" << id;
        util::declare_param(this->pnode, topic.str(), parm_buff, {});

        auto ptr = this->obj_tag_corners.insert({ id, TagDescription::fromRaw(parm_buff) });
        if(!ptr.second || !ptr.first->second)
        {
            this->obj_tag_corners.erase(id);
        }
    }
}

void PerceptionNode::TagDetector::processImg(
    const sensor_msgs::msg::Image::ConstSharedPtr& img,
    PerceptionNode::CameraSubscriber& sub,
    std::vector<TagDetection::Ptr>& detections)
{
    // RCLCPP_INFO(this->pnode->get_logger(), "IMG PROC EXHIBIT A -- %s", (std::stringstream{} << std::this_thread::get_id()).str().c_str());
    detections.clear();
    if(!sub.valid_calib) return;

    std::unique_lock<std::mutex> frame_lock{};
    // cv::Mat& debug_frame = sub.dbg_frame.A(frame_lock);
    cv::Mat& debug_frame = sub.dbg_frame.lock(frame_lock);

    cv_bridge::CvImageConstPtr cv_img = cv_bridge::toCvCopy(*img, "mono8");
    cv::cvtColor(cv_img->image, debug_frame, CV_GRAY2BGR);

    thread_local std::vector<std::vector<cv::Point2f>> tag_corners;
    thread_local std::vector<int> tag_ids;
    thread_local std::vector<cv::Point2f> img_points;
    thread_local std::vector<cv::Point3f> obj_points;
    thread_local std::vector<cv::Vec3d> tvecs, rvecs;
    thread_local std::vector<double> eerrors;

    tag_corners.clear();
    tag_ids.clear();

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
        return;
    }

    cv::aruco::drawDetectedMarkers(debug_frame, tag_corners, tag_ids, cv::Scalar{0, 255, 0});

    const size_t n_detected = tag_ids.size();
    if(n_detected > 0 && n_detected == tag_corners.size())
    {
        rvecs.clear();
        tvecs.clear();
        eerrors.clear();

        obj_points.clear();
        obj_points.reserve(n_detected * 4);
        img_points.clear();
        img_points.reserve(n_detected * 4);

        size_t matches = 0;
        bool all_coplanar = true;
        cv::Vec4d tags_plane = cv::Vec4d::zeros();
        TagDescription::ConstPtr primary_desc;
        std::vector<double> ranges;
        double sum_area = 0.;
        // double highest_individual_rms = 0.;

        ranges.reserve(n_detected * 2);
        for(size_t i = 0; i < n_detected; i++)
        {
            auto search = this->obj_tag_corners.find(tag_ids[i]);
            if(search != this->obj_tag_corners.end())
            {
                auto& result = search->second;
                auto& obj_corners = result->world_corners;
                auto& rel_obj_corners = result->rel_corners;
                auto& img_corners = tag_corners[i];

                obj_points.insert(obj_points.end(), obj_corners.begin(), obj_corners.end());
                img_points.insert(img_points.end(), img_corners.begin(), img_corners.end());

                if(++matches == 1) primary_desc = result;
                sum_area += cv::contourArea(img_corners);

                cv::solvePnPGeneric(
                    rel_obj_corners,
                    img_corners,
                    sub.calibration,
                    sub.distortion,
                    rvecs,
                    tvecs,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE,
                    cv::noArray(),
                    cv::noArray(),
                    eerrors);

                // if(eerrors[0] > highest_individual_rms) highest_individual_rms = eerrors[0];

                geometry_msgs::msg::TransformStamped dbg_tf;
                dbg_tf.header = cv_img->header;
                for(size_t s = 0; s < tvecs.size(); s++)
                {
                    cv::Vec3d& _rvec = rvecs[s];
                    cv::Vec3d& _tvec = tvecs[s];

                    ranges.push_back(cv::norm(_tvec));

                    std::ostringstream cframe;
                    cframe << "tag_" << tag_ids[i] << '_' << s;

                    dbg_tf.child_frame_id = cframe.str();
                    dbg_tf.transform.translation << _tvec;
                    dbg_tf.transform.rotation << cv::Quatd::createFromRvec(_rvec);

                    this->pnode->tf_broadcaster.sendTransform(dbg_tf);
                }

                if(all_coplanar)
                {
                    const cv::Vec4d& _p = reinterpret_cast<const cv::Vec4d&>(result->plane);
                    if(matches == 1) tags_plane = _p;
                    else if(1. - std::abs(tags_plane.dot(_p)) > 1e-6) all_coplanar = false;
                }
            }
        }

        double avg_range = 0.;
        for(double r : ranges)
        {
            avg_range += r;
        }
        avg_range /= ranges.size();

        if(matches == 1)
        {
            Eigen::Isometry3d tf = ( Eigen::Quaterniond{ primary_desc->qw, primary_desc->qx, primary_desc->qy, primary_desc->qz } *
                Eigen::Translation3d{ -reinterpret_cast<const Eigen::Vector3d&>(primary_desc->translation) } );

            const size_t n_solutions = tvecs.size();
            for(size_t i = 0; i < n_solutions; i++)
            {
                Eigen::Quaterniond r;
                Eigen::Translation3d t;
                r << cv::Quatd::createFromRvec(rvecs[i]);
                t << tvecs[i];
                Eigen::Isometry3d _tf = t * (r * tf);

                Eigen::Vector3d _t;
                _t = _tf.translation();
                tvecs[i] = reinterpret_cast<cv::Vec3d&>(_t);

                Eigen::Quaterniond _r;
                _r = _tf.rotation();
                rvecs[i] = cv::Quatd{ _r.w(), _r.x(), _r.y(), _r.z() }.toRotVec();
            }

            // log status
        }
        else if(matches > 1)
        {
            try
            {
                cv::solvePnPGeneric(
                    obj_points,
                    img_points,
                    sub.calibration,
                    sub.distortion,
                    rvecs,
                    tvecs,
                    false,
                    (all_coplanar ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_SQPNP),
                    cv::noArray(),
                    cv::noArray(),
                    eerrors);

                // log status
            }
            catch(const std::exception& e)
            {
                // fail
            }
        }

        const size_t n_solutions = tvecs.size();
        if(n_solutions > 0 && n_solutions == rvecs.size())
        {
            geometry_msgs::msg::TransformStamped cam2base, cam2world, world2cam, world2base;
            cam2world.header = cv_img->header;
            bool valid_cam2base = true;
            try
            {
                cam2base = this->pnode->tf_buffer.lookupTransform(
                    cv_img->header.frame_id,
                    this->pnode->base_frame,
                    util::toTf2TimePoint(cv_img->header.stamp));
            }
            catch(const std::exception& e)
            {
                // log
                valid_cam2base = false;
            }

            for(size_t i = 0; i < n_solutions; i++)
            {
                cv::Vec3d& _rvec = rvecs[i];
                cv::Vec3d& _tvec = tvecs[i];

                cv::drawFrameAxes(debug_frame, sub.calibration, sub.distortion, _rvec, _tvec, 0.5f, 5);

                Eigen::Translation3d t;
                Eigen::Quaterniond q;
                t << _tvec;
                q << cv::Quatd::createFromRvec(_rvec);

                std::ostringstream tframe;
                tframe << "tags_origin_" << i;
                cam2world.child_frame_id = tframe.str();
                cam2world.transform.translation << t;
                cam2world.transform.rotation << q;

                this->pnode->tf_broadcaster.sendTransform(cam2world);

                if(!valid_cam2base) continue;

                Eigen::Isometry3d _w2cam = (t * q).inverse();
                world2cam.transform << _w2cam;

                tf2::doTransform(cam2base, world2base, world2cam);

                detections.emplace_back(std::make_shared<TagDetection>());
                TagDetection::Ptr& _d = detections.back();

                _d->translation << world2base.transform.translation;
                _d->rotation << world2base.transform.rotation;

                _d->time_point = util::toFloatSeconds(cv_img->header.stamp);
                _d->pix_area = sum_area;
                _d->avg_range = avg_range;
                _d->rms = eerrors[i];
                _d->num_tags = matches;
            }
        }
    }
    // sub.dbg_frame.try_spin();
}
