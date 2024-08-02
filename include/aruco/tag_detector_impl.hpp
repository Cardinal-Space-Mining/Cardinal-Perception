#include "tag_detector.hpp"

#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>


template<bool debug_enabled>
void TagDetector::process(
    const cv::Mat& frame,
    cv::InputArray calib,
    cv::InputArray distort,
    const Eigen::Isometry3d& c2base,
    std::vector<Detection::Ptr>& detections,
    cv::OutputArray debug = cv::noArray())
{
    // if( !(frame.channels() == 1 || frame.channels() == 3) || frame.size().area() < 1 ) return;

    // const bool _debug_frame = (&debug != &cv::noArray());

    // thread_local cv::Mat _grey;
    // thread_local std::vector<std::vector<cv::Point2f>> _tag_corners;
    // thread_local std::vector<int> tag_ids;
    // thread_local std::vector<cv::Point2f> img_points;
    // thread_local std::vector<cv::Point3f> obj_points;
    // thread_local std::vector<cv::Vec3d> tvecs, rvecs;
    // thread_local std::vector<double> eerrors;

    // if(frame.channels() == 1) _grey = frame;
    // else cv::cvtColor(frame, _grey, CV_BGR2GRAY);

    // if constexpr(debug_enabled)
    // {
    //     if(_debug_frame) cv::cvtColor(_grey, debug, CV_GRAY2BGR);
    // }

    // _tag_corners.clear();
    // _tag_ids.clear();

    // try {
    //     cv::aruco::detectMarkers(
    //         _grey,
    //         this->aruco_dict,
    //         _tag_corners,
    //         _tag_ids,
    //         this->aruco_params);
    // }
}