#include "aruco/tag_detector.h"

#include <opencv2/core/quaternion.hpp>


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
    reinterpret_cast<cv::Vec3d&>(_desc->plane) = *c;
    _desc->d = c->dot(reinterpret_cast<cv::Vec3d&>(_desc->translation));
    reinterpret_cast<cv::Vec4d&>(_desc->plane) /= cv::norm(reinterpret_cast<cv::Vec4d&>(_desc->plane));		// Eigen cast and normalize() causes crash :|

    return _desc;
}


TagDetector::TagDetector(int dict_id) :
    aruco_dict{ cv::aruco::getPredefinedDictionary(dict_id) },
    aruco_params{ cv::aruco::DetectorParameters::create() }
{}

bool TagDetector::addTag(int id, const std::vector<double>& pts)
{
    std::scoped_lock lock{ this->params_mtx };
    auto ptr = this->obj_tag_corners.insert({ id, TagDescription::fromRaw(pts) });
    return ptr.second && ptr.first->second;
}

cv::Ptr<cv::aruco::Dictionary> accessDictionary(std::unique_lock& lock)
{
    if(lock.mutex() == &this->params_mtx)
    {
        if(!lock.owns_lock()) lock.lock();
    }
    else lock = std::unique_lock{ this->params_mtx };    // locks the mtx

    return this->aruco_dict;
}

cv::Ptr<cv::aruco::DetectorParameters> accessParams(std::unique_lock& lock)
{
    if(lock.mutex() == &this->params_mtx)
    {
        if(!lock.owns_lock()) lock.lock();
    }
    else lock = std::unique_lock{ this->params_mtx };    // locks the mtx

    return this->aruco_params;
}