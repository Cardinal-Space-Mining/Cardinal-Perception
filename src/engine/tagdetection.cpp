#include "perception.hpp"

#include <sstream>

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


TagDetector::TagDetector(PerceptionNode* inst) :
    pnode{ inst },
    aruco_params{ cv::aruco::DetectorParameters::create() }
{
    this->getParams();
}

void TagDetector::getParams()
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
