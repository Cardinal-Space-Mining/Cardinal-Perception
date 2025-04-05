#pragma once

#include <point_def.hpp>


namespace csm
{
namespace perception
{
    using OdomPointType = pcl::PointXYZ;
    using MappingPointType = pcl::PointXYZ;
    using FiducialPointType = csm::perception::PointXYZR;
    using CollisionPointType = pcl::PointXYZLNormal;

};
};
