#pragma once

#include <pcl/point_types.h>


namespace csm
{
namespace perception
{
    using OdomPointType = pcl::PointXYZ;
    using CollisionPointType = pcl::PointXYZLNormal;
    using MappingPointType = pcl::PointXYZ;
};
};
