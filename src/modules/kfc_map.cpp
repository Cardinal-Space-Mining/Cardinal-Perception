#include "../config.hpp"

#ifdef KFC_MAP_PRECOMPILED
    #undef KFC_MAP_PRECOMPILED
#endif

#include "kfc_map.hpp"


using namespace csm::perception;

template class KFCMap<
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType>;

#if PERCEPTION_USE_NULL_RAY_DELETION
template KFCMap<
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType>::UpdateResult
    KFCMap<MappingPointType, MapOctree<MappingPointType>, CollisionPointType>::
        updateMap<KF_COLLISION_DEFAULT_PARAMS, RayDirectionType>(
            Eigen::Vector3f,
            const pcl::PointCloud<MappingPointType>&,
            const std::vector<pcl::Axis>*,
            const pcl::Indices*);
#else
template KFCMap<
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType>::UpdateResult
    KFCMap<MappingPointType, MapOctree<MappingPointType>, CollisionPointType>::
        updateMap<KF_COLLISION_DEFAULT_PARAMS, pcl::Axis>(
            Eigen::Vector3f,
            const pcl::PointCloud<MappingPointType>&,
            const std::vector<pcl::Axis>*,
            const pcl::Indices*);
#endif
