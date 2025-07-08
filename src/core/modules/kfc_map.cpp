#ifdef CORE_MODULES_PCL_PRECOMPILED
    #undef CORE_MODULES_PCL_PRECOMPILED
#endif
#include "../../config.hpp"

#ifdef KFC_MAP_PRECOMPILED
    #undef KFC_MAP_PRECOMPILED
#endif
#include "kfc_map.hpp"


using namespace csm::perception;

KFC_MAP_INSTANTIATE_CLASS_TEMPLATE(
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType)

// KFC_MAP_INSTANTIATE_PCL_DEPENDENCIES(...)    // <-- use if template types are non-pcl (ie. core classes need to be compiled)

#if PERCEPTION_USE_NULL_RAY_DELETION
KFC_MAP_INSTANTIATE_UPDATE_FUNC_TEMPLATE(
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType,
    KF_COLLISION_DEFAULT_PARAMS,
    RayDirectionType)
#else
KFC_MAP_INSTANTIATE_UPDATE_FUNC_TEMPLATE(
    MappingPointType,
    MapOctree<MappingPointType>,
    CollisionPointType,
    KF_COLLISION_DEFAULT_PARAMS,
    pcl::Axis)
#endif
