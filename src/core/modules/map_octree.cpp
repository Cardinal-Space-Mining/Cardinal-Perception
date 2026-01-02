#ifdef CORE_MODULES_PCL_PRECOMPILED
    #undef CORE_MODULES_PCL_PRECOMPILED
#endif
#include <config.hpp>

#ifdef MAP_OCTREE_PRECOMPILED
    #undef MAP_OCTREE_PRECOMPILED
#endif
#include <modules/map_octree.hpp>


using namespace csm::perception;

MAP_OCTREE_INSTANTIATE_CLASS_TEMPLATE(MappingPointType, MAP_OCTREE_STORE_NORMALS)
MAP_OCTREE_INSTANTIATE_PCL_DEPENDENCIES(MappingPointType)
