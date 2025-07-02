#include "../config.hpp"

#ifdef MAP_OCTREE_PRECOMPILED
    #undef MAP_OCTREE_PRECOMPILED
#endif

#include "map_octree.hpp"


template class csm::perception::MapOctree<csm::perception::MappingPointType>;
