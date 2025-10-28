#ifdef CORE_MODULES_PCL_PRECOMPILED
    #undef CORE_MODULES_PCL_PRECOMPILED
#endif
#include <config.hpp>

#ifdef PATH_PLANNER_PRECOMPILED
    #undef PATH_PLANNER_PRECOMPILED
#endif
#include <modules/path_planner.hpp>


using namespace csm::perception;

PATH_PLANNER_INSTANTIATE_CLASS_TEMPLATE(
    TraversibilityPointType,
    TraversibilityMetaType)

// PATH_PLANNER_INSTANTIATE_PCL_DEPENDENCIES(...)   // <-- use if template types are non-pcl (ie. core classes need to be compiled)
