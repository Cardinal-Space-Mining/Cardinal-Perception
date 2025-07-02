#include "../config.hpp"

#ifdef PATH_PLANNER_PRECOMPILED
    #undef PATH_PLANNER_PRECOMPILED
#endif

#include "path_planner.hpp"


template class csm::perception::PathPlanner<
    float,
    csm::perception::TraversibilityPointType,
    csm::perception::TraversibilityMetaType>;
