#ifdef CORE_MODULES_PCL_PRECOMPILED
    #undef CORE_MODULES_PCL_PRECOMPILED
#endif
#include "../../config.hpp"

#ifdef TRAVERSIBILITY_GEN_PRECOMPILED
    #undef TRAVERSIBILITY_GEN_PRECOMPILED
#endif
#include "traversibility_gen.hpp"


using namespace csm::perception;

TRAVERSIBILITY_GEN_INSTANTIATE_CLASS_TEMPLATE(
    TraversibilityPointType,
    TraversibilityMetaType)

TRAVERSIBILITY_GEN_INSTANTIATE_PCL_DEPENDENCIES(
    TraversibilityPointType,
    TraversibilityMetaType)
