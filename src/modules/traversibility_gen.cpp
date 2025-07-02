#include "../config.hpp"

#ifdef TRAVERSIBILITY_GEN_PRECOMPILED
    #undef TRAVERSIBILITY_GEN_PRECOMPILED
#endif

#include "traversibility_gen.hpp"


template class csm::perception::TraversibilityGenerator<
    csm::perception::TraversibilityPointType,
    csm::perception::TraversibilityMetaType>;
