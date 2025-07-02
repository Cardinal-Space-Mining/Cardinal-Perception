#include "../config.hpp"

#ifdef LFD_PRECOMPILED
    #undef LFD_PRECOMPILED
#endif

#include "lf_detector.hpp"


template class csm::perception::LidarFiducialDetector<
    csm::perception::FiducialPointType>;
