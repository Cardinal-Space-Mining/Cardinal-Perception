#ifdef CORE_MODULES_PCL_PRECOMPILED
    #undef CORE_MODULES_PCL_PRECOMPILED
#endif
#include <config.hpp>

#ifdef LIDAR_ODOM_PRECOMPILED
    #undef LIDAR_ODOM_PRECOMPILED
#endif
#include <modules/lidar_odom.hpp>


using namespace csm::perception;

LIDAR_ODOM_INSTANTIATE_CLASS_TEMPLATE(OdomPointType)
LIDAR_ODOM_INSTANTIATE_NANOGICP_DEPENDENCIES(OdomPointType)
