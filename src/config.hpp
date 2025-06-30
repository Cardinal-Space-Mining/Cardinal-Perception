#pragma once

#include <csm_metrics/profiling_config.hpp>

#include <point_def.hpp>


// --- PROFILING CONFIGURATION -----------
#ifndef PROFILING_MODE
    #define PROFILING_MODE PROFILING_MODE_ALL
#endif
#ifndef PROFILING_DEFAULT_BUFFER_SIZE
    #define PROFILING_DEFAULT_BUFFER_SIZE 20
#endif

// --- PRINTING ENABLE/DISABLE -----------
#ifndef PERCEPTION_PRINT_STATUS_DISPLAY
    #define PERCEPTION_PRINT_STATUS_DISPLAY 1
#endif
#ifndef TRANSFORM_SYNC_PRINT_DEBUG
    #define TRANSFORM_SYNC_PRINT_DEBUG 0
#endif

// --- PUBLISHERS ENABLE/DISABLE -------
#ifndef PERCEPTION_PUBLISH_GRAV_ESTIMATION
    #define PERCEPTION_PUBLISH_GRAV_ESTIMATION 1
#endif
#ifndef PERCEPTION_PUBLISH_LIO_DEBUG
    #define PERCEPTION_PUBLISH_LIO_DEBUG 0
#endif
#ifndef PERCEPTION_PUBLISH_LFD_DEBUG
    #define PERCEPTION_PUBLISH_LFD_DEBUG 1
#endif
#ifndef PERCEPTION_PUBLISH_TRJF_DEBUG
    #define PERCEPTION_PUBLISH_TRJF_DEBUG 1
#endif
#ifndef PERCEPTION_PUBLISH_TRAV_DEBUG
    #define PERCEPTION_PUBLISH_TRAV_DEBUG 0
#endif
#ifndef PERCEPTION_PUBLISH_FULL_MAP
    #define PERCEPTION_PUBLISH_FULL_MAP 1
#endif

// --- FEATURES ENABLE/DISABLE --------
#ifndef PERCEPTION_USE_SCAN_DESKEW
    #define PERCEPTION_USE_SCAN_DESKEW 1
#endif
#ifndef PERCEPTION_USE_NULL_RAY_DELETION
    #define PERCEPTION_USE_NULL_RAY_DELETION 0
#endif

// --- PIPELINE STAGES ENABLE/DISABLE -----------
#ifndef PERCEPTION_ENABLE_MAPPING
    #define PERCEPTION_ENABLE_MAPPING 1
#endif
#ifndef PERCEPTION_ENABLE_TRAVERSIBILITY
    #define PERCEPTION_ENABLE_TRAVERSIBILITY 1
#endif
#ifndef PERCEPTION_ENABLE_PATH_PLANNING
    #define PERCEPTION_ENABLE_PATH_PLANNING 1
#endif
#ifndef PERCEPTION_USE_TAG_DETECTION_PIPELINE
    #define PERCEPTION_USE_TAG_DETECTION_PIPELINE 0
#endif
#ifndef PERCEPTION_USE_LFD_PIPELINE
    #define PERCEPTION_USE_LFD_PIPELINE 1
#endif

// --- ROS TOPIC CONFIGURATION ------------------------------------------------
#ifndef PERCEPTION_TOPIC_PREFIX
    #define PERCEPTION_TOPIC_PREFIX "/cardinal_perception"
#endif
#ifndef PERCEPTION_PUBSUB_QOS
    #define PERCEPTION_PUBSUB_QOS rclcpp::SensorDataQoS()
#endif


namespace csm
{
namespace perception
{

using OdomPointType = pcl::PointXYZ;
using MappingPointType = pcl::PointXYZ;
using FiducialPointType = csm::perception::PointXYZR;
using CollisionPointType = pcl::PointXYZLNormal;
using RayDirectionType = pcl::Axis;
using TraversibilityPointType = pcl::PointXYZ;
using TraversibilityMetaType = csm::perception::NormalTraversal;

};  // namespace perception
};  // namespace csm
