# Config.cmake
# Central place to declare all perception-related CMake options

# --- PROFILING CONFIGURATION -----------
set(PROFILING_MODE "PROFILING_MODE_LIMITED" CACHE STRING "Profiling mode")          # full profiling use 'PROFILING_MODE_ALL'
set(PROFILING_DEFAULT_BUFFER_SIZE 1 CACHE STRING "Profiling default buffer size")   # full profiling use 20

# --- PRINTING ENABLE/DISABLE -----------
option(PERCEPTION_PRINT_STARTUP_CONFIGS "Print startup configs" ON)
option(LFD_PRINT_DEBUG "Print LFD debug logs" OFF)
option(PPLAN_PRINT_DEBUG "Print Path Planning debug logs" OFF)
option(TRANSFORM_SYNC_PRINT_DEBUG "Enable transform sync debug logs" OFF)
option(TRAJECTORY_FILTER_PRINT_DEBUG "Enable trajectory filter debug logs" OFF)

# --- PUBLISHERS ENABLE/DISABLE ---------
option(PERCEPTION_PUBLISH_GRAV_ESTIMATION "Publish gravity estimation" ON)
option(PERCEPTION_PUBLISH_LIO_DEBUG "Publish LIO debug" OFF)
option(PERCEPTION_PUBLISH_LFD_DEBUG "Publish LFD debug" ON)
option(PERCEPTION_PUBLISH_TRJF_DEBUG "Publish TRJF debug" ON)
option(PERCEPTION_PUBLISH_TRAV_DEBUG "Publish TRAV debug" OFF)
option(PERCEPTION_PUBLISH_FULL_MAP "Publish full map" ON)

# --- FEATURES ENABLE/DISABLE -----------
option(PERCEPTION_USE_SCAN_DESKEW "Use scan deskew" OFF)
option(PERCEPTION_USE_NULL_RAY_DELETION "Use null ray deletion" OFF)

# --- PIPELINE STAGES ENABLE/DISABLE ----
option(PERCEPTION_USE_TAG_DETECTION_PIPELINE "Use tag detection pipeline" OFF)
option(PERCEPTION_USE_LFD_PIPELINE "Use LFD pipeline" ON)

# --- ROS TOPIC CONFIGURATION -----------
set(PERCEPTION_TOPIC_PREFIX "/cardinal_perception" CACHE STRING "ROS topic prefix")
set(PERCEPTION_PUBSUB_QOS "rclcpp::SensorDataQoS()" CACHE STRING "PubSub QoS config")

# Boolean options
foreach(opt
    PERCEPTION_PRINT_STARTUP_CONFIGS
    LFD_PRINT_DEBUG
    PPLAN_PRINT_DEBUG
    TRANSFORM_SYNC_PRINT_DEBUG
    TRAJECTORY_FILTER_PRINT_DEBUG
    PERCEPTION_PUBLISH_GRAV_ESTIMATION
    PERCEPTION_PUBLISH_LIO_DEBUG
    PERCEPTION_PUBLISH_LFD_DEBUG
    PERCEPTION_PUBLISH_TRJF_DEBUG
    PERCEPTION_PUBLISH_TRAV_DEBUG
    PERCEPTION_PUBLISH_FULL_MAP
    PERCEPTION_USE_SCAN_DESKEW
    PERCEPTION_USE_NULL_RAY_DELETION
    PERCEPTION_USE_TAG_DETECTION_PIPELINE
    PERCEPTION_USE_LFD_PIPELINE
)
    if(${opt})
        set(${opt}_VALUE 1)
    else()
        set(${opt}_VALUE 0)
    endif()
endforeach()
