cmake_minimum_required(VERSION 3.14)
project(cardinal_perception VERSION 0.7.0)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(image_transport REQUIRED)

find_package(csm_metrics REQUIRED)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# --- Presets ------------------------------------------------------------------
include(${CMAKE_SOURCE_DIR}/cmake/Config.cmake)
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/config.hpp.in
    ${CMAKE_BINARY_DIR}/generated/config.hpp )

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# --- Misc. Options ------------------------------------------------------------
include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

if(${cv_bridge_VERSION} GREATER "3.3.0")
    add_compile_definitions(USE_CV_BRIDGE_HPP)
endif()

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(SYSTEM)

link_directories(${PCL_LIBRARY_DIRS})


# --- Custom Types -------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MiningEvalResults.msg"
    "msg/ReflectorHint.msg"
    "msg/TagsTransform.msg"
    "msg/TrajectoryFilterDebug.msg"
    "srv/UpdateMiningEvalMode.srv"
    "srv/UpdatePathPlanningMode.srv"
    DEPENDENCIES geometry_msgs)

rosidl_get_typesupport_target(cardinal_perception_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

# --- Core Modules Static Lib --------------------------------------------------
add_library(core_modules STATIC
    "src/core/modules/kfc_map.cpp"
    "src/core/modules/lf_detector.cpp"
    "src/core/modules/lidar_odom.cpp"
    "src/core/modules/map_octree.cpp"
    "src/core/modules/path_planner.cpp"
    "src/core/modules/traversibility_gen.cpp" )
target_include_directories(core_modules PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated )
target_link_libraries(core_modules
    csm_metrics::profiling
    Eigen3::Eigen
    ${PCL_LIBRARIES} )
target_compile_definitions(core_modules
    PUBLIC KFC_MAP_PRECOMPILED
    PUBLIC LFD_PRECOMPILED
    PUBLIC LIDAR_ODOM_PRECOMPILED
    PUBLIC MAP_OCTREE_PRECOMPILED
    PUBLIC PATH_PLANNER_PRECOMPILED
    PUBLIC TRAVERSIBILITY_GEN_PRECOMPILED
    PUBLIC CORE_MODULES_PCL_PRECOMPILED )
ament_target_dependencies(core_modules
    "std_msgs"
    "sensor_msgs"
    "pcl_conversions"
    "tf2_ros"
    "csm_metrics" )

# --- Perception Node ----------------------------------------------------------
add_executable(perception_node
    "src/perception_node.cpp"
    "src/core/perception_core.cpp"
    "src/core/threads/imu_worker.cpp"
    "src/core/threads/localization_worker.cpp"
    "src/core/threads/mapping_worker.cpp"
    "src/core/threads/mining_eval_worker.cpp"
    "src/core/threads/path_planning_worker.cpp"
    "src/core/threads/traversibility_worker.cpp" )
target_include_directories(perception_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated )
target_link_libraries(perception_node
    core_modules
    csm_metrics::stats
    csm_metrics::profiling
    ${cardinal_perception_custom_types}
    Threads::Threads
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenMP_LIBS} )
target_compile_definitions(perception_node
    PRIVATE PERCEPTION_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    PRIVATE PERCEPTION_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    PRIVATE PERCEPTION_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    PRIVATE GEOM_UTIL_USE_OPENCV=0
    PRIVATE GEOM_UTIL_USE_ROS=1
    PRIVATE USE_GTSAM_PGO=0 )
target_compile_options(perception_node
    PRIVATE ${OpenMP_FLAGS} )
ament_target_dependencies(perception_node
    "rclcpp"
    "pcl_ros"
    "pcl_conversions"
    "std_srvs"
    "sensor_msgs"
    "geometry_msgs"
    "nav_msgs"
    "tf2_ros"
    "tf2_sensor_msgs"
    "tf2_geometry_msgs"
    "csm_metrics" )

# --- Tag Detection Node -------------------------------------------------------
add_executable(tag_detection_node
    "src/tag_detection_node.cpp"
    "src/core/tag_detection.cpp" )
target_include_directories(tag_detection_node PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_BINARY_DIR}/generated )
target_link_libraries(tag_detection_node
    csm_metrics::stats
    ${cardinal_perception_custom_types}
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_calib3d
    opencv_aruco )
target_compile_definitions(tag_detection_node
    PRIVATE GEOM_UTIL_USE_OPENCV=1
    PRIVATE GEOM_UTIL_USE_ROS=1
    PRIVATE USE_GTSAM_PGO=0 )
ament_target_dependencies(tag_detection_node
    "rclcpp"
    "sensor_msgs"
    "geometry_msgs"
    "tf2_ros"
    "tf2_geometry_msgs"
    "cv_bridge"
    "image_transport"
    "csm_metrics" )

# --- Path Planning Client Node ------------------------------------------------
add_executable(pplan_client_node src/pplan_client_node.cpp)
target_link_libraries(pplan_client_node
    ${cardinal_perception_custom_types})
ament_target_dependencies(pplan_client_node
    "rclcpp"
    "geometry_msgs"
    "tf2_ros")


# --- Export -------------------------------------------------------------------
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(
    rclcpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
    rosidl_default_runtime
    csm_metrics )
ament_package()

install(TARGETS
    perception_node
    tag_detection_node
    pplan_client_node
    DESTINATION lib/${PROJECT_NAME} )
install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME} )
