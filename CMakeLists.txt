cmake_minimum_required(VERSION 3.15)
project(cardinal_perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT MSVC)
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)
include_directories(SYSTEM)


rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/TagsTransform.msg"
    "msg/ThreadMetrics.msg"
    "msg/TrajectoryFilterDebug.msg"
    "srv/UpdatePathPlanningMode.srv"
    DEPENDENCIES geometry_msgs)

rosidl_get_typesupport_target(cardinal_perception_custom_types
    ${PROJECT_NAME} rosidl_typesupport_cpp)

  
add_library(nanoflann STATIC
    "src/nano_gicp/nanoflann.cpp" )
target_link_libraries(nanoflann
    ${PCL_LIBRARIES} )
ament_target_dependencies(nanoflann
    csm_metrics )

add_library(nano_gicp STATIC
    "src/nano_gicp/lsq_registration.cpp"
    "src/nano_gicp/nano_gicp.cpp" )
target_link_libraries(nano_gicp
    nanoflann
    ${OpenMP_LIBS}
    ${PCL_LIBRARIES} )
ament_target_dependencies(nano_gicp
    csm_metrics )

# add_library(perception_precomp STATIC "src/core/precomp.cpp")
# target_link_libraries(perception_precomp ${PCL_LIBRARIES} ${OpenMP_LIBS} Eigen3::Eigen "${cardinal_perception_custom_types}")
# ament_target_dependencies(perception_precomp
#   "rclcpp"
#   "sensor_msgs"
#   "geometry_msgs"
#   "tf2_ros" )


add_executable(perception_node
    "src/perception_node.cpp"
    "src/core/odometry.cpp"
    "src/core/perception_core.cpp"
    "src/core/perception_threads.cpp" )
target_link_libraries(perception_node
    nano_gicp
    csm_metrics::stats
    csm_metrics::profiling
    ${cardinal_perception_custom_types}
    Threads::Threads
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenMP_LIBS} )
ament_target_dependencies(perception_node
    "rclcpp"
    "pcl_ros"
    "pcl_conversions"
    "sensor_msgs"
    "geometry_msgs"
    "nav_msgs"
    "tf2_ros"
    "tf2_sensor_msgs"
    "tf2_geometry_msgs"
    "csm_metrics" )

add_executable(tag_detection_node
    "src/tag_detection_node.cpp"
    "src/core/tag_detection.cpp" )
target_link_libraries(tag_detection_node
    csm_metrics::stats
    ${cardinal_perception_custom_types}
    Eigen3::Eigen
    opencv_core
    opencv_imgproc
    opencv_calib3d
    opencv_aruco )
ament_target_dependencies(tag_detection_node
    "rclcpp"
    "sensor_msgs"
    "geometry_msgs"
    "tf2_ros"
    "tf2_geometry_msgs"
    "cv_bridge"
    "image_transport"
    "csm_metrics" )


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(
    rclcpp
    std_msgs
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


install(
    TARGETS perception_node tag_detection_node
    DESTINATION lib/${PROJECT_NAME} )
install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME} )

# add_compile_options(-std=c++17)
link_directories(${PCL_LIBRARY_DIRS})

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

target_compile_definitions(perception_node
    PRIVATE GEOM_UTIL_USE_OPENCV=0
    PRIVATE GEOM_UTIL_USE_ROS=1
    PRIVATE USE_GTSAM_PGO=0 )
target_compile_definitions(tag_detection_node
    PRIVATE GEOM_UTIL_USE_OPENCV=1
    PRIVATE GEOM_UTIL_USE_ROS=1
    PRIVATE USE_GTSAM_PGO=0 )

target_compile_options(perception_node
    PRIVATE ${OpenMP_FLAGS} )
