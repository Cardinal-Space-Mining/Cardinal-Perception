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
  # add_compile_options(-ftime-report)
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

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
# find_package(GTSAM REQUIRED)


set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(include)
include_directories(SYSTEM)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TraceNotification.msg"
  "msg/TraceNotifications.msg"
  "msg/TagsTransform.msg"
  "msg/ProcessMetrics.msg"
  "msg/ThreadMetrics.msg"
  "msg/TrajectoryFilterDebug.msg"
  "srv/UpdatePathPlanningMode.srv"
  DEPENDENCIES geometry_msgs)

rosidl_get_typesupport_target(cardinal_perception_custom_types
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(nanoflann STATIC "src/nano_gicp/nanoflann.cpp")
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann)

add_library(nano_gicp STATIC "src/nano_gicp/lsq_registration.cpp" "src/nano_gicp/nano_gicp.cpp")
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
ament_target_dependencies(nano_gicp)

# add_library(perception_precomp STATIC "src/core/precomp.cpp")
# target_link_libraries(perception_precomp ${PCL_LIBRARIES} ${OpenMP_LIBS} Eigen3::Eigen "${cardinal_perception_custom_types}")
# ament_target_dependencies(perception_precomp
#   "rclcpp"
#   "sensor_msgs"
#   "geometry_msgs"
#   "tf2_ros" )

add_library(stats STATIC "src/stats/stats.cpp")

file(WRITE ${CMAKE_BINARY_DIR}/test_sensors.cpp "#include <sensors/sensors.h> \n
                                                  int main() {return 0;}")
try_compile(HAS_SENSORS ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_sensors.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_sensors.cpp)
if(${HAS_SENSORS})
  add_compile_definitions(HAS_SENSORS)
  target_link_libraries(stats sensors)
  message("HAS_SENSORS")
else()
  message("DOES NOT HAVE SENSORS")
endif()

ament_target_dependencies(stats)


add_executable(perception_node
  "src/perception_node.cpp"
  "src/profiling/profiling.cpp"
  "src/core/odometry.cpp"
  "src/core/perception_core.cpp"
  "src/core/perception_threads.cpp" )
target_link_libraries(perception_node
  ${PCL_LIBRARIES}
  ${OpenMP_LIBS}
  Threads::Threads
  Eigen3::Eigen
  opencv_core
  # opencv_imgproc
  # opencv_calib3d
  # opencv_aruco
  # gtsam
  nano_gicp
  stats
  "${cardinal_perception_custom_types}")
ament_target_dependencies(perception_node
  "rclcpp"
  "nav_msgs"
  "pcl_ros"
  "pcl_conversions"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_sensor_msgs"
  "tf2_geometry_msgs"
  # "cv_bridge"
  "image_transport")

add_executable(tag_detection_node
  "src/tag_detection_node.cpp"
  "src/core/tag_detection.cpp")
target_link_libraries(tag_detection_node
  Eigen3::Eigen
  opencv_core
  opencv_imgproc
  opencv_calib3d
  opencv_aruco
  stats
  "${cardinal_perception_custom_types}")
ament_target_dependencies(tag_detection_node
  "rclcpp"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "cv_bridge"
  "image_transport")

# add_executable(dlo_odom_node "src/dlo/odom_node.cpp" "src/dlo/odom.cpp")
# target_link_libraries(dlo_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)
# ament_target_dependencies(dlo_odom_node
#   "rclcpp"
#   "nav_msgs"
#   "pcl_ros"
#   "pcl_conversions"
#   "sensor_msgs"
#   "geometry_msgs"
#   "tf2_ros"
#   "tf2_sensor_msgs"
#   "tf2_geometry_msgs")

# add_executable (dlo_map_node "src/dlo/map_node.cpp" "src/dlo/map.cpp")
# target_link_libraries(dlo_map_node ${PCL_LIBRARIES}  ${cpp_typesupport_target})
# ament_target_dependencies(dlo_map_node
#   "rclcpp"
#   "tf2_ros"
#   "pcl_ros"
#   "pcl_conversions")


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros tf2_ros tf2_sensor_msgs tf2_geometry_msgs rosidl_default_runtime)
ament_package()


install( TARGETS perception_node tag_detection_node
         DESTINATION lib/${PROJECT_NAME}
)
# install( TARGETS dlo_odom_node dlo_map_node
#          DESTINATION lib/${PROJECT_NAME} 
# )
install( DIRECTORY config launch
         DESTINATION share/${PROJECT_NAME} 
)

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

file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
  message("CPUID support detected!")
else()
  message("No CPUID support detected.")
endif()

if(${cv_bridge_VERSION} GREATER "3.3.0")
  add_compile_definitions(USE_CV_BRIDGE_HPP)
endif()

add_compile_definitions(GEOM_UTIL_USE_OPENCV=1 GEOM_UTIL_USE_ROS=1 USE_GTSAM_PGO=0)

# if(EXISTS "src/build_number.txt")
#   file(READ "src/build_number.txt" BUILD_COUNT)
#   math(EXPR BUILD_COUNT "${BUILD_COUNT} + 1")
#   message(STATUS "[CARDINAL PERCEPTION]: Build count incremented: ${BUILD_COUNT}")
# else()
#   set(BUILD_COUNT "1")
#   message(STATUS "[CARDINAL PERCEPTION]: Build count initialized: ${BUILD_COUNT}")
# endif()
# file(WRITE "src/build_number.txt" ${BUILD_COUNT})

target_compile_options(perception_node
  PRIVATE ${OpenMP_FLAGS}
  # PRIVATE "-DBUILD_NUMBER=${BUILD_COUNT}"
)
# target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})
# target_compile_options(dlo_map_node PRIVATE ${OpenMP_FLAGS})
