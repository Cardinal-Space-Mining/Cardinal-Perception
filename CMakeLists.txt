cmake_minimum_required(VERSION 3.14)
project(cardinal_perception VERSION 0.7.2)

# --- Dependencies -------------------------------------------------------------
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
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

# Default to C++20
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

# --- Patch for Lyrical --------------------------------------------------------
#
# Add the interface targets or definitions, include directories and libraries
# of packages to a target.
#
# Each package name must have been find_package()-ed before.
# Additionally the exported variables must have a prefix with the same case
# and the suffixes must be either _INTERFACES or _DEFINITIONS, _INCLUDE_DIRS,
# _LIBRARIES, _LIBRARY_DIRS, and _LINK_FLAGS.
# If _INTERFACES is not empty it will be used exclusively, otherwise the other
# variables are being used.
# If _LIBRARY_DIRS is not empty, _LIBRARIES which are not absolute paths already
# will be searched in those directories and their absolute paths will be used instead.
#
# :param target: the target name
# :type target: string
# :param ARGN: a list of package names, which can optionally start
#   with a SYSTEM keyword, followed by an INTERFACE or PUBLIC keyword.
#   If it starts with a SYSTEM keyword, it will be used in
#   target_include_directories() calls.
#   If it starts (or follows) with an INTERFACE or PUBLIC keyword,
#   this keyword will be used in the target_*() calls.
# :type ARGN: list of strings
#
# @public
#
function(auto_target_dependencies target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "auto_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    cmake_parse_arguments(ARG "INTERFACE;PUBLIC;SYSTEM" "" "" ${ARGN})
    set(ARGVIND 1)
    set(system_keyword "")
    set(optional_keyword "")
    set(required_keyword "PUBLIC")
    if(ARG_SYSTEM)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "SYSTEM")
        message(FATAL_ERROR "auto_target_dependencies() SYSTEM keyword is only allowed before the package names and other keywords")
      endif()
      set(system_keyword SYSTEM)
      math(EXPR ARGVIND "${ARGVIND} + 1")
    endif()
    if(ARG_INTERFACE)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "INTERFACE")
        message(FATAL_ERROR "auto_target_dependencies() INTERFACE keyword is only allowed before the package names")
      endif()
      set(optional_keyword INTERFACE)
      set(required_keyword INTERFACE)
    endif()
    if(ARG_PUBLIC)
      if(NOT "${ARGV${ARGVIND}}" STREQUAL "PUBLIC")
        message(FATAL_ERROR "auto_target_dependencies() PUBLIC keyword is only allowed before the package names")
      endif()
      set(optional_keyword PUBLIC)
    endif()
    set(definitions "")
    set(include_dirs "")
    set(interfaces "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARG_UNPARSED_ARGUMENTS})
      if(NOT "${${package_name}_FOUND}")
        message(FATAL_ERROR "auto_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      # if a package provides modern CMake interface targets use them
      # exclusively assuming the classic CMake variables only exist for
      # backward compatibility
      set(use_modern_cmake FALSE)
      if(NOT "${${package_name}_TARGETS}" STREQUAL "")
        foreach(_target ${${package_name}_TARGETS})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_target}")
            list_append_unique(interfaces ${_target})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
      endif()
      if(NOT use_modern_cmake AND NOT "${${package_name}_INTERFACES}" STREQUAL "")
        foreach(_interface ${${package_name}_INTERFACES})
          # only use actual targets
          # in case a package uses this variable for other content
          if(TARGET "${_interface}")
            list_append_unique(interfaces ${_interface})
            set(use_modern_cmake TRUE)
          endif()
        endforeach()
        if(use_modern_cmake)
          message(DEPRECATION
            "Package ${package_name} is exporting the variable "
            "${package_name}_INTERFACES which is deprecated, it should export
            ${package_name}_TARGETS instead")
        endif()
      endif()
      if(NOT use_modern_cmake)
        # otherwise use the classic CMake variables
        list_append_unique(definitions ${${package_name}_DEFINITIONS})
        list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
        foreach(library ${${package_name}_LIBRARIES})
          if(NOT "${${package_name}_LIBRARY_DIRS}" STREQUAL "")
            if(NOT IS_ABSOLUTE ${library} OR NOT EXISTS ${library})
              unset(lib CACHE)
              find_library(lib NAMES ${library} PATHS ${${package_name}_LIBRARY_DIRS} NO_DEFAULT_PATH)
              if(lib)
                set(library ${lib})
              endif()
            endif()
          endif()
          list(APPEND libraries ${library})
        endforeach()
        list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
      endif()
    endforeach()
    if(NOT ARG_INTERFACE)
      target_compile_definitions(${target}
        ${required_keyword} ${definitions})
      # the interface include dirs must be ordered
      set(interface_include_dirs)
      foreach(interface ${interfaces})
        get_target_property(_include_dirs ${interface} INTERFACE_INCLUDE_DIRECTORIES)
        if(_include_dirs)
          list_append_unique(interface_include_dirs ${_include_dirs})
        endif()
      endforeach()
      ament_include_directories_order(ordered_interface_include_dirs ${interface_include_dirs})
      # the interface include dirs are used privately to ensure proper order
      # and the interfaces cover the public case
      target_include_directories(${target} ${system_keyword}
        PRIVATE ${ordered_interface_include_dirs})
    endif()
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_link_libraries(${target}
      ${optional_keyword} ${interfaces})
    target_include_directories(${target} ${system_keyword}
      ${required_keyword} ${ordered_include_dirs})
    if(NOT ARG_INTERFACE)
      ament_libraries_deduplicate(unique_libraries ${libraries})
      target_link_libraries(${target}
        ${optional_keyword} ${unique_libraries})
      foreach(link_flag IN LISTS link_flags)
        set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
      endforeach()
    endif()
  endif()
endfunction()

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
auto_target_dependencies(core_modules
    std_msgs
    sensor_msgs
    pcl_conversions
    tf2_ros )

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
auto_target_dependencies(perception_node
    rclcpp
    pcl_conversions
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs )

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
auto_target_dependencies(tag_detection_node
    rclcpp
    sensor_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport )

# --- Path Planning Client Node ------------------------------------------------
add_executable(pplan_client_node src/pplan_client_node.cpp)
target_link_libraries(pplan_client_node
    ${cardinal_perception_custom_types})
auto_target_dependencies(pplan_client_node
    rclcpp
    geometry_msgs
    tf2_ros )


# --- Export -------------------------------------------------------------------
ament_export_dependencies(
    rclcpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
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
