cmake_minimum_required(VERSION 3.16)
project(glim_ros VERSION 1.2.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(BUILD_WITH_CUDA "Build with GPU support" ON)
option(BUILD_WITH_VIEWER "Build with visualizer" ON)
option(BUILD_WITH_CV_BRIDGE "Build with cv_bridge which enables support for processing of camera images" ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(glim REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBXML2 REQUIRED libxml-2.0)
find_library(PCAP_LIBRARY pcap REQUIRED)

if(DEFINED GLIM_USE_OPENCV AND NOT GLIM_USE_OPENCV AND BUILD_WITH_CV_BRIDGE)
  message(WARNING "BUILD_WITH_CV_BRIDGE is disabled because glim is not built with OpenCV.")
  set(BUILD_WITH_CV_BRIDGE OFF CACHE BOOL "glim is not built with OpenCV" FORCE)
endif()

if(BUILD_WITH_CV_BRIDGE)
  add_definitions(-DBUILD_WITH_CV_BRIDGE)
endif()

if(BUILD_WITH_CUDA)
  add_definitions(-DBUILD_GTSAM_POINTS_GPU)
endif()

set(OLD_DISTRO "humble;galactic;foxy")
if($ENV{ROS_DISTRO} IN_LIST OLD_DISTRO)
  add_definitions(-DCV_BRIDGE_INCLUDE_H)
else()
  add_definitions(-DCV_BRIDGE_INCLUDE_HPP)
endif()

### glim_ros ###
ament_auto_add_library(glim_ros SHARED
  src/glim_ros/glim_ros.cpp
  src/glim_ros/ros_qos.cpp
)
target_include_directories(glim_ros PUBLIC
  include
  ${LIBXML2_INCLUDE_DIRS}
)
target_link_libraries(glim_ros
  glim::glim
  ${LIBXML2_LIBRARIES}
)
rclcpp_components_register_nodes(glim_ros "glim::GlimROS")

ament_auto_add_library(rviz_viewer SHARED
  src/glim_ros/rviz_viewer.cpp
)

### glim_rosnode ###
ament_auto_add_executable(glim_rosnode
  src/glim_rosnode.cpp
)
target_link_libraries(glim_rosnode
  glim_ros
)

### glim_rosbag ###
ament_auto_add_executable(glim_rosbag
  src/glim_rosbag.cpp
)
target_include_directories(glim_rosbag PRIVATE ${LIBXML2_INCLUDE_DIRS})
target_link_libraries(glim_rosbag
  glim_ros
  ${LIBXML2_LIBRARIES}
)

### glim_pcap_rosbag ###
ament_auto_add_executable(glim_pcap_rosbag
  src/glim_pcap_rosbag.cpp
  src/iris_pcap_reader.cpp
)
target_include_directories(glim_pcap_rosbag PRIVATE ${LIBXML2_INCLUDE_DIRS})
target_link_libraries(glim_pcap_rosbag
  glim_ros
  ${LIBXML2_LIBRARIES}
  ${PCAP_LIBRARY}
)

### validator_node ###
ament_auto_add_executable(validator_node
  src/validator_node.cpp
)
target_link_libraries(validator_node
  glim_ros
)

if(BUILD_WITH_VIEWER)
  ### offline_viewer ###
  ament_auto_add_executable(offline_viewer
    src/offline_viewer.cpp
  )
  target_include_directories(offline_viewer PUBLIC
    include
  )
  target_link_libraries(offline_viewer
    glim::interactive_viewer
    Boost::program_options
  )

  ### map_editor ###
  ament_auto_add_executable(map_editor
    src/map_editor.cpp
  )
  target_link_libraries(map_editor
    glim::map_editor
    Boost::program_options
  )
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lidar_concat_point_time_test
    test/lidar_concat_point_time_test.cpp
  )
  target_include_directories(lidar_concat_point_time_test PRIVATE
    include
    ${LIBXML2_INCLUDE_DIRS}
  )
  target_link_libraries(lidar_concat_point_time_test
    glim_ros
    ${LIBXML2_LIBRARIES}
  )
endif()

ament_auto_package()
