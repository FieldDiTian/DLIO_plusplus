###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.12.4)
project(gicp_localization)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package( Eigen3 REQUIRED )
find_package(Threads REQUIRED)
find_package(LibXml2 REQUIRED)  # URDF parsing for offline aux-LiDAR extrinsics (lidar_concat)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
include_directories(SYSTEM)

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann)

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
ament_target_dependencies(nano_gicp)

# Localization Node
add_executable(gicp_localization_node src/localization_node.cc src/localization.cc)
target_include_directories(gicp_localization_node PRIVATE ${LIBXML2_INCLUDE_DIR})
target_link_libraries(gicp_localization_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp ${LIBXML2_LIBRARIES})
ament_target_dependencies(gicp_localization_node "tf2_ros" "tf2_geometry_msgs" "tf2_eigen" "tf2_sensor_msgs" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "sensor_msgs" "geometry_msgs" "visualization_msgs")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_ros)
ament_package()

# Binaries
install( TARGETS gicp_localization_node
         DESTINATION lib/${PROJECT_NAME}
)
install( PROGRAMS
         scripts/profile_localization_resources.py
         scripts/plot_localization_profile.py
         scripts/generate_lap_trajectory_plots.py
         DESTINATION lib/${PROJECT_NAME}
)
install( DIRECTORY cfg launch
         DESTINATION share/${PROJECT_NAME}
)
# Install the vehicle URDF alongside the package share so the launch's walk-up
# (share/gicp_localization/launch -> share/gicp_localization) resolves av24.urdf
# in the colcon install tree, not just the source tree. OPTIONAL: the source-tree
# walk-up / static fallback still apply if the file isn't present at install time.
install( FILES av24.urdf
         DESTINATION share/${PROJECT_NAME}
         OPTIONAL
)

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
endif()

target_compile_options(gicp_localization_node PRIVATE ${OpenMP_FLAGS})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_generate_lap_trajectory_plots
    test/test_generate_lap_trajectory_plots.py
  )
endif()
