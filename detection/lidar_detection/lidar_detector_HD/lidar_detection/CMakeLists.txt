cmake_minimum_required(VERSION 2.8.3)
project(lidar_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  tf
  # vector_map_server
  # grid_map_ros
  # grid_map_cv
  # grid_map_msgs
  autoware_msgs
)
find_package(OpenMP)
find_package(OpenCV REQUIRED)

# set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS
        pcl_ros
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        autoware_msgs
        tf
        # vector_map_server
        # grid_map_ros
        # grid_map_cv
        # grid_map_msgs
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# add_executable(lidar_detection src/lidar_detection_node.cpp include/lidar_detection_core.h include/cluster.h include/gpu_euclidean_clustering.h)
add_executable(lidar_detection src/lidar_detection_node.cpp 
        src/lidar_detection_core.cpp 
        src/cluster_hd.cpp
        src/gpu_euclidean_clustering.cu)

find_package(CUDA)
if (${CUDA_FOUND})
    INCLUDE(FindCUDA)
    message("-- USING ACCELERATED CLUSTERING --")
    message("Version: " ${CUDA_VERSION})
    message("Library: " ${CUDA_CUDA_LIBRARY})
    message("Runtime: " ${CUDA_CUDART_LIBRARY})
    set(ROS_VERSION $ENV{ROS_DISTRO})
    target_compile_definitions(lidar_detection PRIVATE
            GPU_CLUSTERING=1
            )

    cuda_add_library(gpu_euclidean_clustering
            include/gpu_euclidean_clustering.h
            src/gpu_euclidean_clustering.cu
            )

    target_link_libraries(lidar_detection
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${YAML_CPP_LIBRARIES}
            gpu_euclidean_clustering)

else ()
    target_link_libraries(lidar_detection
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            # ${YAML_CPP_LIBRARIES}
    )

endif ()

add_dependencies(lidar_detection
        ${catkin_EXPORTED_TARGETS}
        )

if (OPENMP_FOUND)
    set_target_properties(lidar_detection PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

# install(TARGETS
#         lidar_euclidean_cluster_detect
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
