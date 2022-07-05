cmake_minimum_required(VERSION 2.8)
project(vo_project)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O0 -fopenmp -pthread")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    sensor_msgs
    cv_bridge
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
)    

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Ceres
find_package(Ceres 1.14 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

include_directories( 
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_library(vo_lib SHARED
    src/frontend.cpp
    src/feature_solve.cpp
    src/visual_odometry.cpp
    src/visualization.cpp
    src/config.cpp
    src/feature.cpp
    src/frame.cpp
    src/mappoint.cpp
    src/map.cpp
    src/camera.cpp
    src/parameters.cpp
    src/Optimizer.cpp
)
target_link_libraries(vo_lib  
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${Eigen3_LIBRARIES}
    ${Sophus_LIBRARIES} 
    ${CERES_LIBRARIES}
    ${GLOG_LIBRARIES}
    pthread
)

add_executable(vo_odometry_node SubNode_sub.cpp)
target_link_libraries(vo_odometry_node vo_lib)
