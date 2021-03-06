cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)

link_directories(${Boost_LIBRARY_DIRS})

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIR}
)

set(THIRD_LIBS 
    yaml-cpp glog
    ${PCL_LIBRARIES} 
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    ${Boost_LIBRARIES}
)
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
) 

add_library(ceres_loam2 
            src/lib2/cloud_segmentation.cc
            src/lib2/config.cc
            src/lib2/lidar_odometry.cc
            src/lib2/lidar_refination.cc
)
target_link_libraries(ceres_loam2  ${THIRD_LIBS})
add_executable(testlib2 src/test_lib2.cc)
target_link_libraries(testlib2
       ${catkin_LIBRARIES} ceres_loam2
)

# add_executable(aimageProjection src/imageProjection.cpp)
# target_link_libraries(aimageProjection glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

# add_executable(ascanRegistration src/scanRegistration.cpp)
# target_link_libraries(ascanRegistration  glog ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# add_executable(alaserOdometry src/laserOdometry.cpp)
# target_link_libraries(alaserOdometry  glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# add_executable(alaserMapping src/laserMapping.cpp)
# target_link_libraries(alaserMapping  glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

