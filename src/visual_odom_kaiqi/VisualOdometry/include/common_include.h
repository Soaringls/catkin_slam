#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <stdio.h>
#include "std_msgs/String.h"
#include <sys/time.h>   //clock, gettimeofday, time
#include <vector>
#include <chrono>   //
#include <thread>   //
#include <queue>
#include <mutex>    //互斥锁
#include <memory>
#include <map>
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <glog/logging.h>


#include "visualization.h"
#include "config.h"
#include "tic_toc.h"



// typedefs for eigen
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 4, 3> Mat43;
typedef Eigen::Matrix<double, 3, 3> Mat33;
typedef Eigen::Matrix<double, 4, 4> Mat44;
typedef Eigen::Matrix<double, 3, 4> Mat34;

// double vectors
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 5, 1> Vec5;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;

typedef Sophus::SE3d SE3;
typedef Sophus::SO3d SO3;

















