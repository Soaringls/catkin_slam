#pragma once

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>


#include "common_include.h"


using namespace std;



void registerPub(ros::NodeHandle &n);

void pubTrackImage(std_msgs::Header &header, const cv::Mat &image);
void pubBAImage(std_msgs::Header &header, const cv::Mat &image);
void pubCameraPose(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs);
void pubKeyframePose(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs);
void pubPointCloud( std_msgs::Header header, const vector<Eigen::Vector3d> point3d);
void pubMapPoint( std_msgs::Header header, const vector<Eigen::Vector3d> point3d);
void pubPath(const std_msgs::Header header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs);
void pubKeyframePose(const std_msgs::Header header, const Eigen::Vector3d Ps[], const Eigen::Matrix3d Rs[]);
void pubTF(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs);
void pubOdometry(const std_msgs::Header &header, const Eigen::Vector3d Ps, const Eigen::Matrix3d Rs);