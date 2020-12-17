#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

void publishPath(ros::Publisher pub, nav_msgs::Path& path,
                 nav_msgs::Odometry pose) {
  //   Eigen::Translation3d t(pose.translation());
  //   Eigen::Quaterniond q(pose.linear());

  // ref mapOptmization.cpp search "save updated transform"
  nav_msgs::Odometry laserOdometry = pose;
  laserOdometry.pose.pose.position.y = pose.pose.pose.position.x;
  laserOdometry.pose.pose.position.z = pose.pose.pose.position.y;
  laserOdometry.pose.pose.position.x = pose.pose.pose.position.z;

  laserOdometry.header.frame_id = "/map";
  laserOdometry.child_frame_id = "/laser_odom";
  laserOdometry.header.stamp = ros::Time().now();
  //   laserOdometry.pose.pose.orientation.x = q.x();
  //   laserOdometry.pose.pose.orientation.y = q.y();
  //   laserOdometry.pose.pose.orientation.z = q.z();
  //   laserOdometry.pose.pose.orientation.w = q.w();
  //   laserOdometry.pose.pose.position.x = t.x();
  //   laserOdometry.pose.pose.position.y = t.y();
  //   laserOdometry.pose.pose.position.z = t.z();

  geometry_msgs::PoseStamped laserPose;
  laserPose.header = laserOdometry.header;
  laserPose.pose = laserOdometry.pose.pose;
  path.header.stamp = laserOdometry.header.stamp;
  path.poses.push_back(laserPose);
  path.header.frame_id = "/map";
  pub.publish(path);
}