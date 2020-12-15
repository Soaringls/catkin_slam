#pragma once
#include "data_center.h"
#include "timer.h"
#include "utility.h"

namespace ceres_loam {
#define DISTORTION 0
constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

class LidarOdometry {
 public:
  std::shared_ptr<LidarOdometry> LidarOdometryPtr;
  std::shared_ptr<const LidarOdometry> LidarOdometryConstPtr;
  LidarOdometry();
  ~LidarOdometry() = default;

 private:
  void allocateMemory();
  void accessData();

  void transformToStart(PointType const *const pi, PointType *const po);
  void calculateTransformation();

 public:
  void Process();

  const Eigen::Quaterniond GetQuaternion() { return q_w_curr; }
  const Eigen::Vector3d GetPos() { return t_w_curr; }
  const Eigen::Affine3d GetRefinedPose() {
    auto odom_correct_msg = DataCenter::Instance()->GetOdomCorrectMsg();
    q_w_curr = odom_correct_msg->correct_q * q_w_curr;
    t_w_curr =
        odom_correct_msg->correct_q * t_w_curr + odom_correct_msg->correct_t;

    Eigen::Affine3d result =
        Eigen::Translation3d(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()) *
        q_w_curr.normalized();
    return result;
  }

 private:
  bool init_;
  double para_q[4];                // = {0.0, 0.0, 0.0, 1.0};
  double para_t[3];                // = {0.0, 0.0, 0.0};
  Eigen::Quaterniond q_last_curr;  //(para_q);
  Eigen::Vector3d t_last_curr;     //(para_t);

  // transformation from current frame to world frame
  Eigen::Quaterniond q_w_curr;
  Eigen::Vector3d t_w_curr;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast;
  PointCloudPtr laserCloudFullRes;
  PointCloudPtr cornerPointsSharp;
  PointCloudPtr cornerPointsLessSharp;
  PointCloudPtr surfPointsFlat;
  PointCloudPtr surfPointsLessFlat;

  PointCloudPtr laserCloudCornerLast;
  PointCloudPtr laserCloudSurfLast;
};
}  // namespace ceres_loam