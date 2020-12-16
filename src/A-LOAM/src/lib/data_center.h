#pragma once
#include <deque>

#include "lidar_factor.hpp"
#include "utility.h"

namespace ceres_loam {
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

struct ScanSegMsg {
  double timestamp = 0;
  PointCloudPtr laserCloud;             // odom mapping
  PointCloudPtr cornerPointsSharp;      // odom
  PointCloudPtr cornerPointsLessSharp;  // odom mapping
  PointCloudPtr surfPointsFlat;         // odom
  PointCloudPtr surfPointsLessFlat;     // odom mapping
  Eigen::Vector3d odom_to_init_t;       // odom mapping
  Eigen::Quaterniond odom_to_init_q;    // odom mapping

  ScanSegMsg() {
    laserCloud.reset(new PointCloud);
    cornerPointsSharp.reset(new PointCloud);
    cornerPointsLessSharp.reset(new PointCloud);
    surfPointsFlat.reset(new PointCloud);
    surfPointsLessFlat.reset(new PointCloud);
  };

  ScanSegMsg(const double time, const PointCloudPtr cloud1,
             const PointCloudPtr cloud2, const PointCloudPtr cloud3,
             const PointCloudPtr cloud4, const PointCloudPtr cloud5)
      : timestamp(time),
        laserCloud(cloud1),
        cornerPointsSharp(cloud2),
        cornerPointsLessSharp(cloud3),
        surfPointsFlat(cloud4),
        surfPointsLessFlat(cloud5) {}
  void reset() {
    this->timestamp = 0;
    this->laserCloud->clear();
    this->cornerPointsSharp->clear();
    this->cornerPointsLessSharp->clear();
    this->surfPointsFlat->clear();
    this->surfPointsLessFlat->clear();
    this->odom_to_init_t = Eigen::Vector3d::Identity();
    this->odom_to_init_q = Eigen::Quaterniond::Identity();
  }
  void setPose(const Eigen::Vector3d t, const Eigen::Quaterniond q) {
    this->odom_to_init_t = t;
    this->odom_to_init_q = q;
  }

  ScanSegMsg& operator=(const ScanSegMsg& rhs) {
    if (this == &rhs) return *this;
    this->timestamp = rhs.timestamp;
    this->laserCloud = rhs.laserCloud;
    this->cornerPointsSharp = rhs.cornerPointsSharp;
    this->cornerPointsLessSharp = rhs.cornerPointsLessSharp;
    this->surfPointsFlat = rhs.surfPointsFlat;
    this->surfPointsLessFlat = rhs.surfPointsLessFlat;
    return *this;
  }
};
typedef std::shared_ptr<ScanSegMsg> ScanSegMsgPtr;

struct OdomCorrect {
  Eigen::Vector3d correct_t;
  Eigen::Quaterniond correct_q;
  OdomCorrect() {
    correct_t = Eigen::Vector3d::Identity();
    correct_q = Eigen::Quaterniond(1, 0, 0, 0);
  }
  void setPose(const Eigen::Vector3d t, const Eigen::Quaterniond q) {
    this->correct_t = t;
    this->correct_q = q;
  }
};
typedef std::shared_ptr<OdomCorrect> OdomCorrectPtr;

class DataCenter {
 public:
  std::shared_ptr<DataCenter> DataCenterPtr;
  std::shared_ptr<const DataCenter> DataCenterConstPtr;

  DataCenter() {
    seg_msgs_ptr.reset(new ScanSegMsg);
    odom_correct_ptr.reset(new OdomCorrect);
  }
  DataCenter(const DataCenter& rhs) = delete;
  DataCenter& operator=(const DataCenter& rhs) = delete;

 public:
  void SetScanSegMsg(const ScanSegMsgPtr msgs) {
    seg_msgs_ptr->reset();
    *seg_msgs_ptr = *msgs;
  }
  void SetOdomCorrectPose(const Eigen::Vector3d t, const Eigen::Quaterniond q) {
    odom_correct_ptr->setPose(t, q);
  }
  const ScanSegMsgPtr GetScanSegMsg() { return seg_msgs_ptr; }
  const OdomCorrectPtr GetOdomCorrectMsg() { return odom_correct_ptr; }

 public:
  static DataCenter* Instance() {
    static DataCenter instance;
    return &instance;
  }

 private:
  // projection
  //   std::deque<PointCloudPtr> segmented_clouds;
  // segmentation
  ScanSegMsgPtr seg_msgs_ptr;
  OdomCorrectPtr odom_correct_ptr;
};
}  // namespace ceres_loam