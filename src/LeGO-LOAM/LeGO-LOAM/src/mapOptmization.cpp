// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/gtsam::Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "common.h"
#include "utility.h"

using namespace gtsam;

class mapOptimization {
 private:
  // performLoopClosure,saveKeyFramesAndGtsamFactor使用
  NonlinearFactorGraph gtsam_graph;
  ISAM2* isam;  // performLoopClosure,saveKeyFramesAndGtsamFactor使用
  Values initialEstimate;    // saveKeyFramesAndGtsamFactor使用
  Values isam_estimate;      // correctPoses,saveKeyFramesAndGtsamFactor使用
                             //-----------------------------------
  Values optimizedEstimate;  // useless
  /////////////////////////////////////

  // saveKeyFramesAndGtsamFactor
  noiseModel::Diagonal::shared_ptr priorNoise, odometryNoise;
  noiseModel::Diagonal::shared_ptr loop_noise;  // performLoopClosure

  ros::NodeHandle nh;

  ros::Publisher pubLaserCloudSurround, pub_refined_path;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubKeyPoses;

  ros::Publisher pubHistoryKeyFrames;
  ros::Publisher pubIcpKeyFrames;
  ros::Publisher pubRecentKeyFrames;
  ros::Publisher pubRegisteredCloud;

  ros::Subscriber subLaserCloudCornerLast;
  ros::Subscriber subLaserCloudSurfLast;
  ros::Subscriber subOutlierCloudLast;
  ros::Subscriber subLaserOdometry;
  ros::Subscriber subImu;

  nav_msgs::Odometry odomAftMapped;
  tf::StampedTransform aftMappedTrans;
  tf::TransformBroadcaster tfBroadcaster;

  vector<PointCloudXYZI::Ptr> corner_cloud_keyframes;
  vector<PointCloudXYZI::Ptr> surf_cloud_keyframes;
  vector<PointCloudXYZI::Ptr> outlier_cloud_keyframes;

  deque<PointCloudXYZI::Ptr> recentCornerCloudKeyFrames;
  deque<PointCloudXYZI::Ptr> recentSurfCloudKeyFrames;
  deque<PointCloudXYZI::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  vector<int> surroundingExistingKeyPosesID;
  deque<PointCloudXYZI::Ptr> surroundingCornerCloudKeyFrames;
  deque<PointCloudXYZI::Ptr> surroundingSurfCloudKeyFrames;
  deque<PointCloudXYZI::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

  PointCloudXYZI::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr key_poses6D;

  PointCloudXYZI::Ptr surroundingKeyPoses;
  PointCloudXYZI::Ptr surroundingKeyPosesDS;

  PointCloudXYZI::Ptr
      laserCloudCornerLast;  // corner feature set from odoOptimization
  PointCloudXYZI::Ptr
      laserCloudSurfLast;  // surf feature set from odoOptimization
  PointCloudXYZI::Ptr laserCloudCornerLastDS;  // downsampled corner featuer set
                                               // from odoOptimization
  PointCloudXYZI::Ptr laserCloudSurfLastDS;    // downsampled surf featuer set
                                               // from odoOptimization

  PointCloudXYZI::Ptr
      laserCloudOutlierLast;  // corner feature set from odoOptimization
  PointCloudXYZI::Ptr
      laserCloudOutlierLastDS;  // corner feature set from odoOptimization

  PointCloudXYZI::Ptr
      laserCloudSurfTotalLast;  // surf feature set from odoOptimization

  PointCloudXYZI::Ptr laserCloudOri;
  PointCloudXYZI::Ptr coeffSel;

  PointCloudXYZI::Ptr
      laserCloudSurfTotalLastDS;  // scan of scan->map, include outlier and surf
  PointCloudXYZI::Ptr laserCloudCornerFromMap,
      laserCloudCornerFromMapDS;  // scan->corner_map
  PointCloudXYZI::Ptr laserCloudSurfFromMap,
      laserCloudSurfFromMapDS;  // scan->surf_map(include OutlierCloud)

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
  pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

  PointCloudXYZI::Ptr nearHistoryCornerKeyFrameCloud;
  PointCloudXYZI::Ptr nearHistoryCornerKeyFrameCloudDS;
  PointCloudXYZI::Ptr loop_detected_submap;
  PointCloudXYZI::Ptr loop_detected_submap_filtered;

  PointCloudXYZI::Ptr latestCornerKeyFrameCloud;
  PointCloudXYZI::Ptr latest_cloud_keyframe;
  PointCloudXYZI::Ptr latestSurfKeyFrameCloudDS;

  pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
  PointCloudXYZI::Ptr globalMapKeyPoses;
  PointCloudXYZI::Ptr globalMapKeyPosesDS;
  PointCloudXYZI::Ptr globalMapKeyFrames;
  PointCloudXYZI::Ptr globalMapKeyFramesDS;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType>
      downSizeFilterHistoryKeyFrames;  // for histor key frames of loop closure
  pcl::VoxelGrid<PointType>
      downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of
                                          // scan-to-map optimization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyPoses;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyFrames;  // for global map visualization

  double timeLaserCloudCornerLast;
  double timeLaserCloudSurfLast;
  double timeLaserOdometry;
  double timeLaserCloudOutlierLast;
  double timeLastGloalMapPublish;

  bool newLaserCloudCornerLast;
  bool newLaserCloudSurfLast;
  bool newLaserOdometry;
  bool newLaserCloudOutlierLast;

  float transformLast[6];
  float transformSum[6];
  float transformIncre[6];
  float transformTobeMapped[6];  // scan2map优化的结果
  float transformBefMapped[6];  // scan2map优化后使用 "transformSum" update
  float
      transformAftMapped[6];  // scan2map优化后使用"transformTobeMapped" update

  int imuPointerFront;
  int imuPointerLast;

  double imuTime[imuQueLength];
  float imuRoll[imuQueLength];
  float imuPitch[imuQueLength];

  std::mutex mtx;

  double timeLastProcessing;

  PointType pointOri, pointSel, pointProj, coeff;

  cv::Mat matA0;
  cv::Mat matB0;
  cv::Mat matX0;

  cv::Mat matA1;
  cv::Mat matD1;
  cv::Mat matV1;

  bool isDegenerate;
  cv::Mat matP;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  bool potentialLoopFlag;
  double timeSaveFirstCurrentScanForLoopClosure;
  int detected_loop_frame_id;
  int latest_frame_id;

  bool aLoopIsClosed;

  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

 public:
  mapOptimization() : nh("~") {
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    // sub
    subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_corner_last", 2,
        &mapOptimization::laserCloudCornerLastHandler, this);
    subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>(
        "/laser_cloud_surf_last", 2,
        &mapOptimization::laserCloudSurfLastHandler, this);
    subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>(
        "/outlier_cloud_last", 2,
        &mapOptimization::laserCloudOutlierLastHandler, this);
    // sub-odom's global pose
    subLaserOdometry = nh.subscribe<nav_msgs::Odometry>(
        "/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
    subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50,
                                            &mapOptimization::imuHandler, this);
    // pub
    pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
    pubLaserCloudSurround =
        nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init",
                                                        5);  // refined pose
    pub_refined_path = nh.advertise<nav_msgs::Path>("/path/refined_path", 100);
    pubHistoryKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
    pubIcpKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
    pubRecentKeyFrames =
        nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
    pubRegisteredCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);
    // other
    downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
    downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

    downSizeFilterHistoryKeyFrames.setLeafSize(
        0.4, 0.4, 0.4);  // for histor key frames of loop closure
    downSizeFilterSurroundingKeyPoses.setLeafSize(
        1.0, 1.0,
        1.0);  // for surrounding key poses of scan-to-map optimization

    downSizeFilterGlobalMapKeyPoses.setLeafSize(
        1.0, 1.0, 1.0);  // for global map visualization
    downSizeFilterGlobalMapKeyFrames.setLeafSize(
        0.4, 0.4, 0.4);  // for global map visualization

    odomAftMapped.header.frame_id = "/camera_init";
    odomAftMapped.child_frame_id = "/aft_mapped";

    aftMappedTrans.frame_id_ = "/camera_init";
    aftMappedTrans.child_frame_id_ = "/aft_mapped";

    allocateMemory();
  }

  void allocateMemory() {
    cloudKeyPoses3D.reset(new PointCloudXYZI());
    key_poses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    surroundingKeyPoses.reset(new PointCloudXYZI());
    surroundingKeyPosesDS.reset(new PointCloudXYZI());

    laserCloudCornerLast.reset(new PointCloudXYZI());  // corner feature set
                                                       // from odoOptimization
    laserCloudSurfLast.reset(new PointCloudXYZI());    // surf feature set from
                                                       // odoOptimization
    laserCloudCornerLastDS.reset(
        new PointCloudXYZI());  // downsampled corner featuer set
                                // from odoOptimization
    laserCloudSurfLastDS.reset(
        new PointCloudXYZI());  // downsampled surf featuer set from
                                // odoOptimization
    laserCloudOutlierLast.reset(new PointCloudXYZI());  // corner feature set
                                                        // from odoOptimization
    laserCloudOutlierLastDS.reset(
        new PointCloudXYZI());  // downsampled corner feature set
                                // from odoOptimization
    laserCloudSurfTotalLast.reset(
        new PointCloudXYZI());  // surf feature set from
                                // odoOptimization
    laserCloudSurfTotalLastDS.reset(
        new PointCloudXYZI());  // downsampled surf featuer set from
                                // odoOptimization

    laserCloudOri.reset(new PointCloudXYZI());
    coeffSel.reset(new PointCloudXYZI());

    laserCloudCornerFromMap.reset(new PointCloudXYZI());
    laserCloudSurfFromMap.reset(new PointCloudXYZI());
    laserCloudCornerFromMapDS.reset(new PointCloudXYZI());
    laserCloudSurfFromMapDS.reset(new PointCloudXYZI());

    kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

    nearHistoryCornerKeyFrameCloud.reset(new PointCloudXYZI());
    nearHistoryCornerKeyFrameCloudDS.reset(new PointCloudXYZI());
    loop_detected_submap.reset(new PointCloudXYZI());
    loop_detected_submap_filtered.reset(new PointCloudXYZI());

    latestCornerKeyFrameCloud.reset(new PointCloudXYZI());
    latest_cloud_keyframe.reset(new PointCloudXYZI());
    latestSurfKeyFrameCloudDS.reset(new PointCloudXYZI());

    kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
    globalMapKeyPoses.reset(new PointCloudXYZI());
    globalMapKeyPosesDS.reset(new PointCloudXYZI());
    globalMapKeyFrames.reset(new PointCloudXYZI());
    globalMapKeyFramesDS.reset(new PointCloudXYZI());

    timeLaserCloudCornerLast = 0;
    timeLaserCloudSurfLast = 0;
    timeLaserOdometry = 0;
    timeLaserCloudOutlierLast = 0;
    timeLastGloalMapPublish = 0;

    timeLastProcessing = -1;

    newLaserCloudCornerLast = false;
    newLaserCloudSurfLast = false;

    newLaserOdometry = false;
    newLaserCloudOutlierLast = false;

    for (int i = 0; i < 6; ++i) {
      transformLast[i] = 0;
      transformSum[i] = 0;
      transformIncre[i] = 0;
      transformTobeMapped[i] = 0;
      transformBefMapped[i] = 0;
      transformAftMapped[i] = 0;
    }

    imuPointerFront = 0;
    imuPointerLast = -1;

    for (int i = 0; i < imuQueLength; ++i) {
      imuTime[i] = 0;
      imuRoll[i] = 0;
      imuPitch[i] = 0;
    }
    // gtsam noise
    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
    priorNoise = noiseModel::Diagonal::Variances(Vector6);
    odometryNoise = noiseModel::Diagonal::Variances(Vector6);

    matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
    matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
    matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

    matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
    matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
    matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

    isDegenerate = false;
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    laserCloudCornerFromMapDSNum = 0;
    laserCloudSurfFromMapDSNum = 0;
    laserCloudCornerLastDSNum = 0;
    laserCloudSurfLastDSNum = 0;
    laserCloudOutlierLastDSNum = 0;
    laserCloudSurfTotalLastDSNum = 0;

    potentialLoopFlag = false;
    aLoopIsClosed = false;

    latestFrameID = 0;
  }  // end allocateMemory

  //******************* callback receive-msgs
  void laserCloudOutlierLastHandler(
      const sensor_msgs::PointCloud2ConstPtr& msg) {
    timeLaserCloudOutlierLast = msg->header.stamp.toSec();
    laserCloudOutlierLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
    newLaserCloudOutlierLast = true;
  }
  void laserCloudCornerLastHandler(
      const sensor_msgs::PointCloud2ConstPtr& msg) {
    timeLaserCloudCornerLast = msg->header.stamp.toSec();
    laserCloudCornerLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudCornerLast);
    newLaserCloudCornerLast = true;
  }
  void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& msg) {
    timeLaserCloudSurfLast = msg->header.stamp.toSec();
    laserCloudSurfLast->clear();
    pcl::fromROSMsg(*msg, *laserCloudSurfLast);
    newLaserCloudSurfLast = true;
  }
  void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry) {
    timeLaserOdometry = laserOdometry->header.stamp.toSec();
    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
        .getRPY(roll, pitch, yaw);
    transformSum[0] = -pitch;
    transformSum[1] = -yaw;
    transformSum[2] = roll;
    transformSum[3] = laserOdometry->pose.pose.position.x;
    transformSum[4] = laserOdometry->pose.pose.position.y;
    transformSum[5] = laserOdometry->pose.pose.position.z;
    newLaserOdometry = true;
  }
  void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn) {
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
  }

  //首先执行 san2map前执行,类似aloam's lib的setInitialGuess()
  void transformAssociateToMap() {
    float x1 =
        cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) -
        sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
    float y1 = transformBefMapped[4] - transformSum[4];
    float z1 =
        sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) +
        cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

    float x2 = x1;
    float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
    float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

    transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
    transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
    transformIncre[5] = z2;

    float sbcx = sin(transformSum[0]);
    float cbcx = cos(transformSum[0]);
    float sbcy = sin(transformSum[1]);
    float cbcy = cos(transformSum[1]);
    float sbcz = sin(transformSum[2]);
    float cbcz = cos(transformSum[2]);

    float sblx = sin(transformBefMapped[0]);
    float cblx = cos(transformBefMapped[0]);
    float sbly = sin(transformBefMapped[1]);
    float cbly = cos(transformBefMapped[1]);
    float sblz = sin(transformBefMapped[2]);
    float cblz = cos(transformBefMapped[2]);

    float salx = sin(transformAftMapped[0]);
    float calx = cos(transformAftMapped[0]);
    float saly = sin(transformAftMapped[1]);
    float caly = cos(transformAftMapped[1]);
    float salz = sin(transformAftMapped[2]);
    float calz = cos(transformAftMapped[2]);

    float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                         calx * calz * cblx * cblz) -
                cbcx * sbcy *
                    (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                     calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                     cblx * salx * sbly) -
                cbcx * cbcy *
                    (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                     calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                     cblx * cbly * salx);
    transformTobeMapped[0] = -asin(srx);

    float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                           cblx * sblz * (caly * calz + salx * saly * salz) +
                           calx * saly * sblx) -
                   cbcx * cbcy *
                       ((caly * calz + salx * saly * salz) *
                            (cblz * sbly - cbly * sblx * sblz) +
                        (caly * salz - calz * salx * saly) *
                            (sbly * sblz + cbly * cblz * sblx) -
                        calx * cblx * cbly * saly) +
                   cbcx * sbcy *
                       ((caly * calz + salx * saly * salz) *
                            (cbly * cblz + sblx * sbly * sblz) +
                        (caly * salz - calz * salx * saly) *
                            (cbly * sblz - cblz * sblx * sbly) +
                        calx * cblx * saly * sbly);
    float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                           cblx * cblz * (saly * salz + caly * calz * salx) +
                           calx * caly * sblx) +
                   cbcx * cbcy *
                       ((saly * salz + caly * calz * salx) *
                            (sbly * sblz + cbly * cblz * sblx) +
                        (calz * saly - caly * salx * salz) *
                            (cblz * sbly - cbly * sblx * sblz) +
                        calx * caly * cblx * cbly) -
                   cbcx * sbcy *
                       ((saly * salz + caly * calz * salx) *
                            (cbly * sblz - cblz * sblx * sbly) +
                        (calz * saly - caly * salx * salz) *
                            (cbly * cblz + sblx * sbly * sblz) -
                        calx * caly * cblx * sbly);
    transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                   crycrx / cos(transformTobeMapped[0]));

    float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) *
                       (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                        calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                        cblx * cbly * salx) -
                   (cbcy * cbcz + sbcx * sbcy * sbcz) *
                       (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                        calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                        cblx * salx * sbly) +
                   cbcx * sbcz *
                       (salx * sblx + calx * cblx * salz * sblz +
                        calx * calz * cblx * cblz);
    float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) *
                       (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                        calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                        cblx * salx * sbly) -
                   (sbcy * sbcz + cbcy * cbcz * sbcx) *
                       (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                        calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                        cblx * cbly * salx) +
                   cbcx * cbcz *
                       (salx * sblx + calx * cblx * salz * sblz +
                        calx * calz * cblx * cblz);
    transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                   crzcrx / cos(transformTobeMapped[0]));

    x1 = cos(transformTobeMapped[2]) * transformIncre[3] -
         sin(transformTobeMapped[2]) * transformIncre[4];
    y1 = sin(transformTobeMapped[2]) * transformIncre[3] +
         cos(transformTobeMapped[2]) * transformIncre[4];
    z1 = transformIncre[5];

    x2 = x1;
    y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    transformTobeMapped[3] =
        transformAftMapped[3] -
        (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
    transformTobeMapped[4] = transformAftMapped[4] - y2;
    transformTobeMapped[5] =
        transformAftMapped[5] -
        (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
  }

  void transformUpdate() {  // scan2map后执行
    if (imuPointerLast >= 0) {
      float imuRollLast = 0, imuPitchLast = 0;
      while (imuPointerFront != imuPointerLast) {
        if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
        imuRollLast = imuRoll[imuPointerFront];
        imuPitchLast = imuPitch[imuPointerFront];
      } else {
        int imuPointerBack =
            (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront =
            (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) /
            (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack =
            (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) /
            (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollLast = imuRoll[imuPointerFront] * ratioFront +
                      imuRoll[imuPointerBack] * ratioBack;
        imuPitchLast = imuPitch[imuPointerFront] * ratioFront +
                       imuPitch[imuPointerBack] * ratioBack;
      }

      transformTobeMapped[0] =
          0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
      transformTobeMapped[2] =
          0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
    }

    for (int i = 0; i < 6; i++) {
      transformBefMapped[i] = transformSum[i];
      transformAftMapped[i] = transformTobeMapped[i];
    }
  }

  void updatePointAssociateToMapSinCos() {
    cRoll = cos(transformTobeMapped[0]);
    sRoll = sin(transformTobeMapped[0]);

    cPitch = cos(transformTobeMapped[1]);
    sPitch = sin(transformTobeMapped[1]);

    cYaw = cos(transformTobeMapped[2]);
    sYaw = sin(transformTobeMapped[2]);

    tX = transformTobeMapped[3];
    tY = transformTobeMapped[4];
    tZ = transformTobeMapped[5];
  }

  // trans cloud:lidar's frame->global's frame   rotateZXY->add pos
  void pointAssociateToMap(PointType const* const pi, PointType* const po) {
    float x1 = cYaw * pi->x - sYaw * pi->y;  // rotate by z
    float y1 = sYaw * pi->x + cYaw * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cRoll * y1 - sRoll * z1;  // rotate by x
    float z2 = sRoll * y1 + cRoll * z1;

    po->x = cPitch * x2 + sPitch * z2 + tX;  // rotate by y
    po->y = y2 + tY;
    po->z = -sPitch * x2 + cPitch * z2 + tZ;
    po->intensity = pi->intensity;
  }

  void updateTransformPointCloudSinCos(PointTypePose* tIn) {
    ctRoll = cos(tIn->roll);
    stRoll = sin(tIn->roll);

    ctPitch = cos(tIn->pitch);
    stPitch = sin(tIn->pitch);

    ctYaw = cos(tIn->yaw);
    stYaw = sin(tIn->yaw);

    tInX = tIn->x;
    tInY = tIn->y;
    tInZ = tIn->z;
  }

  // !!! DO NOT use pcl for point cloud transformation, results are not
  PointCloudXYZI::Ptr transformPointCloud(PointCloudXYZI::Ptr cloudIn) {
    // !!! DO NOT use pcl for point cloud transformation, results are not
    // accurate Reason: unkown
    PointCloudXYZI::Ptr cloudOut(new PointCloudXYZI());

    PointType* pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
      pointFrom = &cloudIn->points[i];
      float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
      float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
      float z1 = pointFrom->z;

      float x2 = x1;
      float y2 = ctRoll * y1 - stRoll * z1;
      float z2 = stRoll * y1 + ctRoll * z1;

      pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
      pointTo.y = y2 + tInY;
      pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
      pointTo.intensity = pointFrom->intensity;

      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  PointCloudXYZI::Ptr transformPointCloud(PointCloudXYZI::Ptr cloudIn,
                                          PointTypePose* transformIn) {
    PointCloudXYZI::Ptr cloudOut(new PointCloudXYZI());

    PointType* pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i) {
      pointFrom = &cloudIn->points[i];
      float x1 = cos(transformIn->yaw) * pointFrom->x -
                 sin(transformIn->yaw) * pointFrom->y;
      float y1 = sin(transformIn->yaw) * pointFrom->x +
                 cos(transformIn->yaw) * pointFrom->y;
      float z1 = pointFrom->z;

      float x2 = x1;
      float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
      float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

      pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
                  transformIn->x;
      pointTo.y = y2 + transformIn->y;
      pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
                  transformIn->z;
      pointTo.intensity = pointFrom->intensity;

      cloudOut->points[i] = pointTo;
    }
    return cloudOut;
  }

  PointTypePose trans2PointTypePose(float transformIn[]) {
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
  }

  // ********thread1-visual and save map
  void visualizeGlobalMapThread() {
    ros::Rate rate(0.2);
    while (ros::ok()) {
      rate.sleep();
      publishGlobalMap();
    }
    // save final point cloud
    {
      pcl::io::savePCDFileASCII(fileDirectory + "finalCloud.pcd",
                                *globalMapKeyFramesDS);

      string cornerMapString = "/tmp/cornerMap.pcd";
      string surfaceMapString = "/tmp/surfaceMap.pcd";
      string trajectoryString = "/tmp/trajectory.pcd";

      PointCloudXYZI::Ptr cornerMapCloud(new PointCloudXYZI());
      PointCloudXYZI::Ptr cornerMapCloudDS(new PointCloudXYZI());
      PointCloudXYZI::Ptr surfaceMapCloud(new PointCloudXYZI());
      PointCloudXYZI::Ptr surfaceMapCloudDS(new PointCloudXYZI());

      for (int i = 0; i < corner_cloud_keyframes.size(); i++) {
        *cornerMapCloud += *transformPointCloud(corner_cloud_keyframes[i],
                                                &key_poses6D->points[i]);
        *surfaceMapCloud += *transformPointCloud(surf_cloud_keyframes[i],
                                                 &key_poses6D->points[i]);
        *surfaceMapCloud += *transformPointCloud(outlier_cloud_keyframes[i],
                                                 &key_poses6D->points[i]);
      }

      downSizeFilterCorner.setInputCloud(cornerMapCloud);
      downSizeFilterCorner.filter(*cornerMapCloudDS);
      downSizeFilterSurf.setInputCloud(surfaceMapCloud);
      downSizeFilterSurf.filter(*surfaceMapCloudDS);

      pcl::io::savePCDFileASCII(fileDirectory + "cornerMap.pcd",
                                *cornerMapCloudDS);
      pcl::io::savePCDFileASCII(fileDirectory + "surfaceMap.pcd",
                                *surfaceMapCloudDS);
      pcl::io::savePCDFileASCII(fileDirectory + "trajectory.pcd",
                                *cloudKeyPoses3D);
    }
  }

  void publishGlobalMap() {
    if (pubLaserCloudSurround.getNumSubscribers() == 0) return;

    if (cloudKeyPoses3D->points.empty() == true) return;
    // kd-tree to find near key frames to visualize
    std::vector<int> pointSearchIndGlobalMap;
    std::vector<float> pointSearchSqDisGlobalMap;
    // search near key frames to visualize
    mtx.lock();
    kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
    kdtreeGlobalMap->radiusSearch(
        currentRobotPosPoint, globalMapVisualizationSearchRadius,
        pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
    mtx.unlock();

    for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
      globalMapKeyPoses->points.push_back(
          cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
    // downsample near selected key frames
    downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
    downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
    // extract visualized and downsampled key frames
    for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i) {
      int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
      *globalMapKeyFrames += *transformPointCloud(
          corner_cloud_keyframes[thisKeyInd], &key_poses6D->points[thisKeyInd]);
      *globalMapKeyFrames += *transformPointCloud(
          surf_cloud_keyframes[thisKeyInd], &key_poses6D->points[thisKeyInd]);
      *globalMapKeyFrames +=
          *transformPointCloud(outlier_cloud_keyframes[thisKeyInd],
                               &key_poses6D->points[thisKeyInd]);
    }
    // downsample visualized points
    downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
    downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

    sensor_msgs::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    cloudMsgTemp.header.frame_id = "/camera_init";
    pubLaserCloudSurround.publish(cloudMsgTemp);

    globalMapKeyPoses->clear();
    globalMapKeyPosesDS->clear();
    globalMapKeyFrames->clear();
    // globalMapKeyFramesDS->clear();
  }

  // ********thread1-loop closure
  void loopClosureThread() {
    if (loopClosureEnableFlag == false) return;

    ros::Rate rate(1);
    while (ros::ok()) {
      rate.sleep();
      performLoopClosure();
    }
  }

  void performLoopClosure() {
    if (cloudKeyPoses3D->points.empty() == true) return;
    // try to find close key frame if there are any
    if (potentialLoopFlag == false) {
      if (detectLoopClosure() == true) {
        potentialLoopFlag = true;  // find some key frames that is old enough or
                                   // close enough for loop closure
        timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
      }
      if (potentialLoopFlag == false) return;
    }
    {  // reset the flag first no matter icp successes or not
      potentialLoopFlag = false;
      // ICP Settings
      pcl::IterativeClosestPoint<PointType, PointType> icp;
      icp.setMaxCorrespondenceDistance(100);
      icp.setMaximumIterations(100);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setRANSACIterations(0);
      // Align clouds
      icp.setInputSource(latest_cloud_keyframe);          // curr
      icp.setInputTarget(loop_detected_submap_filtered);  // last-from history
      PointCloudXYZI::Ptr unused_result(new PointCloudXYZI());
      icp.align(*unused_result);

      if (icp.hasConverged() == false ||
          icp.getFitnessScore() > loop_align_threshold)
        return;
      // publish loop-corrected cloud
      if (pubIcpKeyFrames.getNumSubscribers() != 0) {
        PointCloudXYZI::Ptr closed_cloud(new PointCloudXYZI());
        pcl::transformPointCloud(*latest_cloud_keyframe, *closed_cloud,
                                 icp.getFinalTransformation());
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = "/camera_init";
        pubIcpKeyFrames.publish(cloudMsgTemp);
      }
      /*
       *get pose constraint
       */
      // get transformation in camera frame(because points are in camera frame)
      Eigen::Affine3f loop_tf_raw = icp.getFinalTransformation();
      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(loop_tf_raw, x, y, z, roll, pitch, yaw);
      Eigen::Affine3f loop_tf =  // convert loop-tf to lidar's frame
          pcl::getTransformation(z, x, y, yaw, roll, pitch);

      // latest frame's pose, camera frame->lidar frame
      Eigen::Affine3f curr_pose =
          convertPoseCameraToLidar(key_poses6D->points[latest_frame_id]);
      // transform from world origin to corrected pose
      Eigen::Affine3f curr_pose_corrected = loop_tf * curr_pose;
      pcl::getTranslationAndEulerAngles(curr_pose_corrected, x, y, z, roll,
                                        pitch, yaw);
      // gtsam's pose from->to: curr-correct's pose-->detect's pose
      gtsam::gtsam::Pose3 poseFrom =
          gtsam::Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
      gtsam::gtsam::Pose3 poseTo =
          pclPointTogtsamPose3(key_poses6D->points[detected_loop_frame_id]);
      // gtsam's loop detect noise
      gtsam::Vector Vector6(6);
      float noiseScore = icp.getFitnessScore();
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
          noiseScore;
      loop_noise = noiseModel::Diagonal::Variances(Vector6);
      // gtsam_graph add between  factor of loop's detection:
      // current_pose-->detect's pose
      std::lock_guard<std::mutex> lock(mtx);
      gtsam_graph.add(
          BetweenFactor<gtsam::Pose3>(latest_frame_id, detected_loop_frame_id,
                                      poseFrom.between(poseTo), loop_noise));
      isam->update(gtsam_graph);
      isam->update();
      gtsam_graph.resize(0);

      aLoopIsClosed = true;
    }
  }

  bool detectLoopClosure() {
    latest_cloud_keyframe
        ->clear();  // only here(curr func) and performLoopClosure
    loop_detected_submap->clear();  // only here(curr func)
    loop_detected_submap_filtered->clear();

    std::lock_guard<std::mutex> lock(mtx);
    // find the closest history key frame
    std::vector<int> pointSearchIndLoop;
    std::vector<float> near_frames_dists;  // useless
    kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
    //以当前位置为中心搜索7m范围内回环点
    kdtreeHistoryKeyPoses->radiusSearch(
        currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop,
        near_frames_dists, 0);

    detected_loop_frame_id = -1;
    for (int i = 0; i < pointSearchIndLoop.size(); ++i) {
      int id = pointSearchIndLoop[i];
      //排除当前点最近一段时间的keyframe
      if (abs(key_poses6D->points[id].time - timeLaserOdometry) > 30.0) {
        detected_loop_frame_id = id;
        break;
      }
    }
    if (detected_loop_frame_id == -1) {
      return false;
    }
    // set current keyframe, old desc:save latest key frames
    {
      latest_frame_id = cloudKeyPoses3D->points.size() - 1;
      *latest_cloud_keyframe +=
          *transformPointCloud(corner_cloud_keyframes[latest_frame_id],
                               &key_poses6D->points[latest_frame_id]);
      *latest_cloud_keyframe +=
          *transformPointCloud(surf_cloud_keyframes[latest_frame_id],
                               &key_poses6D->points[latest_frame_id]);

      PointCloudXYZI::Ptr hahaCloud(new PointCloudXYZI());
      int cloudSize = latest_cloud_keyframe->points.size();
      for (int i = 0; i < cloudSize; ++i) {
        if ((int)latest_cloud_keyframe->points[i].intensity >= 0) {
          hahaCloud->push_back(latest_cloud_keyframe->points[i]);
        }
      }
      latest_cloud_keyframe->clear();
      *latest_cloud_keyframe = *hahaCloud;
    }

    // save history near key frames to generate a submap for loop's detection
    for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum;
         ++j) {  // 2n + 1
      if (detected_loop_frame_id + j < 0 ||
          detected_loop_frame_id + j > latest_frame_id)
        continue;
      *loop_detected_submap += *transformPointCloud(
          corner_cloud_keyframes[detected_loop_frame_id + j],
          &key_poses6D->points[detected_loop_frame_id + j]);
      *loop_detected_submap += *transformPointCloud(
          surf_cloud_keyframes[detected_loop_frame_id + j],
          &key_poses6D->points[detected_loop_frame_id + j]);
    }

    downSizeFilterHistoryKeyFrames.setInputCloud(loop_detected_submap);
    downSizeFilterHistoryKeyFrames.filter(*loop_detected_submap_filtered);
    // publish history near key frames
    if (pubHistoryKeyFrames.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*loop_detected_submap_filtered, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubHistoryKeyFrames.publish(cloudMsgTemp);
    }

    return true;
  }

  gtsam::Pose3 pclPointTogtsamPose3(  // loop closure内调用
      PointTypePose thisPoint) {      // camera frame to lidar frame
    return gtsam::Pose3(
        Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                     double(thisPoint.pitch)),
        Point3(double(thisPoint.z), double(thisPoint.x), double(thisPoint.y)));
  }

  Eigen::Affine3f convertPoseCameraToLidar(  // loop closure内调用
      PointTypePose thisPoint) {             // camera frame->lidar frame
    return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                  thisPoint.yaw, thisPoint.roll,
                                  thisPoint.pitch);
  }

  // ********
  // prepare feature map(corner and surf submap)
  void extractSurroundingKeyFrames() {  // setInitGuess后, scan2map前 执行
    if (cloudKeyPoses3D->points.empty() == true) return;
    // surrounding corner and surf key frames (or map)
    if (loopClosureEnableFlag == true) {  //开启loopdetect
      // only use recent key poses for graph building
      if (recentCornerCloudKeyFrames.size() <
          surroundingKeyframeSearchNum) {  // queue is not full (the beginning
                                           // of mapping or a loop is just
                                           // closed)
        // clear recent key frames queue
        recentCornerCloudKeyFrames.clear();
        recentSurfCloudKeyFrames.clear();
        recentOutlierCloudKeyFrames.clear();
        int numPoses = cloudKeyPoses3D->points.size();
        for (int i = numPoses - 1; i >= 0; --i) {
          int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
          PointTypePose thisTransformation = key_poses6D->points[thisKeyInd];
          updateTransformPointCloudSinCos(&thisTransformation);
          // extract surrounding map
          recentCornerCloudKeyFrames.push_front(
              transformPointCloud(corner_cloud_keyframes[thisKeyInd]));
          recentSurfCloudKeyFrames.push_front(
              transformPointCloud(surf_cloud_keyframes[thisKeyInd]));
          recentOutlierCloudKeyFrames.push_front(
              transformPointCloud(outlier_cloud_keyframes[thisKeyInd]));
          if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
            break;
        }
      } else {  // queue is full, pop the oldest key frame and push the latest
                // key frame
        if (latestFrameID != cloudKeyPoses3D->points.size() -
                                 1) {  // if the robot is not moving, no need to
                                       // update recent frames

          recentCornerCloudKeyFrames.pop_front();
          recentSurfCloudKeyFrames.pop_front();
          recentOutlierCloudKeyFrames.pop_front();
          // push latest scan to the end of queue
          latestFrameID = cloudKeyPoses3D->points.size() - 1;
          PointTypePose thisTransformation = key_poses6D->points[latestFrameID];
          updateTransformPointCloudSinCos(&thisTransformation);
          recentCornerCloudKeyFrames.push_back(
              transformPointCloud(corner_cloud_keyframes[latestFrameID]));
          recentSurfCloudKeyFrames.push_back(
              transformPointCloud(surf_cloud_keyframes[latestFrameID]));
          recentOutlierCloudKeyFrames.push_back(
              transformPointCloud(outlier_cloud_keyframes[latestFrameID]));
        }
      }
      // generate feature map,when loop detect is on
      for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i) {
        *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
        *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
        *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
      }
    } else {
      //未开启loopdetect
      surroundingKeyPoses->clear();
      surroundingKeyPosesDS->clear();
      // extract all the nearby key poses and downsample them
      kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
      kdtreeSurroundingKeyPoses->radiusSearch(
          currentRobotPosPoint, (double)surroundingKeyframeSearchRadius,
          pointSearchInd, pointSearchSqDis, 0);
      for (int i = 0; i < pointSearchInd.size(); ++i)
        surroundingKeyPoses->points.push_back(
            cloudKeyPoses3D->points[pointSearchInd[i]]);
      downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
      downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
      // delete key frames that are not in surrounding region
      int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
      for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
        bool existingFlag = false;
        for (int j = 0; j < numSurroundingPosesDS; ++j) {
          if (surroundingExistingKeyPosesID[i] ==
              (int)surroundingKeyPosesDS->points[j].intensity) {
            existingFlag = true;
            break;
          }
        }
        if (existingFlag == false) {
          surroundingExistingKeyPosesID.erase(
              surroundingExistingKeyPosesID.begin() + i);
          surroundingCornerCloudKeyFrames.erase(
              surroundingCornerCloudKeyFrames.begin() + i);
          surroundingSurfCloudKeyFrames.erase(
              surroundingSurfCloudKeyFrames.begin() + i);
          surroundingOutlierCloudKeyFrames.erase(
              surroundingOutlierCloudKeyFrames.begin() + i);
          --i;
        }
      }
      // add new key frames that are not in calculated existing key frames
      for (int i = 0; i < numSurroundingPosesDS; ++i) {
        bool existingFlag = false;
        for (auto iter = surroundingExistingKeyPosesID.begin();
             iter != surroundingExistingKeyPosesID.end(); ++iter) {
          if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity) {
            existingFlag = true;
            break;
          }
        }
        if (existingFlag == true) {
          continue;
        } else {
          int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
          PointTypePose thisTransformation = key_poses6D->points[thisKeyInd];
          updateTransformPointCloudSinCos(&thisTransformation);
          surroundingExistingKeyPosesID.push_back(thisKeyInd);
          surroundingCornerCloudKeyFrames.push_back(
              transformPointCloud(corner_cloud_keyframes[thisKeyInd]));
          surroundingSurfCloudKeyFrames.push_back(
              transformPointCloud(surf_cloud_keyframes[thisKeyInd]));
          surroundingOutlierCloudKeyFrames.push_back(
              transformPointCloud(outlier_cloud_keyframes[thisKeyInd]));
        }
      }
      // generate feature map,when loop detect is off
      for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
        *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
        *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
        *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
      }
    }
    // downsample corner map
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    // feature's map   scan->corner_map
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();

    // downsample surf map
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    // feature's map  scan->surf_map(include OutlierCloud)
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
  }

  // prepare current corner and surf scan
  void downsampleCurrentScan() {
    ////////////////////////////////
    laserCloudCornerLastDS->clear();  // current corner scan
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();
    ////////////////////////////////
    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
    laserCloudOutlierLastDS->clear();
    downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
    downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
    laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();
    laserCloudSurfTotalLast->clear();
    laserCloudSurfTotalLastDS->clear();  // current surf(include outlier) scan
    *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
    *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
    downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
    downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
    laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
  }

  void cornerOptimization(int iterCount) {
    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
      pointOri = laserCloudCornerLastDS->points[i];
      // lidar's frame->global's frame
      pointAssociateToMap(&pointOri, &pointSel);
      kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                          pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0) {
        float cx = 0, cy = 0, cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
          cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
          cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
        }
        cx /= 5;
        cy /= 5;
        cz /= 5;

        float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax =
              laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
          float ay =
              laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
          float az =
              laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5;
        a12 /= 5;
        a13 /= 5;
        a22 /= 5;
        a23 /= 5;
        a33 /= 5;

        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1);

        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
          float x0 = pointSel.x;
          float y0 = pointSel.y;
          float z0 = pointSel.z;
          float x1 = cx + 0.1 * matV1.at<float>(0, 0);
          float y1 = cy + 0.1 * matV1.at<float>(0, 1);
          float z1 = cz + 0.1 * matV1.at<float>(0, 2);
          float x2 = cx - 0.1 * matV1.at<float>(0, 0);
          float y2 = cy - 0.1 * matV1.at<float>(0, 1);
          float z2 = cz - 0.1 * matV1.at<float>(0, 2);

          float a012 =
              sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                       ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                   ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                   ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                       ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                           (z1 - z2) * (z1 - z2));

          float la =
              ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
               (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
              a012 / l12;

          float lb =
              -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float lc =
              -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float ld2 = a012 / l12;

          float s = 1 - 0.9 * fabs(ld2);

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          if (s > 0.1) {
            laserCloudOri->push_back(pointOri);
            coeffSel->push_back(coeff);
          }
        }
      }
    }
  }

  void surfOptimization(int iterCount) {
    updatePointAssociateToMapSinCos();
    for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
      pointOri = laserCloudSurfTotalLastDS->points[i];
      // lidar's frame->global's frame
      pointAssociateToMap(&pointOri, &pointSel);
      kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                        pointSearchSqDis);

      if (pointSearchSqDis[4] < 1.0) {
        for (int j = 0; j < 5; j++) {
          matA0.at<float>(j, 0) =
              laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
          matA0.at<float>(j, 1) =
              laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
          matA0.at<float>(j, 2) =
              laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
        }
        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

        float pa = matX0.at<float>(0, 0);
        float pb = matX0.at<float>(1, 0);
        float pc = matX0.at<float>(2, 0);
        float pd = 1;

        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                   pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                   pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                   pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

          float s = 1 - 0.9 * fabs(pd2) /
                            sqrt(sqrt(pointSel.x * pointSel.x +
                                      pointSel.y * pointSel.y +
                                      pointSel.z * pointSel.z));

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          if (s > 0.1) {
            laserCloudOri->push_back(pointOri);
            coeffSel->push_back(coeff);
          }
        }
      }
    }
  }

  bool LMOptimization(int iterCount) {
    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    int laserCloudSelNum = laserCloudOri->points.size();
    if (laserCloudSelNum < 50) {
      return false;
    }

    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    for (int i = 0; i < laserCloudSelNum; i++) {
      pointOri = laserCloudOri->points[i];
      coeff = coeffSel->points[i];

      float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
                   srx * sry * pointOri.z) *
                      coeff.x +
                  (-srx * srz * pointOri.x - crz * srx * pointOri.y -
                   crx * pointOri.z) *
                      coeff.y +
                  (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
                   cry * srx * pointOri.z) *
                      coeff.z;

      float ary = ((cry * srx * srz - crz * sry) * pointOri.x +
                   (sry * srz + cry * crz * srx) * pointOri.y +
                   crx * cry * pointOri.z) *
                      coeff.x +
                  ((-cry * crz - srx * sry * srz) * pointOri.x +
                   (cry * srz - crz * srx * sry) * pointOri.y -
                   crx * sry * pointOri.z) *
                      coeff.z;

      float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                   (-cry * crz - srx * sry * srz) * pointOri.y) *
                      coeff.x +
                  (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                  ((sry * srz + cry * crz * srx) * pointOri.x +
                   (crz * sry - cry * srx * srz) * pointOri.y) *
                      coeff.z;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      matA.at<float>(i, 3) = coeff.x;
      matA.at<float>(i, 4) = coeff.y;
      matA.at<float>(i, 5) = coeff.z;
      matB.at<float>(i, 0) = -coeff.intensity;
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate = false;
      float eignThre[6] = {100, 100, 100, 100, 100, 100};
      for (int i = 5; i >= 0; i--) {
        if (matE.at<float>(0, i) < eignThre[i]) {
          for (int j = 0; j < 6; j++) {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        } else {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
      return true;
    }
    return false;
  }

  void scan2MapOptimization() {
    if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
      kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
      kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

      for (int iterCount = 0; iterCount < 10; iterCount++) {
        laserCloudOri->clear();
        coeffSel->clear();

        cornerOptimization(iterCount);
        surfOptimization(iterCount);

        if (LMOptimization(iterCount) == true) break;
      }

      transformUpdate();
    }
  }

  // scan2MapOptimization后执行
  void saveKeyFramesAndGtsamFactor() {
    // 0.3m为一个keyframe
    currentRobotPosPoint.x = transformAftMapped[3];
    currentRobotPosPoint.y = transformAftMapped[4];
    currentRobotPosPoint.z = transformAftMapped[5];
    {
      bool saveThisKeyFrame = true;
      if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) *
                   (previousRobotPosPoint.x - currentRobotPosPoint.x) +
               (previousRobotPosPoint.y - currentRobotPosPoint.y) *
                   (previousRobotPosPoint.y - currentRobotPosPoint.y) +
               (previousRobotPosPoint.z - currentRobotPosPoint.z) *
                   (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3) {
        saveThisKeyFrame = false;
      }

      if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty()) return;

      previousRobotPosPoint = currentRobotPosPoint;
    }

    /**
     * 1.update gtsam graph, 添加prior-factor and between-factor,
     * 2.add value for initialEstimate
     */
    if (cloudKeyPoses3D->points.empty()) {
      // gtsam_graph add prior's factor
      gtsam_graph.add(PriorFactor<gtsam::Pose3>(
          0,
          gtsam::Pose3(
              Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                           transformTobeMapped[1]),
              Point3(transformTobeMapped[5], transformTobeMapped[3],
                     transformTobeMapped[4])),
          priorNoise));
      // initialEstimate add value
      initialEstimate.insert(
          0, gtsam::Pose3(
                 Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                              transformTobeMapped[1]),
                 Point3(transformTobeMapped[5], transformTobeMapped[3],
                        transformTobeMapped[4])));
      // transformLast 只在saveKeyFramesAndGtsamFactor使用
      for (int i = 0; i < 6; ++i) transformLast[i] = transformTobeMapped[i];
    } else {
      gtsam::gtsam::Pose3 poseFrom = gtsam::Pose3(  // last pose
          Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
          Point3(transformLast[5], transformLast[3], transformLast[4]));
      gtsam::gtsam::Pose3 poseTo =  // current pose
          gtsam::Pose3(
              Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4]));
      // gtsam_graph add trans, that is between's factor, index start with "0"
      // poseFrom.between(poseTo) 相当于poseFrom.inverse * poseTo
      gtsam_graph.add(BetweenFactor<gtsam::Pose3>(
          cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(),
          poseFrom.between(poseTo), odometryNoise));
      // initialEstimate add value
      initialEstimate.insert(
          cloudKeyPoses3D->points.size(),
          gtsam::Pose3(
              Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4])));
    }
    /**
     * update iSAM for optimization
     */
    isam->update(gtsam_graph, initialEstimate);
    isam->update();

    gtsam_graph.resize(0);    // last one
    initialEstimate.clear();  // last one

    /**
     * save key optimized's poses
     */

    isam_estimate = isam->calculateEstimate();
    gtsam::Pose3 current_estimate =
        isam_estimate.at<gtsam::Pose3>(isam_estimate.size() - 1);

    PointType thisPose3D;
    thisPose3D.x = current_estimate.translation().y();
    thisPose3D.y = current_estimate.translation().z();
    thisPose3D.z = current_estimate.translation().x();
    thisPose3D.intensity = cloudKeyPoses3D->points.size();  //  used as index
    cloudKeyPoses3D->push_back(thisPose3D);                 //仅在此处添加

    PointTypePose thisPose6D;
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;  //  used as index
    thisPose6D.roll = current_estimate.rotation().pitch();
    thisPose6D.pitch = current_estimate.rotation().yaw();
    thisPose6D.yaw = current_estimate.rotation().roll();  // in camera frame
    thisPose6D.time = timeLaserOdometry;
    key_poses6D->push_back(thisPose6D);  //仅在此处添加

    /**
     * save updated transform with optimizal result
     */
    if (cloudKeyPoses3D->points.size() > 1) {
      transformAftMapped[0] = current_estimate.rotation().pitch();
      transformAftMapped[1] = current_estimate.rotation().yaw();
      transformAftMapped[2] = current_estimate.rotation().roll();
      transformAftMapped[3] = current_estimate.translation().y();
      transformAftMapped[4] = current_estimate.translation().z();
      transformAftMapped[5] = current_estimate.translation().x();
      for (int i = 0; i < 6; ++i) {
        transformLast[i] = transformAftMapped[i];
        transformTobeMapped[i] = transformAftMapped[i];
      }
    }

    PointCloudXYZI::Ptr corner_keyframe(new PointCloudXYZI());
    PointCloudXYZI::Ptr surf_keyframe(new PointCloudXYZI());
    PointCloudXYZI::Ptr outlier_keyframe(new PointCloudXYZI());
    // save corner and surf(参与scan2map优化)
    pcl::copyPointCloud(*laserCloudCornerLastDS, *corner_keyframe);
    pcl::copyPointCloud(*laserCloudSurfLastDS, *surf_keyframe);
    pcl::copyPointCloud(*laserCloudOutlierLastDS, *outlier_keyframe);

    corner_cloud_keyframes.push_back(corner_keyframe);
    surf_cloud_keyframes.push_back(surf_keyframe);
    outlier_cloud_keyframes.push_back(outlier_keyframe);
  }

  void correctPoses() {           // saveKeyFramesAndGtsamFactor后执行
    if (aLoopIsClosed == true) {  // performLoopClosure 内set true
      recentCornerCloudKeyFrames.clear();
      recentSurfCloudKeyFrames.clear();
      recentOutlierCloudKeyFrames.clear();
      // update key poses
      int numPoses = isam_estimate.size();
      for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 temp = isam_estimate.at<gtsam::Pose3>(i);
        cloudKeyPoses3D->points[i].x = temp.translation().y();
        cloudKeyPoses3D->points[i].y = temp.translation().z();
        cloudKeyPoses3D->points[i].z = temp.translation().x();

        key_poses6D->points[i].x = cloudKeyPoses3D->points[i].x;
        key_poses6D->points[i].y = cloudKeyPoses3D->points[i].y;
        key_poses6D->points[i].z = cloudKeyPoses3D->points[i].z;
        key_poses6D->points[i].roll = temp.rotation().pitch();
        key_poses6D->points[i].pitch = temp.rotation().yaw();
        key_poses6D->points[i].yaw = temp.rotation().roll();
      }

      aLoopIsClosed = false;
    }
  }

  void clearCloud() {
    laserCloudCornerFromMap->clear();
    laserCloudCornerFromMapDS->clear();

    laserCloudSurfFromMap->clear();
    laserCloudSurfFromMapDS->clear();
  }

  void run() {
    if (newLaserCloudCornerLast &&
        std::abs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
        newLaserCloudSurfLast &&
        std::abs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
        newLaserCloudOutlierLast &&
        std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
        newLaserOdometry) {
      newLaserCloudCornerLast = false;
      newLaserCloudSurfLast = false;
      newLaserCloudOutlierLast = false;
      newLaserOdometry = false;

      std::lock_guard<std::mutex> lock(mtx);

      if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval) {
        timeLastProcessing = timeLaserOdometry;
        //类似aloam's lib的setInitialGuess()
        transformAssociateToMap();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        saveKeyFramesAndGtsamFactor();

        correctPoses();  // loop detect and correct

        publishTF();

        publishKeyPosesAndFrames();

        clearCloud();
      }
    }
  }

  void publishKeyPosesAndFrames() {
    if (pubKeyPoses.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubKeyPoses.publish(cloudMsgTemp);
    }

    if (pubRecentKeyFrames.getNumSubscribers() != 0) {
      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubRecentKeyFrames.publish(cloudMsgTemp);
    }

    if (pubRegisteredCloud.getNumSubscribers() != 0) {
      PointCloudXYZI::Ptr cloudOut(new PointCloudXYZI());
      PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
      *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
      *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);

      sensor_msgs::PointCloud2 cloudMsgTemp;
      pcl::toROSMsg(*cloudOut, cloudMsgTemp);
      cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
      cloudMsgTemp.header.frame_id = "/camera_init";
      pubRegisteredCloud.publish(cloudMsgTemp);
    }
  }
  void publishTF() {
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
        transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

    odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
    odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
    odomAftMapped.pose.pose.orientation.z = geoQuat.x;
    odomAftMapped.pose.pose.orientation.w = geoQuat.w;
    odomAftMapped.pose.pose.position.x = transformAftMapped[3];
    odomAftMapped.pose.pose.position.y = transformAftMapped[4];
    odomAftMapped.pose.pose.position.z = transformAftMapped[5];
    odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
    odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
    odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
    odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
    odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
    odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
    pubOdomAftMapped.publish(odomAftMapped);
    static nav_msgs::Path refined_path;
    publishPath(pub_refined_path, refined_path, odomAftMapped);

    aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
    aftMappedTrans.setRotation(
        tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    aftMappedTrans.setOrigin(tf::Vector3(
        transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
    tfBroadcaster.sendTransform(aftMappedTrans);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");

  ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

  mapOptimization MO;

  std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
  std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread,
                                 &MO);

  ros::Rate rate(200);
  while (ros::ok()) {
    ros::spinOnce();

    MO.run();

    rate.sleep();
  }

  loopthread.join();
  visualizeMapThread.join();

  return 0;
}
