#include "lio_sam/cloud_info.h"
#include "utility.h"

// Velodyne
struct PointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(float, time, time))

const int queueLength = 2000;

class ImageProjection : public ParamServer {
 private:
  std::mutex imuLock;
  std::mutex odoLock;
  std::mutex insLock;

  ros::Subscriber subLaserCloud;
  ros::Publisher pubLaserCloud;

  ros::Publisher pubExtractedCloud;
  ros::Publisher pubLaserCloudInfo;

  ros::Subscriber subInspva;
  std::deque<adu_common_odometry_msgs::adu_common_odometry> inspvaQueue;

  ros::Subscriber subImu;
  std::deque<sensor_msgs::Imu> imuQueue;

  ros::Subscriber subOdom;
  std::deque<nav_msgs::Odometry> odomQueue;

  std::deque<sensor_msgs::PointCloud2> cloudQueue;
  sensor_msgs::PointCloud2 currentCloudMsg;

  //补偿点云的imu信息 源自角速度
  double *imuTime = new double[queueLength];
  double *imuRotX = new double[queueLength];
  double *imuRotY = new double[queueLength];
  double *imuRotZ = new double[queueLength];

  int imuPointerCur;
  bool firstPointFlag;
  Eigen::Affine3f transStartInverse;

  pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;  //原始scan
  pcl::PointCloud<PointType>::Ptr
      fullCloud;  //补偿点云, 且索引已与原始scan不一样
  pcl::PointCloud<PointType>::Ptr extractedCloud;
  cv::Mat rangeMat;  //原始scan映射的matrix,值为range
  int deskewFlag;

  // odomTopic+"_incremental"的pose中提取的scan首位时间戳pose的trans,
  // roll/pitch/yaw的Incre不使用
  bool odomDeskewFlag;
  float odomIncreX;
  float odomIncreY;
  float odomIncreZ;

  lio_sam::cloud_info cloudInfo;
  double timeScanCur;
  double timeScanEnd;
  std_msgs::Header cloudHeader;

 public:
  ImageProjection() : deskewFlag(0) {
    //-------->sub
    subInspva = nh.subscribe<adu_common_odometry_msgs::adu_common_odometry>(
        inspvaTopic, 2000, &ImageProjection::InpvaHandler, this,
        ros::TransportHints().tcpNoDelay());
    subImu = nh.subscribe<sensor_msgs::Imu>(
        imuTopic, 2000, &ImageProjection::imuHandler, this,
        ros::TransportHints().tcpNoDelay());  // raw-imu
    // sub  "/odometry/imu_incremental"
    subOdom = nh.subscribe<nav_msgs::Odometry>(
        odomTopic + "_incremental", 2000, &ImageProjection::odometryHandler,
        this, ros::TransportHints().tcpNoDelay());
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
        pointCloudTopic, 5, &ImageProjection::cloudHandler, this,
        ros::TransportHints().tcpNoDelay());
    //-------->pub
    pubLaserCloud =
        nh.advertise<sensor_msgs::PointCloud2>("lio_sam/point_raw", 1);
    pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>(
        "lio_sam/deskew/cloud_deskewed", 1);
    pubLaserCloudInfo =
        nh.advertise<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1);

    allocateMemory();
    resetParameters();

    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  void allocateMemory() {
    laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
    fullCloud.reset(new pcl::PointCloud<PointType>());
    extractedCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN * Horizon_SCAN);

    cloudInfo.startRingIndex.assign(N_SCAN, 0);
    cloudInfo.endRingIndex.assign(N_SCAN, 0);

    cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
    cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);

    resetParameters();
  }

  void resetParameters() {
    laserCloudIn->clear();
    extractedCloud->clear();
    // reset range matrix for range image projection
    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

    imuPointerCur = 0;
    firstPointFlag = true;
    odomDeskewFlag = false;

    for (int i = 0; i < queueLength; ++i) {
      imuTime[i] = 0;
      imuRotX[i] = 0;
      imuRotY[i] = 0;
      imuRotZ[i] = 0;
    }
  }

  ~ImageProjection() {}

  void InpvaHandler(
      const adu_common_odometry_msgs::adu_common_odometry::ConstPtr
          &inspva_msg) {
    std::lock_guard<std::mutex> lock(insLock);
    inspvaQueue.push_back(*inspva_msg);

    // debug
  }

  void imuHandler(const sensor_msgs::Imu::ConstPtr &imuMsg) {
    LOG(INFO) << "Debug raw-IMU acc:**************************";
    LOG(INFO) << "raw-imu-acc-x:" << imuMsg->linear_acceleration.x;
    LOG(INFO) << "raw-imu-acc-y:" << imuMsg->linear_acceleration.y;
    LOG(INFO) << "raw-imu-acc-z:" << imuMsg->linear_acceleration.z;
    LOG(INFO) << "Debug raw-IMU gyro:";
    LOG(INFO) << "raw-imu-gyro-x:" << imuMsg->angular_velocity.x;
    LOG(INFO) << "raw-imu-gyro-y:" << imuMsg->angular_velocity.y;
    LOG(INFO) << "raw-imu-gyro-z:" << imuMsg->angular_velocity.z;
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);

    // debug IMU data
    cout << std::setprecision(6);
    LOG(INFO) << "-------------Debug Correct-IMU acc:";
    LOG(INFO) << "correct-imu-acc-x:" << thisImu.linear_acceleration.x;
    LOG(INFO) << "correct-imu-acc-y:" << thisImu.linear_acceleration.y;
    LOG(INFO) << "correct-imu-acc-z:" << thisImu.linear_acceleration.z;

    LOG(INFO) << "-------------Debug Correct-IMU gyro:";
    LOG(INFO) << "correct-imu-gyro-x:" << thisImu.angular_velocity.x;
    LOG(INFO) << "correct-imu-gyro-y:" << thisImu.angular_velocity.y;
    LOG(INFO) << "correct-imu-gyro-z:" << thisImu.angular_velocity.z;
    // double imuRoll, imuPitch, imuYaw;
    // tf::Quaternion orientation;
    // tf::quaternionMsgToTF(thisImu.orientation, orientation);
    // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
    // cout << "IMU roll pitch yaw: " << endl;
    // cout << "roll: " << imuRoll << ", pitch: " << imuPitch
    //      << ", yaw: " << imuYaw << endl
    //      << endl;
  }

  void odometryHandler(const nav_msgs::Odometry::ConstPtr &odometryMsg) {
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
  }

  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp,
    //              lidarFrame);
    // publishRawPoints(ros::Publisher * pub, sensor_msgs::PointCloud2 & msg,
    //                  std::string & frame);
    publishRawPoints(&pubLaserCloud, laserCloudMsg, lidarFrame);

    LOG(INFO) << "***********scan**************";
    if (!cachePointCloud(laserCloudMsg)) return;

    if (!deskewInfo())  //获取点云补偿需要的roll pitch yaw以及odom位移信息
      return;

    projectPointCloud();  //生成rangeMat 和 fullCloud

    cloudExtraction();

    publishClouds();

    resetParameters();
  }

  bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {
    // cache point cloud
    cloudQueue.push_back(*laserCloudMsg);
    if (cloudQueue.size() <= 2) return false;

    // convert cloud
    currentCloudMsg = cloudQueue.front();
    cloudQueue.pop_front();
    pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

    // get timestamp
    cloudHeader = currentCloudMsg.header;
    timeScanCur = cloudHeader.stamp.toSec();
    timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
    if (!use_demo) {
      timeScanEnd = cloudHeader.stamp.toSec();  // neolix
      timeScanCur = timeScanEnd - 0.108;
    }

    LOG(INFO) << std::fixed << std::setprecision(6) << "from:" << timeScanCur
              << "->" << timeScanEnd;
    LOG(INFO) << std::fixed << std::setprecision(6)
              << "scan elapsed:" << timeScanEnd - timeScanCur;

    return true;
  }

  //获取对点云做补偿时需要的imu信息
  bool deskewInfo() {
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);
    std::lock_guard<std::mutex> lock3(insLock);

    // make sure IMU data available for the scan
    if (imuQueue.empty() ||
        imuQueue.front().header.stamp.toSec() > timeScanCur ||
        imuQueue.back().header.stamp.toSec() < timeScanEnd) {
      LOG(INFO) << std::fixed << std::setprecision(6)
                << "scan time:" << timeScanCur << "->" << timeScanEnd;
      LOG(INFO) << std::fixed << std::setprecision(6)
                << "imu time :" << imuQueue.front().header.stamp.toSec() << "->"
                << imuQueue.back().header.stamp.toSec();
      ROS_DEBUG("Waiting for IMU data ...");
      return false;
    }
    imuDeskewInfo();  //提供点云补偿需要的 roll pitch yaw信息(每帧开始时刻都为0)
    odomDeskewInfo();  //提供点云补偿需要的 delta_x delta_y delta_z变化
    return true;
  }
  //提供当前帧点云 time_start到time_end期间各时刻对应的roll pitch
  // yaw(每帧开始时刻都为0),from angular-velocity of imu
  void imuDeskewInfo() {
    cloudInfo.imuAvailable = false;
    // checkout imu data
    while (!imuQueue.empty()) {
      if (imuQueue.front().header.stamp.toSec() <
          timeScanCur - 0.01)  // timeScanCur当前帧点云时间戳
        imuQueue.pop_front();
      else
        break;
    }
    if (imuQueue.empty()) {
      LOG(INFO) << "imuQueue is empty!!!!";
      return;
    }
    // neolix-start
    if (!use_demo) {
      while (!inspvaQueue.empty()) {
        if (inspvaQueue.front().header.stamp.toSec() < timeScanCur - 0.01) {
          // LOG(INFO) << std::fixed << std::setprecision(6) << "scan-ins:"
          //           << timeScanCur -
          inspvaQueue.front().header.stamp.toSec();
          inspvaQueue.pop_front();
        } else
          break;
      }
      if (inspvaQueue.empty()) {
        LOG(INFO) << "inspvaQueue is empty!!!!";
        return;
      }
    }
    // neolix-end

    imuPointerCur = 0;
    //当前帧点云从开始到结束对应的imu的姿态
    for (int i = 0; i < (int)imuQueue.size(); ++i) {
      sensor_msgs::Imu thisImuMsg = imuQueue[i];
      double currentImuTime = thisImuMsg.header.stamp.toSec();
      // demo-start
      if (use_demo) {
        // get roll, pitch, and yaw estimation for this scan
        if (currentImuTime <= timeScanCur) {
          // imu_msg姿态中的roll pitch yaw
          imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit,
                        &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
        }
      } else {
        // neolix-start
        double curInsTime = inspvaQueue[i].header.stamp.toSec();
        if (curInsTime <= timeScanCur) {
          LOG(INFO) << std::fixed << std::setprecision(6)
                    << "Ins's size:" << inspvaQueue.size() << ", ins[" << i
                    << "] - timeScanCur:" << curInsTime - timeScanCur;
          inspva2rosRPY(inspvaQueue[i], &cloudInfo.imuRollInit,
                        &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
        }
        // neolix-end
      }

      if (currentImuTime > timeScanEnd + 0.01) break;

      if (imuPointerCur == 0) {
        imuRotX[0] = 0;
        imuRotY[0] = 0;
        imuRotZ[0] = 0;
        imuTime[0] = currentImuTime;
        ++imuPointerCur;
        continue;
      }

      // get angular velocity
      double gyro_velocity_x, gyro_velocity_y, gyro_velocity_z;
      imuAngular2rosAngular(&thisImuMsg, &gyro_velocity_x, &gyro_velocity_y,
                            &gyro_velocity_z);  // imu_msg中的角速度

      // integrate rotation
      double timeDiff = currentImuTime - imuTime[imuPointerCur - 1];
      imuRotX[imuPointerCur] =
          imuRotX[imuPointerCur - 1] + gyro_velocity_x * timeDiff;
      imuRotY[imuPointerCur] =
          imuRotY[imuPointerCur - 1] + gyro_velocity_y * timeDiff;
      imuRotZ[imuPointerCur] =
          imuRotZ[imuPointerCur - 1] + gyro_velocity_z * timeDiff;
      imuTime[imuPointerCur] = currentImuTime;
      ++imuPointerCur;
    }

    --imuPointerCur;

    if (imuPointerCur <= 0) return;

    cloudInfo.imuAvailable = true;  //补偿点云的imu信息可用
  }
  //提供当前帧点云 time_start到time_end期间lidar的位移 即odomIncreX, odomIncreY,
  // odomIncreZ
  void odomDeskewInfo() {
    cloudInfo.odomAvailable = false;

    while (!odomQueue.empty()) {
      if (odomQueue.front().header.stamp.toSec() <
          timeScanCur - 0.01)  // timeScanCur当前帧点云时间戳
        odomQueue.pop_front();
      else
        break;
    }

    if (odomQueue.empty()) return;

    if (odomQueue.front().header.stamp.toSec() > timeScanCur) return;

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry startOdomMsg;
    for (int i = 0; i < (int)odomQueue.size(); ++i) {
      startOdomMsg =
          odomQueue[i];  // odom_pose from topic:"lio_sam/odomTopic_incremental"
      if (ROS_TIME(&startOdomMsg) < timeScanCur)
        continue;
      else
        break;
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
    cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
    cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.initialGuessRoll = roll;
    cloudInfo.initialGuessPitch = pitch;
    cloudInfo.initialGuessYaw = yaw;
    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    odomDeskewFlag = false;
    if (odomQueue.back().header.stamp.toSec() < timeScanEnd) return;
    nav_msgs::Odometry endOdomMsg;
    for (int i = 0; i < (int)odomQueue.size(); ++i) {
      endOdomMsg = odomQueue[i];
      if (ROS_TIME(&endOdomMsg) < timeScanEnd)
        continue;
      else
        break;
    }

    if (int(round(startOdomMsg.pose.covariance[0])) !=
        int(round(endOdomMsg.pose.covariance[0])))
      return;

    Eigen::Affine3f transBegin = pcl::getTransformation(
        startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
        startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(
        endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y,
        endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY,
                                      odomIncreZ, rollIncre, pitchIncre,
                                      yawIncre);

    odomDeskewFlag = true;
  }

  //获取补偿每个点时需要的roll pitch yaw(第一个点的姿态为初始姿态，都为0)
  void findRotation(double pointTime, float *rotXCur, float *rotYCur,
                    float *rotZCur) {
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    int imuPointerFront = 0;
    while (imuPointerFront < imuPointerCur) {
      if (pointTime < imuTime[imuPointerFront]) break;
      ++imuPointerFront;
    }

    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0) {
      *rotXCur = imuRotX[imuPointerFront];
      *rotYCur = imuRotY[imuPointerFront];
      *rotZCur = imuRotZ[imuPointerFront];
    } else {
      int imuPointerBack = imuPointerFront - 1;
      double ratioFront = (pointTime - imuTime[imuPointerBack]) /
                          (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      double ratioBack = (imuTime[imuPointerFront] - pointTime) /
                         (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      *rotXCur = imuRotX[imuPointerFront] * ratioFront +
                 imuRotX[imuPointerBack] * ratioBack;
      *rotYCur = imuRotY[imuPointerFront] * ratioFront +
                 imuRotY[imuPointerBack] * ratioBack;
      *rotZCur = imuRotZ[imuPointerFront] * ratioFront +
                 imuRotZ[imuPointerBack] * ratioBack;
    }
  }

  //补偿每个点
  PointType deskewPoint(PointType *point, double relTime) {
    // deskewFlag为1时表示scan中每个点有时间戳
    if (deskewFlag == -1 || cloudInfo.imuAvailable == false) return *point;

    double pointTime = timeScanCur + relTime;

    float rotXCur, rotYCur, rotZCur;
    findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

    float posXCur(0), posYCur(0), posZCur(0);
    // findPosition(relTime, &posXCur, &posYCur, &posZCur);

    if (firstPointFlag == true) {
      transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                  rotXCur, rotYCur, rotZCur))
                              .inverse();
      firstPointFlag = false;
    }

    // transform points to start
    Eigen::Affine3f transFinal = pcl::getTransformation(
        posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
    Eigen::Affine3f transBt = transStartInverse * transFinal;
    //补偿当前点, 相当于每个点左乘transBt
    PointType newPoint;
    newPoint.x = transBt(0, 0) * point->x + transBt(0, 1) * point->y +
                 transBt(0, 2) * point->z + transBt(0, 3);
    newPoint.y = transBt(1, 0) * point->x + transBt(1, 1) * point->y +
                 transBt(1, 2) * point->z + transBt(1, 3);
    newPoint.z = transBt(2, 0) * point->x + transBt(2, 1) * point->y +
                 transBt(2, 2) * point->z + transBt(2, 3);
    newPoint.intensity = point->intensity;

    return newPoint;
  }

  void projectPointCloud() {
    LOG(INFO) << "-------projectPointCloud";
    int cloudSize = laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i) {
      PointType thisPoint;
      thisPoint.x = laserCloudIn->points[i].x;
      thisPoint.y = laserCloudIn->points[i].y;
      thisPoint.z = laserCloudIn->points[i].z;
      thisPoint.intensity = laserCloudIn->points[i].intensity;

      float range = pointDistance(thisPoint);
      if (range < lidarMinRange || range > lidarMaxRange) continue;

      int rowIdn(0);

      float vertical_angle = std::asin(thisPoint.z / range);
      int scanID = static_cast<int>((vertical_angle + 15 * M_PI / 180) /
                                    (2 * M_PI / 180));

      if (scanID < 0 || scanID >= 16) {
        continue;
      }

      rowIdn = scanID;
      //   rowIdn = laserCloudIn->points[i].ring;  // demo

      if (rowIdn < 0 || rowIdn >= N_SCAN) continue;  // vlp-16: N_SCAN 16
      if (rowIdn % downsampleRate != 0) continue;
      //每个点对应的列的id, lidar从lidar-frame的x轴负方向顺时针开始(0->>1800)
      float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
      static float ang_res_x =
          360.0 / float(Horizon_SCAN);  // vlp-16: Horizon_SCAN 1800
      int columnIdn =
          -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
      if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;
      if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;
      if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;

      if (use_demo) {
        // Velodyne  校正当前帧的每个点 运动补偿
        thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
      }

      // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t
      // / 1000000000.0); // Ouster

      // lidar-msg---->matrix
      rangeMat.at<float>(rowIdn, columnIdn) = range;  //每一行即为一个激光束的点

      int index = columnIdn + rowIdn * Horizon_SCAN;
      // fullCloud中同一激光束的点索引连续在一起
      fullCloud->points[index] =
          thisPoint;  // matirx中序列计算的index重新存储当前点(即同一点与laserCloudIn中index不一样了)
    }
    LOG(INFO) << "fullCloud''s size:" << fullCloud->size();
  }

  void cloudExtraction() {
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i)  // 0->15
    {
      cloudInfo.startRingIndex[i] = count - 1 + 5;

      for (int j = 0; j < Horizon_SCAN; ++j) {
        if (rangeMat.at<float>(i, j) != FLT_MAX) {
          // mark the points' column index for marking occlusion later
          cloudInfo.pointColInd[count] = j;  //存储每个点列的index
          // save range info
          cloudInfo.pointRange[count] =
              rangeMat.at<float>(i, j);  //存储每个点的range
          // save extracted cloud
          extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
          // size of extracted cloud
          ++count;
        }
      }
      cloudInfo.endRingIndex[i] = count - 1 - 5;
    }
  }

  void publishClouds() {
    cloudInfo.header = cloudHeader;
    // pub: lio_sam/deskew/cloud_deskewed 同一激光束的点索引连续在一起
    cloudInfo.cloud_deskewed = publishCloud(&pubExtractedCloud, extractedCloud,
                                            cloudHeader.stamp, lidarFrame);
    // pub: lio_sam/deskew/cloud_info
    pubLaserCloudInfo.publish(cloudInfo);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam");

  ImageProjection IP;

  ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

  ros::MultiThreadedSpinner spinner(3);
  spinner.spin();

  return 0;
}