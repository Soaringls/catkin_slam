#include "cloud_segmentation.h"

namespace ceres_loam {

CloudSegmentation::CloudSegmentation() {
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();
  nanPoint.intensity = -1;

  allocateMemory();
  resetParameters();

  LOG(INFO) << "construct success!";
}

void CloudSegmentation::allocateMemory() {
  laserCloudIn.reset(new PointCloud());
  laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

  fullCloud.reset(new PointCloud());
  fullInfoCloud.reset(new PointCloud());

  groundCloud.reset(new PointCloud());
  segmentedCloud.reset(new PointCloud());
  segmentedCloudPure.reset(new PointCloud());
  outlierCloud.reset(new PointCloud());

  fullCloud->points.resize(N_SCAN * Horizon_SCAN);
  fullInfoCloud->points.resize(N_SCAN * Horizon_SCAN);

  std::pair<int8_t, int8_t> neighbor;
  neighbor.first = -1;
  neighbor.second = 0;
  neighborIterator.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = 1;
  neighborIterator.push_back(neighbor);
  neighbor.first = 0;
  neighbor.second = -1;
  neighborIterator.push_back(neighbor);
  neighbor.first = 1;
  neighbor.second = 0;
  neighborIterator.push_back(neighbor);

  allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
  allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

  queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
  queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];
}

void CloudSegmentation::resetParameters() {
  timestamp = 0.0;
  laserCloudIn->clear();
  groundCloud->clear();
  segmentedCloud->clear();
  segmentedCloudPure->clear();
  outlierCloud->clear();

  rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
  groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
  labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
  labelCount = 1;

  std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
  std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(),
            nanPoint);
}

void CloudSegmentation::projectPointCloud() {
  // range image projection
  float verticalAngle, horizonAngle, range;
  size_t rowIdn, columnIdn, index, cloudSize;
  PointType thisPoint;

  cloudSize = laserCloudIn->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    thisPoint.x = laserCloudIn->points[i].x;
    thisPoint.y = laserCloudIn->points[i].y;
    thisPoint.z = laserCloudIn->points[i].z;
    // find the row and column index in the iamge for this point
    if (useCloudRing == true) {
      rowIdn = laserCloudInRing->points[i].ring;
    } else {
      verticalAngle = std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x +
                                                   thisPoint.y * thisPoint.y)) *
                      180 / M_PI;
      rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
    }
    if (rowIdn < 0 || rowIdn >= N_SCAN) continue;

    horizonAngle = std::atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

    columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
    if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;

    if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

    range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y +
                 thisPoint.z * thisPoint.z);
    if (range < sensorMinimumRange) continue;

    rangeMat.at<float>(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    index = columnIdn + rowIdn * Horizon_SCAN;
    fullCloud->points[index] = thisPoint;
    fullInfoCloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    fullInfoCloud->points[index].intensity = range;
  }
}
void CloudSegmentation::groundRemoval() {
  size_t lowerInd, upperInd;
  float diffX, diffY, diffZ, angle;
  // groundMat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (size_t j = 0; j < Horizon_SCAN; ++j) {     // Horizon_SCAN: 1800
    for (size_t i = 0; i < groundScanInd; ++i) {  // groundScanInd: 7

      lowerInd = j + (i)*Horizon_SCAN;
      upperInd = j + (i + 1) * Horizon_SCAN;

      if (fullCloud->points[lowerInd].intensity == -1 ||
          fullCloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        groundMat.at<int8_t>(i, j) = -1;
        continue;
      }

      diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
      diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
      diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

      angle =
          std::atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;
      // sensorMountAngle:
      // 0.0,地面点的简单判别，通过查询相邻环上的两个点俯仰角是否超过10度来判定。
      if (std::abs(angle - sensorMountAngle) <= 10) {
        groundMat.at<int8_t>(i, j) = 1;
        groundMat.at<int8_t>(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (groundMat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~N_SCAN-1, need rangeMat
  // for mark label matrix for the 16th scan
  for (size_t i = 0; i < N_SCAN; ++i) {
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
      if (groundMat.at<int8_t>(i, j) == 1 ||
          rangeMat.at<float>(i, j) == FLT_MAX) {
        labelMat.at<int>(i, j) = -1;
      }
    }
  }
  // add ground points
  // for (size_t i = 0; i <= groundScanInd; ++i) {
  //   for (size_t j = 0; j < Horizon_SCAN; ++j) {
  //     if (groundMat.at<int8_t>(i, j) == 1)
  //       groundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
  //   }
  // }
}
void CloudSegmentation::labelComponents(int row, int col) {
  // use std::queue std::vector std::deque will slow the program down greatly
  float d1, d2, alpha, angle;
  int fromIndX, fromIndY, thisIndX, thisIndY;
  bool lineCountFlag[N_SCAN] = {false};

  queueIndX[0] = row;
  queueIndY[0] = col;
  int queueSize = 1;
  int queueStartInd = 0;
  int queueEndInd = 1;

  allPushedIndX[0] = row;
  allPushedIndY[0] = col;
  int allPushedIndSize = 1;

  while (queueSize > 0) {
    // Pop point
    fromIndX = queueIndX[queueStartInd];
    fromIndY = queueIndY[queueStartInd];
    --queueSize;
    ++queueStartInd;
    // Mark popped point
    labelMat.at<int>(fromIndX, fromIndY) = labelCount;
    // Loop through all the neighboring grids of popped grid
    for (auto iter = neighborIterator.begin(); iter != neighborIterator.end();
         ++iter) {
      // new index
      thisIndX = fromIndX + (*iter).first;
      thisIndY = fromIndY + (*iter).second;
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= N_SCAN) continue;
      // at range image margin (left or right side)
      if (thisIndY < 0) thisIndY = Horizon_SCAN - 1;
      if (thisIndY >= Horizon_SCAN) thisIndY = 0;
      // prevent infinite loop (caused by put already examined point back)
      if (labelMat.at<int>(thisIndX, thisIndY) != 0) continue;

      d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                    rangeMat.at<float>(thisIndX, thisIndY));
      d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                    rangeMat.at<float>(thisIndX, thisIndY));

      if ((*iter).first == 0)
        alpha = segmentAlphaX;
      else
        alpha = segmentAlphaY;

      angle = std::atan2(d2 * std::sin(alpha), (d1 - d2 * std::cos(alpha)));

      if (angle > segmentTheta) {
        queueIndX[queueEndInd] = thisIndX;
        queueIndY[queueEndInd] = thisIndY;
        ++queueSize;
        ++queueEndInd;

        labelMat.at<int>(thisIndX, thisIndY) = labelCount;
        lineCountFlag[thisIndX] = true;

        allPushedIndX[allPushedIndSize] = thisIndX;
        allPushedIndY[allPushedIndSize] = thisIndY;
        ++allPushedIndSize;
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (allPushedIndSize >= 30)
    feasibleSegment = true;
  else if (allPushedIndSize >= segmentValidPointNum) {
    int lineCount = 0;
    for (size_t i = 0; i < N_SCAN; ++i)
      if (lineCountFlag[i] == true) ++lineCount;
    if (lineCount >= segmentValidLineNum) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++labelCount;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < allPushedIndSize; ++i) {
      labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
    }
  }
}
void CloudSegmentation::cloudSegmentation() {
  // segmentation process
  for (size_t i = 0; i < N_SCAN; ++i)
    for (size_t j = 0; j < Horizon_SCAN; ++j)
      if (labelMat.at<int>(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (size_t i = 0; i < N_SCAN; ++i) {
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
      if (labelMat.at<int>(i, j) > 0 || groundMat.at<int8_t>(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (labelMat.at<int>(i, j) == 999999) {
          if (i > groundScanInd && j % 5 == 0) {  // groundScanInd: 7
            outlierCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (groundMat.at<int8_t>(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < Horizon_SCAN - 5) continue;
        }

        // save seg cloud
        segmentedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }
  }
}

template <typename PointT>
void CloudSegmentation::removeClosedPointCloud(
    const typename pcl::PointCloud<PointT>& cloud_in,
    typename pcl::PointCloud<PointT>& cloud_out, float thres) {
  if (&cloud_in != &cloud_out) {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize(cloud_in.points.size());
  }

  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size(); ++i) {
    if (cloud_in.points[i].x * cloud_in.points[i].x +
            cloud_in.points[i].y * cloud_in.points[i].y +
            cloud_in.points[i].z * cloud_in.points[i].z <
        thres * thres)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size()) {
    cloud_out.points.resize(j);
  }

  cloud_out.height = 1;
  cloud_out.width = static_cast<uint32_t>(j);
  cloud_out.is_dense = true;
}
template void CloudSegmentation::removeClosedPointCloud<pcl::PointXYZ>(
    const typename pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    typename pcl::PointCloud<pcl::PointXYZ>& cloud_out, float thres);
template void CloudSegmentation::removeClosedPointCloud<pcl::PointXYZI>(
    const typename pcl::PointCloud<pcl::PointXYZI>& cloud_in,
    typename pcl::PointCloud<pcl::PointXYZI>& cloud_out, float thres);

void CloudSegmentation::classifyEdgePlanar() {
  removeClosedPointCloud(*segmentedCloud, *segmentedCloud, MINIMUM_RANGE);

  int cloudSize = (*segmentedCloud).points.size();
  float startOri =
      -std::atan2((*segmentedCloud).points[0].y, (*segmentedCloud).points[0].x);
  float endOri = -std::atan2((*segmentedCloud).points[cloudSize - 1].y,
                             (*segmentedCloud).points[cloudSize - 1].x) +
                 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
  std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
  for (int i = 0; i < cloudSize; i++) {
    point.x = (*segmentedCloud).points[i].x;
    point.y = (*segmentedCloud).points[i].y;
    point.z = (*segmentedCloud).points[i].z;

    float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) *
                  180 / M_PI;
    int scanID = 0;

    if (N_SCANS == 16) {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (N_SCANS == 32) {
      scanID = int((angle + 92.0 / 3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        count--;
        continue;
      }
    } else if (N_SCANS == 64) {
      if (angle >= -8.83)
        scanID = int((2 - angle) * 3.0 + 0.5);
      else
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

      // use [0 50]  > 50 remove outlies
      if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
        count--;
        continue;
      }
    } else {
      LOG(FATAL) << "wrong scan number!!!";
    }

    float ori = -std::atan2(point.y, point.x);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;
      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + scanPeriod * relTime;
    laserCloudScans[scanID].push_back(point);
  }

  cloudSize = count;
  // LOG(INFO) << "points size:" << cloudSize;

  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  PointCloudPtr laserCloud(new PointCloud());
  for (int i = 0; i < N_SCANS; i++) {
    scanStartInd[i] = laserCloud->size() + 5;
    *laserCloud += laserCloudScans[i];
    scanEndInd[i] = laserCloud->size() - 6;
  }

  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x +
                  laserCloud->points[i - 3].x + laserCloud->points[i - 2].x +
                  laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x +
                  laserCloud->points[i + 1].x + laserCloud->points[i + 2].x +
                  laserCloud->points[i + 3].x + laserCloud->points[i + 4].x +
                  laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y +
                  laserCloud->points[i - 3].y + laserCloud->points[i - 2].y +
                  laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y +
                  laserCloud->points[i + 1].y + laserCloud->points[i + 2].y +
                  laserCloud->points[i + 3].y + laserCloud->points[i + 4].y +
                  laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z +
                  laserCloud->points[i - 3].z + laserCloud->points[i - 2].z +
                  laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z +
                  laserCloud->points[i + 1].z + laserCloud->points[i + 2].z +
                  laserCloud->points[i + 3].z + laserCloud->points[i + 4].z +
                  laserCloud->points[i + 5].z;

    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
  }

  PointCloudPtr cornerPointsSharp(new PointCloud());
  PointCloudPtr cornerPointsLessSharp(new PointCloud());
  PointCloudPtr surfPointsFlat(new PointCloud());
  PointCloudPtr surfPointsLessFlat(new PointCloud());

  for (int i = 0; i < N_SCANS; i++) {
    if (scanEndInd[i] - scanStartInd[i] < 6) continue;
    PointCloudPtr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
      int ep =
          scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

      std::sort(cloudSortInd + sp, cloudSortInd + ep + 1, comp);

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1) {
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];

        if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1) {
          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x -
                          laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y -
                          laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z -
                          laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    *surfPointsLessFlat += surfPointsLessFlatScanDS;
  }
  // inject msgs to datacenter
  auto data_center = DataCenter::Instance();
  ScanSegMsgPtr segmsg(new ScanSegMsg(timestamp, laserCloud, cornerPointsSharp,
                                      cornerPointsLessSharp, surfPointsFlat,
                                      surfPointsLessFlat));

  data_center->SetScanSegMsg(segmsg);
}

void CloudSegmentation::CloudHandler(const PointCloudPtr cloud,
                                     const double stamp) {
  Timer elapsed_time;

  timestamp = stamp;
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *laserCloudIn, indices);
  LOG(INFO) << std::fixed << std::setprecision(6) << "stamp:" << stamp
            << ", pts:" << laserCloudIn->size();

  // step. projection
  projectPointCloud();
  // step. Mark ground points
  groundRemoval();
  // step. Point cloud segmentation
  cloudSegmentation();
  // step. segmentation
  classifyEdgePlanar();
  // step. Reset parameters for next iteration
  resetParameters();

  LOG(INFO) << std::fixed << std::setprecision(6)
            << "cloud-segmentation elapsed time:" << elapsed_time.end()
            << " [ms]";
}
}  // namespace ceres_loam
