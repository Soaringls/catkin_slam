#pragma once

#include "const_variables.h"
#include "data_center.h"
#include "timer.h"

namespace ceres_loam {
double MINIMUM_RANGE = 0.3;
int N_SCANS = 16;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }

class CloudSegmentation {
 public:
  std::shared_ptr<CloudSegmentation> CloudSegmentationPtr;
  std::shared_ptr<const CloudSegmentation> CloudSegmentationConstPtr;

 public:
  CloudSegmentation();
  ~CloudSegmentation() = default;

 private:
  void allocateMemory();
  void resetParameters();

  // image projection
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();

  // segmentation

  template <typename PointT>
  void removeClosedPointCloud(const typename pcl::PointCloud<PointT> &cloud_in,
                              typename pcl::PointCloud<PointT> &cloud_out,
                              float thres);
  void labelComponents(int row, int col);
  void classifyEdgePlanar();

 public:
  void CloudHandler(const PointCloudPtr cloud, const double stamp);

 private:
  double timestamp;
  PointCloudPtr laserCloudIn;
  pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

  // projected velodyne raw cloud, but saved in the form of 1-D matrix
  PointCloudPtr fullCloud;
  // same as fullCloud, but with intensity - range
  PointCloudPtr fullInfoCloud;

  PointCloudPtr groundCloud;
  PointCloudPtr segmentedCloud;
  PointCloudPtr segmentedCloudPure;
  PointCloudPtr outlierCloud;

  PointType nanPoint;  // fill in fullCloud at each iteration

  cv::Mat rangeMat;   // range matrix for range image
  cv::Mat labelMat;   // label matrix for segmentaiton marking
  cv::Mat groundMat;  // ground matrix for ground cloud marking
  int labelCount;

  float startOrientation;
  float endOrientation;
  // neighbor iterator for segmentaiton process
  std::vector<std::pair<int8_t, int8_t> > neighborIterator;

  uint16_t *allPushedIndX;  // array for tracking points of a segmented object
  uint16_t *allPushedIndY;

  uint16_t *queueIndX;  // array for breadth-first search process of
                        // segmentation, for speed
  uint16_t *queueIndY;
};

}  // namespace ceres_loam
