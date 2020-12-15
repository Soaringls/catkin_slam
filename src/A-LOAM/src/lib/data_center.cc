#include "data_center.h"
namespace ceres_loam {
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;

struct ScanSegMsg {
  double timestamp;
  PointCloudPtr laserCloud;
  PointCloudPtr cornerPointsSharp;
  PointCloudPtr cornerPointsLessSharp;
  PointCloudPtr surfPointsFlat;
  PointCloudPtr surfPointsLessFlat;
  ScanSegMsg(const double time, const PointCloudPtr cloud1,
             const PointCloudPtr cloud2, const PointCloudPtr cloud3,
             const PointCloudPtr cloud4)
      : timestamp(time),
        laserCloud(cloud1),
        cornerPointsSharp(cloud2),
        cornerPointsLessSharp(cloud3),
        surfPointsLessFlat(cloud4) {}
};

class DataCenter {
 public:
  std::shared_ptr<DataCenter> DataCenterPtr;
  std::shared_ptr<const DataCenter> DataCenterConstPtr;

  DataCenter() = default;
  DataCenter(const DataCenter& rhs) = delete;
  DataCenter& operator=(const DataCenter& rhs) = delete;

 public:
  static DataCenterPtr Instance() {
    static DataCenterPtr instance(new DataCenter);
    return instance;
  }

 private:
  // projection
  std::deque<PointCloudPtr> segmented_clouds;
  // segmentation
  std::deque<PointCloudPtr> segmented_clouds;
  std::deque<PointCloudPtr> segmented_clouds;
  std::deque<PointCloudPtr> segmented_clouds;
  std::deque<PointCloudPtr> segmented_clouds;
};
}  // namespace ceres_loam