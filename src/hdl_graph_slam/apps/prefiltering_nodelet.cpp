#include <ros/ros.h>
#include <ros/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace hdl_graph_slam {

class PrefilteringNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;

  PrefilteringNodelet() {}
  virtual ~PrefilteringNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    private_nh = getPrivateNodeHandle();

    initialize_params();
    std::string topic = private_nh.param<std::string>("topic_name", "/velodyne_points");
    ROS_INFO_STREAM("PrefilteringNodelet:topic_name: "<<topic);
    points_sub = nh.subscribe(topic, 64, &PrefilteringNodelet::cloud_callback, this);
    points_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 32);
  }

private:
  void initialize_params() {
    std::string downsample_method = private_nh.param<std::string>("downsample_method", "VOXELGRID");
    double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);

    if(downsample_method == "VOXELGRID") {
      std::cout << "downsample: VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
      voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = voxelgrid;
    } else if(downsample_method == "APPROX_VOXELGRID") {
      std::cout << "downsample: APPROX_VOXELGRID " << downsample_resolution << std::endl;
      boost::shared_ptr<pcl::ApproximateVoxelGrid<PointT>> approx_voxelgrid(new pcl::ApproximateVoxelGrid<PointT>());
      approx_voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
      downsample_filter = approx_voxelgrid;
    } else {
      if(downsample_method != "NONE") {
        std::cerr << "warning: unknown downsampling type (" << downsample_method << ")" << std::endl;
        std::cerr << "       : use passthrough filter" <<std::endl;
      }
      std::cout << "downsample: NONE" << std::endl;
    }

    std::string outlier_removal_method = private_nh.param<std::string>("outlier_removal_method", "STATISTICAL");
    if(outlier_removal_method == "STATISTICAL") {
      int mean_k = private_nh.param<int>("statistical_mean_k", 20);
      double stddev_mul_thresh = private_nh.param<double>("statistical_stddev", 1.0);
      std::cout << "outlier_removal: STATISTICAL " << mean_k << " - " << stddev_mul_thresh << std::endl;

      pcl::StatisticalOutlierRemoval<PointT>::Ptr sor(new pcl::StatisticalOutlierRemoval<PointT>());
      sor->setMeanK(mean_k);
      sor->setStddevMulThresh(stddev_mul_thresh);
      outlier_removal_filter = sor;
    } else if(outlier_removal_method == "RADIUS") {
      double radius = private_nh.param<double>("radius_radius", 0.8);
      int min_neighbors = private_nh.param<int>("radius_min_neighbors", 2);
      std::cout << "outlier_removal: RADIUS " << radius << " - " << min_neighbors << std::endl;

      pcl::RadiusOutlierRemoval<PointT>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT>());
      rad->setRadiusSearch(radius);
      rad->setMinNeighborsInRadius(min_neighbors);
      outlier_removal_filter = rad;
    } else {
      std::cout << "outlier_removal: NONE" << std::endl;
    }

    use_distance_filter = private_nh.param<bool>("use_distance_filter", true); 
    use_filter_lane = private_nh.param<bool>("use_filter_lane",true);
    distance_near_thresh = private_nh.param<double>("distance_near_thresh", 1.0);
    distance_far_thresh = private_nh.param<double>("distance_far_thresh", 100.0);

    base_link_frame = private_nh.param<std::string>("base_link_frame", "");
  }

  void cloud_callback(pcl::PointCloud<PointT>::ConstPtr src_cloud) {
    static int i=0;
    ROS_INFO_STREAM("frame id-----------------: "<<i); i++;
    if(src_cloud->empty()) {
      return;
    }

    //if(!base_link_frame.empty()) {
    //  if(!tf_listener.canTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0))) {
    //    std::cerr << "failed to find transform between " << base_link_frame << " and " << src_cloud->header.frame_id << std::endl;
    //  }
//
    //  tf::StampedTransform transform;
    //  tf_listener.waitForTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
    //  tf_listener.lookupTransform(base_link_frame, src_cloud->header.frame_id, ros::Time(0), transform);
//
    //  pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
    //  pcl_ros::transformPointCloud(*src_cloud, *transformed, transform);
    //  transformed->header.frame_id = base_link_frame;
    //  transformed->header.stamp = src_cloud->header.stamp;
    //  src_cloud = transformed;
    //}

    pcl::PointCloud<PointT>::ConstPtr filtered = distance_filter(src_cloud);
    filtered = downsample(filtered);
    filtered = outlier_removal(filtered);

    //lvs
    //method 1
    float dist = 0, z_threshold = 0;
    PointT pt;
    for(std::size_t i = 0; i < filtered->points.size(); )
    {
      pt = filtered->points[i];
      if (pt.y < 0.1 && pt.y > -0.1)
      {
        dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if(z_threshold > pt.z && dist < 5)
           z_threshold = pt.z;
      }
      i=i+2;
    }

    pcl::PointCloud<PointT>::Ptr filtered_ground(new pcl::PointCloud<PointT>);
    //remove ground plane
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_threshold-2, z_threshold+0.25);
    pass.setFilterLimitsNegative(true);
    pass.filter(*filtered_ground);

    filtered_ground->width = filtered_ground->size();
    filtered_ground->height = 1;
    filtered_ground->is_dense = false;
    filtered_ground->header = filtered->header; 
    //remove car
    pcl::PointCloud<PointT>::Ptr filtered_clean(new pcl::PointCloud<PointT>);
    for(std::size_t i = 0; i < filtered_ground->points.size(); ++i)
    {
      pt = filtered_ground->points[i];
      dist = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if(dist > 3.3)
      {
          filtered_clean->push_back(pt);
      }
    }
    filtered_clean->width = filtered_clean->size();
    filtered_clean->height = 1;
    filtered_clean->is_dense = false;
    filtered_clean->header = filtered->header; 
    // std::cout<<"filtered size:        "<<filtered->points.size()<<std::endl; static int k =0;k++;
    // std::cout<<"filtered_clean size:  "<<filtered_clean->points.size()<<std::endl;
    // std::cout<<"************************************************************* "<<k<<std::endl;
    //end lvs
    use_filter_lane ? points_pub.publish(filtered_clean) : points_pub.publish(filtered);
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr outlier_removal(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!outlier_removal_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    outlier_removal_filter->setInputCloud(cloud);
    outlier_removal_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  pcl::PointCloud<PointT>::ConstPtr distance_filter(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    filtered->reserve(cloud->size());

    std::copy_if(cloud->begin(), cloud->end(), std::back_inserter(filtered->points),
      [&](const PointT& p) {
        double d = p.getVector3fMap().norm();
        return d > distance_near_thresh && d < distance_far_thresh;
      }
    );

    filtered->width = filtered->size();
    filtered->height = 1;
    filtered->is_dense = false;

    filtered->header = cloud->header;

    return filtered;
  }

private:
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  ros::Subscriber points_sub;
  ros::Publisher points_pub;

  tf::TransformListener tf_listener;

  std::string base_link_frame;

  bool use_distance_filter;
  bool use_filter_lane;
  double distance_near_thresh;
  double distance_far_thresh;

  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Filter<PointT>::Ptr outlier_removal_filter;

};

}

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::PrefilteringNodelet, nodelet::Nodelet)
