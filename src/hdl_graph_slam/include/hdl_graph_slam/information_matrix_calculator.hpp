#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_graph_slam {

class InformationMatrixCalculator {
public:
  using PointT = pcl::PointXYZI;

  InformationMatrixCalculator(){}
  InformationMatrixCalculator(ros::NodeHandle& nh);
  ~InformationMatrixCalculator();

  template<typename ParamServer>
  void load(ParamServer& params) {
    use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
    const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
    const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

    var_gain_a = params.template param<double>("var_gain_a", 20.0);
    min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
    max_stddev_x = params.template param<double>("max_stddev_x", 5.0);
    min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
    max_stddev_q = params.template param<double>("max_stddev_q", 0.2);
    fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
  }

  static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, //current
                                   const pcl::PointCloud<PointT>::ConstPtr& cloud2, //pre
                                   const Eigen::Isometry3d& relpose,                //pre在current中坐标
                                   double max_range = std::numeric_limits<double>::max());

  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, //current
                                          const pcl::PointCloud<PointT>::ConstPtr& cloud2, //pre
                                          const Eigen::Isometry3d& relpose) const;         //pre在current中坐标
private:
  double weight(double var_gain_a,     //var_gain_a
                double fitness_score_thresh, //fitness_score_thresh
                double min_var, //min_var_x  min_var_q
                double max_var, //max_var_x  max_var_q
                double fitness_score) const {//fitness_score,相邻2关键帧之间匹配得分
    double y = (1.0 - std::exp(-var_gain_a * fitness_score)) / (1.0 - std::exp(-var_gain_a * fitness_score_thresh));
    return min_var + (max_var - min_var) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_x;
  double const_stddev_q;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;

};

}

#endif // INFORMATION_MATRIX_CALCULATOR_HPP
