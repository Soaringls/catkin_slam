#include "lidar_refination.h"
namespace ceres_loam {

LidarRefination::LidarRefination() {
  allocateMemory();
  laserCloudCenWidth = 10;
  laserCloudCenHeight = 10;
  laserCloudCenDepth = 5;

  double opti_params[7] = {0, 0, 0, 1, 0, 0, 0};
  memcpy(parameters, opti_params, sizeof(opti_params));

  q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
  t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  double lineRes = 0.2;
  voxel_filter_corner_.setLeafSize(lineRes, lineRes, lineRes);
  double planeRes = 0.4;
  voxel_filter_surf_.setLeafSize(planeRes, planeRes, planeRes);

  LOG(INFO) << __FUNCTION__ << ":construct success!";
}

void LidarRefination::allocateMemory() {
  feature_scan_corner_.reset(new PointCloud);
  feature_scan_corner_filtered_.reset(new PointCloud);
  feature_scan_surf_.reset(new PointCloud);
  feature_scan_surf_filtered_.reset(new PointCloud);
  cloud_full_res.reset(new PointCloud);

  corner_map_.reset(new PointCloud());
  surf_map_.reset(new PointCloud());
  surround_map_.reset(new PointCloud());

  for (int i = 0; i < laserCloudNum; i++) {  // 4851
    corners_pool[i].reset(new PointCloud());
    surfs_pool[i].reset(new PointCloud());
  }

  // kd-tree
  kdtree_corner_map_.reset(new pcl::KdTreeFLANN<PointType>());
  kdtree_surf_map_.reset(new pcl::KdTreeFLANN<PointType>());
  globis_map.reset(new PointCloud);
}

void LidarRefination::pointAssociateToMap(PointType const *const pi,
                                          PointType *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr_ * point_curr + t_w_curr_;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
}

void LidarRefination::accessData() {
  auto msgs = DataCenter::Instance()->GetScanSegMsg();
  feature_scan_corner_->clear();
  feature_scan_corner_ = msgs->cornerPointsLessSharp;

  feature_scan_surf_->clear();
  feature_scan_surf_ = msgs->surfPointsLessFlat;

  cloud_full_res->clear();
  cloud_full_res = msgs->laserCloud;
  t_wodom_curr = msgs->odom_to_init_t;
  q_wodom_curr = msgs->odom_to_init_q;
}

void LidarRefination::accessAvailableCubicNum(int &scans_valid_num,
                                              int &sub_scan_valid_num) {
  LOG(INFO) << __FUNCTION__ << ":laserCloudCenWidth-H-D1:" << laserCloudCenWidth
            << ", " << laserCloudCenHeight << ", "
            << laserCloudCenDepth;  // 10 10 5
  int centerCubeI = int((t_w_curr_.x() + CubeSize / 2) / CubeSize) +
                    laserCloudCenWidth;  // init 10
  int centerCubeJ = int((t_w_curr_.y() + CubeSize / 2) / CubeSize) +
                    laserCloudCenHeight;  // init 10
  int centerCubeK = int((t_w_curr_.z() + CubeSize / 2) / CubeSize) +
                    laserCloudCenDepth;  // init 5
  // LOG(INFO) << __FUNCTION__ << ":centerCubeI-J-K-1:" << centerCubeI << ", "
  // << centerCubeJ << ", " << centerCubeK;
  if (t_w_curr_.x() + CubeSize / 2 < 0) centerCubeI--;
  if (t_w_curr_.y() + CubeSize / 2 < 0) centerCubeJ--;
  if (t_w_curr_.z() + CubeSize / 2 < 0) centerCubeK--;
  // LOG(INFO) << __FUNCTION__ << ":centerCubeI-J-K-2:" << centerCubeI << ", "
  //           << centerCubeJ << ", " << centerCubeK;
  while (centerCubeI < 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int i = laserCloudWidth - 1;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; i >= 1; i--) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i - 1 + laserCloudWidth * j +
                           laserCloudWidth * laserCloudHeight * k];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i - 1 + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudCubeCornerPointer->clear();

        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeSurfPointer->clear();
      }
    }
    centerCubeI++;
    laserCloudCenWidth++;
  }
  LOG(INFO) << __FUNCTION__ << ":centerCubeI-J-K-3:" << centerCubeI << ", "
            << centerCubeJ << ", " << centerCubeK;

  while (centerCubeI >= laserCloudWidth - 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int i = 0;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; i < laserCloudWidth - 1; i++) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i + 1 + laserCloudWidth * j +
                           laserCloudWidth * laserCloudHeight * k];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i + 1 + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI--;
    laserCloudCenWidth--;
  }
  LOG(INFO) << __FUNCTION__ << ":centerCubeI-J-K-4:" << centerCubeI << ", "
            << centerCubeJ << ", " << centerCubeK;
  while (centerCubeJ < 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int j = laserCloudHeight - 1;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; j >= 1; j--) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i + laserCloudWidth * (j - 1) +
                           laserCloudWidth * laserCloudHeight * k];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i + laserCloudWidth * (j - 1) +
                         laserCloudWidth * laserCloudHeight * k];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ++;
    laserCloudCenHeight++;
  }

  while (centerCubeJ >= laserCloudHeight - 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int j = 0;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; j < laserCloudHeight - 1; j++) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i + laserCloudWidth * (j + 1) +
                           laserCloudWidth * laserCloudHeight * k];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i + laserCloudWidth * (j + 1) +
                         laserCloudWidth * laserCloudHeight * k];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeJ--;
    laserCloudCenHeight--;
  }

  while (centerCubeK < 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int j = 0; j < laserCloudHeight; j++) {
        int k = laserCloudDepth - 1;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; k >= 1; k--) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i + laserCloudWidth * j +
                           laserCloudWidth * laserCloudHeight * (k - 1)];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * (k - 1)];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK++;
    laserCloudCenDepth++;
  }

  while (centerCubeK >= laserCloudDepth - 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int j = 0; j < laserCloudHeight; j++) {
        int k = 0;
        PointCloudPtr laserCloudCubeCornerPointer =
            corners_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * k];
        PointCloudPtr laserCloudCubeSurfPointer =
            surfs_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k];
        for (; k < laserCloudDepth - 1; k++) {
          corners_pool[i + laserCloudWidth * j +
                       laserCloudWidth * laserCloudHeight * k] =
              corners_pool[i + laserCloudWidth * j +
                           laserCloudWidth * laserCloudHeight * (k + 1)];
          surfs_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
              surfs_pool[i + laserCloudWidth * j +
                         laserCloudWidth * laserCloudHeight * (k + 1)];
        }
        corners_pool[i + laserCloudWidth * j +
                     laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        surfs_pool[i + laserCloudWidth * j +
                   laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenDepth--;
  }

  LOG(INFO) << __FUNCTION__
            << ":accessAvailableCubicNum:laserCloudCenWidth-H-D2:"
            << laserCloudCenWidth << ", " << laserCloudCenHeight << ", "
            << laserCloudCenDepth;
  LOG(INFO) << __FUNCTION__ << ":centerCubeI-J-K-5:" << centerCubeI << ", "
            << centerCubeJ << ", " << centerCubeK;

  // for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
  //   for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
  //     for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 1; j <= centerCubeJ + 1; j++) {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
        // 21 21 11
        if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
            k >= 0 && k < laserCloudDepth) {
          scans_valid_indices[scans_valid_num] =
              i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          scans_valid_num++;
          subscans_valid_indices[sub_scan_valid_num] =
              i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          sub_scan_valid_num++;
        }
      }
    }
  }
}

void LidarRefination::Process() {
  Timer elapsed_time;
  accessData();
  // set initial guess
  q_w_curr_ = q_wmap_wodom * q_wodom_curr;
  t_w_curr_ = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
  // prepare data
  int scans_valid_num = 0;
  int subscans_valid_num = 0;
  accessAvailableCubicNum(scans_valid_num, subscans_valid_num);
  LOG(INFO) << __FUNCTION__ << ":scans valid num:" << scans_valid_num
            << ", subscans valid num:" << subscans_valid_num;
  prepareData(subscans_valid_num);
  // calculate refination result
  calculateTransformation();
  // update good pose
  updateOptimizedResult();

  // add feature scan to pool
  // todo: add feature pts by fix distance or angle-threshold
  static int cnt(0);
  if (cnt++ % 30 == 0) {
    addFeatureCloudtoPool();
    downSampleCornerSurfArray(scans_valid_num);
  }

  LOG(INFO) << __FUNCTION__ << std::fixed << std::setprecision(6)
            << ":laser-refination elapsed time:" << elapsed_time.end()
            << " [ms]";
}

void LidarRefination::prepareData(const int featuremap_scans) {
  // update map_corner and map_surf
  corner_map_->clear();
  surf_map_->clear();
  // generate regis-msp, use subscans_valid_num or scans_valid_num
  for (int i = 0; i < featuremap_scans; i++) {
    // *corner_map_ += *corners_pool[scans_valid_indices[i]];
    // *surf_map_ += *surfs_pool[scans_valid_indices[i]];
    int ind = subscans_valid_indices[i];
    *corner_map_ += *corners_pool[ind];
    *surf_map_ += *surfs_pool[ind];
  }

  feature_scan_corner_filtered_->clear();
  voxel_filter_corner_.setInputCloud(feature_scan_corner_);
  voxel_filter_corner_.filter(*feature_scan_corner_filtered_);

  feature_scan_surf_filtered_->clear();
  voxel_filter_surf_.setInputCloud(feature_scan_surf_);
  voxel_filter_surf_.filter(*feature_scan_surf_filtered_);

  LOG(INFO) << __FUNCTION__ << ":regis to cornermap:" << corner_map_->size()
            << "->" << feature_scan_corner_filtered_->points.size();
  LOG(INFO) << __FUNCTION__ << ":regis to surfmap  :" << surf_map_->size()
            << "->" << feature_scan_surf_filtered_->points.size();
}

void LidarRefination::calculateTransformation() {
  int corner_map_num = corner_map_->points.size();
  int surf_map_num = surf_map_->points.size();

  if (corner_map_num > 10 && surf_map_num > 50) {
    Timer t_opt;
    Timer t_tree;
    kdtree_corner_map_->setInputCloud(corner_map_);
    kdtree_surf_map_->setInputCloud(surf_map_);
    LOG(INFO) << __FUNCTION__ << ":build kdtree time:" << t_tree.end()
              << " [ms]";

    for (int iterCount = 0; iterCount < 2; iterCount++) {
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::Problem::Options problem_options;
      ceres::Problem problem(problem_options);

      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      problem.AddParameterBlock(parameters, 4, q_parameterization);
      problem.AddParameterBlock(parameters + 4, 3);

      //   Timer t_data;
      calculateTransformationCorner(problem, loss_function);
      calculateTransformationSurf(problem, loss_function);
      //   LOG(INFO) << __FUNCTION__  " data assosiation time:" << t_data.end()
      //   << " [ms] ";

      Timer t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      LOG(INFO) << __FUNCTION__ << ":refination solver time:" << t_solver.end()
                << " [ms]";
    }

    LOG(INFO) << __FUNCTION__ << ":refination optimization time:" << t_opt.end()
              << " [ms]";
  } else {
    LOG(INFO) << __FUNCTION__ << ":time Map corner and surf num are not enough";
  }
}

void LidarRefination::updateOptimizedResult() {
  // refination global pose, good result
  t_w_curr_ = Eigen::Vector3d(parameters[4], parameters[5], parameters[6]);
  q_w_curr_ = Eigen::Quaterniond(parameters[3], parameters[0], parameters[1],
                                 parameters[2]);
  // correct transform to optimized the global pose of lidar-odom
  q_wmap_wodom = q_w_curr_ * q_wodom_curr.inverse();
  t_wmap_wodom = t_w_curr_ - q_wmap_wodom * t_wodom_curr;
  auto data_center = DataCenter::Instance();
  data_center->SetOdomCorrectPose(t_wmap_wodom, q_wmap_wodom);
}

void LidarRefination::addFeatureCloudtoPool() {
  LOG(INFO) << __FUNCTION__ << ":addFeatureCloudtoPool:laserCloudCenWidth-H-D:"
            << laserCloudCenWidth << ", " << laserCloudCenHeight << ", "
            << laserCloudCenDepth;  // 10 10 5

  // add new points for corners_pool
  PointType point_globis;
  int num_pts = feature_scan_corner_filtered_->points.size();
  for (int i = 0; i < num_pts; i++) {
    pointAssociateToMap(&feature_scan_corner_filtered_->points[i],
                        &point_globis);

    int cubeI =
        int((point_globis.x + CubeSize / 2) / CubeSize) + laserCloudCenWidth;
    int cubeJ =
        int((point_globis.y + CubeSize / 2) / CubeSize) + laserCloudCenHeight;
    int cubeK =
        int((point_globis.z + CubeSize / 2) / CubeSize) + laserCloudCenDepth;

    if (point_globis.x + CubeSize / 2 < 0) cubeI--;
    if (point_globis.y + CubeSize / 2 < 0) cubeJ--;
    if (point_globis.z + CubeSize / 2 < 0) cubeK--;

    // 21 21 11
    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
        cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ +
                    laserCloudWidth * laserCloudHeight * cubeK;
      corners_pool[cubeInd]->push_back(point_globis);
    }
  }
  // add new points for surfs_pool
  num_pts = feature_scan_surf_filtered_->points.size();
  for (int i = 0; i < num_pts; i++) {
    pointAssociateToMap(&feature_scan_surf_filtered_->points[i], &point_globis);

    int cubeI =
        int((point_globis.x + CubeSize / 2) / CubeSize) + laserCloudCenWidth;
    int cubeJ =
        int((point_globis.y + CubeSize / 2) / CubeSize) + laserCloudCenHeight;
    int cubeK =
        int((point_globis.z + CubeSize / 2) / CubeSize) + laserCloudCenDepth;

    if (point_globis.x + CubeSize / 2 < 0) cubeI--;
    if (point_globis.y + CubeSize / 2 < 0) cubeJ--;
    if (point_globis.z + CubeSize / 2 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
        cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ +
                    laserCloudWidth * laserCloudHeight * cubeK;
      surfs_pool[cubeInd]->push_back(point_globis);
    }
  }
}

void LidarRefination::calculateTransformationCorner(
    ceres::Problem &problem, ceres::LossFunction *loss_function) {
  int scan_corner_num = feature_scan_corner_filtered_->points.size();
  PointType point_laser, point_globis;
  for (int i = 0; i < scan_corner_num; i++) {
    point_laser = feature_scan_corner_filtered_->points[i];
    pointAssociateToMap(&point_laser, &point_globis);
    kdtree_corner_map_->nearestKSearch(point_globis, 5, neighbor_pts_indices,
                                       neighbor_pts_dist);

    if (neighbor_pts_dist[4] < 1.0) {
      std::vector<Eigen::Vector3d> nearCorners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(corner_map_->points[neighbor_pts_indices[j]].x,
                            corner_map_->points[neighbor_pts_indices[j]].y,
                            corner_map_->points[neighbor_pts_indices[j]].z);
        center = center + tmp;
        nearCorners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
        covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      // if is indeed line feature
      // note Eigen library sort eigenvalues in increasing order
      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(point_laser.x, point_laser.y, point_laser.z);
      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;

        ceres::CostFunction *cost_function =
            LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
        problem.AddResidualBlock(cost_function, loss_function, parameters,
                                 parameters + 4);
      }
    }
  }
}

void LidarRefination::calculateTransformationSurf(
    ceres::Problem &problem, ceres::LossFunction *loss_function) {
  int scan_surf_num = feature_scan_surf_filtered_->points.size();
  PointType point_laser, point_globis;
  for (int i = 0; i < scan_surf_num; i++) {
    point_laser = feature_scan_surf_filtered_->points[i];

    pointAssociateToMap(&point_laser, &point_globis);
    kdtree_surf_map_->nearestKSearch(point_globis, 5, neighbor_pts_indices,
                                     neighbor_pts_dist);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 =
        -1 * Eigen::Matrix<double, 5, 1>::Ones();
    if (neighbor_pts_dist[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) = surf_map_->points[neighbor_pts_indices[j]].x;
        matA0(j, 1) = surf_map_->points[neighbor_pts_indices[j]].y;
        matA0(j, 2) = surf_map_->points[neighbor_pts_indices[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      // Here n(pa, pb, pc) is unit norm of plane
      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        // if OX * n > 0.2, then plane is not fit well
        if (fabs(norm(0) * surf_map_->points[neighbor_pts_indices[j]].x +
                 norm(1) * surf_map_->points[neighbor_pts_indices[j]].y +
                 norm(2) * surf_map_->points[neighbor_pts_indices[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          planeValid = false;
          break;
        }
      }
      Eigen::Vector3d curr_point(point_laser.x, point_laser.y, point_laser.z);
      if (planeValid) {
        ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(
            curr_point, norm, negative_OA_dot_norm);
        problem.AddResidualBlock(cost_function, loss_function, parameters,
                                 parameters + 4);
      }
    }
  }
}

void LidarRefination::downSampleCornerSurfArray(const int scans_valid_num) {
  for (int i = 0; i < scans_valid_num; i++) {
    int ind = scans_valid_indices[i];

    PointCloudPtr tmpCorner(new PointCloud());
    voxel_filter_corner_.setInputCloud(corners_pool[ind]);
    voxel_filter_corner_.filter(*tmpCorner);
    corners_pool[ind] = tmpCorner;

    PointCloudPtr tmpSurf(new PointCloud());
    voxel_filter_surf_.setInputCloud(surfs_pool[ind]);
    voxel_filter_surf_.filter(*tmpSurf);
    surfs_pool[ind] = tmpSurf;
  }
}

const PointCloudPtr LidarRefination::GenerateWholeMap() {
  static int frame_cnt(0);
  if (frame_cnt++ % 20 == 0) {
    globis_map->clear();
    for (int i = 0; i < laserCloudNum; i++) {
      *globis_map += *corners_pool[i];
      *globis_map += *surfs_pool[i];
    }
  }
  LOG(INFO) << __FUNCTION__ << ":global map's size:" << globis_map->size();
  return globis_map;
}

const PointCloudPtr LidarRefination::GenerateSurroundMap() {
  surround_map_->clear();
  *surround_map_ += *corner_map_;
  *surround_map_ += *surf_map_;
  return surround_map_;
}

}  // namespace ceres_loam