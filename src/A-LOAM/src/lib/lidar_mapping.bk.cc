#include "lidar_mapping.h"
namespace ceres_loam {

LidarMapping::LidarMapping() {
  allocateMemory();
  laserCloudCenWidth = 10;
  laserCloudCenHeight = 10;
  laserCloudCenDepth = 5;

  double opti_params[7] = {0, 0, 0, 1, 0, 0, 0};
  memcpy(parameters, opti_params, sizeof(opti_params));

  q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
  t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

  double lineRes = 0.2;
  downSizeFilterCorner.setLeafSize(lineRes, lineRes, lineRes);
  double planeRes = 0.4;
  downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);
  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
    laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
  }

  laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());

  // kd-tree
  kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
  kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

  LOG(INFO) << "construct success!";
}

void LidarMapping::allocateMemory() {
  laserCloudCornerLast.reset(new PointCloud);
  laserCloudSurfLast.reset(new PointCloud);
  laserCloudFullRes.reset(new PointCloud);
  laserCloudMap.reset(new PointCloud);
}

void LidarMapping::pointAssociateToMap(PointType const *const pi,
                                       PointType *const po) {
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;

  // po->intensity = 1.0;
}

void LidarMapping::accessData() {
  auto msgs = DataCenter::Instance()->GetScanSegMsg();
  laserCloudCornerLast->clear();
  laserCloudCornerLast = msgs->cornerPointsLessSharp;

  laserCloudSurfLast->clear();
  laserCloudSurfLast = msgs->surfPointsLessFlat;

  laserCloudFullRes->clear();
  laserCloudFullRes = msgs->laserCloud;
  t_wodom_curr = msgs->odom_to_init_t;
  q_wodom_curr = msgs->odom_to_init_q;
}

void LidarMapping::Process() {
  accessData();
  static int frame_cnt(0);
  LOG(INFO) << std::fixed << std::setprecision(6)
            << "Mapping-debug begin process t_w_curr:" << t_w_curr.x() << ", "
            << t_w_curr.y() << ", " << t_w_curr.z()
            << "  q_w_curr(wxyz):" << q_w_curr.w() << ", " << q_w_curr.x()
            << ", " << q_w_curr.y() << ", " << q_w_curr.z();
  LOG(INFO) << std::fixed << std::setprecision(6)
            << "Mapping-debug begin process laser-od:" << t_wodom_curr.x()
            << ", " << t_wodom_curr.y() << ", " << t_wodom_curr.z()
            << "  laser-od(wxyz):" << q_wodom_curr.w() << ", "
            << q_wodom_curr.x() << ", " << q_wodom_curr.y() << ", "
            << q_wodom_curr.z();
  LOG(INFO) << std::fixed << std::setprecision(6)
            << "Mapping-debug begin process wmap_wodo:" << t_wmap_wodom.x()
            << ", " << t_wmap_wodom.y() << ", " << t_wmap_wodom.z()
            << "  wmap_wodo(wxyz):" << q_wmap_wodom.w() << ", "
            << q_wmap_wodom.x() << ", " << q_wmap_wodom.y() << ", "
            << q_wmap_wodom.z();

  // set initial guess
  q_w_curr = q_wmap_wodom * q_wodom_curr;
  t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;

  int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
  int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
  int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

  if (t_w_curr.x() + 25.0 < 0) centerCubeI--;
  if (t_w_curr.y() + 25.0 < 0) centerCubeJ--;
  if (t_w_curr.z() + 25.0 < 0) centerCubeK--;

  while (centerCubeI < 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int i = laserCloudWidth - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; i >= 1; i--) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i - 1 + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i - 1 + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
                            laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI++;
    laserCloudCenWidth++;
  }

  while (centerCubeI >= laserCloudWidth - 3) {
    for (int j = 0; j < laserCloudHeight; j++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int i = 0;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; i < laserCloudWidth - 1; i++) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + 1 + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + 1 + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
                            laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeI--;
    laserCloudCenWidth--;
  }

  while (centerCubeJ < 3) {
    for (int i = 0; i < laserCloudWidth; i++) {
      for (int k = 0; k < laserCloudDepth; k++) {
        int j = laserCloudHeight - 1;
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; j >= 1; j--) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j - 1) +
                                    laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j - 1) +
                                  laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
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
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; j < laserCloudHeight - 1; j++) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * (j + 1) +
                                    laserCloudWidth * laserCloudHeight * k];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * (j + 1) +
                                  laserCloudWidth * laserCloudHeight * k];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
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
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; k >= 1; k--) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight *
                                        (k - 1)];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * (k - 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
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
        pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
            laserCloudCornerArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * k];
        pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
            laserCloudSurfArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k];
        for (; k < laserCloudDepth - 1; k++) {
          laserCloudCornerArray[i + laserCloudWidth * j +
                                laserCloudWidth * laserCloudHeight * k] =
              laserCloudCornerArray[i + laserCloudWidth * j +
                                    laserCloudWidth * laserCloudHeight *
                                        (k + 1)];
          laserCloudSurfArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
              laserCloudSurfArray[i + laserCloudWidth * j +
                                  laserCloudWidth * laserCloudHeight * (k + 1)];
        }
        laserCloudCornerArray[i + laserCloudWidth * j +
                              laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeCornerPointer;
        laserCloudSurfArray[i + laserCloudWidth * j +
                            laserCloudWidth * laserCloudHeight * k] =
            laserCloudCubeSurfPointer;
        laserCloudCubeCornerPointer->clear();
        laserCloudCubeSurfPointer->clear();
      }
    }

    centerCubeK--;
    laserCloudCenDepth--;
  }

  int laserCloudValidNum = 0;
  int laserCloudSurroundNum = 0;

  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
      for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
        if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight &&
            k >= 0 && k < laserCloudDepth) {
          laserCloudValidInd[laserCloudValidNum] =
              i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          laserCloudValidNum++;
          laserCloudSurroundInd[laserCloudSurroundNum] =
              i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
          laserCloudSurroundNum++;
        }
      }
    }
  }
  //   for (int i = 0; i < laserCloudValidNum; i++) {
  //     LOG(INFO) << "laserCloudCornerArray[laserCloudValidInd[" << i
  //               << "]]:" <<
  //               laserCloudCornerArray[laserCloudValidInd[i]]->size();
  //     LOG(INFO) << "laserCloudSurfArray[laserCloudValidInd[" << i
  //               << "]]:" <<
  //               laserCloudSurfArray[laserCloudValidInd[i]]->size();
  //   }
  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  for (int i = 0; i < laserCloudValidNum; i++) {
    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
  }

  int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
  int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(
      new pcl::PointCloud<PointType>());
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerStack);
  int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

  pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(
      new pcl::PointCloud<PointType>());
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfStack);
  int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

  LOG(INFO) << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
  LOG(INFO) << "Mapping regis corner:" << laserCloudCornerFromMap->size()
            << "->" << laserCloudCornerStackNum;
  LOG(INFO) << "Mapping regis surf  :" << laserCloudSurfFromMap->size() << "->"
            << laserCloudSurfStackNum;
  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50) {
    Timer t_opt;
    Timer t_tree;
    kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
    kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
    LOG(INFO) << "build tree time:" << t_tree.end() << " [ms]";

    for (int iterCount = 0; iterCount < 1; iterCount++) {  // test temp 2->1
      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(parameters, 4, q_parameterization);
      problem.AddParameterBlock(parameters + 4, 3);

      Timer t_data;
      int corner_num = 0;

      for (int i = 0; i < laserCloudCornerStackNum;
           i++) {  // test temp laserCloudCornerStackNum->2
        pointOri = laserCloudCornerStack->points[i];
        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                            pointSearchSqDis);
        // LOG(INFO) << std::fixed << std::setprecision(6)
        //           << "Mapping-pointAssociateToMap t_w_curr:" << t_w_curr.x()
        //           << ", " << t_w_curr.y() << ", " << t_w_curr.z()
        //           << "  q_w_curr(wxyz):" << q_w_curr.w() << ", " <<
        //           q_w_curr.x()
        //           << ", " << q_w_curr.y() << ", " << q_w_curr.z();
        // LOG(INFO) << std::fixed << std::setprecision(6) << "Mapping itera["
        // << i
        //           << "]:pointOri(" << pointOri.x << ", " << pointOri.y << ",
        //           "
        //           << pointOri.z << ")";
        // LOG(INFO) << std::fixed << std::setprecision(6) << "Mapping itera["
        // << i
        //           << "]:pointSel(" << pointSel.x << ", " << pointSel.y << ",
        //           "
        //           << pointSel.z << "), pointSearchInd:" <<
        //           pointSearchInd.size()
        //           << ", pointSearchSqDis:" << pointSearchSqDis.size()
        //           << ", pointSearchSqDis[4]:" << pointSearchSqDis[4];
        if (pointSearchSqDis[4] < 1.0) {
          std::vector<Eigen::Vector3d> nearCorners;
          Eigen::Vector3d center(0, 0, 0);
          for (int j = 0; j < 5; j++) {
            Eigen::Vector3d tmp(
                laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
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
          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
          if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;
            // LOG(INFO) << std::fixed << std::setprecision(6) << "Mapping
            // itera["
            //           << i << "]:curr_point(" << curr_point.x() << ", "
            //           << curr_point.y() << "),  last_point_a(" << point_a.x()
            //           << ", " << point_a.y() << "),  last_point_b("
            //           << point_b.x() << ", " << point_b.y() << ")";
            ceres::CostFunction *cost_function =
                LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, parameters,
                                     parameters + 4);
            corner_num++;
          }
        }
      }

      int surf_num = 0;
      for (int i = 0; i < laserCloudSurfStackNum; i++) {
        pointOri = laserCloudSurfStack->points[i];

        pointAssociateToMap(&pointOri, &pointSel);
        kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd,
                                          pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 =
            -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0) {
          for (int j = 0; j < 5; j++) {
            matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
            matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
            matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
          }
          // find the norm of plane
          Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
          double negative_OA_dot_norm = 1 / norm.norm();
          norm.normalize();

          // Here n(pa, pb, pc) is unit norm of plane
          bool planeValid = true;
          for (int j = 0; j < 5; j++) {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) *
                         laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                     norm(1) *
                         laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                     norm(2) *
                         laserCloudSurfFromMap->points[pointSearchInd[j]].z +
                     negative_OA_dot_norm) > 0.2) {
              planeValid = false;
              break;
            }
          }
          Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
          if (planeValid) {
            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(
                curr_point, norm, negative_OA_dot_norm);
            problem.AddResidualBlock(cost_function, loss_function, parameters,
                                     parameters + 4);
            surf_num++;
          }
        }
      }
      LOG(INFO) << "mapping data assosiation time:" << t_data.end() << " [ms]";

      Timer t_solver;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      LOG(INFO) << "mapping solver time:" << t_solver.end() << " [ms]";
    }

    LOG(INFO) << "mapping optimization time:" << t_opt.end() << " [ms]";
  } else {
    LOG(INFO) << "time Map corner and surf num are not enough";
  }

  // transformUpdate();
  t_w_curr = Eigen::Vector3d(parameters[4], parameters[5], parameters[6]);
  q_w_curr = Eigen::Quaterniond(parameters[3], parameters[0], parameters[1],
                                parameters[2]);
  q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
  t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
  LOG(INFO) << "transformUpdate's t_wmap_wodom:" << t_wmap_wodom.x() << ", "
            << t_wmap_wodom.y() << ", " << t_wmap_wodom.z()
            << "  q_wmap_wodom(wxyz):" << q_wmap_wodom.w() << ", "
            << q_wmap_wodom.x() << ", " << q_wmap_wodom.y() << ", "
            << q_wmap_wodom.z();
  Timer t_add;
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
        cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ +
                    laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudCornerArray[cubeInd]->push_back(pointSel);
    }
  }
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth && cubeJ >= 0 &&
        cubeJ < laserCloudHeight && cubeK >= 0 && cubeK < laserCloudDepth) {
      int cubeInd = cubeI + laserCloudWidth * cubeJ +
                    laserCloudWidth * laserCloudHeight * cubeK;
      laserCloudSurfArray[cubeInd]->push_back(pointSel);
    }
  }
  LOG(INFO) << "add points time:" << t_add.end() << " [ms]";
  // current
  Timer t_filter;
  for (int i = 0; i < laserCloudValidNum; i++) {
    int ind = laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
    downSizeFilterCorner.filter(*tmpCorner);
    laserCloudCornerArray[ind] = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
    downSizeFilterSurf.filter(*tmpSurf);
    laserCloudSurfArray[ind] = tmpSurf;
  }
  LOG(INFO) << "filter time:" << t_filter.end() << " [ms]";

  LOG(INFO) << std::fixed << std::setprecision(6)
            << "Mapping-debug t_w_curr:" << t_w_curr.x() << ", " << t_w_curr.y()
            << ", " << t_w_curr.z() << "  q_w_curr(wxyz):" << q_w_curr.w()
            << ", " << q_w_curr.x() << ", " << q_w_curr.y() << ", "
            << q_w_curr.z();
  if (frame_cnt % 20 == 0) {
    laserCloudMap->clear();
    for (int i = 0; i < 4851; i++) {
      *laserCloudMap += *laserCloudCornerArray[i];
      *laserCloudMap += *laserCloudSurfArray[i];
    }
  }
  frame_cnt++;
}
}  // namespace ceres_loam