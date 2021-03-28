/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 4/13/18.
//

#ifndef LIO_PRIORFACTOR_H_
#define LIO_PRIORFACTOR_H_

#include <ceres/ceres.h>
#include "imu_processor/IntegrationBase.h"
#include "utils/math_utils.h"

namespace lio {

using namespace mathutils;

class PriorFactor : public ceres::SizedCostFunction<6, 7> {

 public:
  PriorFactor() = delete;
  PriorFactor(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot) : pos_{pos}, rot_{rot} {}

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  void Check(double const *const *parameters);

  Eigen::Vector3d pos_;
  Eigen::Quaterniond rot_;

};

}

#endif //LIO_PRIORFACTOR_H_
