/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP

#include <cstdlib>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <boost/bind.hpp>

#include "../mtk/build_manifold.hpp"
#include "../mtk/startIdx.hpp"
#include "../mtk/types/S2.hpp"
#include "../mtk/types/SOn.hpp"
#include "../mtk/types/vect.hpp"
#include "util.hpp"

//#define USE_sparse

namespace esekfom {

using namespace Eigen;

// used for iterated error state EKF update
// for the aim to calculate  measurement (z), estimate measurement (h), partial
// differention matrices (h_x, h_v) and the noise covariance (R) at the same
// time, by only one function. applied for measurement as a manifold.
template <typename S, typename M, int measurement_noise_dof = M::DOF>
struct share_datastruct {
  bool valid;
  bool converge;
  M z;
  Eigen::Matrix<typename S::scalar, M::DOF, measurement_noise_dof> h_v;
  Eigen::Matrix<typename S::scalar, M::DOF, S::DOF> h_x;
  Eigen::Matrix<typename S::scalar, measurement_noise_dof,
                measurement_noise_dof>
      R;
};

// used for iterated error state EKF update
// for the aim to calculate  measurement (z), estimate measurement (h), partial
// differention matrices (h_x, h_v) and the noise covariance (R) at the same
// time, by only one function. applied for measurement as an Eigen matrix whose
// dimension is changing
template <typename T>
struct dyn_share_datastruct {
  bool valid;
  bool converge;
  Eigen::Matrix<T, Eigen::Dynamic, 1> z;
  Eigen::Matrix<T, Eigen::Dynamic, 1> h;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_v;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
};

// used for iterated error state EKF update
// for the aim to calculate  measurement (z), estimate measurement (h), partial
// differention matrices (h_x, h_v) and the noise covariance (R) at the same
// time, by only one function. applied for measurement as a dynamic manifold
// whose dimension or type is changing
template <typename T>
struct dyn_runtime_share_datastruct {
  bool valid;
  bool converge;
  // Z z;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_v;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> h_x;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> R;
};

template <typename state, int process_noise_dof/*=12*/, typename input = state,
          typename measurement = state, int measurement_noise_dof = 0>
class esekf {
  typedef esekf self;
  enum { n = state::DOF, m = state::DIM, l = measurement::DOF };

 public:
  typedef typename state::scalar scalar_type;
  typedef Matrix<scalar_type, n, n> cov;
  typedef Matrix<scalar_type, m, n> cov_;
  typedef SparseMatrix<scalar_type> spMt;
  typedef Matrix<scalar_type, n, 1> vectorized_state;
  typedef Matrix<scalar_type, m, 1> flatted_state;
  typedef flatted_state processModel(state &, const input &);
  typedef Eigen::Matrix<scalar_type, m, n> processMatrix1(state &,
                                                          const input &);
  typedef Eigen::Matrix<scalar_type, m, process_noise_dof> processMatrix2(
      state &, const input &);
  typedef Eigen::Matrix<scalar_type, process_noise_dof, process_noise_dof>
      processnoisecovariance;
  typedef measurement measurementModel(state &, bool &);
  typedef measurement measurementModel_share(
      state &, share_datastruct<state, measurement, measurement_noise_dof> &);
  typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> measurementModel_dyn(
      state &, bool &); 
  typedef void measurementModel_dyn_share(state &,
                                          dyn_share_datastruct<scalar_type> &);
  typedef Eigen::Matrix<scalar_type, l, n> measurementMatrix1(state &, bool &);
  typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, n> measurementMatrix1_dyn(
      state &, bool &);
  typedef Eigen::Matrix<scalar_type, l, measurement_noise_dof>
  measurementMatrix2(state &, bool &);
  typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>
  measurementMatrix2_dyn(state &, bool &);
  typedef Eigen::Matrix<scalar_type, measurement_noise_dof,
                        measurement_noise_dof>
      measurementnoisecovariance;
  typedef Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>
      measurementnoisecovariance_dyn;

  esekf(const state &x = state(), const cov &P = cov::Identity())
      : x_(x), P_(P) {
#ifdef USE_sparse
    SparseMatrix<scalar_type> ref(n, n);
    ref.setIdentity();
    l_ = ref;
    f_x_2 = ref;
    f_x_1 = ref;
#endif
  };

  /*all useless interface*/
  {
    //*all useless init interface*/
    {
      // receive system-specific models and their differentions.
      // for measurement as a manifold.
      /********** useless default  **********/
      void init(processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in,
                measurementModel h_in, measurementMatrix1 h_x_in,
                measurementMatrix2 h_v_in, int maximum_iteration,
                scalar_type limit_vector[n]) {
        f = f_in;
        f_x = f_x_in;
        f_w = f_w_in;
        h = h_in;
        h_x = h_x_in;
        h_v = h_v_in;

        maximum_iter = maximum_iteration;
        for (int i = 0; i < n; i++) {
          limit[i] = limit_vector[i];
        }

        x_.build_S2_state();
        x_.build_SO3_state();
        x_.build_vect_state();
      }

      // receive system-specific models and their differentions.
      // for measurement as an Eigen matrix whose dimention is chaing.
      /********** useless default  **********/
      void init_dyn(processModel f_in, processMatrix1 f_x_in,
                    processMatrix2 f_w_in, measurementModel_dyn h_in,
                    measurementMatrix1_dyn h_x_in,
                    measurementMatrix2_dyn h_v_in, int maximum_iteration,
                    scalar_type limit_vector[n]) {
        f = f_in;
        f_x = f_x_in;
        f_w = f_w_in;
        h_dyn = h_in;
        h_x_dyn = h_x_in;
        h_v_dyn = h_v_in;

        maximum_iter = maximum_iteration;
        for (int i = 0; i < n; i++) {
          limit[i] = limit_vector[i];
        }

        x_.build_S2_state();
        x_.build_SO3_state();
        x_.build_vect_state();
      }

      // receive system-specific models and their differentions.
      // for measurement as a dynamic manifold whose dimension or type is
      // changing.
      /********** useless default  **********/
      void init_dyn_runtime(
          processModel f_in, processMatrix1 f_x_in, processMatrix2 f_w_in,
          measurementMatrix1_dyn h_x_in, measurementMatrix2_dyn h_v_in,
          int maximum_iteration, scalar_type limit_vector[n]) {
        f = f_in;
        f_x = f_x_in;
        f_w = f_w_in;
        h_x_dyn = h_x_in;
        h_v_dyn = h_v_in;

        maximum_iter = maximum_iteration;
        for (int i = 0; i < n; i++) {
          limit[i] = limit_vector[i];
        }

        x_.build_S2_state();
        x_.build_SO3_state();
        x_.build_vect_state();
      }

      // receive system-specific models and their differentions
      // for measurement as a manifold.
      // calculate  measurement (z), estimate measurement (h), partial
      // differention matrices (h_x, h_v) and the noise covariance (R) at the
      // same time, by only one function (h_share_in).
      /********** useless default  **********/
      void init_share(processModel f_in, processMatrix1 f_x_in,
                      processMatrix2 f_w_in, measurementModel_share h_share_in,
                      int maximum_iteration, scalar_type limit_vector[n]) {
        f = f_in;
        f_x = f_x_in;
        f_w = f_w_in;
        h_share = h_share_in;

        maximum_iter = maximum_iteration;
        for (int i = 0; i < n; i++) {
          limit[i] = limit_vector[i];
        }

        x_.build_S2_state();
        x_.build_SO3_state();
        x_.build_vect_state();
      }
    }

    // receive system-specific models and their differentions
    // for measurement as a dynamic manifold whose dimension  or type is
    // changing. calculate  measurement (z), estimate measurement (h), partial
    // differention matrices (h_x, h_v) and the noise covariance (R) at the same
    // time, by only one function (h_dyn_share_in). for any scenarios where it
    // is needed
    /********** useless default  **********/
    void init_dyn_runtime_share(processModel f_in, processMatrix1 f_x_in,
                                processMatrix2 f_w_in, int maximum_iteration,
                                scalar_type limit_vector[n]) {}
    // iterated error state EKF update for measurement as a manifold.
    /********** useless default  **********/
    void update_iterated(measurement & z, measurementnoisecovariance & R) {}
    // iterated error state EKF update for measurement as a manifold.
    // calculate measurement (z), estimate measurement (h), partial differention
    // matrices (h_x, h_v) and the noise covariance (R) at the same time, by
    // only one function.
    /********** useless default  **********/
    void update_iterated_share() {}
    // iterated error state EKF update for measurement as an Eigen matrix whose
    // dimension is changing.
    /********** useless default  **********/
    void update_iterated_dyn(Eigen::Matrix<scalar_type, Eigen::Dynamic, 1> z,
                             measurementnoisecovariance_dyn R) {}
    // iterated error state EKF update for measurement as an Eigen matrix whose
    // dimension is changing. calculate measurement (z), estimate measurement
    // (h), partial differention matrices (h_x, h_v) and the noise covariance
    // (R) at the same time, by only one function.
    /********** useless default  **********/
    void update_iterated_dyn_share() {}

    // iterated error state EKF update for measurement as a dynamic manifold,
    // whose dimension or type is changing. the measurement and the measurement
    // model are received in a dynamic manner.
    /********** useless default  **********/
    template <typename measurement_runtime, typename measurementModel_runtime>
    void update_iterated_dyn_runtime(measurement_runtime z,
                                     measurementnoisecovariance_dyn R,
                                     measurementModel_runtime h_runtime) {}

    // iterated error state EKF update for measurement as a dynamic manifold,
    // whose dimension or type is changing. the measurement and the measurement
    // model are received in a dynamic manner. calculate measurement (z),
    // estimate measurement (h), partial differention matrices (h_x, h_v) and
    // the noise covariance (R) at the same time, by only one function.
    /********** useless default  **********/
    template <typename measurement_runtime,
              typename measurementModel_dyn_runtime_share>
    void update_iterated_dyn_runtime_share(
        measurement_runtime z, measurementModel_dyn_runtime_share h) {}
  }

  // iterated error state EKF propogation
  /********** use in IMU_Processing.hpp **********/
  void predict(double &dt, processnoisecovariance &Q, const input &i_in) {
    flatted_state f_ = f(x_, i_in);
    cov_ f_x_ = f_x(x_, i_in);
    cov f_x_final;

    Matrix<scalar_type, m, process_noise_dof> f_w_ = f_w(x_, i_in);
    Matrix<scalar_type, n, process_noise_dof> f_w_final;
    state x_before = x_;
    x_.oplus(f_, dt);

    F_x1 = cov::Identity();
    /*************** loop 1 of x_.vect_state************/
    for (std::vector<std::pair<std::pair<int, int>, int>>::iterator it = x_.vect_state.begin();it != x_.vect_state.end(); it++) {
      int idx = (*it).first.first;
      int dim = (*it).first.second;
      int dof = (*it).second;
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < dof; j++) {
          f_x_final(idx + j, i) = f_x_(dim + j, i);
        }
      }
      for (int i = 0; i < process_noise_dof/*=12*/; i++) {
        for (int j = 0; j < dof; j++) {
          f_w_final(idx + j, i) = f_w_(dim + j, i);
        }
      }
    } /*------------------- loop 1 end of x_.vect_state-------------------*/
    Matrix<scalar_type, 3, 3> res_temp_SO3;
    MTK::vect<3, scalar_type> seg_SO3;
    /*************** loop 2 of x_.SO3_state************/
    for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin();it != x_.SO3_state.end(); it++) {
      int idx = (*it).first;
      int dim = (*it).second;
      for (int i = 0; i < 3; i++) {
        seg_SO3(i) = -1 * f_(dim + i) * dt;
      }
      MTK::SO3<scalar_type> res;//quaternion <----seg_SO3
      res.w() = MTK::exp<scalar_type, 3>(res.vec(), seg_SO3/*3x1*/, scalar_type(1 / 2));
#ifdef USE_sparse
      res_temp_SO3 = res.toRotationMatrix();
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          f_x_1.coeffRef(idx + i, idx + j) = res_temp_SO3(i, j);
        }
      }
#else
      F_x1.template block<3, 3>(idx, idx) = res.toRotationMatrix();//current SO3, 3X3
#endif
      res_temp_SO3 = MTK::A_matrix(seg_SO3);
      for (int i = 0; i < n; i++) {
        f_x_final.template block<3, 1>(idx, i) =
            res_temp_SO3 * (f_x_.template block<3, 1>(dim, i));
      }
      for (int i = 0; i < process_noise_dof; i++) {
        f_w_final/*matrix: nx12*/.template block<3, 1>(idx, i) =
            res_temp_SO3 * (f_w_.template block<3, 1>(dim, i));
      }
    }/*------------------- loop 2 end  of x_.SO3_state-------------------*/

    Matrix<scalar_type, 2, 3> res_temp_S2;
    Matrix<scalar_type, 2, 2> res_temp_S2_;
    MTK::vect<3, scalar_type> seg_S2;
    /*************** loop 3 of x_.S2_state************/
    for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
      int idx = (*it).first;
      int dim = (*it).second;
      for (int i = 0; i < 3; i++) {
        seg_S2(i) = f_(dim + i) * dt;
      }
      MTK::vect<2, scalar_type> vec = MTK::vect<2, scalar_type>::Zero();
      MTK::SO3<scalar_type> res;
      res.w() = MTK::exp<scalar_type, 3>(res.vec(), seg_S2, scalar_type(1 / 2));
      Eigen::Matrix<scalar_type, 2, 3> Nx;
      Eigen::Matrix<scalar_type, 3, 2> Mx;
      x_.S2_Nx_yy(Nx, idx);
      x_before.S2_Mx(Mx, vec, idx);
#ifdef USE_sparse
      res_temp_S2_ = Nx * res.toRotationMatrix() * Mx;
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
          f_x_1.coeffRef(idx + i, idx + j) = res_temp_S2_(i, j);
        }
      }
#else
      F_x1.template block<2, 2>(idx, idx) = Nx * res.toRotationMatrix() * Mx;
#endif

      Eigen::Matrix<scalar_type, 3, 3> x_before_hat;
      x_before.S2_hat(x_before_hat, idx);
      res_temp_S2 = -Nx * res.toRotationMatrix() * x_before_hat *
                    MTK::A_matrix(seg_S2).transpose();

      for (int i = 0; i < n; i++) {
        f_x_final.template block<2, 1>(idx, i) =
            res_temp_S2 * (f_x_.template block<3, 1>(dim, i));
      }
      for (int i = 0; i < process_noise_dof; i++) {
        f_w_final.template block<2, 1>(idx, i) =
            res_temp_S2 * (f_w_.template block<3, 1>(dim, i));
      }
    }/*------------------- loop 3 end  of x_.S2_state-------------------*/

#ifdef USE_sparse
    f_x_1.makeCompressed();
    spMt f_x2 = f_x_final.sparseView();
    spMt f_w1 = f_w_final.sparseView();
    spMt xp = f_x_1 + f_x2 * dt;
    P_ = xp * P_ * xp.transpose() + (f_w1 * dt) * Q * (f_w1 * dt).transpose();
#else
    F_x1 += f_x_final * dt;
    //paper formula(8) precidt P
    P_ = (F_x1)*P_ * (F_x1).transpose() +
         (dt * f_w_final) * Q * (dt * f_w_final).transpose();
#endif
  }//end of func "predict"

  void change_P(cov &input_cov) { P_ = input_cov; }
  void change_x(state &input_state) {
    x_ = input_state;
    if ((!x_.vect_state.size()) && (!x_.SO3_state.size()) &&
        (!x_.S2_state.size())) {
      x_.build_S2_state();
      x_.build_SO3_state();
      x_.build_vect_state();
    }
  }

  /********** use in IMU_Processing.hpp and laserMapping.cpp **********/
  const state &get_x() const { return x_; }
  const cov &get_P() const { return P_; }

  /********** use in laserMapping.cpp **********/
  // iterated error state EKF update modified for one specific system.
  void update_iterated_dyn_share_modified(double R, double &solve_time) {  // USE_sparse=false
    dyn_share_datastruct<scalar_type> dyn_share;
    dyn_share.valid = true;
    dyn_share.converge = true;
    int t = 0;
    state x_propagated = x_;
    cov P_propagated = P_;
    int dof_Measurement;

    Matrix<scalar_type, n, 1> K_h;
    Matrix<scalar_type, n, n> K_x;

    vectorized_state dx_new = vectorized_state::Zero();
    for (int i = -1; i < maximum_iter;i++) {  // end util update_iterated_dyn_share_modified finish
      dyn_share.valid = true;
      h_dyn_share(x_, dyn_share);

      if (!dyn_share.valid) {
        continue;
      }

      Eigen::Matrix<scalar_type, Eigen::Dynamic/*="num of icp measures"*/, 12> h_x_ = dyn_share.h_x;
      double solve_start = omp_get_wtime();
      dof_Measurement = h_x_.rows();
      vectorized_state dx;
      x_.boxminus(dx, x_propagated);
      dx_new = dx;

      P_ = P_propagated;

      Matrix<scalar_type, 3, 3> res_temp_SO3;
      MTK::vect<3, scalar_type> seg_SO3;
      for (std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin();
                                                      it != x_.SO3_state.end();   it++) {
        int idx = (*it).first;
        int dim = (*it).second;
        for (int i = 0; i < 3; i++) {
          seg_SO3(i) = dx(idx + i);
        }

        res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
        dx_new.template block<3, 1>(idx, 0) =
            res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
        for (int i = 0; i < n; i++) {
          P_.template block<3, 1>(idx, i) =
              res_temp_SO3 * (P_.template block<3, 1>(idx, i));
        }
        for (int i = 0; i < n; i++) {
          P_.template block<1, 3>(i, idx) =
              (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
        }
      }

      Matrix<scalar_type, 2, 2> res_temp_S2;
      MTK::vect<2, scalar_type> seg_S2;
      for (std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
        int idx = (*it).first;
        int dim = (*it).second;
        for (int i = 0; i < 2; i++) {
          seg_S2(i) = dx(idx + i);
        }

        Eigen::Matrix<scalar_type, 2, 3> Nx;
        Eigen::Matrix<scalar_type, 3, 2> Mx;
        x_.S2_Nx_yy(Nx, idx);
        x_propagated.S2_Mx(Mx, seg_S2, idx);
        res_temp_S2 = Nx * Mx;
        dx_new.template block<2, 1>(idx, 0) =
            res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
        for (int i = 0; i < n; i++) {
          P_.template block<2, 1>(idx, i) =
              res_temp_S2 * (P_.template block<2, 1>(idx, i));
        }
        for (int i = 0; i < n; i++) {
          P_.template block<1, 2>(i, idx) =
              (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
        }
      }

      if (n > dof_Measurement) {
        Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur =
            Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(
                dof_Measurement, n);
        h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_;

        Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_ =
            P_ * h_x_cur.transpose() *
            (h_x_cur * P_ * h_x_cur.transpose() / R +
             Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement,
                                                               dof_Measurement))
                .inverse() /
            R;
        K_h = K_ * dyn_share.h;
        K_x = K_ * h_x_cur;
      } else {
        cov P_temp = (P_ / R).inverse();
        // Eigen::Matrix<scalar_type, 12, Eigen::Dynamic> h_T =
        // h_x_.transpose();
        Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_;
        P_temp.template block<12, 12>(0, 0) += HTH;
        cov P_inv = P_temp.inverse();
        // std::cout << "line 1781" << std::endl;
        K_h =
            P_inv.template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h;
        // cov HTH_cur = cov::Zero();
        // HTH_cur. template block<12, 12>(0, 0) = HTH;
        K_x.setZero();  // = cov::Zero();
        K_x.template block<n, 12>(0, 0) =
            P_inv.template block<n, 12>(0, 0) * HTH;
      }

      // K_x = K_ * h_x_;
      Matrix<scalar_type, n, 1> dx_ =
          K_h + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
      state x_before = x_;
      x_.boxplus(dx_);
      dyn_share.converge = true;
      for (int i = 0; i < n; i++) {
        if (std::fabs(dx_[i]) > limit[i]) {
          dyn_share.converge = false;
          break;
        }
      }
      if (dyn_share.converge) t++;

      if (!t && i == maximum_iter - 2) {
        dyn_share.converge = true;
      }

      if (t > 1 || i == maximum_iter - 1) {
        L_ = P_;
        Matrix<scalar_type, 3, 3> res_temp_SO3;
        MTK::vect<3, scalar_type> seg_SO3;
        for (typename std::vector<std::pair<int, int>>::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
          int idx = (*it).first;
          for (int i = 0; i < 3; i++) {
            seg_SO3(i) = dx_(i + idx);
          }
          res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
          for (int i = 0; i < n; i++) {
            L_.template block<3, 1>(idx, i) =
                res_temp_SO3 * (P_.template block<3, 1>(idx, i));
          }
          for (int i = 0; i < 12; i++) {
            K_x.template block<3, 1>(idx, i) =
                res_temp_SO3 * (K_x.template block<3, 1>(idx, i));
          }
          for (int i = 0; i < n; i++) {
            L_.template block<1, 3>(i, idx) =
                (L_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
            P_.template block<1, 3>(i, idx) =
                (P_.template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
          }
        }

        Matrix<scalar_type, 2, 2> res_temp_S2;
        MTK::vect<2, scalar_type> seg_S2;
        for (typename std::vector<std::pair<int, int>>::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
          int idx = (*it).first;

          for (int i = 0; i < 2; i++) {
            seg_S2(i) = dx_(i + idx);
          }

          Eigen::Matrix<scalar_type, 2, 3> Nx;
          Eigen::Matrix<scalar_type, 3, 2> Mx;
          x_.S2_Nx_yy(Nx, idx);
          x_propagated.S2_Mx(Mx, seg_S2, idx);
          res_temp_S2 = Nx * Mx;
          for (int i = 0; i < n; i++) {
            L_.template block<2, 1>(idx, i) =
                res_temp_S2 * (P_.template block<2, 1>(idx, i));
          }
          for (int i = 0; i < 12; i++) {
            K_x.template block<2, 1>(idx, i) =
                res_temp_S2 * (K_x.template block<2, 1>(idx, i));
          }
          for (int i = 0; i < n; i++) {
            L_.template block<1, 2>(i, idx) =
                (L_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
            P_.template block<1, 2>(i, idx) =
                (P_.template block<1, 2>(i, idx)) * res_temp_S2.transpose();
          }
        }
        P_ = L_ -
             K_x.template block<n, 12>(0, 0) * P_.template block<12, n>(0, 0);
        solve_time += omp_get_wtime() - solve_start;
        return;
      }
      solve_time += omp_get_wtime() - solve_start;
    }
  }

  // receive system-specific models and their differentions
  // for measurement as an Eigen matrix whose dimension is changing.
  // calculate  measurement (z), estimate measurement (h), partial differention
  // matrices (h_x, h_v) and the noise covariance (R) at the same time, by only
  // one function (h_dyn_share_in).
  void init_dyn_share(processModel f_in, 
                      processMatrix1 f_x_in,
                      processMatrix2 f_w_in,
                      measurementModel_dyn_share h_dyn_share_in,
                      int maximum_iteration, scalar_type limit_vector[n]) {
    f = f_in;
    f_x = f_x_in;
    f_w = f_w_in;
    h_dyn_share = h_dyn_share_in; /*icp-matching func "h_share_model" in
                                     "laserMapping.cpp"*/

    maximum_iter = maximum_iteration;
    for (int i = 0; i < n; i++) {
      limit[i] = limit_vector[i];
    }

    x_.build_S2_state();
    x_.build_SO3_state();
    x_.build_vect_state();
  }

 private:
  state x_;
  measurement m_;
  cov P_;
  spMt l_;
  spMt f_x_1;
  spMt f_x_2;
  cov F_x1 = cov::Identity();
  cov F_x2 = cov::Identity();
  cov L_ = cov::Identity();

  processModel *f;
  processMatrix1 *f_x;
  processMatrix2 *f_w;

  measurementModel *init_dyn_share;
  measurementMatrix1 *h_x;
  measurementMatrix2 *h_v;

  measurementModel_dyn *h_dyn;
  measurementMatrix1_dyn *h_x_dyn;
  measurementMatrix2_dyn *h_v_dyn;

  measurementModel_share *h_share;
  measurementModel_dyn_share *h_dyn_share;

  int maximum_iter = 0;
  scalar_type limit[n];

  template <typename T>
  T check_safe_update(T _temp_vec) {
    T temp_vec = _temp_vec;
    if (std::isnan(temp_vec(0, 0))) {
      temp_vec.setZero();
      return temp_vec;
    }
    double angular_dis = temp_vec.block(0, 0, 3, 1).norm() * 57.3;
    double pos_dis = temp_vec.block(3, 0, 3, 1).norm();
    if (angular_dis >= 20 || pos_dis > 1) {
      printf("Angular dis = %.2f, pos dis = %.2f\r\n", angular_dis, pos_dis);
      temp_vec.setZero();
    }
    return temp_vec;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace esekfom

#endif  //  ESEKFOM_EKF_HPP
