/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

/***********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, SMRT-AIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#include "dlio/dlio.h"
#include "nano_gicp/lsq_registration.h"

template class nano_gicp::LsqRegistration<PointType, PointType>;

namespace nano_gicp {

template <typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::LsqRegistration() {
  this->reg_name_ = "LsqRegistration";
  max_iterations_ = 64;
  rotation_epsilon_ = 2e-3;
  transformation_epsilon_ = 5e-4;

  lsq_optimizer_type_ = LSQ_OPTIMIZER_TYPE::LevenbergMarquardt;
  lm_debug_print_ = false;
  lm_max_iterations_ = 10;
  lm_init_lambda_factor_ = 1e-9;
  lm_lambda_ = -1.0;

  final_hessian_.setIdentity();
  final_error_ = 0.;
}

template <typename PointTarget, typename PointSource>
LsqRegistration<PointTarget, PointSource>::~LsqRegistration() {}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setRotationEpsilon(double eps) {
  rotation_epsilon_ = eps;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setTransformationEpsilon(double eps) {
  transformation_epsilon_ = eps;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setMaximumIterations(int iter) {
  max_iterations_ = iter;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setInitialLambdaFactor(double init_lambda_factor) {
  lm_init_lambda_factor_ = init_lambda_factor;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::setDebugPrint(bool lm_debug_print) {
  lm_debug_print_ = lm_debug_print;
}

template <typename PointTarget, typename PointSource>
const Eigen::Matrix<double, 6, 6>& LsqRegistration<PointTarget, PointSource>::getFinalHessian() const {
  return final_hessian_;
}

template <typename PointTarget, typename PointSource>
double LsqRegistration<PointTarget, PointSource>::getFinalError() const {
  return final_error_;
}

template <typename PointTarget, typename PointSource>
void LsqRegistration<PointTarget, PointSource>::computeTransformation(PointCloudSource& output, const Matrix4& guess) {
  Eigen::Isometry3d x0 = Eigen::Isometry3d(guess.template cast<double>());

  lm_lambda_ = -1.0;
  converged_ = false;

  if (lm_debug_print_) {
    std::cout << "********************************************" << std::endl;
    std::cout << "***************** optimize *****************" << std::endl;
    std::cout << "********************************************" << std::endl;
  }

  for (int i = 0; i < max_iterations_ && !converged_; i++) {
    nr_iterations_ = i;

    Eigen::Isometry3d delta;
    if (!step_optimize(x0, delta)) {
      std::cerr << "lm not converged!!" << std::endl;
      break;
    }

    converged_ = is_converged(delta);
  }

  final_transformation_ = x0.cast<float>().matrix();
  // The aligned-cloud output is unused by gicp_localization (no aligned-cloud
  // topic); skip the unconditional pcl::transformPointCloud that PCL's default
  // would do. Pose still available via getFinalTransformation().
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::is_converged(const Eigen::Isometry3d& delta) const {
  double accum = 0.0;
  Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = delta.translation();

  Eigen::Matrix3d r_delta = 1.0 / rotation_epsilon_ * R.array().abs();
  Eigen::Vector3d t_delta = 1.0 / transformation_epsilon_ * t.array().abs();

  return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}


template <typename PointTarget, typename PointSource>
double LsqRegistration<PointTarget, PointSource>::prior_error(const Eigen::Isometry3d& x0) const {
  if (!rot_prior_set_) return 0.0;
  const Eigen::Matrix3d R_err = x0.linear() * rot_prior_target_.transpose();
  const Eigen::AngleAxisd aa(R_err);
  const Eigen::Vector3d e = aa.angle() * aa.axis();
  return e.dot(rot_prior_info_.asDiagonal() * e);
}

template <typename PointTarget, typename PointSource>
double LsqRegistration<PointTarget, PointSource>::apply_constraints(const Eigen::Isometry3d& x0,
                                                                    Eigen::Matrix<double, 6, 6>* H,
                                                                    Eigen::Matrix<double, 6, 1>* b) const {
  double y_prior = 0.0;
  // Soft attitude prior (yaw-defect fix): e = Log(R_x0 * R_target^T) lives in
  // the same left/world tangent as d.head<3>(), so J_e ~ I for small e and the
  // Gauss-Newton contribution is H_rot += W, b_rot += W * e. Included in the
  // returned error so LM's rho ratio sees the same objective it solves.
  if (rot_prior_set_) {
    const Eigen::Matrix3d R_err = x0.linear() * rot_prior_target_.transpose();
    const Eigen::AngleAxisd aa(R_err);
    const Eigen::Vector3d e = aa.angle() * aa.axis();
    const Eigen::Matrix3d W = rot_prior_info_.asDiagonal();
    H->template block<3, 3>(0, 0) += W;
    b->template head<3>() += W * e;
    y_prior = e.dot(W * e);
  }
  // Hard DoF mask (4-DoF / 3-DoF registration): zero the masked rotation rows
  // and columns and pin the diagonal at the unmasked scale, so the solve
  // yields exactly d(i) = 0 — the attitude axis never moves from the initial
  // guess. Done AFTER the prior so a masked axis is fully fixed regardless of
  // prior settings. Diagonal pinned at max|diag| (not 1.0) so the reported
  // final hessian's spectrum stays in range for the downstream degeneracy
  // analysis ("externally constrained" reads as a stiff axis, not a null one).
  if (dof_fix_[0] || dof_fix_[1] || dof_fix_[2]) {
    const double pin = std::max(1.0, H->diagonal().array().abs().maxCoeff());
    for (int i = 0; i < 3; ++i) {
      if (!dof_fix_[i]) continue;
      H->row(i).setZero();
      H->col(i).setZero();
      (*H)(i, i) = pin;
      (*b)(i) = 0.0;
    }
  }
  return y_prior;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_optimize(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  switch (lsq_optimizer_type_) {
    case LSQ_OPTIMIZER_TYPE::LevenbergMarquardt:
      return step_lm(x0, delta);
    case LSQ_OPTIMIZER_TYPE::GaussNewton:
      return step_gn(x0, delta);
  }

  return step_lm(x0, delta);
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_gn(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;
  double y0 = linearize(x0, &H, &b);
  // [REVIEW FIX 2026-07-08 P1] Keep the RAW LiDAR-geometry hessian for the
  // consumer-facing final_hessian_: apply_constraints injects the DoF-mask
  // pinning and rotation-prior information, which would otherwise leak
  // artificial stiffness into the degeneracy / yaw-stiffness diagnostics.
  const Eigen::Matrix<double, 6, 6> H_raw = H;
  y0 += apply_constraints(x0, &H, &b);

  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);
  Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

  delta.setIdentity();
  delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
  delta.translation() = d.tail<3>();

  x0 = delta * x0;
  final_hessian_ = H_raw;  // raw LiDAR geometry (see comment above)
  final_error_ = y0;

  return true;
}

template <typename PointTarget, typename PointSource>
bool LsqRegistration<PointTarget, PointSource>::step_lm(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta) {
  Eigen::Matrix<double, 6, 6> H;
  Eigen::Matrix<double, 6, 1> b;
  double y0 = linearize(x0, &H, &b);
  // [REVIEW FIX 2026-07-08 P1] Same raw-hessian capture as step_gn: the
  // augmented system drives the LM solve only; diagnostics get pure geometry.
  const Eigen::Matrix<double, 6, 6> H_raw = H;
  y0 += apply_constraints(x0, &H, &b);

  if (lm_lambda_ < 0.0) {
    lm_lambda_ = lm_init_lambda_factor_ * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < lm_max_iterations_; i++) {
    Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H + lm_lambda_ * Eigen::Matrix<double, 6, 6>::Identity());
    Eigen::Matrix<double, 6, 1> d = solver.solve(-b);

    delta.setIdentity();
    delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
    delta.translation() = d.tail<3>();

    Eigen::Isometry3d xi = delta * x0;
    double yi = compute_error(xi) + prior_error(xi);
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b));

    if (lm_debug_print_) {
      if (i == 0) {
        std::cout << boost::format("--- LM optimization ---\n%5s %15s %15s %15s %15s %15s %5s\n") % "i" % "y0" % "yi" % "rho" % "lambda" % "|delta|" % "dec";
      }
      char dec = rho > 0.0 ? 'x' : ' ';
      std::cout << boost::format("%5d %15g %15g %15g %15g %15g %5c") % i % y0 % yi % rho % lm_lambda_ % d.norm() % dec << std::endl;
    }

    if (rho < 0) {
      if (is_converged(delta)) {
        return true;
      }

      lm_lambda_ = nu * lm_lambda_;
      nu = 2 * nu;
      continue;
    }

    x0 = xi;
    lm_lambda_ = lm_lambda_ * std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    // NOTE (2026-07-10 port): H_raw is the PRE-step linearization point (the
    // x0 this step departed from, not the accepted xi) — a known staleness
    // gap vs GICP_plusplus, which re-linearizes at the final pose.
    final_hessian_ = H_raw;  // raw LiDAR geometry (see comment above)
    final_error_ = yi;
    return true;
  }

  return false;
}

}  // namespace nano_gicp
