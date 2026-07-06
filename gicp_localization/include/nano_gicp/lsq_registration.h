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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

namespace nano_gicp {

enum class LSQ_OPTIMIZER_TYPE { GaussNewton, LevenbergMarquardt };

inline Eigen::Matrix3f skew(const Eigen::Vector3f& x) {
  Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
  skew(0, 1) = -x[2];
  skew(0, 2) = x[1];
  skew(1, 0) = x[2];
  skew(1, 2) = -x[0];
  skew(2, 0) = -x[1];
  skew(2, 1) = x[0];

  return skew;
}

inline Eigen::Matrix3d skewd(const Eigen::Vector3d& x) {
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew(0, 1) = -x[2];
  skew(0, 2) = x[1];
  skew(1, 0) = x[2];
  skew(1, 2) = -x[0];
  skew(2, 0) = -x[1];
  skew(2, 1) = x[0];

  return skew;
}

inline Eigen::Quaterniond so3_exp(const Eigen::Vector3d& omega) {
  double theta_sq = omega.dot(omega);

  double theta;
  double imag_factor;
  double real_factor;
  if(theta_sq < 1e-10) {
    theta = 0;
    double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
    real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
  } else {
    theta = std::sqrt(theta_sq);
    double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(), imag_factor * omega.z());
}

template<typename PointSource, typename PointTarget>
class LsqRegistration : public pcl::Registration<PointSource, PointTarget, float> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::nr_iterations_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::converged_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LsqRegistration();
  virtual ~LsqRegistration();

  void setRotationEpsilon(double eps);
  void setTransformationEpsilon(double eps);
  void setMaximumIterations(int iter);
  void setInitialLambdaFactor(double init_lambda_factor);
  void setDebugPrint(bool lm_debug_print);

  const Eigen::Matrix<double, 6, 6>& getFinalHessian() const;
  double getFinalError() const;

  // ---- Ground-vehicle constraints (yaw-defect fix, 2026-07-05) -------------
  // The LM/GN tangent is d = [omega; t] with omega a WORLD-frame rotation
  // (left-multiplicative update x0 = exp(d) * x0). Rotation matrices are
  // center-independent, so masking omega components constrains the pose's
  // ATTITUDE change regardless of the rotation-about-origin parameterization.
  //
  // setDoFMask(fix_roll, fix_pitch, fix_yaw): masked rotation axes never move
  // from the initial guess — 4-DoF registration (fix roll+pitch) or 3-DoF
  // attitude-locked registration (fix all three). Translation is always free.
  void setDoFMask(bool fix_roll, bool fix_pitch, bool fix_yaw) {
    dof_fix_[0] = fix_roll;
    dof_fix_[1] = fix_pitch;
    dof_fix_[2] = fix_yaw;
  }
  // Soft attitude prior: adds e^T W e to the objective with
  // e = Log(R_x0 * R_target^T) (world tangent, matches d) and
  // W = diag(info) [rad^-2]. Zero info disables per axis. The prior is
  // included in both the normal equations AND the LM error, so rho stays
  // consistent. Calibrate info against the published marginal yaw stiffness
  // (gicp/localization/debug/yaw_marginal_stiffness).
  void setRotationPrior(const Eigen::Matrix3d& R_target, const Eigen::Vector3d& info) {
    rot_prior_target_ = R_target;
    rot_prior_info_ = info;
    rot_prior_set_ = info.maxCoeff() > 0.0;
  }
  void clearRotationPrior() { rot_prior_set_ = false; }

  virtual void swapSourceAndTarget() {}
  virtual void clearSource() {}
  virtual void clearTarget() {}

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  bool is_converged(const Eigen::Isometry3d& delta) const;

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H = nullptr, Eigen::Matrix<double, 6, 1>* b = nullptr) = 0;
  virtual double compute_error(const Eigen::Isometry3d& trans) = 0;

  bool step_optimize(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);
  bool step_gn(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);
  bool step_lm(Eigen::Isometry3d& x0, Eigen::Isometry3d& delta);

protected:
  double rotation_epsilon_;
  double transformation_epsilon_;
  int max_iterations_;


  LSQ_OPTIMIZER_TYPE lsq_optimizer_type_;
  int lm_max_iterations_;
  double lm_init_lambda_factor_;
  double lm_lambda_;
  bool lm_debug_print_;

  Eigen::Matrix<double, 6, 6> final_hessian_;
  double final_error_;

  // Ground-vehicle constraints (see setDoFMask / setRotationPrior above).
  bool dof_fix_[3] = {false, false, false};
  bool rot_prior_set_ = false;
  Eigen::Matrix3d rot_prior_target_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d rot_prior_info_ = Eigen::Vector3d::Zero();

  // Apply the soft rotation prior to (y, H, b) at pose x0, and the DoF mask to
  // (H, b). Shared by step_gn / step_lm. Returns the prior's error term.
  double apply_constraints(const Eigen::Isometry3d& x0,
                           Eigen::Matrix<double, 6, 6>* H,
                           Eigen::Matrix<double, 6, 1>* b) const;
  // Prior error contribution only (for LM's compute_error consistency).
  double prior_error(const Eigen::Isometry3d& x0) const;
};
}  // namespace nano_gicp
