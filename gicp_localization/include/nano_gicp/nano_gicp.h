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

#include "nano_gicp/lsq_registration.h"
#include "nano_gicp/nanoflann_adaptor.h"

namespace nano_gicp {

typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> CovarianceList;

enum class RegularizationMethod { NONE, MIN_EIG, NORMALIZED_MIN_EIG, PLANE, FROBENIUS };

template<typename PointSource, typename PointTarget>
class NanoGICP : public LsqRegistration<PointSource, PointTarget> {
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
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::final_transformation_;

public:
  NanoGICP();
  virtual ~NanoGICP() override;

  void setNumThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setMaxCorrespondenceDistance(double corr);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const std::shared_ptr<const CovarianceList>& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  virtual void setTargetCovariances(const std::shared_ptr<const CovarianceList>& covs);

  virtual void registerInputSource(const PointCloudSourceConstPtr& cloud);
  virtual void registerInputTarget(const PointCloudTargetConstPtr& cloud);

  virtual bool calculateSourceCovariances();
  virtual bool calculateTargetCovariances();

  std::shared_ptr<const CovarianceList> getSourceCovariances() const {
    return source_covs_;
  }

  std::shared_ptr<const CovarianceList> getTargetCovariances() const {
    return target_covs_;
  }

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

  // Faster getFitnessScore that reuses sq_distances_ cached by the last
  // update_correspondences() call (fired from linearize() inside align()).
  // Avoids re-transforming the source and re-querying the target kd-tree —
  // ~10-30 ms/scan against a 9.7M-point map. Returns max double when no
  // fresh cache is available for the current input.
  //
  // Caveats:
  // - Name-shadows pcl::Registration::getFitnessScore (which is non-virtual);
  //   calls through a pcl::Registration<>* still hit the slow base impl.
  // - The cached distances reflect the linearization-point pose used in the
  //   last linearize() call, not the final post-update pose. For converged
  //   scans the delta is sub-mm/sub-mrad and the score is effectively the
  //   final-pose value; in non-converging cases it can lag by one LM step.
  double getFitnessScore(double max_range = std::numeric_limits<double>::max());

  // HONEST fitness recomputed at final_transformation_ (fresh kd-tree pass) —
  // use for gating on non-converged / large-final-step scans where the cached
  // score above lags the applied pose (yaw-defect fix, P2a).
  double getFitnessScoreAtFinal(double max_range = std::numeric_limits<double>::max());

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, const nanoflann::KdTreeFLANN<PointT>& kdtree, CovarianceList& covariances, float& density);

public:
  // Owning, mutable kd-trees. Reusing the same instance across scans keeps
  // nanoflann's internal node pool warm and avoids a make_shared per scan.
  // setInputCloud() rebuilds the index in place.
  std::shared_ptr<nanoflann::KdTreeFLANN<PointSource>> source_kdtree_;
  std::shared_ptr<nanoflann::KdTreeFLANN<PointTarget>> target_kdtree_;

  // Active covariance pointers — may alias either *_covs_owned_ (when computed
  // internally) or an externally-supplied list (via setSourceCovariances etc).
  std::shared_ptr<const CovarianceList> source_covs_;
  std::shared_ptr<const CovarianceList> target_covs_;

protected:
  // Internal mutable caches; reused across scans via resize() instead of a
  // fresh make_shared each call.
  std::shared_ptr<CovarianceList> source_covs_owned_;
  std::shared_ptr<CovarianceList> target_covs_owned_;

public:

  float source_density_;
  float target_density_;

  int num_correspondences;

protected:
  int num_threads_;
  int k_correspondences_;
  double corr_dist_threshold_;

  RegularizationMethod regularization_method_;

  CovarianceList mahalanobis_;

  std::vector<int> correspondences_;
  std::vector<float> sq_distances_;
};
}  // namespace nano_gicp

