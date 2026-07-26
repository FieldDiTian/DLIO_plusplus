#include <glim/mapping/global_mapping_pose_graph.hpp>

#include <algorithm>
#include <stdexcept>
#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/isam2_ext_dummy.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/util/parallelism.hpp>

#include <glim/util/config.hpp>
#include <glim/util/serialization.hpp>
#include <glim/mapping/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

using Callbacks = GlobalMappingCallbacks;

GlobalMappingPoseGraphParams::GlobalMappingPoseGraphParams() {
  Config config(GlobalConfig::get_config_path("config_global_mapping"));

  enable_optimization = config.param<bool>("global_mapping", "enable_optimization", true);
  registration_type = config.param<std::string>("global_mapping", "registration_type", "GICP");

  min_travel_dist = config.param<double>("global_mapping", "min_travel_dist", 100.0);
  max_neighbor_dist = config.param<double>("global_mapping", "max_neighbor_dist", 10.0);
  min_inliear_fraction = config.param<double>("global_mapping", "min_inliear_fraction", 0.5);

  subsample_target = config.param<int>("global_mapping", "subsample_target", 10000);
  subsample_rate = config.param<double>("global_mapping", "subsample_rate", 0.1);
  gicp_max_correspondence_dist = config.param<double>("global_mapping", "gicp_max_correspondence_dist", 2.0);
  vgicp_voxel_resolution = config.param<double>("global_mapping", "vgicp_voxel_resolution", 2.0);

  odom_factor_stddev = config.param<double>("global_mapping", "odom_factor_stddev", 1e-3);
  loop_factor_stddev = config.param<double>("global_mapping", "loop_factor_stddev", 0.1);
  loop_factor_robust_width = config.param<double>("global_mapping", "loop_factor_robust_width", 1.0);

  loop_candidate_buffer_size = config.param<int>("global_mapping", "loop_candidate_buffer_size", 100);
  loop_candidate_eval_per_thread = config.param<int>("global_mapping", "loop_candidate_eval_per_thread", 2);

  use_isam2_dogleg = config.param<bool>("global_mapping", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("global_mapping", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("global_mapping", "isam2_relinearize_thresh", 0.1);

  init_pose_damping_scale = config.param<double>("global_mapping", "init_pose_damping_scale", 1e10);
  offload_points_dir = config.param<std::string>("global_mapping", "offload_points_dir", "");

  num_threads = config.param<int>("global_mapping", "num_threads", 2);
}

GlobalMappingPoseGraphParams::~GlobalMappingPoseGraphParams() {}

GlobalMappingPoseGraph::GlobalMappingPoseGraph(const GlobalMappingPoseGraphParams& params) : params(params) {
  if (!params.offload_points_dir.empty()) {
    const boost::filesystem::path offload_dir(params.offload_points_dir);
    if (!offload_dir.is_absolute()) {
      throw std::invalid_argument("global_mapping.offload_points_dir must be an absolute path");
    }
    if (boost::filesystem::exists(offload_dir) && !boost::filesystem::is_empty(offload_dir)) {
      throw std::runtime_error(
        "global_mapping.offload_points_dir is not empty: " + offload_dir.string() + " (use a unique per-run directory so stale point payloads cannot enter a map)");
    }
    boost::filesystem::create_directories(offload_dir);
    logger->info("dense submap point offload enabled: {}", offload_dir.string());
  }

  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  gtsam::ISAM2Params isam2_params;
  if (params.use_isam2_dogleg) {
    gtsam::ISAM2DoglegParams dogleg_params;
    isam2_params.setOptimizationParams(dogleg_params);
  }
  isam2_params.relinearizeSkip = params.isam2_relinearize_skip;
  isam2_params.setRelinearizeThreshold(params.isam2_relinearize_thresh);

  if (params.enable_optimization) {
    isam2.reset(new gtsam_points::ISAM2Ext(isam2_params));
  } else {
    isam2.reset(new gtsam_points::ISAM2ExtDummy(isam2_params));
  }

#ifdef GTSAM_USE_TBB
  tbb_task_arena.reset(new tbb::task_arena(params.num_threads));
#endif

  loop_detection_finalized = false;
  loop_candidates_proposed = 0;
  loop_candidates_evaluated = 0;
  loop_candidates_dropped = 0;
  loop_factors_accepted = 0;
  loop_detection_thread = std::thread([this] { loop_detection_task(); });
}

GlobalMappingPoseGraph::~GlobalMappingPoseGraph() {
  finish_loop_detection();
}

void GlobalMappingPoseGraph::finish_loop_detection() {
  if (loop_detection_finalized.exchange(true)) {
    return;
  }

  loop_candidates.submit_end_of_data();
  if (loop_detection_thread.joinable()) {
    loop_detection_thread.join();
  }

  logger->info(
    "loop closure summary: proposed={} evaluated={} accepted={} dropped={}",
    loop_candidates_proposed.load(),
    loop_candidates_evaluated.load(),
    loop_factors_accepted.load(),
    loop_candidates_dropped.load());
}

void GlobalMappingPoseGraph::insert_submap(const SubMap::Ptr& submap) {
  const int current = submaps.size();
  const int last = current - 1;
  insert_submap(current, submap);

  gtsam::Pose3 current_T_world_submap = gtsam::Pose3::Identity();
  gtsam::Pose3 last_T_world_submap = gtsam::Pose3::Identity();

  if (current != 0) {
    if (isam2->valueExists(X(last))) {
      last_T_world_submap = isam2->calculateEstimate<gtsam::Pose3>(X(last));
    } else {
      last_T_world_submap = new_values->at<gtsam::Pose3>(X(last));
    }

    const Eigen::Isometry3d T_origin0_endpointR0 = submaps[last]->T_origin_endpoint_R;
    const Eigen::Isometry3d T_origin1_endpointL1 = submaps[current]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_endpointR0_endpointL1 = submaps[last]->odom_frames.back()->T_world_sensor().inverse() * submaps[current]->odom_frames.front()->T_world_sensor();
    const Eigen::Isometry3d T_origin0_origin1 = T_origin0_endpointR0 * T_endpointR0_endpointL1 * T_origin1_endpointL1.inverse();

    current_T_world_submap = last_T_world_submap * gtsam::Pose3(T_origin0_origin1.matrix());
  } else {
    current_T_world_submap = gtsam::Pose3(submap->T_world_origin.matrix());
  }

  new_values->insert(X(current), current_T_world_submap);
  submap->T_world_origin = Eigen::Isometry3d(current_T_world_submap.matrix());

  Callbacks::on_insert_submap(submap);

  submap->drop_frame_points();

  if (current == 0) {
    new_factors->emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params.init_pose_damping_scale);
  } else {
    new_factors->add(*create_odometry_factors(current));

    find_loop_candidates(current);
    new_factors->add(*collect_detected_loops());
  }

  Callbacks::on_smoother_update(*isam2, *new_factors, *new_values);
  try {
    gtsam_points::ISAM2ResultExt result;
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      result = isam2->update(*new_factors, *new_values);
#ifdef GTSAM_USE_TBB
    });
#endif

    Callbacks::on_smoother_update_result(*isam2, result);

  } catch (std::exception& e) {
    logger->error("an exception was caught during global map optimization!!");
    logger->error(e.what());
  }
  new_values.reset(new gtsam::Values);
  new_factors.reset(new gtsam::NonlinearFactorGraph);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingPoseGraph::optimize() {
  if (isam2->empty()) {
    return;
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  new_factors.add(*collect_detected_loops());

  Callbacks::on_smoother_update(*isam2, new_factors, new_values);

  gtsam_points::ISAM2ResultExt result;
#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    result = isam2->update(new_factors, new_values);
#ifdef GTSAM_USE_TBB
  });
#endif

  Callbacks::on_smoother_update_result(*isam2, result);

  update_submaps();
  Callbacks::on_update_submaps(submaps);
}

void GlobalMappingPoseGraph::save(const std::string& path) {
  // Loop detection is asynchronous.  Taking the final graph snapshot before
  // draining it silently loses every accepted factor still in the detector's
  // local/worker queues.  This is especially damaging for per-scan submaps,
  // where a lap boundary can enqueue many candidates near end-of-bag.
  finish_loop_detection();
  optimize();

  boost::filesystem::create_directories(path);

  gtsam::NonlinearFactorGraph serializable_factors = isam2->getFactorsUnsafe();

  logger->info("serializing factor graph to {}/graph.bin", path);
  serializeToBinaryFile(serializable_factors, path + "/graph.bin");
  serializeToBinaryFile(isam2->calculateEstimate(), path + "/values.bin");

  std::ofstream ofs(path + "/graph.txt");
  ofs << "num_submaps: " << submaps.size() << std::endl;
  ofs << "num_all_frames: " << std::accumulate(submaps.begin(), submaps.end(), 0, [](int sum, const SubMap::ConstPtr& submap) { return sum + submap->frames.size(); }) << std::endl;

  ofs << "num_matching_cost_factors: " << 0 << std::endl;
  ofs << "num_loop_candidates_proposed: " << loop_candidates_proposed.load() << std::endl;
  ofs << "num_loop_candidates_evaluated: " << loop_candidates_evaluated.load() << std::endl;
  ofs << "num_loop_candidates_dropped: " << loop_candidates_dropped.load() << std::endl;
  ofs << "num_loop_closure_factors: " << loop_factors_accepted.load() << std::endl;

  std::ofstream odom_lidar_ofs(path + "/odom_lidar.txt");
  std::ofstream traj_lidar_ofs(path + "/traj_lidar.txt");

  std::ofstream odom_imu_ofs(path + "/odom_imu.txt");
  std::ofstream traj_imu_ofs(path + "/traj_imu.txt");

  const auto write_tum_frame = [](std::ofstream& ofs, const double stamp, const Eigen::Isometry3d& pose) {
    const Eigen::Quaterniond quat(pose.linear());
    const Eigen::Vector3d trans(pose.translation());
    ofs << boost::format("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f") % stamp % trans.x() % trans.y() % trans.z() % quat.x() % quat.y() % quat.z() % quat.w() << std::endl;
  };

  for (int i = 0; i < submaps.size(); i++) {
    for (const auto& frame : submaps[i]->odom_frames) {
      write_tum_frame(odom_lidar_ofs, frame->stamp, frame->T_world_lidar);
      write_tum_frame(odom_imu_ofs, frame->stamp, frame->T_world_imu);
    }

    const Eigen::Isometry3d T_world_endpoint_L = submaps[i]->T_world_origin * submaps[i]->T_origin_endpoint_L;
    const Eigen::Isometry3d T_odom_lidar0 = submaps[i]->frames.front()->T_world_lidar;
    const Eigen::Isometry3d T_odom_imu0 = submaps[i]->frames.front()->T_world_imu;

    for (const auto& frame : submaps[i]->frames) {
      const Eigen::Isometry3d T_world_imu = T_world_endpoint_L * T_odom_imu0.inverse() * frame->T_world_imu;
      const Eigen::Isometry3d T_world_lidar = T_world_imu * frame->T_lidar_imu.inverse();

      write_tum_frame(traj_imu_ofs, frame->stamp, T_world_imu);
      write_tum_frame(traj_lidar_ofs, frame->stamp, T_world_lidar);
    }

    const std::string output_submap_dir = (boost::format("%s/%06d") % path % i).str();
    submaps[i]->save(output_submap_dir);
    restore_offloaded_points(i, output_submap_dir);
  }
}

gtsam_points::PointCloud::Ptr GlobalMappingPoseGraph::export_points() {
  return std::make_shared<gtsam_points::PointCloudCPU>();
}

void GlobalMappingPoseGraph::insert_submap(int current, const SubMap::Ptr& submap) {
  logger->debug("insert_submap id={}", submap->id);

  submap->voxelmaps.clear();

  submaps.push_back(submap);

  auto target = std::make_shared<SubMapTarget>();
  target->submap = submap;

  // Subsample points for registration
  if (params.subsample_target > 0) {
    const double sampling_rate = std::min(1.0, static_cast<double>(params.subsample_target) / submap->frame->size());
    target->subsampled = sampling_rate < 1.0 ? gtsam_points::random_sampling(submap->frame, sampling_rate, mt) : submap->frame;
  } else {
    if (params.subsample_rate > 0.99) {
      target->subsampled = submap->frame;
    } else {
      target->subsampled = gtsam_points::random_sampling(submap->frame, params.subsample_rate, mt);
    }
  }

  // Dense per-scan submaps are the quality lever, but loop registration only
  // needs the bounded sample above. Persist the dense compact payload before
  // replacing the in-memory frame with that sample. save() restores the dense
  // payload into the numbered dump directory.
  if (!params.offload_points_dir.empty()) {
    const std::string offload_submap_dir = (boost::format("%s/%06d") % params.offload_points_dir % current).str();
    boost::filesystem::create_directories(offload_submap_dir);
    submap->frame->save_compact(offload_submap_dir);
    offloaded_point_dirs.push_back(offload_submap_dir);
    target->registration_target = target->subsampled;
    submap->frame = std::const_pointer_cast<gtsam_points::PointCloud>(target->registration_target);
  } else {
    offloaded_point_dirs.emplace_back();
    target->registration_target = submap->frame;
  }

  // Create nearest neighbor search
  if (params.registration_type == "GICP") {
    target->tree = std::make_shared<gtsam_points::KdTree>(target->registration_target->points, target->registration_target->size());
  } else if (params.registration_type == "VGICP") {
    target->voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(params.vgicp_voxel_resolution);
    target->voxels->insert(*target->registration_target);
  } else {
    logger->warn("unknown registration type: {}", params.registration_type);
  }

  if (current == 0) {
    target->travel_dist = 0.0;
  } else {
    const double displacement = (submaps[current - 1]->T_world_origin.translation() - submaps[current]->T_world_origin.translation()).norm();
    target->travel_dist = submap_targets.back()->travel_dist + displacement;
  }

  submap_targets.push_back(target);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::create_odometry_factors(int current) const {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();
  if (current == 0) {
    return factors;
  }

  const int last = current - 1;
  const gtsam::Pose3 T_last_current = gtsam::Pose3((submaps[last]->origin_frame()->T_world_sensor().inverse() * submaps[current]->origin_frame()->T_world_sensor()).matrix());
  factors->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), T_last_current, gtsam::noiseModel::Isotropic::Sigma(6, params.odom_factor_stddev));

  return factors;
}

void GlobalMappingPoseGraph::find_loop_candidates(int current) {
  std::vector<LoopCandidate> new_candidates;
  for (int i = 0; i < submaps.size() - 1; i++) {
    // Skip if the direct distance between submaps is too far.
    const double direct_dist = (submaps[current]->T_world_origin.translation() - submaps[i]->T_world_origin.translation()).norm();
    if (direct_dist > params.max_neighbor_dist) {
      // Fast forward if the direct distance is too far.
      if (i != 0 && direct_dist > params.max_neighbor_dist * 2) {
        const int average_window = 3;
        const int left = std::max(0, i - average_window);
        const double travel_dist_avg = (submap_targets[i]->travel_dist - submap_targets[left]->travel_dist) / std::max(i - left, 1);
        const int step = 0.8 * direct_dist / std::min(travel_dist_avg, 100.0);

        i += std::min(10, step);
      }

      continue;
    }

    // Break if the travel distance is too short.
    const double travel_dist = submap_targets[current]->travel_dist - submap_targets[i]->travel_dist;
    if (travel_dist < params.min_travel_dist) {
      break;
    }

    // Add a loop candidate.
    const Eigen::Isometry3d T_target_source = submaps[i]->T_world_origin.inverse() * submaps[current]->T_world_origin;
    new_candidates.emplace_back(LoopCandidate{submap_targets[i], submap_targets[current], T_target_source});
  }

  loop_candidates_proposed.fetch_add(new_candidates.size());
  loop_candidates.insert(new_candidates);
}

std::shared_ptr<gtsam::NonlinearFactorGraph> GlobalMappingPoseGraph::collect_detected_loops() {
  auto factors = std::make_shared<gtsam::NonlinearFactorGraph>();

  factors->add(detected_loops.get_all_and_clear());

  return factors;
}

void GlobalMappingPoseGraph::loop_detection_task() {
  // Keep candidate selection reproducible and independent from registration
  // point subsampling, which uses the class-level engine.
  std::mt19937 loop_mt;
  std::deque<LoopCandidate> candidates_buffer;  // Local loop candidate buffer

  while (true) {
    logger->debug("wait for loop candidates");
    auto new_candidates = loop_candidates.get_all_and_clear_wait();
    candidates_buffer.insert(candidates_buffer.end(), new_candidates.begin(), new_candidates.end());

    logger->debug("|candidates_buffer|={}", candidates_buffer.size());

    // An empty batch is only returned after submit_end_of_data().  Keep
    // iterating while the local buffer is non-empty so EOF drains it instead
    // of abandoning candidates during save/destruction.
    if (new_candidates.empty() && candidates_buffer.empty()) {
      break;
    }

    const size_t buffer_limit = static_cast<size_t>(std::max(1, params.loop_candidate_buffer_size));
    if (candidates_buffer.size() > buffer_limit) {
      // Uniform sampling preserves candidate diversity.  Retaining only the
      // nearest poses over-constrained repeated views of the same Laguna
      // surface and regressed the trusted-map A/B.
      std::shuffle(candidates_buffer.begin(), candidates_buffer.end(), loop_mt);
      loop_candidates_dropped.fetch_add(candidates_buffer.size() - buffer_limit);
      candidates_buffer.resize(buffer_limit);
    }

    // Take a subset of the candidates to evaluate.
    const int eval_count = std::max(1, params.loop_candidate_eval_per_thread * params.num_threads);
    std::vector<LoopCandidate> candidates;
    if (candidates_buffer.size() < eval_count) {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.end());
      candidates_buffer.clear();
    } else {
      candidates.assign(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
      candidates_buffer.erase(candidates_buffer.begin(), candidates_buffer.begin() + eval_count);
    }
    loop_candidates_evaluated.fetch_add(candidates.size());

    std::vector<double> inlier_fractions(candidates.size(), 0.0);
    std::vector<gtsam::Pose3> T_target_source(candidates.size());

    const auto evaluate_candidate = [&](int i) {
      const auto candidate = candidates[i];
      const auto target = candidates[i].target;
      const auto source = candidates[i].source;

      gtsam::Values values;
      values.insert(0, gtsam::Pose3(candidates[i].init_T_target_source.matrix()));

      double error, inlier_fraction;

      if (params.registration_type == "GICP") {
        auto factor =
          gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(gtsam::Pose3(), 0, candidate.target->registration_target, candidate.source->subsampled, candidate.target->tree);
        factor->set_max_correspondence_distance(params.gicp_max_correspondence_dist);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);
        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else if (params.registration_type == "VGICP") {
        auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), 0, candidate.target->voxels, candidate.source->subsampled);

        gtsam::NonlinearFactorGraph graph;
        graph.add(factor);

        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(10);

        values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();

        error = factor->error(values);
        inlier_fraction = factor->inlier_fraction();
      } else {
        logger->warn("unknown registration type: {}", params.registration_type);
        return;
      }

      logger->debug("target={}, source={}, error={}, inlier_fraction={}", target->submap->id, source->submap->id, error, inlier_fraction);

      inlier_fractions[i] = inlier_fraction;
      T_target_source[i] = values.at<gtsam::Pose3>(0);
    };

    // Evaluate loop candidates in parallel.
#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      if (gtsam_points::is_omp_default()) {
#pragma omp parallel for num_threads(params.num_threads) schedule(dynamic)
        for (int i = 0; i < candidates.size(); i++) {
          evaluate_candidate(i);
        }
      } else {
#ifdef GTSAM_POINTS_USE_TBB
        tbb::parallel_for(tbb::blocked_range<int>(0, candidates.size(), 2), [&](const tbb::blocked_range<int>& range) {
          for (int i = range.begin(); i < range.end(); i++) {
            evaluate_candidate(i);
          }
        });
#else
      std::cerr << "error : TBB is not enabled" << std::endl;
      abort();
#endif
      }

#ifdef GTSAM_USE_TBB
    });
#endif

    // Check the matching results.
    std::vector<gtsam::NonlinearFactor::shared_ptr> factors;
    for (int i = 0; i < candidates.size(); i++) {
      // Check if the inlier fraction (overlap with target) is large enough.
      if (inlier_fractions[i] < params.min_inliear_fraction) {
        continue;
      }

      // Create factor.
      gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Isotropic::Sigma(6, params.loop_factor_stddev);
      noise_model = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(params.loop_factor_robust_width), noise_model);
      factors.emplace_back(
        gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(candidates[i].target->submap->id), X(candidates[i].source->submap->id), T_target_source[i], noise_model));
    }

    loop_factors_accepted.fetch_add(factors.size());
    detected_loops.insert(factors);
  }
}

void GlobalMappingPoseGraph::update_submaps() {
  for (int i = 0; i < submaps.size(); i++) {
    submaps[i]->T_world_origin = Eigen::Isometry3d(isam2->calculateEstimate<gtsam::Pose3>(X(i)).matrix());
  }
}

void GlobalMappingPoseGraph::restore_offloaded_points(size_t index, const std::string& output_submap_dir) {
  if (index >= offloaded_point_dirs.size() || offloaded_point_dirs[index].empty()) {
    return;
  }

  const boost::filesystem::path source_dir(offloaded_point_dirs[index]);
  const boost::filesystem::path destination_dir(output_submap_dir);
  if (boost::filesystem::absolute(source_dir) == boost::filesystem::absolute(destination_dir)) {
    return;
  }

  size_t restored_files = 0;
  for (boost::filesystem::directory_iterator it(source_dir), end; it != end; ++it) {
    if (!boost::filesystem::is_regular_file(it->path())) {
      continue;
    }
    const std::string filename = it->path().filename().string();
    if (filename.size() < 12 || filename.compare(filename.size() - 12, 12, "_compact.bin") != 0) {
      continue;
    }
    boost::filesystem::copy_file(it->path(), destination_dir / it->path().filename(), boost::filesystem::copy_options::overwrite_existing);
    ++restored_files;
  }

  if (restored_files == 0) {
    throw std::runtime_error("dense point offload payload is missing for submap " + std::to_string(index) + ": " + source_dir.string());
  }

  logger->debug("restored {} dense compact point files for submap {} from {}", restored_files, index, source_dir.string());
  offloaded_point_dirs[index] = destination_dir.string();

  // The dense payload is now durable in the dump. Reclaim only the temporary
  // per-run offload copy; never remove an earlier dump used as the source of a
  // subsequent save.
  if (source_dir.parent_path() == boost::filesystem::path(params.offload_points_dir)) {
    boost::filesystem::remove_all(source_dir);
  }
}

}  // namespace glim
