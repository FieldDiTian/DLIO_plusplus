// Headless GLIM dump -> single PCD map exporter.
//
// Loads a GLIM dump directory (the dump_path of glim_rosbag / glim_rosnode),
// merges all submaps through GlobalMapping::export_points(), and writes one
// binary PCD (fields: x y z intensity) ready for gicp_localization's
// localization/map_path.
//
// This is the scriptable companion to the offline_viewer GUI export. The GUI
// remains the recommended QA pass (visual inspection, re-optimization, manual
// loop closure); use this tool for automated pipelines and quick looks.
//
// Usage:
//   ros2 run glim_ros glim_dump_to_pcd <dump_dir> <output.pcd> [config_path]
//
// config_path defaults to "<dump_dir>/config" (the config GLIM saved with the
// dump) and falls back to the glim package config when absent.

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <glim/util/config.hpp>
#include <glim/util/logging.hpp>
#include <glim/mapping/global_mapping.hpp>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: glim_dump_to_pcd <dump_dir> <output.pcd> [config_path]" << std::endl;
    return 1;
  }
  const std::string dump_path = argv[1];
  const std::string out_path = argv[2];

  auto logger = spdlog::stdout_color_mt("glim");
  spdlog::set_default_logger(logger);

  std::string config_path;
  if (argc >= 4) {
    config_path = argv[3];
  } else if (std::filesystem::exists(dump_path + "/config/config.json")) {
    config_path = dump_path + "/config";
  } else {
    config_path = ament_index_cpp::get_package_share_directory("glim") + "/config";
  }
  spdlog::info("config_path: {}", config_path);
  glim::GlobalConfig::instance(config_path);

  glim::GlobalMapping global_mapping;
  spdlog::info("loading dump: {}", dump_path);
  if (!global_mapping.load(dump_path)) {
    spdlog::error("failed to load dump from {}", dump_path);
    return 1;
  }

  const auto points = global_mapping.export_points();
  if (!points || points->size() == 0) {
    spdlog::error("dump contains no points");
    return 1;
  }
  const bool has_intensity = points->has_intensities();
  spdlog::info("exporting {} points (intensity: {})", points->size(), has_intensity ? "yes" : "no");

  std::ofstream ofs(out_path, std::ios::binary);
  if (!ofs) {
    spdlog::error("cannot open output file {}", out_path);
    return 1;
  }
  const size_t n = points->size();
  ofs << "# .PCD v0.7 - Point Cloud Data file format\n"
      << "VERSION 0.7\n"
      << "FIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\n"
      << "TYPE F F F F\n"
      << "COUNT 1 1 1 1\n"
      << "WIDTH " << n << "\n"
      << "HEIGHT 1\n"
      << "VIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << n << "\n"
      << "DATA binary\n";

  std::vector<float> buf(n * 4);
  for (size_t i = 0; i < n; i++) {
    buf[i * 4 + 0] = static_cast<float>(points->points[i].x());
    buf[i * 4 + 1] = static_cast<float>(points->points[i].y());
    buf[i * 4 + 2] = static_cast<float>(points->points[i].z());
    buf[i * 4 + 3] = has_intensity ? static_cast<float>(points->intensities[i]) : 0.0f;
  }
  ofs.write(reinterpret_cast<const char*>(buf.data()), buf.size() * sizeof(float));
  ofs.close();

  spdlog::info("wrote {} points to {}", n, out_path);
  return 0;
}
