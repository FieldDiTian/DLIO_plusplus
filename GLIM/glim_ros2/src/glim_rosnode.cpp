#include <iostream>
#include <spdlog/spdlog.h>
#include <rclcpp/rclcpp.hpp>

#include <glim_ros/glim_ros.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto glim = std::make_shared<glim::GlimROS>(options);

  // Online (live) mapping has been removed: GLIM builds maps OFFLINE only.
  if (!glim->online_mapping_enabled()) {
    spdlog::error(
      "Online GLIM mapping is disabled. GLIM builds maps OFFLINE only -- run "
      "'glim_rosbag <bag>' or 'glim_pcap_rosbag <pcap>'. "
      "(Set glim_ros/enable_online_mapping=true in config_ros.json to override.)");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(glim);
  rclcpp::shutdown();

  std::string dump_path = "/tmp/dump";
  glim->declare_parameter<std::string>("dump_path", dump_path);
  glim->get_parameter<std::string>("dump_path", dump_path);

  glim->wait();
  glim->save(dump_path);

  return 0;
}