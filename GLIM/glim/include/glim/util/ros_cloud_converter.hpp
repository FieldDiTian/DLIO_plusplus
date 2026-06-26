#pragma once

#include <memory>
#include <vector>
#include <iostream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>
#include <boost/format.hpp>

#include <Eigen/Core>
#include <gtsam_points/types/point_cloud.hpp>
#include <glim/util/raw_points.hpp>

#ifdef GLIM_ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace glim {
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
using PointField = sensor_msgs::msg::PointField;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.sec + stamp.nanosec / 1e9;
}

inline builtin_interfaces::msg::Time from_sec(const double time) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = std::floor(time);
  stamp.nanosec = (time - stamp.sec) * 1e9;
  return stamp;
}

}  // namespace glim
#else
#include <sensor_msgs/PointCloud2.h>
namespace glim {
using PointCloud2 = sensor_msgs::PointCloud2;
using PointCloud2Ptr = sensor_msgs::PointCloud2::Ptr;
using PointCloud2ConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using PointField = sensor_msgs::PointField;

template <typename Stamp>
double to_sec(const Stamp& stamp) {
  return stamp.toSec();
}

inline ros::Time from_sec(const double time) {
  ros::Time stamp;
  stamp.sec = std::floor(time);
  stamp.nsec = (time - stamp.sec) * 1e9;
  return stamp;
}

}  // namespace glim
#endif

namespace glim {

template <typename T>
Eigen::Vector4d get_vec4(const void* x, const void* y, const void* z) {
  return Eigen::Vector4d(*reinterpret_cast<const T*>(x), *reinterpret_cast<const T*>(y), *reinterpret_cast<const T*>(z), 1.0);
}

// `epoch_anchor_count` (default -1 = unused) is the number of leading points that
// belong to the PRIMARY scan in a multi-LiDAR concatenated cloud (lidar_concat
// copies primary bytes first, so they are the first epoch_anchor_count entries). The
// epoch-axis safeguard below anchors its rebase offset on the minimum over just
// these primary points instead of the global merged minimum, so an aux scan that
// began before the primary does not drag the whole merged sweep late. For a
// single sensor it is left at -1 and the global minimum is used (unchanged).
static RawPoints::Ptr extract_raw_points(const PointCloud2& points_msg, const std::string& intensity_channel, const std::string& ring_channel, int epoch_anchor_count = -1) {
  int num_points = points_msg.width * points_msg.height;

  int x_type = 0;
  int y_type = 0;
  int z_type = 0;
  int time_type = 0;  // ouster and livox
  int intensity_type = 0;
  int color_type = 0;
  int ring_type = 0;

  int x_offset = -1;
  int y_offset = -1;
  int z_offset = -1;
  int time_offset = -1;
  int time_count = 0;
  int intensity_offset = -1;
  int color_offset = -1;
  int ring_offset = -1;

  std::unordered_map<std::string, std::pair<int*, int*>> fields;
  fields["x"] = std::make_pair(&x_type, &x_offset);
  fields["y"] = std::make_pair(&y_type, &y_offset);
  fields["z"] = std::make_pair(&z_type, &z_offset);
  fields["t"] = std::make_pair(&time_type, &time_offset);
  fields["time"] = std::make_pair(&time_type, &time_offset);
  fields["time_stamp"] = std::make_pair(&time_type, &time_offset);
  fields["timestamp"] = std::make_pair(&time_type, &time_offset);
  fields[intensity_channel] = std::make_pair(&intensity_type, &intensity_offset);
  fields["rgba"] = std::make_pair(&color_type, &color_offset);
  fields[ring_channel] = std::make_pair(&ring_type, &ring_offset);

  for (const auto& field : points_msg.fields) {
    auto found = fields.find(field.name);
    if (found == fields.end()) {
      continue;
    }

    *found->second.first = field.datatype;
    *found->second.second = field.offset;
    if (field.name == "t" || field.name == "time" || field.name == "time_stamp" || field.name == "timestamp") {
      time_count = field.count;
    }
  }

  if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
    spdlog::warn("missing point coordinate fields");
    return nullptr;
  }

  if ((x_type != PointField::FLOAT32 && x_type != PointField::FLOAT64) || x_type != y_type || x_type != z_type) {
    spdlog::warn("unsupported points type");
    return nullptr;
  }

  auto raw_points = std::make_shared<RawPoints>();

  raw_points->points.resize(num_points);

  if (x_type == PointField::FLOAT32 && y_offset == x_offset + sizeof(float) && z_offset == y_offset + sizeof(float)) {
    // Special case: contiguous 3 floats
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data[points_msg.point_step * i + x_offset];
      raw_points->points[i] << Eigen::Map<const Eigen::Vector3f>(reinterpret_cast<const float*>(x_ptr)).cast<double>(), 1.0;
    }
  } else if (x_type == PointField::FLOAT64 && y_offset == x_offset + sizeof(double) && z_offset == y_offset + sizeof(double)) {
    // Special case: contiguous 3 doubles
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data[points_msg.point_step * i + x_offset];
      raw_points->points[i] << Eigen::Map<const Eigen::Vector3d>(reinterpret_cast<const double*>(x_ptr)), 1.0;
    }
  } else {
    for (int i = 0; i < num_points; i++) {
      const auto* x_ptr = &points_msg.data[points_msg.point_step * i + x_offset];
      const auto* y_ptr = &points_msg.data[points_msg.point_step * i + y_offset];
      const auto* z_ptr = &points_msg.data[points_msg.point_step * i + z_offset];

      if (x_type == PointField::FLOAT32) {
        raw_points->points[i] = get_vec4<float>(x_ptr, y_ptr, z_ptr);
      } else {
        raw_points->points[i] = get_vec4<double>(x_ptr, y_ptr, z_ptr);
      }
    }
  }

  if (time_offset >= 0) {
    raw_points->times.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* time_ptr = &points_msg.data[points_msg.point_step * i + time_offset];
      switch (time_type) {
        case PointField::UINT32:
          raw_points->times[i] = *reinterpret_cast<const uint32_t*>(time_ptr) / 1e9;
          break;
        case PointField::FLOAT32:
          raw_points->times[i] = *reinterpret_cast<const float*>(time_ptr);
          break;
        case PointField::FLOAT64:
          raw_points->times[i] = *reinterpret_cast<const double*>(time_ptr);
          break;
        case PointField::UINT8:
          if (time_count == 8) {
            // Luminar Iris: little-endian uint64 PTP epoch nanoseconds.
            // Per the Luminar Iris Data Output Specification v1.3.0 the
            // sensor splits this into 48-bit epoch seconds in the packet
            // header (§2.1) and a 32-bit sub-second nanosecond count per ray
            // (§2.2/§2.6.3); the single uint64 epoch-ns read here is the
            // upstream driver's reconstruction (seconds*1e9 + ns). Divide by
            // 1e9 to get epoch seconds. NOTE: GLIM's TimeKeeper treats these as
            // ABSOLUTE and overwrites the frame stamp with the first point time,
            // so the epoch must match the IMU/header epoch -- see the epoch-axis
            // safeguard after this loop.
            std::uint64_t time_ns = 0;
            std::memcpy(&time_ns, time_ptr, sizeof(std::uint64_t));
            raw_points->times[i] = static_cast<double>(time_ns) / 1e9;
          } else {
            spdlog::warn("unsupported time type {} count {}", time_type, time_count);
            return nullptr;
          }
          break;
        default:
          spdlog::warn("unsupported time type {} count {}", time_type, time_count);
          return nullptr;
      }
    }

    // Epoch-axis safeguard (Luminar / absolute per-point times on the live path).
    // Absolute per-point timestamps (e.g. Luminar UINT8[8] epoch ns) can sit on
    // the sensor's own clock rather than the ROS/header epoch when the sensor is
    // not PTP-locked to an epoch grandmaster (observed in raw bags: point times
    // ~2e13 ns while header.stamp / IMU were on the Unix epoch). GLIM's TimeKeeper
    // OVERWRITES the frame stamp with the first absolute point time, so a
    // mismatched epoch desyncs the scan against the IMU and the frame is dropped
    // as unsynchronized. Here we rebase the absolute times onto the header epoch
    // (preserving the intra-scan span) when they are clearly on a different axis.
    // This generalizes scripts/prep_bag.py's offline repair to the live pipeline.
    // It is a no-op for already-aligned / prepped data (|diff| < 1 s) and for
    // scan-relative encodings (max_time < 1.0, e.g. Ouster/Velodyne ns- or
    // s-since-scan-start). GICP needs no equivalent: it anchors deskew at
    // header.stamp and uses only relative (ts - min_ts) offsets.
    if (!raw_points->times.empty()) {
      const double max_time = *std::max_element(raw_points->times.begin(), raw_points->times.end());
      // Anchor the rebase on the PRIMARY scan's earliest time, NOT the global
      // merged minimum. In a multi-LiDAR concat (lidar_concat) the primary points
      // are the first `epoch_anchor_count` entries; an aux scan that began before
      // the primary would otherwise become the global min and map onto the header
      // stamp, shifting the whole merged sweep late by the aux-primary offset. The
      // offset is applied to ALL points, so aux points keep their true relative
      // timing to the primary (and TimeKeeper can then take the genuine earliest
      // capture time across all sensors). For a single sensor epoch_anchor_count
      // is < 0 / >= size and this reduces to the global min (unchanged). Mirrors
      // the GICP deskew primary anchor.
      const size_t anchor_n =
        (epoch_anchor_count > 0 && static_cast<size_t>(epoch_anchor_count) <= raw_points->times.size())
          ? static_cast<size_t>(epoch_anchor_count)
          : raw_points->times.size();
      const double min_time = *std::min_element(raw_points->times.begin(), raw_points->times.begin() + anchor_n);
      const double header_sec = to_sec(points_msg.header.stamp);
      if (max_time >= 1.0 && std::abs(header_sec - min_time) > 1.0) {
        const double offset = header_sec - min_time;
        for (auto& t : raw_points->times) {
          t += offset;
        }
        static bool warned = false;
        if (!warned) {
          spdlog::warn(
            "ros_cloud_converter: per-point timestamps are on a different epoch than header.stamp "
            "(primary_min={:.6f}s header={:.6f}s diff={:.3f}s anchor={}); rebasing onto the header epoch "
            "(intra-scan span preserved). Likely an unsynced sensor clock -- confirm PTP lock.",
            min_time, header_sec, offset, (anchor_n < raw_points->times.size() ? "primary" : "global"));
          warned = true;
        }
      }
    }
  }

  if (intensity_offset >= 0) {
    raw_points->intensities.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* intensity_ptr = &points_msg.data[points_msg.point_step * i + intensity_offset];
      switch (intensity_type) {
        case PointField::UINT8:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint8_t*>(intensity_ptr);
          break;
        case PointField::UINT16:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint16_t*>(intensity_ptr);
          break;
        case PointField::UINT32:
          raw_points->intensities[i] = *reinterpret_cast<const std::uint32_t*>(intensity_ptr);
          break;
        case PointField::FLOAT32:
          raw_points->intensities[i] = *reinterpret_cast<const float*>(intensity_ptr);
          break;
        case PointField::FLOAT64:
          raw_points->intensities[i] = *reinterpret_cast<const double*>(intensity_ptr);
          break;
        default:
          spdlog::warn("unsupported intensity type {}", intensity_type);
          return nullptr;
      }
    }
  }

  if (color_offset >= 0) {
    if (color_type != PointField::UINT32) {
      spdlog::warn("unsupported color type {}", color_type);
    } else {
      raw_points->colors.resize(num_points);

      for (int i = 0; i < num_points; i++) {
        const auto* color_ptr = &points_msg.data[points_msg.point_step * i + color_offset];
        raw_points->colors[i] = Eigen::Matrix<unsigned char, 4, 1>(reinterpret_cast<const std::uint8_t*>(color_ptr)).cast<double>() / 255.0;
      }
    }
  }

  if (ring_offset >= 0) {
    raw_points->rings.resize(num_points);

    for (int i = 0; i < num_points; i++) {
      const auto* ring_ptr = &points_msg.data[points_msg.point_step * i + ring_offset];
      switch (ring_type) {
        case PointField::UINT8:
          raw_points->rings[i] = *reinterpret_cast<const std::uint8_t*>(ring_ptr);
          break;
        case PointField::UINT16:
          raw_points->rings[i] = *reinterpret_cast<const std::uint16_t*>(ring_ptr);
          break;
        case PointField::UINT32:
          raw_points->rings[i] = *reinterpret_cast<const std::uint32_t*>(ring_ptr);
          break;
        default:
          spdlog::warn("unsupported ring type {}", ring_type);
          return nullptr;
      }
    }
  }

  raw_points->stamp = to_sec(points_msg.header.stamp);
  return raw_points;
}

static RawPoints::Ptr extract_raw_points(const PointCloud2& points_msg, const std::string& intensity_channel = "intensity") {
  return extract_raw_points(points_msg, intensity_channel, "");
}

static RawPoints::Ptr extract_raw_points(const PointCloud2ConstPtr& points_msg, const std::string& intensity_channel = "intensity") {
  return extract_raw_points(*points_msg, intensity_channel, "");
}

static PointCloud2ConstPtr frame_to_pointcloud2(const std::string& frame_id, const double stamp, const gtsam_points::PointCloud& frame) {
  PointCloud2Ptr msg(new PointCloud2);
  msg->header.frame_id = frame_id;
  msg->header.stamp = from_sec(stamp);

  msg->width = frame.size();
  msg->height = 1;

  const auto create_field = [](const std::string& name, int offset, int datatype, int count) {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  int point_step = 0;
  msg->fields.reserve(6);

  msg->fields.emplace_back(create_field("x", sizeof(float) * 0, PointField::FLOAT32, 1));
  msg->fields.emplace_back(create_field("y", sizeof(float) * 1, PointField::FLOAT32, 1));
  msg->fields.emplace_back(create_field("z", sizeof(float) * 2, PointField::FLOAT32, 1));
  point_step += sizeof(float) * 3;

  if (frame.times) {
    msg->fields.emplace_back(create_field("t", point_step, PointField::FLOAT32, 1));
    point_step += sizeof(float);
  }

  if (frame.intensities) {
    msg->fields.emplace_back(create_field("intensity", point_step, PointField::FLOAT32, 1));
    point_step += sizeof(float);
  }

  const Eigen::Vector4f* colors = frame.aux_attributes.count("colors") ? frame.aux_attribute<Eigen::Vector4f>("colors") : nullptr;
  if (colors) {
    msg->fields.emplace_back(create_field("rgba", point_step, PointField::UINT32, 1));
    point_step += sizeof(std::uint32_t);
  }

  msg->is_bigendian = false;
  msg->point_step = point_step;
  msg->row_step = point_step * frame.size();

  msg->data.resize(point_step * frame.size());
  for (int i = 0; i < frame.size(); i++) {
    unsigned char* point_bytes = msg->data.data() + msg->point_step * i;
    Eigen::Map<Eigen::Vector3f> xyz(reinterpret_cast<float*>(point_bytes));
    xyz = frame.points[i].head<3>().cast<float>();
    point_bytes += sizeof(float) * 3;

    if (frame.times) {
      *reinterpret_cast<float*>(point_bytes) = frame.times[i];
      point_bytes += sizeof(float);
    }

    if (frame.intensities) {
      *reinterpret_cast<float*>(point_bytes) = frame.intensities[i];
      point_bytes += sizeof(float);
    }

    if (colors) {
      const Eigen::Matrix<std::uint8_t, 4, 1> rgba = (colors[i].array() * 255.0f).min(255.0f).max(0.0f).cast<std::uint8_t>();
      point_bytes[0] = rgba[0];
      point_bytes[1] = rgba[1];
      point_bytes[2] = rgba[2];
      point_bytes[3] = rgba[3];
      point_bytes += sizeof(std::uint32_t);
    }
  }

  return msg;
}

}  // namespace glim
