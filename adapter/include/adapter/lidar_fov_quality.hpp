#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace adapter {

struct VerticalFovMeasurement {
  bool valid = false;
  double min_elevation_deg = std::numeric_limits<double>::quiet_NaN();
  double max_elevation_deg = std::numeric_limits<double>::quiet_NaN();
  double vertical_fov_deg = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0;
  std::string source;
  std::string error;
};

namespace detail {

template <typename T>
inline T read_scalar(const std::uint8_t* data, bool swap_bytes) {
  T value;
  std::memcpy(&value, data, sizeof(T));
  if (swap_bytes) {
    auto* first = reinterpret_cast<std::uint8_t*>(&value);
    std::reverse(first, first + sizeof(T));
  }
  return value;
}

inline bool host_is_big_endian() {
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 0x01;
}

inline bool read_field_value(const sensor_msgs::msg::PointField& field,
                             const std::uint8_t* point, bool swap_bytes,
                             double& value) {
  const auto* data = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      value = static_cast<double>(read_scalar<float>(data, swap_bytes));
      return true;
    case sensor_msgs::msg::PointField::FLOAT64:
      value = read_scalar<double>(data, swap_bytes);
      return true;
    default:
      return false;
  }
}

inline std::size_t field_size(const sensor_msgs::msg::PointField& field) {
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      return sizeof(float);
    case sensor_msgs::msg::PointField::FLOAT64:
      return sizeof(double);
    default:
      return 0;
  }
}

inline bool field_fits_point(const sensor_msgs::msg::PointField& field,
                             std::size_t point_step) {
  const std::size_t size = field_size(field);
  return size > 0 && static_cast<std::size_t>(field.offset) <= point_step &&
         size <= point_step - static_cast<std::size_t>(field.offset);
}

inline const sensor_msgs::msg::PointField* find_field(
    const sensor_msgs::msg::PointCloud2& cloud, const std::string& name) {
  const auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                               [&name](const auto& field) {
                                 return field.name == name && field.count > 0;
                               });
  return it == cloud.fields.end() ? nullptr : &*it;
}

}  // namespace detail

// Measure the scan pattern, not only successful Cartesian returns. Luminar Iris
// publishes an `elevation` value for every commanded ray, including rays whose
// x/y/z/depth are zero because nothing reflected. That field is therefore the
// authoritative vertical-FOV signal. XYZ is only a fail-closed fallback for
// non-Luminar clouds when require_elevation_field is false.
inline VerticalFovMeasurement measure_vertical_fov(
    const sensor_msgs::msg::PointCloud2& cloud,
    const std::string& elevation_field = "elevation",
    bool elevation_in_radians = true,
    std::size_t min_samples = 1000,
    bool require_elevation_field = true) {
  VerticalFovMeasurement result;

  if (cloud.point_step == 0 || cloud.width == 0 || cloud.height == 0) {
    result.error = "empty PointCloud2 or zero point_step";
    return result;
  }
  if (static_cast<std::size_t>(cloud.row_step) <
      static_cast<std::size_t>(cloud.point_step) * cloud.width) {
    result.error = "PointCloud2 row_step is shorter than width * point_step";
    return result;
  }

  const std::size_t required_bytes =
      static_cast<std::size_t>(cloud.row_step) * cloud.height;
  if (cloud.data.size() < required_bytes) {
    result.error = "PointCloud2 data is shorter than row_step * height";
    return result;
  }

  const auto* elevation = detail::find_field(cloud, elevation_field);
  const auto* x = detail::find_field(cloud, "x");
  const auto* y = detail::find_field(cloud, "y");
  const auto* z = detail::find_field(cloud, "z");
  if (!elevation && require_elevation_field) {
    result.error = "required elevation field '" + elevation_field + "' is missing";
    return result;
  }
  if (!elevation && (!x || !y || !z)) {
    result.error = "neither elevation nor complete x/y/z fields are available";
    return result;
  }
  if (elevation && !detail::field_fits_point(*elevation, cloud.point_step)) {
    result.error = "elevation field must be FLOAT32/FLOAT64 and fit within point_step";
    return result;
  }
  if (!elevation &&
      (!detail::field_fits_point(*x, cloud.point_step) ||
       !detail::field_fits_point(*y, cloud.point_step) ||
       !detail::field_fits_point(*z, cloud.point_step))) {
    result.error = "x/y/z fallback fields must be FLOAT32/FLOAT64 and fit within point_step";
    return result;
  }

  const bool swap_bytes = cloud.is_bigendian != detail::host_is_big_endian();
  double min_deg = std::numeric_limits<double>::infinity();
  double max_deg = -std::numeric_limits<double>::infinity();
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

  for (std::size_t row = 0; row < cloud.height; ++row) {
    const auto* row_data = cloud.data.data() + row * cloud.row_step;
    for (std::size_t col = 0; col < cloud.width; ++col) {
      const auto* point = row_data + col * cloud.point_step;
      double angle_deg = 0.0;
      if (elevation) {
        double angle = 0.0;
        if (!detail::read_field_value(*elevation, point, swap_bytes, angle)) {
          result.error = "elevation field must be FLOAT32 or FLOAT64";
          return result;
        }
        if (!std::isfinite(angle)) {
          continue;
        }
        angle_deg = elevation_in_radians ? angle * kRadToDeg : angle;
        result.source = elevation_field;
      } else {
        double px = 0.0;
        double py = 0.0;
        double pz = 0.0;
        if (!detail::read_field_value(*x, point, swap_bytes, px) ||
            !detail::read_field_value(*y, point, swap_bytes, py) ||
            !detail::read_field_value(*z, point, swap_bytes, pz)) {
          result.error = "x/y/z fallback fields must be FLOAT32 or FLOAT64";
          return result;
        }
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        const double horizontal_range = std::hypot(px, py);
        if (horizontal_range < 0.1 && std::abs(pz) < 0.1) {
          continue;
        }
        angle_deg = std::atan2(pz, horizontal_range) * kRadToDeg;
        result.source = "xyz_fallback";
      }

      min_deg = std::min(min_deg, angle_deg);
      max_deg = std::max(max_deg, angle_deg);
      ++result.sample_count;
    }
  }

  if (result.sample_count < min_samples) {
    result.error = "only " + std::to_string(result.sample_count) +
                   " usable rays; require at least " + std::to_string(min_samples);
    return result;
  }

  result.min_elevation_deg = min_deg;
  result.max_elevation_deg = max_deg;
  result.vertical_fov_deg = max_deg - min_deg;
  result.valid = std::isfinite(result.vertical_fov_deg);
  if (!result.valid) {
    result.error = "computed vertical FOV is not finite";
  }
  return result;
}

inline bool meets_vertical_fov(const VerticalFovMeasurement& measurement,
                               double minimum_fov_deg) {
  constexpr double kFloatingPointToleranceDeg = 1e-6;
  return measurement.valid &&
         measurement.vertical_fov_deg + kFloatingPointToleranceDeg >= minimum_fov_deg;
}

}  // namespace adapter
