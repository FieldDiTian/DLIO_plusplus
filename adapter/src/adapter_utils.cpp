#include "adapter/adapter_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace adapter {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr uint8_t kRtkFixed = 4;

}  // namespace

double stampToSec(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time secToStamp(double sec)
{
  builtin_interfaces::msg::Time stamp;
  // [P2 FIX 2026-07-14] Guard the int32 seconds field. `sec * 1e9` for a
  // non-finite or out-of-range input overflows the int32 cast (UB): the
  // FusionEngine 0xFFFFFFFF "time unavailable" sentinel (~4.29e9 s) lands
  // directly past INT32_MAX seconds (~2.147e9). Clamp to a representable
  // range so a poisoned stamp becomes a benign bounded value, never UB.
  if (!std::isfinite(sec) || sec <= 0.0) {
    stamp.sec = 0;
    stamp.nanosec = 0;
    return stamp;
  }
  constexpr double kMaxStampSec = 2147483647.0;  // INT32_MAX seconds
  if (sec > kMaxStampSec) {
    sec = kMaxStampSec;
  }
  const int64_t ns = static_cast<int64_t>(std::llround(sec * 1e9));
  stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return stamp;
}

double p1ToSec(const fusion_engine_msgs::msg::Timestamp& stamp)
{
  return static_cast<double>(stamp.seconds) + static_cast<double>(stamp.fraction_ns) * 1e-9;
}

Eigen::Quaterniond rpyToQuat(double roll_deg, double pitch_deg, double yaw_deg)
{
  const Eigen::AngleAxisd roll(roll_deg * kDegToRad, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(pitch_deg * kDegToRad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(yaw_deg * kDegToRad, Eigen::Vector3d::UnitZ());
  return Eigen::Quaterniond(yaw * pitch * roll).normalized();
}

bool parseLocalEnuOrigin(const std::string& text, double& lat_deg, double& lon_deg, double& alt_m)
{
  if (text.empty()) {
    return false;
  }

  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::istringstream in(normalized);
  if (!(in >> lat_deg >> lon_deg >> alt_m)) {
    return false;
  }
  return std::isfinite(lat_deg) && std::isfinite(lon_deg) && std::isfinite(alt_m);
}

bool parseLocalEnuOriginTtl(const std::string& path, double& lat_deg, double& lon_deg, double& alt_m)
{
  std::ifstream in(path);
  if (!in) {
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (cells.size() < 3) {
      return false;
    }
    try {
      lat_deg = std::stod(cells[cells.size() - 3]);
      lon_deg = std::stod(cells[cells.size() - 2]);
      alt_m = std::stod(cells[cells.size() - 1]);
    } catch (const std::exception&) {
      return false;
    }
    return std::isfinite(lat_deg) && std::isfinite(lon_deg) && std::isfinite(alt_m);
  }
  return false;
}

std::vector<double> retimeArrivalStamps(const std::vector<double>& arrivals, double imu_period_sec)
{
  std::vector<double> stamps = arrivals;
  if (stamps.empty()) {
    return stamps;
  }
  for (int i = static_cast<int>(stamps.size()) - 2; i >= 0; --i) {
    const double ceil = stamps[static_cast<size_t>(i + 1)] - imu_period_sec;
    if (stamps[static_cast<size_t>(i)] > ceil) {
      stamps[static_cast<size_t>(i)] = ceil;
    }
  }
  return stamps;
}

bool posePassesRtkGate(const fusion_engine_msgs::msg::Pose& msg,
                       double max_var_xy,
                       double max_var_z)
{
  // [P3 FIX 2026-07-10] Require finite NONNEGATIVE covariance: `<= max`
  // alone let a negative sentinel (-1 = "unknown" in some publishers) pass
  // the accuracy gate whenever solution_type said RTK_FIXED.
  const auto ok = [](double v, double mx) { return std::isfinite(v) && v >= 0.0 && v <= mx; };
  return msg.solution_type == kRtkFixed &&
         ok(msg.position_covariance[0], max_var_xy) &&
         ok(msg.position_covariance[4], max_var_xy) &&
         ok(msg.position_covariance[8], max_var_z);
}

// [P2 FIX 2026-07-09] bin_seconds <= 0 previously produced NaN/inf bins and
// UB int casts; clamp to a sane floor.
P1ClockMapper::P1ClockMapper(double bin_seconds) : bin_seconds_(std::max(1.0, bin_seconds)) {}

void P1ClockMapper::addPosePair(double arrival_ros, double p1_time)
{
  // [P2 FIX 2026-07-09] Input validation + epoch-reset handling. FusionEngine
  // encodes "time not yet available" as 0xFFFFFFFF in both Timestamp fields
  // (~4.29e9 s): as the FIRST sample it used to poison first_p1_ (every later
  // genuine sample -> bin < 0 -> mapper never ready -> zero IMU all run); as a
  // LATER sample it triggered a ~1.7 GB bins_.resize() inside a callback.
  // A P1 regression (device power-cycle, bag loop) used to freeze the mapper
  // forever (bin < 0 on every subsequent pair).
  constexpr double kInvalidP1SentinelSec = 4.0e9;  // sentinel converts to ~4.29e9
  constexpr double kResetThresholdSec = 5.0;       // backward jump = epoch change
  constexpr double kMaxSessionSec = 24.0 * 3600.0; // forward cap (bin count <= 1440)
  constexpr int kGlitchResetCount = 100;           // persistent forward jump = epoch change

  if (!std::isfinite(arrival_ros) || !std::isfinite(p1_time) || p1_time < 0.0 ||
      p1_time >= kInvalidP1SentinelSec) {
    return;
  }
  if (std::isfinite(first_p1_)) {
    const double rel = p1_time - first_p1_;
    if (rel < -kResetThresholdSec) {
      // Backward epoch change: re-learn from scratch.
      bins_.clear();
      first_p1_ = p1_time;
      forward_glitch_streak_ = 0;
      applied_offset_ = std::numeric_limits<double>::quiet_NaN();  // P3: re-anchor slew
    } else if (rel > kMaxSessionSec) {
      // Single forward glitches are rejected; a persistent forward jump is a
      // real epoch change and resets the mapper.
      if (++forward_glitch_streak_ >= kGlitchResetCount) {
        bins_.clear();
        first_p1_ = p1_time;
        forward_glitch_streak_ = 0;
        applied_offset_ = std::numeric_limits<double>::quiet_NaN();  // P3: re-anchor slew
      } else {
        return;
      }
    } else {
      forward_glitch_streak_ = 0;
    }
  }
  if (!std::isfinite(first_p1_)) {
    first_p1_ = p1_time;
  }

  const int bin = static_cast<int>(std::floor((p1_time - first_p1_) / bin_seconds_));
  if (bin < 0) {
    return;
  }
  if (static_cast<size_t>(bin) >= bins_.size()) {
    bins_.resize(static_cast<size_t>(bin) + 1);
  }

  auto& b = bins_[static_cast<size_t>(bin)];
  b.count++;
  b.center_sum += p1_time;
  b.min_lag = std::min(b.min_lag, arrival_ros - p1_time);
}

bool P1ClockMapper::ready() const
{
  return std::any_of(bins_.begin(), bins_.end(), [](const Bin& b) { return b.count > 0; });
}

void P1ClockMapper::reset()
{
  bins_.clear();
  first_p1_ = std::numeric_limits<double>::quiet_NaN();
  applied_offset_ = std::numeric_limits<double>::quiet_NaN();
  last_slew_p1_ = std::numeric_limits<double>::quiet_NaN();
  forward_glitch_streak_ = 0;
}

double P1ClockMapper::toRos(double p1_time)
{
  std::vector<std::pair<double, double>> envelope;
  envelope.reserve(bins_.size());
  for (const auto& b : bins_) {
    if (b.count > 0) {
      envelope.emplace_back(b.center_sum / static_cast<double>(b.count), b.min_lag);
    }
  }
  if (envelope.empty()) {
    return p1_time;
  }
  double target_offset;
  if (envelope.size() == 1 || offsetDrift(envelope) < 0.005) {
    std::vector<double> offsets;
    offsets.reserve(envelope.size());
    for (const auto& p : envelope) {
      offsets.push_back(p.second);
    }
    const size_t mid = offsets.size() / 2;
    std::nth_element(offsets.begin(), offsets.begin() + static_cast<long>(mid), offsets.end());
    target_offset = offsets[mid];
  } else if (p1_time <= envelope.front().first) {
    target_offset = envelope.front().second;
  } else {
    target_offset = envelope.back().second;
    for (size_t i = 1; i < envelope.size(); ++i) {
      if (p1_time <= envelope[i].first) {
        const double t0 = envelope[i - 1].first;
        const double t1 = envelope[i].first;
        const double u = (p1_time - t0) / std::max(1e-9, t1 - t0);
        target_offset = envelope[i - 1].second * (1.0 - u) + envelope[i].second * u;
        break;
      }
    }
  }

  // [P3 FIX 2026-07-10] Slew instead of stepping: bounded offset motion of
  // 0.5 ms per second of stream keeps consecutive output stamps' dt within
  // ~0.5% of nominal even while bin refinement moves the raw estimate.
  constexpr double kMaxSlewPerSec = 5e-4;
  if (!std::isfinite(applied_offset_) || !std::isfinite(last_slew_p1_) ||
      p1_time < last_slew_p1_ - 1.0) {
    applied_offset_ = target_offset;  // first use or epoch reset: re-anchor
  } else {
    const double budget = kMaxSlewPerSec * std::max(0.0, p1_time - last_slew_p1_) + 1e-12;
    const double delta = target_offset - applied_offset_;
    applied_offset_ += std::clamp(delta, -budget, budget);
  }
  last_slew_p1_ = p1_time;
  return p1_time + applied_offset_;
}

double P1ClockMapper::driftMs() const
{
  std::vector<std::pair<double, double>> envelope;
  for (const auto& b : bins_) {
    if (b.count > 0) {
      envelope.emplace_back(b.center_sum / static_cast<double>(b.count), b.min_lag);
    }
  }
  return offsetDrift(envelope) * 1e3;
}

double P1ClockMapper::offsetDrift(const std::vector<std::pair<double, double>>& envelope)
{
  if (envelope.empty()) {
    return 0.0;
  }
  auto [min_it, max_it] = std::minmax_element(
    envelope.begin(), envelope.end(),
    [](const auto& a, const auto& b) { return a.second < b.second; });
  return max_it->second - min_it->second;
}

}  // namespace adapter
