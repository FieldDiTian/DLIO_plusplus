#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "builtin_interfaces/msg/time.hpp"
#include "fusion_engine_msgs/msg/pose.hpp"
#include "fusion_engine_msgs/msg/timestamp.hpp"

namespace adapter {

double stampToSec(const builtin_interfaces::msg::Time& stamp);
builtin_interfaces::msg::Time secToStamp(double sec);
double p1ToSec(const fusion_engine_msgs::msg::Timestamp& stamp);
Eigen::Quaterniond rpyToQuat(double roll_deg, double pitch_deg, double yaw_deg);

bool parseLocalEnuOrigin(const std::string& text, double& lat_deg, double& lon_deg, double& alt_m);
bool parseLocalEnuOriginTtl(const std::string& path, double& lat_deg, double& lon_deg, double& alt_m);

std::vector<double> retimeArrivalStamps(const std::vector<double>& arrivals, double imu_period_sec);

bool posePassesRtkGate(const fusion_engine_msgs::msg::Pose& msg,
                       double max_var_xy,
                       double max_var_z);

class P1ClockMapper {
public:
  explicit P1ClockMapper(double bin_seconds);

  void addPosePair(double arrival_ros, double p1_time);
  bool ready() const;
  // [P1 FIX 2026-07-15] Force a re-anchor (clear bins + slew + streaks). The
  // node calls this when it accepts an epoch change (backward power-cycle OR a
  // persistence-confirmed forward epoch), so the mapper never carries a stale
  // min-lag from the previous epoch into the new one.
  void reset();
  // [P3 FIX 2026-07-10] Non-const: applies a SLEW-LIMITED offset. Online
  // bin refinement moves the raw offset in steps (each new bin contributes
  // its first sample's full transport latency before min_lag converges);
  // stepping the output stamp domain mid-stream produced anomalous dt for
  // consumers. The applied offset now moves toward the target at a bounded
  // rate; mapper resets (epoch change) re-anchor it instantly.
  double toRos(double p1_time);
  double driftMs() const;

private:
  struct Bin {
    size_t count = 0;
    double center_sum = 0.0;
    double min_lag = std::numeric_limits<double>::infinity();
  };

  static double offsetDrift(const std::vector<std::pair<double, double>>& envelope);

  double bin_seconds_;
  double applied_offset_ = std::numeric_limits<double>::quiet_NaN();  // P3: slewed offset
  double last_slew_p1_ = std::numeric_limits<double>::quiet_NaN();
  double first_p1_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<Bin> bins_;
  // [P2 FIX 2026-07-09] consecutive forward-glitch counter: a persistent
  // forward jump (device epoch change) resets the mapper after
  // kGlitchResetCount rejects instead of freezing it forever.
  int forward_glitch_streak_ = 0;
};

}  // namespace adapter
